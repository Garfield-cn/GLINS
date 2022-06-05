/**
* @Function: GNSS/IMU/Camera initialization
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/fusion/gnss_imu_camera_initialization.h"
#include "gici/gnss/position_error.h"
#include "gici/gnss/gnss_parameter_blocks.h"
#include "gici/gnss/gnss_relative_errors.h"
#include "gici/imu/imu_common.h"
#include "gici/imu/imu_error.h"
#include "gici/utility/common.h"
#include "gici/estimate/pose_parameter_block.h"
#include "gici/estimate/speed_and_bias_parameter_block.h"
#include "gici/imu/speed_and_bias_error.h"
#include "gici/estimate/pose_error.h"
#include "gici/imu/yaw_error.h"
#include "gici/imu/roll_and_pitch_error.h"
#include "gici/utility/transform.h"
#include "gici/vision/reprojection_error.h"

namespace gici {


// The default constructor
GnssImuCameraInitialization::GnssImuCameraInitialization(
    const GnssImuCameraInitializationOptions& options, 
    const std::shared_ptr<Graph>& graph) :
  options_(options), graph_(graph),
  cauchy_loss_function_ptr_(new ceres::CauchyLoss(1)),
  huber_loss_function_ptr_(new ceres::HuberLoss(1)), 
  zero_motion_finished_(false), finished_(false), coordinate_(nullptr),
  gnss_extrinsics_id_(0), camera_extrinsics_id_(0)
{}

// The default destructor
GnssImuCameraInitialization::~GnssImuCameraInitialization()
{}

// Add GNSS measurements 
bool GnssImuCameraInitialization::addGnssMeasurement(const GnssSolution& gnss_solution)
{
  // Check if we have already finished initialization
  if (finished_) return true;

  // Store GNSS measurements
  gnss_solutions_.push_back(gnss_solution);
  if (gnss_solutions_.size() < options_.min_gnss_window_length) {
    // we do not have enough GNSS now
    return false;
  }

  // Check if we have finished the zero-motion initialization
  if (!zero_motion_finished_) return false;

  // Check if we have enough keyframes
  const MapPtr map = feature_handler_->getMap();
  if (map->numKeyframes() < options_.min_keyframe_window_length) {
    return false;
  }

  // Check if we can start the dynamic step initialization
  // we need 1 of the motions in the window has velocity larger than min_velocity
  int valid_cnt = 0;
  if (checkZero(gnss_solutions_.front().velocity) && 
      checkZero(gnss_solutions_.back().velocity)) {
    // we do not have velocity measurement
    for (size_t i = 1; i < gnss_solutions_.size(); i++) {
      if (gnss_solutions_[i].status != gnss_solutions_[i - 1].status) {
        // avoid crazy jump
        continue;
      }
      double dt = gnss_solutions_[i].timestamp - gnss_solutions_[i - 1].timestamp;
      Eigen::Vector3d velocity = 
        (gnss_solutions_[i].position - gnss_solutions_[i - 1].position) / dt;
      if (velocity.norm() > options_.min_velocity) {
        valid_cnt++;
      }
    }
  }
  else {
    has_velocity_ = true;
    for (size_t i = 0; i < gnss_solutions_.size(); i++) {
      if (gnss_solutions_[i].velocity.norm() > options_.min_velocity) {
        valid_cnt++;
      }
    }
  }
  if (valid_cnt < 1) {
    LOG(INFO) << "We need at least " << 1 << " states has velocity larger than "
              << options_.min_velocity << ". Current number of valid states is "
              << valid_cnt;
    return false;
  }

  // check coordinate
  if (coordinate_ == nullptr) {
    LOG(ERROR) << "Coordinate not setted!";
    return false;
  }

  // We need a GNSS solution at front.
  std::vector<FramePtr> frames;
  map->getSortedKeyframes(frames);
  for (int i = frames.size() - 1; i >= 0 && i >= frames.size() - 
       options_.min_keyframe_window_length; i--) {
    keyframes_.push_front(frames[i]);
  }
  if (gnss_solutions_.front().timestamp > keyframes_.front()->getTimestampSec()) {
    LOG(INFO) << "We still need more GNSS measurements to align time window.";
    return false;
  }

  // Erase redundant measurements
  while (gnss_solutions_.size() > options_.min_gnss_window_length && 
         gnss_solutions_.front().timestamp <= keyframes_.front()->getTimestampSec() && 
         gnss_solutions_[1].timestamp <= keyframes_.front()->getTimestampSec()) {
    gnss_solutions_.pop_front();
  }

  return true;
}

// Add IMU measurement
void GnssImuCameraInitialization::addImuMeasurement(const ImuMeasurement& imu_measurement)
{
  if (finished_) return;

  if (imu_measurements_.size() != 0 && 
      imu_measurements_.back().timestamp > imu_measurement.timestamp) {
    LOG(WARNING) << "Received IMU with previous timestamp!";
  }
  else {
    imu_mutex_.lock();
    imu_measurements_.push_back(imu_measurement);
    imu_mutex_.unlock();
  }

  // Under zero motion, we determine intial pitch, roll, and anguler rate bias
  double timestamp_end = imu_measurements_.back().timestamp;
  double timestamp_start = timestamp_end - options_.time_window_length_zero_motion;
  if (!zero_motion_finished_ && gravity_setted_ && 
      timestamp_start >= imu_measurements_.front().timestamp) {
    ImuMeasurements imu_measurements;
    for (auto it = imu_measurements_.rbegin(); it != imu_measurements_.rend(); it++) {
      ImuMeasurement& imu_measurement = *it;
      if (imu_measurement.timestamp < timestamp_start) break;
      imu_measurements.push_front(imu_measurement);
    }
    if (initPoseAndBiases(imu_measurements, options_.imu_parameters.g, 
      T_WS_0_, speed_and_bias_0_)) {
      zero_motion_finished_ = true;
      // set rotation prior to visual initializer
      feature_handler_->setRotationPrior(T_WS_0_.getEigenQuaternion());
    }
  }

  imu_mutex_.lock();
  while (imu_measurements_.front().timestamp < gnss_solutions_.front().timestamp - 0.1) {
    imu_measurements_.pop_front();
  }
  imu_mutex_.unlock();
}

// Apply initialization process
void GnssImuCameraInitialization::initialize()
{
  // Start dynamic initialization
  LOG(INFO) << "Start initialization with GNSS solutions from " << std::fixed
            << gnss_solutions_.front().timestamp << " to " 
            << gnss_solutions_.back().timestamp << ", and frames from "
            << keyframes_.front()->getTimestampSec() << " to "
            << keyframes_.back()->getTimestampSec();

  // Step 1: GNSS/INS initialization
  optimizeGnssImu();

  // Step 2: Rescale keyframe poses and corresponding landmark positions
  rescaleVisual();

  // Step 3: Optimize GNSS/INS/Camera parameters together
  optimizeGnssImuCamera();

  // Done
  LOG(INFO) << "Backend initialized.";
  finished_ = true;
}

// Marginalize the used measurements to a given window size 
bool GnssImuCameraInitialization::marginalization(const int window_length,
            const std::shared_ptr<MarginalizationError>& marginalization_ptr,
            std::deque<State>& states, 
            ceres::ResidualBlockId& marginalization_residual_id, 
            BackendId& gnss_extrinsics_id, BackendId& camera_extrinsics_id,
            PointMap& landmarks_map, std::deque<FramePtr>& keyframes)
{
  if (!finished_) return false;

  // margin all states before the front keyframe
  int front_index = keyframes_.size() - window_length;
  if (front_index < 0) front_index = 0;
  BackendId front_id = createNFrameId(keyframes_[front_index]->bundleId());

  // these will keep track of what we want to marginalize out.
  std::vector<uint64_t> parameter_blocks_to_be_marginalized;
  std::vector<bool> keep_parameter_blocks;

  // Add state items
  for (auto it = states_.begin(); it != states_.end();)
  {
    State& state = *it;
    if (state.id == front_id) break;

    // Pose parameter
    BackendId pose_id = state.id;
    if (graph_->parameterBlockExists(pose_id.asInteger())) {
      parameter_blocks_to_be_marginalized.push_back(pose_id.asInteger());
      keep_parameter_blocks.push_back(false);

      // Get all residuals connected to this state.
      Graph::ResidualBlockCollection residuals = 
        graph_->residuals(pose_id.asInteger());
      for (size_t r = 0; r < residuals.size(); ++r) {
        // not now
        if (residuals[r].error_interface_ptr->typeInfo() == 
            ErrorType::kReprojectionError) continue;
        marginalization_ptr->addResidualBlock(
              residuals[r].residual_block_id);
      }
    }

    // Speed and bias parameter
    BackendId speed_and_bias_id = changeIdType(pose_id, IdType::ImuStates);
    if (graph_->parameterBlockExists(speed_and_bias_id.asInteger())) {
      parameter_blocks_to_be_marginalized.push_back(speed_and_bias_id.asInteger());
      keep_parameter_blocks.push_back(false);

      // Get all residuals connected to this state.
      Graph::ResidualBlockCollection residuals = 
        graph_->residuals(speed_and_bias_id.asInteger());
      for (size_t r = 0; r < residuals.size(); ++r) {
        marginalization_ptr->addResidualBlock(
              residuals[r].residual_block_id);
      }
    }

    // Erase state
    it = states_.erase(it);
    if (pose_id.type() == IdType::cNFrame) 
    for (auto frame = keyframes_.begin(); frame != keyframes_.end(); frame++) {
      if ((*frame)->bundleId() == pose_id.bundleId()) {
        keyframes_.erase(frame);
        break;
      }
    }
  }

  // Add landmarks. We only keep landmarks that can be observed by at least
  // two kept frames
  if (front_index > 0)
  for(PointMap::iterator pit = landmarks_map_.begin();
      pit != landmarks_map_.end();)
  {
    Graph::ResidualBlockCollection residuals =
        graph_->residuals(pit->first.asInteger());
    CHECK(residuals.size() != 0);

    // First loop: check if we can skip
    bool at_pose_to_margin = false;
    size_t num_at_other_poses = 0;
    for (size_t r = 0; r < residuals.size(); ++r)
    {
      if (residuals[r].error_interface_ptr->typeInfo() 
          != ErrorType::kReprojectionError) continue;

      BackendId pose_id(graph_->parameters(
          residuals[r].residual_block_id).at(0).first);

      // if the landmark is visible in the frames to marginalize
      if (pose_id.bundleId() < front_id.bundleId()) {
        at_pose_to_margin = true;
      }

      // the landmark is still visible in keyframes of the sliding window
      if (pose_id.bundleId() >= front_id.bundleId()) {
        num_at_other_poses++;
      }
    }

    // the landmark is not affected by the marginalization
    if(!at_pose_to_margin) {
      pit++;
      continue;
    }

    // Second loop: actually collect residuals to marginalize
    bool do_margin = false;
    for (size_t r = 0; r < residuals.size(); ++r)
    {
      if (residuals[r].error_interface_ptr->typeInfo() 
          != ErrorType::kReprojectionError) continue;
      
      BackendId pose_id(graph_->parameters(
        residuals[r].residual_block_id).at(0).first);
      
      // can be observed by at least two keyframes, margin the observation 
      // at current keyframe
      if (at_pose_to_margin && num_at_other_poses >= 2)
      {
        if (pose_id.bundleId() < front_id.bundleId()) {
          marginalization_ptr->addResidualBlock(
                residuals[r].residual_block_id);
        }
      }
      // cannot be observed by at least two other keyframes, marginalize 
      // the landmark and its observations
      else if(at_pose_to_margin && num_at_other_poses < 2)
      {
        // add information to be considered in marginalization later.
        do_margin = true;
        // the residual term is deleted from the map as well
        marginalization_ptr->addResidualBlock(
              residuals[r].residual_block_id);
      }
    }

    // deal with parameter blocks
    if (do_margin) {
      parameter_blocks_to_be_marginalized.push_back(pit->first.asInteger());
      keep_parameter_blocks.push_back(false);
      pit->second.point->in_ba_graph_ = false;
      pit = landmarks_map_.erase(pit);
      continue;
    }

    pit++;
  } 

  // Apply marginalization
  marginalization_ptr->marginalizeOut(parameter_blocks_to_be_marginalized,
                           keep_parameter_blocks);

  // update error computation
  if(parameter_blocks_to_be_marginalized.size() > 0) {
    marginalization_ptr->updateErrorComputation();
  }                              

  if (marginalization_ptr)
  {
    std::vector<std::shared_ptr<ParameterBlock> > parameter_block_ptrs;
    marginalization_ptr->getParameterBlockPtrs(parameter_block_ptrs);
    marginalization_residual_id = graph_->addResidualBlock(
          marginalization_ptr, nullptr, parameter_block_ptrs);
    CHECK(marginalization_residual_id)
        << "could not add marginalization error";
    if (!marginalization_residual_id)
    {
      return false;
    }
  }

  // Pass parameters
  states = states_;
  gnss_extrinsics_id = gnss_extrinsics_id_;
  camera_extrinsics_id = camera_extrinsics_id_;
  landmarks_map = landmarks_map_;
  keyframes = keyframes_;

  return true;
}

// Initialize pose, velocity, and biases with GNSS solutions and IMU measurements
void GnssImuCameraInitialization::optimizeGnssImu()
{
  CHECK(gnss_solutions_.size() - options_.min_gnss_window_length >= 0);
  for (size_t i = gnss_solutions_.size() - options_.min_gnss_window_length; 
      i < gnss_solutions_.size(); i++) 
  {
    auto& cur_gnss = gnss_solutions_[i];
    Eigen::Vector3d cur_position = coordinate_->convert(
      cur_gnss.position, GeoType::ECEF, GeoType::ENU);

    // Get prior
    double timestamp = cur_gnss.timestamp;
    Eigen::Vector3d t_SR_S = options_.gnss_extrinsics;

    // get initial pose
    bool has_pose = false;
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    if (!has_velocity_ && i != 0 && 
        gnss_solutions_[i].status == gnss_solutions_[i - 1].status) {
      double dt = gnss_solutions_[i].timestamp - gnss_solutions_[i - 1].timestamp;
      Eigen::Vector3d p_0 = coordinate_->convert(
        gnss_solutions_[i - 1].position, GeoType::ECEF, GeoType::ENU);
      Eigen::Vector3d p_1 = coordinate_->convert(
        gnss_solutions_[i].position, GeoType::ECEF, GeoType::ENU);
      velocity = (p_1 - p_0) / dt;
    }
    else if (has_velocity_) {
      velocity = gnss_solutions_[i].velocity;
    }
    bool found_angular_velocity = false;
    Eigen::Vector3d angular_velocity;
    imu_mutex_.lock();
    for (auto imu : imu_measurements_) {
      if (checkEqual(imu.timestamp, timestamp, 0.05)) {
        angular_velocity = imu.angular_velocity;
        found_angular_velocity = true;
        break;
      }
    }
    imu_mutex_.unlock();
    if (velocity.norm() > options_.min_velocity && 
        found_angular_velocity && fabs(angular_velocity.norm()) < 0.1) {
      // initialize using velocity
      double yaw;
      initYawFromVelocity(velocity, yaw);

      Eigen::Vector3d rpy_0 = quaternionToEulerAngle(T_WS_0_.getEigenQuaternion());
      rpy_0.z() = yaw;
      Eigen::Quaterniond quat_coarse = eulerAngleToQuaternion(rpy_0);
      T_WS_0_ = Transformation(T_WS_0_.getPosition(), quat_coarse);
    }

    // Add parameter blocks in current timestamp
    // Pose block
    BackendId pose_id = createGnssPoseId(cur_gnss.id);
    std::shared_ptr<PoseParameterBlock> pose_parameter_block = 
      std::make_shared<PoseParameterBlock>(T_WS_0_, pose_id.asInteger());
    CHECK(graph_->addParameterBlock(pose_parameter_block, Graph::Pose6d));
    gnss_states_.push_back(std::make_pair(timestamp, pose_id));

    // Speed and bias block
    BackendId speed_and_bias_id = changeIdType(pose_id, IdType::ImuStates);
    std::shared_ptr<SpeedAndBiasParameterBlock> speed_and_bias_parameter_block = 
      std::make_shared<SpeedAndBiasParameterBlock>(
      speed_and_bias_0_, speed_and_bias_id.asInteger());
    CHECK(graph_->addParameterBlock(speed_and_bias_parameter_block));

    // GNSS extrinsics
    if (!gnss_extrinsics_id_.valid()) {
      gnss_extrinsics_id_ = changeIdType(pose_id, IdType::gExtrinsics);
      std::shared_ptr<PositionParameterBlock> gnss_extrinsic_parameter_block = 
        std::make_shared<PositionParameterBlock>(t_SR_S, gnss_extrinsics_id_.asInteger());
      CHECK(graph_->addParameterBlock(gnss_extrinsic_parameter_block));
    }

    // Add GNSS error
    std::shared_ptr<PositionError<7, 3>> position_error = 
      std::make_shared<PositionError<7, 3>>(gnss_solutions_[i].position, 
      gnss_solutions_[i].covariance.topLeftCorner(3, 3).inverse());
    position_error->setCoordinate(coordinate_);
    graph_->addResidualBlock(position_error, 
      nullptr, 
      graph_->parameterBlockPtr(pose_id.asInteger()),
      graph_->parameterBlockPtr(gnss_extrinsics_id_.asInteger()));

    // Add IMU error
    double last_timestamp;
    BackendId last_pose_id;
    if (i != 0) {
      last_timestamp = gnss_solutions_[i - 1].timestamp;
      last_pose_id = createGnssPoseId(gnss_solutions_[i - 1].id);

      BackendId last_speed_and_bias_id = changeIdType(last_pose_id, IdType::ImuStates);
      BackendId speed_and_bias_id = changeIdType(pose_id, IdType::ImuStates);
      imu_mutex_.lock();
      std::shared_ptr<ImuError> imu_error =
        std::make_shared<ImuError>(imu_measurements_, options_.imu_parameters,
                                  last_timestamp, timestamp);
      imu_mutex_.unlock();
      graph_->addResidualBlock(imu_error, nullptr, 
        graph_->parameterBlockPtr(last_pose_id.asInteger()), 
        graph_->parameterBlockPtr(last_speed_and_bias_id.asInteger()), 
        graph_->parameterBlockPtr(pose_id.asInteger()), 
        graph_->parameterBlockPtr(speed_and_bias_id.asInteger()));
    }

    // Initial errors
    if (i == 0) {
      // Initial GNSS extrinsics error
      std::shared_ptr<PositionError<3>> extrinsic_error = 
        std::make_shared<PositionError<3>>(options_.gnss_extrinsics, 
        Eigen::Matrix3d::Identity() * square(1.0 / options_.gnss_extrinsic_initial_std));
      graph_->addResidualBlock(extrinsic_error, nullptr,
        graph_->parameterBlockPtr(gnss_extrinsics_id_.asInteger()));

      // Initial bias error
      std::shared_ptr<SpeedAndBiasError> speed_and_bias_error = 
        std::make_shared<SpeedAndBiasError>(speed_and_bias_0_, 
        square(options_.min_velocity * 2.0), 
        square(options_.imu_parameters.sigma_bg), 
        square(options_.imu_parameters.sigma_ba));
      graph_->addResidualBlock(speed_and_bias_error, nullptr,
        graph_->parameterBlockPtr(speed_and_bias_id.asInteger()));
    }
  }

  // Optimize
  graph_->options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  graph_->options.trust_region_strategy_type = ceres::DOGLEG;
  graph_->options.num_threads = options_.num_threads;
  graph_->options.max_num_iterations = options_.max_iteration;

  if (options_.verbose) {
    graph_->options.minimizer_progress_to_stdout = true;
  }
  else {
    graph_->options.logging_type = ceres::LoggingType::SILENT;
    graph_->options.minimizer_progress_to_stdout = false;
  }

  // call solver
  graph_->solve();

  if (options_.verbose) {
    LOG(INFO) << "GNSS/IMU optimization step: " << std::endl 
              << graph_->summary.BriefReport();
  }
}

// Rescale keyframe poses and corresponding landmark positions with initialized poses
void GnssImuCameraInitialization::rescaleVisual()
{
  // Integrate poses to keyframe timestamps
  for (auto& frame : keyframes_) 
  {
    double timestamp = frame->getTimestampSec();
    Transformation T_WS;
    SpeedAndBias speed_and_bias;

    // find nearest state
    double timestamp_zero;
    BackendId pose_id;
    for (size_t i = 0; i < gnss_states_.size(); i++) {
      if (gnss_states_[i].first <= timestamp && 
          ((i + 1 < gnss_states_.size() && gnss_states_[i + 1].first > timestamp) || 
          (i + 1 >= gnss_states_.size()))) {
        timestamp_zero = gnss_states_[i].first;
        pose_id = gnss_states_[i].second;
      }
    }

    // get parameters
    std::shared_ptr<PoseParameterBlock> block_pose =
        std::static_pointer_cast<PoseParameterBlock>(
          graph_->parameterBlockPtr(pose_id.asInteger()));
    CHECK(block_pose != nullptr);
    T_WS = block_pose->estimate();

    BackendId speed_and_bias_id = changeIdType(pose_id, IdType::ImuStates);
    std::shared_ptr<SpeedAndBiasParameterBlock> block_speed_and_bias =
        std::static_pointer_cast<SpeedAndBiasParameterBlock>(
          graph_->parameterBlockPtr(speed_and_bias_id.asInteger()));
    CHECK(block_speed_and_bias != nullptr);
    speed_and_bias = block_speed_and_bias->estimate();

    // integrate to keyframe timestamp
    CHECK(gravity_setted_);
    ImuError::propagation(
      imu_measurements_, options_.imu_parameters, T_WS, speed_and_bias,
      timestamp_zero, timestamp, nullptr, nullptr);
    T_WS.getRotation().normalize();
    keyframe_poses_.push_back(T_WS);
    keyframe_speed_and_biases_.push_back(speed_and_bias);
  }

  // Compute scale
  std::vector<double> scales;
  for (size_t i = 1; i < keyframes_.size(); i++) {
    // compute scale
    double distance = keyframe_poses_[i].getRotation().inverse().rotate(
      keyframe_poses_[i].getPosition() - keyframe_poses_[i - 1].getPosition()).norm();
    double distance_0 = keyframes_[i]->T_world_imu().getRotation().inverse().rotate(
      keyframes_[i]->T_world_imu().getPosition() - 
      keyframes_[i - 1]->T_world_imu().getPosition()).norm();
    double scale = distance / distance_0;
    scales.push_back(scale);
  }
  double scale = vk::getMedian(scales);
  for (size_t i = 0; i < scales.size(); i++) {
    if (fabs(scales[i] - scale) > 0.2) {
      LOG(WARNING) << "Scale outlier detected (" << scale << " vs " << scales[i] << ")."
                   << " The quality of initialization maybe not good enough!";
    }
  }

  // Rescale and redirect poses and landmarks
  std::unordered_set<int> rescaled_point_ids;
  for (size_t i = 0; i < keyframes_.size(); i++) {
    // rescale pose
    Transformation T_f_w_unscaled = keyframes_[i]->T_f_w_;
    keyframes_[i]->set_T_w_imu(keyframe_poses_[i]);

    // rescale landmarks
    for (auto& landmark : keyframes_[i]->landmark_vec_) {
      if (landmark == nullptr) continue;
      // check if already rescaled
      if (rescaled_point_ids.find(landmark->id()) != rescaled_point_ids.end()) {
        continue;
      }
      Eigen::Vector3d p_W = landmark->pos();
      Eigen::Vector3d p_C = T_f_w_unscaled * p_W;
      landmark->pos_ = keyframes_[i]->T_f_w_.inverse() * (p_C * scale);
      rescaled_point_ids.insert(landmark->id());
    }
  }

  // Erase other frames and landmark observations in feature handler
  feature_handler_->setGlobalInitializationResult(keyframes_);
}

// Optimize GNSS/INS/Camera parameters together
void GnssImuCameraInitialization::optimizeGnssImuCamera()
{
  // Delete all IMU error terms, we will re-add them latter
  Graph::ResidualBlockCollection residuals = graph_->residuals();
  for (size_t i = 0; i < residuals.size(); i++) {
    if (residuals[i].error_interface_ptr->typeInfo() == ErrorType::kIMUError) {
      graph_->removeResidualBlock(residuals[i].residual_block_id);
    }
  }

  // Arrange measurement sequence
  std::multimap<double, std::pair<SensorType, size_t>> sequence;
  for (size_t i = 0; i < gnss_solutions_.size(); i++) {
    sequence.insert(std::make_pair(
      gnss_solutions_[i].timestamp, std::make_pair(SensorType::GNSS, i)));
  }
  for (size_t i = 0; i < keyframes_.size(); i++) {
    sequence.insert(std::make_pair(
      keyframes_[i]->getTimestampSec(), std::make_pair(SensorType::Camera, i)));
  }

  // Add error terms
  double last_timestamp;
  BackendId last_pose_id;
  for (auto it : sequence)
  {
    double timestamp = it.first;
    SensorType sensor_type = it.second.first;
    size_t idx = it.second.second;
    State state;
    state.timestamp = timestamp;

    BackendId pose_id;
    if (sensor_type == SensorType::GNSS) {
      // we have already added the GNSS states and measurements
      pose_id = createGnssPoseId(gnss_solutions_[idx].id);
      state.id = pose_id;
    }
    else if (sensor_type == SensorType::Camera) {
      const FramePtr& frame = keyframes_[idx];
      const Transformation& T_WS = keyframe_poses_[idx];
      const SpeedAndBias& speed_and_bias = keyframe_speed_and_biases_[idx];

      // Pose block
      pose_id = createNFrameId(frame->bundleId());
      std::shared_ptr<PoseParameterBlock> pose_parameter_block = 
        std::make_shared<PoseParameterBlock>(T_WS, pose_id.asInteger());
      CHECK(graph_->addParameterBlock(pose_parameter_block, Graph::Pose6d));
      state.id = pose_id;

      // Speed and bias block
      BackendId speed_and_bias_id = changeIdType(pose_id, IdType::ImuStates);
      std::shared_ptr<SpeedAndBiasParameterBlock> speed_and_bias_parameter_block = 
        std::make_shared<SpeedAndBiasParameterBlock>(
        speed_and_bias, speed_and_bias_id.asInteger());
      CHECK(graph_->addParameterBlock(speed_and_bias_parameter_block));

      // Camera extrinsics
      if (!camera_extrinsics_id_.valid()) {
        const Transformation& T_SC = frame->T_imu_cam();
        camera_extrinsics_id_ = changeIdType(pose_id, IdType::cExtrinsics, size_t(0));
        std::shared_ptr<PoseParameterBlock> extrinsics_parameter_block = 
          std::make_shared<PoseParameterBlock>(T_SC, camera_extrinsics_id_.asInteger());
        CHECK(graph_->addParameterBlock(extrinsics_parameter_block, Graph::Pose6d));
        graph_->setParameterBlockConstant(camera_extrinsics_id_.asInteger());
      }
    }

    // Add IMU error
    if (last_pose_id.valid()) {
      BackendId last_speed_and_bias_id = changeIdType(last_pose_id, IdType::ImuStates);
      BackendId speed_and_bias_id = changeIdType(pose_id, IdType::ImuStates);
      imu_mutex_.lock();
      std::shared_ptr<ImuError> imu_error =
        std::make_shared<ImuError>(imu_measurements_, options_.imu_parameters,
                                  last_timestamp, timestamp);
      imu_mutex_.unlock();
      state.imu_residual_to_lhs = graph_->addResidualBlock(imu_error, nullptr, 
        graph_->parameterBlockPtr(last_pose_id.asInteger()), 
        graph_->parameterBlockPtr(last_speed_and_bias_id.asInteger()), 
        graph_->parameterBlockPtr(pose_id.asInteger()), 
        graph_->parameterBlockPtr(speed_and_bias_id.asInteger()));
    }

    // Store states for marginalization latter
    states_.push_back(state); 

    // Store last pose ID for adding IMU errors
    last_timestamp = timestamp;
    last_pose_id = pose_id;
  }

  // Add landmarks
  int keyframes_front_id = keyframes_.front()->id();
  int keyframes_back_id = keyframes_.back()->id();
  for (auto frame : keyframes_)
  for (size_t kp_idx = 0; kp_idx < frame->numFeatures(); ++kp_idx)
  {
    const PointPtr& point = frame->landmark_vec_[kp_idx];
    const FeatureType& type = frame->type_vec_[kp_idx];

    // check if feature is associated to landmark
    if (point == nullptr || !isSeed(frame->type_vec_[kp_idx])) {
      continue;
    }

    // already added
    if (isLandmarkInEstimator(createLandmarkId(point->id()))) continue;

    CHECK(!std::isnan(point->pos_[0])) << "Point is nan!";

    // add the landmark
    BackendId landmark_backend_id = createLandmarkId(point->id());
    std::shared_ptr<HomogeneousPointParameterBlock>
        point_parameter_block =
        std::make_shared<HomogeneousPointParameterBlock>(
          point->pos(), landmark_backend_id.asInteger());
    CHECK(graph_->addParameterBlock(point_parameter_block,
                                    Graph::HomogeneousPoint));

    // add landmark to map
    auto it_landmark = landmarks_map_.emplace_hint(
          landmarks_map_.end(),
          landmark_backend_id, MapPoint(point));
    point->in_ba_graph_ = true;

    // add observation
    size_t num_obs = 0;
    for (auto obs = point->obs_.begin(); obs != point->obs_.end(); obs++) {
      if (FramePtr f = obs->frame.lock()) {
        // we only add in-window keyframes
        if (!f->isKeyframe() || (f->id() < keyframes_front_id || 
          f->id() > keyframes_back_id)) continue;
        if (!addLandmarkObservation(f, obs->keypoint_index_)) {
          LOG(ERROR) << "Failed to add an observation!";
          continue;
        }
        num_obs++;
      }
      else {
        LOG(ERROR) << "Unable to unlock frame.";
      }
    }

    // check if observation is sufficient
    if (num_obs < 2) {
      // erase residuals and landmark
      Graph::ResidualBlockCollection residuals = 
        graph_->residuals(landmark_backend_id.asInteger());
      for (auto residual : residuals) {
        graph_->removeResidualBlock(residual.residual_block_id);
      }
      graph_->removeParameterBlock(landmark_backend_id.asInteger());
      landmarks_map_.erase(it_landmark);
      point->in_ba_graph_ = false;
    }
  }

  // Optimize
  graph_->options.linear_solver_type = ceres::DENSE_SCHUR;
  graph_->options.trust_region_strategy_type = ceres::DOGLEG;
  graph_->options.num_threads = options_.num_threads;
  graph_->options.max_num_iterations = options_.max_iteration;

  if (options_.verbose) {
    graph_->options.minimizer_progress_to_stdout = true;
  }
  else {
    graph_->options.logging_type = ceres::LoggingType::SILENT;
    graph_->options.minimizer_progress_to_stdout = false;
  }

  // call solver
  graph_->solve();

  if (options_.verbose) {
    LOG(INFO) << "GNSS/IMU/Camera optimization step: " << std::endl 
              << graph_->summary.BriefReport();
  }

  // Update landmarks and frames
  for(auto& id_and_map_point : landmarks_map_) {
    id_and_map_point.second.point->pos_=
        id_and_map_point.second.hom_coordinates.head<3>();
  }
  for (auto& frame : keyframes_) {
    std::shared_ptr<PoseParameterBlock> block_pose =
        std::static_pointer_cast<PoseParameterBlock>(
          graph_->parameterBlockPtr(createNFrameId(frame->bundleId()).asInteger()));
    CHECK(block_pose != nullptr);
    Transformation T_WS = block_pose->estimate();
    frame->set_T_w_imu(T_WS);
  }

  // for (auto it : sequence)
  // {
  //   double timestamp = it.first;
  //   SensorType sensor_type = it.second.first;
  //   size_t idx = it.second.second;

  //   BackendId pose_id;
  //   if (sensor_type == SensorType::GNSS) {
  //     pose_id = createGnssPoseId(gnss_solutions_[idx].id);
  //   }
  //   else if (sensor_type == SensorType::Camera) {
  //     pose_id = createNFrameId(keyframes_[idx]->bundleId());
  //   }

  //   std::shared_ptr<PoseParameterBlock> block_pose =
  //       std::static_pointer_cast<PoseParameterBlock>(
  //         graph_->parameterBlockPtr(pose_id.asInteger()));
  //   CHECK(block_pose != nullptr);
  //   Transformation T_WS = block_pose->estimate();

  //   std::ofstream outfile;
  //   outfile.open("/home/cc/datasets/tmp/log.txt", std::ios::out | std::ios::app);
  //   outfile << std::fixed << std::setw(8) << std::setprecision(4) 
  //     << T_WS.getPosition().transpose() << " "
  //     << T_WS.getRotation().vector().transpose() << std::endl;
  //   outfile.close();
  // }
}

// Add landmark observation
ceres::ResidualBlockId GnssImuCameraInitialization::addLandmarkObservation(
  const FramePtr& frame, const size_t keypoint_idx)
{
  const BackendId pose_id = createNFrameId(frame->bundleId());
  CHECK_GE(frame->level_vec_(keypoint_idx), 0);
  const int cam_idx = frame->getNFrameIndex();
  // get Landmark ID.
  const BackendId landmark_backend_id = createLandmarkId(
        frame->track_id_vec_[keypoint_idx]);
  CHECK(isLandmarkInEstimator(landmark_backend_id)) << "landmark not added";

  KeypointIdentifier kid(frame, keypoint_idx);
  // check for double observations
  CHECK(landmarks_map_.at(landmark_backend_id).observations.find(kid)
              == landmarks_map_.at(landmark_backend_id).observations.end())
      << "Trying to add the same landmark for the second time";

  Eigen::Matrix2d information = Eigen::Matrix2d::Identity();
  information *= options_.feature_error_std / 
    static_cast<double>(1 << frame->level_vec_(keypoint_idx));

  // create error term
  std::shared_ptr<ReprojectionError> reprojection_error =
      std::make_shared<ReprojectionError>(
        frame->cam(),
        frame->px_vec_.col(keypoint_idx), information);
  ceres::ResidualBlockId ret_val = graph_->addResidualBlock(
        reprojection_error,
        cauchy_loss_function_ptr_ ? cauchy_loss_function_ptr_.get() : nullptr,
        graph_->parameterBlockPtr(pose_id.asInteger()),
        graph_->parameterBlockPtr(landmark_backend_id.asInteger()),
        graph_->parameterBlockPtr(camera_extrinsics_id_.asInteger()));

  // remember
  landmarks_map_.at(landmark_backend_id).observations.insert(
        std::pair<KeypointIdentifier, uint64_t>(
          kid, reinterpret_cast<uint64_t>(ret_val)));

  return ret_val;
}

}