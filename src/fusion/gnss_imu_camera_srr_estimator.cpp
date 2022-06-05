/**
* @Function: Couples GNSS solution, camera feature, and IMU raw measuremnet
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/fusion/gnss_imu_camera_srr_estimator.h"
#include "gici/gnss/position_error.h"
#include "gici/gnss/gnss_parameter_blocks.h"
#include "gici/gnss/gnss_relative_errors.h"
#include "gici/imu/imu_common.h"
#include "gici/imu/imu_error.h"
#include "gici/utility/common.h"
#include "gici/estimate/pose_error.h"
#include "gici/imu/speed_and_bias_error.h"
#include "gici/imu/yaw_error.h"
#include "gici/vision/reprojection_error.h"

namespace gici {

// The default constructor
GnssImuCameraSrrEstimator::GnssImuCameraSrrEstimator(
                     const GnssImuCameraSrrEstimatorOptions& options) :
  options_(options), graph_(std::make_shared<Graph>()),
  cauchy_loss_function_ptr_(new ceres::CauchyLoss(1)),
  huber_loss_function_ptr_(new ceres::HuberLoss(1)),
  marginalization_residual_id_(0), initialized_(false), camera_extrinsics_id_(0),
  gnss_extrinsics_id_(0)
{
  marginalization_error_ptr_.reset(new MarginalizationError(*graph_.get()));

  // For debug
  debug_callback_.reset(new CeresDebugCallback());
  graph_->options.callbacks.push_back(debug_callback_.get());
}

// The default destructor
GnssImuCameraSrrEstimator::~GnssImuCameraSrrEstimator()
{}

// Set initialization result 
void GnssImuCameraSrrEstimator::setInitializationResult(
  const std::shared_ptr<GnssImuCameraInitialization>& initializer)
{
  CHECK(initializer->finished()) << "This function should be called after the "
    << "initializer is finished!";
  
  // Margin the used measurements and states to the given window length
  std::deque<GnssImuCameraInitialization::State> init_states;
  initializer->marginalization(options_.max_keyframes, marginalization_error_ptr_, 
    init_states, marginalization_residual_id_, gnss_extrinsics_id_, 
    camera_extrinsics_id_, landmarks_map_, active_keyframes_);

  // Convert state format
  states_.clear();
  for (auto init_state : init_states) {
    State state;
    state.id = init_state.id;
    state.imu_residual_to_lhs = init_state.imu_residual_to_lhs;
    state.timestamp = init_state.timestamp;
    if (init_state.id.type() == IdType::cNFrame) {
      state.is_keyframe = true;
    }
    states_.push_back(state);
  }

  // pre-allocate memory of measurements 
  gnss_solutions_.push_back(GnssSolution());
  frame_bundles_.push_back(nullptr);

  initialized_ = true;
}

// Add GNSS measurements and state
bool GnssImuCameraSrrEstimator::addGnssMeasurementAndState(const GnssSolution& gnss_solution)
{
  // Check initialization
  if (!initialized_) return false;

  // Wait for IMU data
  double timestamp = gnss_solution.timestamp;
  if (!waitForImuData(timestamp)) return false;

  // Add measurement
  curGnss() = gnss_solution;

  // Add state in current timestamp
  BackendId pose_id = createGnssPoseId(gnss_solution.id);
  if (checkLessEqual(lastState().timestamp, timestamp)) {
    if (!pushBackState(timestamp, pose_id)) {
      LOG(ERROR) << "Failed to push a state back!";
      return false;
    }
  }
  else {
    if (!insertState(timestamp, pose_id)) {
      LOG(ERROR) << "Failed to insert a state";
      return false;
    }
  }

  // GNSS extrinsics parameter block
  if (!gnss_extrinsics_id_.valid()) {
    gnss_extrinsics_id_ = changeIdType(pose_id, IdType::gExtrinsics);
    std::shared_ptr<PositionParameterBlock> gnss_extrinsic_parameter_block = 
      std::make_shared<PositionParameterBlock>(
        options_.initialize.gnss_extrinsics, gnss_extrinsics_id_.asInteger());
    if (!graph_->addParameterBlock(gnss_extrinsic_parameter_block)) {
      return false;
    }
  }

  // Add GNSS error
  if (!isFirstEpoch()) {
    std::shared_ptr<PositionError<7, 3>> position_error = 
      std::make_shared<PositionError<7, 3>>(curGnss().position, 
      curGnss().covariance.topLeftCorner(3, 3).inverse());
    position_error->setCoordinate(coordinate_);
    graph_->addResidualBlock(position_error, 
      // huber_loss_function_ptr_ ? huber_loss_function_ptr_.get() : nullptr,
      nullptr,   
      graph_->parameterBlockPtr(curState().id.asInteger()),
      graph_->parameterBlockPtr(gnss_extrinsics_id_.asInteger()));
  }

  new_state_type_ = IdType::gPose;

  return true;
}

// Add image measurements and state
bool GnssImuCameraSrrEstimator::addImageMeasurementAndState(
  const FrameBundlePtr& frame_bundle)
{
  // Check if initialized
  if (!initialized_) return false;

  // The first frame after initialization should be keyframe
  if (!lastFrameState().valid() && !frame_bundle->isKeyframe()) return false;

  // Wait for IMU data
  double timestamp = frame_bundle->getMinTimestampSeconds();
  if (!waitForImuData(timestamp)) return false;

  // If initialized, we are supposed not at the first epoch
  CHECK(!isFirstEpoch());

  frame_bundles_.back() = frame_bundle;

  // Add state in current timestamp
  BackendId pose_id = createNFrameId(frame_bundle->at(0)->bundleId());
  if (checkLessEqual(lastState().timestamp, timestamp)) {
    if (!pushBackState(timestamp, pose_id, frame_bundle->isKeyframe())) {
      LOG(ERROR) << "Failed to push a state back!";
      return false;
    }
  }
  else {
    if (!insertState(timestamp, pose_id, frame_bundle->isKeyframe())) {
      LOG(ERROR) << "Failed to insert a state";
      return false;
    }
  }

  // Extrinsics parameter block
  const Transformation T_SC = curFrame()->T_imu_cam();
  if (!camera_extrinsics_id_.valid()) {
    camera_extrinsics_id_ = 
      changeIdType(pose_id, IdType::cExtrinsics, size_t(0));
    std::shared_ptr<PoseParameterBlock> extrinsics_parameter_block = 
      std::make_shared<PoseParameterBlock>(T_SC, camera_extrinsics_id_.asInteger());
    if (!graph_->addParameterBlock(extrinsics_parameter_block, Graph::Pose6d)) {
      return false;
    }
    // we do not estimate extrinsics
    graph_->setParameterBlockConstant(camera_extrinsics_id_.asInteger());
  }

  // Add Landmarks
  for (size_t kp_idx = 0; kp_idx < curFrame()->numFeatures(); ++kp_idx)
  {
    const FramePtr& frame = curFrame();
    const PointPtr& point = frame->landmark_vec_[kp_idx];
    const FeatureType& type = frame->type_vec_[kp_idx];

    // check if feature is associated to landmark
    if (point == nullptr || !isSeed(frame->type_vec_[kp_idx])) {
      continue;
    }

    // check if landmark was already in to backend, if yes just add observation.
    if (isLandmarkInEstimator(createLandmarkId(point->id()))) {
      if (!addLandmarkObservation(frame, kp_idx)) {
        LOG(WARNING) << "Failed to add an observation!";
        continue;
      }
    }
    // add new landmarks at keyframe
    else if (frame->isKeyframe()) {
      // check if we have enough observations. Might not be the case if seed
      // original frame was already dropped.
      if (point->obs_.size() < options_.min_num_obs) {
        continue;
      }
      // check if the first frame is current keyframe
      if (point->obs_.front().frame_id == frame->id()) {
        continue;
      }

      CHECK(!std::isnan(point->pos_[0])) << "Point is nan!";

      // add the landmark
      BackendId landmark_backend_id = createLandmarkId(point->id());
      std::shared_ptr<HomogeneousPointParameterBlock>
          point_parameter_block =
          std::make_shared<HomogeneousPointParameterBlock>(
            point->pos(), landmark_backend_id.asInteger());
      if (!graph_->addParameterBlock(point_parameter_block,
                                      Graph::HomogeneousPoint)) {
        return false;
      }

      // add landmark to map
      auto it_landmark = landmarks_map_.emplace_hint(
            landmarks_map_.end(),
            landmark_backend_id, MapPoint(point));
      point->in_ba_graph_ = true;

      // add observation
      for (auto obs = point->obs_.begin(); obs != point->obs_.end(); obs++) {
        // check if frames ahead
        if (obs->frame_id > frame->id()) continue;
        // check if margined out
        if (obs->frame_id < active_keyframes_.front()->id()) {
          continue;
        }
        if (FramePtr f = obs->frame.lock()) {
          // add the first and the keyframe
          if (!checkEqual(f->getTimestampSec(), timestamp)) {
            continue;
          }
          if (!addLandmarkObservation(f, obs->keypoint_index_)) {
            LOG(ERROR) << "Failed to add an observation!";
            continue;
          }
        }
        else {
          LOG(ERROR) << "Unable to unlock frame.";
        }
      }
    }
  }

  // Store keyframes
  if (frame_bundle->at(0)->isKeyframe()) {
    active_keyframes_.push_back(frame_bundle->at(0));
  }

  new_state_type_ = IdType::cNFrame;

  return true;
}

// Add IMU measurement
void GnssImuCameraSrrEstimator::addImuMeasurement(const ImuMeasurement& imu_measurement)
{
  if (imu_measurements_.size() != 0 && 
      imu_measurements_.back().timestamp > imu_measurement.timestamp) {
    LOG(WARNING) << "Received IMU with previous timestamp!";
  }
  else {
    imu_mutex_.lock();
    imu_measurements_.push_back(imu_measurement);
    imu_mutex_.unlock();
  }

  // delete used IMU measurement
  imu_mutex_.lock();
  while (imu_measurements_.front().timestamp < oldestState().timestamp - 1.0) {
    imu_measurements_.pop_front();
  }
  imu_mutex_.unlock();
}

// Apply ceres optimization
void GnssImuCameraSrrEstimator::optimize()
{
  graph_->options.linear_solver_type = ceres::DENSE_SCHUR;
  graph_->options.trust_region_strategy_type = ceres::DOGLEG;
  graph_->options.num_threads = options_.num_threads;
  graph_->options.max_num_iterations = options_.max_iteration;
  graph_->options.max_solver_time_in_seconds = 0.04;

  // if (options_.verbose) {
  //   graph_->options.minimizer_progress_to_stdout = true;
  // }
  // else {
  //   graph_->options.logging_type = ceres::LoggingType::SILENT;
  //   graph_->options.minimizer_progress_to_stdout = false;
  // }
    graph_->options.logging_type = ceres::LoggingType::SILENT;
    graph_->options.minimizer_progress_to_stdout = false;

  // call solver
  graph_->solve();

  // {
  //   std::ofstream outfile;
  //   outfile.open("/home/cc/datasets/tmp/log.txt", std::ios::out | std::ios::trunc);
  //   outfile << graph_->summary.BriefReport() << std::endl;
    
  //   for (auto residual_map : graph_->residual_block_id_to_parameter_block_collection_map_) {
  //     auto residual = residual_map.first; ;
  //     auto error_interface_ptr = graph_->residual_block_id_to_residual_block_spec_map_.at(residual_map.first).error_interface_ptr;
  //     size_t size = error_interface_ptr->residualDim();
  //     size_t para_size = residual_map.second.size();

  //     Eigen::VectorXd Residuals = Eigen::VectorXd::Zero(size);
  //     graph_->problem_->EvaluateResidualBlock(
  //         residual, false, nullptr, Residuals.data(), nullptr);
  //     double norm_residual = Residuals.norm();
  //     if (norm_residual < 10.0)
  //       outfile << "Residual " << static_cast<int>(error_interface_ptr->typeInfo()) << ": " 
  //               << residual << ": " << Residuals.transpose() << std::endl;
  //     else 
  //       outfile << "*Residual " << static_cast<int>(error_interface_ptr->typeInfo()) << ": " 
  //               << residual << ": " << Residuals.transpose() << std::endl;
  //   }
  //   outfile.close();
  // }

  if (graph_->summary.initial_cost > 1e4) {
    LOG(FATAL) << "NOT GOOD";
  }

  // update landmarks
  for(auto &id_and_map_point : landmarks_map_) {
    // update coordinates
    id_and_map_point.second.hom_coordinates =
        std::static_pointer_cast<HomogeneousPointParameterBlock>(
          graph_->parameterBlockPtr(
            id_and_map_point.first.asInteger()))->estimate();
  }

  if (options_.verbose) {
    LOG(INFO) << graph_->summary.BriefReport();
  }

  // Marginalization
  if (new_state_type_ == IdType::cNFrame) {
    marginalization();
  }

  LOG(INFO) << "### State size = " << states_.size() << ", Para size = " << graph_->parameters().size()
    << " " << ", Res size = " << graph_->residuals().size() << ", per " 
    << (double)graph_->residuals().size() / (double)graph_->parameters().size()
    << ", num_keyframes = " << sizeOfKeyframeStates();
    
  // Shift state and measurement
  if (new_state_type_ == IdType::gPose) gnss_solutions_.push_back(GnssSolution());
  if (new_state_type_ == IdType::cNFrame) frame_bundles_.push_back(nullptr);
  states_.push_back(State());
  // only keep measurement data for two epochs
  if (gnss_solutions_.size() > 2) gnss_solutions_.pop_front();
  if (frame_bundles_.size() > 2) frame_bundles_.pop_front();
}

// Get latest pose
Transformation GnssImuCameraSrrEstimator::getPoseEstimate()
{
  State& state = lastState();
  if (!graph_->parameterBlockExists(state.id.asInteger())) {
    return Transformation();
  }
  return getPoseEstimate(state);
}

// Get latest speed and bias
SpeedAndBias GnssImuCameraSrrEstimator::getSpeedAndBias()
{
  State& state = lastState();
  BackendId speed_and_bias_id = changeIdType(state.id, IdType::ImuStates);
  if (!graph_->parameterBlockExists(speed_and_bias_id.asInteger())) {
    return SpeedAndBias::Zero();
  }
  return getSpeedAndBias(state);
}

// Get latest GNSS extrinsics
Eigen::Vector3d GnssImuCameraSrrEstimator::getGnssExtrinsic()
{
  State& state = lastGnssState();
  CHECK(state.valid());
  BackendId gnss_extrinsics_id = changeIdType(state.id, IdType::gExtrinsics);
  if (!graph_->parameterBlockExists(gnss_extrinsics_id.asInteger())) {
    return Eigen::Vector3d::Zero();
  }

  std::shared_ptr<PositionParameterBlock> block_ptr =
      std::static_pointer_cast<PositionParameterBlock>(
        graph_->parameterBlockPtr(gnss_extrinsics_id.asInteger()));
  CHECK(block_ptr != nullptr);
  return block_ptr->estimate();
}

// Update map points of in-windows keyframes
void GnssImuCameraSrrEstimator::updateMap()
{
  // Update map points
  for(auto &id_and_map_point : landmarks_map_) {
    id_and_map_point.second.point->pos_=
        id_and_map_point.second.hom_coordinates.head<3>();
  }
}

// Marginalization
bool GnssImuCameraSrrEstimator::marginalization()
{
  if (isFirstEpoch()) return true;
  State& last_frame_state = lastFrameState();
  if (!last_frame_state.valid()) return true;

  // remove linear marginalizationError, if existing
  if (marginalization_error_ptr_ && marginalization_residual_id_)
  {
    bool success = graph_->removeResidualBlock(marginalization_residual_id_);
    CHECK(success) << "could not remove marginalization error";
    marginalization_residual_id_ = 0;
    if (!success) return false;
  }

  // these will keep track of what we want to marginalize out.
  std::vector<uint64_t> parameter_blocks_to_be_marginalized;
  std::vector<bool> keep_parameter_blocks;

  // Case 1: Last frame is not a keyframe. Through last frame away.
  if (!last_frame_state.is_keyframe)
  {
    // remove feature measurements
    Graph::ResidualBlockCollection residuals = 
      graph_->residuals(last_frame_state.id.asInteger());
    for (size_t r = 0; r < residuals.size(); ++r) {
      if (residuals[r].error_interface_ptr->typeInfo() 
          != ErrorType::kReprojectionError) continue;

      removeLandmarkObservation(residuals[r].residual_block_id);
    }

    // remove state
    eraseState(last_frame_state.timestamp, last_frame_state.id);
  }
  // Case 2: Last frame is a keyframe. Marginalize the oldest keyframe and corresponding 
  // IMU and GNSS states out. 
  else if (sizeOfKeyframeStates() > options_.max_keyframes)
  {
    // marginalize oldest keyframe state, which actually, all the states before the 
    // second keyframe state
    bool passed_first_keyframe = false;
    BackendId margin_keyframe_id;
    for (auto it = states_.begin(); it != states_.end();) {
      State& state = *it;
      // reached the second keyframe
      if (passed_first_keyframe && state.is_keyframe) break;
      // passed the first keyframe
      if (state.is_keyframe) passed_first_keyframe = true;

      BackendId pose_id = state.id;
      if (state.is_keyframe) margin_keyframe_id = pose_id;

      // Pose parameter
      if (graph_->parameterBlockExists(pose_id.asInteger())) {
        parameter_blocks_to_be_marginalized.push_back(pose_id.asInteger());
        keep_parameter_blocks.push_back(false);

        // Get all residuals connected to this state.
        Graph::ResidualBlockCollection residuals = 
          graph_->residuals(pose_id.asInteger());
        for (size_t r = 0; r < residuals.size(); ++r) {
          // Not now for keyframe, we add the reprojection errors latter.
          // For non-keyframe reprojection errors, we have already erased most of them
          // at Case 1. The rest were added when a new landmark parameter was registered.
          // Whether they can be or cannot be observed by other keyframes, we do not need
          // them any more. So we can margin them now.
          if (state.is_keyframe && residuals[r].error_interface_ptr->typeInfo() 
              == ErrorType::kReprojectionError) continue;

          marginalization_error_ptr_->addResidualBlock(
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
          marginalization_error_ptr_->addResidualBlock(
                residuals[r].residual_block_id);
        }
      }

      // Camera extrinsics parameter
      // we do not need to marginalize camera extrinsics because we setted it as constant
      if (pose_id.type() == IdType::cNFrame && false) {
        
      }

      // GNSS extrinsics parameter
      // we do not need to marginalize GNSS extrinsics because we setted it as global
      if (pose_id.type() == IdType::gPose) {

      }

      // Erase state
      it = states_.erase(it);
    }

    // landmarks
    for(PointMap::iterator pit = landmarks_map_.begin();
        pit != landmarks_map_.end();)
    {
      Graph::ResidualBlockCollection residuals =
          graph_->residuals(pit->first.asInteger());
      // TODO: error here when replay with high frequency
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
        bool is_keyframe = false;
        for (auto state : states_) {
          if (state.is_keyframe && (state.id == pose_id)) {
            is_keyframe = true; break;
          }
        }

        // if the landmark is visible in the frames to marginalize
        if(pose_id == margin_keyframe_id) at_pose_to_margin = true;

        // the landmark is still visible in keyframes of the sliding window
        if(is_keyframe && (pose_id.bundleId() != margin_keyframe_id.bundleId())) {
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
          if (pose_id == margin_keyframe_id) {
            marginalization_error_ptr_->addResidualBlock(
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
          marginalization_error_ptr_->addResidualBlock(
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

    // Delete active keyframe
    active_keyframes_.pop_front();
  }

  // remove redundant GNSS
  eraseRedundantGnssMeasurements();

  // Apply marginalization
  marginalization_error_ptr_->marginalizeOut(parameter_blocks_to_be_marginalized,
                                             keep_parameter_blocks);

  // update error computation
  if(parameter_blocks_to_be_marginalized.size() > 0) {
    marginalization_error_ptr_->updateErrorComputation();
  }                              

  // add the marginalization term again
  if(marginalization_error_ptr_->num_residuals()==0)
  {
    marginalization_error_ptr_.reset();
  }
  if (marginalization_error_ptr_)
  {
    std::vector<std::shared_ptr<ParameterBlock> > parameter_block_ptrs;
    marginalization_error_ptr_->getParameterBlockPtrs(parameter_block_ptrs);
    marginalization_residual_id_ = graph_->addResidualBlock(
          marginalization_error_ptr_, nullptr, parameter_block_ptrs);
    CHECK(marginalization_residual_id_)
        << "could not add marginalization error";
    if (!marginalization_residual_id_)
    {
      return false;
    }
  }

  return true;
}

// Wait for IMU data
bool GnssImuCameraSrrEstimator::waitForImuData(double timestamp)
{
  int num_wait = 0, max_wait = 10;
  imu_mutex_.lock();
  double imu_timestamp = imu_measurements_.back().timestamp;
  imu_mutex_.unlock();
  while (timestamp - options_.imu_parameters.delay_imu_cam > 
          imu_timestamp) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if (++num_wait > max_wait) {
      LOG(ERROR) << "Waiting time for IMU exceeded! Timestamp needed = "
                  << std::fixed << timestamp - options_.imu_parameters.delay_imu_cam 
                  << ". Latest IMU timestamp = " 
                  << std::fixed << imu_timestamp;
      return false;
    }
  }
  return true;
}

// Add landmark observation
ceres::ResidualBlockId GnssImuCameraSrrEstimator::addLandmarkObservation(
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

  // get the keypoint measurement
  if (std::find_if(states_.begin(), states_.end(), [pose_id](State& state) 
      { return (pose_id == state.id); }) == states_.end()) {
    LOG(ERROR) << "Tried to add observation for frame that is either already "
               << "marginalized out or not yet added to the state. ID = "
               << pose_id.bundleId();
    return nullptr;
  }

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

// Remove landmark observation
bool GnssImuCameraSrrEstimator::removeLandmarkObservation(
    ceres::ResidualBlockId residual_block_id)
{
  const Graph::ParameterBlockCollection parameters =
      graph_->parameters(residual_block_id);
  const BackendId landmarkId(parameters.at(1).first);
  CHECK(landmarkId.type() == IdType::cLandmark);
  // remove in landmarksMap
  MapPoint& map_point = landmarks_map_.at(landmarkId);
  for(auto it = map_point.observations.begin();
      it!= map_point.observations.end();)
  {
    if(it->second == uint64_t(residual_block_id))
    {
      it = map_point.observations.erase(it);
    }
    else
    {
      ++it;
    }
  }
  // remove residual block
  graph_->removeResidualBlock(residual_block_id);
  return true;
}

// Erase redundant GNSS measurements
int GnssImuCameraSrrEstimator::eraseRedundantGnssMeasurements()
{
  int num_left = 0;
  bool has_space = true;
  // TODO: 反向迭代
  for (int i = states_.size() - 1; i >= 0; i--) {
    State& state = states_[i];
    if (state.id.type() == IdType::gPose) {
      if (has_space) {
        has_space = false;
        num_left++; continue;
      }
      // erase redundant
      Graph::ResidualBlockCollection residuals = 
        graph_->residuals(state.id.asInteger());
      for (size_t r = 0; r < residuals.size(); ++r) {
        if (residuals[r].error_interface_ptr->typeInfo() 
            == ErrorType::kPositionError) {
          graph_->removeResidualBlock(residuals[r].residual_block_id);
        }
      }
      eraseState(state.timestamp, state.id);
    }

    if (state.is_keyframe) has_space = true;
  }

  return num_left;
}

// Get pose estimate at a given ID
Transformation GnssImuCameraSrrEstimator::getPoseEstimate(const State& state)
{
  BackendId id = state.id;
  std::shared_ptr<PoseParameterBlock> block_ptr =
      std::static_pointer_cast<PoseParameterBlock>(
        graph_->parameterBlockPtr(id.asInteger()));
  CHECK(block_ptr != nullptr);
  return block_ptr->estimate();
}

// Get speed and bias estimate at a given state
SpeedAndBias GnssImuCameraSrrEstimator::getSpeedAndBias(const State& state)
{
  BackendId id = changeIdType(state.id, IdType::ImuStates);
  std::shared_ptr<SpeedAndBiasParameterBlock> block_ptr =
      std::static_pointer_cast<SpeedAndBiasParameterBlock>(
        graph_->parameterBlockPtr(id.asInteger()));
  CHECK(block_ptr != nullptr);
  return block_ptr->estimate();
}

// Add a state at end of window
bool GnssImuCameraSrrEstimator::pushBackState(
  double timestamp, BackendId id, bool is_keyframe)
{
  // Get last state
  State& last_state = lastState();
  CHECK(last_state.valid() && checkLessEqual(last_state.timestamp, timestamp));

  // Add new state and IMU connection
  // propagate state
  Transformation T_WS = getPoseEstimate(last_state);
  SpeedAndBias speed_and_bias = getSpeedAndBias(last_state);
  double last_timestamp = last_state.timestamp;
  imu_mutex_.lock();
  ImuError::propagation(
    imu_measurements_, options_.imu_parameters, T_WS, speed_and_bias,
    last_timestamp, timestamp, nullptr, nullptr);
  imu_mutex_.unlock();
  T_WS.getRotation().normalize();

  // add pose block
  std::shared_ptr<PoseParameterBlock> pose_parameter_block = 
    std::make_shared<PoseParameterBlock>(T_WS, id.asInteger());
  CHECK(graph_->addParameterBlock(pose_parameter_block, Graph::Pose6d));
  curState().timestamp = timestamp;
  curState().id = id;
  curState().is_keyframe = is_keyframe;

  // add speed and bias block
  BackendId speed_and_bias_id = changeIdType(id, IdType::ImuStates);
  std::shared_ptr<SpeedAndBiasParameterBlock> speed_and_bias_parameter_block = 
    std::make_shared<SpeedAndBiasParameterBlock>( 
    speed_and_bias, speed_and_bias_id.asInteger());
  CHECK(graph_->addParameterBlock(speed_and_bias_parameter_block));

  // add IMU connection
  BackendId last_speed_and_bias_id = changeIdType(last_state.id, IdType::ImuStates);
  imu_mutex_.lock();
  std::shared_ptr<ImuError> imu_error =
    std::make_shared<ImuError>(imu_measurements_, options_.imu_parameters,
                              last_timestamp, timestamp);
  imu_mutex_.unlock();
  curState().imu_residual_to_lhs = 
    graph_->addResidualBlock(imu_error, nullptr, 
      graph_->parameterBlockPtr(last_state.id.asInteger()), 
      graph_->parameterBlockPtr(last_speed_and_bias_id.asInteger()), 
      graph_->parameterBlockPtr(curState().id.asInteger()), 
      graph_->parameterBlockPtr(speed_and_bias_id.asInteger()));

  // LOG(INFO) << "Added a state at back! " << curState().imu_residual_to_lhs << " "
  //   << std::fixed << timestamp;

  return true;
} 

// Insert a state inside the window
bool GnssImuCameraSrrEstimator::insertState(
  double timestamp, BackendId id, bool is_keyframe)
{
  // Find two states around the timestamp
  State state_lhs, state_rhs;
  size_t index_lhs, index_rhs;
  for (size_t i = states_.size() - 1; i >= 0; i--) {
    State& state = states_[i];
    if (state.valid() && state.timestamp > timestamp) {
      state_rhs = state;
      index_rhs = i;
    }
    if (state.valid() && state.timestamp <= timestamp) {
      state_lhs = state;
      index_lhs = i;
      break;
    }
  }
  CHECK(state_lhs.valid() && state_rhs.valid()) << "Timestamp " << std::fixed
    << timestamp << " is not in current sliding window!";

  // Delete IMU connection
  bool ret = graph_->removeResidualBlock(state_rhs.imu_residual_to_lhs);
  CHECK(ret) << "Cannot remove IMU residual block!";

  // Add new state and IMU connection
  // propagate state
  Transformation T_WS = getPoseEstimate(state_lhs);
  SpeedAndBias speed_and_bias = getSpeedAndBias(state_lhs);
  double timestamp_lhs = state_lhs.timestamp;
  imu_mutex_.lock();
  ImuError::propagation(
    imu_measurements_, options_.imu_parameters, T_WS, speed_and_bias,
    timestamp_lhs, timestamp, nullptr, nullptr);
  imu_mutex_.unlock();
  T_WS.getRotation().normalize();

  // add pose block
  std::shared_ptr<PoseParameterBlock> pose_parameter_block = 
    std::make_shared<PoseParameterBlock>(T_WS, id.asInteger());
  CHECK(graph_->addParameterBlock(pose_parameter_block, Graph::Pose6d));
  auto it_lhs = states_.begin(); std::advance(it_lhs, index_lhs + 1);
  State state;
  state.timestamp = timestamp;
  state.id = id;
  state.is_keyframe = is_keyframe;
  auto it_cur = states_.insert(it_lhs, state);
  State& cur_state = *it_cur;
  // free the pre-occupied state memory
  states_.pop_back();

  // add speed and bias block
  BackendId speed_and_bias_id = changeIdType(id, IdType::ImuStates);
  std::shared_ptr<SpeedAndBiasParameterBlock> speed_and_bias_parameter_block = 
    std::make_shared<SpeedAndBiasParameterBlock>(
    speed_and_bias, speed_and_bias_id.asInteger());
  CHECK(graph_->addParameterBlock(speed_and_bias_parameter_block));

  // add LHS IMU connection
  BackendId speed_and_bias_id_lhs = changeIdType(state_lhs.id, IdType::ImuStates);
  imu_mutex_.lock();
  std::shared_ptr<ImuError> imu_error_lhs =
    std::make_shared<ImuError>(imu_measurements_, options_.imu_parameters,
                              timestamp_lhs, timestamp);
  imu_mutex_.unlock();
  cur_state.imu_residual_to_lhs = 
    graph_->addResidualBlock(imu_error_lhs, nullptr, 
      graph_->parameterBlockPtr(state_lhs.id.asInteger()), 
      graph_->parameterBlockPtr(speed_and_bias_id_lhs.asInteger()), 
      graph_->parameterBlockPtr(cur_state.id.asInteger()), 
      graph_->parameterBlockPtr(speed_and_bias_id.asInteger()));

  // add RHS IMU connection
  BackendId speed_and_bias_id_rhs = changeIdType(state_rhs.id, IdType::ImuStates);
  double timestamp_rhs = state_rhs.timestamp;
  imu_mutex_.lock();
  std::shared_ptr<ImuError> imu_error_rhs =
    std::make_shared<ImuError>(imu_measurements_, options_.imu_parameters,
                              timestamp, timestamp_rhs);
  imu_mutex_.unlock();
  auto it_rhs = states_.begin(); std::advance(it_rhs, index_rhs + 1);
  it_rhs->imu_residual_to_lhs = 
    graph_->addResidualBlock(imu_error_rhs, nullptr, 
      graph_->parameterBlockPtr(cur_state.id.asInteger()), 
      graph_->parameterBlockPtr(speed_and_bias_id.asInteger()), 
      graph_->parameterBlockPtr(it_rhs->id.asInteger()), 
      graph_->parameterBlockPtr(speed_and_bias_id_rhs.asInteger()));

  // LOG(INFO) << "Added a state at middle! " << cur_state.imu_residual_to_lhs << " " << it_rhs->imu_residual_to_lhs;

  return true;
}

// Erase a state inside the window
bool GnssImuCameraSrrEstimator::eraseState(
  double timestamp, BackendId id)
{
  // Find two states around the timestamp
  State state_lhs, state_rhs, state_cur;
  for (size_t i = states_.size() - 1; i >= 0; i--) {
    State& state = states_[i];
    if (checkEqual(timestamp, state.timestamp) && id == state.id) {
      state_cur = state; 
      CHECK((i + 1 < states_.size()) && (i - 1 >= 0));
      state_rhs = states_[i + 1];
      state_lhs = states_[i - 1];
      break;
    }
  }
  CHECK(state_lhs.valid() && state_rhs.valid()) << "Timestamp " << std::fixed
    << timestamp << " is not in current sliding window!";

  // Erase state
  Graph::ResidualBlockCollection residuals = 
    graph_->residuals(id.asInteger());
  for (size_t r = 0; r < residuals.size(); ++r) {
    if (residuals[r].error_interface_ptr->typeInfo() == ErrorType::kIMUError) {
    }
    graph_->removeResidualBlock(residuals[r].residual_block_id);
  }
  graph_->removeParameterBlock(id.asInteger());
  graph_->removeParameterBlock(changeIdType(id, IdType::ImuStates).asInteger());

  // Connect lhs and rhs state
  BackendId speed_and_bias_id_lhs = changeIdType(state_lhs.id, IdType::ImuStates);
  BackendId speed_and_bias_id_rhs = changeIdType(state_rhs.id, IdType::ImuStates);
  imu_mutex_.lock();
  std::shared_ptr<ImuError> imu_error_lhs =
    std::make_shared<ImuError>(imu_measurements_, options_.imu_parameters,
                              state_lhs.timestamp, state_rhs.timestamp);
  imu_mutex_.unlock();
  state_lhs.imu_residual_to_lhs = 
    graph_->addResidualBlock(imu_error_lhs, nullptr, 
      graph_->parameterBlockPtr(state_lhs.id.asInteger()), 
      graph_->parameterBlockPtr(speed_and_bias_id_lhs.asInteger()), 
      graph_->parameterBlockPtr(state_rhs.id.asInteger()), 
      graph_->parameterBlockPtr(speed_and_bias_id_rhs.asInteger()));

  // Erase state handle
  for (auto it = states_.begin(); it != states_.end(); it++) {
    if (it->id == id) {
      states_.erase(it); break;
    }
  }

  // LOG(INFO) << "Erase a state at middle! " << state_lhs.imu_residual_to_lhs << 
  //   " delete " << std::fixed << timestamp << " add " << state_lhs.timestamp << 
  //   " and " << state_rhs.timestamp;

  return true;
}

}