/**
* @Function: GNSS/IMU loosely integration
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/fusion/gnss_imu_lc_estimator.h"
#include "gici/gnss/position_error.h"
#include "gici/gnss/gnss_parameter_blocks.h"
#include "gici/gnss/gnss_relative_errors.h"
#include "gici/imu/imu_common.h"
#include "gici/imu/imu_error.h"
#include "gici/utility/common.h"
#include "gici/estimate/pose_error.h"
#include "gici/imu/speed_and_bias_error.h"
#include "gici/imu/yaw_error.h"
#include "gici/utility/transform.h"

namespace gici {

// The default constructor
GnssImuLcEstimator::GnssImuLcEstimator(
                     const GnssImuLcEstimatorOptions& options) :
  options_(options), graph_(std::make_shared<Graph>()),
  cauchy_loss_function_ptr_(new ceres::CauchyLoss(1)),
  huber_loss_function_ptr_(new ceres::HuberLoss(1)),
  marginalization_residual_id_(0), initialized_(false)
{
  marginalization_error_ptr_.reset(new MarginalizationError(*graph_.get()));
}

// The default destructor
GnssImuLcEstimator::~GnssImuLcEstimator()
{}

// Set initialization result 
void GnssImuLcEstimator::setInitializationResult(
  const std::shared_ptr<GnssImuInitialization>& initializer)
{
  CHECK(initializer->finished()) << "This function should be called after the "
    << "initializer is finished!";
  
  // Margin the used measurements and states to the given window length
  initializer->marginalization(options_.window_length, 
    marginalization_error_ptr_, gnss_solutions_, marginalization_residual_id_);
  for (auto& gnss : gnss_solutions_) {
    states_.push_back(State());
    states_.back().timestamp = gnss.timestamp;
    states_.back().id = createGnssPoseId(gnss.id);
  }

  initialized_ = true;
}

// Add GNSS measurements and state
bool GnssImuLcEstimator::addGnssMeasurementAndState(const GnssSolution& gnss_solution)
{
  // Check initialization
  if (!initialized_) return false;

  // Wait for IMU data
  double timestamp = gnss_solution.timestamp;
  int num_wait = 0, max_wait = 10;
  while (timestamp - options_.imu_parameters.delay_imu_cam > 
          imu_measurements_.back().timestamp) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if (++num_wait > max_wait) {
      // This commonly cannot happen because IMU data generates faster than GNSS data.
      // If it happnes, maybe there is a serious problem on your time synchronization.
      LOG(ERROR) << "Waiting time for IMU exceeded! Timestamp needed = "
                  << std::fixed << timestamp - options_.imu_parameters.delay_imu_cam 
                  << ". Latest IMU timestamp = " 
                  << std::fixed << imu_measurements_.back().timestamp;
      return false;
    }
  }

  // Add measurement
  curGnss() = gnss_solution;

  // Get prior from IMU integration
  Transformation T_WS;
  SpeedAndBias speed_and_bias;
  Eigen::Vector3d t_SR_S;
  if (!isFirstEpoch())
  {
    double last_timestamp = lastState().timestamp;
    // get the previous states
    T_WS = getPoseEstimate();
    speed_and_bias = getSpeedAndBias();
    t_SR_S = getGnssExtrinsic();

    // propagation using IMU
    int num_used_imu_measurements =
        ImuError::propagation(
          imu_measurements_, options_.imu_parameters, T_WS, speed_and_bias,
          last_timestamp, timestamp, nullptr, nullptr);
    T_WS.getRotation().normalize();
    if (num_used_imu_measurements < 1) {
      LOG(ERROR) << "num_used_imu_measurements = " << num_used_imu_measurements;
      return false;
    }
  }

  // Add parameter blocks in current timestamp
  // Pose block
  BackendId pose_id = createGnssPoseId(curGnss().id);
  std::shared_ptr<PoseParameterBlock> pose_parameter_block = 
    std::make_shared<PoseParameterBlock>(T_WS, pose_id.asInteger());
  if (!graph_->addParameterBlock(pose_parameter_block, Graph::Pose6d)) {
    return false;
  }

  // Speed and bias block
  BackendId speed_and_bias_id = changeIdType(pose_id, IdType::ImuStates);
  std::shared_ptr<SpeedAndBiasParameterBlock> speed_and_bias_parameter_block = 
    std::make_shared<SpeedAndBiasParameterBlock>(
    speed_and_bias, speed_and_bias_id.asInteger());
  if (!graph_->addParameterBlock(speed_and_bias_parameter_block)) {
    return false;
  }

  // GNSS extrinsics error
  BackendId gnss_extrinsics_id = changeIdType(pose_id, IdType::gExtrinsics);
  std::shared_ptr<PositionParameterBlock> gnss_extrinsic_parameter_block = 
    std::make_shared<PositionParameterBlock>(t_SR_S, gnss_extrinsics_id.asInteger());
  if (!graph_->addParameterBlock(gnss_extrinsic_parameter_block)) {
    return false;
  }

  curState().id = pose_id;
  curState().timestamp = timestamp;

  // Add GNSS error
  if (!isFirstEpoch()) {
    std::shared_ptr<PositionError<7, 3>> position_error = 
      std::make_shared<PositionError<7, 3>>(curGnss().position, 
      curGnss().covariance.topLeftCorner(3, 3).inverse());
    position_error->setCoordinate(coordinate_);
    graph_->addResidualBlock(position_error, 
      huber_loss_function_ptr_ ? huber_loss_function_ptr_.get() : nullptr,
      // nullptr, 
      graph_->parameterBlockPtr(curState().id.asInteger()),
      graph_->parameterBlockPtr(gnss_extrinsics_id.asInteger()));
  }

  // Add IMU error
  if (!isFirstEpoch()) {
    double last_timestamp = lastState().timestamp;
    BackendId last_speed_and_bias_id = changeIdType(lastState().id, IdType::ImuStates);
    BackendId speed_and_bias_id = changeIdType(curState().id, IdType::ImuStates);
    std::shared_ptr<ImuError> imu_error =
      std::make_shared<ImuError>(imu_measurements_, options_.imu_parameters,
                                last_timestamp, timestamp);
    graph_->addResidualBlock(imu_error, nullptr, 
      graph_->parameterBlockPtr(lastState().id.asInteger()), 
      graph_->parameterBlockPtr(last_speed_and_bias_id.asInteger()), 
      graph_->parameterBlockPtr(curState().id.asInteger()), 
      graph_->parameterBlockPtr(speed_and_bias_id.asInteger()));

    // delete used IMU measurement, we keep IMUs for two epochs
    while (imu_measurements_.front().timestamp < last_timestamp) {
      imu_measurements_.pop_front();
    }
  }

  // Yaw error
  if (!isFirstEpoch()) {
    // Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    // if (checkZero(curGnss().velocity) && checkZero(lastGnss().velocity) && 
    //     curGnss().status == lastGnss().status) {
    //   double dt = curGnss().timestamp - lastGnss().timestamp;
    //   Eigen::Vector3d p_0 = coordinate_->convert(
    //     lastGnss().position, GeoType::ECEF, GeoType::ENU);
    //   Eigen::Vector3d p_1 = coordinate_->convert(
    //     curGnss().position, GeoType::ECEF, GeoType::ENU);
    //   velocity = (p_1 - p_0) / dt;
    // }
    // else if (!checkZero(curGnss().velocity)) {
    //   velocity = curGnss().velocity;
    // }
    // bool found_angular_velocity = false;
    // Eigen::Vector3d angular_velocity;
    // for (auto imu : imu_measurements_) {
    //   if (checkEqual(imu.timestamp, timestamp, 0.05)) {
    //     angular_velocity = imu.angular_velocity;
    //     found_angular_velocity = true;
    //     break;
    //   }
    // }
    // if (velocity.norm() > 5.0 && 
    //     found_angular_velocity && fabs(angular_velocity.norm()) < 0.1) {
    //   // initialize using velocity
    //   double yaw;
    //   initYawFromVelocity(velocity, yaw);

    //   std::shared_ptr<YawError> yaw_error = 
    //     std::make_shared<YawError>(yaw, 100.0);
    //   graph_->addResidualBlock(yaw_error, 
    //     nullptr,
    //     graph_->parameterBlockPtr(curState().id.asInteger()));
    // }
  }


  // Time constraints 
  if (!isFirstEpoch())
  {
    // GNSS extrinsics
    BackendId last_gnss_extrinsic_id = changeIdType(lastState().id, IdType::gExtrinsics);
    Eigen::Matrix3d relative_extrinsic_information = 
      Eigen::Matrix3d::Identity() * square(1.0 / options_.gnss_relative_extrinsic_std);
    std::shared_ptr<RelativePositionError> relative_extrinsic_error = 
      std::make_shared<RelativePositionError>(relative_extrinsic_information);
    graph_->addResidualBlock(relative_extrinsic_error, nullptr,
      graph_->parameterBlockPtr(last_gnss_extrinsic_id.asInteger()),
      graph_->parameterBlockPtr(gnss_extrinsics_id.asInteger()));
  }

  return true;
}

// Add IMU measurement
void GnssImuLcEstimator::addImuMeasurement(const ImuMeasurement& imu_measurement)
{
  if (imu_measurements_.size() != 0 && 
      imu_measurements_.back().timestamp > imu_measurement.timestamp) {
    LOG(WARNING) << "Received IMU with previous timestamp!";
  }
  else {
    imu_measurements_.push_back(imu_measurement);
  }
}

// Apply ceres optimization
void GnssImuLcEstimator::optimize()
{
  graph_->options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  graph_->options.trust_region_strategy_type = ceres::DOGLEG;
  graph_->options.num_threads = options_.num_threads;
  graph_->options.max_num_iterations = options_.max_iteration;
  // graph_->options.function_tolerance = 1e-12;
  // graph_->options.gradient_tolerance = 1e-12;
  // graph_->options.parameter_tolerance = 1e-12;

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
    LOG(INFO) << graph_->summary.BriefReport();
  }

  // Marginalization
  marginalization();

  // Shift state and measurement
  states_.push_back(State());
  gnss_solutions_.push_back(GnssSolution());
  if (states_.size() > options_.window_length) {
    states_.pop_front();
    gnss_solutions_.pop_front();
  }
}

// Get pose
Transformation GnssImuLcEstimator::getPoseEstimate()
{
  State& state = lastState();
  if (!graph_->parameterBlockExists(state.id.asInteger())) {
    return Transformation();
  }

  std::shared_ptr<ParameterBlock> base_ptr =
      graph_->parameterBlockPtr(state.id.asInteger());
  if (base_ptr != nullptr) {
    std::shared_ptr<PoseParameterBlock> block_ptr = 
      std::dynamic_pointer_cast<PoseParameterBlock>(base_ptr);
    CHECK(block_ptr != nullptr);
    return block_ptr->estimate();
  }

  return Transformation();
}

// Get speed and bias
SpeedAndBias GnssImuLcEstimator::getSpeedAndBias()
{
  State& state = lastState();
  BackendId speed_and_bias_id = changeIdType(state.id, IdType::ImuStates);
  if (!graph_->parameterBlockExists(speed_and_bias_id.asInteger())) {
    return SpeedAndBias::Zero();
  }

  std::shared_ptr<ParameterBlock> base_ptr =
      graph_->parameterBlockPtr(speed_and_bias_id.asInteger());
  if (base_ptr != nullptr) {
    std::shared_ptr<SpeedAndBiasParameterBlock> block_ptr = 
      std::dynamic_pointer_cast<SpeedAndBiasParameterBlock>(base_ptr);
    CHECK(block_ptr != nullptr);
    return block_ptr->estimate();
  }

  return SpeedAndBias::Zero();
}

// Get GNSS extrinsics
Eigen::Vector3d GnssImuLcEstimator::getGnssExtrinsic()
{
  State& state = lastState();
  BackendId gnss_extrinsics_id = changeIdType(state.id, IdType::gExtrinsics);
  if (!graph_->parameterBlockExists(gnss_extrinsics_id.asInteger())) {
    return Eigen::Vector3d::Zero();
  }

  std::shared_ptr<ParameterBlock> base_ptr =
      graph_->parameterBlockPtr(gnss_extrinsics_id.asInteger());
  if (base_ptr != nullptr) {
    std::shared_ptr<PositionParameterBlock> block_ptr = 
      std::dynamic_pointer_cast<PositionParameterBlock>(base_ptr);
    CHECK(block_ptr != nullptr);
    return block_ptr->estimate();
  }

  return Eigen::Vector3d::Zero();
}

// Marginalization
bool GnssImuLcEstimator::marginalization()
{
  // Check if we need marginalization
  if (states_.size() < options_.window_length) {
    return true;
  }

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

  // Pose parameter
  BackendId pose_id = oldestState().id;
  if (graph_->parameterBlockExists(pose_id.asInteger())) {
    parameter_blocks_to_be_marginalized.push_back(pose_id.asInteger());
    keep_parameter_blocks.push_back(false);

    // Get all residuals connected to this state.
    Graph::ResidualBlockCollection residuals = 
      graph_->residuals(pose_id.asInteger());
    for (size_t r = 0; r < residuals.size(); ++r) {
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

  // GNSS Extrinsic parameter
  BackendId gnss_extrinsics_id = changeIdType(pose_id, IdType::gExtrinsics);
  if (graph_->parameterBlockExists(gnss_extrinsics_id.asInteger())) {
    parameter_blocks_to_be_marginalized.push_back(gnss_extrinsics_id.asInteger());
    keep_parameter_blocks.push_back(false);

    // Get all residuals connected to this state.
    Graph::ResidualBlockCollection residuals = 
      graph_->residuals(gnss_extrinsics_id.asInteger());
    for (size_t r = 0; r < residuals.size(); ++r) {
      marginalization_error_ptr_->addResidualBlock(
            residuals[r].residual_block_id);
    }
  }

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

}