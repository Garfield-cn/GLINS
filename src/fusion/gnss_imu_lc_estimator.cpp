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

namespace gici {

// The default constructor
GnssImuLcEstimator::GnssImuLcEstimator(
                     const GnssImuLcEstimatorOptions& options) :
  options_(options), graph_ptr_(std::make_shared<Graph>()),
  cauchy_loss_function_ptr_(new ceres::CauchyLoss(1)),
  huber_loss_function_ptr_(new ceres::HuberLoss(1)),
  marginalization_residual_id_(0), imu_initialized_(false)
{
  marginalization_error_ptr_.reset(new MarginalizationError(*graph_ptr_.get()));

  initializer_.reset(new GnssImuInitialization(options.initialize, graph_ptr_));
}

// The default destructor
GnssImuLcEstimator::~GnssImuLcEstimator()
{}

// Add GNSS measurements and state
bool GnssImuLcEstimator::addGnssMeasurementAndState(const GnssSolution& gnss_solution)
{
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

  // Initialization
  Transformation T_WS;
  SpeedAndBias speed_and_bias;
  Eigen::Vector3d t_SR_S;
  Eigen::Matrix<double, 7, 7> cov_T_WS;
  Eigen::Matrix<double, 9, 9> cov_speed_and_bias;
  Eigen::Matrix3d cov_t_SR_S;
  if (!imu_initialized_)
  {
    initializer_->addGnssMeasurement(gnss_solution);
    if (!initializer_->getResult(
      T_WS, cov_T_WS, speed_and_bias, cov_speed_and_bias, t_SR_S, cov_t_SR_S, true)) {
      LOG(INFO) << "Initializing...";
      return false;
    }
    else {
      // add used states and measurments
      initializer_->marginalization(options_.window_length, 
        marginalization_error_ptr_, gnss_solutions_, marginalization_residual_id_);
      for (auto& gnss : gnss_solutions_) {
        states_.push_back(State());
        states_.back().timestamp = gnss.timestamp;
        states_.back().id = createGnssPoseId(gnss.id);
      }

      imu_initialized_ = true;
      return true;
    }
  }

  // Add measurement
  curGnss() = gnss_solution;

  // Get prior from IMU integration
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
  if (!graph_ptr_->addParameterBlock(pose_parameter_block, Graph::Pose6d)) {
    return false;
  }

  // Speed and bias block
  BackendId speed_and_bias_id = changeIdType(pose_id, IdType::ImuStates);
  std::shared_ptr<SpeedAndBiasParameterBlock> speed_and_bias_parameter_block = 
    std::make_shared<SpeedAndBiasParameterBlock>(
    speed_and_bias, speed_and_bias_id.asInteger());
  if (!graph_ptr_->addParameterBlock(speed_and_bias_parameter_block)) {
    return false;
  }

  // GNSS extrinsic error
  BackendId gnss_extrinsic_id = changeIdType(pose_id, IdType::gExtrinsics);
  std::shared_ptr<PositionParameterBlock> gnss_extrinsic_parameter_block = 
    std::make_shared<PositionParameterBlock>(t_SR_S, gnss_extrinsic_id.asInteger());
  if (!graph_ptr_->addParameterBlock(gnss_extrinsic_parameter_block)) {
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
    graph_ptr_->addResidualBlock(position_error, 
      huber_loss_function_ptr_ ? huber_loss_function_ptr_.get() : nullptr,
      graph_ptr_->parameterBlockPtr(curState().id.asInteger()),
      graph_ptr_->parameterBlockPtr(gnss_extrinsic_id.asInteger()));
  }

  // Add IMU error
  if (!isFirstEpoch()) {
    double last_timestamp = lastState().timestamp;
    BackendId last_speed_and_bias_id = changeIdType(lastState().id, IdType::ImuStates);
    BackendId speed_and_bias_id = changeIdType(curState().id, IdType::ImuStates);
    std::shared_ptr<ImuError> imu_error =
      std::make_shared<ImuError>(imu_measurements_, options_.imu_parameters,
                                last_timestamp, timestamp);
    graph_ptr_->addResidualBlock(imu_error, nullptr, 
      graph_ptr_->parameterBlockPtr(lastState().id.asInteger()), 
      graph_ptr_->parameterBlockPtr(last_speed_and_bias_id.asInteger()), 
      graph_ptr_->parameterBlockPtr(curState().id.asInteger()), 
      graph_ptr_->parameterBlockPtr(speed_and_bias_id.asInteger()));

    // delete used IMU measurement, we keep IMUs for two epochs
    while (imu_measurements_.front().timestamp < last_timestamp) {
      imu_measurements_.pop_front();
    }
  }

  // Time constraints 
  if (!isFirstEpoch())
  {
    // GNSS extrinsic
    BackendId last_gnss_extrinsic_id = changeIdType(lastState().id, IdType::gExtrinsics);
    Eigen::Matrix3d relative_extrinsic_information = 
      Eigen::Matrix3d::Identity() * square(1.0 / options_.gnss_relative_extrinsic_std);
    std::shared_ptr<RelativePositionError> relative_extrinsic_error = 
      std::make_shared<RelativePositionError>(relative_extrinsic_information);
    graph_ptr_->addResidualBlock(relative_extrinsic_error, nullptr,
      graph_ptr_->parameterBlockPtr(last_gnss_extrinsic_id.asInteger()),
      graph_ptr_->parameterBlockPtr(gnss_extrinsic_id.asInteger()));
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
    initializer_->addImuMeasurement(imu_measurement);
  }
}

// Start ceres optimization
void GnssImuLcEstimator::optimize()
{
  graph_ptr_->options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  graph_ptr_->options.trust_region_strategy_type = ceres::DOGLEG;
  graph_ptr_->options.num_threads = options_.num_threads;
  graph_ptr_->options.max_num_iterations = options_.max_iteration;
  // graph_ptr_->options.function_tolerance = 1e-12;
  // graph_ptr_->options.gradient_tolerance = 1e-12;
  // graph_ptr_->options.parameter_tolerance = 1e-12;

  // For debug
  // debug_callback_.reset(new CeresDebugCallback());
  // graph_ptr_->options.callbacks.push_back(debug_callback_.get());

  if (options_.verbose) {
    graph_ptr_->options.minimizer_progress_to_stdout = true;
  }
  else {
    graph_ptr_->options.logging_type = ceres::LoggingType::SILENT;
    graph_ptr_->options.minimizer_progress_to_stdout = false;
  }

  // call solver
  graph_ptr_->solve();

  if (options_.verbose) {
    LOG(INFO) << graph_ptr_->summary.BriefReport();
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
  if (!graph_ptr_->parameterBlockExists(state.id.asInteger())) {
    return Transformation();
  }

  std::shared_ptr<ParameterBlock> base_ptr =
      graph_ptr_->parameterBlockPtr(state.id.asInteger());
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
  if (!graph_ptr_->parameterBlockExists(speed_and_bias_id.asInteger())) {
    return SpeedAndBias::Zero();
  }

  std::shared_ptr<ParameterBlock> base_ptr =
      graph_ptr_->parameterBlockPtr(speed_and_bias_id.asInteger());
  if (base_ptr != nullptr) {
    std::shared_ptr<SpeedAndBiasParameterBlock> block_ptr = 
      std::dynamic_pointer_cast<SpeedAndBiasParameterBlock>(base_ptr);
    CHECK(block_ptr != nullptr);
    return block_ptr->estimate();
  }

  return SpeedAndBias::Zero();
}

// Get Relative position between GNSS and IMU
Eigen::Vector3d GnssImuLcEstimator::getGnssExtrinsic()
{
  State& state = lastState();
  BackendId gnss_extrinsic_id = changeIdType(state.id, IdType::gExtrinsics);
  if (!graph_ptr_->parameterBlockExists(gnss_extrinsic_id.asInteger())) {
    return Eigen::Vector3d::Zero();
  }

  std::shared_ptr<ParameterBlock> base_ptr =
      graph_ptr_->parameterBlockPtr(gnss_extrinsic_id.asInteger());
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
    bool success = graph_ptr_->removeResidualBlock(marginalization_residual_id_);
    CHECK(success) << "could not remove marginalization error";
    marginalization_residual_id_ = 0;
    if (!success) return false;
  }

  // these will keep track of what we want to marginalize out.
  std::vector<uint64_t> parameter_blocks_to_be_marginalized;
  std::vector<bool> keep_parameter_blocks;

  // Pose parameter
  BackendId pose_id = oldestState().id;
  if (graph_ptr_->parameterBlockExists(pose_id.asInteger())) {
    parameter_blocks_to_be_marginalized.push_back(pose_id.asInteger());
    keep_parameter_blocks.push_back(false);

    // Get all residuals connected to this state.
    Graph::ResidualBlockCollection residuals = 
      graph_ptr_->residuals(pose_id.asInteger());
    for (size_t r = 0; r < residuals.size(); ++r) {
      marginalization_error_ptr_->addResidualBlock(
            residuals[r].residual_block_id);
    }
  }
  
  // Speed and bias parameter
  BackendId speed_and_bias_id = changeIdType(pose_id, IdType::ImuStates);
  if (graph_ptr_->parameterBlockExists(speed_and_bias_id.asInteger())) {
    parameter_blocks_to_be_marginalized.push_back(speed_and_bias_id.asInteger());
    keep_parameter_blocks.push_back(false);

    // Get all residuals connected to this state.
    Graph::ResidualBlockCollection residuals = 
      graph_ptr_->residuals(speed_and_bias_id.asInteger());
    for (size_t r = 0; r < residuals.size(); ++r) {
      marginalization_error_ptr_->addResidualBlock(
            residuals[r].residual_block_id);
    }
  }

  // GNSS Extrinsic parameter
  BackendId gnss_extrinsic_id = changeIdType(pose_id, IdType::gExtrinsics);
  if (graph_ptr_->parameterBlockExists(gnss_extrinsic_id.asInteger())) {
    parameter_blocks_to_be_marginalized.push_back(gnss_extrinsic_id.asInteger());
    keep_parameter_blocks.push_back(false);

    // Get all residuals connected to this state.
    Graph::ResidualBlockCollection residuals = 
      graph_ptr_->residuals(gnss_extrinsic_id.asInteger());
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
    marginalization_residual_id_ = graph_ptr_->addResidualBlock(
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