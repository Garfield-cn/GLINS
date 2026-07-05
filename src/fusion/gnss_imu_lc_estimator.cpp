/**
* @Function: GNSS/IMU loosely couple estimator
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/fusion/gnss_imu_lc_estimator.h"
#include "gici/fusion/gnss_imu_initializer.h"

// !!!
// #define TEST_NON_CONSTANT

// !!!
#ifdef TEST_NON_CONSTANT
#include "gici/gnss/gnss_relative_errors.h"
#endif  

namespace gici {

// The default constructor
GnssImuLcEstimator::GnssImuLcEstimator(
               const GnssImuLcEstimatorOptions& options, 
               const GnssImuInitializerOptions& init_options, 
               const GnssLooseEstimatorBaseOptions& gnss_loose_base_options, 
               const ImuEstimatorBaseOptions& imu_base_options,
               const EstimatorBaseOptions& base_options) :
  lc_options_(options), 
  GnssLooseEstimatorBase(gnss_loose_base_options, base_options),
  ImuEstimatorBase(imu_base_options, base_options),
  EstimatorBase(base_options)
{
  type_ = EstimatorType::GnssImuLc;
  shiftMemory();

  // Initialization control
  initializer_.reset(new GnssImuInitializer(
    init_options, gnss_loose_base_options, imu_base_options, 
    base_options, graph_));
}

// The default destructor
GnssImuLcEstimator::~GnssImuLcEstimator()
{}

// Add measurement
bool GnssImuLcEstimator::addMeasurement(const EstimatorDataCluster& measurement)
{
  // Initialization
  if (coordinate_ == nullptr || !gravity_setted_) return false;
  if (!initializer_->finished()) {
    if (initializer_->getCoordinate() == nullptr) {
      initializer_->setCoordinate(coordinate_);
      initializer_->setGravity(imu_base_options_.imu_parameters.g);
    }
    if (initializer_->addMeasurement(measurement)) {
      initializer_->estimate();
      // set result to estimator
      setInitializationResult(initializer_);
    }
    return false;
  }

  // Add IMU
  if (measurement.imu && measurement.imu_role == ImuRole::Major) {
    addImuMeasurement(*measurement.imu);
  }

  // Add GNSS by solution measurement
  if (measurement.solution && 
      (measurement.solution_role == SolutionRole::Position || 
       measurement.solution_role == SolutionRole::Velocity ||
       measurement.solution_role == SolutionRole::PositionAndVelocity)) {
    GnssSolution gnss_solution = convertSolutionToGnssSolution(
      *measurement.solution, measurement.solution_role);
    return addGnssSolutionMeasurementAndState(gnss_solution);
  }

  return false;
}

// Add GNSS measurements and state
bool GnssImuLcEstimator::addGnssSolutionMeasurementAndState(
  const GnssSolution& measurement)
{
  // Set to local measurement handle
  curGnssSolution() = measurement;

  // Add parameter blocks
  double timestamp = curGnssSolution().timestamp;
  // pose and speed and bias block
  const int32_t bundle_id = curGnssSolution().id;
  BackendId pose_id = createGnssPoseId(bundle_id);
  size_t index = insertImuState(timestamp, pose_id);
  CHECK(index == states_.size() - 1);
  curState().status = curGnssSolution().status;
  // GNSS extrinsics, it should be added at initialization step
  CHECK(gnss_extrinsics_id_.valid());

  // Add residual blocks
  // GNSS position
  addGnssPositionResidualBlock(curGnssSolution(), curState());
  // GNSS velocity
  addGnssVelocityResidualBlock(curGnssSolution(), curState(), 
    getImuMeasurementNear(timestamp).angular_velocity);
  // ZUPT
  addZUPTResidualBlock(curState());
  // Car motion
  if (imu_base_options_.car_motion) {
    // heading measurement constraint
    addHMCResidualBlock(curState());
    // non-holonomic constraint
    addNHCResidualBlock(curState());
  }

  // !!!
#ifdef TEST_NON_CONSTANT
  {
    BackendId last_id = gnss_extrinsics_id_;
    gnss_extrinsics_id_ = addGnssExtrinsicsParameterBlock(bundle_id, getGnssExtrinsicsEstimate());

    Eigen::Vector3d dp_error = Eigen::Vector3d::Ones() * 1.0e-8;
    Eigen::Matrix3d dp_covariance = (cwiseSquare(dp_error)).asDiagonal();

    std::shared_ptr<RelativeConstError<3, ErrorType::kRelativePositionError>> relative_position_error = 
      std::make_shared<RelativeConstError<3, ErrorType::kRelativePositionError>>(dp_covariance.inverse());
    graph_->addResidualBlock(relative_position_error, nullptr,
      graph_->parameterBlockPtr(last_id.asInteger()),
      graph_->parameterBlockPtr(gnss_extrinsics_id_.asInteger()));
  }
#endif

  return true;
}

// Solve current graph
bool GnssImuLcEstimator::estimate()
{
  // Optimize
  int optimize_cnt = 0;
  if (gnss_loose_base_options_.use_outlier_rejection)
  while (1)
  {
    optimize();
    optimize_cnt++;

    // reject outlier
    if (!rejectGnssPositionAndVelocityOutliers(curState())) break;
  }
  // Optimize without FDE
  else {
    optimize();
  }

  // Check if we rejected too many GNSS residuals
  if (optimize_cnt > 1) num_cotinuous_reject_gnss_++;
  else num_cotinuous_reject_gnss_ = 0;
  if (num_cotinuous_reject_gnss_ > 
      gnss_loose_base_options_.diverge_min_num_continuous_reject) {
    LOG(WARNING) << "Estimator diverge: Too many GNSS outliers rejected!";
    status_ = EstimatorStatus::Diverged;
    num_cotinuous_reject_gnss_ = 0;
  }

  // Log information
  if (base_options_.verbose_output) {
    LOG(INFO) << estimatorTypeToString(type_) << ": " 
      << "Iterations: " << graph_->summary.iterations.size() << ", "
      << std::scientific << std::setprecision(3) 
      << "Initial cost: " << graph_->summary.initial_cost << ", "
      << "Final cost: " << graph_->summary.final_cost;
  }

  auto cov = getCovariance(curState());
  LOG(INFO) << "Std XYZ: " << sqrt(cov(0, 0)) << " " << sqrt(cov(1, 1)) << " " << sqrt(cov(2, 2));
  LOG(INFO) << "Std RPY: " << sqrt(cov(3, 3)) * R2D << " " << sqrt(cov(4, 4)) * R2D << " "
            << sqrt(cov(5, 5)) * R2D << "--*--*-*-*-*-*-*-*-*-*--";

  // Apply marginalization
  marginalization();

  // Shift memory for states and measurements
  shiftMemory();

  return true;
}

// Set initializatin result
void GnssImuLcEstimator::setInitializationResult(
  const std::shared_ptr<MultisensorInitializerBase>& initializer)
{
  CHECK(initializer->finished());

  // Cast to desired initializer
  std::shared_ptr<GnssImuInitializer> gnss_imu_initializer = 
    std::static_pointer_cast<GnssImuInitializer>(initializer);
  CHECK_NOTNULL(gnss_imu_initializer);
  
  // Arrange to window length
  ImuMeasurements imu_measurements;
  gnss_imu_initializer->arrangeToEstimator(
    lc_options_.max_window_length, marginalization_error_, states_, 
    marginalization_residual_id_, gnss_extrinsics_id_, 
    gnss_solution_measurements_, imu_measurements);
  for (auto it = imu_measurements.rbegin(); it != imu_measurements.rend(); it++) {
    imu_mutex_.lock();
    imu_measurements_.push_front(*it);
    imu_mutex_.unlock();
  }

  // Shift memory for states and measurements
  shiftMemory();

  // Set flags
  can_compute_covariance_ = true;
}

// Marginalization
bool GnssImuLcEstimator::marginalization()
{
  // Check if we need marginalization
  if (states_.size() < lc_options_.max_window_length) {
    return true;
  }

  // Erase old marginalization item
  if (!eraseOldMarginalization()) return false;

  // !!!
#ifdef TEST_NON_CONSTANT
  auto residuals = graph_->residuals(oldestState().id_in_graph.asInteger());
  for (auto residual : residuals) {
    if (residual.error_interface_ptr->typeInfo() != ErrorType::kPositionError) continue;
    auto parameters = graph_->parameters(residual.residual_block_id);
    for (auto parameter : parameters) {
      if (parameter.second->typeInfo() != "PositionParameterBlock") continue;
      int num_position_residual = 0;
      for (auto r : graph_->residuals(parameter.first)) {
        if (r.error_interface_ptr->typeInfo() == ErrorType::kPositionError) num_position_residual++;
      }
      if (num_position_residual <= 1) {
        BackendId id(parameter.first);
        CHECK(graph_->parameterBlockExists(id.asInteger()));
        Graph::ResidualBlockCollection rs = graph_->residuals(id.asInteger());
        for (size_t r = 0; r < rs.size(); ++r) {
          marginalization_error_->addResidualBlock(
                residuals[r].residual_block_id);
        }
        marginalization_parameter_ids_.push_back(id);
        marginalization_keep_parameter_blocks_.push_back(false);
      }
    }
  }
  // 0.326393 -0.0291516  0.0888827
#endif

  // Add marginalization items
  // IMU states and residuals
  addImuStateMarginBlockWithResiduals(oldestState());

  // Apply marginalization and add the item into graph
  return applyMarginalization();
}

};
