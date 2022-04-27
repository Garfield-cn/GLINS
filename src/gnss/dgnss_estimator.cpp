/**
* @Function: DGNSS (Double differenced pseudorange positioning) implementation
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/gnss/dgnss_estimator.h"

#include "gici/gnss/pseudorange_error_dd.h"
#include "gici/gnss/gnss_parameter_blocks.h"
#include "gici/gnss/gnss_common.h"

namespace gici {

// The default constructor
DgnssEstimator::DgnssEstimator(const DgnssEstimatorOptions& options) :
  options_(options), graph_ptr_(std::make_shared<Graph>()),
  cauchy_loss_function_ptr_(new ceres::CauchyLoss(1)),
  huber_loss_function_ptr_(new ceres::HuberLoss(1)),
  debug_callback_(new CeresDebugCallback())
{
  // For debug
  graph_ptr_->options.callbacks.push_back(debug_callback_.get());
}

// The default destructor
DgnssEstimator::~DgnssEstimator()
{}

// Add GNSS measurements and state
bool DgnssEstimator::addGnssMeasurementAndState(
                    const GnssMeasurement& measurement_rov, 
                    const GnssMeasurement& measurement_ref)
{
  // Check timestamp
  if (!checkEqual(measurement_rov.timestamp, measurement_ref.timestamp, 
    options_.max_age)) {
    LOG(WARNING) << "Max age between two measurements exceeded! "
      << "age = " << fabs(measurement_rov.timestamp - measurement_ref.timestamp)
      << "max_age = " << options_.max_age;
    return false;
  }

  measurement_rov_ = measurement_rov;
  measurement_ref_ = measurement_ref;

  // Get last estimate
  Eigen::Vector3d last_position = getPositionEstimate();

  // Add parameter blocks in current timestamp
  double timestamp = measurement_rov_.timestamp;

  // Erase all parameters
  for (auto id : parameter_ids_) {
    graph_ptr_->removeParameterBlock(id.asInteger());
  }
  parameter_ids_.clear();

  // position block
  BackendId position_id = createGnssPositionId(measurement_rov_.id);
  Eigen::Vector3d position_prior = measurement_rov_.position;
  if (!checkZero(last_position)) position_prior = last_position;
  std::shared_ptr<PositionParameterBlock> position_parameter_block = 
    std::make_shared<PositionParameterBlock>(position_prior, position_id.asInteger());
  if (!graph_ptr_->addParameterBlock(position_parameter_block)) {
    return false;
  }
  parameter_ids_.push_back(position_id);

  // select double difference pairs
  GnssMeasurementDDIndexPairs dd_pairs = gnss_common::formPseudorangeDDPair(
      measurement_rov_, measurement_ref_, options_.common);

  // Set to state
  current_state_.id = position_id;
  current_state_.timestamp = timestamp;

  // Add residual blocks
  int num_residual_block = dd_pairs.size();
  int num_satellites = 0;
  std::string last_prn = "";
  for (auto dd_pair : dd_pairs) 
  {
    GnssMeasurementIndex& index = dd_pair.rov;
    auto& satellite = measurement_rov_.getSat(index);

    std::shared_ptr<PseudorangeErrorDD<3>> pseudorange_error = 
      std::make_shared<PseudorangeErrorDD<3>>(measurement_rov_, measurement_ref_,
      dd_pair.rov, dd_pair.ref, dd_pair.rov_base, dd_pair.ref_base, 
      options_.error_parameter);
    graph_ptr_->addResidualBlock(pseudorange_error, 
      huber_loss_function_ptr_ ? huber_loss_function_ptr_.get() : nullptr,
      graph_ptr_->parameterBlockPtr(position_id.asInteger()));

    // get number of satellites
    if (last_prn != satellite.prn) {
      num_satellites++;
      last_prn = satellite.prn;
    }
  }

  // No observation added
  if (num_residual_block == 0) {
    LOG(WARNING) << "No satellite observed!";
    return false;
  }

  // Insufficient satellites
  if (num_satellites < 3) {
    LOG(WARNING) << "Insufficient satellites! Num = " << num_satellites;
    return false;
  }

  return true;
}

// Start ceres optimization
void DgnssEstimator::optimize()
{
  graph_ptr_->options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
  graph_ptr_->options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  graph_ptr_->options.num_threads = options_.num_threads;
  graph_ptr_->options.max_num_iterations = options_.max_iteration;

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
}

// Get position in ECEF coordinate
Eigen::Vector3d DgnssEstimator::getPositionEstimate()
{
  if (!graph_ptr_->parameterBlockExists(current_state_.id.asInteger())) {
    return Eigen::Vector3d::Zero();
  }

  std::shared_ptr<ParameterBlock> base_ptr =
      graph_ptr_->parameterBlockPtr(current_state_.id.asInteger());
  if (base_ptr != nullptr) {
    std::shared_ptr<PositionParameterBlock> block_ptr = 
      std::dynamic_pointer_cast<PositionParameterBlock>(base_ptr);
    CHECK(block_ptr != nullptr);
    return block_ptr->estimate();
  }

  return Eigen::Vector3d::Zero();
}

}