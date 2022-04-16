/**
* @Function: Single differenced pseudorange positioning implementation
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/gnss/dgnss_estimator.h"

#include "gici/gnss/pseudorange_error_sd.h"
#include "gici/gnss/gnss_parameter_blocks.h"
#include "gici/gnss/gnss_common.h"

namespace gici {

// The default constructor
DGNSSEstimator::DGNSSEstimator(const DGNSSEstimatorOptions& options) :
  options_(options), graph_ptr_(std::make_shared<Graph>()),
  cauchy_loss_function_ptr_(new ceres::CauchyLoss(1)),
  huber_loss_function_ptr_(new ceres::HuberLoss(1))
{}

// The default destructor
DGNSSEstimator::~DGNSSEstimator()
{}

// Add GNSS measurements and state
bool DGNSSEstimator::addGNSSMeasurementAndState(
                    const GNSSMeasurement& measurement_rov, 
                    const GNSSMeasurement& measurement_ref)
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
  BackendId position_id = createGNSSPositionId(measurement_rov_.id);
  Eigen::Vector3d position_prior = measurement_rov_.position;
  if (!checkZero(last_position)) position_prior = last_position;
  std::shared_ptr<PositionParameterBlock> position_parameter_block = 
    std::make_shared<PositionParameterBlock>(position_prior, position_id.asInteger());
  if (!graph_ptr_->addParameterBlock(position_parameter_block)) {
    return false;
  }
  parameter_ids_.push_back(position_id);

  // select single difference pairs
  GNSSMeasurementIndexPairs index_pairs = 
    gnss_common::formPseudorangePair(measurement_rov_, measurement_ref_, options_.common);

  // check if any system does not have vaild satellite
  std::map<char, int> system_observation_cnt;
  for (auto index_pair : index_pairs) 
  {
    GNSSMeasurementIndex& index = index_pair.first;
    auto& satellite = measurement_rov_.getSat(index);
    char system = satellite.getSystem();
    if (system_observation_cnt.find(system) == system_observation_cnt.end()) 
      system_observation_cnt.insert(std::make_pair(system, 0));
    system_observation_cnt.at(system)++;
  }

  // clock blocks
  int num_clock_blocks = 0;
  for (auto index_pair : index_pairs) 
  {
    GNSSMeasurementIndex& index = index_pair.first;
    auto& satellite = measurement_rov_.getSat(index);
    BackendId clock_id = createGNSSClockId(satellite.getSystem(), measurement_rov_.id);
    if (system_observation_cnt.at(satellite.getSystem()) > 0 &&
        !graph_ptr_->parameterBlockExists(clock_id.asInteger())) 
    {
      Eigen::Matrix<double, 1, 1> clock_init;
      clock_init.setZero();
      std::shared_ptr<ClockParameterBlock> clock_parameter_block = 
        std::make_shared<ClockParameterBlock>(clock_init, clock_id.asInteger());
      if (!graph_ptr_->addParameterBlock(clock_parameter_block)) {
        return false;
      }
      num_clock_blocks++;
      parameter_ids_.push_back(clock_id);
    }
  }

  // Set to state
  current_state_.id = position_id;
  current_state_.timestamp = timestamp;

  // Add residual blocks
  int num_residual_block = index_pairs.size();
  int num_satellites = 0;
  std::string last_prn = "";
  for (auto index_pair : index_pairs) 
  {
    GNSSMeasurementIndex& index = index_pair.first;
    auto& satellite = measurement_rov_.getSat(index);

    BackendId clock_id = createGNSSClockId(satellite.getSystem(), measurement_rov_.id);
    std::shared_ptr<PseudorangeErrorSD<3, 1>> pseudorange_error = 
      std::make_shared<PseudorangeErrorSD<3, 1>>(measurement_rov_, measurement_ref_,
      index_pair.first, index_pair.second, options_.error_parameter);
    graph_ptr_->addResidualBlock(pseudorange_error, 
      huber_loss_function_ptr_ ? huber_loss_function_ptr_.get() : nullptr,
      graph_ptr_->parameterBlockPtr(position_id.asInteger()),
      graph_ptr_->parameterBlockPtr(clock_id.asInteger()));

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
  if (num_satellites < num_clock_blocks + 3) {
    LOG(WARNING) << "Insufficient satellites! Num = " << num_satellites;
  }

  return true;
}

// Start ceres optimization
void DGNSSEstimator::optimize()
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
Eigen::Vector3d DGNSSEstimator::getPositionEstimate()
{
  if (!graph_ptr_->parameterBlockExists(current_state_.id.asInteger())) {
    return Eigen::Vector3d::Zero();
  }

  std::shared_ptr<ParameterBlock> base_ptr =
      graph_ptr_->parameterBlockPtr(current_state_.id.asInteger());
  if (base_ptr != nullptr) {
    std::shared_ptr<PositionParameterBlock> block_ptr = 
      std::dynamic_pointer_cast<PositionParameterBlock>(base_ptr);
    CHECK(block_ptr != nullptr) << "Incorrect pointer cast detected!";
    return block_ptr->estimate();
  }

  return Eigen::Vector3d::Zero();
}

}