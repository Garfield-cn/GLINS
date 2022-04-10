/**
* @Function: Single Point positioning implementation
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/gnss/spp_estimator.h"

#include "gici/gnss/pseudorange_error_sole.h"
#include "gici/gnss/gnss_parameter_blocks.h"

namespace gici {

// The default constructor
SPPEstimator::SPPEstimator(const SPPEstimatorOptions& options) :
  options_(options), graph_ptr_(std::make_shared<Graph>()),
  cauchy_loss_function_ptr_(new ceres::CauchyLoss(1)),
  huber_loss_function_ptr_(new ceres::HuberLoss(1))
{}

// The default destructor
SPPEstimator::~SPPEstimator()
{}

// Add GNSS measurements and state
bool SPPEstimator::addGNSSMeasurementAndState(
    const GNSSMeasurement& measurement)
{
  // Get last estimate
  Eigen::Vector3d last_position = getPositionEstimate();

  // Add parameter blocks in current timestamp
  double timestamp = measurement.timestamp;

  // Erase all parameters
  for (auto id : parameter_ids_) {
    graph_ptr_->removeParameterBlock(id.asInteger());
  }
  parameter_ids_.clear();

  // position block
  BackendId position_id = createGNSSPositionId(measurement.id);
  std::shared_ptr<PositionParameterBlock> position_parameter_block = 
    std::make_shared<PositionParameterBlock>(last_position, position_id.asInteger());
  if (!graph_ptr_->addParameterBlock(position_parameter_block)) {
    return false;
  }
  parameter_ids_.push_back(position_id);

  // check if any system does not have vaild satellite
  std::map<char, int> system_observation_cnt;
  for (auto satellite : measurement.satellites) {
    char system = satellite.getSystem();
    if (!useSystem(system)) continue;
    if (system_observation_cnt.find(system) == system_observation_cnt.end()) 
      system_observation_cnt.insert(std::make_pair(system, 0));
    if (useSatellite(satellite.prn)) {
      system_observation_cnt.at(system)++;
    }
  }

  // clock blocks
  for (auto satellite : measurement.satellites) {
    BackendId clock_id = createGNSSClockId(satellite.getSystem(), measurement.id);
    if (useSystem(satellite.getSystem()) && 
        system_observation_cnt.at(satellite.getSystem()) > 0 &&
        !graph_ptr_->parameterBlockExists(clock_id.asInteger())) {
      Eigen::Matrix<double, 1, 1> clock_init;
      clock_init.setZero();
      std::shared_ptr<ClockParameterBlock> clock_parameter_block = 
        std::make_shared<ClockParameterBlock>(clock_init, clock_id.asInteger());
      if (!graph_ptr_->addParameterBlock(clock_parameter_block)) {
        return false;
      }
      parameter_ids_.push_back(clock_id);
    }
  }

  // Set to state
  current_state_.id = position_id;
  current_state_.timestamp = timestamp;

  // Add residual blocks
  for (size_t i = 0; i < measurement.satellites.size(); i++) {
    auto& satellite = measurement.satellites[i];
    if (!useSystem(satellite.getSystem()) || !useSatellite(satellite.prn)) continue;
    for (auto observation : satellite.observations) {
      if (!useCode(observation.first)) continue;
      BackendId clock_id = createGNSSClockId(satellite.getSystem(), measurement.id);
      std::shared_ptr<PseudorangeErrorSole> pseudorange_error = 
        std::make_shared<PseudorangeErrorSole>(measurement, i, observation.first,
        options_.error_parameter);
      graph_ptr_->addResidualBlock(pseudorange_error, 
        cauchy_loss_function_ptr_ ? cauchy_loss_function_ptr_.get() : nullptr,
        graph_ptr_->parameterBlockPtr(position_id.asInteger()),
        graph_ptr_->parameterBlockPtr(clock_id.asInteger()));
    }
  }

  return true;
}

// Start ceres optimization
void SPPEstimator::optimize()
{
  graph_ptr_->options.linear_solver_type = ceres::DENSE_SCHUR;
  // graph_ptr_->options.initial_trust_region_radius = 1.0e4;
  // graph_ptr_->options.initial_trust_region_radius = 2.0e6;
  // graph_ptr_->options.preconditioner_type = ceres::IDENTITY;
  graph_ptr_->options.trust_region_strategy_type = ceres::DOGLEG;
  // graph_ptr_->options.use_nonmonotonic_steps = true;
  // graph_ptr_->options.max_consecutive_nonmonotonic_steps = 10;
  // graph_ptr_->options.function_tolerance = 1e-12;
  // graph_ptr_->options.gradient_tolerance = 1e-12;
  // graph_ptr_->options.jacobi_scaling = false;
#ifdef USE_OPENMP
  graph_ptr_->options.num_threads = options_.num_threads;
#endif
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
Eigen::Vector3d SPPEstimator::getPositionEstimate()
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

// Get Satellite clock
double SPPEstimator::getClockEstimate(const char system, double& clock)
{
  BackendId id = changeIdType(current_state_.id, IdType::gClock, system);
  if (!graph_ptr_->parameterBlockExists(id.asInteger())) {
    return 0.0;
  }

  std::shared_ptr<ParameterBlock> base_ptr =
      graph_ptr_->parameterBlockPtr(id.asInteger());
  if (base_ptr != nullptr) {
    std::shared_ptr<ClockParameterBlock> block_ptr = 
      std::dynamic_pointer_cast<ClockParameterBlock>(base_ptr);
    CHECK(block_ptr != nullptr) << "Incorrect pointer cast detected!";
    return *block_ptr->estimate().data();
  }

  return 0.0;
}

// Check whether the system is used
bool SPPEstimator::useSystem(const char system)
{
  auto it = std::find(options_.system_exclude.begin(), 
    options_.system_exclude.end(), system);
  if (it == options_.system_exclude.end()) return true;
  else return false;
}

// Check whether the satellite is used
bool SPPEstimator::useSatellite(const std::string prn)
{
  auto it = std::find(options_.satellite_exclude.begin(), 
    options_.satellite_exclude.end(), prn);
  if (it == options_.satellite_exclude.end()) return true;
  else return false;
}

// Check whether the code type is used
bool SPPEstimator::useCode(const int code_type)
{
  auto it = std::find(options_.code_exclude.begin(), 
    options_.code_exclude.end(), code_type);
  if (it == options_.code_exclude.end()) return true;
  else return false;
}

};