/**
* @Function: Single Point positioning implementation
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/gnss/spp_estimator.h"

#include "gici/gnss/pseudorange_error.h"
#include "gici/gnss/gnss_parameter_blocks.h"
#include "gici/gnss/gnss_common.h"

namespace gici {

// The default constructor
SppEstimator::SppEstimator(const SppEstimatorOptions& options) :
  options_(options), graph_(std::make_shared<Graph>()),
  cauchy_loss_function_ptr_(new ceres::CauchyLoss(1)),
  huber_loss_function_ptr_(new ceres::HuberLoss(1))
{}

// The default destructor
SppEstimator::~SppEstimator()
{}

// Add GNSS measurements and state
bool SppEstimator::addGnssMeasurementAndState(
    const GnssMeasurement& measurement)
{
  // Get last estimate
  Eigen::Vector3d last_position = getPositionEstimate();

  // Add parameter blocks in current timestamp
  double timestamp = measurement.timestamp;
  measurement_ = measurement;

  // Erase all parameters
  for (auto id : parameter_ids_) {
    graph_->removeParameterBlock(id.asInteger());
  }
  parameter_ids_.clear();

  // position block
  BackendId position_id = createGnssPositionId(measurement_.id);
  std::shared_ptr<PositionParameterBlock> position_parameter_block = 
    std::make_shared<PositionParameterBlock>(last_position, position_id.asInteger());
  if (!graph_->addParameterBlock(position_parameter_block)) {
    return false;
  }
  parameter_ids_.push_back(position_id);

  // check if any system does not have vaild satellite
  std::map<char, int> system_observation_cnt;
  for (auto& sat : measurement_.satellites) 
  {
    auto& satellite = sat.second;
    char system = satellite.getSystem();
    if (!gnss_common::useSystem(options_.common, system)) continue;
    if (system_observation_cnt.find(system) == system_observation_cnt.end()) 
      system_observation_cnt.insert(std::make_pair(system, 0));
    for (auto obs : satellite.observations) {
      if (gnss_common::checkObservationValid(measurement_, 
          GnssMeasurementIndex(satellite.prn, obs.first), 
          ObservationType::Pseudorange, options_.common)) {
        system_observation_cnt.at(system)++;
      }
    }
  }

  // clock blocks
  int num_clock_blocks = 0;
  for (auto& sat : measurement_.satellites) 
  {
    Satellite& satellite = sat.second;
    BackendId clock_id = createGnssClockId(satellite.getSystem(), measurement_.id);
    if (gnss_common::useSystem(options_.common, satellite.getSystem()) && 
        system_observation_cnt.at(satellite.getSystem()) > 0 &&
        !graph_->parameterBlockExists(clock_id.asInteger())) 
    {
      Eigen::Matrix<double, 1, 1> clock_init;
      clock_init.setZero();
      std::shared_ptr<ClockParameterBlock> clock_parameter_block = 
        std::make_shared<ClockParameterBlock>(clock_init, clock_id.asInteger());
      if (!graph_->addParameterBlock(clock_parameter_block)) {
        return false;
      }
      num_clock_blocks++;
      parameter_ids_.push_back(clock_id);
    }
  }

  // Set to state
  current_state_.id = position_id;
  current_state_.timestamp = timestamp;

  // Correct TGD
  correctDCB(measurement_);

  // Add residual blocks
  int num_residual_block = 0;
  int num_satellites = 0;
  for (auto& sat : measurement_.satellites) 
  {
    Satellite& satellite = sat.second;
    std::vector<Observation> observations_frequency;

    if (!gnss_common::useSystem(options_.common, satellite.getSystem()) || 
        !gnss_common::useSatellite(options_.common, satellite.prn)) continue;

    for (auto obs : satellite.observations) {
      if (!gnss_common::checkObservationValid(measurement_, 
        GnssMeasurementIndex(satellite.prn, obs.first), 
        ObservationType::Pseudorange, options_.common)) continue;

      BackendId clock_id = createGnssClockId(satellite.getSystem(), measurement_.id);
      std::shared_ptr<PseudorangeError<3, 1>> pseudorange_error = 
        std::make_shared<PseudorangeError<3, 1>>(measurement_, 
        GnssMeasurementIndex(satellite.prn, obs.first), options_.error_parameter);
      graph_->addResidualBlock(pseudorange_error, 
        huber_loss_function_ptr_ ? huber_loss_function_ptr_.get() : nullptr,
        graph_->parameterBlockPtr(position_id.asInteger()),
        graph_->parameterBlockPtr(clock_id.asInteger()));
        
      observations_frequency.push_back(obs.second);
      num_residual_block++;
    }
    if (observations_frequency.size()) num_satellites++;
    // compute ionosphere delay using dual-frequency observation
    if (observations_frequency.size() > 1 && satellite.ionosphere == 0.0) 
    {
      double ionosphere = gnss_common::ionosphereDualFrequency(
        observations_frequency[0], observations_frequency[1]);
      satellite.ionosphere = gnss_common::ionosphereConvertToBase(
        ionosphere, observations_frequency[0].wavelength);
      satellite.ionosphere_type = IonoType::DualFrequency;
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
    return false;
  }
  num_satellites_ = num_satellites;

  return true;
}

// Apply ceres optimization
void SppEstimator::optimize()
{
  graph_->options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
  graph_->options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  // graph_->options.num_threads = options_.num_threads;
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
    LOG(INFO) << graph_->summary.BriefReport();
  }
}

// Get position in ECEF coordinate
Eigen::Vector3d SppEstimator::getPositionEstimate()
{
  if (!graph_->parameterBlockExists(current_state_.id.asInteger())) {
    return Eigen::Vector3d::Zero();
  }

  std::shared_ptr<ParameterBlock> base_ptr =
      graph_->parameterBlockPtr(current_state_.id.asInteger());
  if (base_ptr != nullptr) {
    std::shared_ptr<PositionParameterBlock> block_ptr = 
      std::dynamic_pointer_cast<PositionParameterBlock>(base_ptr);
    CHECK(block_ptr != nullptr);
    return block_ptr->estimate();
  }

  return Eigen::Vector3d::Zero();
}

// Get Satellite clock
double SppEstimator::getClockEstimate(const char system)
{
  BackendId id = changeIdType(current_state_.id, IdType::gClock, system);
  if (!graph_->parameterBlockExists(id.asInteger())) {
    return 0.0;
  }

  std::shared_ptr<ParameterBlock> base_ptr =
      graph_->parameterBlockPtr(id.asInteger());
  if (base_ptr != nullptr) {
    std::shared_ptr<ClockParameterBlock> block_ptr = 
      std::dynamic_pointer_cast<ClockParameterBlock>(base_ptr);
    CHECK(block_ptr != nullptr);
    return *block_ptr->estimate().data();
  }

  return 0.0;
}

// Get solution
GnssSolution SppEstimator::getSolution()
{
  GnssSolution solution;
  std::vector<uint64_t> parameter_block_ids;

  // Position
  solution.timestamp = current_state_.timestamp;
  solution.id = current_state_.id.asInteger();
  solution.status = GnssSolutionStatus::Single;
  solution.covariance.setZero();
  solution.position.setZero();
  solution.velocity.setZero();
  solution.num_satellites = num_satellites_;
  solution.differential_age = 0;
  if (!graph_->parameterBlockExists(current_state_.id.asInteger())) {
    return solution;
  }
  else {
    parameter_block_ids.push_back(current_state_.id.asInteger());

    std::shared_ptr<ParameterBlock> base_ptr =
        graph_->parameterBlockPtr(current_state_.id.asInteger());
    if (base_ptr != nullptr) {
      std::shared_ptr<PositionParameterBlock> block_ptr = 
        std::dynamic_pointer_cast<PositionParameterBlock>(base_ptr);
      CHECK(block_ptr != nullptr);
      solution.position = block_ptr->estimate();
    }
  }

  // velocity
  BackendId velocity_id = changeIdType(current_state_.id, IdType::gVelocity);
  if (!graph_->parameterBlockExists(velocity_id.asInteger())) {
    // we did not estimate velocity
    // get the position covariance and return
    Eigen::MatrixXd position_covariance;
    graph_->computeCovariance(parameter_block_ids, position_covariance);
    CHECK(position_covariance.cols() == 3);
    solution.covariance.topLeftCorner(3, 3) = position_covariance;
  }
  else {
    parameter_block_ids.push_back(velocity_id.asInteger());

    std::shared_ptr<ParameterBlock> base_ptr =
        graph_->parameterBlockPtr(velocity_id.asInteger());
    if (base_ptr != nullptr) {
      std::shared_ptr<VelocityParameterBlock> block_ptr = 
        std::dynamic_pointer_cast<VelocityParameterBlock>(base_ptr);
      CHECK(block_ptr != nullptr);
      solution.velocity = block_ptr->estimate();
    }

    Eigen::MatrixXd covariance;
    graph_->computeCovariance(parameter_block_ids, covariance);
    CHECK(covariance.cols() == 6);
    solution.covariance = covariance;
  }

  return solution;
}

// Correct DCB (or TGD)
void SppEstimator::correctDCB(GnssMeasurement& measurement)
{
  // TODO
}

// Compute and set coarse position on measurement
bool SppEstimator::setCoarsePosition(GnssMeasurement& measurement)
{
  // Already has a position
  if (!checkZero(measurement.position)) return true;

  // no elevation mask  
  SppEstimatorOptions options;
  options.common.min_elevation = 0.0;
  std::unique_ptr<SppEstimator> estimator = 
    std::make_unique<SppEstimator>(options);

  if (!estimator->addGnssMeasurementAndState(measurement)) {
    return false;
  }

  estimator->optimize();
  measurement.position = estimator->getPositionEstimate();
  return true;
}

};