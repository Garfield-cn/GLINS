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
  GNSSMeasurement measurement_local = measurement;

  // Erase all parameters
  for (auto id : parameter_ids_) {
    graph_ptr_->removeParameterBlock(id.asInteger());
  }
  parameter_ids_.clear();

  // position block
  BackendId position_id = createGNSSPositionId(measurement_local.id);
  std::shared_ptr<PositionParameterBlock> position_parameter_block = 
    std::make_shared<PositionParameterBlock>(last_position, position_id.asInteger());
  if (!graph_ptr_->addParameterBlock(position_parameter_block)) {
    return false;
  }
  parameter_ids_.push_back(position_id);

  // check if any system does not have vaild satellite
  std::map<char, int> system_observation_cnt;
  for (size_t i = 0; i < measurement_local.satellites.size(); i++) 
  {
    auto& satellite = measurement_local.satellites[i];
    char system = satellite.getSystem();
    if (!gnss_common::useSystem(options_.common, system)) continue;
    if (system_observation_cnt.find(system) == system_observation_cnt.end()) 
      system_observation_cnt.insert(std::make_pair(system, 0));
    for (auto observation : satellite.observations) {
      if (checkObservationValid(measurement_local, 
          GNSSMeasurementIndex(i, observation.first))) {
        system_observation_cnt.at(system)++;
      }
    }
  }

  // clock blocks
  int num_clock_blocks = 0;
  for (auto satellite : measurement_local.satellites) 
  {
    BackendId clock_id = createGNSSClockId(satellite.getSystem(), measurement_local.id);
    if (gnss_common::useSystem(options_.common, satellite.getSystem()) && 
        system_observation_cnt.at(satellite.getSystem()) > 0 &&
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
  int num_residual_block = 0;
  int num_satellites = 0;
  for (size_t i = 0; i < measurement_local.satellites.size(); i++) 
  {
    auto& satellite = measurement_local.satellites[i];
    std::vector<Observation> observations_frequency;

    if (!gnss_common::useSystem(options_.common, satellite.getSystem()) || 
        !gnss_common::useSatellite(options_.common, satellite.prn)) continue;

    for (auto observation : satellite.observations) {
      if (!checkObservationValid(measurement_local, 
        GNSSMeasurementIndex(i, observation.first))) continue;

      BackendId clock_id = createGNSSClockId(satellite.getSystem(), measurement_local.id);
      std::shared_ptr<PseudorangeError<3, 1>> pseudorange_error = 
        std::make_shared<PseudorangeError<3, 1>>(measurement_local, 
        GNSSMeasurementIndex(i, observation.first), options_.error_parameter);
      graph_ptr_->addResidualBlock(pseudorange_error, 
        huber_loss_function_ptr_ ? huber_loss_function_ptr_.get() : nullptr,
        graph_ptr_->parameterBlockPtr(position_id.asInteger()),
        graph_ptr_->parameterBlockPtr(clock_id.asInteger()));
        
      observations_frequency.push_back(observation.second);
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
  }

  return true;
}

// Start ceres optimization
void SPPEstimator::optimize()
{
  graph_ptr_->options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
  graph_ptr_->options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  // graph_ptr_->options.num_threads = options_.num_threads;
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

// Check observation valid
bool SPPEstimator::checkObservationValid(
                            const GNSSMeasurement& measurement,
                            const GNSSMeasurementIndex& index)
{
  GNSSCommonOptions options = options_.common;

  // Satellite index invalid
  if(!(measurement.satellites.size() > index.satellite_index)) return false;

  auto& satellite = measurement.satellites.at(index.satellite_index);

  // System not used 
  if (!gnss_common::useSystem(options, satellite.getSystem())) return false;

  // Satellite not used
  if (!gnss_common::useSatellite(options, satellite.prn)) return false;

  // Ephemeris invalid
  if (satellite.sat_type == SatEphType::None ||
      checkZero(satellite.sat_position) ||
      satellite.sat_clock == 0.0) {
    return false;
  }

  // Elevation mask
  if (!gnss_common::checkElevation(options, measurement, 
      index.satellite_index)) {
    return false;
  }

  auto obs = satellite.observations.find(index.code_type);

  // Cannot find given code type
  if(obs == satellite.observations.end()) return false;

  // Code type not used
  if (!gnss_common::useCode(options, obs->first)) return false;

  auto& observation = obs->second;
  
  // Observation invalid
  if (observation.wavelength == 0.0 || 
      observation.pesudorange == 0.0) {
    return false;
  }

  return true;
}

// Compute and set coarse position on measurement
bool SPPEstimator::setCoarsePosition(GNSSMeasurement& measurement)
{
  // Already has a position
  if (!checkZero(measurement.position)) return true;

  SPPEstimatorOptions options;
  options.common.min_elevation = 0.0;
  SPPEstimatorPtr estimator = std::make_shared<SPPEstimator>(options);

  if (!estimator->addGNSSMeasurementAndState(measurement)) {
    return false;
  }

  estimator->optimize();
  measurement.position = estimator->getPositionEstimate();
  return true;
}

};