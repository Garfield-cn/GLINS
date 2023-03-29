/**
* @Function: Real-Time Kinematic implementation
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/gnss/rtk_estimator.h"

#include "gici/gnss/gnss_parameter_blocks.h"
#include "gici/gnss/gnss_common.h"

namespace gici {

// The default constructor
RtkEstimator::RtkEstimator(const RtkEstimatorOptions& options, 
               const GnssEstimatorBaseOptions& gnss_base_options, 
               const EstimatorBaseOptions& base_options,
               const AmbiguityResolutionOptions& ambiguity_options) :
  rtk_options_(options), 
  GnssEstimatorBase(gnss_base_options, base_options),
  EstimatorBase(base_options)
{
  type_ = EstimatorType::Rtk;
  is_use_phase_ = true;
  if (options.estimate_velocity) has_velocity_estimate_ = true;
  can_compute_covariance_ = true;
  shiftMemory();

  // SPP estimator for setting initial states
  spp_estimator_.reset(new SppEstimator(gnss_base_options));

  // Ambiguity resolution
  ambiguity_resolution_.reset(new AmbiguityResolution(ambiguity_options, graph_));
}

// The default destructor
RtkEstimator::~RtkEstimator()
{}

// Add measurement
bool RtkEstimator::addMeasurement(const EstimatorDataCluster& measurement)
{
  GnssMeasurement rov, ref;
  meausrement_align_.add(measurement);
  if (meausrement_align_.get(rtk_options_.max_age, rov, ref)) {
    return addGnssMeasurementAndState(rov, ref);
  }

  return false;
}

// Add GNSS measurements and state
bool RtkEstimator::addGnssMeasurementAndState(
    const GnssMeasurement& measurement_rov, 
    const GnssMeasurement& measurement_ref)
{
  // Get prior states
  if (!spp_estimator_->addGnssMeasurementAndState(measurement_rov)) {
    return false;
  }
  if (!spp_estimator_->estimate()) {
    return false;
  }
  Eigen::Vector3d position_prior = spp_estimator_->getPositionEstimate();
  Eigen::Vector3d velocity_prior = spp_estimator_->getVelocityEstimate();
  curState().status = GnssSolutionStatus::Single;

  // Set to local measurement handle
  curGnssRov() = measurement_rov;
  curGnssRov().position = position_prior;
  curGnssRef() = measurement_ref;

  // Erase duplicated phases, arrange to one observation per phase
  gnss_common::rearrangePhasesAndCodes(curGnssRov());
  gnss_common::rearrangePhasesAndCodes(curGnssRef());

  // Form double difference pair
  GnssMeasurementDDIndexPairs index_pairs = gnss_common::formPhaserangeDDPair(
    curGnssRov(), curGnssRef(), gnss_base_options_.common);

  // Cycle-slip detection
  if (!isFirstEpoch()) {
    cycleSlipDetectionSD(lastGnssRov(), lastGnssRef(), 
      curGnssRov(), curGnssRef(), gnss_base_options_.common);
  }
  
  // Add parameter blocks
  double timestamp = curGnssRov().timestamp;
  curState().timestamp = timestamp;
  // position block
  BackendId position_id = addGnssPositionParameterBlock(curGnssRov().id, position_prior);
  curState().id = position_id;
  curState().id_in_graph = position_id;
  // ambiguity blocks
  addSdAmbiguityParameterBlocks(curGnssRov(), 
    curGnssRef(), index_pairs, curGnssRov().id, curAmbiguityState());
  if (rtk_options_.estimate_velocity) {
    // velocity block
    addGnssVelocityParameterBlock(curGnssRov().id, velocity_prior);
    // frequency block
    int num_valid_doppler_system = 0;
    addFrequencyParameterBlocks(curGnssRov(), curGnssRov().id, num_valid_doppler_system);
  }
  
  // Add pseudorange residual blocks
  int num_valid_satellite = 0;
  addDdPseudorangeResidualBlocks(curGnssRov(), 
    curGnssRef(), index_pairs, curState(), num_valid_satellite);

  // Check if insufficient satellites
  if (!checkSufficientSatellite(num_valid_satellite, 0)) {
    // erase parameters in current state
    eraseGnssPositionParameterBlock(curState());
    eraseAmbiguityParameterBlocks(curAmbiguityState());
    if (rtk_options_.estimate_velocity) {
      eraseGnssVelocityParameterBlock(curState());
      eraseFrequencyParameterBlocks(curState());
    }

    return false;
  }
  num_satellites_ = num_valid_satellite;

  // Add phaserange residual blocks
  addDdPhaserangeResidualBlocks(curGnssRov(), curGnssRef(), index_pairs, curState());

  // Add doppler residual blocks
  if (rtk_options_.estimate_velocity) {
    addDopplerResidualBlocks(curGnssRov(), curState(), num_valid_satellite);
  }

  // Add relative errors
  if (!isFirstEpoch()) {
    if (!rtk_options_.estimate_velocity) {
      // position
      addRelativePositionResidualBlock(lastState(), curState());
    }
    else {
      // position and velocity
      addRelativePositionAndVelocityBlock(lastState(), curState());
      // frequency
      addRelativeFrequencyBlock(lastState(), curState());
    }
    // ambiguity
    addRelativeAmbiguityResidualBlock(
      lastGnssRov(), curGnssRov(), lastAmbiguityState(), curAmbiguityState());
  }

  return true;
}

// Solve current graph
bool RtkEstimator::estimate()
{
  // Optimize with FDE
  if (gnss_base_options_.use_outlier_rejection)
  while (1)
  {
    optimize();

    // reject outlier
    if (!rejectPseudorangeOutlier(curState(), 
        gnss_base_options_.reject_one_outlier_once) && 
        !rejectPhaserangeOutlier(curState(), curAmbiguityState(),
        gnss_base_options_.reject_one_outlier_once)) break;
    if (!gnss_base_options_.reject_one_outlier_once) break;
  }
  // Optimize without FDE
  else {
    optimize();
  }

  // Ambiguity resolution
  curState().status = GnssSolutionStatus::Float;
  if (rtk_options_.use_ambiguity_resolution) {
    // get covariance of ambiguities
    Eigen::MatrixXd ambiguity_covariance;
    std::vector<uint64_t> parameter_block_ids;
    for (auto id : curAmbiguityState().ids) {
      parameter_block_ids.push_back(id.asInteger());
    }
    graph_->computeCovariance(parameter_block_ids, ambiguity_covariance);
    // solve
    AmbiguityResolution::Result ret = ambiguity_resolution_->solveRtk(
      curState().id, curAmbiguityState().ids, 
      ambiguity_covariance, gnss_measurement_pairs_.back());
    if (ret == AmbiguityResolution::Result::NlFix) {
      curState().status = GnssSolutionStatus::Fixed;
    }
  }

  // Log information
  if (base_options_.verbose_output) {
    LOG(INFO) << estimatorTypeToString(type_) << ": " 
      << "Iterations: " << graph_->summary.iterations.size() << ", "
      << std::scientific << std::setprecision(3) 
      << "Initial cost: " << graph_->summary.initial_cost << ", "
      << "Final cost: " << graph_->summary.final_cost
      << ", Sat number: " << std::setw(2) << num_satellites_
      << ", Fix status: " << std::setw(1) << static_cast<int>(curState().status);
  }

  // Apply marginalization
  marginalization();

  // Shift memory for states and measurements
  shiftMemory();

  return true;
}

// Marginalization
bool RtkEstimator::marginalization()
{
  // Check if we need marginalization
  if (states_.size() < rtk_options_.max_window_length) {
    return true;
  }

  // Erase old marginalization item
  if (!eraseOldMarginalization()) return false;

  // Add marginalization items
  // position
  addGnssPositionMarginBlockWithResiduals(oldestState());
  // ambiguity
  addAmbiguityMarginBlocksWithResiduals(oldestAmbiguityState());
  if (rtk_options_.estimate_velocity) {
    // velocity
    addGnssVelocityMarginBlockWithResiduals(oldestState());
    // frequency
    addFrequencyMarginBlocksWithResiduals(oldestState());
  }

  // Apply marginalization and add the item into graph
  return applyMarginalization();
}

};