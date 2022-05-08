/**
* @Function: Single differenced pseudorange positioning implementation
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/gnss/rtk_estimator.h"

#include "gici/gnss/pseudorange_error_dd.h"
#include "gici/gnss/phaserange_error_dd.h"
#include "gici/gnss/gnss_parameter_blocks.h"
#include "gici/gnss/gnss_common.h"
#include "gici/gnss/gnss_relative_errors.h"
#include "gici/gnss/ambiguity_error.h"

namespace gici {

// The default constructor
RtkEstimator::RtkEstimator(const RtkEstimatorOptions& options) :
  options_(options), graph_ptr_(std::make_shared<Graph>()),
  cauchy_loss_function_ptr_(new ceres::CauchyLoss(1)),
  huber_loss_function_ptr_(new ceres::HuberLoss(1)),
  marginalization_residual_id_(0)
{
  states_.push_back(State());
  measurements_.push_back(std::make_pair(GnssMeasurement(), GnssMeasurement()));

  marginalization_error_ptr_.reset(new MarginalizationError(*graph_ptr_.get()));

  ambiguity_resolution_.reset(
    new AmbiguityResolution(options.ambiguity_resolution, graph_ptr_));
}

// The default destructor
RtkEstimator::~RtkEstimator()
{}

// Add GNSS measurements and state
bool RtkEstimator::addGnssMeasurementAndState(
                    const GnssMeasurement& measurement_rov, 
                    const GnssMeasurement& measurement_ref)
{
  // Check timestamp
  differential_age_ = fabs(measurement_rov.timestamp - measurement_ref.timestamp);
  if (differential_age_ > options_.max_age) {
    LOG(WARNING) << "Max age between two measurements exceeded! "
      << "age = " << differential_age_ << ", max_age = " << options_.max_age;
    return false;
  }

  curMeasRov() = measurement_rov;
  curMeasRef() = measurement_ref;

  // Get last estimate
  Eigen::Vector3d last_position; last_position.setZero();
  if (!isFirstEpoch()) {
    last_position = getPositionEstimate();
  }

  // Add parameter blocks in current timestamp
  double timestamp = curMeasRov().timestamp;

  // position block
  BackendId position_id = createGnssPositionId(curMeasRov().id);
  Eigen::Vector3d position_prior = curMeasRov().position;
  if (!checkZero(last_position)) position_prior = last_position;
  std::shared_ptr<PositionParameterBlock> position_parameter_block = 
    std::make_shared<PositionParameterBlock>(position_prior, position_id.asInteger());
  if (!graph_ptr_->addParameterBlock(position_parameter_block)) {
    return false;
  }

  // Set to state
  curState().id = position_id;
  curState().timestamp = timestamp;

  // select single difference pairs
  GnssMeasurementDDIndexPairs dd_pairs = 
      gnss_common::formPhaserangeDDPair(curMeasRov(), curMeasRef(), options_.common);

  // Add pseudorange residual blocks
  int num_residual_block = dd_pairs.size();
  int num_satellites = 0;
  std::string last_prn = "";
  for (auto dd_pair : dd_pairs) 
  {
    GnssMeasurementIndex& index = dd_pair.rov;
    auto& satellite = curMeasRov().getSat(index);

    std::shared_ptr<PseudorangeErrorDD<3>> pseudorange_error = 
      std::make_shared<PseudorangeErrorDD<3>>(curMeasRov(), curMeasRef(),
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
    LOG(WARNING) << "No valid satellite! Input num_rov_sat = " 
                 << measurement_rov.satellites.size() << ", num_ref_sat = "
                 << measurement_ref.satellites.size();
    clearCurrentStateAndMeasurement();
    return false;
  }

  // Insufficient satellites
  // It is ok if it is not a first epoch, because we always have time constraints to 
  // ensure its observability.
  if (num_satellites < 4 && isFirstEpoch()) {
    LOG(WARNING) << "Insufficient satellites! Num = " << num_satellites 
                 << ". Input num_rov_sat = " << measurement_rov.satellites.size()
                 << ", num_ref_sat = " << measurement_ref.satellites.size();
    clearCurrentStateAndMeasurement();
    return false;
  }
  num_satellites_ = num_satellites;

  curState().status = GnssSolutionStatus::DGNSS;

  // Add Phaserange residual blocks and ambiguity states
  for (auto dd_pair : dd_pairs) 
  {
    auto& satellite = curMeasRov().getSat(dd_pair.rov);
    auto& satellite_base = curMeasRov().getSat(dd_pair.rov_base);
    auto& observation = satellite.observations.at(dd_pair.rov.code_type);
    auto& observation_base = satellite.observations.at(dd_pair.rov_base.code_type);

    int phase_id = gnss_common::getPhaseID(
      satellite.getSystem(), dd_pair.rov.code_type, observation.wavelength);
    BackendId ambiguity_id = 
      createGnssAmbiguityId(satellite.prn, phase_id, curMeasRov().id);
    BackendId ambiguity_base_id = 
      createGnssAmbiguityId(satellite_base.prn, phase_id, curMeasRov().id);

    // Create an ambiguity parameter block
    Eigen::Matrix<double, 1, 1> ambiguity_init;
    ambiguity_init(0, 0) = getInitialAmbiguity(curMeasRov(), curMeasRef(), 
      dd_pair.rov, dd_pair.ref);
    std::shared_ptr<AmbiguityParameterBlock> ambiguity_parameter_block = 
      std::make_shared<AmbiguityParameterBlock>(ambiguity_init, ambiguity_id.asInteger());
    if (!graph_ptr_->addParameterBlock(ambiguity_parameter_block)) {
      continue;
    }
    // add to state
    curState().ambiguity_ids.push_back(ambiguity_id);

    // Create an ambiguity parameter block for base satellite
    if (!graph_ptr_->parameterBlockExists(ambiguity_base_id.asInteger())) {
      ambiguity_init(0, 0) = getInitialAmbiguity(curMeasRov(), curMeasRef(), 
        dd_pair.rov_base, dd_pair.ref_base);
      std::shared_ptr<AmbiguityParameterBlock> ambiguity_base_parameter_block = 
        std::make_shared<AmbiguityParameterBlock>(ambiguity_init, ambiguity_base_id.asInteger());
      if (!graph_ptr_->addParameterBlock(ambiguity_base_parameter_block)) {
        continue;
      }
      // add to state
      curState().ambiguity_ids.push_back(ambiguity_base_id);

      // add a residual to avoid rand deficiency
      std::vector<double> coefficients = {1.0};
      std::shared_ptr<AmbiguityError1Coef> ambiguity_error = 
        std::make_shared<AmbiguityError1Coef>(ambiguity_init(0, 0), 1e-2, coefficients);
      graph_ptr_->addResidualBlock(ambiguity_error, nullptr, 
        ambiguity_base_parameter_block);
    }

    // Add residual block
    std::shared_ptr<PhaserangeErrorDD<3, 1, 1>> phaserange_error = 
      std::make_shared<PhaserangeErrorDD<3, 1, 1>>(curMeasRov(), curMeasRef(),
      dd_pair.rov, dd_pair.ref, dd_pair.rov_base, dd_pair.ref_base, options_.error_parameter);
    // TODO: we thought that the phaseranges are far less affected by uncontrollable noises, 
    // so we unused robust function for phase to make it faster convergence. But the result was 
    // not as good as we expected.
    graph_ptr_->addResidualBlock(phaserange_error, 
      huber_loss_function_ptr_ ? huber_loss_function_ptr_.get() : nullptr,
      graph_ptr_->parameterBlockPtr(position_id.asInteger()),
      graph_ptr_->parameterBlockPtr(ambiguity_id.asInteger()), 
      graph_ptr_->parameterBlockPtr(ambiguity_base_id.asInteger()));

    curState().status = GnssSolutionStatus::Float;
  }

  // Time contraints
  if (!isFirstEpoch()) 
  {
    // Relative position
    Eigen::Matrix3d relative_position_information = Eigen::Matrix3d::Identity() * 1e-4;
    std::shared_ptr<RelativePositionError> relative_position_error = 
      std::make_shared<RelativePositionError>(relative_position_information);
    graph_ptr_->addResidualBlock(relative_position_error, nullptr,
      graph_ptr_->parameterBlockPtr(lastState().id.asInteger()),
      graph_ptr_->parameterBlockPtr(curState().id.asInteger()));

    // cycle slip detection
    cycleSlipDetectionSD(lastMeasRov(), lastMeasRef(), 
      curMeasRov(), curMeasRef(), options_.common);
    
    // Relative ambiguity
    for (size_t i = 0; i < lastState().ambiguity_ids.size(); i++) {
      // find in current epoch
      for (size_t j = 0; j < curState().ambiguity_ids.size(); j++) {
        if (!sameAmbiguity(lastState().ambiguity_ids[i], curState().ambiguity_ids[j])) continue;
        
        // check cycle slip
        std::string prn = curState().ambiguity_ids[j].gPrn();
        int phase_id = curState().ambiguity_ids[j].gPhaseId();
        Satellite satellite = curMeasRov().getSat(prn);
        bool slip = false;

        for (auto obs : satellite.observations) {
          if (gnss_common::getPhaseID(satellite.getSystem(), 
            obs.first, obs.second.wavelength) == phase_id) {
            slip = obs.second.slip;
          }
        }
        // if slip happened, we do not add ambiguity time constraint
        if (slip) {
          // LOG(INFO) << "Cycle slip detected! Prn = " << prn << ", phase_id = " << phase_id;
          continue;
        }

        // Add constraint
        Eigen::Matrix<double, 1, 1> relative_ambiguity_information = 
          Eigen::Matrix<double, 1, 1>::Identity() * 1e8;
        std::shared_ptr<RelativeAmbiguityError> relative_ambiguity_error = 
          std::make_shared<RelativeAmbiguityError>(relative_ambiguity_information);
        graph_ptr_->addResidualBlock(relative_ambiguity_error, nullptr,
          graph_ptr_->parameterBlockPtr(lastState().ambiguity_ids[i].asInteger()),
          graph_ptr_->parameterBlockPtr(curState().ambiguity_ids[j].asInteger()));
      }
    }
  }

  return true;
}

// Start ceres optimization
void RtkEstimator::optimize()
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

  // Ambiguity resolution
  setPositionEstimateToMeas();
  if (options_.use_ambiguity_resolution && 
    ambiguity_resolution_->solve(
    curState().id, curState().ambiguity_ids, measurements_.back())) {
    curState().status = GnssSolutionStatus::Fixed;
  }

  if (options_.verbose) {
    LOG(INFO) << graph_ptr_->summary.BriefReport();
  }

  // Marginalization
  marginalization();

  // Shift state and measurement
  states_.push_back(State());
  measurements_.push_back(std::make_pair(GnssMeasurement(), GnssMeasurement()));
  if (states_.size() > options_.window_length) {
    states_.pop_front();
    measurements_.pop_front();
  }
}

// Get position in ECEF coordinate
Eigen::Vector3d RtkEstimator::getPositionEstimate()
{
  State& state = lastState();
  if (!graph_ptr_->parameterBlockExists(state.id.asInteger())) {
    return Eigen::Vector3d::Zero();
  }

  std::shared_ptr<ParameterBlock> base_ptr =
      graph_ptr_->parameterBlockPtr(state.id.asInteger());
  if (base_ptr != nullptr) {
    std::shared_ptr<PositionParameterBlock> block_ptr = 
      std::dynamic_pointer_cast<PositionParameterBlock>(base_ptr);
    CHECK(block_ptr != nullptr);
    return block_ptr->estimate();
  }

  return Eigen::Vector3d::Zero();
}

// Get solution
GnssSolution RtkEstimator::getSolution()
{
  GnssSolution solution;
  std::vector<uint64_t> parameter_block_ids;

  // Position
  State& state = lastState();
  solution.timestamp = state.timestamp;
  solution.id = lastMeasRov().id;
  solution.status = GnssSolutionStatus::None;
  solution.covariance.setZero();
  solution.position.setZero();
  solution.velocity.setZero();
  solution.num_satellites = num_satellites_;
  solution.differential_age = differential_age_;
  if (!graph_ptr_->parameterBlockExists(state.id.asInteger())) {
    return solution;
  }
  else {
    parameter_block_ids.push_back(state.id.asInteger());

    std::shared_ptr<ParameterBlock> base_ptr =
        graph_ptr_->parameterBlockPtr(state.id.asInteger());
    if (base_ptr != nullptr) {
      std::shared_ptr<PositionParameterBlock> block_ptr = 
        std::dynamic_pointer_cast<PositionParameterBlock>(base_ptr);
      CHECK(block_ptr != nullptr);
      solution.position = block_ptr->estimate();
    }
  }

  // velocity
  BackendId velocity_id = changeIdType(state.id, IdType::gVelocity);
  if (!graph_ptr_->parameterBlockExists(velocity_id.asInteger())) {
    // we did not estimate velocity
    // get the position covariance and return
    Eigen::MatrixXd position_covariance;
    graph_ptr_->computeCovariance(parameter_block_ids, position_covariance);
    CHECK(position_covariance.cols() == 3);
    solution.covariance.topLeftCorner(3, 3) = position_covariance;
  }
  else {
    parameter_block_ids.push_back(velocity_id.asInteger());

    std::shared_ptr<ParameterBlock> base_ptr =
        graph_ptr_->parameterBlockPtr(velocity_id.asInteger());
    if (base_ptr != nullptr) {
      std::shared_ptr<VelocityParameterBlock> block_ptr = 
        std::dynamic_pointer_cast<VelocityParameterBlock>(base_ptr);
      CHECK(block_ptr != nullptr);
      solution.velocity = block_ptr->estimate();
    }

    Eigen::MatrixXd covariance;
    graph_ptr_->computeCovariance(parameter_block_ids, covariance);
    CHECK(covariance.cols() == 6);
    solution.covariance = covariance;
  }

  // status
  solution.status = getSolutionStatus();

  return solution;
}

// Compute initial ambiguity
double RtkEstimator::getInitialAmbiguity(
            const GnssMeasurement& measurement_rov, 
            const GnssMeasurement& measurement_ref,
            const GnssMeasurementIndex& index_rov,
            const GnssMeasurementIndex& index_ref)
{
  auto& observation_1 = measurement_rov.satellites.at(index_rov.prn).
                        observations.at(index_rov.code_type);
  double pseudorange_1 = observation_1.pseudorange;
  double phaserange_1 = observation_1.phaserange;

  auto& observation_2 = measurement_ref.satellites.at(index_ref.prn).
                        observations.at(index_ref.code_type);
  double pseudorange_2 = observation_2.pseudorange;
  double phaserange_2 = observation_2.phaserange;

  return phaserange_1 - phaserange_2 - (pseudorange_1 - pseudorange_2);
}

// Marginalization
bool RtkEstimator::marginalization()
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

  // Position parameter
  BackendId position_id = oldestState().id;
  if (graph_ptr_->parameterBlockExists(position_id.asInteger())) {
    parameter_blocks_to_be_marginalized.push_back(position_id.asInteger());
    keep_parameter_blocks.push_back(false);

    // Get all residuals connected to this state.
    Graph::ResidualBlockCollection residuals = 
      graph_ptr_->residuals(position_id.asInteger());
    for (size_t r = 0; r < residuals.size(); ++r) {
      marginalization_error_ptr_->addResidualBlock(
            residuals[r].residual_block_id);
    }
  }

  // Ambiguity parameter
  auto& ambiguity_ids = oldestState().ambiguity_ids;
  for (size_t i = 0; i < ambiguity_ids.size(); i++) {
    BackendId ambiguity_id = ambiguity_ids[i];
    if (!graph_ptr_->parameterBlockExists(ambiguity_id.asInteger())) continue;
    parameter_blocks_to_be_marginalized.push_back(ambiguity_id.asInteger());
    keep_parameter_blocks.push_back(false);

    Graph::ResidualBlockCollection residuals = 
      graph_ptr_->residuals(ambiguity_id.asInteger());
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

// Delete current states and measurement
void RtkEstimator::clearCurrentStateAndMeasurement()
{
  // Position parameter
  BackendId position_id = curState().id;
  if (graph_ptr_->parameterBlockExists(position_id.asInteger())) {
    graph_ptr_->removeParameterBlock(position_id.asInteger());
  }

  // Ambiguity parameter
  auto& ambiguity_ids = curState().ambiguity_ids;
  for (size_t i = 0; i < ambiguity_ids.size(); i++) {
    BackendId ambiguity_id = ambiguity_ids[i];
    if (!graph_ptr_->parameterBlockExists(ambiguity_id.asInteger())) continue;
    graph_ptr_->removeParameterBlock(ambiguity_id.asInteger());
  }

  // Clear current state
  curState().clear();
}

// Set position estimate to measurement 
void RtkEstimator::setPositionEstimateToMeas()
{
  State& state = curState();
  if (!graph_ptr_->parameterBlockExists(state.id.asInteger())) {
    return;
  }

  std::shared_ptr<ParameterBlock> base_ptr =
      graph_ptr_->parameterBlockPtr(state.id.asInteger());
  if (base_ptr != nullptr) {
    std::shared_ptr<PositionParameterBlock> block_ptr = 
      std::dynamic_pointer_cast<PositionParameterBlock>(base_ptr);
    CHECK(block_ptr != nullptr);
    curMeasRov().position = block_ptr->estimate();
  }
}

}