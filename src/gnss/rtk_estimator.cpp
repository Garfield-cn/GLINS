/**
* @Function: Single differenced pseudorange positioning implementation
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/gnss/rtk_estimator.h"

#include "gici/gnss/pseudorange_error_sd.h"
#include "gici/gnss/phaserange_error_sd.h"
#include "gici/gnss/gnss_parameter_blocks.h"
#include "gici/gnss/gnss_common.h"
#include "gici/gnss/gnss_relative_const_errors.h"

namespace gici {

// The default constructor
RTKEstimator::RTKEstimator(const RTKEstimatorOptions& options) :
  options_(options), graph_ptr_(std::make_shared<Graph>()),
  cauchy_loss_function_ptr_(new ceres::CauchyLoss(1)),
  huber_loss_function_ptr_(new ceres::HuberLoss(1)),
  marginalization_residual_id_(0)
{
  states_.push_back(State());
  measurements_.push_back(std::make_pair(GNSSMeasurement(), GNSSMeasurement()));

  marginalization_error_ptr_.reset(new MarginalizationError(*graph_ptr_.get()));

  ambiguity_resolution_.reset(
    new AmbiguityResolution(options.ambiguity_resolution, graph_ptr_));
}

// The default destructor
RTKEstimator::~RTKEstimator()
{}

// Add GNSS measurements and state
bool RTKEstimator::addGNSSMeasurementAndState(
                    const GNSSMeasurement& measurement_rov, 
                    const GNSSMeasurement& measurement_ref)
{
  // Check timestamp
  if (fabs(measurement_rov.timestamp - measurement_ref.timestamp) > options_.max_age) {
    LOG(WARNING) << "Max age between two measurements exceeded! "
      << "age = " << fabs(measurement_rov.timestamp - measurement_ref.timestamp)
      << "max_age = " << options_.max_age;
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
  BackendId position_id = createGNSSPositionId(curMeasRov().id);
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
  GNSSMeasurementIndexPairs pseudorange_index_pairs = 
      gnss_common::formPseudorangePair(curMeasRov(), curMeasRef(), options_.common);
  GNSSMeasurementIndexPairs phaserange_index_pairs = 
      gnss_common::formPhaserangePair(curMeasRov(), curMeasRef(), options_.common);

  // check if any system does not have vaild satellite
  std::map<char, int> system_observation_cnt;
  for (auto index_pair : pseudorange_index_pairs) 
  {
    GNSSMeasurementIndex& index = index_pair.first;
    auto& satellite = curMeasRov().getSat(index);
    char system = satellite.getSystem();
    if (system_observation_cnt.find(system) == system_observation_cnt.end()) 
      system_observation_cnt.insert(std::make_pair(system, 0));
    system_observation_cnt.at(system)++;
  }

  // clock blocks
  int num_clock_blocks = 0;
  for (auto index_pair : pseudorange_index_pairs) 
  {
    GNSSMeasurementIndex& index = index_pair.first;
    auto& satellite = curMeasRov().getSat(index);
    BackendId clock_id = createGNSSClockId(satellite.getSystem(), curMeasRov().id);
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
    }
  }

  // Pseudorange ---------------------------------------------------
  // Add pseudorange residual blocks
  int num_residual_block = pseudorange_index_pairs.size();
  int num_satellites = 0;
  std::string last_prn = "";
  for (auto index_pair : pseudorange_index_pairs) 
  {
    GNSSMeasurementIndex& index = index_pair.first;
    auto& satellite = curMeasRov().getSat(index);

    BackendId clock_id = createGNSSClockId(satellite.getSystem(), curMeasRov().id);
    std::shared_ptr<PseudorangeErrorSD<3, 1>> pseudorange_error = 
      std::make_shared<PseudorangeErrorSD<3, 1>>(curMeasRov(), curMeasRef(),
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
    LOG(WARNING) << "No valid satellite! Input num_rov_sat = " 
                 << measurement_rov.satellites.size() << ", num_ref_sat = "
                 << measurement_ref.satellites.size();
    clearCurrentStateAndMeasurement();
    return false;
  }

  // Insufficient satellites
  // It is ok if it is not a first epoch, because we always have time constraints to 
  // ensure its observability.
  if (num_satellites < num_clock_blocks + 4 && isFirstEpoch()) {
    LOG(WARNING) << "Insufficient satellites! Num = " << num_satellites 
                 << ". Input num_rov_sat = " << measurement_rov.satellites.size()
                 << ", num_ref_sat = " << measurement_ref.satellites.size();
    clearCurrentStateAndMeasurement();
    return false;
  }

  // Phaserange ---------------------------------------------------
  // Add Phaserange residual blocks and ambiguity states
  for (auto index_pair : phaserange_index_pairs) 
  {
    GNSSMeasurementIndex& index = index_pair.first;
    auto& satellite = curMeasRov().getSat(index);
    auto& observation = satellite.observations.at(index.code_type);

    BackendId clock_id = createGNSSClockId(satellite.getSystem(), curMeasRov().id);

    int phase_id = gnss_common::getPhaseID(
      satellite.getSystem(), index.code_type, observation.wavelength);
    BackendId ambiguity_id = createGNSSAmbiguityId(satellite.prn, phase_id, curMeasRov().id);

    // Create an ambiguity parameter block
    Eigen::Matrix<double, 1, 1> ambiguity_init;
    ambiguity_init(0, 0) = getInitialAmbiguity(curMeasRov(), curMeasRef(), 
      index_pair.first, index_pair.second);
    std::shared_ptr<AmbiguityParameterBlock> ambiguity_parameter_block = 
      std::make_shared<AmbiguityParameterBlock>(ambiguity_init, ambiguity_id.asInteger());
    if (!graph_ptr_->addParameterBlock(ambiguity_parameter_block)) {
      continue;
    }
    // add to state
    curState().ambiguity_ids.push_back(ambiguity_id);

    // Add residual block
    std::shared_ptr<PhaserangeErrorSD<3, 1, 1>> phaserange_error = 
      std::make_shared<PhaserangeErrorSD<3, 1, 1>>(curMeasRov(), curMeasRef(),
      index_pair.first, index_pair.second, options_.error_parameter);
    // TODO: we thought that the phaseranges are far less affected by uncontrollable noises, 
    // so we unused robust function for phase to make it faster convergence. But the result was 
    // not as good as we expected.
    graph_ptr_->addResidualBlock(phaserange_error, 
      huber_loss_function_ptr_ ? huber_loss_function_ptr_.get() : nullptr,
      graph_ptr_->parameterBlockPtr(position_id.asInteger()),
      graph_ptr_->parameterBlockPtr(clock_id.asInteger()), 
      graph_ptr_->parameterBlockPtr(ambiguity_id.asInteger()));
  }

  // Time constraints ---------------------------------------------------
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
        if (slip) continue;

        // Add constraint
        Eigen::Matrix<double, 1, 1> relative_ambiguity_information = 
          Eigen::Matrix<double, 1, 1>::Identity() * 1e6;
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
void RTKEstimator::optimize()
{
  graph_ptr_->options.linear_solver_type = ceres::DENSE_SCHUR;
  graph_ptr_->options.trust_region_strategy_type = ceres::DOGLEG;
  graph_ptr_->options.num_threads = options_.num_threads;
  graph_ptr_->options.max_num_iterations = options_.max_iteration;

  // For debug
  // debug_callback_.reset(new CeresDebugCallback());
  // graph_ptr_->options.callbacks.push_back(debug_callback_.get());

  if (options_.verbose) {
    graph_ptr_->options.logging_type = ceres::LoggingType::SILENT;
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
  ambiguity_resolution_->solve(
    curState().id, curState().ambiguity_ids, measurements_.back());

  if (options_.verbose) {
    LOG(INFO) << graph_ptr_->summary.BriefReport();
#if 0
    std::ofstream outfile;
    outfile.open("/home/cc/datasets/tmp/log.txt", std::ios::out | std::ios::trunc);
    outfile << graph_ptr_->summary.BriefReport() << std::endl;
    for (auto residual_map : graph_ptr_->residual_block_id_to_residual_block_spec_map_) {
      auto residual = residual_map.first;
      auto& error_interface_ptr = residual_map.second.error_interface_ptr;
      size_t size = residual_map.second.error_interface_ptr->residualDim();

      Eigen::VectorXd Residuals = Eigen::VectorXd::Zero(size);
      graph_ptr_->problem_->EvaluateResidualBlock(
          residual, false, nullptr, Residuals.data(), nullptr);
      // if (Residuals.maxCoeff() > 1.0 || Residuals.minCoeff() < -1.0)
      // if (size > 2)
        outfile << "Residual " << static_cast<int>(error_interface_ptr->typeInfo()) << ": " 
                << residual << ": " << Residuals.transpose() << std::endl;

        // auto parameters = graph_ptr_->parameters(residual);
        // for (Graph::ParameterBlockSpec &parameter : parameters) {
        //   outfile << "    Parameter: " << parameter.first << std::endl;
        // }
    }
    outfile.close();

    if (graph_ptr_->summary.final_cost > 1) LOG(FATAL) << "aa";
#endif

  }

  // test: get covariance
  // std::vector<uint64_t> parameter_block_ids;
  // parameter_block_ids.push_back(curState().id.asInteger());
  // parameter_block_ids.push_back(changeIdType(curState().id, IdType::gClock, 'G').asInteger());
  // parameter_block_ids.push_back(curState().ambiguity_ids[0].asInteger());
  // Eigen::MatrixXd covariance;
  // graph_ptr_->computeCovariance(parameter_block_ids, covariance);
  // LOG(INFO) << "Covariance: " << std::endl << covariance;

  // Marginalization
  marginalization();

  // Shift state and measurement
  states_.push_back(State());
  measurements_.push_back(std::make_pair(GNSSMeasurement(), GNSSMeasurement()));
  if (states_.size() > options_.window_length) {
    states_.pop_front();
    measurements_.pop_front();
  }
}

// Get position in ECEF coordinate
Eigen::Vector3d RTKEstimator::getPositionEstimate()
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
    CHECK(block_ptr != nullptr) << "Incorrect pointer cast detected!";
    return block_ptr->estimate();
  }

  return Eigen::Vector3d::Zero();
}

// Get Satellite clock
double RTKEstimator::getClockEstimate(const char system, double& dclock)
{
  State& state = lastState();
  BackendId id = changeIdType(state.id, IdType::gClock, system);
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

// Compute initial ambiguity
double RTKEstimator::getInitialAmbiguity(
            const GNSSMeasurement& measurement_rov, 
            const GNSSMeasurement& measurement_ref,
            const GNSSMeasurementIndex& index_rov,
            const GNSSMeasurementIndex& index_ref)
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
bool RTKEstimator::marginalization()
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

  // Clock parameter
  for (size_t i = 0; i < GNSSSystems.size(); i++) {
    BackendId clock_id = changeIdType(position_id, IdType::gClock, GNSSSystems[i]);
    if (!graph_ptr_->parameterBlockExists(clock_id.asInteger())) continue;
    parameter_blocks_to_be_marginalized.push_back(clock_id.asInteger());
    keep_parameter_blocks.push_back(false);

    Graph::ResidualBlockCollection residuals = 
      graph_ptr_->residuals(clock_id.asInteger());
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
void RTKEstimator::clearCurrentStateAndMeasurement()
{
  // Position parameter
  BackendId position_id = curState().id;
  if (graph_ptr_->parameterBlockExists(position_id.asInteger())) {
    graph_ptr_->removeParameterBlock(position_id.asInteger());
  }

  // Clock parameter
  for (size_t i = 0; i < GNSSSystems.size(); i++) {
    BackendId clock_id = changeIdType(position_id, IdType::gClock, GNSSSystems[i]);
    if (!graph_ptr_->parameterBlockExists(clock_id.asInteger())) continue;
    graph_ptr_->removeParameterBlock(clock_id.asInteger());
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
void RTKEstimator::setPositionEstimateToMeas()
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
    CHECK(block_ptr != nullptr) << "Incorrect pointer cast detected!";
    curMeasRov().position = block_ptr->estimate();
  }
}

}