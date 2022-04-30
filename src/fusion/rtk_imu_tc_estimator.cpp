/**
* @Function: RTK/IMU tightly integration
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/fusion/rtk_imu_tc_estimator.h"
#include "gici/gnss/position_error.h"
#include "gici/gnss/pseudorange_error_dd.h"
#include "gici/gnss/phaserange_error_dd.h"
#include "gici/gnss/gnss_parameter_blocks.h"
#include "gici/gnss/gnss_common.h"
#include "gici/gnss/gnss_relative_errors.h"
#include "gici/gnss/ambiguity_error.h"
#include "gici/imu/imu_common.h"
#include "gici/imu/imu_error.h"
#include "gici/utility/common.h"
#include "gici/estimate/pose_error.h"
#include "gici/imu/speed_and_bias_error.h"
#include "gici/imu/yaw_error.h"

namespace gici {

// The default constructor
RtkImuTcEstimator::RtkImuTcEstimator(
                     const RtkImuTcEstimatorOptions& options, 
                     const GnssImuInitializationOptions& initial_options) :
  options_(options), graph_ptr_(std::make_shared<Graph>()),
  cauchy_loss_function_ptr_(new ceres::CauchyLoss(1)),
  huber_loss_function_ptr_(new ceres::HuberLoss(1)),
  marginalization_residual_id_(0), imu_initialized_(false),
  initial_options_(initial_options)
{
  marginalization_error_ptr_.reset(new MarginalizationError(*graph_ptr_.get()));

  initializer_.reset(new GnssImuInitialization(initial_options, graph_ptr_));

  // RTK estimator is only used for initialization
  RtkEstimatorOptions rtk_options;
  rtk_options.max_iteration = options.max_iteration;
  rtk_options.num_threads = options.num_threads;
  rtk_options.max_age = options.max_age;
  rtk_options.use_ambiguity_resolution = options.use_ambiguity_resolution;
  rtk_options.common = options.gnss_common;
  rtk_options.error_parameter = options.gnss_error_parameter;
  rtk_options.ambiguity_resolution = options.ambiguity_resolution;
  rtk_estimator_.reset(new RtkEstimator(rtk_options));

  ambiguity_resolution_.reset(
    new AmbiguityResolution(options.ambiguity_resolution, graph_ptr_));
}

// The default destructor
RtkImuTcEstimator::~RtkImuTcEstimator()
{}

// Add GNSS measurements and state
bool RtkImuTcEstimator::addGnssMeasurementAndState(
                    const GnssMeasurement& measurement_rov, 
                    const GnssMeasurement& measurement_ref)
{
  // Check timestamp
  if (fabs(measurement_rov.timestamp - measurement_ref.timestamp) > options_.max_age) {
    LOG(WARNING) << "Max age between two measurements exceeded! "
      << "age = " << fabs(measurement_rov.timestamp - measurement_ref.timestamp)
      << ", max_age = " << options_.max_age;
    return false;
  }

  // Wait for IMU data
  double timestamp = measurement_rov.timestamp;
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
    // store measurements
    gnss_measurements_.push_back(std::make_pair(measurement_rov, measurement_ref));

    // get RTK solution
    if (!rtk_estimator_->addGnssMeasurementAndState(measurement_rov, measurement_ref)) {
      return false;
    }
    rtk_estimator_->optimize();
    GnssSolution gnss_solution = rtk_estimator_->getSolution();

    // initialize
    initializer_->addGnssMeasurement(gnss_solution);
    if (!initializer_->getResult(
      T_WS, cov_T_WS, speed_and_bias, cov_speed_and_bias, t_SR_S, cov_t_SR_S, true)) {
      LOG(INFO) << "Initializing...";
      return false;
    }
    else {
      // add used states and measurments
      std::deque<GnssSolution> gnss_solutions;
      initializer_->marginalization(options_.window_length, 
        marginalization_error_ptr_, gnss_solutions, marginalization_residual_id_);
      for (auto& gnss : gnss_solutions) {
        states_.push_back(State());
        states_.back().timestamp = gnss.timestamp;
        states_.back().id = createGnssPoseId(gnss.id);
      }

      // delete useless measuremnets
      while (gnss_measurements_.size() > gnss_solutions.size()) {
        gnss_measurements_.pop_front();
      }

      imu_initialized_ = true;
      return true;
    }
  }

  // Add measurement
  curGnssRov() = measurement_rov;
  curGnssRef() = measurement_ref;

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
  BackendId pose_id = createGnssPoseId(curGnssRov().id);
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

  // Set to state
  curState().id = pose_id;
  curState().timestamp = timestamp;

  // select single difference pairs
  GnssMeasurementDDIndexPairs dd_pairs = gnss_common::formPhaserangeDDPair(
    curGnssRov(), curGnssRef(), options_.gnss_common);

  // Add pseudorange residual blocks
  for (auto dd_pair : dd_pairs) 
  {
    GnssMeasurementIndex& index = dd_pair.rov;
    auto& satellite = curGnssRov().getSat(index);

    std::shared_ptr<PseudorangeErrorDD<7, 3>> pseudorange_error = 
      std::make_shared<PseudorangeErrorDD<7, 3>>(curGnssRov(), curGnssRef(),
      dd_pair.rov, dd_pair.ref, dd_pair.rov_base, dd_pair.ref_base, 
      options_.gnss_error_parameter);
    pseudorange_error->setCoordinate(coordinate_);
    graph_ptr_->addResidualBlock(pseudorange_error, 
      huber_loss_function_ptr_ ? huber_loss_function_ptr_.get() : nullptr,
      graph_ptr_->parameterBlockPtr(pose_id.asInteger()), 
      graph_ptr_->parameterBlockPtr(gnss_extrinsic_id.asInteger()));
  }

  // we do not need to check satellite number here, because the TC estimator could
  // work under GNSS insufficient case.

  curState().status = GnssSolutionStatus::DGNSS;

  // Add Phaserange residual blocks and ambiguity states
  for (auto dd_pair : dd_pairs) 
  {
    auto& satellite = curGnssRov().getSat(dd_pair.rov);
    auto& satellite_base = curGnssRov().getSat(dd_pair.rov_base);
    auto& observation = satellite.observations.at(dd_pair.rov.code_type);
    auto& observation_base = satellite.observations.at(dd_pair.rov_base.code_type);

    int phase_id = gnss_common::getPhaseID(
      satellite.getSystem(), dd_pair.rov.code_type, observation.wavelength);
    BackendId ambiguity_id = 
      createGnssAmbiguityId(satellite.prn, phase_id, curGnssRov().id);
    BackendId ambiguity_base_id = 
      createGnssAmbiguityId(satellite_base.prn, phase_id, curGnssRov().id);

    // Create an ambiguity parameter block
    Eigen::Matrix<double, 1, 1> ambiguity_init;
    ambiguity_init(0, 0) = rtk_estimator_->getInitialAmbiguity(
      curGnssRov(), curGnssRef(), dd_pair.rov, dd_pair.ref);
    std::shared_ptr<AmbiguityParameterBlock> ambiguity_parameter_block = 
      std::make_shared<AmbiguityParameterBlock>(ambiguity_init, ambiguity_id.asInteger());
    if (!graph_ptr_->addParameterBlock(ambiguity_parameter_block)) {
      continue;
    }
    // add to state
    curState().ambiguity_ids.push_back(ambiguity_id);

    // Create an ambiguity parameter block for base satellite
    if (!graph_ptr_->parameterBlockExists(ambiguity_base_id.asInteger())) {
      ambiguity_init(0, 0) = rtk_estimator_->getInitialAmbiguity(
        curGnssRov(), curGnssRef(), dd_pair.rov_base, dd_pair.ref_base);
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
        std::make_shared<AmbiguityError1Coef>(ambiguity_init(0, 0), 1.0, coefficients);
      graph_ptr_->addResidualBlock(ambiguity_error, nullptr, 
        ambiguity_base_parameter_block);
    }

    // Add residual block
    std::shared_ptr<PhaserangeErrorDD<7, 3, 1, 1>> phaserange_error = 
      std::make_shared<PhaserangeErrorDD<7, 3, 1, 1>>(curGnssRov(), curGnssRef(),
      dd_pair.rov, dd_pair.ref, dd_pair.rov_base, dd_pair.ref_base, 
      options_.gnss_error_parameter);
    phaserange_error->setCoordinate(coordinate_);
    graph_ptr_->addResidualBlock(phaserange_error, 
      huber_loss_function_ptr_ ? huber_loss_function_ptr_.get() : nullptr,
      graph_ptr_->parameterBlockPtr(pose_id.asInteger()),
      graph_ptr_->parameterBlockPtr(gnss_extrinsic_id.asInteger()),
      graph_ptr_->parameterBlockPtr(ambiguity_id.asInteger()), 
      graph_ptr_->parameterBlockPtr(ambiguity_base_id.asInteger()));

    curState().status = GnssSolutionStatus::Float;
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

    // cycle slip detection
    cycleSlipDetectionSD(lastGnssRov(), lastGnssRef(), 
      curGnssRov(), curGnssRef(), options_.gnss_common);
    
    // Relative ambiguity
    for (size_t i = 0; i < lastState().ambiguity_ids.size(); i++) {
      // find in current epoch
      for (size_t j = 0; j < curState().ambiguity_ids.size(); j++) {
        if (!sameAmbiguity(lastState().ambiguity_ids[i], curState().ambiguity_ids[j])) continue;
        
        // check cycle slip
        std::string prn = curState().ambiguity_ids[j].gPrn();
        int phase_id = curState().ambiguity_ids[j].gPhaseId();
        Satellite satellite = curGnssRov().getSat(prn);
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

// Add IMU measurement
void RtkImuTcEstimator::addImuMeasurement(const ImuMeasurement& imu_measurement)
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
void RtkImuTcEstimator::optimize()
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
  if (options_.use_ambiguity_resolution && 
    ambiguity_resolution_->solve(
    curState().id, curState().ambiguity_ids, gnss_measurements_.back())) {
    curState().status = GnssSolutionStatus::Fixed;
  }

  if (options_.verbose) {
    LOG(INFO) << graph_ptr_->summary.BriefReport();
  }

  // Marginalization
  marginalization();

  // Shift state and measurement
  states_.push_back(State());
  gnss_measurements_.push_back(std::make_pair(GnssMeasurement(), GnssMeasurement()));
  if (states_.size() > options_.window_length) {
    states_.pop_front();
    gnss_measurements_.pop_front();
  }
}

// Get pose
Transformation RtkImuTcEstimator::getPoseEstimate()
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
SpeedAndBias RtkImuTcEstimator::getSpeedAndBias()
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
Eigen::Vector3d RtkImuTcEstimator::getGnssExtrinsic()
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
bool RtkImuTcEstimator::marginalization()
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

// Set position estimate to measurement 
void RtkImuTcEstimator::setPositionEstimateToMeas()
{
  State& state = curState();
  if (!graph_ptr_->parameterBlockExists(state.id.asInteger())) {
    return;
  }

  std::shared_ptr<ParameterBlock> base_ptr =
      graph_ptr_->parameterBlockPtr(state.id.asInteger());
  if (base_ptr != nullptr) {
    std::shared_ptr<PoseParameterBlock> block_ptr = 
      std::dynamic_pointer_cast<PoseParameterBlock>(base_ptr);
    CHECK(block_ptr != nullptr);
    curGnssRov().position = coordinate_->convert(
      block_ptr->estimate().getPosition(), GeoType::ENU, GeoType::ECEF);
  }
}

}