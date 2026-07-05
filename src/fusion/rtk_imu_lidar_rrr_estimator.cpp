/**
 * @Function: GNSS/IMU/LiDAR tightly coupled estimator using RTK
 *
 * @Author  : Jiahui Liu
 * @Email   : jh.liu@sjtu.edu.cn
 *
 * Copyright (C) 2024 by Jiahui Liu, All rights reserved.
 **/
#include "gici/fusion/rtk_imu_lidar_rrr_estimator.h"

#include <iomanip>

#include "gici/gnss/position_error.h"

namespace gici {

// The default constructor
RtkImuLidarRrrEstimator::RtkImuLidarRrrEstimator(
    const RtkImuLidarRrrEstimatorOptions& options, const GnssImuInitializerOptions& init_options,
    const RtkEstimatorOptions rtk_options, const GnssEstimatorBaseOptions& gnss_base_options,
    const GnssLooseEstimatorBaseOptions& gnss_loose_base_options,
    const LidarEstimatorBaseOptions& lidar_base_options,
    const ImuEstimatorBaseOptions& imu_base_options, const EstimatorBaseOptions& base_options,
    const AmbiguityResolutionOptions& ambiguity_options)
    : EstimatorBase(base_options),
      GnssEstimatorBase(gnss_base_options, base_options),
      LidarEstimatorBase(lidar_base_options, base_options),
      ImuEstimatorBase(imu_base_options, base_options),
      rrr_options_(options),
      rtk_options_(rtk_options)
{
  type_ = EstimatorType::RtkImuLidarRrr;
  is_use_phase_ = true;
  states_.push_back(State());
  gnss_measurement_pairs_.push_back(std::make_pair(GnssMeasurement(), GnssMeasurement()));
  scans_.push_back(nullptr);
  num_satellites_ = 0;

  // Initialization control
  initializer_sub_estimator_.reset(
      new RtkEstimator(rtk_options, gnss_base_options, base_options, ambiguity_options));
  gnss_imu_initializer_.reset(new GnssImuInitializer(init_options, gnss_loose_base_options,
                                                     imu_base_options, base_options, graph_,
                                                     initializer_sub_estimator_));

  // Ambiguity resolution
  ambiguity_resolution_.reset(new AmbiguityResolution(ambiguity_options, graph_));

  // RTK estimator used for ambiguity covariance estimation
  RtkEstimatorOptions sub_rtk_options = rtk_options;
  EstimatorBaseOptions sub_base_options = base_options;
  GnssEstimatorBaseOptions sub_gnss_base_options = gnss_base_options;
  sub_rtk_options.use_ambiguity_resolution = false;
  sub_rtk_options.max_window_length = 2;
  sub_base_options.verbose_output = false;
  sub_gnss_base_options.use_outlier_rejection = false;
  ambiguity_covariance_estimator_.reset(new RtkEstimator(sub_rtk_options, sub_gnss_base_options,
                                                         sub_base_options, ambiguity_options));
}

// The default destructor
RtkImuLidarRrrEstimator::~RtkImuLidarRrrEstimator()
{}

// Add measurement
bool RtkImuLidarRrrEstimator::addMeasurement(const EstimatorDataCluster& measurement)
{
  // GNSS/IMU initialization
  if (coordinate_ == nullptr || !gravity_setted_) return false;
  if (!gnss_imu_initializer_->finished()) {
    if (gnss_imu_initializer_->getCoordinate() == nullptr) {
      gnss_imu_initializer_->setCoordinate(coordinate_);
      initializer_sub_estimator_->setCoordinate(coordinate_);
      gnss_imu_initializer_->setGravity(imu_base_options_.imu_parameters.g);
    }
    if (gnss_imu_initializer_->addMeasurement(measurement)) {
      gnss_imu_initializer_->estimate();
      // Set result to estimator
      setInitializationResult(gnss_imu_initializer_);
    }
    return false;
  }

  // Add IMU
  if (measurement.imu && measurement.imu_role == ImuRole::Major) {
    addImuMeasurement(*measurement.imu);
  }

  // Add GNSS
  if (measurement.gnss) {
    // Feed covariance estimator
    if (!ambiguity_covariance_coordinate_set_ && coordinate_) {
      ambiguity_covariance_estimator_->setCoordinate(coordinate_);
      ambiguity_covariance_coordinate_set_ = true;
    }
    if (coordinate_ && ambiguity_covariance_estimator_->addMeasurement(measurement)) {
      ambiguity_covariance_estimator_->estimate();
    }
    // Align local rover and reference measurements
    GnssMeasurement rov, ref;
    measurement_align_.add(measurement);
    if (measurement_align_.get(rtk_options_.max_age, rov, ref)) {
      return addGnssMeasurementAndState(rov, ref);
    }
  }

  // Add LiDAR
  if (measurement.lidar) {
    if (!lidar_initialized_) return lidarInitialization(measurement.lidar);
    return addLidarMeasurementAndState(measurement.lidar);
  }

  return false;
}

// Add GNSS measurements and state
bool RtkImuLidarRrrEstimator::addGnssMeasurementAndState(const GnssMeasurement& measurement_rov,
                                                         const GnssMeasurement& measurement_ref)
{
  // Get prior states
  Eigen::Vector3d position_prior =
      coordinate_->convert(getPoseEstimate().getPosition(), GeoType::ENU, GeoType::ECEF);

  // Set to local measurement handle
  curGnssRov() = measurement_rov;
  curGnssRov().position = position_prior;
  curGnssRef() = measurement_ref;

  // Erase duplicated phases, arrange to one observation per phase
  gnss_common::rearrangePhasesAndCodes(curGnssRov());
  gnss_common::rearrangePhasesAndCodes(curGnssRef());

  // Form double difference pair
  std::map<char, std::string> system_to_base_prn;
  GnssMeasurementDDIndexPairs phase_index_pairs = gnss_common::formPhaserangeDDPair(
      curGnssRov(), curGnssRef(), system_to_base_prn, gnss_base_options_.common);
  GnssMeasurementDDIndexPairs code_index_pairs = gnss_common::formPseudorangeDDPair(
      curGnssRov(), curGnssRef(), system_to_base_prn, gnss_base_options_.common);
  // Cycle-slip detection
  if (!isFirstEpoch() && gnss_measurement_pairs_.size() >= 2) {
    cycleSlipDetectionSD(lastGnssRov(), lastGnssRef(), curGnssRov(), curGnssRef(),
                         gnss_base_options_.common);
  }
  // Add parameter blocks
  double timestamp = curGnssRov().timestamp;
  // Pose and speed-and-bias blocks
  const int32_t bundle_id = curGnssRov().id;
  BackendId pose_id = createGnssPoseId(bundle_id);
  size_t index = insertImuState(timestamp, pose_id);
  states_[index].status = GnssSolutionStatus::Single;
  latest_state_index_ = index;
  // GNSS extrinsics are added during initialization
  CHECK(gnss_extrinsics_id_.valid());
  // Ambiguity blocks
  addSdAmbiguityParameterBlocks(curGnssRov(), curGnssRef(), phase_index_pairs, curGnssRov().id,
                                curAmbiguityState());
  // Frequency block
  int num_valid_doppler_system = 0;
  addFrequencyParameterBlocks(curGnssRov(), curGnssRov().id, num_valid_doppler_system);
  // Add pseudorange residual blocks
  int num_valid_satellite = 0;
  addDdPseudorangeResidualBlocks(curGnssRov(), curGnssRef(), code_index_pairs, states_[index],
                                 num_valid_satellite);
  // A tightly coupled estimator can retain an epoch with insufficient satellites
  if (!checkSufficientSatellite(num_valid_satellite, 0)) {
    // Retain the state for constraints from other sensors
  }
  num_satellites_ = num_valid_satellite;
  // No satellite
  if (num_satellites_ == 0) {
    // Erase parameters in current state
    eraseFrequencyParameterBlocks(states_[index]);
    eraseImuState(states_[index]);
    eraseAmbiguityParameterBlocks(curAmbiguityState());
    return false;
  }
  // Add phaserange residual blocks
  addDdPhaserangeResidualBlocks(curGnssRov(), curGnssRef(), phase_index_pairs, states_[index]);

  // Add doppler residual blocks
  addDopplerResidualBlocks(curGnssRov(), states_[index], num_valid_satellite, false,
                           getImuMeasurementNear(timestamp).angular_velocity);

  // Add relative errors
  if (lastGnssState().valid()) {  // maybe invalid here because of long term GNSS absent
    // Frequency
    addRelativeFrequencyResidualBlock(lastGnssState(), states_[index]);
    // Ambiguity
    addRelativeAmbiguityResidualBlock(lastGnssRov(), curGnssRov(), lastAmbiguityState(),
                                      curAmbiguityState());
  }
  // ZUPT
  addZUPTResidualBlock(states_[index]);

  // Car motion
  if (imu_base_options_.car_motion) {
    // Heading measurement constraint
    addHMCResidualBlock(states_[index]);
    // Non-holonomic constraint
    addNHCResidualBlock(states_[index]);
  }

  // Compute DOP
  updateGdop(curGnssRov(), code_index_pairs);

  return true;
}

// Add LiDAR measurement and state
bool RtkImuLidarRrrEstimator::addLidarMeasurementAndState(const ScanPtr& scan,
                                                          const SpeedAndBias& speed_and_bias_p,
                                                          const Transformation& T_WS_p)
{
  // An initialized estimator must contain a previous state
  CHECK(!isFirstEpoch());

  // Sort points by curvature (time offset)
  std::sort(scan->cloud_ptr->points.begin(), scan->cloud_ptr->points.end(),
            [](const Point_lidar& a, const Point_lidar& b) { return a.curvature < b.curvature; });

  // Set to local measurement handle
  curScan() = scan;

  // Propagate the state to the scan end time
  Transformation T_WB_prior, T_WB_base;
  SpeedAndBias Speedbias_prior, Speedbias_base;
  getPoseEstimateAt(curScan()->timefinal, T_WB_prior);
  getPoseEstimateAt(curScan()->timebase, T_WB_base);
  getSpeedAndBiasEstimateAt(curScan()->timefinal, Speedbias_prior);
  getSpeedAndBiasEstimateAt(curScan()->timebase, Speedbias_base);

  Transformation T_WB_k, delta_T;
  Transformation T_B_L_inv = lidar_base_options_.T_B_L.inverse();
  Transformation T_WB_prior_inv = T_WB_prior.inverse();

  auto& points = curScan()->cloud_ptr->points;
  double last_timestamp = curScan()->timebase;
  Transformation T_WB_curr = T_WB_base;  // start pose at scan begin
  SpeedAndBias Speedbias_curr = Speedbias_base;

  // Deskew scan points to the scan end time
  for (auto& point : points) {
    // Absolute time of the current point
    const double cur_timestamp = curScan()->timebase + point.curvature;

    // Integrate IMU from last point to current point
    imuIntegration(last_timestamp, cur_timestamp, T_WB_curr, Speedbias_curr);

    // Deskew the point using the relative pose within the scan
    delta_T = T_B_L_inv * T_WB_prior_inv * T_WB_curr * lidar_base_options_.T_B_L;
    tflidarpoint(point, delta_T);

    // Update reference time for the next iteration
    last_timestamp = cur_timestamp;
  }

  // Pose and speed-and-bias blocks
  double timestamp = curScan()->timefinal;
  const int32_t bundle_id = static_cast<int32_t>(curScan()->seq);
  BackendId pose_id = createLidarPoseId(bundle_id);
  size_t index;
  if (speed_and_bias_p != SpeedAndBias::Zero()) {
    index = insertImuState(timestamp, pose_id, T_WS_p, speed_and_bias_p, true);
  } else {
    index = insertImuState(timestamp, pose_id, T_WB_prior, Speedbias_prior, true);
  }
  states_[index].status = latestGnssState().status;
  states_[index].current_scan_l = curScan()->cloud_ptr;
  latest_state_index_ = index;

  // LiDAR extrinsics
  if (!lidar_extrinsics_id_.valid()) {
    lidar_extrinsics_id_ =
        addLidarExtrinsicsParameterBlock(bundle_id, lidar_base_options_.T_B_L, false);
  }

  // Select keyframe
  selectKeyFrame(curScan(), T_WB_prior);

  // Add keyframe parameter and residual blocks
  if (curScan()->is_keyframe) {
    // Add parameter blocks
    states_[latest_state_index_].is_keyframe = true;
    addPlaneParameterBlocksWithResiduals(states_[index], curScan());
  }

  // Accumulate non-keyframe scan
  if (!curScan()->is_keyframe) {
    states_[latest_state_index_].is_keyframe = false;
    addPlaneResidualBlocks(states_[index], curScan());
  }

  return true;
}

// LiDAR initialization
bool RtkImuLidarRrrEstimator::lidarInitialization(const ScanPtr& scan)
{
  // Retain IMU measurements until LiDAR initialization completes
  do_not_remove_imu_measurements_ = true;

  // Check whether the GNSS/IMU estimator has converged
  auto cov = computeAndGetCovariance(lastState());
  double std_yaw = sqrt(cov(5, 5));
  if (std_yaw > rrr_options_.min_yaw_std_init_lidar * D2R) return false;

  Transformation T_WB_prior;
  SpeedAndBias Speedbias_prior;
  getPoseEstimateAt(scan->timefinal, T_WB_prior);
  getSpeedAndBiasEstimateAt(scan->timefinal, Speedbias_prior);

  // Deskew the first scan
  Transformation T_WB_k, delta_T;
  for (size_t i = 0; i < scan->cloud_ptr->size(); i++) {
    // Propagate pose to point time
    ImuEstimatorBase::getPoseEstimateAt(scan->timebase + scan->cloud_ptr->points[i].curvature,
                                        T_WB_k);

    // Deskew point to the scan end time
    delta_T = lidar_base_options_.T_B_L.inverse() * T_WB_prior.inverse() * T_WB_k *
              lidar_base_options_.T_B_L;
    tflidarpoint(scan->cloud_ptr->points[i], delta_T);
  }

  // Pose and speed-and-bias blocks
  double timestamp = scan->timefinal;
  const int32_t bundle_id = static_cast<int32_t>(scan->seq);
  BackendId pose_id = createLidarPoseId(bundle_id);
  size_t index;

  index = insertImuState(timestamp, pose_id, T_WB_prior, Speedbias_prior, true);
  states_[index].status = latestGnssState().status;
  states_[index].is_keyframe = true;
  states_[index].current_scan_l = scan->cloud_ptr;
  latest_state_index_ = index;

  // Initialize global point and voxel maps
  Cloud_ptr cloud_w(new Cloud);
  pcl::transformPointCloud(*scan->cloud_ptr, *cloud_w,
                           transTomat(T_WB_prior * lidar_base_options_.T_B_L), true);
  *local_map_ += *cloud_w;
  tree_handler_->mapBuild(cloud_w);
  tree_handler_->buildVoxelMap(cloud_w);

  lidar_initialized_ = true;
  do_not_remove_imu_measurements_ = false;
  can_compute_covariance_ = true;
  return false;
}

// Solve current graph
bool RtkImuLidarRrrEstimator::estimate()
{
  status_ = EstimatorStatus::Converged;

  optimize();

  State& new_state = states_[latest_state_index_];
  IdType new_state_type = states_[latest_state_index_].id.type();

  if (base_options_.verbose_output) {
    LOG(INFO) << estimatorTypeToString(type_)
              << ": Iterations: " << graph_->summary.iterations.size() << ", " << std::scientific
              << std::setprecision(3) << "Initial cost: " << graph_->summary.initial_cost
              << ", Final cost: " << graph_->summary.final_cost;
  }

  if (new_state_type == IdType::gPose) {
    // Reject GNSS outliers
    size_t n_pseudorange = numPseudorangeError(states_[latest_state_index_]);
    size_t n_phaserange = numPhaserangeError(states_[latest_state_index_]);
    size_t n_doppler = numDopplerError(states_[latest_state_index_]);
    if (gnss_base_options_.use_outlier_rejection) {
      rejectPseudorangeOutlier(states_[latest_state_index_], curAmbiguityState());
      rejectDopplerOutlier(states_[latest_state_index_]);
      rejectPhaserangeOutlier(states_[latest_state_index_], curAmbiguityState());
    }

    // Check if we rejected too many GNSS residuals
    double ratio_pseudorange =
        n_pseudorange == 0.0
            ? 0.0
            : 1.0 - getDivide(numPseudorangeError(states_[latest_state_index_]), n_pseudorange);
    double ratio_phaserange =
        n_phaserange == 0.0
            ? 0.0
            : 1.0 - getDivide(numPhaserangeError(states_[latest_state_index_]), n_phaserange);
    double ratio_doppler =
        n_doppler == 0.0
            ? 0.0
            : 1.0 - getDivide(numDopplerError(states_[latest_state_index_]), n_doppler);
    const double thr = gnss_base_options_.diverge_max_reject_ratio;
    if (isGnssGoodObservation() &&
        (ratio_pseudorange > thr || ratio_phaserange > thr || ratio_doppler > thr)) {
      num_continuous_reject_gnss_++;
    } else
      num_continuous_reject_gnss_ = 0;
    if (num_continuous_reject_gnss_ > gnss_base_options_.diverge_min_num_continuous_reject) {
      LOG(WARNING) << "Estimator diverge: Too many GNSS outliers rejected!";
      num_continuous_reject_gnss_ = 0;
    }

    // Ambiguity resolution
    for (size_t i = latest_state_index_; i < states_.size(); i++) {
      states_[i].status = GnssSolutionStatus::Float;
    }
    if (rtk_options_.use_ambiguity_resolution) {
      // get covariance of ambiguities
      Eigen::MatrixXd ambiguity_covariance;
      if (estimateAmbiguityCovariance(states_[latest_state_index_], ambiguity_covariance)) {
        // solve
        AmbiguityResolution::Result ret = ambiguity_resolution_->solveRtk(
            states_[latest_state_index_].id, curAmbiguityState().ids, ambiguity_covariance,
            gnss_measurement_pairs_.back());
        if (ret == AmbiguityResolution::Result::NlFix) {
          for (size_t i = latest_state_index_; i < states_.size(); i++) {
            states_[i].status = GnssSolutionStatus::Fixed;
          }
        }
      }
    }

    // Check if we continuously cannot fix ambiguity, while we have good observations
    if (rtk_options_.use_ambiguity_resolution) {
      const double thr = gnss_base_options_.good_observation_max_reject_ratio;
      if (isGnssGoodObservation() && ratio_pseudorange < thr && ratio_phaserange < thr &&
          ratio_doppler < thr) {
        if (curState().status != GnssSolutionStatus::Fixed)
          num_continuous_unfix_++;
        else
          num_continuous_unfix_ = 0;
      } else
        num_continuous_unfix_ = 0;
      if (num_continuous_unfix_ > gnss_base_options_.reset_ambiguity_min_num_continuous_unfix) {
        resetAmbiguityEstimation();
        ambiguity_covariance_estimator_->resetAmbiguityEstimation();
        num_continuous_unfix_ = 0;
      }
    }
  }

  // LiDAR processing
  if (new_state_type == IdType::lPose) {
    // Update point cloud map for visualization
    updateCloudMap(new_state);

    // Update landmarks
    updateLandmarks();

    // Reject excessive residuals
    rejectExcessiveResiduals(new_state);

    eraseEmptyLandmarks();

    // Update keys in voxel map
    tree_handler_->updateVoxelKey();

    // Update current scan to voxel map
    tree_handler_->updateVoxelMap(scan_to_map_W_);

    // Voxel map slide to reduce memory
    tree_handler_->mapSlide(getPoseEstimate(new_state).getPosition());
  }

  marginalization(new_state_type);

  // Shift memory for states and measurements
  if (new_state_type == IdType::gPose) {
    gnss_measurement_pairs_.push_back(std::make_pair(GnssMeasurement(), GnssMeasurement()));
    ambiguity_states_.push_back(AmbiguityState());
  }
  if (new_state_type == IdType::lPose) scans_.push_back(nullptr);
  states_.push_back(State());
  while (!lidar_initialized_ && states_.size() > rrr_options_.max_gnss_window_length_minor) {
    states_.pop_front();
    ambiguity_states_.pop_front();
    gnss_measurement_pairs_.pop_front();
  }
  // Keep LiDAR measurements for two epochs
  while (scans_.size() > 5) scans_.pop_front();

  return true;
}

// Set initialization result
void RtkImuLidarRrrEstimator::setInitializationResult(
    const std::shared_ptr<MultisensorInitializerBase>& initializer)
{
  CHECK(initializer->finished());

  // Cast to desired initializer
  std::shared_ptr<GnssImuInitializer> gnss_imu_initializer =
      std::static_pointer_cast<GnssImuInitializer>(initializer);
  CHECK_NOTNULL(gnss_imu_initializer);

  // Arrange to window length
  ImuMeasurements imu_measurements;
  // Placeholder required by the common GNSS/IMU initializer interface
  std::deque<GnssSolution> gnss_measurement_temp;
  gnss_imu_initializer->arrangeToEstimator(
      rrr_options_.max_gnss_window_length_minor, marginalization_error_, states_,
      marginalization_residual_id_, gnss_extrinsics_id_, gnss_measurement_temp, imu_measurements);
  for (auto it = imu_measurements.rbegin(); it != imu_measurements.rend(); it++) {
    imu_mutex_.lock();
    imu_measurements_.push_front(*it);
    imu_mutex_.unlock();
  }

  // Shift memory for states and measurements
  states_.push_back(State());
  gnss_measurement_pairs_.resize(states_.size());
  ambiguity_states_.resize(states_.size());
  scans_.push_back(nullptr);
}

// Marginalization
bool RtkImuLidarRrrEstimator::marginalization(const IdType& type)
{
  if (type == IdType::lPose)
    return lidarMarginalization();
  else if (type == IdType::gPose)
    return gnssMarginalization();
  else
    return false;
}

// Marginalization when the new state is a LiDAR state
bool RtkImuLidarRrrEstimator::lidarMarginalization()
{
  // Check if we need marginalization
  if (isFirstEpoch()) return true;

  // Make sure that only current state can be non-keyframe
  for (size_t i = 0; i + 1 < states_.size();) {
    if (states_[i].id.type() != IdType::lPose || states_[i].is_keyframe) {
      ++i;
      continue;
    }
    erasePlaneErrorResidualBlocks(states_[i]);
    eraseRegistrationErrorResidualBlocks(states_[i]);
    eraseImuState(states_[i]);
  }

  // If current frame is a keyframe. Marginalize the oldest keyframe and corresponding
  // IMU and GNSS states out. And sparsify GNSS states.
  if (sizeOfLidarkeyframeStates() > rrr_options_.max_keyframes) {
    // Erase old marginalization item
    if (!eraseOldMarginalization()) return false;

    // Add marginalization items
    // marginalize oldest keyframe state
    bool reached_first_keyframe = false;
    State margin_keyframe_state;
    for (auto it = states_.begin(); it != states_.end();) {
      State& state = *it;
      // Reached the first keyframe
      if (state.is_keyframe) reached_first_keyframe = true;

      // Mark marginalized keyframe state
      if (state.is_keyframe) margin_keyframe_state = state;

      // Add marginalization blocks and LiDAR residuals
      if (state.is_keyframe) {
        eraseRegistrationErrorResidualBlocks(state);
        erasePlaneErrorResidualBlocks(state);

        addImuStateMarginBlock(state);
        addImuResidualMarginBlocks(state);
      }
      // GNSS state not yet marginalized by a GNSS step
      else {
        auto it_ambiguity = ambiguityStateAt(state.timestamp);
        if (it_ambiguity != ambiguity_states_.end()) {
          addAmbiguityMarginBlocksWithResiduals(*it_ambiguity);
        } else {
          addGnssLooseResidualMarginBlocks(state);
        }
        addImuStateMarginBlock(state);
        addImuResidualMarginBlocks(state);
        addFrequencyMarginBlocksWithResiduals(state);
        addGnssResidualMarginBlocks(state);

        // Erase ambiguity state and GNSS measurements
        if (it_ambiguity != ambiguity_states_.end()) {
          ambiguity_states_.erase(it_ambiguity);
        }
        if (gnssMeasurementPairAt(state.timestamp) != gnss_measurement_pairs_.end()) {
          gnss_measurement_pairs_.erase(gnssMeasurementPairAt(state.timestamp));
        }
      }

      // Erase state
      it = states_.erase(it);

      if (reached_first_keyframe) break;
    }

    // Landmarks
    eraseEmptyLandmarks();
    addLandmarkParameterMarginBlocksWithResiduals(margin_keyframe_state);

    // Apply marginalization and add the item into graph
    bool ret = applyMarginalization();

    return ret;
  }

  return true;
}

// Marginalization when the new state is a GNSS state
bool RtkImuLidarRrrEstimator::gnssMarginalization()
{
  // Check if we need marginalization
  if (isFirstEpoch()) return true;

  // Before LiDAR initialization, only handle GNSS and INS
  if (!lidar_initialized_) {
    // Check if we need marginalization
    if (states_.size() < rrr_options_.max_gnss_window_length_minor) {
      return true;
    }

    // Erase old marginalization item
    if (!eraseOldMarginalization()) return false;

    // Add marginalization items
    // IMU states and residuals
    addImuStateMarginBlockWithResiduals(oldestState());
    // Ambiguity
    addAmbiguityMarginBlocksWithResiduals(oldestAmbiguityState());
    // Frequency
    addFrequencyMarginBlocksWithResiduals(oldestState());

    // Apply marginalization and add the item into graph
    bool ret = applyMarginalization();

    return ret;
  }
  // After LiDAR initialization, handle all sensors
  else {
    // Sparsify GNSS states
    sparsifyGnssStates();

    // Marginalize the GNSS states that in front of the oldest keyframe
    // Perform this in both GNSS and LiDAR steps to smooth the computational load
    for (auto it = states_.begin(); it != states_.end();) {
      State& state = *it;
      // Reached the oldest keyframe
      if (state.is_keyframe) break;
      // Check for a GNSS state
      if (state.id.type() != IdType::gPose) {
        ++it;
        continue;
      }

      // Erase old marginalization item
      if (!eraseOldMarginalization()) return false;

      // Add marginalization blocks
      auto it_ambiguity = ambiguityStateAt(state.timestamp);
      if (it_ambiguity != ambiguity_states_.end()) {
        addAmbiguityMarginBlocksWithResiduals(*it_ambiguity);
      } else {
        addGnssLooseResidualMarginBlocks(state);
      }
      addImuStateMarginBlock(state);
      addImuResidualMarginBlocks(state);
      addFrequencyMarginBlocksWithResiduals(state);
      addGnssResidualMarginBlocks(state);

      // Erase ambiguity state and GNSS measurements
      if (it_ambiguity != ambiguity_states_.end()) {
        ambiguity_states_.erase(it_ambiguity);
      }
      if (gnssMeasurementPairAt(state.timestamp) != gnss_measurement_pairs_.end()) {
        gnss_measurement_pairs_.erase(gnssMeasurementPairAt(state.timestamp));
      }

      // Erase state
      it = states_.erase(it);

      // Handle one state per iteration
      bool ret = applyMarginalization();

      return ret;
    }
  }

  return true;
}

// Sparsify GNSS states to bound computational load
void RtkImuLidarRrrEstimator::sparsifyGnssStates()
{
  // Check if we need to sparsify
  std::vector<BackendId> gnss_ids;
  std::vector<int> num_neighbors;
  int max_num_neighbors = 0;
  for (size_t i = 0; i < states_.size(); i++) {
    if (states_[i].id.type() != IdType::gPose) continue;
    gnss_ids.push_back(states_[i].id);
    // find neighbors
    num_neighbors.push_back(0);
    const size_t idx = num_neighbors.size() - 1;
    for (size_t j = i; j > 0;) {
      --j;
      if (states_[j].id.type() != IdType::gPose) break;
      num_neighbors[idx]++;
    }
    for (size_t j = i; j < states_.size(); j++) {
      if (states_[j].id.type() != IdType::gPose) break;
      num_neighbors[idx]++;
    }
    if (max_num_neighbors < num_neighbors[idx]) {
      max_num_neighbors = num_neighbors[idx];
    }
  }
  if (gnss_ids.size() <= rrr_options_.max_keyframes) return;

  // Erase some GNSS states
  // The states with most neighbors will be erased first
  const size_t num_to_erase = gnss_ids.size() - rrr_options_.max_keyframes;
  std::vector<BackendId> ids_to_erase;
  for (int m = max_num_neighbors; m >= 0; m--) {
    for (size_t i = 0; i < num_neighbors.size(); i++) {
      if (num_neighbors[i] != m) continue;
      if (i == 0) continue;  // in case the first one connects to margin block
      ids_to_erase.push_back(gnss_ids[i]);
      if (ids_to_erase.size() >= num_to_erase) break;
    }
    if (ids_to_erase.size() >= num_to_erase) break;
  }
  CHECK(ids_to_erase.size() >= num_to_erase);
  for (size_t i = 0; i < states_.size();) {
    if (std::find(ids_to_erase.begin(), ids_to_erase.end(), states_[i].id) == ids_to_erase.end()) {
      ++i;
      continue;
    }
    State& state = states_[i];
    const double timestamp = state.timestamp;

    auto it_ambiguity = ambiguityStateAt(timestamp);
    // the first one may be connected with margin error
    if (it_ambiguity == ambiguity_states_.begin()) {
      ++i;
      continue;
    }

    eraseGnssMeasurementResidualBlocks(state);
    eraseFrequencyParameterBlocks(state);

    // we may failed to find corresponding ambiguity state because the GNSS state can be
    // loosely coupled state during initialization.
    if (it_ambiguity != ambiguity_states_.end()) {
      eraseAmbiguityParameterBlocks(*it_ambiguity);
      ambiguity_states_.erase(it_ambiguity);
    } else {
      eraseGnssLooseResidualBlocks(state);
    }

    auto it_measurement = gnssMeasurementPairAt(timestamp);
    eraseImuState(state);

    if (it_measurement != gnss_measurement_pairs_.end()) {
      gnss_measurement_pairs_.erase(it_measurement);
    }

  }
}

// Compute ambiguity covariance at current epoch
bool RtkImuLidarRrrEstimator::estimateAmbiguityCovariance(const State& state,
                                                          Eigen::MatrixXd& covariance)
{
  // Estimate a coarse ambiguity covariance with a parallel optimizer and empirical constraints
  std::shared_ptr<Graph> sub_graph = ambiguity_covariance_estimator_->getGraph();
  const State sub_state = ambiguity_covariance_estimator_->getState();

  // Compute covariance
  std::vector<uint64_t> parameter_block_ids;
  // curAmbiguityState() and ambiguity_covariance_estimator_ use the same ambiguity IDs
  for (auto id : curAmbiguityState().ids) {
    if (!sub_graph->parameterBlockExists(id.asInteger())) return false;
    parameter_block_ids.push_back(id.asInteger());
  }
  sub_graph->computeCovariance(parameter_block_ids, covariance);

  return true;
}

};
