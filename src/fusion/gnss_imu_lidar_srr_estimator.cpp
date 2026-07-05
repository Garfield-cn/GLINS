/**
 * @Function: GNSS/IMU/LiDAR semi-tightly coupled estimator
 *
 * @Author  : Jiahui Liu
 * @Email   : jh.liu@sjtu.edu.cn
 **/
#include "gici/fusion/gnss_imu_lidar_srr_estimator.h"

#include <iomanip>

namespace gici {

// The default constructor
GnssImuLidarSrrEstimator::GnssImuLidarSrrEstimator(
    const GnssImuLidarSrrEstimatorOptions& options, const GnssImuInitializerOptions& init_options,
    const LidarEstimatorBaseOptions& lidar_base_options,
    const GnssLooseEstimatorBaseOptions& gnss_loose_base_options,
    const ImuEstimatorBaseOptions& imu_base_options, const EstimatorBaseOptions& base_options)
    : EstimatorBase(base_options),
      GnssLooseEstimatorBase(gnss_loose_base_options, base_options),
      ImuEstimatorBase(imu_base_options, base_options),
      LidarEstimatorBase(lidar_base_options, base_options),
      lidar_srr_options_(options)
{
  type_ = EstimatorType::GnssImuLidarSrr;
  states_.push_back(State());
  gnss_solution_measurements_.push_back(GnssSolution());
  scans_.push_back(nullptr);

  // Initialization control
  gnss_imu_initializer_.reset(new GnssImuInitializer(init_options, gnss_loose_base_options,
                                                     imu_base_options, base_options, graph_));
}

// The default destructor
GnssImuLidarSrrEstimator::~GnssImuLidarSrrEstimator()
{}

// Add measurement
bool GnssImuLidarSrrEstimator::addMeasurement(const EstimatorDataCluster& measurement)
{
  // GNSS/IMU initialization
  if (coordinate_ == nullptr || !gravity_setted_) return false;
  if (!gnss_imu_initializer_->finished()) {
    if (gnss_imu_initializer_->getCoordinate() == nullptr) {
      gnss_imu_initializer_->setCoordinate(coordinate_);
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

  // Add GNSS by solution measurement
  if (measurement.solution && (measurement.solution_role == SolutionRole::Position ||
                               measurement.solution_role == SolutionRole::Velocity ||
                               measurement.solution_role == SolutionRole::PositionAndVelocity)) {
    GnssSolution gnss_solution =
        convertSolutionToGnssSolution(*measurement.solution, measurement.solution_role);
    return addGnssSolutionMeasurementAndState(gnss_solution);
  }

  // Add LiDAR
  if (measurement.lidar) {
    if (!lidar_initialized_) return lidarInitialization(measurement.lidar);

    return addLidarMeasurementAndState(measurement.lidar);
  }

  return false;
}

// Add GNSS solution measurement and state
bool GnssImuLidarSrrEstimator::addGnssSolutionMeasurementAndState(const GnssSolution& measurement)
{
  if (lidar_initialized_ && lidar_srr_options_.lio_mode) return false;

  if (lidar_initialized_) {
    if (rejectGnssSolution(measurement)) {
      return false;
    }
  }

  // Set to local measurement handle
  curGnssSolution() = measurement;

  // Add parameter blocks
  double timestamp = curGnssSolution().timestamp;
  // Pose and speed-and-bias blocks
  const int32_t bundle_id = curGnssSolution().id;
  BackendId pose_id = createGnssPoseId(bundle_id);
  size_t index = insertImuState(timestamp, pose_id);
  states_[index].status = curGnssSolution().status;
  states_[index].is_keyframe = false;
  latest_state_index_ = index;

  // GNSS extrinsics are added during initialization
  CHECK(gnss_extrinsics_id_.valid());
  // Add residual blocks
  // GNSS position
  addGnssPositionResidualBlock(curGnssSolution(), states_[index]);

  // GNSS velocity
  addGnssVelocityResidualBlock(curGnssSolution(), states_[index],
                               getImuMeasurementNear(timestamp).angular_velocity);

  // ZUPT
  addZUPTResidualBlock(states_[index]);
  // Car motion
  if (imu_base_options_.car_motion) {
    // Heading measurement constraint
    addHMCResidualBlock(states_[index]);
    // Non-holonomic constraint
    addNHCResidualBlock(states_[index]);
  }

  return true;
}

// Add LiDAR measurement and state
bool GnssImuLidarSrrEstimator::addLidarMeasurementAndState(const ScanPtr& scan,
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
    if (!gnss_denied_ && !gnssJustRecovered(curScan()) && !lidar_srr_options_.lio_mode) {
      addPlaneParameterBlocksWithResiduals(states_[index], curScan());
    }
  }

  // Accumulate non-keyframe scan
  if (!curScan()->is_keyframe) {
    states_[latest_state_index_].is_keyframe = false;
    if (!gnss_denied_ && !gnssJustRecovered(curScan()) && !lidar_srr_options_.lio_mode) {
      addPlaneResidualBlocks(states_[index], curScan());
    }
  }

  if (gnss_denied_ || lidar_srr_options_.lio_mode) {
    addRegistrationErrorResidualBlocks(states_[index], curScan());
  }
  addZUPTResidualBlock(states_[index]);
  return true;
}

// LiDAR initialization
bool GnssImuLidarSrrEstimator::lidarInitialization(const ScanPtr& scan)
{
  // Retain IMU measurements until LiDAR initialization completes
  do_not_remove_imu_measurements_ = true;

  // Check whether the GNSS/IMU estimator has converged
  auto cov = computeAndGetCovariance(lastState());
  double std_yaw = sqrt(cov(5, 5));
  if (std_yaw > lidar_srr_options_.min_yaw_std_init_lidar * D2R) return false;

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
bool GnssImuLidarSrrEstimator::estimate()
{
  optimize();

  State& new_state = states_[latest_state_index_];
  IdType new_state_type = states_[latest_state_index_].id.type();

  if (base_options_.verbose_output) {
    LOG(INFO) << estimatorTypeToString(type_)
              << ": Iterations: " << graph_->summary.iterations.size() << ", " << std::scientific
              << std::setprecision(3) << "Initial cost: " << graph_->summary.initial_cost
              << ", Final cost: " << graph_->summary.final_cost;
  }

  if (new_state_type == IdType::lPose) {
    if (gnss_denied_ && new_state.timestamp - last_set_map_time_ > 2) {
      tree_handler_->buildVoxelMap(scan_to_map_W_);
      last_set_map_time_ = new_state.timestamp;
    }

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

  // Update covariance
  if (can_compute_covariance_) {
    if (new_state_type == IdType::gPose ||
        (new_state_type == IdType::lPose && new_state.is_keyframe)) {
      updateCovariance(latestState());
    }
  }

  if (new_state_type == IdType::gPose) {
    gnss_solution_measurements_.push_back(GnssSolution());
  }

  if (new_state_type == IdType::lPose) {
    scans_.push_back(nullptr);
  }

  // Shift memory for states and measurements
  states_.push_back(State());

  while (!lidar_initialized_ && states_.size() > lidar_srr_options_.max_gnss_window_length_minor) {
    states_.pop_front();
  }
  // Retain recent measurements for asynchronous GNSS/LiDAR alignment
  while (gnss_solution_measurements_.size() > 4) gnss_solution_measurements_.pop_front();
  while (scans_.size() > 5) scans_.pop_front();

  return true;
}

// Set initialization result
void GnssImuLidarSrrEstimator::setInitializationResult(
    const std::shared_ptr<MultisensorInitializerBase>& initializer)
{
  CHECK(initializer->finished());

  // Cast to desired initializer
  std::shared_ptr<GnssImuInitializer> gnss_imu_initializer =
      std::static_pointer_cast<GnssImuInitializer>(initializer);
  CHECK_NOTNULL(gnss_imu_initializer);

  // Arrange to window length
  ImuMeasurements imu_measurements;
  gnss_imu_initializer->arrangeToEstimator(lidar_srr_options_.max_gnss_window_length_minor,
                                           marginalization_error_, states_,
                                           marginalization_residual_id_, gnss_extrinsics_id_,
                                           gnss_solution_measurements_, imu_measurements);
  for (auto it = imu_measurements.rbegin(); it != imu_measurements.rend(); it++) {
    imu_mutex_.lock();
    imu_measurements_.push_front(*it);
    imu_mutex_.unlock();
  }

  // Shift memory for states and measurements
  states_.push_back(State());
  gnss_solution_measurements_.push_back(GnssSolution());
}

// Marginalization
bool GnssImuLidarSrrEstimator::marginalization(const IdType& type)
{
  if (type == IdType::lPose)
    return lidarMarginalization();
  else if (type == IdType::gPose)
    return gnssMarginalization();
  else
    return false;
}

// Marginalization when the new state is a LiDAR state
bool GnssImuLidarSrrEstimator::lidarMarginalization()
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
  if (sizeOfLidarkeyframeStates() > lidar_srr_options_.max_window_length) {
    // Erase old marginalization item
    if (!eraseOldMarginalization()) return false;

    // Add marginalization items
    // marginalize oldest keyframe state, which actually, all the states before the
    // second keyframe state
    bool passed_first_keyframe = false;
    State margin_keyframe_state;
    for (auto it = states_.begin(); it != states_.end();) {
      State& state = *it;
      // Reached the second keyframe
      if (passed_first_keyframe && state.is_keyframe) break;
      // Passed the first keyframe
      if (state.is_keyframe) passed_first_keyframe = true;

      // Mark marginalized keyframe state
      if (state.is_keyframe) margin_keyframe_state = state;

      // Add marginalization blocks
      if (state.is_keyframe) {
        // Remove LiDAR residuals before marginalizing the keyframe state
        eraseRegistrationErrorResidualBlocks(state);
        erasePlaneErrorResidualBlocks(state);

        // Add IMU state and residuals to marginalization
        addImuStateMarginBlock(state);
        addImuResidualMarginBlocks(state);
      }
      // GNSS states
      else {
        CHECK(state.id.type() == IdType::gPose) << (int)state.id.asInteger();
        addImuStateMarginBlock(state);
        addImuResidualMarginBlocks(state);
        addGnssLooseResidualMarginBlocks(state);
      }

      // Erase state
      it = states_.erase(it);
    }

    // Add plane landmarks to marginalization
    eraseEmptyLandmarks();
    addLandmarkParameterMarginBlocksWithResiduals(margin_keyframe_state);

    // Apply marginalization and add the item into graph
    bool ret = applyMarginalization();

    return ret;
  }
  return true;
}

// Marginalization when the new state is a GNSS state
bool GnssImuLidarSrrEstimator::gnssMarginalization()
{
  // Check if we need marginalization
  if (isFirstEpoch()) return true;

  // LiDAR not initialized, only handle GNSS and INS
  if (!lidar_initialized_) {
    // check if we need marginalization
    if (states_.size() < lidar_srr_options_.max_gnss_window_length_minor) {
      return true;
    }

    // Erase old marginalization item
    if (!eraseOldMarginalization()) return false;

    // Add marginalization items
    // IMU states and residuals
    addImuStateMarginBlockWithResiduals(oldestState());

    // Apply marginalization and add the item into graph
    bool ret = applyMarginalization();

    return ret;
  }
  // LiDAR initialized, handle all the sensors
  else {
    sparsifyGnssStates();
  }

  return true;
}

// Sparsify GNSS states to bound computational load
void GnssImuLidarSrrEstimator::sparsifyGnssStates()
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
  if (gnss_ids.size() <= lidar_srr_options_.max_window_length) return;

  // Erase some GNSS states
  // The states with most neighbors will be erased first
  const size_t num_to_erase = gnss_ids.size() - lidar_srr_options_.max_window_length;
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
    eraseGnssLooseResidualBlocks(states_[i]);
    eraseImuState(states_[i]);
  }
}

// Reject GNSS solution
bool GnssImuLidarSrrEstimator::rejectGnssSolution(const GnssSolution& measurement)
{
  static double last_exceeded_time = 0.0;
  static double last_normal_time = 0.0;
  double std_threshold = lidar_srr_options_.reject_solution_std;
  double deny_threshold = 0.2;
  double recover_threshold = 2;

  double std_position = sqrt(measurement.covariance(0, 0));

  if (std_position > std_threshold) {
    if (last_exceeded_time == 0.0) {
      last_exceeded_time = measurement.timestamp;
    }

    if (measurement.timestamp - last_exceeded_time > deny_threshold) {
      gnss_denied_ = true;
    }

    last_normal_time = 0.0;
  } else {
    if (last_normal_time == 0.0) {
      last_normal_time = measurement.timestamp;
    }

    if (measurement.timestamp - last_normal_time > recover_threshold) {
      if (gnss_denied_) {
        gnss_recover_timestamp_ = measurement.timestamp;
      }
      gnss_denied_ = false;
    }
    last_exceeded_time = 0.0;
  }
  return gnss_denied_;
}

bool GnssImuLidarSrrEstimator::gnssJustRecovered(const ScanPtr& scan)
{
  if (scan->timefinal > gnss_recover_timestamp_ &&
      scan->timefinal < gnss_recover_timestamp_ + lidar_srr_options_.solution_recover_time) {
    return true;
  } else
    return false;
}

};
