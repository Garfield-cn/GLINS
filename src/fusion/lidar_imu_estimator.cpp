/**
 * @Function: LiDAR/IMU odometry estimator
 *
 * @Author  : Jiahui Liu
 * @Email   : jh.liu@sjtu.edu.cn
 **/
#include "gici/fusion/lidar_imu_estimator.h"

#include <iomanip>

#include "gici/estimate/pose_error.h"

namespace gici {

LidarImuEstimator::LidarImuEstimator(const LidarImuEstimatorOptions& options,
                                     const LidarEstimatorBaseOptions& lidar_base_options,
                                     const ImuEstimatorBaseOptions& imu_base_options,
                                     const EstimatorBaseOptions& base_options)
    : EstimatorBase(base_options),
      ImuEstimatorBase(imu_base_options, base_options),
      LidarEstimatorBase(lidar_base_options, base_options),
      lidar_imu_options_(options)
{
  type_ = EstimatorType::LidarImu;
  imu_base_options_.imu_parameters.delay_imu_cam = lidar_imu_options_.time_offset;
  states_.push_back(State());
  scans_.push_back(nullptr);

  // Preserve the initialization interval while no valid state exists.
  do_not_remove_imu_measurements_ = true;
}

LidarImuEstimator::~LidarImuEstimator()
{}

bool LidarImuEstimator::addMeasurement(const EstimatorDataCluster& measurement)
{
  if (coordinate_ == nullptr || !gravity_setted_) return false;

  if (measurement.imu && measurement.imu_role == ImuRole::Major) {
    addImuMeasurement(*measurement.imu);
  }

  if (measurement.lidar) {
    if (!initialized_) return initialize(measurement.lidar);
    return addLidarMeasurementAndState(measurement.lidar);
  }

  return false;
}

bool LidarImuEstimator::initialize(const ScanPtr& scan)
{
  CHECK_NOTNULL(tree_handler_.get());
  if (scan == nullptr || scan->cloud_ptr == nullptr || scan->cloud_ptr->empty()) return false;

  std::sort(scan->cloud_ptr->points.begin(), scan->cloud_ptr->points.end(),
            [](const Point_lidar& lhs, const Point_lidar& rhs) {
              return lhs.curvature < rhs.curvature;
            });

  const double imu_timebase = scan->timebase - lidar_imu_options_.time_offset;
  const double imu_timefinal = scan->timefinal - lidar_imu_options_.time_offset;
  const double initialization_start =
      imu_timebase - lidar_imu_options_.initialization_duration;
  Eigen::Vector3d mean_acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d mean_angular_velocity = Eigen::Vector3d::Zero();
  std::vector<double> accelerations[3];
  std::vector<double> angular_velocities[3];
  size_t num_measurements = 0;

  imu_mutex_.lock();
  if (imu_measurements_.empty() || imu_measurements_.front().timestamp > initialization_start ||
      imu_measurements_.back().timestamp < imu_timefinal) {
    imu_mutex_.unlock();
    return false;
  }
  for (const ImuMeasurement& imu : imu_measurements_) {
    if (imu.timestamp < initialization_start || imu.timestamp > imu_timebase) continue;
    if (std::abs(imu.linear_acceleration.norm() - imu_base_options_.imu_parameters.g) > 0.5) {
      imu_mutex_.unlock();
      return false;
    }
    mean_acceleration += imu.linear_acceleration;
    mean_angular_velocity += imu.angular_velocity;
    for (size_t axis = 0; axis < 3; ++axis) {
      accelerations[axis].push_back(imu.linear_acceleration[axis]);
      angular_velocities[axis].push_back(imu.angular_velocity[axis]);
    }
    ++num_measurements;
  }
  imu_mutex_.unlock();

  if (num_measurements == 0 || mean_acceleration.norm() < 1.0e-6) return false;
  for (size_t axis = 0; axis < 3; ++axis) {
    const double median_acceleration = getMedian(accelerations[axis]);
    const double median_angular_velocity = getMedian(angular_velocities[axis]);
    if (getStandardDeviation(accelerations[axis], median_acceleration) >
            imu_base_options_.zupt_max_acc_std ||
        getStandardDeviation(angular_velocities[axis], median_angular_velocity) >
            imu_base_options_.zupt_max_gyro_std ||
        std::abs(median_angular_velocity) > imu_base_options_.zupt_max_gyro_median) {
      return false;
    }
  }
  mean_acceleration /= static_cast<double>(num_measurements);
  mean_angular_velocity /= static_cast<double>(num_measurements);
  const Eigen::Vector3d acceleration_direction = mean_acceleration.normalized();

  // Gravity fixes roll and pitch. Position and yaw define the local LIO frame.
  const Eigen::Quaterniond q_W_B =
      Eigen::Quaterniond::FromTwoVectors(acceleration_direction, Eigen::Vector3d::UnitZ());
  Transformation T_WB_base(Eigen::Vector3d::Zero(), q_W_B.normalized());
  SpeedAndBias speed_and_bias_base = SpeedAndBias::Zero();
  speed_and_bias_base.segment<3>(3) = mean_angular_velocity;
  const Eigen::Vector3d gravity_W(0.0, 0.0, imu_base_options_.imu_parameters.g);
  speed_and_bias_base.tail<3>() =
      mean_acceleration - T_WB_base.getRotationMatrix().transpose() * gravity_W;
  Transformation T_WB_end = T_WB_base;
  SpeedAndBias speed_and_bias_end = speed_and_bias_base;
  if (!imuIntegration(scan->timebase, scan->timefinal, T_WB_end, speed_and_bias_end)) return false;

  // Deskew the first scan from its acquisition time to the scan end time.
  Transformation T_WB_point = T_WB_base;
  SpeedAndBias speed_and_bias_point = speed_and_bias_base;
  const Transformation T_B_L_inv = lidar_base_options_.T_B_L.inverse();
  const Transformation T_WB_end_inv = T_WB_end.inverse();
  double last_timestamp = scan->timebase;
  for (Point_lidar& point : scan->cloud_ptr->points) {
    const double point_timestamp = scan->timebase + point.curvature;
    imuIntegration(last_timestamp, point_timestamp, T_WB_point, speed_and_bias_point);
    const Transformation delta_T =
        T_B_L_inv * T_WB_end_inv * T_WB_point * lidar_base_options_.T_B_L;
    tflidarpoint(point, delta_T);
    last_timestamp = point_timestamp;
  }

  const int32_t bundle_id = static_cast<int32_t>(scan->seq);
  const BackendId pose_id = createLidarPoseId(bundle_id);
  const size_t index =
      insertImuState(scan->timefinal, pose_id, T_WB_end, speed_and_bias_end, true);
  states_[index].is_keyframe = true;
  states_[index].current_scan_l = scan->cloud_ptr;
  latest_state_index_ = index;
  selectKeyFrame(scan, T_WB_end);

  // Anchor the otherwise unobservable local position and yaw, and initialize velocity and biases.
  const double pose_variance = square(lidar_imu_options_.initial_pose_std);
  std::shared_ptr<PoseError> pose_prior =
      std::make_shared<PoseError>(T_WB_end, pose_variance, pose_variance);
  graph_->addResidualBlock(pose_prior, nullptr,
                           graph_->parameterBlockPtr(states_[index].id_in_graph.asInteger()));
  addImuSpeedAndBiasResidualBlock(states_[index], speed_and_bias_end,
                                  lidar_imu_options_.initial_speed_std,
                                  lidar_imu_options_.initial_bg_std,
                                  lidar_imu_options_.initial_ba_std);

  Cloud_ptr cloud_w(new Cloud);
  pcl::transformPointCloud(*scan->cloud_ptr, *cloud_w,
                           transTomat(T_WB_end * lidar_base_options_.T_B_L), true);
  *local_map_ = *cloud_w;
  tree_handler_->mapBuild(cloud_w);
  tree_handler_->buildVoxelMap(cloud_w);

  initialized_ = true;
  do_not_remove_imu_measurements_ = false;
  can_compute_covariance_ = true;
  status_ = EstimatorStatus::Converged;
  LOG(INFO) << "LiDAR/IMU odometry initialized at " << std::fixed << scan->timefinal;
  return false;
}

bool LidarImuEstimator::addLidarMeasurementAndState(const ScanPtr& scan)
{
  CHECK_NOTNULL(tree_handler_.get());
  if (scan == nullptr || scan->cloud_ptr == nullptr || scan->cloud_ptr->empty()) return false;

  std::sort(scan->cloud_ptr->points.begin(), scan->cloud_ptr->points.end(),
            [](const Point_lidar& lhs, const Point_lidar& rhs) {
              return lhs.curvature < rhs.curvature;
            });
  curScan() = scan;

  Transformation T_WB_end, T_WB_point;
  SpeedAndBias speed_and_bias_end, speed_and_bias_point;
  if (!getPoseEstimateAt(scan->timefinal, T_WB_end) ||
      !getPoseEstimateAt(scan->timebase, T_WB_point) ||
      !getSpeedAndBiasEstimateAt(scan->timefinal, speed_and_bias_end) ||
      !getSpeedAndBiasEstimateAt(scan->timebase, speed_and_bias_point)) {
    return false;
  }

  const Transformation T_B_L_inv = lidar_base_options_.T_B_L.inverse();
  const Transformation T_WB_end_inv = T_WB_end.inverse();
  double last_timestamp = scan->timebase;
  for (Point_lidar& point : scan->cloud_ptr->points) {
    const double point_timestamp = scan->timebase + point.curvature;
    imuIntegration(last_timestamp, point_timestamp, T_WB_point, speed_and_bias_point);
    const Transformation delta_T =
        T_B_L_inv * T_WB_end_inv * T_WB_point * lidar_base_options_.T_B_L;
    tflidarpoint(point, delta_T);
    last_timestamp = point_timestamp;
  }

  const int32_t bundle_id = static_cast<int32_t>(scan->seq);
  const BackendId pose_id = createLidarPoseId(bundle_id);
  const size_t index =
      insertImuState(scan->timefinal, pose_id, T_WB_end, speed_and_bias_end, true);
  states_[index].current_scan_l = scan->cloud_ptr;
  latest_state_index_ = index;

  selectKeyFrame(scan, T_WB_end);
  states_[index].is_keyframe = scan->is_keyframe;

  // LIO uses only scan-to-map registration; no landmark is inserted into the graph.
  registration_residual_count_ = addRegistrationErrorResidualBlocks(states_[index], scan);
  if (imu_base_options_.use_zupt) addZUPTResidualBlock(states_[index]);
  if (imu_base_options_.car_motion) {
    addHMCResidualBlock(states_[index]);
    addNHCResidualBlock(states_[index]);
  }

  return true;
}

bool LidarImuEstimator::estimate()
{
  optimize();

  State& new_state = states_[latest_state_index_];
  if (base_options_.verbose_output) {
    LOG(INFO) << estimatorTypeToString(type_)
              << ": Iterations: " << graph_->summary.iterations.size() << ", " << std::scientific
              << std::setprecision(3) << "Initial cost: " << graph_->summary.initial_cost
              << ", Final cost: " << graph_->summary.final_cost << ", Registration residuals: "
              << registration_residual_count_;
  }

  const bool registration_valid =
      registration_residual_count_ >= lidar_imu_options_.min_registration_residuals;
  if (registration_valid) {
    updateCloudMap(new_state);
    tree_handler_->updateVoxelMap(scan_to_map_W_);
    tree_handler_->mapSlide(getPoseEstimate(new_state).getPosition());
    if (registration_degraded_) {
      LOG(INFO) << "LiDAR registration recovered with " << registration_residual_count_
                << " residuals.";
      registration_degraded_ = false;
    }
  } else if (!registration_degraded_) {
    LOG(WARNING) << "LiDAR registration degraded to " << registration_residual_count_
                 << " residuals; skipping map insertion.";
    registration_degraded_ = true;
  }

  if (can_compute_covariance_ && new_state.is_keyframe) updateCovariance(new_state);

  if (!marginalization()) return false;
  latest_state_index_ = states_.size() - 1;

  scans_.push_back(nullptr);
  states_.push_back(State());
  while (scans_.size() > 5) scans_.pop_front();

  return true;
}

bool LidarImuEstimator::marginalization()
{
  // Only the newest state may remain a non-keyframe in the optimization window.
  for (size_t i = 0; i + 1 < states_.size();) {
    if (states_[i].is_keyframe) {
      ++i;
      continue;
    }
    eraseRegistrationErrorResidualBlocks(states_[i]);
    eraseImuState(states_[i]);
  }

  if (sizeOfLidarkeyframeStates() <= lidar_imu_options_.max_window_length) return true;
  if (!eraseOldMarginalization()) return false;

  bool passed_first_keyframe = false;
  for (auto it = states_.begin(); it != states_.end();) {
    State& state = *it;
    if (passed_first_keyframe && state.is_keyframe) break;
    if (state.is_keyframe) passed_first_keyframe = true;

    eraseRegistrationErrorResidualBlocks(state);
    addImuStateMarginBlock(state);
    addImuResidualMarginBlocks(state);
    it = states_.erase(it);
  }

  return applyMarginalization();
}

}  // namespace gici
