/**
 * @Function: LiDAR/IMU odometry estimator
 *
 * @Author  : Jiahui Liu
 * @Email   : jh.liu@sjtu.edu.cn
 **/
#pragma once

#include "gici/imu/imu_estimator_base.h"
#include "gici/lidar/lidar_estimator_base.h"

namespace gici {

// LiDAR/IMU odometry estimator options
struct LidarImuEstimatorOptions {
  // Maximum number of LiDAR keyframes in the optimization window
  size_t max_window_length = 10;

  // Static IMU interval used to initialize roll and pitch, in seconds
  double initialization_duration = 1.0;

  // LiDAR timestamp minus IMU timestamp, in seconds
  double time_offset = 0.0;

  // Minimum scan-to-map residual count required before inserting a scan into the map
  size_t min_registration_residuals = 30;

  // Initial pose prior standard deviation, in meters and radians
  double initial_pose_std = 1.0e-4;

  // Initial velocity prior standard deviation, in meters per second
  double initial_speed_std = 0.1;

  // Initial gyroscope bias prior standard deviation, in radians per second
  double initial_bg_std = 0.01;

  // Initial accelerometer bias prior standard deviation, in meters per square second
  double initial_ba_std = 0.1;
};

// LiDAR/IMU odometry estimator using IMU pre-integration and scan-to-map registration
class LidarImuEstimator : public ImuEstimatorBase, public LidarEstimatorBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LidarImuEstimator(const LidarImuEstimatorOptions& options,
                    const LidarEstimatorBaseOptions& lidar_base_options,
                    const ImuEstimatorBaseOptions& imu_base_options,
                    const EstimatorBaseOptions& base_options);
  ~LidarImuEstimator();

  // Add an IMU or LiDAR measurement
  bool addMeasurement(const EstimatorDataCluster& measurement) override;

  // Estimate the current graph
  bool estimate() override;

protected:
  // Initialize gravity alignment and the LiDAR maps
  bool initialize(const ScanPtr& scan);

  // Add a deskewed LiDAR scan and its IMU state
  bool addLidarMeasurementAndState(const ScanPtr& scan) override;

  // Remove old non-keyframes and marginalize the oldest keyframe
  bool marginalization();

protected:
  LidarImuEstimatorOptions lidar_imu_options_;
  bool initialized_ = false;
  size_t registration_residual_count_ = 0;
  bool registration_degraded_ = false;
};

}  // namespace gici
