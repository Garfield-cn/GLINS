/**
 * @Function: GNSS/IMU/LiDAR semi-tightly coupled estimator
 *
 * @Author  : Jiahui Liu
 * @Email   : jh.liu@sjtu.edu.cn
 **/
#pragma once

#include "gici/gnss/gnss_loose_estimator_base.h"
#include "gici/imu/imu_estimator_base.h"
#include "gici/lidar/lidar_estimator_base.h"
#include "gici/fusion/gnss_imu_initializer.h"

namespace gici {

// GNSS solution, raw IMU and raw LiDAR (SRR) estimator options
struct GnssImuLidarSrrEstimatorOptions {
  // Maximum number of LiDAR keyframes in the optimization window
  size_t max_window_length = 10;

  // GNSS state window length before LiDAR initialization
  size_t max_gnss_window_length_minor = 10;

  // Maximum yaw standard deviation for LiDAR initialization, in degrees
  double min_yaw_std_init_lidar = 0.5;

  // GNSS position standard deviation rejection threshold, in meters
  double reject_solution_std = 0.2;

  // Hold-off after GNSS recovery before reusing GNSS constraints, in seconds
  double solution_recover_time = 0.5;

  // Ignore GNSS solution states after LiDAR initialization and run in LIO mode
  bool lio_mode = false;
};

// Estimator
class GnssImuLidarSrrEstimator : public GnssLooseEstimatorBase,
                                 public ImuEstimatorBase,
                                 public LidarEstimatorBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GnssImuLidarSrrEstimator(const GnssImuLidarSrrEstimatorOptions& options,
                           const GnssImuInitializerOptions& init_options,
                           const LidarEstimatorBaseOptions& lidar_base_options,
                           const GnssLooseEstimatorBaseOptions& gnss_loose_base_options,
                           const ImuEstimatorBaseOptions& imu_base_options,
                           const EstimatorBaseOptions& base_options);
  ~GnssImuLidarSrrEstimator();

  // Add measurement
  bool addMeasurement(const EstimatorDataCluster& measurement) override;

  // Initialize the LiDAR maps after yaw convergence
  bool lidarInitialization(const ScanPtr& scan);

  // Estimate current graph
  bool estimate() override;

  // Set initialization result
  void setInitializationResult(
      const std::shared_ptr<MultisensorInitializerBase>& initializer) override;

protected:
  // Add GNSS solution measurement and state
  bool addGnssSolutionMeasurementAndState(const GnssSolution& measurement);

  // Add LiDAR measurement and state
  bool addLidarMeasurementAndState(const ScanPtr& scan,
                                   const SpeedAndBias& speed_and_bias = SpeedAndBias::Zero(),
                                   const Transformation& T_WS = Transformation(
                                       Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()));

  // Marginalization
  bool marginalization(const IdType& type);

  // Marginalization when the new state is a GNSS state
  bool gnssMarginalization();

  // Marginalization when the new state is a LiDAR state
  bool lidarMarginalization();

  // Sparsify GNSS states to bound computational load
  void sparsifyGnssStates();

  // Reject GNSS solution using its reported position uncertainty
  bool rejectGnssSolution(const GnssSolution& measurement);

  // Check whether GNSS has just recovered
  bool gnssJustRecovered(const ScanPtr& scan);

protected:
  // Options
  GnssImuLidarSrrEstimatorOptions lidar_srr_options_;

  // Initialization control
  std::shared_ptr<GnssImuInitializer> gnss_imu_initializer_;
  bool lidar_initialized_ = false;

  bool gnss_denied_ = false;
  double gnss_recover_timestamp_ = 0.0;
  double last_set_map_time_ = 0.0;
};

}
