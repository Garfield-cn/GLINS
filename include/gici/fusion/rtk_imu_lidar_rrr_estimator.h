/**
 * @Function: GNSS/IMU/LiDAR tightly coupled estimator using RTK
 *
 * @Author  : Jiahui Liu
 * @Email   : jh.liu@sjtu.edu.cn
 *
 * Copyright (C) 2024 by Jiahui Liu, All rights reserved.
 **/
#pragma once

#include "gici/gnss/gnss_estimator_base.h"
#include "gici/imu/imu_estimator_base.h"
#include "gici/lidar/lidar_estimator_base.h"
#include "gici/gnss/rtk_estimator.h"
#include "gici/fusion/gnss_imu_initializer.h"

namespace gici {

// Raw GNSS, raw IMU and raw LiDAR (RRR) estimator options
struct RtkImuLidarRrrEstimatorOptions {
  // Maximum number of LiDAR keyframes in the optimization window
  // We only keep GNSS measurements near to keyframes (one-to-one) and throw the others
  // away after one optimization, because the GNSS measurement errors, especially for
  // the multipath, are highly correlated between epochs when we have a slow or zero motion.
  // Besides, we need at least 2 GNSS states in window. If current setting cannot ensure
  // this condition, we will ignore this option and extend the windows length.
  size_t max_keyframes = 5;

  // GNSS state window length before LiDAR initialization
  size_t max_gnss_window_length_minor = 3;

  // Maximum yaw standard deviation for LiDAR initialization, in degrees
  double min_yaw_std_init_lidar = 0.5;
};

// Estimator
class RtkImuLidarRrrEstimator : public GnssEstimatorBase,
                                public LidarEstimatorBase,
                                public ImuEstimatorBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RtkImuLidarRrrEstimator(const RtkImuLidarRrrEstimatorOptions& options,
                          const GnssImuInitializerOptions& init_options,
                          const RtkEstimatorOptions rtk_options,
                          const GnssEstimatorBaseOptions& gnss_base_options,
                          const GnssLooseEstimatorBaseOptions& gnss_loose_base_options,
                          const LidarEstimatorBaseOptions& lidar_base_options,
                          const ImuEstimatorBaseOptions& imu_base_options,
                          const EstimatorBaseOptions& base_options,
                          const AmbiguityResolutionOptions& ambiguity_options);
  ~RtkImuLidarRrrEstimator();

  // Add measurement
  bool addMeasurement(const EstimatorDataCluster& measurement) override;

  // Estimate current graph
  bool estimate() override;

  // Set initialization result
  void setInitializationResult(
      const std::shared_ptr<MultisensorInitializerBase>& initializer) override;

protected:
  // Add rover and reference GNSS measurements and state
  bool addGnssMeasurementAndState(const GnssMeasurement& measurement_rov,
                                  const GnssMeasurement& measurement_ref);

  // Add LiDAR measurement and state
  bool addLidarMeasurementAndState(const ScanPtr& scan,
                                   const SpeedAndBias& speed_and_bias = SpeedAndBias::Zero(),
                                   const Transformation& T_WS = Transformation(
                                       Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()));

  // Initialize the LiDAR maps after yaw convergence
  bool lidarInitialization(const ScanPtr& scan);

  // Marginalization
  bool marginalization(const IdType& type);

  // Marginalization when the new state is a LiDAR state
  bool lidarMarginalization();

  // Marginalization when the new state is a GNSS state
  bool gnssMarginalization();

  // Sparsify GNSS states to bound computational load
  void sparsifyGnssStates();

  // Compute ambiguity covariance at current epoch
  bool estimateAmbiguityCovariance(const State& state, Eigen::MatrixXd& covariance);

  // Get latest state
  inline State& latestState() override
  {
    return states_[latest_state_index_];
  }

protected:
  // Options
  RtkImuLidarRrrEstimatorOptions rrr_options_;
  RtkEstimatorOptions rtk_options_;

  // Initialization control
  std::shared_ptr<GnssImuInitializer> gnss_imu_initializer_;
  std::shared_ptr<RtkEstimator> initializer_sub_estimator_;
  bool lidar_initialized_ = false;

  // Measurement alignment handle
  DifferentialMeasurementsAlign measurement_align_;

  // RTK estimator used for ambiguity covariance estimation
  std::unique_ptr<RtkEstimator> ambiguity_covariance_estimator_;
  bool ambiguity_covariance_coordinate_set_ = false;

  // Status control
  size_t num_continuous_unfix_ = 0;
  size_t num_continuous_reject_gnss_ = 0;
};

}
