/**
* @Function: GNSS/IMU initialization
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#pragma once

#include <memory>

#include "gici/optimizer/graph.hpp"
#include "gici/optimizer/estimator_types.hpp"
#include "gici/optimizer/ceres_iteration_callback.h"
#include "gici/gnss/gnss_types.h"
#include "gici/gnss/geodetic_coordinate.h"
#include "gici/imu/imu_types.h"
#include "gici/optimizer/marginalization_error.hpp"

namespace gici {

// IMU initialization options
struct GnssImuInitializationOptions {
  // Max iteration number for ceres optimization
  int max_iteration = 100;

  // Number of threads used for ceres optimization
  int num_threads = 2;

  // Verbose optimization output
  bool verbose = false;

  // We should keep stady during this period to initialize roll, pitch and angular rate bias.
  double time_window_length_zero_motion = 0.5;

  // GNSS measurement window length, we use GNSS measurements and the corresponding 
  // IMU measurments in this window to optimize the initial values.
  int window_length_optimize = 10;

  // Relative position from IMU to GNSS in IMU frame
  Eigen::Vector3d gnss_extrinsic;

  // GNSS extrinsic initial variance
  double gnss_extrinsic_initial_std = 0.5;

  // GNSS extrinsic variation variance
  double gnss_relative_extrinsic_std = 1.0e-4;

  // Min velocity to start initialization, we need a relatively large velocity to ensure 
  // the observability of yaw attitude.
  double min_velocity = 2.0;

  // IMU parameters
  ImuParameters imu_parameters;
};

// Estimator
class GnssImuInitialization {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GnssImuInitialization(const GnssImuInitializationOptions& options);
  GnssImuInitialization(const GnssImuInitializationOptions& options, 
                        const std::shared_ptr<Graph>& graph_ptr);
  ~GnssImuInitialization();

  // Set coordinate for world frame transformation
  void setCoordinate(const GeoCoordinatePtr& coordinate) { 
    coordinate_ = coordinate; 
  }

  // Add GNSS measurement
  bool addGnssMeasurement(const GnssSolution& gnss_solution);

  // Add IMU measurement
  void addIMUMeasurement(const ImuMeasurement& imu_measurement);

  // Get initialization result
  bool getResult(Transformation& T_WS, Eigen::Matrix<double, 7, 7>& cov_T_WS,
                 SpeedAndBias& speed_and_bias, 
                 Eigen::Matrix<double, 9, 9>& cov_speed_and_bias,
                 Eigen::Vector3d& t_SR_S, Eigen::Matrix3d& cov_t_SR_S,
                 bool compute_covariance = false);

  // Marginalize the used measurements to a given window length 
  bool marginalization(const int window_length,
                       const std::shared_ptr<MarginalizationError>& marginalization_ptr,
                       std::deque<GnssSolution>& left_gnss_solutions, 
                       ceres::ResidualBlockId& marginalization_residual_id);

private:
  // Graph that handles residuals and states
  std::shared_ptr<Graph> graph_ptr_;

  // Options
  GnssImuInitializationOptions options_;

  // loss function 
  std::shared_ptr< ceres::LossFunction> cauchy_loss_function_ptr_; ///< Cauchy loss.
  std::shared_ptr< ceres::LossFunction> huber_loss_function_ptr_; ///< Huber loss.

  // flag
  bool zero_motion_finished_ = false;
  bool finished_ = false;

  // initialized by zero motion
  Transformation T_WS_0_;
  SpeedAndBias speed_and_bias_0_;

  // Measurements
  std::deque<GnssSolution> gnss_solutions_;
  ImuMeasurements imu_measurements_;
  GeoCoordinatePtr coordinate_;

  // Debug
  std::unique_ptr<CeresDebugCallback> debug_callback_;
};

using GnssImuInitializationPtr = std::shared_ptr<GnssImuInitialization>;

}