/**
* @Function: GNSS/IMU initialization
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#pragma once

#include <memory>

#include "gici/estimate/graph.h"
#include "gici/estimate/estimator_types.h"
#include "gici/estimate/ceres_iteration_callback.h"
#include "gici/gnss/gnss_types.h"
#include "gici/gnss/geodetic_coordinate.h"
#include "gici/imu/imu_types.h"
#include "gici/estimate/marginalization_error.h"

namespace gici {

// IMU initialization options
struct GnssImuInitializationOptions {
  // Max iteration number for ceres optimization
  int max_iteration = 30;

  // Number of threads used for ceres optimization
  int num_threads = 2;

  // Verbose optimization output
  bool verbose = false;

  // We should keep stady during this period to initialize roll, pitch and angular rate bias.
  double time_window_length_zero_motion = 0.5;

  // GNSS measurement window length, we use GNSS measurements and the corresponding 
  // IMU measurments in this window to optimize the initial values.
  int min_window_length = 20;

  // Relative position from IMU to GNSS in IMU frame
  Eigen::Vector3d gnss_extrinsics;

  // GNSS extrinsics initial variance
  double gnss_extrinsic_initial_std = 0.5;

  // GNSS extrinsics variation variance
  double gnss_relative_extrinsic_std = 1.0e-6;

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
                        const std::shared_ptr<Graph>& graph);
  ~GnssImuInitialization();

  // Set a graph pointer 
  void setGraph(const std::shared_ptr<Graph> graph) {
    graph_ = graph;
  }

  // Set coordinate for world frame transformation
  void setCoordinate(const GeoCoordinatePtr& coordinate) { 
    coordinate_ = coordinate; 
  }

  // Set gravity
  void setGravity(double gravity) { 
    options_.imu_parameters.g = gravity;
    gravity_setted_ = true;
  }

  // Add GNSS measurement
  bool addGnssMeasurement(const GnssSolution& gnss_solution);

  // Add IMU measurement
  void addImuMeasurement(const ImuMeasurement& imu_measurement);

  // Check if finished
  bool finished() const { return finished_; }

  // Get initialization result
  bool getResult(Transformation& T_WS, Eigen::Matrix<double, 6, 6>* cov_T_WS,
                 SpeedAndBias& speed_and_bias, 
                 Eigen::Matrix<double, 9, 9>* cov_speed_and_bias,
                 Eigen::Vector3d& t_SR_S, Eigen::Matrix3d* cov_t_SR_S);

  // Marginalize the used measurements to a given window length 
  bool marginalization(const int window_length,
                       const std::shared_ptr<MarginalizationError>& marginalization_ptr,
                       std::deque<GnssSolution>& left_gnss_solutions, 
                       ceres::ResidualBlockId& marginalization_residual_id);

protected:
  // Graph that handles residuals and states
  std::shared_ptr<Graph> graph_;

  // Options
  GnssImuInitializationOptions options_;

  // loss function 
  std::shared_ptr<ceres::LossFunction> cauchy_loss_function_ptr_; 
  std::shared_ptr<ceres::LossFunction> huber_loss_function_ptr_; 

  // flag
  bool use_outside_graph_ = false;
  bool gravity_setted_ = false;
  bool zero_motion_finished_ = false;
  bool finished_ = false;

  // initialized by zero motion
  Transformation T_WS_0_;
  SpeedAndBias speed_and_bias_0_;

  // Measurements
  std::deque<GnssSolution> gnss_solutions_;
  ImuMeasurements imu_measurements_;
  std::mutex imu_mutex_;
  GeoCoordinatePtr coordinate_;

  // Debug
  std::unique_ptr<CeresDebugCallback> debug_callback_;
};

}