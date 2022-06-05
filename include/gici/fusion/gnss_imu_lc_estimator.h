/**
* @Function: GNSS/IMU loosely integration
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#pragma once

#include <memory>

#include "gici/estimate/graph.h"
#include "gici/estimate/estimator_types.h"
#include "gici/estimate/ceres_iteration_callback.h"
#include "gici/estimate/marginalization_error.h"
#include "gici/gnss/gnss_types.h"
#include "gici/gnss/geodetic_coordinate.h"
#include "gici/imu/imu_types.h"
#include "gici/fusion/gnss_imu_initialization.h"

namespace gici {

// Options
struct GnssImuLcEstimatorOptions {
  // Max iteration number for ceres optimization
  int max_iteration = 30;

  // Number of threads used for ceres optimization
  int num_threads = 2;

  // Verbose optimization output
  bool verbose = false;

  // State window length
  int window_length = 3;

  // GNSS extrinsics variation variance
  double gnss_relative_extrinsic_std = 1.0e-6;

  // IMU parameters
  ImuParameters imu_parameters;

  // Initialization options
  GnssImuInitializationOptions initialize;
};

// Estimator
class GnssImuLcEstimator {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct State {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BackendId id; // id in pose type
    double timestamp = 0.0;
  };

  GnssImuLcEstimator(const GnssImuLcEstimatorOptions& options);
  ~GnssImuLcEstimator();

  // Set initialization result 
  void setInitializationResult(
    const std::shared_ptr<GnssImuInitialization>& initializer);

  // Add GNSS measurements and state
  bool addGnssMeasurementAndState(const GnssSolution& gnss_solution);

  // Add IMU measurement
  void addImuMeasurement(const ImuMeasurement& imu_measurement);

  // Apply ceres optimization
  void optimize();

  // Set gravity
  void setGravity(double gravity) { 
    options_.imu_parameters.g = gravity;
  }

  // Get graph pointer
  const std::shared_ptr<Graph>& getGraph() const { return graph_; }

  // Get timestamp
  double getTimestamp() { return lastState().timestamp; }

  // Get pose
  Transformation getPoseEstimate();

  // Get speed and bias
  SpeedAndBias getSpeedAndBias();

  // Get GNSS extrinsics
  Eigen::Vector3d getGnssExtrinsic();

  // Set coordinate handle
  void setCoordinate(const GeoCoordinatePtr& coordinate) { 
    coordinate_ = coordinate;
  }

  // Get coordinate handle
  GeoCoordinatePtr& getCoordinate() { return coordinate_; }

  // Get IMU measurements
  ImuMeasurements& getImuMeasurements() { return imu_measurements_; }

  // Get IMU parameters
  ImuParameters& getImuParameters() { return options_.imu_parameters; }

  // Check if it is the first epoch
  bool isFirstEpoch() { return states_.size() < 2; }

private:
  // Marginalization
  bool marginalization();

  // Getters
  inline GnssSolution& curGnss() { return gnss_solutions_.back(); }
  inline GnssSolution& lastGnss() { 
    CHECK(gnss_solutions_.size() >= 2);
    return gnss_solutions_.at(gnss_solutions_.size() - 2); 
  }
  inline State& curState() { return states_.back(); }
  inline State& lastState() { 
    // TODO: If we add a breakpoint on any position of this class, when the breakpoint
    // is hitted, this function will be unexpectedly called. Why??
#ifndef NDEBUG
    if (states_.size() >= 2)
      return states_.at(states_.size() - 2); 
    else return states_.back();
#else
    CHECK(states_.size() >= 2);
    return states_.at(states_.size() - 2); 
#endif
  }
  inline State& oldestState() { return states_.front(); }

private:
  // Graph that handles residuals and states
  std::shared_ptr<Graph> graph_;

  // Options
  GnssImuLcEstimatorOptions options_;

  // loss function 
  std::shared_ptr<ceres::LossFunction> cauchy_loss_function_ptr_; 
  std::shared_ptr<ceres::LossFunction> huber_loss_function_ptr_; 

  // States
  std::deque<State> states_;

  // Measurements
  std::deque<GnssSolution> gnss_solutions_;
  ImuMeasurements imu_measurements_;
  GeoCoordinatePtr coordinate_;

  // Initialization
  bool initialized_ = false;

  // the marginalized error term
  std::shared_ptr<MarginalizationError> marginalization_error_ptr_;
  ceres::ResidualBlockId marginalization_residual_id_;

  // Debug
  std::unique_ptr<CeresDebugCallback> debug_callback_;
};

}