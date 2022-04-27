/**
* @Function: GNSS/IMU tightly integration
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#pragma once

#include <memory>

#include "gici/optimizer/graph.hpp"
#include "gici/optimizer/estimator_types.hpp"
#include "gici/optimizer/ceres_iteration_callback.h"
#include "gici/optimizer/marginalization_error.hpp"
#include "gici/gnss/gnss_types.h"
#include "gici/gnss/geodetic_coordinate.h"
#include "gici/imu/imu_types.h"
#include "gici/fusion/gnss_imu_initialization.h"

namespace gici {

// Options
struct GnssImuTcEstimatorOptions {
  // Max iteration number for ceres optimization
  int max_iteration = 30;

  // Number of threads used for ceres optimization
  int num_threads = 2;

  // Verbose optimization output
  bool verbose = false;

  // State window length
  int window_length = 5;

  // GNSS extrinsic variation variance
  double gnss_relative_extrinsic_std = 1.0e-4;

  // IMU parameters
  ImuParameters imu_parameters;
};

// Estimator
class GnssImuTcEstimator {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct State {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BackendId id; // id in pose type
    double timestamp = 0.0;
  };

  GnssImuTcEstimator(const GnssImuTcEstimatorOptions& options, 
                     const GnssImuInitializationOptions& initial_options);
  ~GnssImuTcEstimator();

  // Add GNSS measurements and state
  bool addGnssMeasurementAndState(const GnssSolution& gnss_solution);

  // Add IMU measurement
  void addIMUMeasurement(const ImuMeasurement& imu_measurement);

  // Start ceres optimization
  void optimize();

  // Get pose
  Transformation getPoseEstimate();

  // Get pose at current IMU timestamp
  Transformation getPoseIntegrated();

  // Get current integrated IMU timestamp
  double getTimeIntegrated() const { return timestamp_integrate_; }

  // Get speed and bias
  SpeedAndBias getSpeedAndBias();

  // Get Relative position between GNSS and IMU
  Eigen::Vector3d getGnssExtrinsic();

  // Set coordinate handle
  void setCoordinate(const GeoCoordinatePtr& coordinate) { 
    coordinate_ = coordinate;
    initializer_->setCoordinate(coordinate_);
  }

  // Get coordinate handle
  GeoCoordinatePtr& getCoordinate() { return coordinate_; }

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
  std::shared_ptr<Graph> graph_ptr_;

  // Options
  GnssImuTcEstimatorOptions options_;
  GnssImuInitializationOptions initial_options_;

  // loss function 
  std::shared_ptr< ceres::LossFunction> cauchy_loss_function_ptr_; ///< Cauchy loss.
  std::shared_ptr< ceres::LossFunction> huber_loss_function_ptr_; ///< Huber loss.

  // States
  std::deque<State> states_;
  // the two parameters are used for integrating current solution to IMU timestamp
  double timestamp_integrate_;  // Current integrated timestamp
  Transformation T_WS_integrate_;  // Current integrated IMU pose

  // Measurements
  std::deque<GnssSolution> gnss_solutions_;
  ImuMeasurements imu_measurements_;
  GeoCoordinatePtr coordinate_;

  // Initialization
  GnssImuInitializationPtr initializer_;
  bool imu_initialized_ = false;

  // the marginalized error term
  std::shared_ptr<MarginalizationError> marginalization_error_ptr_;
  ceres::ResidualBlockId marginalization_residual_id_;

  // Debug
  std::unique_ptr<CeresDebugCallback> debug_callback_;
};

}