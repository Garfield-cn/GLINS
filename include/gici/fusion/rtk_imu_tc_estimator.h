/**
* @Function: RTK/IMU tightly integration
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
#include "gici/gnss/rtk_estimator.h"
#include "gici/imu/imu_types.h"
#include "gici/fusion/gnss_imu_initialization.h"

namespace gici {

// Options
struct RtkImuTcEstimatorOptions {
  // Max iteration number for ceres optimization
  int max_iteration = 30;

  // Number of threads used for ceres optimization
  int num_threads = 2;

  // Verbose optimization output
  bool verbose = false;

  // Max age to apply difference
  double max_age = 20.0;

  // State window length
  int window_length = 3;

  // Use ambiguity resolution
  bool use_ambiguity_resolution = true;

  // GNSS extrinsic variation variance
  double gnss_relative_extrinsic_std = 1.0e-6;

  // GNSS common options
  GnssCommonOptions gnss_common;

  // GNSS error parameter
  GnssErrorParameter gnss_error_parameter;

  // Ambiguity resolution parameter
  AmbiguityResolutionOptions ambiguity_resolution;

  // IMU parameters
  ImuParameters imu_parameters;

  // Initialization options
  GnssImuInitializationOptions initialize;
};

// Estimator
class RtkImuTcEstimator {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct State {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BackendId id;
    std::vector<BackendId> ambiguity_ids;
    double timestamp = 0.0;
    GnssSolutionStatus status = GnssSolutionStatus::Single;
    void clear() {
      id = BackendId(0); ambiguity_ids.clear(); 
      timestamp = 0.0; status = GnssSolutionStatus::Single;
    }
  };

  RtkImuTcEstimator(const RtkImuTcEstimatorOptions& options);
  ~RtkImuTcEstimator();

  // Add GNSS measurements and state
  bool addGnssMeasurementAndState(const GnssMeasurement& measurement_rov, 
                                  const GnssMeasurement& measurement_ref);

  // Add IMU measurement
  void addImuMeasurement(const ImuMeasurement& imu_measurement);

  // Start ceres optimization
  void optimize();

  // Set gravity
  void setGravity(double gravity) { 
    options_.imu_parameters.g = gravity;
    initializer_->setGravity(gravity);
  }

  // Get pose
  Transformation getPoseEstimate();

  // Get speed and bias
  SpeedAndBias getSpeedAndBias();

  // Get Relative position between GNSS and IMU
  Eigen::Vector3d getGnssExtrinsic();

  // Get status
  GnssSolutionStatus getSolutionStatus() { return lastState().status; }

  // Get number of satellites
  int getNumSatellites() { return num_satellites_; }

  // Get differential age
  int getAge() { return differential_age_; }

  // Get timestamp
  double getTimestamp() { return lastState().timestamp; }

  // Set coordinate handle
  void setCoordinate(const GeoCoordinatePtr& coordinate) { 
    coordinate_ = coordinate;
    initializer_->setCoordinate(coordinate_);
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

  // Set position estimate to measurement 
  void setPositionEstimateToMeas();

  // Getters
  inline GnssMeasurement& curGnssRef() { return gnss_measurements_.back().second; }
  inline GnssMeasurement& curGnssRov() { return gnss_measurements_.back().first; }
  inline GnssMeasurement& lastGnssRef() { 
    CHECK(gnss_measurements_.size() >= 2);
    return gnss_measurements_.at(gnss_measurements_.size() - 2).second;
  }
  inline GnssMeasurement& lastGnssRov() { 
    CHECK(gnss_measurements_.size() >= 2);
    return gnss_measurements_.at(gnss_measurements_.size() - 2).first;
  }
  inline State& curState() { return states_.back(); }
  inline State& lastState() { 
    CHECK(states_.size() >= 2);
    return states_.at(states_.size() - 2); 
  }
  inline State& oldestState() { return states_.front(); }

private:
  // Graph that handles residuals and states
  std::shared_ptr<Graph> graph_ptr_;

  // Options
  RtkImuTcEstimatorOptions options_;

  // loss function 
  std::shared_ptr< ceres::LossFunction> cauchy_loss_function_ptr_; ///< Cauchy loss.
  std::shared_ptr< ceres::LossFunction> huber_loss_function_ptr_; ///< Huber loss.

  // States
  std::deque<State> states_;
  int num_satellites_;
  int differential_age_;

  // Measurements
  std::deque<std::pair<GnssMeasurement, GnssMeasurement>> gnss_measurements_;
  ImuMeasurements imu_measurements_;
  GeoCoordinatePtr coordinate_;

  // Initialization
  std::unique_ptr<GnssImuInitialization> initializer_;
  std::unique_ptr<RtkEstimator> rtk_estimator_;
  bool imu_initialized_ = false;

  // Ambiguity resolution
  std::unique_ptr<AmbiguityResolution> ambiguity_resolution_;

  // the marginalized error term
  std::shared_ptr<MarginalizationError> marginalization_error_ptr_;
  ceres::ResidualBlockId marginalization_residual_id_;

  // Debug
  std::unique_ptr<CeresDebugCallback> debug_callback_;
};

}