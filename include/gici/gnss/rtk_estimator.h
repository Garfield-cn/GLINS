/**
* @Function: Short baseline RTK implementation
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#pragma once

#include <memory>

#include "gici/estimate/graph.h"
#include "gici/gnss/gnss_types.h"
#include "gici/estimate/estimator_types.h"
#include "gici/estimate/ceres_iteration_callback.h"
#include "gici/gnss/ambiguity.h"
#include "gici/estimate/marginalization_error.h"

namespace gici {

// Options
struct RtkEstimatorOptions {
  // Max iteration number for ceres optimization
  int max_iteration = 15;

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

  // GNSS common options
  GnssCommonOptions common;

  // GNSS error parameter
  GnssErrorParameter error_parameter;

  // Ambiguity resolution parameter
  AmbiguityResolutionOptions ambiguity_resolution;
};

// Estimator
class RtkEstimator {
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

  RtkEstimator(const RtkEstimatorOptions& options);
  ~RtkEstimator();

  // Add GNSS measurements and state
  bool addGnssMeasurementAndState(const GnssMeasurement& measurement_rov, 
                                  const GnssMeasurement& measurement_ref);

  // Start ceres optimization
  void optimize();

  // Get position in ECEF coordinate
  Eigen::Vector3d getPositionEstimate();

  // Get solution status
  GnssSolutionStatus getSolutionStatus() { return lastState().status; }

  // Get solution
  GnssSolution getSolution();

  // Check if it is the first epoch
  bool isFirstEpoch() { return states_.size() < 2; }

  // Compute initial ambiguity
  double getInitialAmbiguity(const GnssMeasurement& measurement_rov, 
                             const GnssMeasurement& measurement_ref,
                             const GnssMeasurementIndex& index_rov,
                             const GnssMeasurementIndex& index_ref);

private:
  // Marginalization
  bool marginalization();

  // Delete current states and measurement
  void clearCurrentStateAndMeasurement();

  // Set position estimate to measurement 
  void setPositionEstimateToMeas();

  // Getters
  inline GnssMeasurement& curMeasRef() { return measurements_.back().second; }
  inline GnssMeasurement& curMeasRov() { return measurements_.back().first; }
  inline GnssMeasurement& lastMeasRef() { 
    CHECK(measurements_.size() >= 2);
    return measurements_.at(measurements_.size() - 2).second;
  }
  inline GnssMeasurement& lastMeasRov() { 
    CHECK(measurements_.size() >= 2);
    return measurements_.at(measurements_.size() - 2).first;
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
  RtkEstimatorOptions options_;

  // loss function 
  std::shared_ptr< ceres::LossFunction> cauchy_loss_function_ptr_; ///< Cauchy loss.
  std::shared_ptr< ceres::LossFunction> huber_loss_function_ptr_; ///< Huber loss.

  // States
  std::deque<State> states_;
  int num_satellites_;
  int differential_age_;

  // Measurements
  std::deque<std::pair<GnssMeasurement, GnssMeasurement>> measurements_;

  // Ambiguity resolution
  std::unique_ptr<AmbiguityResolution> ambiguity_resolution_;

  // the marginalized error term
  std::shared_ptr<MarginalizationError> marginalization_error_ptr_;
  ceres::ResidualBlockId marginalization_residual_id_;

  // Debug
  std::unique_ptr<CeresDebugCallback> debug_callback_;
};

}