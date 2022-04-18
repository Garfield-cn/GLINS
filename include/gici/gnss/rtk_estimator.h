/**
* @Function: Short baseline RTK implementation
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#pragma once

#include <memory>

#include "gici/optimizer/graph.hpp"
#include "gici/gnss/gnss_types.h"
#include "gici/optimizer/estimator_types.hpp"
#include "gici/optimizer/ceres_iteration_callback.h"
#include "gici/gnss/ambiguity.h"
#include "gici/optimizer/marginalization_error.hpp"

namespace gici {

// RTK options
struct RTKEstimatorOptions {
  // Max iteration number for ceres optimization
  int max_iteration = 15;

  // Number of threads used for ceres optimization
  int num_threads = 2;

  // Verbose optimization output
  bool verbose = false;

  // Max age to apply difference
  double max_age = 20.0;

  // State window length
  int window_length = 5;

  // GNSS common options
  GNSSCommonOptions common;

  // GNSS error parameter
  GNSSErrorParameter error_parameter;

  // Ambiguity resolution parameter
  AmbiguityResolutionOptions ambiguity_resolution;
};

// SPP estimator
class RTKEstimator {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct State {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BackendId id;
    std::vector<BackendId> ambiguity_ids;
    double timestamp = 0.0;
    GNSSSolutionStatus status = GNSSSolutionStatus::Single;
    void clear() {
      id = BackendId(0); ambiguity_ids.clear(); 
      timestamp = 0.0; status = GNSSSolutionStatus::Single;
    }
  };

  RTKEstimator(const RTKEstimatorOptions& options);
  ~RTKEstimator();

  // Add GNSS measurements and state
  // measurement_ref should from the reference station
  bool addGNSSMeasurementAndState(const GNSSMeasurement& measurement_rov, 
                                  const GNSSMeasurement& measurement_ref);

  // Start ceres optimization
  void optimize();

  // Get position in ECEF coordinate
  Eigen::Vector3d getPositionEstimate();

  // Get solution status
  GNSSSolutionStatus getSolutionStatus() { return lastState().status; }

  // Check if it is the first epoch
  bool isFirstEpoch() { return states_.size() < 2; }

private:
  // Compute initial ambiguity
  double getInitialAmbiguity(const GNSSMeasurement& measurement_rov, 
                             const GNSSMeasurement& measurement_ref,
                             const GNSSMeasurementIndex& index_rov,
                             const GNSSMeasurementIndex& index_ref);

  // Marginalization
  bool marginalization();

  // Delete current states and measurement
  void clearCurrentStateAndMeasurement();

  // Set position estimate to measurement 
  void setPositionEstimateToMeas();

  // Getters
  inline GNSSMeasurement& curMeasRef() { return measurements_.back().second; }
  inline GNSSMeasurement& curMeasRov() { return measurements_.back().first; }
  inline GNSSMeasurement& lastMeasRef() { 
    CHECK(measurements_.size() >= 2);
    return measurements_.at(measurements_.size() - 2).second;
  }
  inline GNSSMeasurement& lastMeasRov() { 
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
  RTKEstimatorOptions options_;

  // loss function 
  std::shared_ptr< ceres::LossFunction> cauchy_loss_function_ptr_; ///< Cauchy loss.
  std::shared_ptr< ceres::LossFunction> huber_loss_function_ptr_; ///< Huber loss.

  // States
  std::deque<State> states_;

  // Measurements
  std::deque<std::pair<GNSSMeasurement, GNSSMeasurement>> measurements_;

  // Ambiguity resolution
  std::unique_ptr<AmbiguityResolution> ambiguity_resolution_;

  // the marginalized error term
  std::shared_ptr<MarginalizationError> marginalization_error_ptr_;
  ceres::ResidualBlockId marginalization_residual_id_;

  // Debug
  std::unique_ptr<CeresDebugCallback> debug_callback_;
};

}