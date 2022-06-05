/**
* @Function: Single differenced pseudorange positioning implementation
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#pragma once

#include <memory>

#include "gici/estimate/graph.h"
#include "gici/gnss/gnss_types.h"
#include "gici/estimate/estimator_types.h"

namespace gici {

// SDGNSS options
struct SdgnssEstimatorOptions {
  // Max iteration number for ceres optimization
  int max_iteration = 15;

  // Number of threads used for ceres optimization
  int num_threads = 2;

  // Verbose optimization output
  bool verbose = false;

  // Max age to apply difference
  double max_age = 30.0;

  // GNSS common options
  GnssCommonOptions common;

  // GNSS error parameter
  GnssErrorParameter error_parameter;
};

// Estimator
class SdgnssEstimator {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct State {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BackendId id;
    double timestamp = 0.0;
  };

  SdgnssEstimator(const SdgnssEstimatorOptions& options);
  ~SdgnssEstimator();

  // Add GNSS measurements and state
  // measurement_ref should from the reference station
  bool addGnssMeasurementAndState(const GnssMeasurement& measurement_rov, 
                                  const GnssMeasurement& measurement_ref);

  // Apply ceres optimization
  void optimize();

  // Get position in ECEF coordinate
  Eigen::Vector3d getPositionEstimate();

private:
  // Graph that handles residuals and states
  std::shared_ptr<Graph> graph_;

  // Options
  SdgnssEstimatorOptions options_;

  // loss function
  std::shared_ptr<ceres::LossFunction> cauchy_loss_function_ptr_; 
  std::shared_ptr<ceres::LossFunction> huber_loss_function_ptr_; 

  // Measurement
  GnssMeasurement measurement_rov_;
  GnssMeasurement measurement_ref_;

  // States
  State current_state_;
  std::vector<BackendId> parameter_ids_;
};

}