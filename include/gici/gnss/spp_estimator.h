/**
* @Function: Single Point positioning implementation
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#pragma once

#include <memory>

#include "gici/optimizer/graph.hpp"
#include "gici/gnss/gnss_types.h"
#include "gici/optimizer/estimator_types.hpp"

namespace gici {

// SPP options
struct SppEstimatorOptions {
  // Max iteration number for ceres optimization
  int max_iteration = 15;

  // Number of threads used for ceres optimization
  int num_threads = 2;

  // Verbose optimization output
  bool verbose = false;

  // GNSS common options
  GnssCommonOptions common;

  // GNSS error parameter
  GnssErrorParameter error_parameter;
};

// Estimator
class SppEstimator {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct State {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BackendId id;
    double timestamp = 0.0;
  };

  SppEstimator(const SppEstimatorOptions& options);
  ~SppEstimator();

  // Add GNSS measurements and state
  bool addGnssMeasurementAndState(const GnssMeasurement& measurement);

  // Start ceres optimization
  void optimize();

  // Get position in ECEF coordinate
  Eigen::Vector3d getPositionEstimate();

  // Get Satellite clock
  double getClockEstimate(const char system);

  // Correct DCB (or TGD)
  void correctDCB(GnssMeasurement& measurement);

  // Compute and set coarse position on measurement
  static bool setCoarsePosition(GnssMeasurement& measurement);

private:
  // Graph that handles residuals and states
  std::shared_ptr<Graph> graph_ptr_;

  // Options
  SppEstimatorOptions options_;

  // loss function
  std::shared_ptr< ceres::LossFunction> cauchy_loss_function_ptr_; ///< Cauchy loss.
  std::shared_ptr< ceres::LossFunction> huber_loss_function_ptr_; ///< Huber loss.

  // Measurement
  GnssMeasurement measurement_;

  // States
  State current_state_;
  std::vector<BackendId> parameter_ids_;
};

using SppEstimatorPtr = std::shared_ptr<SppEstimator>;

}