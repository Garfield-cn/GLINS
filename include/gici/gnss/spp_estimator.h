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
struct SPPEstimatorOptions {
  // Max iteration number for ceres optimization
  int max_iteration = 15;

  // Number of threads used for ceres optimization
  int num_threads = 2;

  // Verbose optimization output
  bool verbose = false;

  // GNSS common options
  GNSSCommonOptions common;

  // GNSS error parameter
  GNSSErrorParameter error_parameter;
};

// SPP estimator
class SPPEstimator {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct State {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BackendId id;
    double timestamp = 0.0;
  };

  SPPEstimator(const SPPEstimatorOptions& options);
  ~SPPEstimator();

  // Add GNSS measurements and state
  bool addGNSSMeasurementAndState(const GNSSMeasurement& measurement);

  // Start ceres optimization
  void optimize();

  // Get position in ECEF coordinate
  Eigen::Vector3d getPositionEstimate();

  // Get Satellite clock
  double getClockEstimate(const char system);

  // Correct DCB (or TGD)
  void correctDCB(GNSSMeasurement& measurement);

  // Compute and set coarse position on measurement
  static bool setCoarsePosition(GNSSMeasurement& measurement);

private:
  // Graph that handles residuals and states
  std::shared_ptr<Graph> graph_ptr_;

  // Options
  SPPEstimatorOptions options_;

  // loss function
  std::shared_ptr< ceres::LossFunction> cauchy_loss_function_ptr_; ///< Cauchy loss.
  std::shared_ptr< ceres::LossFunction> huber_loss_function_ptr_; ///< Huber loss.

  // Measurement
  GNSSMeasurement measurement_;

  // States
  State current_state_;
  std::vector<BackendId> parameter_ids_;
};

using SPPEstimatorPtr = std::shared_ptr<SPPEstimator>;

}