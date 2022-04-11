/**
* @Function: Single differenced pseudorange positioning implementation
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
struct DGNSSEstimatorOptions {
  // Max iteration number for ceres optimization
  int max_iteration = 15;

  // Number of threads used for ceres optimization
  int num_threads = 2;

  // Verbose optimization output
  bool verbose = false;

  // Max age to apply difference
  double max_age = 30.0;

  // GNSS common options
  GNSSCommonOptions common;

  // GNSS error parameter
  GNSSErrorParameter error_parameter;
};

// SPP estimator
class DGNSSEstimator {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct State {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BackendId id;
    double timestamp = 0.0;
  };

  DGNSSEstimator(const DGNSSEstimatorOptions& options);
  ~DGNSSEstimator();

  // Add GNSS measurements and state
  // measurement_2 should from the reference station
  bool addGNSSMeasurementAndState(const GNSSMeasurement& measurement_1, 
                                  const GNSSMeasurement& measurement_2);

  // Start ceres optimization
  void optimize();

  // Get position in ECEF coordinate
  Eigen::Vector3d getPositionEstimate();

  // Get Satellite clock
  double getClockEstimate(const char system, double& dclock);

private:
  // Check observation valid
  bool checkObservationValid(const GNSSMeasurement& measurement,
                             const GNSSMeasurementIndex& index);

  // Form single difference pair
  GNSSMeasurementIndexPairs formMeasurementPair(
    const GNSSMeasurement& measurement_1, const GNSSMeasurement& measurement_2);

  // Graph that handles residuals and states
  std::shared_ptr<Graph> graph_ptr_;

  // Options
  DGNSSEstimatorOptions options_;

  // loss function for reprojection errors
  std::shared_ptr< ceres::LossFunction> cauchy_loss_function_ptr_; ///< Cauchy loss.
  std::shared_ptr< ceres::LossFunction> huber_loss_function_ptr_; ///< Huber loss.

  // States
  State current_state_;
  std::vector<BackendId> parameter_ids_;
};

}