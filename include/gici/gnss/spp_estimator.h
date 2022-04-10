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
  // Usage of satellite systems
  // In default, we use all systems
  std::vector<char> system_exclude;

  // Usage of specific satellite
  // In default, we use all satellites
  std::vector<std::string> satellite_exclude;

  // Usage of code types
  // In default, we use all code types
  std::vector<int> code_exclude;

  // Max iteration number for ceres optimization
  int max_iteration = 10;

  // Number of threads used for ceres optimization
  int num_threads = 2;

  // Verbose optimization output
  bool verbose = false;

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
  double getClockEstimate(const char system, double& clock);

  // Check whether the system is used
  bool useSystem(const char system);

  // Check whether the satellite is used
  bool useSatellite(const std::string prn);

  // Check whether the code type is used
  bool useCode(const int code_type);

private:
  // Graph that handles residuals and states
  std::shared_ptr<Graph> graph_ptr_;

  // Options
  SPPEstimatorOptions options_;

  // loss function for reprojection errors
  std::shared_ptr< ceres::LossFunction> cauchy_loss_function_ptr_; ///< Cauchy loss.
  std::shared_ptr< ceres::LossFunction> huber_loss_function_ptr_; ///< Huber loss.

  // States
  State current_state_;
  std::vector<BackendId> parameter_ids_;
};

}