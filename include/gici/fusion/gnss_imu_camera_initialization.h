/**
* @Function: GNSS/IMU/Camera initialization
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#pragma once

#include <memory>

#include "gici/estimate/graph.h"
#include "gici/estimate/estimator_types.h"
#include "gici/estimate/ceres_iteration_callback.h"
#include "gici/gnss/gnss_types.h"
#include "gici/gnss/geodetic_coordinate.h"
#include "gici/imu/imu_types.h"
#include "gici/estimate/marginalization_error.h"
#include "gici/vision/feature_handler.h"
#include "gici/vision/image_types.h"

namespace gici {

// Initialization options
struct GnssImuCameraInitializationOptions {
  // Max iteration number for ceres optimization
  int max_iteration = 30;

  // Number of threads used for ceres optimization
  int num_threads = 2;

  // Verbose optimization output
  bool verbose = false;

  // We should keep stady during this period to initialize roll, pitch and angular rate bias.
  double time_window_length_zero_motion = 0.5;

  // Minimum GNSS measurement window length, when both the GNSS and keyframes reaches the 
  // minimum window length, we start the initialization.
  int min_gnss_window_length = 5;

  // Minimum keyframe window length
  int min_keyframe_window_length = 10;

  // Relative position from IMU to GNSS in IMU frame
  Eigen::Vector3d gnss_extrinsics;

  // GNSS extrinsics initial variance
  double gnss_extrinsic_initial_std = 0.5;

  // Min velocity to start initialization, we need a relatively large velocity to ensure 
  // the observability of yaw attitude.
  double min_velocity = 2.0;

  // Feature error STD (pixel)
  double feature_error_std = 1.0;

  // IMU parameters
  ImuParameters imu_parameters;
};

// Estimator
class GnssImuCameraInitialization {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct State {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BackendId id; // id in pose type
    double timestamp = 0.0;
    ceres::ResidualBlockId imu_residual_to_lhs;
  };

  GnssImuCameraInitialization(const GnssImuCameraInitializationOptions& options, 
                        const std::shared_ptr<Graph>& graph);
  ~GnssImuCameraInitialization();

  // Set coordinate for world frame transformation
  void setCoordinate(const GeoCoordinatePtr& coordinate) { 
    coordinate_ = coordinate; 
  }

  // Set gravity
  void setGravity(double gravity) { 
    options_.imu_parameters.g = gravity;
    gravity_setted_ = true;
  }

  // Add GNSS measurement
  // If the minimum windows lengths for GNSS and keyframes are ensured, this function will 
  // start initialization and return true if it successes.
  bool addGnssMeasurement(const GnssSolution& gnss_solution);

  // Add IMU measurement
  void addImuMeasurement(const ImuMeasurement& imu_measurement);

  // Set visual front-end handler
  // The front-end should be processed outside this class (in a separated thread)
  void setFeatureHandler(const std::shared_ptr<FeatureHandler>& feature_handler) {
    feature_handler_ = feature_handler;
  }

  // Apply initialization process
  void initialize();

  // Check if finished
  bool finished() const { return finished_; }

  // Marginalize the used measurements to a given keyframe window length 
  bool marginalization(const int window_length,
            const std::shared_ptr<MarginalizationError>& marginalization_ptr,
            std::deque<State>& states, 
            ceres::ResidualBlockId& marginalization_residual_id, 
            BackendId& gnss_extrinsics_id, BackendId& camera_extrinsics_id,
            PointMap& landmarks_map, std::deque<FramePtr>& keyframes);

private: 
  // Initialize pose, velocity, and biases with GNSS solutions and IMU measurements
  void optimizeGnssImu();

  // Rescale keyframe poses and corresponding landmark positions with initialized poses
  void rescaleVisual();

  // Optimize GNSS/INS/Camera parameters together
  void optimizeGnssImuCamera();

  // Check if a landmark is in estimator
  inline bool isLandmarkInEstimator(BackendId landmark_id) const {
    return landmarks_map_.find(landmark_id) != landmarks_map_.end();
  }

  // Add landmark observation
  ceres::ResidualBlockId addLandmarkObservation(
    const FramePtr& frame, const size_t keypoint_idx);

protected:
  // Graph that handles residuals and states
  std::shared_ptr<Graph> graph_;

  // Options
  GnssImuCameraInitializationOptions options_;

  // loss function 
  std::shared_ptr<ceres::LossFunction> cauchy_loss_function_ptr_; 
  std::shared_ptr<ceres::LossFunction> huber_loss_function_ptr_; 

  // flag
  bool gravity_setted_ = false;
  bool has_velocity_ = false;
  bool zero_motion_finished_ = false;
  bool finished_ = false;

  // initialized by zero motion
  Transformation T_WS_0_;
  SpeedAndBias speed_and_bias_0_;

  // Measurements
  std::deque<GnssSolution> gnss_solutions_;
  std::deque<FramePtr> keyframes_;
  ImuMeasurements imu_measurements_;
  std::mutex imu_mutex_;
  GeoCoordinatePtr coordinate_;

  // States
  std::vector<std::pair<double, BackendId>> gnss_states_;
  std::deque<State> states_;
  BackendId gnss_extrinsics_id_;
  BackendId camera_extrinsics_id_;
  std::deque<Transformation> keyframe_poses_;
  std::deque<SpeedAndBias> keyframe_speed_and_biases_;
  PointMap landmarks_map_;

  // Feature handler
  std::shared_ptr<FeatureHandler> feature_handler_;

  // Debug
  std::unique_ptr<CeresDebugCallback> debug_callback_;
};

}