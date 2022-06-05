/**
* @Function: Couples GNSS solution, camera feature, and IMU raw measuremnet
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
#include "gici/imu/imu_types.h"
#include "gici/fusion/gnss_imu_camera_initialization.h"
#include "gici/vision/image_types.h"

namespace gici {

// Options
struct GnssImuCameraSrrEstimatorOptions {
  // Max iteration number for ceres optimization
  int max_iteration = 10;

  // Number of threads used for ceres optimization
  int num_threads = 2;

  // Verbose optimization output
  bool verbose = false;

  // Frame state window length
  // We only keep GNSS measurements near to keyframes (one-to-one) and throw the others 
  // away after one optimization, because the GNSS measurement errors, especially for 
  // the multipath, are highly correlated between epochs when we have a slow or zero motion.
  // Besides, we need at least 2 GNSS states in window. If current setting cannot ensure 
  // this condition, we will ignore this option and extend the windows length.
  int max_keyframes = 5;

  // Feature error STD (pixel)
  double feature_error_std = 1.0;

  // Only add landmarks to backend with minimum number of observations
  int min_num_obs = 2;

  // Landmark outliter rejection threshold (n sigma)
  double landmark_outlier_rejection_threshold = 3.0;

  // IMU parameters
  ImuParameters imu_parameters;

  // Initialization options
  GnssImuCameraInitializationOptions initialize;
};

// Estimator
class GnssImuCameraSrrEstimator {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct State {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BackendId id; // id in pose type
    double timestamp = 0.0;
    bool is_keyframe = false;
    ceres::ResidualBlockId imu_residual_to_lhs;

    bool valid() { return (timestamp != 0.0); }
  };

  GnssImuCameraSrrEstimator(const GnssImuCameraSrrEstimatorOptions& options);
  ~GnssImuCameraSrrEstimator();

  // Set visual front-end handler. 
  void setFeatureHandler(const std::shared_ptr<FeatureHandler>& feature_handler) {
    feature_handler_ = feature_handler;
  }

  // Set initialization result 
  void setInitializationResult(
    const std::shared_ptr<GnssImuCameraInitialization>& initializer);

  // Add GNSS measurements and state
  bool addGnssMeasurementAndState(const GnssSolution& gnss_solution);

  // Add image measurements and state
  bool addImageMeasurementAndState(const FrameBundlePtr& frame_bundle);

  // Add IMU measurement
  void addImuMeasurement(const ImuMeasurement& imu_measurement);

  // Apply ceres optimization
  void optimize();

  // Set gravity
  void setGravity(double gravity) { 
    options_.imu_parameters.g = gravity;
  }

  // Get graph pointer
  const std::shared_ptr<Graph>& getGraph() const { return graph_; }

  // Get timestamp
  double getTimestamp() { return lastState().timestamp; }

  // Get latest pose
  Transformation getPoseEstimate();

  // Get latest speed and bias
  SpeedAndBias getSpeedAndBias();

  // Get latest GNSS extrinsics
  Eigen::Vector3d getGnssExtrinsic();

  // Set coordinate handle
  void setCoordinate(const GeoCoordinatePtr& coordinate) { 
    coordinate_ = coordinate;
  }

  // Get coordinate handle
  GeoCoordinatePtr& getCoordinate() { return coordinate_; }

  // Get IMU measurements
  ImuMeasurements& getImuMeasurements() { return imu_measurements_; }

  // Get IMU parameters
  ImuParameters& getImuParameters() { return options_.imu_parameters; }

  // Check if it is the first epoch
  bool isFirstEpoch() { return states_.size() < 2; }

  // Check if initialized
  bool isInitialized() { return initialized_; }

private:
  // Marginalization
  bool marginalization();

  // Wait for IMU data
  bool waitForImuData(double timestamp);

  // Check if a landmark is in estimator
  inline bool isLandmarkInEstimator(BackendId landmark_id) const {
    return landmarks_map_.find(landmark_id) != landmarks_map_.end();
  }

  // Add landmark observation
  ceres::ResidualBlockId addLandmarkObservation(
    const FramePtr& frame, const size_t keypoint_idx);

  // Remove landmark observation
  bool removeLandmarkObservation(ceres::ResidualBlockId residual_block_id);

  // Erase redundant GNSS measurements
  // we keep current GNSS measurement and old measurements that near to keyframes.
  int eraseRedundantGnssMeasurements();

  // Get pose estimate at a given state
  Transformation getPoseEstimate(const State& state);

  // Get speed and bias estimate at a given state
  SpeedAndBias getSpeedAndBias(const State& state);

  // Add a state at end of window
  bool pushBackState(double timestamp, BackendId id, bool is_keyframe = false);

  // Insert a state inside the window. Because of the different latency of sensors, we may 
  // add states at previous timestamp. In this case, we should cut off the previous IMU 
  // connection, add a new state between two states, and then form two new IMU connections.
  bool insertState(double timestamp, BackendId id, bool is_keyframe = false);

  // Erase a state inside the window
  bool eraseState(double timestamp, BackendId id);

  // Reject landmark outliers at current frame
  void rejectLandmarkOutliers();

  // Getters
  inline GnssSolution& curGnss() { return gnss_solutions_.back(); }
  inline GnssSolution& lastGnss() { 
    CHECK(gnss_solutions_.size() >= 2);
    return gnss_solutions_.at(gnss_solutions_.size() - 2); 
  }
  inline FramePtr& curFrame() { return frame_bundles_.back()->frames_.at(0); }
  inline FramePtr& lastFrame() { 
    CHECK(frame_bundles_.size() >= 2);
    return frame_bundles_.at(frame_bundles_.size() - 2)->frames_.at(0);
  }
  inline State& curState() { return states_.back(); }
  inline State& lastState() { 
    CHECK(states_.size() >= 2);
    return states_.at(states_.size() - 2);
  }
  inline State& oldestState() { return states_.front(); }
  inline State& lastGnssState() {
    int hit_count = 0;
    for (auto it = states_.rbegin(); it != states_.rend(); it++) {
      State& state = *it;
      // current state set set
      if (!state.valid()) hit_count++;
      else if (state.id.type() == IdType::gPose) hit_count++;
      if (hit_count == 2) return state;
    }
    return null_state_;
  }
  inline State& lastFrameState() {
    int hit_count = 0;
    for (auto it = states_.rbegin(); it != states_.rend(); it++) {
      State& state = *it;
      if (!state.valid()) hit_count++;
      else if (state.id.type() == IdType::cNFrame) hit_count++;
      if (hit_count == 2) return state;
    }
    return null_state_;
  }
  inline int sizeOfKeyframeStates() {
    return std::count_if(states_.begin(), states_.end(), 
           [](State& state) { return state.is_keyframe; });
  }
  inline int sizeOfGnssStates() {
    return std::count_if(states_.begin(), states_.end(), 
           [](State& state) { 
           return BackendId::sensorType(state.id.type()) == SensorType::GNSS; });
  }
  inline int sizeOfActiveGnssStates() {
    return std::count_if(states_.begin(), states_.end(), 
           [](State& state) { 
           return (BackendId::sensorType(state.id.type()) == SensorType::GNSS); });
  }
  inline State& oldestFrameState() {
    for (auto& it : states_) if (it.id.type() == IdType::cNFrame) return it;
    return null_state_;
  }
  inline State& oldestKeyframeState() {
    for (auto& it : states_) if (it.is_keyframe) return it;
    return null_state_;
  }

private:
  // Graph that handles residuals and states
  std::shared_ptr<Graph> graph_;

  // Options
  GnssImuCameraSrrEstimatorOptions options_;

  // loss function 
  std::shared_ptr<ceres::LossFunction> cauchy_loss_function_ptr_; 
  std::shared_ptr<ceres::LossFunction> huber_loss_function_ptr_; 

  // States
  std::deque<State> states_;
  State null_state_;  // just for check
  BackendId camera_extrinsics_id_;  // only one because we do not estimate it
  BackendId gnss_extrinsics_id_;
  PointMap landmarks_map_;
  std::deque<FramePtr> active_keyframes_;
  IdType new_state_type_;

  // Measurements
  std::deque<GnssSolution> gnss_solutions_;
  std::deque<FrameBundlePtr> frame_bundles_;
  ImuMeasurements imu_measurements_;
  std::mutex imu_mutex_;
  GeoCoordinatePtr coordinate_;

  // Initialization
  bool initialized_ = false;

  // Feature handler
  std::shared_ptr<FeatureHandler> feature_handler_;

  // the marginalized error term
  std::shared_ptr<MarginalizationError> marginalization_error_ptr_;
  ceres::ResidualBlockId marginalization_residual_id_;

  // Debug
  std::unique_ptr<CeresDebugCallback> debug_callback_;
};

}