/**
* @Function: Feature detecting and tracking
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#pragma once

#include <memory>

#include "gici/utility/svo.h"
#include "gici/imu/imu_types.h"
#include "gici/vision/feature_matcher.h"
#include "gici/vision/feature_tracker.h"
#include "gici/vision/visual_initialization.h"

namespace gici {

// Feature handler options
struct FeatureHandlerOptions {
  // Max number of keyframes to keep
  size_t max_n_kfs = 10;

  // Max features per frame
  size_t max_features_per_frame = 120;

  // If we have less than this amount of features we always select a new keyframe.
  size_t kfselect_min_numkfs = 60;

  // Minimum disparity to select a new keyframe
  double kfselect_min_disparity = 10.0;

  // Minimum distance in meters before a new keyframe is selected.
  double kfselect_min_dist_metric = 0.5;

  // Minimum angle in degrees to closest KF
  double kfselect_min_angle = 5.0;

  // Image max pyramid level
  int max_pyramid_level = 4;

  // Minimum disparity to triangulate a landmark
  double min_disparity_init_landmark = 5.0;

  // Feature detector options
  DetectorOptions detector;

  // Feature tracker options
  FeatureTrackerOptions tracker;

  // Initialization options
  VisualInitializationOptions initialization;

  // Camera model, can be ATAN, Pinhole or Ocam (see vikit)
  CameraBundlePtr cameras;
};

// Estimator
class FeatureHandler {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FeatureHandler(const FeatureHandlerOptions& options);
  ~FeatureHandler();

  // Add images
  bool addImageBundle(const std::vector<cv::Mat>& imgs, 
                      const double timestamp);

  // Add camera pose to a specific frame at given timestamp
  bool addCameraPose(const double timestamp, 
                     const Transformation& T_WS);

  // After the poses and landmarks were adjusted to global, call this function to 
  // apply the changes.
  void setGlobalInitializationResult(const std::deque<FramePtr>& scaled_frames);

  // Add IMU measurement for pose integration
  void addImuMeasurement(const ImuMeasurement& imu_measurement);

  // Set current speed and bias
  void setSpeedAndBias(const double timestamp, 
                       const SpeedAndBias speed_and_bias);

  // Set a rotation prior for initialization
  inline void setRotationPrior(const Eigen::Quaterniond& R_WS) {
    initializer_->setRotationPrior(R_WS);
  }

  // Get current map
  inline const MapPtr& getMap() const { return map_; }

  // Get camera bundle parameters
  inline const CameraBundle::Ptr& getNCamera() const { return cams_; }

  // Get camera parameters
  inline CameraPtr getCamera() const { return cams_->getCameraShared(0); }

  // Get feature detector
  inline const DetectorPtr& getDetector() const { return detector_; }

  // Get current processed frame bundle
  FrameBundlePtr getFrameBundle() { return lastFrames(); }

  // Check if it is the first frame
  bool isFirstFrame() { return frame_bundles_.size() < 2; }

  // Get current processed frame
  FramePtr getFrame() { return lastFrame(); }

private:
  // Get frame
  const FramePtr& lastFrame() {
    CHECK(frame_bundles_.size() > 1);
    return frame_bundles_[frame_bundles_.size() - 2]->at(0);
  }
  const FramePtr& curFrame() {
    CHECK(frame_bundles_.size() > 0);
    return frame_bundles_.back()->at(0);
  }
  FrameBundlePtr& lastFrames() {
    CHECK(frame_bundles_.size() > 1);
    return frame_bundles_[frame_bundles_.size() - 2];
  }
  FrameBundlePtr& curFrames() {
    CHECK(frame_bundles_.size() > 0);
    return frame_bundles_.back();
  }

  // Keyframe selection criterion.
  bool needNewKeyFrame();

  // Detect features
  void detectFeatures(const FramePtr& frame);

  // Track featuers using LK optical flow
  void trackFeatures();

  // Optimize pose of new frame using tracked (and initialized) landmarks
  void optimizePose();

  // Check disparity between two frames
  double getDisparity(const FramePtr& ref_frame, 
                      const FramePtr& cur_frame);

  // Add observation to landmarks
  void addObservation(const FramePtr& frame);

  // Set the new frame as keyframe
  void setNewKeyFrame();

  // Initialize landmarks 
  void initializeNewLandmarks();

  // Initialize scale and landmarks at first
  bool initialize();

  // Processes frame bundle
  bool processFrameBundle();

  // Processes frames
  bool processFrame();

protected:
  // Options
  FeatureHandlerOptions options_;

  // Camera model
  CameraBundlePtr cams_;

  // Frames
  std::deque<FrameBundlePtr> frame_bundles_;
  size_t frame_counter_ = 0;
  bool initialized_;
  bool global_scale_initialized_;

  // IMU
  ImuMeasurements imu_measurements_;
  std::mutex imu_mutex_;
  double speed_and_bias_timestamp_;
  SpeedAndBias speed_and_bias_;

  // Modules
  DetectorPtr detector_;
  FeatureTrackerPtr tracker_;
  AbstractVisualInitialization::UniquePtr initializer_;

  // Map that handles keyframes and keypoints
  MapPtr map_;
};

}