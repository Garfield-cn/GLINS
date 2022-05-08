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
#include "gici/vision/matcher.h"
#include "gici/vision/tracker.h"

namespace gici {

// SDGNSS options
struct FeatureHandlerOptions {
  // Max number of keyframes to keep
  size_t max_n_kfs = 10;

  // Max features per frame
  size_t max_features_per_frame = 120;

  // If we have less than this amount of features we always select a new keyframe.
  size_t kfselect_numkfs_lower_thresh = 80;

  // Keyframe selection for FORWARD : Minimum distance in meters (set initial
  // scale!) before a new keyframe is selected.
  double kfselect_min_dist_metric = 0.5;

  // Minimum angle in degrees to closest KF
  double kfselect_min_angle = 5.0;

  // Minimum disparity to select a new keyframe
  double kfselect_min_disparity = -1;

  // Image max pyramid level
  int max_pyramid_level = 4;

  // Minimum disparity to initialize a new landmark (for triangulation)
  double min_disparity_new_landmark = 5.0;

  // Feature detector options
  DetectorOptions detector;

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

  // Add images with current pose. The pose can help improve the accuracy and efficiency
  // of feature tracking. If you cannot get a good pose estimate, try to ensure the precision
  // of the relative pose between current and last frame.
  bool addImageBundle(const std::vector<cv::Mat>& imgs, 
                      const double timestamp,
                      const Transformation& T_WS);

  // Add images with current pose and its covariance. We will consider the uncertainty of 
  // the given pose, i.e. enlarge the search area. This will increase the reliability.
  bool addImageBundle(const std::vector<cv::Mat>& imgs, 
                      const double timestamp,
                      const Transformation& T_WS,
                      const Eigen::Matrix<double, 6, 6>& T_WS_covariance);

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

protected:
  // Pose measurement
  struct Pose {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Pose() {}
    Pose(const Transformation T_WS_in) {
      T_WS = T_WS_in; has_pose = true;
    }
    Pose(const Transformation T_WS_in, 
         const Eigen::Matrix<double, 6, 6>& T_WS_covariance_in) {
      T_WS = T_WS_in; has_pose = true;
      T_WS_covariance = T_WS_covariance_in; 
      has_covariance = true;
    }

    Transformation T_WS;
    Eigen::Matrix<double, 6, 6> T_WS_covariance;
    bool has_pose = false;
    bool has_covariance = false;
  };

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

  // Get pose
  Pose& lastPose() {
    CHECK(poses_.size() > 1);
    return poses_[poses_.size() - 2];
  }
  Pose& curPose() {
    CHECK(poses_.size() > 0);
    return poses_.back();
  }

  // Keyframe selection criterion.
  bool needNewKeyFrame();

  // Detect features
  void detectFeatures(const FramePtr& frame);

  // Check disparity between two frames
  double getDisparity(const FramePtr& ref_frame, 
                      const FramePtr& cur_frame);

  // Add observation to landmarks
  void addObservation(const FramePtr& frame);

  // Set the new frame as keyframe
  void setNewKeyFrame();

  // Initialize landmarks in new keyframe
  void initializeNewKeyFrame();

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
  std::deque<Pose> poses_;
  bool first_keyframe_initialized_;

  // Feature detector
  DetectorPtr detector_;

  // Map that handles keyframes and keypoints
  MapPtr map_;
  std::vector<std::vector<FramePtr>> overlap_kfs_;
};

}