/**
* @Function: Feature detecting and tracking
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/vision/feature_handler.h"

namespace gici {

// The default constructor
FeatureHandler::FeatureHandler(const FeatureHandlerOptions& options) :
  options_(options), cams_(options.cameras), map_(new Map()),
  first_keyframe_initialized_(false)
{
  // initialize modules
  detector_ = feature_detection_utils::makeDetector(
    options.detector, cams_->getCameraShared(0));
  overlap_kfs_.resize(cams_->getNumCameras());

  frame_bundles_.push_back(std::make_shared<FrameBundle>(std::vector<FramePtr>()));
  poses_.push_back(Pose());
}

// The default destructor
FeatureHandler::~FeatureHandler()
{}

// Add images
bool FeatureHandler::addImageBundle(
  const std::vector<cv::Mat>& imgs, const double timestamp)
{
  if (!isFirstFrame())
  {
    // check if the timestamp is valid
    if (lastFrames()->getMinTimestampSeconds() >= timestamp) {
      LOG(WARNING) << "Dropping frame: timestamp older than last frame of id " 
      << lastFrames()->getBundleId();
      return false;
    }
  }

  CHECK_EQ(imgs.size(), cams_->getNumCameras());
  std::vector<FramePtr> frames;
  for (size_t i = 0; i < imgs.size(); ++i)
  {
    frames.push_back(std::make_shared<Frame>(
      cams_->getCameraShared(i), imgs[i].clone(), 
      static_cast<int64_t>(timestamp * 1.0e9), options_.max_pyramid_level + 1));
    frames.back()->set_T_cam_imu(cams_->get_T_C_B(i));
    frames.back()->setNFrameIndex(i);
  }
  FrameBundlePtr frame_bundle(new FrameBundle(frames));

  // set pose on frame
  if (curPose().has_pose) {
    // In our implementation, B (Body) = S (Sensor) = I (IMU)
    frame_bundle->set_T_W_B(curPose().T_WS);
  }

  // Add to pipeline.
  curFrames() = frame_bundle;
  ++frame_counter_;

  // Perform detecting and tracking
  processFrameBundle();

  // Shift memory
  frame_bundles_.push_back(std::make_shared<FrameBundle>(std::vector<FramePtr>()));
  poses_.push_back(Pose());
  if (frame_bundles_.size() > options_.min_disparity_new_landmark * 2) {
    frame_bundles_.pop_front();
    poses_.pop_front();
  }

  return true;
}

bool FeatureHandler::addImageBundle(const std::vector<cv::Mat>& imgs, 
                    const double timestamp,
                    const Transformation& T_WS)
{
  curPose() = Pose(T_WS);
  return addImageBundle(imgs, timestamp);
}

bool FeatureHandler::addImageBundle(
                    const std::vector<cv::Mat>& imgs, 
                    const double timestamp,
                    const Transformation& T_WS,
                    const Eigen::Matrix<double, 6, 6>& T_WS_covariance)
{
  curPose() = Pose(T_WS, T_WS_covariance);
  return addImageBundle(imgs, timestamp);
}

// Keyframe selection criterion.
bool FeatureHandler::needNewKeyFrame()
{
  const std::vector<FramePtr>& visible_kfs = overlap_kfs_.at(0);

  size_t n_tracked_fts = curFrames()->numFeatures();

  // Number of tracked features are too little
  if (n_tracked_fts < options_.kfselect_numkfs_lower_thresh)
  {
    VLOG(40) << "KF Select: NEW KEYFRAME Below lower bound";
    return true;
  }

  // check that we have at least X disparity w.r.t to last keyframe
  if (options_.kfselect_min_disparity > 0)
  {
    FramePtr last_keyframe = map_->getLastKeyframe();
    double disparity = getDisparity(last_keyframe, curFrame());

    if (disparity < options_.kfselect_min_disparity) {
        VLOG(40) << "KF Select: NO NEW KEYFRAME disparity not large enough";
        return false;
    }
  }

  for (const auto& kf : visible_kfs)
  {
    // TODO: doesn't generalize to rig!
    const double a =
        Quaternion::log(curFrames()->at(0)->T_f_w_.getRotation() *
                        kf->T_f_w_.getRotation().inverse()).norm()
            * 180/M_PI;
    const double d = (curFrames()->at(0)->pos() - kf->pos()).norm();
    if (a < options_.kfselect_min_angle
        && d < options_.kfselect_min_dist_metric)
    {
      VLOG(40) << "KF Select: NO NEW KEYFRAME Min angle = " << a
               << ", min dist = " << d;
      return false;
    }
  }
  VLOG(40) << "KF Select: NEW KEYFRAME";
  return true;
}

// Detect features
void FeatureHandler::detectFeatures(const FramePtr& frame)
{
  // Detect new features.
  Keypoints new_px;
  Scores new_scores;
  Levels new_levels;
  Gradients new_grads;
  FeatureTypes new_types;
  Bearings new_f;

  const int max_n_features = options_.max_features_per_frame - frame->numFeatures();
  if(max_n_features <= 0)
  {
    VLOG(3) << "Skip seed initialization. Have already enough features.";
    return;
  }

  detector_->detect(
        frame->img_pyr_, frame->getMask(), max_n_features, new_px, new_scores,
        new_levels, new_grads, new_types);
  frame_utils::computeNormalizedBearingVectors(new_px, *frame->cam(), &new_f);

  // Add features to frame.
  const size_t n_old = frame->num_features_;
  const size_t n_new = new_px.cols();
  frame->resizeFeatureStorage(n_old + n_new);
  for(size_t i = 0, j = n_old; i < n_new; ++i, ++j) {
    frame->px_vec_.col(j) = new_px.col(i);
    frame->f_vec_.col(j) = new_f.col(i);
    frame->grad_vec_.col(j) = new_grads.col(i);
    frame->score_vec_(j) = new_scores(i);
    frame->level_vec_(j) = new_levels(i);

    frame->landmark_vec_[j] = std::make_shared<Point>(Eigen::Vector3d::Zero());
    frame->track_id_vec_(j) = frame->landmark_vec_[j]->id();

    // add observation to landmark
    frame->landmark_vec_[j]->addObservation(frame, j);

    if(new_types[i] == FeatureType::kCorner)
      frame->type_vec_[j] = FeatureType::kCornerSeed;
    else if(new_types[i] == FeatureType::kEdgelet)
      frame->type_vec_[j] = FeatureType::kEdgeletSeed;
    else if(new_types[i] == FeatureType::kMapPoint)
      frame->type_vec_[j] = FeatureType::kMapPointSeed;
    else
      LOG(FATAL) << "Unknown feature types.";

    frame->num_features_++;
  }
}

// Check disparity between two frames
double FeatureHandler::getDisparity(
                    const FramePtr& ref_frame, 
                    const FramePtr& cur_frame)
{
  std::vector<std::pair<size_t, size_t>> matches_ref_cur;
  feature_tracking_utils::getFeatureMatches(*ref_frame, *cur_frame, &matches_ref_cur);

  std::vector<double> disparities;
  for (size_t i = 0; i < matches_ref_cur.size(); i++) {
    double disparity = (ref_frame->px_vec_.col(matches_ref_cur[i].first) - 
      cur_frame->px_vec_.col(matches_ref_cur[i].second)).norm();
    disparities.push_back(disparity);
  }

  if (!disparities.empty()) return vk::getMedian(disparities);
  else return 0.0;
}

// Add observation to landmarks
void FeatureHandler::addObservation(const FramePtr& frame)
{
  if (frame->isKeyframe()) {
    // We already added observation during feature detection
    return;
  }
  else {
    FramePtr keyframe = map_->getLastKeyframe();
    std::vector<std::pair<size_t, size_t>> matches;
    feature_tracking_utils::getFeatureMatches(*keyframe, *frame, &matches);
    for (size_t i = 0; i < matches.size(); i++) {
      PointPtr& landmark = keyframe->landmark_vec_[matches[i].first];
      CHECK(landmark != nullptr);

      frame->landmark_vec_[matches[i].second] = landmark;
      landmark->addObservation(frame, matches[i].second);
    }
  }
}

// Set the new frame as keyframe
void FeatureHandler::setNewKeyFrame()
{
  // set as keyframe
  curFrame()->setKeyframe();
  curFrames()->setKeyframe();

  // add keyframe to map
  map_->addKeyframe(curFrame(), true);

  // if limited number of keyframes, remove the one furthest apart
  if(options_.max_n_kfs > 2)
  {
    while(map_->size() > options_.max_n_kfs)
    {
      map_->removeOldestKeyframe();
    }
  }
}

// Initialize landmarks in new keyframe
void FeatureHandler::initializeNewKeyFrame()
{
  FramePtr keyframe = map_->getLastKeyframe();
  FramePtr frame = curFrame();
  
  // check disparity
  double disparity = getDisparity(keyframe, frame);
  if (disparity < options_.min_disparity_new_landmark) {
    VLOG(10) << "Not sufficient disparity to initialize landmarks.";
    return;
  }

  // Initialize landmarks
  std::vector<std::pair<size_t, size_t>> matches;
  feature_tracking_utils::getFeatureMatches(*keyframe, *frame, &matches);
  for (size_t i = 0; i < matches.size(); i++) {
    PointPtr& landmark = keyframe->landmark_vec_[matches[i].first];
    CHECK(landmark != nullptr);

    // already initialized
    if (landmark->pos_ != Eigen::Vector3d::Zero()) continue;

    // rejected by bundle adjuster
    if (keyframe->type_vec_[matches[i].first] == FeatureType::kOutlier) continue;

    Transformation T_cur_ref = frame->T_f_w_ * keyframe->T_f_w_.inverse();
    BearingVector f_ref = keyframe->f_vec_.col(matches[i].first);
    BearingVector f_cur = frame->f_vec_.col(matches[i].second);
    double depth;
    matcher_utils::depthFromTriangulation(T_cur_ref, f_ref, f_cur, &depth);
    // Note that the follow equation should not be T * f_ref * depth.
    // Because the operater "*" is applied in homogeneous coordinate
    landmark->pos_ = keyframe->T_world_cam() * (f_ref * depth);

    // Optimize landmark to get a better prior
    // landmark->optimize(5, false);
  }

  // set flag
  first_keyframe_initialized_ = true;
}

// Processes frame bundle
bool FeatureHandler::processFrameBundle()
{
  // currently we only support one camera
  return processFrame();
}

// Processes frames
bool FeatureHandler::processFrame()
{
  // First frame
  if (isFirstFrame()) {
    detectFeatures(curFrame());
    setNewKeyFrame();
    return true;
  }

  // compute overlap keyframes
  for (size_t camera_idx = 0; camera_idx < cams_->numCameras(); ++camera_idx)
  {
    overlap_kfs_.at(camera_idx).clear();
    if (first_keyframe_initialized_) {
      map_->getClosestNKeyframesWithOverlap(
            curFrames()->at(camera_idx),
            options_.max_n_kfs,
            &overlap_kfs_.at(camera_idx));
    }
    else {
      overlap_kfs_.at(camera_idx).push_back(map_->getLastKeyframe());
    }
  }

  // Reset grid
  detector_->grid_.reset();

  // Track features
  bool use_pose_prediction = false;
  if (curPose().has_pose && lastPose().has_pose) {
    use_pose_prediction = true;
  }
  trackFeaturesPyrLK(
    lastFrame(), curFrame(), detector_->grid_, use_pose_prediction);

  // Update features to landmark
  addObservation(curFrame());

  // initialize landmarks in lastest keyframe
  initializeNewKeyFrame();

  // Select keyframe
  if(!needNewKeyFrame()) return true;

  // set as keyframe
  setNewKeyFrame();

  // Detect features in new keyframe
  detectFeatures(curFrame());

  return true;
}

}