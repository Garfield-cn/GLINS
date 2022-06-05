/**
* @Function: Feature detecting and tracking
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/vision/feature_handler.h"

#include "gici/utility/common.h"
#include "gici/estimate/graph.h"
#include "gici/estimate/estimator_types.h"
#include "gici/estimate/pose_parameter_block.h"
#include "gici/estimate/homogeneous_point_parameter_block.h"
#include "gici/estimate/pose_error.h"
#include "gici/vision/reprojection_error.h"

namespace gici {

// The default constructor
FeatureHandler::FeatureHandler(const FeatureHandlerOptions& options) :
  options_(options), cams_(options.cameras), map_(new Map()),
  initialized_(false), speed_and_bias_timestamp_(0.0), 
  speed_and_bias_(SpeedAndBias::Zero()), global_scale_initialized_(false)
{
  // initialize modules
  detector_ = feature_detection_utils::makeDetector(
    options.detector, cams_->getCameraShared(0));
  tracker_ = std::make_shared<FeatureTracker>(options.tracker);
  initializer_ = makeVisualInitializer(options.initialization, detector_, tracker_);

  frame_bundles_.push_back(std::make_shared<FrameBundle>(std::vector<FramePtr>()));
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
  
  // Add IMU measurements
  if (!isFirstFrame()) 
  {
    imu_mutex_.lock();
    ImuMeasurements::iterator imu_measurement = imu_measurements_.begin();
    ImuMeasurements::iterator imu_measurements_end = imu_measurements_.end();
    imu_mutex_.unlock();
    ImuMeasurements::iterator next_imu_measurement;
    bool start_fill = false, end_fill = false;
    double last_timestamp = lastFrames()->getMinTimestampSeconds();
    double keep_imu_front_timestamp;
    for (; imu_measurement != imu_measurements_end; imu_measurement++) {
      next_imu_measurement = imu_measurement; next_imu_measurement++;
      if (next_imu_measurement == imu_measurements_end && !start_fill) break;
      if (next_imu_measurement->timestamp >= last_timestamp && 
          imu_measurement->timestamp < last_timestamp) {
        // start add imu
        start_fill = true;
      }
      if (start_fill) {
        size_t k = frame_bundle->imu_timestamps_ns_.cols();
        frame_bundle->imu_timestamps_ns_.conservativeResize(Eigen::NoChange, k + 1);
        frame_bundle->imu_measurements_.conservativeResize(Eigen::NoChange, k + 1);
        frame_bundle->imu_timestamps_ns_.col(k)(0) = 
          static_cast<int64_t>(imu_measurement->timestamp * 1.0e9);
        frame_bundle->imu_measurements_.col(k).segment<3>(0) = 
          imu_measurement->linear_acceleration;
        frame_bundle->imu_measurements_.col(k).segment<3>(3) = 
          imu_measurement->angular_velocity;
      }
      if (end_fill) break;
      if (start_fill && next_imu_measurement != imu_measurements_end && 
          next_imu_measurement->timestamp > timestamp) {
        // add last imu
        end_fill = true;
        keep_imu_front_timestamp = imu_measurement->timestamp;
      }
    }
    // erase IMU measurements
    imu_mutex_.lock();
    while (imu_measurements_.front().timestamp < keep_imu_front_timestamp) {
      imu_measurements_.pop_front();
    }
    imu_mutex_.unlock();
  }

  // Add to pipeline.
  curFrames() = frame_bundle;
  ++frame_counter_;

  // Perform detecting and tracking
  bool ret = processFrameBundle();

  // Shift memory
  frame_bundles_.push_back(std::make_shared<FrameBundle>(std::vector<FramePtr>()));
  if (!initialized_) {
    while (frame_bundles_.size() > 2) {
      frame_bundles_.pop_front();
    }
  }
  else {
    while (frame_bundles_.front()->at(0)->id() < 
          map_->getOldsestKeyframe()->id()) {
      frame_bundles_.pop_front();
    }
  }

  return true;
}

// Add camera pose to a specific frame at given timestamp
bool FeatureHandler::addCameraPose(const double timestamp, 
                    const Transformation& T_WS)
{
  int index = -1;
  Transformation T_W_B_ref_store;
  for (int i = frame_bundles_.size() - 1; i >= 0; i--) {
    if (frame_bundles_[i]->frames_.size() == 0) continue;
    if (checkEqual(timestamp, frame_bundles_[i]->getMinTimestampSeconds())) {
      frame_bundles_[i]->set_T_W_B(T_WS);
      T_W_B_ref_store = frame_bundles_[i]->get_T_W_B();
      index = i;
    }
  }

  // adjust poses behind
  if (index != -1)
  {
    for (int i = index + 1; i < frame_bundles_.size(); i++) {
      if (frame_bundles_[i]->frames_.size() == 0) continue;
      Transformation T_W_B_cur = frame_bundles_[i]->get_T_W_B();
      Transformation T_cur_ref = T_W_B_cur.inverse() * T_W_B_ref_store;
      Transformation T_W_B_cur_adjusted = T_WS * T_cur_ref.inverse();
      frame_bundles_[i]->set_T_W_B(T_W_B_cur_adjusted);

      // check scale
      // if (i - 2 >= 0) {
      //   Transformation T_WS_l2 = frame_bundles_[i - 2]->get_T_W_B();
      //   Transformation T_WS_l1 = frame_bundles_[i - 1]->get_T_W_B();
      //   Transformation T_WS_0 = frame_bundles_[i]->get_T_W_B();
      //   double distance = T_WS_l1.getRotation().inverse().rotate(
      //     T_WS_l1.getPosition() - T_WS_l2.getPosition()).norm();
      //   double distance_0 = T_WS_0.getRotation().inverse().rotate(
      //     T_WS_0.getPosition() - T_WS_l1.getPosition()).norm();
      //   double scale = distance / distance_0;
      //   // large scale difference, rescale current frame
      //   if (fabs(scale - 1.0) > 0.05) {
      //     LOG(INFO) << "Tolerant of scale change exceeded (" << fabs(scale - 1.0) 
      //               << " vs 0.05)! Rescaling frame poses.";
      //     for (size_t k = 0; k < frame_bundles_[i]->frames_.size(); k++) {
      //       const FramePtr& frame = frame_bundles_[i]->at(k);
      //       const FramePtr& ref_frame = frame_bundles_[i - 1]->at(k);
      //       frame->T_f_w_.getPosition() = -frame->T_f_w_.getRotation().rotate(
      //         ref_frame->pos() + scale * (frame->pos() - ref_frame->pos()));
      //     }
      //   }
      // }
    }

    return true;
  }

  LOG(WARNING) << "Cannot find image bundle at timestamp " 
               << std::fixed << timestamp << "!";
  return false;
}

// After the poses and landmarks were adjusted to global, call this function to 
// apply the changes.
void FeatureHandler::setGlobalInitializationResult(
  const std::deque<FramePtr>& scaled_frames)
{
  // Erase all the unscaled frames and landmark observations
  // frames
  for (auto it = frame_bundles_.begin(); it != frame_bundles_.end(); )
  {
    if ((*it)->frames_.size() == 0) {
      it++; continue;
    }
    FramePtr frame = (*it)->at(0);

    bool found = false;
    for (auto scaled_frame : scaled_frames) {
      if (scaled_frame->id() == frame->id()) {
        found = true; break;
      }
    }
    if (found) {
      it++; continue;
    }
    it = frame_bundles_.erase(it);
  }

  // landmarks
  for (auto& frame : scaled_frames) {
    for (auto& landmark : frame->landmark_vec_) {
      if (landmark == nullptr) continue;
      for (auto it = landmark->obs_.begin(); it != landmark->obs_.end(); ) {
        if (FramePtr f = it->frame.lock()) {
          // valid observation
          it++;
        }
        else {
          it = landmark->obs_.erase(it);
        }
      }
    }
  }

  // Set flag
  global_scale_initialized_ = true;
}

// Add IMU measurement for pose integration
void FeatureHandler::addImuMeasurement(const ImuMeasurement& imu_measurement)
{
  if (imu_measurements_.size() != 0 && 
      imu_measurements_.back().timestamp > imu_measurement.timestamp) {
    LOG(WARNING) << "Received IMU with previous timestamp!";
  }
  else {
    imu_mutex_.lock();
    imu_measurements_.push_back(imu_measurement);
    imu_mutex_.unlock();
  }
}

// Set current speed and bias
void FeatureHandler::setSpeedAndBias(const double timestamp, 
                      const SpeedAndBias speed_and_bias)
{
  imu_mutex_.lock();
  speed_and_bias_timestamp_ = timestamp;
  speed_and_bias_ = speed_and_bias;
  imu_mutex_.unlock();
}

// Keyframe selection criterion.
bool FeatureHandler::needNewKeyFrame()
{
  // Get last keyframe
  const FramePtr keyframe = map_->getLastKeyframe();

  // Number of tracked features are too little
  std::vector<std::pair<size_t, size_t>> matches_ref_cur;
  getFeatureMatches(*keyframe, *curFrame(), &matches_ref_cur);
  size_t n_tracked_fts = matches_ref_cur.size();
  if (n_tracked_fts < options_.kfselect_min_numkfs) {
    LOG(INFO) << "Select new keyframe by number: " 
              << n_tracked_fts << " vs " << options_.kfselect_min_numkfs;
    return true;
  }

  // If global scale initialized, we use distance and angle matric.
  if (global_scale_initialized_)
  {
    const double a =
        Quaternion::log(curFrames()->at(0)->T_f_w_.getRotation() *
                        keyframe->T_f_w_.getRotation().inverse()).norm()
            * 180/M_PI;
    const double d = (curFrames()->at(0)->pos() - keyframe->pos()).norm();
    if (!(a < options_.kfselect_min_angle
        && d < options_.kfselect_min_dist_metric))
    {
      LOG(INFO) << "Select new keyframe by motivation: angle = " 
                << a << ", distance = " << d;
      return true;
    }
  }
  // Select keyframe with disparity
  else {
    double disparity = getDisparity(keyframe, curFrame());
    if (disparity > options_.kfselect_min_disparity) {
      LOG(INFO) << "Select new keyframe by disparity: " 
                << disparity << " vs " << options_.kfselect_min_disparity;
      return true;
    }
  }

  // Check time duration
  const double time_duration = 
    curFrame()->getTimestampSec() - keyframe->getTimestampSec();
  if (time_duration > options_.kfselect_min_dt) {
    LOG(INFO) << "Select new keyframe by time duration: " 
              << time_duration << " vs " << options_.kfselect_min_dt;
    return true;
  }

  // Not a new keyframe
  return false;
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
  if(max_n_features <= 0) return;

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
    frame->landmark_vec_[j]->addObservation(frame, j);

    frame->num_features_++;
  }
}

// Track featuers using LK optical flow
void FeatureHandler::trackFeatures()
{
  // Integrate attitude
  bool has_rotation_prior = false;
  Eigen::Quaterniond q_cur_ref;
  if (curFrames()->imu_timestamps_ns_.cols() > 0) {
    const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& imu_timestamps_ns =
        curFrames()->imu_timestamps_ns_;
    const Eigen::Matrix<double, 6, Eigen::Dynamic>& imu_measurements =
        curFrames()->imu_measurements_;
    Eigen::Vector3d gyro_bias = speed_and_bias_.segment<3>(3); 
    const size_t num_measurements = imu_timestamps_ns.cols();
    Quaternion delta_R;
    for (size_t m_idx = 0u; m_idx < num_measurements - 1u; ++m_idx)
    {
      const double delta_t_seconds =
          (imu_timestamps_ns(m_idx + 1) - imu_timestamps_ns(m_idx))
          * 1.0e-9;
      if (checkLessEqual(delta_t_seconds, 1e-12)) {
        LOG(FATAL) << "IMU timestamps need to be strictly increasing.";
      }

      const Eigen::Vector3d w = imu_measurements.col(m_idx).tail<3>() - gyro_bias;
      const Quaternion R_incr = Quaternion::exp(w * delta_t_seconds);
      delta_R = delta_R * R_incr;
    }
    q_cur_ref = delta_R.toImplementation();
    has_rotation_prior = true;
  }

  if (has_rotation_prior) {
    tracker_->track(
      lastFrame(), curFrame(), detector_->grid_, q_cur_ref);
  }
  else {
    tracker_->track(
      lastFrame(), curFrame(), detector_->grid_);
  }
}

// Optimize pose of new frame using tracked (and initialized) landmarks
void FeatureHandler::optimizePose()
{
  std::unique_ptr<Graph> graph = std::make_unique<Graph>();
  std::shared_ptr<ceres::LossFunction> loss_function = 
    std::make_shared<ceres::CauchyLoss>(1);
  FramePtr ref_frame = map_->getLastKeyframe();
  FramePtr frame = curFrame();

  // Approximate pose
  Transformation T_WS_aprox = ref_frame->T_world_imu();
  // not the just initialized frame
  if (!(map_->size() == 2 && lastFrame()->is_keyframe_)) {
    T_WS_aprox = lastFrames()->get_T_W_B() * 
      (frame_bundles_.at(frame_bundles_.size() - 3)->get_T_W_B().inverse() * 
      lastFrames()->get_T_W_B());
    T_WS_aprox.getRotation().normalize();
  }

  // Pose of current frame
  BackendId pose_id = createNFrameId(frame->bundleId());
  std::shared_ptr<PoseParameterBlock> pose_parameter_block = 
    std::make_shared<PoseParameterBlock>(T_WS_aprox, pose_id.asInteger());
  CHECK(graph->addParameterBlock(pose_parameter_block, Graph::Pose6d));

  // Extrinsics parameter (constant)
  const Transformation T_SC = ref_frame->T_imu_cam();
  BackendId extrinsics_id = changeIdType(pose_id, IdType::cExtrinsics, size_t(0));
  std::shared_ptr<PoseParameterBlock> extrinsics_parameter_block = 
    std::make_shared<PoseParameterBlock>(T_SC, extrinsics_id.asInteger());
  CHECK(graph->addParameterBlock(extrinsics_parameter_block, Graph::Pose6d));
  graph->setParameterBlockConstant(extrinsics_id.asInteger());

  // Landmarks
  int num_valid_features = 0;
  for (size_t kp_idx = 0; kp_idx < frame->numFeatures(); ++kp_idx)
  {
    // make sure the landmark has been initialized, and can be observed by 
    // both current frame and last keyframe
    const auto& landmark = frame->landmark_vec_[kp_idx];
    if (!isSeed(frame->type_vec_[kp_idx])) continue;
    int ref_obs_index = -1, obs_index = -1;
    for (size_t i = 0; i < landmark->obs_.size(); i++) {
      auto& obs = landmark->obs_[i];
      if (obs.frame_id == ref_frame->id()) ref_obs_index = obs.keypoint_index_;
      if (obs.frame_id == frame->id()) obs_index = obs.keypoint_index_;
    }
    if (obs_index == -1 || ref_obs_index == -1) continue;
    CHECK(kp_idx == obs_index);

    // Add landmark parameter block and set as constant. We do not optimize
    // landmark here to save time consumption. It will be optimized in the 
    // backend graph.
    BackendId landmark_id = createLandmarkId(landmark->id());
    std::shared_ptr<HomogeneousPointParameterBlock> landmark_parameter_block =
      std::make_shared<HomogeneousPointParameterBlock>(
        landmark->pos(), landmark_id.asInteger());
    CHECK(graph->addParameterBlock(
      landmark_parameter_block, Graph::HomogeneousPoint));
    graph->setParameterBlockConstant(landmark_id.asInteger());

    // Add reprojection errors
    Eigen::Matrix2d information = Eigen::Matrix2d::Identity();
    information *= 1.0 / 
      static_cast<double>(1 << frame->level_vec_(kp_idx));

    std::shared_ptr<ReprojectionError> reprojection_error =
        std::make_shared<ReprojectionError>(
          frame->cam(),
          frame->px_vec_.col(kp_idx), information);
    graph->addResidualBlock(
          reprojection_error,
          loss_function.get(),
          graph->parameterBlockPtr(pose_id.asInteger()),
          graph->parameterBlockPtr(landmark_id.asInteger()),
          graph->parameterBlockPtr(extrinsics_id.asInteger()));
    
    num_valid_features++;
  }

  // If insufficient features are tracked, we set pose as the approximate one.
  if (num_valid_features < 10) {
    LOG(WARNING) << "Insufficient tracked features to estimate "
    << "current pose! num_valid_features = " << num_valid_features;
    frame->set_T_w_imu(T_WS_aprox);
    return;
  }

  // Optimize
  graph->options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  graph->options.trust_region_strategy_type = ceres::DOGLEG;
  graph->options.max_num_iterations = 2;
  graph->solve();

  // Get solution
  std::shared_ptr<PoseParameterBlock> block_ptr =
      std::static_pointer_cast<PoseParameterBlock>(
        graph->parameterBlockPtr(pose_id.asInteger()));
  Transformation T_WS = block_ptr->estimate();
  frame->set_T_w_imu(T_WS);
}

// Check disparity between two frames
double FeatureHandler::getDisparity(
                    const FramePtr& ref_frame, 
                    const FramePtr& cur_frame)
{
  std::vector<std::pair<size_t, size_t>> matches_ref_cur;
  getFeatureMatches(*ref_frame, *cur_frame, &matches_ref_cur);

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
  FramePtr ref_frame = lastFrame();
  std::vector<std::pair<size_t, size_t>> matches;
  getFeatureMatches(*ref_frame, *frame, &matches);
  for (size_t i = 0; i < matches.size(); i++) {
    PointPtr& landmark = ref_frame->landmark_vec_[matches[i].first];
    CHECK(landmark != nullptr);

    frame->landmark_vec_[matches[i].second] = landmark;
    landmark->addObservation(frame, matches[i].second);
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

// Initialize landmarks
void FeatureHandler::initializeNewLandmarks()
{
  FramePtr frame = curFrame();

  // Initialize landmarks
  for (size_t i = 0; i < frame->numFeatures(); i++) {
    auto& landmark = frame->landmark_vec_[i];
    CHECK(landmark != nullptr);

    // already initialized
    if (isSeed(frame->type_vec_[i])) continue;

    // rejected by bundle adjuster
    if (frame->type_vec_[i] == FeatureType::kOutlier) continue;

    // get the first observation in window
    FramePtr ref_frame = nullptr;
    size_t index_ref = 0;
    for (auto obs : landmark->obs_) {
      if (FramePtr f = obs.frame.lock()) {
        ref_frame = f; 
        index_ref = obs.keypoint_index_;
        break;
      }
    }
    CHECK_NOTNULL(ref_frame);
    if (ref_frame->id() == frame->id()) continue;

    // check disparity
    double disparity = (ref_frame->px_vec_.col(index_ref) - 
      frame->px_vec_.col(i)).norm();
    if (disparity < options_.min_disparity_init_landmark) continue;

    Transformation T_cur_ref = frame->T_f_w_ * ref_frame->T_f_w_.inverse();
    BearingVector f_ref = ref_frame->f_vec_.col(index_ref);
    BearingVector f_cur = frame->f_vec_.col(i);
    double depth;
    if (!(depthFromTriangulation(T_cur_ref, f_ref, f_cur, &depth) 
        == FeatureMatcher::MatchResult::kSuccess)) continue;
    // Note that the follow equation should not be T * f_ref * depth.
    // Because the operater "*" is applied in homogeneous coordinate
    landmark->pos_ = ref_frame->T_world_cam() * (f_ref * depth);

    // Change feature type
    for (auto obs : landmark->obs_) {
      if (FramePtr f = obs.frame.lock()) {
        changeFeatureTypeToSeed(f->type_vec_[obs.keypoint_index_]);
      }
    }
  }
}

// Initialize scale and landmarks at first
bool FeatureHandler::initialize()
{
  VisualInitResult ret = initializer_->addFrameBundle(curFrames());
  if (ret == VisualInitResult::kTracking) {
    return false;
  }
  if (ret == VisualInitResult::kFailure) {
    initializer_->reset();
    return false;
  }

  // Set the first and the latest frames as keyframes
  initializer_->ref_frames_->setKeyframe();
  initializer_->ref_frames_->at(0)->setKeyframe();
  map_->addKeyframe(initializer_->ref_frames_->at(0), true);
  curFrames()->setKeyframe();
  curFrames()->at(0)->setKeyframe();
  map_->addKeyframe(curFrames()->at(0), true);

  return true;
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
  // Initialization
  if (!initialized_) {
    if (initialize()) {
      LOG(INFO) << "Feature handler initialized.";
      initialized_ = true; return true;
    }
    else return false;
  }

  // Reset grid
  detector_->grid_.reset();

  // Track features
  trackFeatures();

  // Optimize pose using tracked (and initialized) landmarks
  if (!curFrame()->has_transform_) {
    optimizePose();
  }

  // Detect features in new frame
  detectFeatures(curFrame());

  // initialize landmarks 
  initializeNewLandmarks();

  // Select keyframe
  if(!needNewKeyFrame()) return true;

  // set as keyframe
  setNewKeyFrame();

  return true;
}

}