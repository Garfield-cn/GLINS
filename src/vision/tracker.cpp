/**
* @Function: Feature tracking using LK optical flow
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/vision/tracker.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

namespace gici {

// Predict initial point for optical flow tracking
static void predictFeatureTracking(const FramePtr& ref_frame,
    const FramePtr& cur_frame,
    const std::vector<cv::Point2f>& ref_points,
    std::vector<cv::Point2f>& pre_points)
{
  Transformation T_cur_ref = cur_frame->T_cam_world() * ref_frame->T_world_cam();
  Eigen::Matrix3d R_cur_ref = T_cur_ref.getRotationMatrix();
  
  if (pre_points.size() != ref_points.size()) pre_points.resize(ref_points.size());
  for (size_t i = 0; i < ref_points.size(); i++) {
    Eigen::Vector2d px(ref_points[i].x, ref_points[i].y);
    BearingVector ref_bearing;
    ref_frame->cam()->backProject3(px, &ref_bearing);
    BearingVector pre_bearing = R_cur_ref * ref_bearing;
    Eigen::Vector2d px_pre;
    cur_frame->cam()->project3(pre_bearing, &px_pre);
    pre_points[i] = cv::Point2f(px_pre(0), px_pre(1));
  }
}

// Track features by LK optical flow
void trackFeaturesPyrLK(const FramePtr& ref_frame,
      const FramePtr& cur_frame, OccupandyGrid2D& grid,
      bool use_pose_prediction)
{
  // Apply LK optical flow
  std::vector<cv::Point2f> ref_points;
  std::vector<cv::Point2f> cur_points;
  for (size_t i = 0; i < ref_frame->num_features_; i++) {
    ref_points.push_back(cv::Point2f(
      ref_frame->px_vec_.col(i)[0], ref_frame->px_vec_.col(i)[1]));
  }
  
  std::vector<unsigned char> status;
  if (use_pose_prediction) {
    // Predict feature pixel using relative orientation
    predictFeatureTracking(ref_frame, cur_frame, ref_points, cur_points);

    cv::calcOpticalFlowPyrLK(ref_frame->img_pyr_[0], cur_frame->img_pyr_[0],
        ref_points, cur_points, status, cv::noArray(), cv::Size(21, 21), 3, 
        cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);
  }
  else {
    cv::calcOpticalFlowPyrLK(ref_frame->img_pyr_[0], cur_frame->img_pyr_[0],
        ref_points, cur_points, status, cv::noArray());
  }

  // Apply RANSAC
  std::vector<cv::Point2f> ref_points_undist;
  std::vector<cv::Point2f> cur_points_undist;
  std::unordered_map<int, int> index_map;
  int num_tracked = 0;
  for (size_t i = 0; i < ref_frame->num_features_; i++) {
    if (!status[i]) continue;

    // remove out bound features
    if ((cur_points[i].x < 0 || cur_points[i].x > ref_frame->cam()->imageWidth()) ||
        (cur_points[i].y < 0 || cur_points[i].y > ref_frame->cam()->imageHeight())) {
      status[i] = 0; continue;
    }

    BearingVector bearing;
    cv::Point2f point;
    Eigen::Vector2d px = Eigen::Vector2d(ref_points[i].x, ref_points[i].y);
    ref_frame->cam()->backProject3(px, &bearing);
    point.x = ref_frame->cam()->getIntrinsicParameters()(0) *
      bearing(0) / bearing(2) + ref_frame->cam()->imageWidth() / 2.0;
    point.y = ref_frame->cam()->getIntrinsicParameters()(0) *
      bearing(1) / bearing(2) + ref_frame->cam()->imageHeight() / 2.0;
    ref_points_undist.push_back(point);

    px = Eigen::Vector2d(cur_points[i].x, cur_points[i].y);
    cur_frame->cam()->backProject3(px, &bearing);
    point.x = ref_frame->cam()->getIntrinsicParameters()(0) *
      bearing(0) / bearing(2) + ref_frame->cam()->imageWidth() / 2.0;
    point.y = ref_frame->cam()->getIntrinsicParameters()(0) *
      bearing(1) / bearing(2) + ref_frame->cam()->imageHeight() / 2.0;
    cur_points_undist.push_back(point);

    index_map.insert(std::make_pair(num_tracked, i));
    num_tracked++;
  }
  status.clear();
  cv::findFundamentalMat(ref_points_undist, cur_points_undist, 
    cv::FM_RANSAC, 1, 0.99, status);

  // Put valid features in current frame
  const cv::Mat& mask = cur_frame->cam()->getMask();
  cur_frame->resizeFeatureStorage(num_tracked);
  for (int i = 0; i < num_tracked; i++) {
    if (!status[i]) continue;

    int j = index_map.at(i);

    if (ref_frame->type_vec_[j] == FeatureType::kOutlier) continue;

    if(!mask.empty() && mask.at<uint8_t>(
      static_cast<int>(cur_points[j].y), static_cast<int>(cur_points[j].x)) == 0) {
      continue;
    }

    size_t grid_index = grid.getCellIndex(cur_points[j].x,cur_points[j].y, 1);
    if (!grid.isOccupied(grid_index)) {
      cur_frame->px_vec_.col(cur_frame->num_features_) = 
        Eigen::Vector2d(cur_points[j].x, cur_points[j].y);
      cur_frame->track_id_vec_[cur_frame->num_features_] = ref_frame->track_id_vec_[j];
      cur_frame->grad_vec_.col(cur_frame->num_features_) = ref_frame->grad_vec_.col(j);
      cur_frame->score_vec_[cur_frame->num_features_] = ref_frame->score_vec_[j];
      cur_frame->level_vec_[cur_frame->num_features_] = ref_frame->level_vec_[j];
      cur_frame->type_vec_[cur_frame->num_features_] = ref_frame->type_vec_[j];
      cur_frame->num_features_++;
      grid.setOccupied(grid_index);
    }
  }
  frame_utils::computeNormalizedBearingVectors(
    cur_frame->px_vec_, *cur_frame->cam(), &cur_frame->f_vec_);
}

}