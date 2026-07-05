/**
 * @Function: KD-tree management
 *
 * @Author  : Jiahui Liu
 * @Email   : jh.liu@sjtu.edu.cn
 **/
#include "gici/lidar/tree_handler.h"

#include "gici/utility/common.h"
#include "gici/estimate/graph.h"
#include "gici/estimate/estimator_types.h"
#include "gici/estimate/pose_parameter_block.h"
#include "gici/estimate/homogeneous_point_parameter_block.h"
#include "gici/estimate/pose_error.h"
#include "gici/imu/imu_error.h"

namespace gici {
// The default constructor
TreeHandler::TreeHandler(const TreeHandlerOptions& options,
                         const ImuEstimatorBaseOptions& imu_options, const Transformation& T_B_L) :
  options_(options),
  speed_and_bias_timestamp_(0.0),
  speed_and_bias_(SpeedAndBias::Zero())
{
  // Set LiDAR extrinsics
  set_T_BL(T_B_L);
  set_T_BL_MAT();

  // Allocating memory
  laser_map_ptr_.reset(new Cloud());
  plane_ptr_.reset(new PlaneCloud());
  plane_map_ptr_.reset(new PlaneCloud());
  landmark_map_ptr_.reset(new PlaneCloud());
  current_new_landmarks_.reset(new PlaneCloud());
  map_tree_.reset(new Kd_Tree<Point_lidar>(options));
  plane_tree_.reset(new Kd_Tree<Plane>(options));
  landmark_tree_.reset(new Kd_Tree<Plane>(options));

  residual_cloud_.reset(new Cloud());

  // Backend parameters
  backend_landmark_tree_.reset(new Kd_Tree<Plane>(options));
}

// The default destructor
TreeHandler::~TreeHandler()
{
  clearVoxelMap();
}

void TreeHandler::clearVoxelMap()
{
  for (auto& voxel : voxel_map_) {
    delete voxel.second;
  }
  voxel_map_.clear();
}

bool TreeHandler::processLidar(std::shared_ptr<LidarMeasurement>& scan)
{
  // Downsample point cloud
  Cloud_ptr cloud_full_down = filterCloud(scan->cloud_ptr, options_.filter_radius);

  // Calculate covariance of each point
  calculateCov(cloud_full_down, options_.std_range, options_.std_angle);

  scan->cloud_ptr = cloud_full_down;

  current_new_landmarks_->clear();
  return true;
}

Cloud_ptr TreeHandler::filterCloud(Cloud_ptr cloud_in, double filter_radius)
{
  Cloud_ptr cloud_out(new Cloud());
  static pcl::VoxelGrid<Point_lidar> filter;
  filter.setInputCloud(cloud_in);
  filter.setLeafSize(filter_radius, filter_radius, filter_radius);
  filter.filter(*cloud_out);

  return cloud_out;
}

void TreeHandler::calculateCov(Cloud_ptr cloud, double std_range, double std_angle)
{
  double range_var = std_range * std_range;
  double angle_var = std::pow(std::sin(DEG2RAD(std_angle)), 2);

  Eigen::Matrix2d direction_var = Eigen::Matrix2d::Identity() * angle_var;

  for (size_t i = 0; i < cloud->points.size(); ++i) {
    auto& point = cloud->points[i];

    Eigen::Vector3d p(point.x, point.y, point.z);
    double range = p.norm();

    Eigen::Vector3d angle = p / range;

    Eigen::Vector3d base_vector1(1, 1, -(angle.x() + angle.y()) / angle.z());
    base_vector1.normalize();
    Eigen::Vector3d base_vector2 = base_vector1.cross(angle);
    base_vector2.normalize();

    Eigen::Matrix<double, 3, 2> N;
    N << base_vector1, base_vector2;
    Eigen::Matrix<double, 3, 2> A = range * skewSymmetric(angle) * N;

    Eigen::Matrix3d cov_matrix =
        angle * range_var * angle.transpose() + A * direction_var * A.transpose();

    for (int row = 0, k = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col, ++k) {
        point.covariance[k] = cov_matrix(row, col);
      }
    }
  }
}

bool TreeHandler::treeInit()
{
  if (!tree_init_) {
    return false;
  } else
    return true;
}

void TreeHandler::mapBuild(Cloud_ptr cloud)
{
  mutex_.lock();

  map_tree_->treeBuild(cloud);
  laser_map_ptr_.reset(new Cloud());
  *laser_map_ptr_ += *cloud;

  tree_init_ = true;

  mutex_.unlock();

  CHECK_EQ(laser_map_ptr_->size(), getMapSize());
}

void TreeHandler::mapGrow(Cloud_ptr input_scan)
{
  mutex_.lock();

  map_tree_->treeGrow(input_scan);
  *laser_map_ptr_ += *input_scan;

  mutex_.unlock();
}

bool TreeHandler::keypointExtract(ScanPtr& scan, const Transformation T_WS)
{
  Cloud_ptr cloud_w(new Cloud);

  Eigen::Vector4d plane_params;

  // Transform current scan to world frame
  pcl::transformPointCloud(*scan->cloud_ptr, *cloud_w, transTomat(T_WS * T_BL_), true);

  // Search keypoint
  size_t cnt_search = 0;
  plane_ptr_->clear();
  mutex_.lock();

  for (size_t idx = 0; idx < cloud_w->size(); idx++) {
    if (map_tree_->search(cloud_w->points[idx], plane_params)) {
      // Save keypoint index
      cnt_search += 1;
      scan->feature_indices.push_back(idx);

      // Save normal vector for backend
      scan->cloud_ptr->points[idx].normal_x = plane_params[0];
      scan->cloud_ptr->points[idx].normal_y = plane_params[1];
      scan->cloud_ptr->points[idx].normal_z = plane_params[2];
      scan->cloud_ptr->points[idx].d = plane_params[3];

      // Save normal for frontend association
      cloud_w->points[idx].normal_x = plane_params[0];
      cloud_w->points[idx].normal_y = plane_params[1];
      cloud_w->points[idx].normal_z = plane_params[2];
      cloud_w->points[idx].d = plane_params[3];

      // Transfer to plane
      Plane plane(cloud_w->points[idx]);
      // Copy Cartesian point covariance to the plane candidate
      std::copy(std::begin(scan->cloud_ptr->points[idx].covariance),
                std::end(scan->cloud_ptr->points[idx].covariance), plane.covariance);
      plane_ptr_->push_back(plane);
    }

    else
      continue;
  }
  mutex_.unlock();

  if (cnt_search == 0)
    return false;
  else
    return true;
}

void TreeHandler::planeGrow(ScanPtr& scan)
{
  CHECK(plane_ptr_->size() > 0);

  // Initialize plane tree
  if (plane_tree_->isEmpty()) {
    PlaneCloud_ptr cloud_init(new PlaneCloud());
    cloud_init->push_back(plane_ptr_->points[0]);
    cloud_init->push_back(plane_ptr_->points[1]);
    plane_tree_->treeBuild(cloud_init);
  }

  std::vector<Plane, Eigen::aligned_allocator<Plane>> points_near;
  std::vector<float> distances(3);

  for (size_t i = 0; i < plane_ptr_->size(); i++) {
    auto& point = plane_ptr_->points[i];

    // Search near points
    points_near.clear();
    distances.clear();
    if (!plane_tree_->search(point, points_near, distances)) {
      // No near points, add to tree
      plane_tree_->treeGrow(point);
      plane_map_ptr_->points.push_back(point);
    } else {
      // Find the most similar point based on normal_dot
      double max_normal_dot = -1.0;  // Initialize with an invalid value
      Plane* best_match = nullptr;

      for (auto& near_point : points_near) {
        // Compute the normal vector similarity
        double normal_dot = normalDot(point, near_point);

        if (normal_dot > max_normal_dot) {
          max_normal_dot = normal_dot;
          best_match = &near_point;
        }
      }

      // If the most similar point is above the threshold, merge it
      if (best_match && max_normal_dot > 0.9) {
        size_t idx = scan->feature_indices[i];
        if (best_match->sub_id != 0) {
          scan->correspondences.emplace_back(idx, best_match->sub_id);
        } else if (best_match->id != 0) {
          scan->correspondences.emplace_back(idx, best_match->id);
        } else {
          // Merge planes
          mergePlanes(*best_match, point);
          updatePlane(*best_match);
        }
      } else {
        // Otherwise, grow the tree with the current point
        plane_tree_->treeGrow(point);
        plane_map_ptr_->points.push_back(point);
      }
    }
  }
}

void TreeHandler::mergePlanes(Plane& current, const Plane& target)
{
  Eigen::Map<const Eigen::Matrix3d> target_cov(target.covariance);
  Eigen::Map<const Eigen::Matrix3d> current_cov(current.covariance);

  Eigen::Matrix3d target_cov_inv = target_cov.inverse();
  Eigen::Matrix3d current_cov_inv = current_cov.inverse();

  Eigen::Matrix3d weight = (target_cov_inv + current_cov_inv).inverse();

  // Fuse normal vectors
  Eigen::Vector3d current_normal(current.normal_x, current.normal_y, current.normal_z);
  Eigen::Vector3d target_normal(target.normal_x, target.normal_y, target.normal_z);

  Eigen::Vector3d fused_normal =
      weight * (target_cov_inv * target_normal + current_cov_inv * current_normal);
  fused_normal.normalize();

  current.normal_x = fused_normal.x();
  current.normal_y = fused_normal.y();
  current.normal_z = fused_normal.z();

  // Update plane count
  current.count += 1;

  // Update covariance matrix
  Eigen::Matrix3d fused_cov = weight;
  std::copy(fused_cov.data(), fused_cov.data() + 9, current.covariance);

  if (current.count == options_.merge_size) {
    // Initialize landmark tree
    if (landmark_tree_->isEmpty()) {
      current.id = ++id_;
      PlaneCloud_ptr cloud_init(new PlaneCloud());
      cloud_init->push_back(current);
      landmark_tree_->treeBuild(cloud_init);
      landmark_map_ptr_->points.push_back(current);
      current_new_landmarks_->points.push_back(current);
    } else {
      Plane nearest_landmark;
      if (landmark_tree_->search(current, nearest_landmark)) {
        if (normalDot(current, nearest_landmark) > 0.9) {
          // Merge planes
          current.sub_id = nearest_landmark.id;
          return;
        }
      }

      current.id = ++id_;
      landmark_tree_->treeGrow(current);
      landmark_map_ptr_->points.push_back(current);
      current_new_landmarks_->points.push_back(current);
    }
  }
}

void TreeHandler::updatePlane(Plane& plane)
{
  plane_map_ptr_->points.push_back(plane);
  plane_tree_->treeUpdate(plane);
}

void TreeHandler::landmarkTreeBuild(PlaneCloud_ptr cloud)
{
  backend_landmark_tree_->treeBuild(cloud);
}

void TreeHandler::landmarkTreeGrow(PlaneCloud_ptr cloud)
{
  backend_landmark_tree_->treeGrow(cloud);
}

std::vector<pointWithVar> TreeHandler::convertCloudtoVec(const Cloud_ptr& cloud_w)
{
  std::vector<pointWithVar> points;
  points.reserve(cloud_w->points.size());
  for (size_t i = 0; i < cloud_w->points.size(); i++) {
    pointWithVar pv;
    const auto& pt = cloud_w->points[i];
    pv.point_w << pt.x, pt.y, pt.z;
    pv.var = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(pt.covariance);
    points.push_back(pv);
  }
  return points;
}

void TreeHandler::buildVoxelMap(Cloud_ptr cloud_w)
{
  double voxel_size = options_.voxel_size;
  int max_layer = options_.max_layer;
  int max_points_num = options_.max_points_num;
  int layer_init_num = options_.layer_init_num;
  double planner_threshold = options_.plane_threshold;

  std::vector<pointWithVar> input_points = convertCloudtoVec(cloud_w);

  mutex_.lock();
  clearVoxelMap();

  for (size_t i = 0; i < input_points.size(); i++) {
    const pointWithVar& p_v = input_points[i];
    float loc_xyz[3];
    convertVoxelKey(p_v.point_w, voxel_size, loc_xyz);

    VOXEL_LOCATION position(static_cast<int64_t>(loc_xyz[0]), static_cast<int64_t>(loc_xyz[1]),
                            static_cast<int64_t>(loc_xyz[2]));
    auto iter = voxel_map_.find(position);
    if (iter != voxel_map_.end()) {
      voxel_map_[position]->temp_points_.push_back(p_v);
      voxel_map_[position]->new_points_++;
    } else {
      VoxelOctoTree* octo_tree =
          new VoxelOctoTree(max_layer, 0, layer_init_num, max_points_num, planner_threshold);
      voxel_map_[position] = octo_tree;
      voxel_map_[position]->quater_length_ = voxel_size / 4.0;
      voxel_map_[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
      voxel_map_[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
      voxel_map_[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
      voxel_map_[position]->temp_points_.push_back(p_v);
      voxel_map_[position]->new_points_++;
      voxel_map_[position]->layer_init_num_ = layer_init_num;
    }
  }

  for (auto iter = voxel_map_.begin(); iter != voxel_map_.end(); ++iter) {
    iter->second->init_octo_tree();
  }
  mutex_.unlock();
}

bool TreeHandler::searchLandmarks(const Point_lidar& point_w, VoxelPlanePtr& plane_out,
                                  const ObsId& obs_id)
{
  Eigen::Vector3d point(point_w.x, point_w.y, point_w.z);
  float loc_xyz[3];
  convertVoxelKey(point, options_.voxel_size, loc_xyz);
  VOXEL_LOCATION position(static_cast<int64_t>(loc_xyz[0]), static_cast<int64_t>(loc_xyz[1]),
                          static_cast<int64_t>(loc_xyz[2]));

  mutex_.lock();

  auto iter = voxel_map_.find(position);
  if (iter == voxel_map_.end()) {
    mutex_.unlock();
    return false;
  }

  VoxelOctoTree* current_octo = iter->second;

  if (current_octo->plane_ptr_->is_landmark_) {
    if (checkPlane(current_octo, point_w, 3)) {
      current_octo->plane_ptr_->obs_.push_back(obs_id);
      plane_out = current_octo->plane_ptr_;
      mutex_.unlock();
      return true;
    }
  }
  mutex_.unlock();
  return false;
}

bool TreeHandler::searchVoxelMap(const Point_lidar& point_w, VoxelPlane& plane_out,
                                 const ObsId& obs_id)
{
  Eigen::Vector3d point(point_w.x, point_w.y, point_w.z);
  float loc_xyz[3];
  convertVoxelKey(point, options_.voxel_size, loc_xyz);
  VOXEL_LOCATION position(static_cast<int64_t>(loc_xyz[0]), static_cast<int64_t>(loc_xyz[1]),
                          static_cast<int64_t>(loc_xyz[2]));

  auto iter = voxel_map_.find(position);
  if (iter == voxel_map_.end()) {
    return false;
  }

  VoxelOctoTree* current_octo = iter->second;
  return searchVoxelRecursive(point_w, plane_out, current_octo, 0, obs_id);
}

bool TreeHandler::searchVoxelRecursive(const Point_lidar& point_w, VoxelPlane& plane_out,
                                       VoxelOctoTree* current_octo, int layer,
                                       const ObsId& obs_id)
{
  if (checkPlane(current_octo, point_w, 3)) {
    if (current_octo->plane_ptr_->is_landmark_) {
      current_octo->plane_ptr_->obs_.push_back(obs_id);
    }
    plane_out = *current_octo->plane_ptr_;
    return true;
  }

  // If no plane or not matched, search children
  if (layer < options_.max_layer) {
    for (size_t leafnum = 0; leafnum < 8; leafnum++) {
      if (current_octo->leaves_[leafnum] != nullptr) {
        if (searchVoxelRecursive(point_w, plane_out, current_octo->leaves_[leafnum], layer + 1,
                                 obs_id)) {
          return true;
        }
      }
    }
  }
  return false;
}

bool TreeHandler::checkPlane(VoxelOctoTree* current_octo, const Point_lidar& point_w,
                             const int sigma_n)
{
  if (!current_octo->plane_ptr_->is_plane_) return false;

  VoxelPlane& plane = *current_octo->plane_ptr_;

  Eigen::Vector3d point(point_w.x, point_w.y, point_w.z);
  Eigen::Vector3d normal(plane.normal_(0), plane.normal_(1), plane.normal_(2));
  float dist_to_plane = normal.dot(point - plane.center_);
  float dist_center = (point - plane.center_).norm();
  float dist_range = std::sqrt(dist_center * dist_center - dist_to_plane * dist_to_plane);

  if (dist_range < sigma_n * plane.radius_) {
    // Check distance using the projected point covariance
    Eigen::Matrix3d cov =
        Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(point_w.covariance);
    float dist_threshold = sigma_n * std::sqrt(normal.transpose() * cov * normal);
    if (std::fabs(dist_to_plane) < dist_threshold) {
      return true;
    }
  }

  return false;
}

void TreeHandler::updateVoxelMap(Cloud_ptr cloud_w)
{
  double voxel_size = options_.voxel_size;
  int max_layer = options_.max_layer;
  int max_points_num = options_.max_points_num;
  int layer_init_num = options_.layer_init_num;
  double planner_threshold = options_.plane_threshold;

  std::vector<pointWithVar> input_points = convertCloudtoVec(cloud_w);

  mutex_.lock();
  // Insert world-frame points into their root voxels
  for (size_t i = 0; i < input_points.size(); i++) {
    auto pv = input_points[i];

    float loc_xyz[3];
    convertVoxelKey(pv.point_w, voxel_size, loc_xyz);

    VOXEL_LOCATION position(static_cast<int64_t>(loc_xyz[0]), static_cast<int64_t>(loc_xyz[1]),
                            static_cast<int64_t>(loc_xyz[2]));

    auto iter = voxel_map_.find(position);
    if (iter != voxel_map_.end()) {
      voxel_map_[position]->UpdateOctoTree(pv);
    } else {
      VoxelOctoTree* octo_tree =
          new VoxelOctoTree(max_layer, 0, layer_init_num, max_points_num, planner_threshold);
      voxel_map_[position] = octo_tree;
      voxel_map_[position]->layer_init_num_ = layer_init_num;
      voxel_map_[position]->quater_length_ = voxel_size / 4.0;
      voxel_map_[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
      voxel_map_[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
      voxel_map_[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
      voxel_map_[position]->UpdateOctoTree(pv);
    }
  }
  mutex_.unlock();
}

void TreeHandler::updateVoxelKey()
{
  std::vector<VOXEL_LOCATION> keys_to_remove;
  keys_to_remove.reserve(voxel_map_.size());

  for (auto it = voxel_map_.begin(); it != voxel_map_.end(); ++it) {
    VoxelOctoTree* octo_node = it->second;
    if (!octo_node) continue;

    VoxelPlanePtr plane_ptr = octo_node->plane_ptr_;
    // Only re-key landmarks owned by the backend graph
    if (plane_ptr && plane_ptr->is_landmark_ && plane_ptr->is_in_graph_) {
      VOXEL_LOCATION old_key = it->first;

      Eigen::Vector3d new_center = plane_ptr->center_;
      float loc_xyz[3];
      convertVoxelKey(new_center, options_.voxel_size, loc_xyz);

      VOXEL_LOCATION new_key(static_cast<int64_t>(loc_xyz[0]), static_cast<int64_t>(loc_xyz[1]),
                             static_cast<int64_t>(loc_xyz[2]));

      if (new_key == old_key) {
        continue;
      }

      auto new_it = voxel_map_.find(new_key);
      if (new_it == voxel_map_.end()) {
        // Move the complete octree when the target voxel is empty
        voxel_map_[new_key] = octo_node;
        keys_to_remove.push_back(old_key);

      } else {
        VoxelOctoTree* existing_node = new_it->second;
        VoxelPlanePtr existing_plane = existing_node->plane_ptr_;

        // Backend-owned landmarks take precedence over frontend-only voxels
        if (!existing_plane || !existing_plane->is_landmark_ || !existing_plane->is_in_graph_) {
          delete existing_node;
          voxel_map_[new_key] = octo_node;
          keys_to_remove.push_back(old_key);

        }
      }
    }
  }

  // Erase stale keys without removing a node already moved to a new key
  for (const auto& k : keys_to_remove) {
    auto it_erase = voxel_map_.find(k);
    if (it_erase != voxel_map_.end()) {
      voxel_map_.erase(it_erase);
    }
  }
}

PlaneCloud_ptr TreeHandler::visualizePlanes()
{
  PlaneCloud_ptr cloud(new PlaneCloud);

  mutex_.lock();
  for (auto iter = voxel_map_.begin(); iter != voxel_map_.end(); ++iter) {
    if (iter->second->plane_ptr_->is_in_graph_) {
      Plane pt;
      VoxelPlane* plane = iter->second->plane_ptr_.get();

      if (!plane->is_plane_) continue;

      pt.x = plane->center_(0);
      pt.y = plane->center_(1);
      pt.z = plane->center_(2);
      pt.normal_x = plane->normal_(0);
      pt.normal_y = plane->normal_(1);
      pt.normal_z = plane->normal_(2);

      Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(pt.covariance) = plane->covariance_;

      cloud->points.push_back(pt);
    }
  }
  mutex_.unlock();
  return cloud;
}

void TreeHandler::mapSlide(const Eigen::Vector3d& current_pos)
{
  double distance_moved = (current_pos - mapslide_pose_last_).norm();
  if (distance_moved < options_.mapslide_step) {
    return;
  }

  mapslide_pose_last_ = current_pos;

  float loc_xyz[3];
  convertVoxelKey(mapslide_pose_last_, options_.voxel_size, loc_xyz);

  int64_t x_center = static_cast<int64_t>(loc_xyz[0]);
  int64_t y_center = static_cast<int64_t>(loc_xyz[1]);
  int64_t z_center = static_cast<int64_t>(loc_xyz[2]);

  int64_t x_max = x_center + options_.mapslide_size;
  int64_t x_min = x_center - options_.mapslide_size;
  int64_t y_max = y_center + options_.mapslide_size;
  int64_t y_min = y_center - options_.mapslide_size;
  int64_t z_max = z_center + options_.mapslide_size;
  int64_t z_min = z_center - options_.mapslide_size;

  mutex_.lock();
  for (auto it = voxel_map_.begin(); it != voxel_map_.end();) {
    const VOXEL_LOCATION& loc = it->first;
    bool out_of_bounds = (loc.x > x_max || loc.x < x_min || loc.y > y_max || loc.y < y_min ||
                          loc.z > z_max || loc.z < z_min);
    if (out_of_bounds) {
      delete it->second;
      it = voxel_map_.erase(it);
    } else {
      ++it;
    }
  }
  mutex_.unlock();
}

}  // namespace gici
