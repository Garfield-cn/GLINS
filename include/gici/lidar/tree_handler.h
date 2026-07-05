/**
 * @Function: KD-tree management
 *
 * @Author  : Jiahui Liu
 * @Email   : jh.liu@sjtu.edu.cn
 **/
#pragma once

#include <memory>

#include "gici/imu/imu_types.h"
#include "gici/utility/common.h"
#include "gici/imu/imu_estimator_base.h"
#include "gici/lidar/ikd_Tree.h"
#include "gici/lidar/voxel_types.h"

namespace gici {

using ScanPtr = shared_ptr<LidarMeasurement>;

// LiDAR frontend and voxel map options
struct TreeHandlerOptions {
  // LiDAR range standard deviation, in meters
  double std_range = 0.2;

  // LiDAR angular standard deviation, in degrees
  double std_angle = 0.3;

  // Voxel-filter leaf size, in meters
  double filter_radius = 0.6;

  // Maximum nearest-neighbor distance, in meters
  double max_near_dis = 1;

  // Maximum point-to-plane fitting distance, in meters
  double max_plane_dis = 0.1;

  // Number of consistent observations required to promote a plane to a landmark
  int merge_size = 40;

  // Voxel side length, in meters
  double voxel_size = 5;

  // Maximum octree subdivision depth inside a root voxel
  int max_layer = 2;

  // Maximum number of points retained before an octree node is subdivided
  int max_points_num = 50;

  // Minimum number of points used to initialize a plane at each octree layer
  int layer_init_num = 10;

  // Planarity threshold used when accepting an octree node as a plane
  double plane_threshold = 0.1;

  // Translation that triggers map sliding, in meters
  double mapslide_step = 20;

  // Half-width of the retained local map, in root-voxel cells
  double mapslide_size = 50;
};

template <typename PointT>
class Kd_Tree {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Kd_Tree(const TreeHandlerOptions& options)
  {
    kdtree_.reset(new KD_TREE<PointT>());
    setOptions(options);
  };

  ~Kd_Tree(){};

  inline void setOptions(const TreeHandlerOptions& options)
  {
    options_ = options;
  }

  inline int getTreeSize()
  {
    return kdtree_->size();
  }

  bool isEmpty()
  {
    return kdtree_->size() == 0;
  }

  inline void treeBuild(typename pcl::PointCloud<PointT>::Ptr cloud)
  {
    kdtree_.reset(new KD_TREE<PointT>());
    kdtree_->Build(cloud->points);
  }

  inline void treeGrow(typename pcl::PointCloud<PointT>::Ptr input_scan)
  {
    // Since ikd-tree only supports a vector adding to the tree, we have to convert data
    points_add_.clear();
    for (size_t i = 0; i < input_scan->size(); i++) {
      points_add_.push_back(input_scan->points[i]);
    }
    kdtree_->Add_Points(points_add_, true);
  }

  inline void treeGrow(PointT point)
  {
    // Since ikd-tree only supports a vector adding to the tree, we have to convert data
    points_add_.clear();
    points_add_.push_back(point);
    kdtree_->Add_Points(points_add_, true);
  }

  inline void treeUpdate(PointT point)
  {
    points_add_.clear();
    points_add_.push_back(point);

    kdtree_->Delete_Points(points_add_);
    kdtree_->Add_Points(points_add_, false);
  }

  // Search nearest points to get keypoint planes
  inline bool search(const PointT point, Eigen::Vector4d& params)
  {
    std::vector<PointT, Eigen::aligned_allocator<PointT>> points_near;
    vector<float> dis(5);

    // Search nearest five points
    kdtree_->Nearest_Search(point, 5, points_near, dis);

    if (points_near.size() < 5 || dis[4] > options_.max_near_dis ||
        dis[3] > options_.max_near_dis) {
      return false;
    }

    // Check the searched points
    CHECK_EQ(points_near.size(), 5);

    // Plane fitting
    Eigen::Matrix<double, 5, 3> A;
    Eigen::Matrix<double, 5, 1> b;
    A.setZero();
    b.setOnes();
    b *= -1;

    // Solve least square problem to get normal vector
    for (auto k = 0; k < 5; k++) {
      A(k, 0) = points_near[k].x;
      A(k, 1) = points_near[k].y;
      A(k, 2) = points_near[k].z;
    }

    Eigen::Vector3d n = A.colPivHouseholderQr().solve(b);
    double d = 1 / n.norm();
    n.normalize();

    // Check distance threshold
    for (auto j = 0; j < 5; j++) {
      if (abs(n.x() * points_near[j].x + n.y() * points_near[j].y + n.z() * points_near[j].z + d) >
          options_.max_plane_dis) {
        return false;
      }
    }

    params.block<3, 1>(0, 0) = n;
    params(3) = d;

    return true;
  }

  // Plane tree search for plane merging
  inline bool search(const PointT point,
                     std::vector<PointT, Eigen::aligned_allocator<PointT>>& points_near,
                     vector<float>& dis)
  {
    // Search nearest two planes
    kdtree_->Nearest_Search(point, 3, points_near, dis);

    if (points_near.size() < 3 || dis[0] == 0 || dis[1] == 0 || (dis[0] > 3 && dis[1] > 3)) {
      return false;
    }

    // Check the searched points
    CHECK_EQ(points_near.size(), 3);

    return true;
  }

  // Landmark nearest search to avoid duplicate landmarks
  inline bool search(const PointT point, PointT& point_near)
  {
    // Search nearest two planes
    std::vector<PointT, Eigen::aligned_allocator<PointT>> points_near;
    vector<float> dis(1);
    kdtree_->Nearest_Search(point, 1, points_near, dis);

    if (dis[0] == 0 || dis[0] > 5) {
      return false;
    }

    // Check the searched points
    CHECK_EQ(points_near.size(), 1);

    point_near = points_near[0];

    return true;
  }

protected:
  // Options
  TreeHandlerOptions options_;

  shared_ptr<KD_TREE<PointT>> kdtree_;

  // Scan points to be added to map
  vector<PointT, Eigen::aligned_allocator<PointT>> points_add_;
};

// LiDAR point and voxel map handler
class TreeHandler {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TreeHandler(const TreeHandlerOptions& options, const ImuEstimatorBaseOptions& imu_options,
              const Transformation& T_B_L);
  ~TreeHandler();

  // Downsample scan and compute point covariance
  bool processLidar(std::shared_ptr<LidarMeasurement>& scan);

  inline void set_T_BL(Transformation T)
  {
    T_BL_ = T;
  }

  inline void set_T_BL_MAT()
  {
    T_BL_MAT_.setIdentity();
    T_BL_MAT_.block<3, 3>(0, 0) = T_BL_.getRotationMatrix();
    T_BL_MAT_.block<3, 1>(0, 3) = T_BL_.getPosition();
  }

  inline int getMapSize()
  {
    return laser_map_ptr_->size();
  }

  inline int getTreeSize()
  {
    return map_tree_->getTreeSize();
  }

  inline Cloud_ptr getMap()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return laser_map_ptr_;
  }

  inline Cloud_ptr getResidual()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return residual_cloud_;
  }

  inline void setResidual(Cloud_ptr cloud)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    residual_cloud_ = cloud;
  }

  inline PlaneCloud_ptr getPlanesFrontend()
  {
    return plane_ptr_;
  }

  inline PlaneCloud_ptr getPlaneMap()
  {
    return plane_map_ptr_;
  }

  inline PlaneCloud_ptr getLandmarks()
  {
    return landmark_map_ptr_;
  }

  inline Eigen::Matrix4d get_T_BL_MAT()
  {
    return T_BL_MAT_;
  }

  inline Eigen::Matrix4d transTomat(Transformation T)
  {
    Eigen::Matrix4d M;
    M.setIdentity();
    M.block<3, 3>(0, 0) = T.getRotationMatrix();
    M.block<3, 1>(0, 3) = T.getPosition();
    return M;
  }

  inline double normalDot(const Plane& plane1, const Plane& plane2)
  {
    return std::abs(plane1.normal_x * plane2.normal_x + plane1.normal_y * plane2.normal_y +
                    plane1.normal_z * plane2.normal_z);
  }

  inline void convertVoxelKey(const Eigen::Vector3d& point, float voxel_size, float loc_xyz[3])
  {
    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = static_cast<float>(point[j] / voxel_size);
      if (loc_xyz[j] < 0) {
        loc_xyz[j] -= 1.0f;
      }
    }
  }

  Cloud_ptr filterCloud(Cloud_ptr cloud_in, double filter_radius);

  void calculateCov(Cloud_ptr cloud, double std_range, double std_angle);

  bool treeInit();

  // Build the global point map used for nearest-neighbor registration
  void mapBuild(Cloud_ptr cloud);

  void mapGrow(Cloud_ptr input_scan);

  // Extract scan-to-map plane correspondences
  bool keypointExtract(ScanPtr& scan, const Transformation T_WS);

  void planeGrow(ScanPtr& scan);

  void mergePlanes(Plane& current, const Plane& target);

  void updatePlane(Plane& plane);

  void landmarkTreeBuild(PlaneCloud_ptr cloud);

  void landmarkTreeGrow(PlaneCloud_ptr cloud);

  inline std::shared_ptr<Kd_Tree<Point_lidar>> getGlobalTree()
  {
    return map_tree_;
  }

  inline std::shared_ptr<Kd_Tree<Point_lidar>> getLocalTree()
  {
    return map_tree_;
  }

  // Voxel map inputs use world-frame coordinates
  std::vector<pointWithVar> convertCloudtoVec(const Cloud_ptr& cloud_w);

  // Initialize the voxel map
  void buildVoxelMap(Cloud_ptr cloud_w);

  // Search optimized landmarks; obs_id prevents self-association
  bool searchLandmarks(const Point_lidar& point_w, VoxelPlanePtr& plane_out, const ObsId& obs_id);

  // Search all valid voxel planes
  bool searchVoxelMap(const Point_lidar& point_w, VoxelPlane& plane_out, const ObsId& obs_id);

  bool searchVoxelRecursive(const Point_lidar& point_w, VoxelPlane& plane_out,
                            VoxelOctoTree* current_octo, int layer, const ObsId& obs_id);

  bool checkPlane(VoxelOctoTree* current_octo, const Point_lidar& point_w, const int sigma_n);

  // Insert a world-frame scan into the voxel map
  void updateVoxelMap(Cloud_ptr cloud_w);

  // Re-key voxel landmarks after backend optimization
  void updateVoxelKey();

  // Export graph-owned planes for visualization
  PlaneCloud_ptr visualizePlanes();

  // Prune voxels outside the moving local map
  void mapSlide(const Eigen::Vector3d& current_pos);

public:
  Cloud_ptr residual_cloud_;

protected:
  // Release all root voxel nodes and their children
  void clearVoxelMap();

  // Options
  TreeHandlerOptions options_;

  // IMU
  double speed_and_bias_timestamp_;
  SpeedAndBias speed_and_bias_;

  // LiDAR-to-body extrinsic transformation T_B_L
  Transformation T_BL_;
  Eigen::Matrix4d T_BL_MAT_;

  // Output map
  Cloud_ptr laser_map_ptr_;

  PlaneCloud_ptr plane_ptr_, plane_map_ptr_, landmark_map_ptr_;

  PlaneCloud_ptr current_new_landmarks_;

  // Initialized flag
  bool tree_init_ = false;
  bool local_map = false;

  // Scan points to be added to map
  vector<Point_lidar, Eigen::aligned_allocator<Point_lidar>> points_add_;

  // Local map
  Eigen::Vector3d lmap_leftbot, lmap_righttop, lmap_center;

  // Trees
  std::shared_ptr<Kd_Tree<Point_lidar>> map_tree_;
  std::shared_ptr<Kd_Tree<Plane>> plane_tree_;
  std::shared_ptr<Kd_Tree<Plane>> landmark_tree_;

  int id_ = 0;

  std::mutex mutex_;

  // Backend landmark tree
  std::shared_ptr<Kd_Tree<Plane>> backend_landmark_tree_;

  // Voxel map
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree*> voxel_map_;

  Eigen::Vector3d mapslide_pose_last_;
};

}
