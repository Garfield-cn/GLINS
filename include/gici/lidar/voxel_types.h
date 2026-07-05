/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.

Modified by Jiahui Liu <jh.liu@sjtu.edu.cn>
*/

#ifndef VOXEL_MAP_H_
#define VOXEL_MAP_H_

#include <Eigen/Dense>
#include <fstream>
#include <math.h>
#include <mutex>
#include <omp.h>
#include <pcl/common/io.h>
#include <thread>
#include <unistd.h>
#include <unordered_map>

#define VOXELMAP_HASH_P 116101
#define VOXELMAP_MAX_N 10000000000

namespace gici {

static int voxel_plane_id = 0;
static int plane_landmark_id = 0;

using M3D = Eigen::Matrix3d;
using V3D = Eigen::Vector3d;

typedef struct pointWithVar {
  Eigen::Vector3d point_b;      // Point in the LiDAR frame
  Eigen::Vector3d point_i;      // Point in the IMU/body frame
  Eigen::Vector3d point_w;      // Point in the world frame
  Eigen::Matrix3d var_nostate;  // Point covariance without state uncertainty
  Eigen::Matrix3d body_var;
  Eigen::Matrix3d var;
  Eigen::Matrix3d point_crossmat;
  Eigen::Vector3d normal;

  pointWithVar()
  {
    var_nostate = Eigen::Matrix3d::Zero();
    var = Eigen::Matrix3d::Zero();
    body_var = Eigen::Matrix3d::Zero();
    point_crossmat = Eigen::Matrix3d::Zero();
    point_b = Eigen::Vector3d::Zero();
    point_i = Eigen::Vector3d::Zero();
    point_w = Eigen::Vector3d::Zero();
    normal = Eigen::Vector3d::Zero();
  };
} pointWithVar;

typedef struct VoxelMapConfig {
  double max_voxel_size_;
  int max_layer_;
  int max_iterations_;
  int layer_init_num_;
  int max_points_num_;
  double planner_threshold_;
  double beam_err_;
  double dept_err_;
  double sigma_num_;
  bool is_pub_plane_map_;

  // Local map sliding
  double sliding_thresh;
  bool map_sliding_en;
  int half_map_size;
} VoxelMapConfig;

typedef struct PointToPlane {
  Eigen::Vector3d point_b_;
  Eigen::Vector3d point_w_;
  Eigen::Vector3d normal_;
  Eigen::Vector3d center_;
  Eigen::Matrix<double, 6, 6> plane_var_;
  M3D body_cov_;
  int layer_;
  double d_;
  double eigen_value_;
  bool is_valid_;
  float dis_to_plane_;
} PointToPlane;

typedef struct VoxelPlane {
  Eigen::Vector3d center_;
  Eigen::Vector3d normal_;
  Eigen::Vector3d y_normal_;
  Eigen::Vector3d x_normal_;
  Eigen::Matrix3d covariance_;
  Eigen::Matrix<double, 6, 6> plane_var_;
  float radius_ = 0;
  float min_eigen_value_ = 1;
  float mid_eigen_value_ = 1;
  float max_eigen_value_ = 1;
  float d_ = 0;
  size_t points_size_ = 0;
  bool is_plane_ = false;
  bool is_init_ = false;
  int id_ = 0;
  bool is_update_ = false;
  bool is_landmark_ = false;
  bool is_in_graph_ = false;
  int landmark_id_ = 0;
  std::vector<ObsId> obs_;

  VoxelPlane()
  {
    plane_var_ = Eigen::Matrix<double, 6, 6>::Zero();
    covariance_ = Eigen::Matrix3d::Zero();
    center_ = Eigen::Vector3d::Zero();
    normal_ = Eigen::Vector3d::Zero();
  }
} VoxelPlane;

using VoxelPlanePtr = std::shared_ptr<VoxelPlane>;

struct MapPlane {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief Default constructor. Point is nullptr.
  MapPlane() :
    plane(nullptr),
    fixed_position(false)
  {}
  /**
   * @brief Constructor.
   */
  MapPlane(const VoxelPlanePtr& plane) :
    plane(plane),
    fixed_position(false)
  {
    hom_coordinates << plane->center_, 1;
  }

  Eigen::Vector4d hom_coordinates;  ///< Continuosly updates position of point

  VoxelPlanePtr plane;

  std::map<ObsId, uint64_t> observations;

  // Observation counter since the landmark has been initialized
  size_t num_observations_historical = 0;

  bool fixed_position;
};

class VOXEL_LOCATION {
public:
  int64_t x, y, z;

  VOXEL_LOCATION(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0) :
    x(vx),
    y(vy),
    z(vz)
  {}

  bool operator==(const VOXEL_LOCATION& other) const
  {
    return (x == other.x && y == other.y && z == other.z);
  }
};

struct DS_POINT {
  float xyz[3];
  float intensity;
  int count = 0;
};

class VoxelOctoTree {
public:
  std::vector<pointWithVar> temp_points_;
  VoxelPlanePtr plane_ptr_;
  int layer_;
  int octo_state_;  // 0 is end of tree, 1 is not
  VoxelOctoTree* leaves_[8];
  double voxel_center_[3];  // x, y, z
  size_t layer_init_num_;
  float quater_length_;
  float planer_threshold_;
  size_t points_size_threshold_;
  size_t update_size_threshold_;
  size_t max_points_num_;
  int max_layer_;
  size_t new_points_;
  bool init_octo_;
  bool update_enable_;

  VoxelOctoTree(int max_layer, int layer, size_t points_size_threshold, size_t max_points_num,
                float planer_threshold)
      : plane_ptr_(std::make_shared<VoxelPlane>()),
        layer_(layer),
        octo_state_(0),
        leaves_{},
        voxel_center_{},
        layer_init_num_(points_size_threshold),
        quater_length_(0.0f),
        planer_threshold_(planer_threshold),
        points_size_threshold_(points_size_threshold),
        update_size_threshold_(5),
        max_points_num_(max_points_num),
        max_layer_(max_layer),
        new_points_(0),
        init_octo_(false),
        update_enable_(true)
  {}

  ~VoxelOctoTree()
  {
    for (int i = 0; i < 8; i++) {
      delete leaves_[i];
    }
  }

  void init_plane(const std::vector<pointWithVar>& points, VoxelPlanePtr plane);

  void init_octo_tree();

  void cut_octo_tree();

  void UpdateOctoTree(const pointWithVar& pv);

  VoxelOctoTree* find_correspond(Eigen::Vector3d pw);

  VoxelOctoTree* Insert(const pointWithVar& pv);
};

inline void VoxelOctoTree::init_plane(const std::vector<pointWithVar>& points, VoxelPlanePtr plane)
{
  plane->plane_var_ = Eigen::Matrix<double, 6, 6>::Zero();
  plane->covariance_ = Eigen::Matrix3d::Zero();
  plane->center_ = Eigen::Vector3d::Zero();
  plane->normal_ = Eigen::Vector3d::Zero();
  plane->points_size_ = points.size();
  plane->radius_ = 0;
  for (auto pv : points) {
    plane->covariance_ += pv.point_w * pv.point_w.transpose();
    plane->center_ += pv.point_w;
  }
  plane->center_ = plane->center_ / plane->points_size_;
  plane->covariance_ =
      plane->covariance_ / plane->points_size_ - plane->center_ * plane->center_.transpose();
  Eigen::EigenSolver<Eigen::Matrix3d> es(plane->covariance_);
  Eigen::Matrix3cd evecs = es.eigenvectors();
  Eigen::Vector3cd evals = es.eigenvalues();
  Eigen::Vector3d evalsReal;
  evalsReal = evals.real();
  Eigen::Matrix3f::Index evalsMin, evalsMax;
  evalsReal.rowwise().sum().minCoeff(&evalsMin);
  evalsReal.rowwise().sum().maxCoeff(&evalsMax);
  int evalsMid = 3 - evalsMin - evalsMax;
  Eigen::Matrix3d J_Q;
  J_Q << 1.0 / plane->points_size_, 0, 0, 0, 1.0 / plane->points_size_, 0, 0, 0,
      1.0 / plane->points_size_;
  if (evalsReal(evalsMin) < planer_threshold_) {
    for (size_t i = 0; i < points.size(); i++) {
      Eigen::Matrix<double, 6, 3> J;
      Eigen::Matrix3d F;
      for (int m = 0; m < 3; m++) {
        if (m != (int)evalsMin) {
          Eigen::Matrix<double, 1, 3> F_m =
              (points[i].point_w - plane->center_).transpose() /
              ((plane->points_size_) * (evalsReal[evalsMin] - evalsReal[m])) *
              (evecs.real().col(m) * evecs.real().col(evalsMin).transpose() +
               evecs.real().col(evalsMin) * evecs.real().col(m).transpose());
          F.row(m) = F_m;
        } else {
          Eigen::Matrix<double, 1, 3> F_m;
          F_m << 0, 0, 0;
          F.row(m) = F_m;
        }
      }
      J.block<3, 3>(0, 0) = evecs.real() * F;
      J.block<3, 3>(3, 0) = J_Q;
      plane->plane_var_ += J * points[i].var * J.transpose();
    }

    plane->normal_ << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin),
        evecs.real()(2, evalsMin);
    plane->y_normal_ << evecs.real()(0, evalsMid), evecs.real()(1, evalsMid),
        evecs.real()(2, evalsMid);
    plane->x_normal_ << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax),
        evecs.real()(2, evalsMax);
    plane->min_eigen_value_ = evalsReal(evalsMin);
    plane->mid_eigen_value_ = evalsReal(evalsMid);
    plane->max_eigen_value_ = evalsReal(evalsMax);
    plane->radius_ = sqrt(evalsReal(evalsMax));
    plane->d_ = -(plane->normal_(0) * plane->center_(0) + plane->normal_(1) * plane->center_(1) +
                  plane->normal_(2) * plane->center_(2));
    plane->is_plane_ = true;
    plane->is_update_ = true;
    if (!plane->is_init_) {
      plane->id_ = voxel_plane_id;
      voxel_plane_id++;
      plane->is_init_ = true;
    }
  } else {
    plane->is_update_ = true;
    plane->is_plane_ = false;
  }
}

inline void VoxelOctoTree::init_octo_tree()
{
  if (temp_points_.size() > points_size_threshold_) {
    init_plane(temp_points_, plane_ptr_);
    if (plane_ptr_->is_plane_ == true) {
      octo_state_ = 0;
      if (temp_points_.size() > max_points_num_) {
        update_enable_ = false;
        std::vector<pointWithVar>().swap(temp_points_);
        new_points_ = 0;
        if (octo_state_ == 0 && plane_ptr_->min_eigen_value_ < 0.1) {
          plane_ptr_->is_landmark_ = true;
          plane_landmark_id++;
          plane_ptr_->landmark_id_ = plane_landmark_id;
        }
      }
    } else {
      octo_state_ = 1;
      cut_octo_tree();
    }
    init_octo_ = true;
    new_points_ = 0;
  }
}

inline void VoxelOctoTree::cut_octo_tree()
{
  if (layer_ >= max_layer_) {
    octo_state_ = 0;
    return;
  }
  for (size_t i = 0; i < temp_points_.size(); i++) {
    int xyz[3] = {0, 0, 0};
    if (temp_points_[i].point_w[0] > voxel_center_[0]) {
      xyz[0] = 1;
    }
    if (temp_points_[i].point_w[1] > voxel_center_[1]) {
      xyz[1] = 1;
    }
    if (temp_points_[i].point_w[2] > voxel_center_[2]) {
      xyz[2] = 1;
    }
    int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
    if (leaves_[leafnum] == nullptr) {
      leaves_[leafnum] = new VoxelOctoTree(max_layer_, layer_ + 1, layer_init_num_, max_points_num_,
                                           planer_threshold_);
      leaves_[leafnum]->layer_init_num_ = layer_init_num_;
      leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
      leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
      leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
      leaves_[leafnum]->quater_length_ = quater_length_ / 2;
    }
    leaves_[leafnum]->temp_points_.push_back(temp_points_[i]);
    leaves_[leafnum]->new_points_++;
  }
  for (uint i = 0; i < 8; i++) {
    if (leaves_[i] != nullptr) {
      if (leaves_[i]->temp_points_.size() > leaves_[i]->points_size_threshold_) {
        init_plane(leaves_[i]->temp_points_, leaves_[i]->plane_ptr_);
        if (leaves_[i]->plane_ptr_->is_plane_) {
          leaves_[i]->octo_state_ = 0;
          if (leaves_[i]->temp_points_.size() > leaves_[i]->max_points_num_) {
            leaves_[i]->update_enable_ = false;
            std::vector<pointWithVar>().swap(leaves_[i]->temp_points_);
            new_points_ = 0;
          }
        } else {
          leaves_[i]->octo_state_ = 1;
          leaves_[i]->cut_octo_tree();
        }
        leaves_[i]->init_octo_ = true;
        leaves_[i]->new_points_ = 0;
      }
    }
  }
}

inline void VoxelOctoTree::UpdateOctoTree(const pointWithVar& pv)
{
  if (!init_octo_) {
    new_points_++;
    temp_points_.push_back(pv);
    if (temp_points_.size() > points_size_threshold_) {
      init_octo_tree();
    }
  } else {
    if (plane_ptr_->is_plane_) {
      if (update_enable_) {
        new_points_++;
        temp_points_.push_back(pv);
        if (new_points_ > update_size_threshold_) {
          init_plane(temp_points_, plane_ptr_);
          new_points_ = 0;
        }
        if (temp_points_.size() >= max_points_num_) {
          update_enable_ = false;
          std::vector<pointWithVar>().swap(temp_points_);
          new_points_ = 0;
          if (octo_state_ == 0 && plane_ptr_->min_eigen_value_ < 0.1) {
            plane_ptr_->is_landmark_ = true;
            plane_landmark_id++;
            plane_ptr_->landmark_id_ = plane_landmark_id;
          }
        }
      }
    } else {
      if (layer_ < max_layer_) {
        int xyz[3] = {0, 0, 0};
        if (pv.point_w[0] > voxel_center_[0]) {
          xyz[0] = 1;
        }
        if (pv.point_w[1] > voxel_center_[1]) {
          xyz[1] = 1;
        }
        if (pv.point_w[2] > voxel_center_[2]) {
          xyz[2] = 1;
        }
        int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
        if (leaves_[leafnum] != nullptr) {
          leaves_[leafnum]->UpdateOctoTree(pv);
        } else {
          leaves_[leafnum] = new VoxelOctoTree(max_layer_, layer_ + 1, layer_init_num_,
                                               max_points_num_, planer_threshold_);
          leaves_[leafnum]->layer_init_num_ = layer_init_num_;
          leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
          leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
          leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
          leaves_[leafnum]->quater_length_ = quater_length_ / 2;
          leaves_[leafnum]->UpdateOctoTree(pv);
        }
      } else {
        if (update_enable_) {
          new_points_++;
          temp_points_.push_back(pv);
          if (new_points_ > update_size_threshold_) {
            init_plane(temp_points_, plane_ptr_);
            new_points_ = 0;
          }
          if (temp_points_.size() > max_points_num_) {
            update_enable_ = false;
            std::vector<pointWithVar>().swap(temp_points_);
            new_points_ = 0;
          }
        }
      }
    }
  }
}

inline VoxelOctoTree* VoxelOctoTree::find_correspond(Eigen::Vector3d pw)
{
  if (!init_octo_ || plane_ptr_->is_plane_ || (layer_ >= max_layer_)) return this;

  int xyz[3] = {0, 0, 0};
  xyz[0] = pw[0] > voxel_center_[0] ? 1 : 0;
  xyz[1] = pw[1] > voxel_center_[1] ? 1 : 0;
  xyz[2] = pw[2] > voxel_center_[2] ? 1 : 0;
  int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

  return (leaves_[leafnum] != nullptr) ? leaves_[leafnum]->find_correspond(pw) : this;
}

inline VoxelOctoTree* VoxelOctoTree::Insert(const pointWithVar& pv)
{
  if ((!init_octo_) || (init_octo_ && plane_ptr_->is_plane_) ||
      (init_octo_ && (!plane_ptr_->is_plane_) && (layer_ >= max_layer_))) {
    new_points_++;
    temp_points_.push_back(pv);
    return this;
  }

  if (init_octo_ && (!plane_ptr_->is_plane_) && (layer_ < max_layer_)) {
    int xyz[3] = {0, 0, 0};
    xyz[0] = pv.point_w[0] > voxel_center_[0] ? 1 : 0;
    xyz[1] = pv.point_w[1] > voxel_center_[1] ? 1 : 0;
    xyz[2] = pv.point_w[2] > voxel_center_[2] ? 1 : 0;
    int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
    if (leaves_[leafnum] != nullptr) {
      return leaves_[leafnum]->Insert(pv);
    } else {
      leaves_[leafnum] = new VoxelOctoTree(max_layer_, layer_ + 1, layer_init_num_, max_points_num_,
                                           planer_threshold_);
      leaves_[leafnum]->layer_init_num_ = layer_init_num_;
      leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
      leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
      leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
      leaves_[leafnum]->quater_length_ = quater_length_ / 2;
      return leaves_[leafnum]->Insert(pv);
    }
  }
  return nullptr;
}

}  // namespace gici

// Hash value
namespace std {
template <>
struct hash<gici::VOXEL_LOCATION> {
  int64_t operator()(const gici::VOXEL_LOCATION& s) const
  {
    using std::hash;
    using std::size_t;
    return ((((s.z) * VOXELMAP_HASH_P) % VOXELMAP_MAX_N + (s.y)) * VOXELMAP_HASH_P) %
               VOXELMAP_MAX_N +
           (s.x);
  }
};
}  // namespace std

#endif  // VOXEL_MAP_H_
