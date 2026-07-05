/**
 * @Function: LiDAR data types
 *
 * @Author  : Jiahui Liu
 * @Email   : jh.liu@sjtu.edu.cn
 **/

#pragma once
#define PCL_NO_PRECOMPILE

#include <iostream>
#include <vector>
#include <deque>
#include <memory>
#include <execution>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_search.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/joint_icp.h>
#include <pcl/registration/gicp.h>

namespace gici {

// W is the local ENU world frame, B is the IMU/body frame and L is the LiDAR frame
// T_A_B maps coordinates from frame B to frame A

// Role of formator
enum class LidarRole { None, Front, Side };

struct ObsId {
  bool iskeyframe;
  int id;
  size_t point_idx;

  ObsId(const bool _isKeyframe, const int _id, const size_t _point_idx) :
    iskeyframe(_isKeyframe),
    id(_id),
    point_idx(_point_idx)
  {}

  inline bool operator==(const ObsId& other) const
  {
    return id == other.id && point_idx == other.point_idx;
  }

  bool operator<(const ObsId& rhs) const
  {
    if (id == rhs.id) {
      return point_idx < rhs.point_idx;
    }
    return id < rhs.id;
  }
};

struct EIGEN_ALIGN16 PointXYZINormalCov {
  PCL_ADD_POINT4D;
  PCL_ADD_NORMAL4D;

  float intensity;
  // Per-point time offset from scan start, in seconds
  float curvature;
  double d;
  // Row-major Cartesian point covariance, in square meters
  double covariance[9];

  PointXYZINormalCov()
  {
    x = y = z = 0.0f;
    data[3] = 1.0f;
    normal_x = normal_y = normal_z = d = 0.0f;
    data_n[3] = 0.0f;
    intensity = 0.0f;
    curvature = 0.0f;
    std::fill(std::begin(covariance), std::end(covariance), 0.0f);
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 PlaneXYZNormalCov {
  PCL_ADD_POINT4D;
  PCL_ADD_NORMAL4D;

  double d;
  // Row-major covariance of the plane center, in square meters
  double covariance[9];
  // Plane observation time in Unix/ROS seconds
  double timestamp;
  float intensity;
  int id;
  int sub_id;
  int count;

  PlaneXYZNormalCov()
  {
    x = y = z = 0.0f;
    data[3] = 1.0f;
    normal_x = normal_y = normal_z = d = 0.0f;
    data_n[3] = 0.0f;
    id = sub_id = 0;
    count = 0;
    timestamp = 0.0;
    std::fill(std::begin(covariance), std::end(covariance), 0.0f);
  }

  PlaneXYZNormalCov(const PointXYZINormalCov& point)
  {
    x = point.x;
    y = point.y;
    z = point.z;
    data[3] = point.data[3];

    normal_x = point.normal_x;
    normal_y = point.normal_y;
    normal_z = point.normal_z;
    data_n[3] = point.data_n[3];

    d = point.d;
    intensity = point.intensity;

    id = sub_id = 0;
    count = 0;
    timestamp = 0.0;
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using Point_lidar = PointXYZINormalCov;
using Plane = PlaneXYZNormalCov;
using Cloud = pcl::PointCloud<Point_lidar>;
using Cloud_ptr = Cloud::Ptr;
using PlaneCloud = pcl::PointCloud<Plane>;
using PlaneCloud_ptr = PlaneCloud::Ptr;

// LiDAR scan in frame L; timebase and timefinal are absolute Unix/ROS timestamps
struct LidarMeasurement {
  LidarMeasurement(const double timebase, const double timefinal, const int seq)
      : timebase(timebase),
        timefinal(timefinal),
        need_frontend(false),
        is_keyframe(false),
        seq(seq),
        cloud_ptr(new Cloud()),
        feature_indices(),
        correspondences(),
        new_landmarks(new PlaneCloud()) {}

  double timebase;
  double timefinal;
  bool need_frontend;
  bool is_keyframe;
  int seq;
  Cloud_ptr cloud_ptr;
  std::vector<size_t> feature_indices;
  std::vector<std::pair<size_t, int>> correspondences;
  PlaneCloud_ptr new_landmarks;
};

// PointCloud2 XYZIRT input; time is the offset from the scan header stamp, in seconds
struct EIGEN_ALIGN16 PointXYZIRT {
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY
  std::uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace gici

POINT_CLOUD_REGISTER_POINT_STRUCT(gici::PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z,
                                                             z)(float, intensity,
                                                                intensity)(std::uint16_t, ring,
                                                                           ring)(float, time, time))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    gici::PointXYZINormalCov,
    (float, x, x)(float, y, y)(float, z, z)(float, normal_x, normal_x)(float, normal_y, normal_y)(
        float, normal_z, normal_z)(float, intensity, intensity)(float, curvature, curvature)(
        double, d, d)(float[9], covariance, covariance))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    gici::PlaneXYZNormalCov,
    (float, x, x)(float, y, y)(float, z, z)(float, normal_x, normal_x)(float, normal_y, normal_y)(
        float, normal_z, normal_z)(float, intensity, intensity)(double, d, d)(
        double, timestamp, timestamp)(int, id, id)(int, count, count)(float[9], covariance,
                                                                      covariance))
