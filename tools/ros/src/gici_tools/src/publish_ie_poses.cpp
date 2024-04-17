/**
* @Function: Publish poses
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/ros_utility/ros_publisher.h"

using namespace gici;

#define DEFAULT_PRECISION 1.0e-4

// Check equal for float types
template<typename FloatT>
inline bool checkEqual(FloatT x, FloatT y, 
                       float precision = DEFAULT_PRECISION) {
  return (fabs(x - y) < precision);
}

// Check float type equals zero
template<typename FloatT>
inline bool checkZero(FloatT x, 
                      float precision = DEFAULT_PRECISION) {
  return checkEqual<FloatT>(x, 0.0, precision);
}

// Check equal for float matrix
template<typename FloatT, int Rows, int Cols>
inline bool checkEqual(Eigen::Matrix<FloatT, Rows, Cols> mat_x,
                       Eigen::Matrix<FloatT, Rows, Cols> mat_y,
                       float precision = DEFAULT_PRECISION) {
  bool has_none_equal = false;
  for (size_t i = 0; i < mat_x.rows(); i++) {
    for (size_t j = 0; j < mat_x.cols(); j++) {
      if (!checkEqual(mat_x(i, j), mat_y(i, j), precision)) {
        has_none_equal = true; break;
      }
    }
  }
  return !has_none_equal;
}

// Square
template<typename T>
inline T square(T x) {
  return (x * x);
}

// Convert quartion to euler angle
Eigen::Vector3d quaternionToEulerAngle(const Eigen::Quaternion<double>& q)
{
  double roll, pitch, yaw;

  // roll (x-axis rotation)
  double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  roll = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
  if (fabs(sinp) >= 1)
  pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
  pitch = asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  yaw = atan2(siny_cosp, cosy_cosp);

  return Eigen::Vector3d(roll, pitch, yaw);
}

// Convert euler angle to quarternion
Eigen::Quaternion<double> eulerAngleToQuaternion(const Eigen::Vector3d& rpy)
{
  // Abbreviations for the various angular functions
  double roll = rpy(0);
  double pitch = rpy(1);
  double yaw = rpy(2);
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  Eigen::Quaternion<double> q;
  q.w() = cr * cp * cy + sr * sp * sy;
  q.x() = sr * cp * cy - cr * sp * sy;
  q.y() = cr * sp * cy + sr * cp * sy;
  q.z() = cr * cp * sy - sr * sp * cy;

  return q;
}

// Euler angle to rotation matrix
enum class Order {
  ZYX,
  ZXY
};
Eigen::Matrix3d eulerAngleToRotationMatrix(const Eigen::Vector3d& rpy, Order order)
{
  double x = rpy(0);
  double y = rpy(1);
  double z = rpy(2);
  Eigen::Matrix3d Rx, Ry, Rz; 
  Rx << 1.0, 0.0, 0.0, 
        0.0, cos(x), -sin(x),
        0.0, sin(x), cos(x);
  Ry << cos(y), 0.0, sin(y),
        0.0, 1.0, 0.0,
        -sin(y), 0.0, cos(y);
  Rz << cos(z), -sin(z), 0.0,
        sin(z), cos(z), 0.0,
        0.0, 0.0, 1.0;
  if (order == Order::ZYX) return Rz * Ry * Rx;
  if (order == Order::ZXY) return Rz * Rx * Ry;
  return Eigen::Matrix3d::Zero();
}

// Rotation matrix to euler angle
Eigen::Vector3d rotationMatrixToEulerAngle(const Eigen::Matrix3d& R, Order order)
{
  Eigen::Vector3d euler = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R_varify;
  if (order == Order::ZYX) 
  {
    // Assume cos(y) >= 0
    euler(0) = atan2(R(2,1), R(2,2));
    euler(1) = atan2(-R(2,0), sqrt(square(R(2,1))+square(R(2,2))));
    euler(2) = atan2(R(1,0), R(0,0));
    R_varify = eulerAngleToRotationMatrix(euler, order);
    if (checkEqual(R_varify, R)) return euler;

    // Assume cos(y) < 0
    euler(0) = atan2(-R(2,1), -R(2,2));
    euler(1) = atan2(-R(2,0), -sqrt(square(R(2,1))+square(R(2,2))));
    euler(2) = atan2(-R(1,0), -R(0,0));
    R_varify = eulerAngleToRotationMatrix(euler, order);
    if (checkEqual(R_varify, R)) return euler;
  }
  if (order == Order::ZXY)
  {
    // Assume cos(y) >= 0
    euler(0) = atan2(R(2,1), sqrt(square(R(2,0))+square(R(2,2))));
    euler(1) = atan2(-R(2,0), R(2,2));
    euler(2) = atan2(-R(0,1), R(1,1));
    R_varify = eulerAngleToRotationMatrix(euler, order);
    if (checkEqual(R_varify, R)) return euler;

    // Assume cos(y) < 0
    euler(0) = atan2(R(2,1), -sqrt(square(R(2,0))+square(R(2,2))));
    euler(1) = atan2(R(2,0), -R(2,2));
    euler(2) = atan2(R(0,1), -R(1,1));
    R_varify = eulerAngleToRotationMatrix(euler, order);
    if (checkEqual(R_varify, R)) return euler;
  }

  std::cout << "rotationMatrixToEulerAngle: No valid solution!" << std::endl;
  return euler;
}

// Convert IE euler angle to gici euler angle
// IE: roll, pitch, heading (negative yaw), rotation sequence is zxy.
// gici: roll, pitch, yaw, rotation sequence is zyx
Eigen::Vector3d ieEulerToGiciEuler(const Eigen::Vector3d rph)
{
  Eigen::Vector3d rpy_zxy; 
  rpy_zxy << rph(1), rph(0), -rph(2);
  Eigen::Matrix3d R = eulerAngleToRotationMatrix(rpy_zxy, Order::ZXY);
  Eigen::Quaterniond q(R);
  return quaternionToEulerAngle(q);
}

// Convert gici euler angle to IE euler angle 
Eigen::Vector3d giciEulerToIeEuler(const Eigen::Vector3d rpy)
{
  Eigen::Matrix3d R = eulerAngleToRotationMatrix(rpy, Order::ZYX);
  Eigen::Vector3d rpy_zxy = rotationMatrixToEulerAngle(R, Order::ZXY);
  Eigen::Vector3d rph;
  rph << rpy_zxy(1), rpy_zxy(0), -rpy_zxy(2);
  return rph;
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "gici");
  ros::NodeHandle nh("~");

  // Get file
  if (argc != 4) {
    std::cerr << "Invalid input variables! Supported variables are: "
              << "<path-to-executable> <path-to-file> <topic-name> <time-duration>" << std::endl;
    return -1;
  }
  std::string file_path = argv[1];
  std::string topic_name = argv[2];
  double duration = atof(argv[3]);
  
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/" + topic_name + "/path", 10);
  ros::Publisher pose_pub = nh.advertise<nav_msgs::Odometry>("/" + topic_name + "/pose", 3);

  std::unique_ptr<tf::TransformBroadcaster> tranform_broadcaster_ = std::make_unique<tf::TransformBroadcaster>();
  std::unique_ptr<PathPublisher> path_publisher_ = std::make_unique<PathPublisher>();

  char buf[1024];
  FILE *fp_ie = fopen(file_path.data(), "r");

  // Eigen::Vector3d acc(0.0, 0.0, 1.0);
  // Eigen::Quaterniond rot = eulerAngleToQuaternion(Eigen::Vector3d(-60.0, 0.0, 180.0) * D2R);
  // acc = rot.inverse() * acc;
  // std::cout << std::fixed << acc.transpose() << std::endl;
  // return -1;

  ros::Rate r(1.0 / duration);
  Eigen::Vector3d ref_ecef = Eigen::Vector3d::Zero();
  while (ros::ok() && !(fgets(buf, 1024 * sizeof(char), fp_ie) == NULL))
  {
    if (buf[0] == 'W') continue;
    std::vector<std::string> strs;
    strs.push_back("");
    for (int i = 0; i < strlen(buf); i++) {
      if (buf[i] == ' ' || buf[i] == '\t') {
        if (strs.back().size() > 0) strs.push_back("");
        continue;
      }
      if (buf[i] == '\r' || buf[i] == '\n') break;
      strs.back().push_back(buf[i]);
    }
    int gpsweek, status;
    double gpstow, lla[3], vel[3], rph[3], pos[3];
    gpsweek = atoi(strs[0].data());
    gpstow = atof(strs[1].data());
    lla[1] = atof(strs[3].data()) * D2R;
    lla[0] = atof(strs[4].data()) * D2R;
    lla[2] = atof(strs[5].data());
    rph[2] = atof(strs[6].data()) * D2R;
    rph[1] = atof(strs[7].data()) * D2R;
    rph[0] = atof(strs[8].data()) * D2R;

    pos2ecef(lla, pos);

    gtime_t gtime = gpst2time(gpsweek, gpstow);
    double time = (double)gtime.time + gtime.sec;

    Eigen::Vector3d att = ieEulerToGiciEuler(Eigen::Map<Eigen::Vector3d>(rph));
    Eigen::Quaterniond q = eulerAngleToQuaternion(att);

    Eigen::Vector3d p_enu = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_ecef = Eigen::Vector3d::Zero();
    pos2ecef(lla, p_ecef.data());
    if (ref_ecef == Eigen::Vector3d::Zero()) {
      ref_ecef = p_ecef;
    }
    double ref_position[3];
    ecef2pos(ref_ecef.data(), ref_position);
    Eigen::Vector3d dp = p_ecef - ref_ecef;
    ecef2enu(ref_position, dp.data(), p_enu.data());

    /// !!!!!!
    // Eigen::Quaterniond rot = eulerAngleToQuaternion(Eigen::Vector3d(60.0, 0.0, 180.0) * D2R);
    // q = q * rot.inverse();

    Transformation T_WS(p_enu, q.normalized());

    path_publisher_->addPoseAndPublish(path_pub, T_WS, ros::Time(time), "World");
    publishOdometry(pose_pub, *tranform_broadcaster_, T_WS, 
      Eigen::Vector3d::Zero(), Eigen::Matrix<double, 9, 9>::Zero(), 
      ros::Time(time), "World", "Body");

    r.sleep();
  }

  fclose(fp_ie);

  return 0;
}