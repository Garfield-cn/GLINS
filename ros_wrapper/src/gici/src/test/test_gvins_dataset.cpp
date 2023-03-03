/**
* @Function: test
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include <iostream>
#include <stdio.h>
#include <cmath>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <gnss_comm/GnssTimeMsg.h>
#include <gnss_comm/GnssEphemMsg.h>
#include <gnss_comm/GnssGloEphemMsg.h>
#include <gnss_comm/GnssObsMsg.h>
#include <gnss_comm/GnssMeasMsg.h>
#include <gnss_comm/GnssBestXYZMsg.h>
#include <gnss_comm/GnssPVTSolnMsg.h>
#include <gnss_comm/GnssSvsMsg.h>
#include <gnss_comm/GnssTimePulseInfoMsg.h>
#include <gnss_comm/StampedFloat64Array.h>

#include "gici/ros_interface/ros_publisher.h"
#include "gici/estimate/estimating.h"
#include "gici/gnss/gnss_common.h"
#include "gici/ros_interface/ros_node_handle.h"
#include "gici/utility/signal_handle.h"
#include "gici/utility/node_option_handle.h"

using namespace gici;

// Global types
using EstimatorDataCallback = DataIntegrationBase::EstimatorDataCallback;
using EstimatorDataCallbacks = DataIntegrationBase::EstimatorDataCallbacks;

// Global variables
const std::string formator_tag_ = "ros_temp";
EstimatorDataCallbacks estimator_callbacks_;
GeoCoordinatePtr coordinate_;
ros::Publisher rtk_pose_pub_, rtk_path_pub_;
PathPublisher rtk_path_publisher_;

// GNSS position and velocity callback
void gnssSolutionCallback(const gnss_comm::GnssPVTSolnMsgConstPtr& msg)
{
  // The GNSS message always reaches before the IMU messages (for same timestamp). This is strange.
  // Hence, we add some latency for GNSS message time to make it more realistic.
  static const int shift_epoch = 4;
  static std::deque<Solution> solutions;

  // Convert GNSS solution data
  Solution solution;
  solution.timestamp = gnss_common::gtimeToDouble(
    gpst2utc(gpst2time(msg->time.week, msg->time.tow)));
  const std::map<uint8_t, GnssSolutionStatus> status_map = {
    {0, GnssSolutionStatus::Float}, {1, GnssSolutionStatus::DeadReckoning},
    {2, GnssSolutionStatus::Float}, {3, GnssSolutionStatus::Fixed},
    {4, GnssSolutionStatus::DeadReckoning}, {5, GnssSolutionStatus::Float}};
  solution.status = status_map.at(msg->fix_type);
  if (!msg->valid_fix && solution.status == GnssSolutionStatus::Fixed) {
    solution.status = GnssSolutionStatus::Float;
  }
  solution.num_satellites = msg->num_sv;
  Eigen::Vector3d lla(msg->latitude * D2R, msg->longitude * D2R, msg->altitude);
  if (coordinate_ == nullptr) {
    coordinate_.reset(new GeoCoordinate(lla, GeoType::LLA));
  }
  solution.coordinate = coordinate_;
  solution.pose.getPosition() = coordinate_->convert(lla, GeoType::LLA, GeoType::ENU);
  solution.covariance.setZero();
  solution.covariance.topLeftCorner(3, 3) = 
    cwiseSquare(Eigen::Vector3d(msg->h_acc, msg->h_acc, msg->v_acc)).asDiagonal();
  solution.speed_and_bias.head<3>() = 
    Eigen::Vector3d(msg->vel_e, msg->vel_n, -msg->vel_d);
  solution.covariance.block<3, 3>(6, 6) = 
    cwiseSquare(Eigen::Vector3d(msg->vel_acc, msg->vel_acc, msg->vel_acc)).asDiagonal();

  // Shift 
  solutions.push_back(solution);
  if (solutions.size() < shift_epoch) return;
  Solution current_solution = solutions.front();
  solutions.pop_front();
  
  // Call GNSS loosely processor
  for (auto it_gnss_callback : estimator_callbacks_) {
    EstimatorDataCluster estimator_data(
      current_solution, SolutionRole::PositionAndVelocity, "gnss");
    it_gnss_callback(estimator_data);
  }

  // Publish RTK trajectory
  publishPoseStamped(rtk_pose_pub_, 
    current_solution.pose, ros::Time(current_solution.timestamp), "World");
  rtk_path_publisher_.addPoseAndPublish(rtk_path_pub_, 
    current_solution.pose, ros::Time(current_solution.timestamp), "World");
}

int main(int argc, char** argv)
{
  // ros interface
  ros::init(argc, argv, "gici");
  ros::NodeHandle nh("~");

  // Additional ROS subscribers
  ros::Subscriber gnss_solution_sub = nh.subscribe<
    gnss_comm::GnssPVTSolnMsg>("/ublox_driver/receiver_pvt", 5, gnssSolutionCallback);

  // Publish RTK trajectory
  rtk_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("rtk", 5);
  rtk_path_pub_ = nh.advertise<nav_msgs::Path>("rtk_path", 100);

  // Get config file
  if (argc != 2) {
    std::cerr << "Invalid input variables! Supported variables are: "
              << "<path-to-executable> <path-to-config>" << std::endl;
    return -1;
  }
  std::string config_file_path = argv[1];
  YAML::Node yaml_node;
  try {
     yaml_node = YAML::LoadFile(config_file_path);
  } catch(YAML::BadFile &e) {
    std::cerr << "Unable to load config file!" << std::endl;
    return -1;
  }

  // Initialize glog for logging
  bool enable_logging = false;
  if (yaml_node["logging"].IsDefined() && 
      option_tools::safeGet(yaml_node["logging"], "enable", &enable_logging) && 
      enable_logging == true) {
    YAML::Node logging_node = yaml_node["logging"];
    google::InitGoogleLogging("gici");
    int min_log_level = 0;
    if (option_tools::safeGet(
        logging_node, "min_log_level", &min_log_level)) {
      FLAGS_minloglevel = min_log_level;
      FLAGS_stderrthreshold = min_log_level;
    }
    option_tools::safeGet(logging_node, "log_to_stderr", &FLAGS_logtostderr);
    option_tools::safeGet(logging_node, "file_directory", &FLAGS_log_dir);
  }

  // Initialize signal handles to catch faults
  initializeSignalHandles();

  // Organize nodes
  NodeOptionHandlePtr node_option_handle = 
    std::make_shared<NodeOptionHandle>(yaml_node);
  if (!node_option_handle->valid) {
    std::cerr << "Invalid configurations!" << std::endl;
    return -1;
  }

  // Initialize nodes
  std::unique_ptr<RosNodeHandle> node_handle = 
    std::make_unique<RosNodeHandle>(nh, node_option_handle);

  // Bind estimator inputs
  const std::vector<std::shared_ptr<EstimatingBase>>& estimatings = 
    node_handle->getEstimatings();
  for (const auto& estimating : estimatings) {
    EstimatorDataCallback estimator_callback = 
      std::bind(&EstimatingBase::estimatorDataCallback, 
        estimating.get(), std::placeholders::_1);
    estimator_callbacks_.push_back(estimator_callback);
  }

  // Loop
  ros::spin();

	return 0;
}

