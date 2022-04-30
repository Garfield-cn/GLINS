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
#include <yaml-cpp/yaml.h>

#include "gici/stream/stream_handle.h"
#include "gici/utility/signal_handle.h"
#include "gici/utility/spin_control.h"
#include "gici/gnss/gnss_types.h"
#include "gici/imu/imu_types.h"
#include "gici/imu/imu_common.h"
#include "gici/imu/imu_error.hpp"
#include "gici/ros_interface/publisher.h"

using namespace gici;

ros::Publisher imu_raw_pub_;
ros::Publisher pose_pub_;
ros::Publisher path_pub_;
PathPublisher path_publisher_;

ImuMeasurements imu_measurements_;
ImuParameters imu_parameters_;

void imuCallback(ImuMeasurement& data)
{
  // data.angular_velocity = -data.angular_velocity.eval();
  imu_parameters_.g = 9.7940;
  ImuMeasurement data_convert = data;
  // data_convert.linear_acceleration *= imu_parameters_.g / 9.8;
  // data_convert.linear_acceleration.x() = data.linear_acceleration.z();
  // data_convert.linear_acceleration.y() = -data.linear_acceleration.y();
  // data_convert.linear_acceleration.z() = data.linear_acceleration.x();
  // data_convert.angular_velocity.x() = data.angular_velocity.z();
  // data_convert.angular_velocity.y() = -data.angular_velocity.y();
  // data_convert.angular_velocity.z() = data.angular_velocity.x();
  // data_convert.angular_velocity = -data_convert.angular_velocity.eval();
  imu_measurements_.push_back(data_convert);

  sensor_msgs::Imu imu_msg;
  imu_msg.header.stamp = ros::Time(data.timestamp);
  imu_msg.linear_acceleration.x = data.linear_acceleration(0);
  imu_msg.linear_acceleration.y = data.linear_acceleration(1);
  imu_msg.linear_acceleration.z = data.linear_acceleration(2);
  imu_msg.angular_velocity.x = data.angular_velocity(0);
  imu_msg.angular_velocity.y = data.angular_velocity(1);
  imu_msg.angular_velocity.z = data.angular_velocity(2);
  imu_raw_pub_.publish(imu_msg);
}

int main(int argc, char** argv)
{
  // ros interface
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;

  // ROS publishers
	pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/pose", 10);
	path_pub_ = nh.advertise<nav_msgs::Path>("/path", 1000);
  imu_raw_pub_ = nh.advertise<sensor_msgs::Imu>("/imu_raw", 100);

	// It should be declared after ros::init
	tf::TransformBroadcaster body_broadcaster;

  google::InitGoogleLogging("test");
  // FLAGS_log_dir = log_dir; 
  FLAGS_minloglevel = 0;
  FLAGS_logtostderr = true;
  FLAGS_stderrthreshold = 0;

  YAML::Node config;
  try{
     config = YAML::LoadFile("/home/cc/linux/softwares/gici/option/gic_replay.yaml");
  } catch(YAML::BadFile &e) {
    std::cout<<"read error!"<<std::endl;
    return -1;
  }

	initializeSignalHandles();

  YAML::Node stream_config = config["stream"];
  StreamHandle stream_handle(stream_config);

  StreamHandle::ImuCallback imu_callback = std::bind(imuCallback, std::placeholders::_1);
  stream_handle.setImuCallback(imu_callback);

  std::ofstream outfile;
  outfile.open("/home/cc/datasets/tmp/log.txt", std::ios::out | std::ios::trunc);
  outfile.close();

	Transformation T_WS;
  SpeedAndBias speed_and_bias = SpeedAndBias::Zero();
  double timestamp = 0.0;
  bool is_initialized = false;
  const int init_window = 6000;

  ros::Rate loop_rate(10);
	while (ros::ok())
	{	
    // enable ros topic handlers
    ros::spinOnce();

    if (imu_measurements_.size() > 0)
    {
      double cur_timestamp = imu_measurements_.back().timestamp;
      if (!is_initialized && imu_measurements_.size() > init_window) {
        timestamp = cur_timestamp;
        if (initPoseAndBiases(imu_measurements_, imu_parameters_.g, T_WS, speed_and_bias)) {

          // Eigen::Vector3d ypr = T_WS.getEigenQuaternion().matrix().eulerAngles(2, 1, 0);
          // Eigen::Quaterniond quat = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) *
          //   Eigen::AngleAxisd(ypr(1), Eigen::Vector3d::UnitY()) * 
          //   Eigen::AngleAxisd(ypr(2), Eigen::Vector3d::UnitX());
          // T_WS = Transformation(T_WS.getPosition(), quat);

          is_initialized = true;
          LOG(INFO) << "Initialized!!!!!!!!!!!!!!!!";
        }
      }
      else if (is_initialized && cur_timestamp > timestamp) {
        ImuError::propagation(
          imu_measurements_, imu_parameters_, T_WS, speed_and_bias,
          timestamp, cur_timestamp, nullptr, nullptr);
        T_WS.getRotation().normalize();
        while (imu_measurements_.front().timestamp < cur_timestamp) {
          imu_measurements_.pop_front();
        }
        timestamp = cur_timestamp;
      }
    }
    
    // publish body pose and transform
    ros::Time ros_time(timestamp);
    Transformation pose = 
      Transformation(T_WS.getPosition(), 
                     T_WS.getEigenQuaternion());
    publishPoseWithTransform(pose_pub_, body_broadcaster, pose, ros_time, "Body", "World");

    // publish path
    path_publisher_.addPoseAndPublish(path_pub_, pose, ros_time, "World");

		loop_rate.sleep();
	}

	return 0;
}

