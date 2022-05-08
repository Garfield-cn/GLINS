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
#include "gici/ros_interface/ros_publisher.h"

using namespace gici;

ros::Publisher pose_pub_;
ros::Publisher path_pub_;
ros::Publisher gnss_path_pub_;
PathPublisher path_publisher_;
PathPublisher gnss_path_publisher_;

int main(int argc, char** argv)
{
  // ros interface
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;

  // ROS publishers
	pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/pose", 10);
	path_pub_ = nh.advertise<nav_msgs::Path>("/path", 1000);
  gnss_path_pub_ = nh.advertise<nav_msgs::Path>("/gnss_path", 1000);

	// It should be declared after ros::init
	tf::TransformBroadcaster body_broadcaster;

  google::InitGoogleLogging("test");
  // FLAGS_log_dir = log_dir; 
  FLAGS_minloglevel = 0;
  FLAGS_logtostderr = true;
  FLAGS_stderrthreshold = 0;

  FILE* file = fopen("/home/cc/datasets/tmp/log.txt", "r");
  if(file == NULL){
      return 0;          
  }

  char buf[512];
	Transformation T_WS;
  Transformation gnss_pose;

  ros::Rate loop_rate(5);
	while (ros::ok() && !feof(file))
	{	
    // enable ros topic handlers
    ros::spinOnce();

    fgets(buf, 512, file);
    double data[10];
    sscanf(buf, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &data[0], &data[1], &data[2], 
      &data[3], &data[4], &data[5], &data[6], &data[7], &data[8], &data[9]);

    Eigen::Vector3d position(data[0], data[1], data[2]);
    Eigen::Quaterniond quat(data[6], data[7], data[8], data[9]);
    quat.normalize();
    T_WS = Transformation(position, quat);
    
    // publish body pose and transform
    ros::Time ros_time = ros::Time::now();
    Transformation pose = 
      Transformation(T_WS.getPosition(), 
                     T_WS.getEigenQuaternion());
    publishPoseWithTransform(pose_pub_, body_broadcaster, pose, ros_time, "Body", "World");

    // publish path
    path_publisher_.addPoseAndPublish(path_pub_, pose, ros_time, "World");

    gnss_pose = Transformation(Eigen::Vector3d(data[3], data[4], data[5]), Eigen::Quaterniond::Identity());
    gnss_path_publisher_.addPoseAndPublish(gnss_path_pub_, gnss_pose, ros_time, "World");

		loop_rate.sleep();
	}

  fclose(file);

	return 0;
}

