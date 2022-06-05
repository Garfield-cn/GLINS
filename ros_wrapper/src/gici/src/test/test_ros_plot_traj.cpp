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

void plotFile(const char *file_path, ros::Publisher& pose_pub, 
  ros::Publisher& path_pub, PathPublisher& path_publisher)
{
  char buf[512];
	Transformation T_WS;

  FILE* file = fopen(file_path, "r");
  if(file == NULL){
      return;          
  }

  while (!feof(file))
  {
    fgets(buf, 512, file);
    double data[7];
    sscanf(buf, "%lf %lf %lf %lf %lf %lf %lf", &data[0], &data[1], &data[2], 
      &data[3], &data[4], &data[5], &data[6]);

    Eigen::Vector3d position(data[0], data[1], data[2]);
    Eigen::Quaterniond quat(data[3], data[4], data[5], data[6]);
    quat.normalize();
    T_WS = Transformation(position, quat);
    
    // publish body pose and transform
    ros::Time ros_time = ros::Time::now();
    Transformation pose = 
      Transformation(T_WS.getPosition(), 
                      T_WS.getEigenQuaternion());
    publishPoseStamped(pose_pub, pose, ros_time, "World");

    // publish path
    path_publisher.addPoseAndPublish(path_pub, pose, ros_time, "World");

    sleepms(100);
  }

  fclose(file);
}

int main(int argc, char** argv)
{
  // ros interface
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;

  // ROS publishers
	ros::Publisher pose_pub_1 = nh.advertise<geometry_msgs::PoseStamped>("/pose_1", 10);
	ros::Publisher path_pub_1 = nh.advertise<nav_msgs::Path>("/path_1", 1000);
	ros::Publisher pose_pub_2 = nh.advertise<geometry_msgs::PoseStamped>("/pose_2", 10);
	ros::Publisher path_pub_2 = nh.advertise<nav_msgs::Path>("/path_2", 1000);
  PathPublisher path_publisher_1;
  PathPublisher path_publisher_2;

  google::InitGoogleLogging("test");
  // FLAGS_log_dir = log_dir; 
  FLAGS_minloglevel = 0;
  FLAGS_logtostderr = true;
  FLAGS_stderrthreshold = 0;

  ros::Rate loop_rate(5);
	while (ros::ok())
	{	
    // enable ros topic handlers
    ros::spinOnce();

    plotFile("/home/cc/datasets/tmp/log_gi.txt", pose_pub_1, path_pub_1, path_publisher_1);
    path_publisher_1.clear();
    plotFile("/home/cc/datasets/tmp/log.txt", pose_pub_2, path_pub_2, path_publisher_2);
    path_publisher_2.clear();

		loop_rate.sleep();
	}

	return 0;
}

