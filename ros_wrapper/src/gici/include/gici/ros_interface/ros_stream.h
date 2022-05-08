/**
* @Function: Handle ROS stream publish and subscribe
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#pragma once

#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <functional>
#include <glog/logging.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "gici/stream/stream_handle.h"
#include "gici/estimate/estimating.h"
#include "gici/ros_interface/ros_publisher.h"

namespace gici {

// ROS data format
enum class RosDataFormat {
  Image,
  Imu,
  GnssRaw,
  PoseStamped,
  PoseWithCovarianceStamped,
  Marker,
  Path
};

class RosStream {
public:
  using GnssCallback = StreamHandle::GnssCallback;
  using ImuCallback = StreamHandle::ImuCallback;
  using ImageCallback = StreamHandle::ImageCallback;
  using GnssCallbacks = StreamHandle::GnssCallbacks;
  using ImuCallbacks = StreamHandle::ImuCallbacks;
  using ImageCallbacks = StreamHandle::ImageCallbacks;
  using SolutionCallback = EstimatingBase::SolutionCallback;
  using SolutionCallbacks = std::vector<std::pair<SolutionCallback, std::vector<std::string>>>;

  RosStream(ros::NodeHandle& nh, YAML::Node& node, int istreamer);
  ~RosStream();

  // Set GNSS epoch data callback
  void setGnssCallback(const GnssCallback& gnss_callback, 
                       const std::vector<std::string>& tags) {
    gnss_callbacks_.push_back(std::make_pair(gnss_callback, tags));
  }

  // Set IMU epoch data callback
  void setImuCallback(const ImuCallback& imu_callback,
                      const std::vector<std::string>& tags) {
    imu_callbacks_.push_back(std::make_pair(imu_callback, tags));
  }

  // Set Image epoch data callback
  void setImageCallback(const ImageCallback& image_callback,
                        const std::vector<std::string>& tags) {
    image_callbacks_.push_back(std::make_pair(image_callback, tags));
  }

  // Set Solution data callback
  void setSolutionCallback(const SolutionCallback& solution_callback,
                        const std::vector<std::string>& tags) {
    solution_callbacks_.push_back(std::make_pair(solution_callback, tags));
  }

  // Send solution data to ROS topic
  void solutionOutputCallback(
    std::string tag, SolutionRole role, Solution& solution);

  // Send featured image to ROS topic
  void featuredImageCallback(FramePtr& frame);

  // Send features as marker to ROS topic
  void mapPointCallback(MapPtr& map);

  // Get formator tag
  std::string getFormatorTag() { return formator_tag_; }

private:
  // ROS callbacks
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void imuCallback(const sensor_msgs::ImuConstPtr& msg);
  // void gnssRawCallback()
  void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

protected:
  // Stream control
  std::string tag_;  // streamer tag
  std::string formator_tag_;  // formator tag
  StreamIOType io_type_;
  GnssCallbacks gnss_callbacks_;
  ImuCallbacks imu_callbacks_;
  ImageCallbacks image_callbacks_;
  SolutionCallbacks solution_callbacks_;
  std::vector<std::string> roles_;

  // ROS handles
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  ros::Subscriber subscriber_;
  std::unique_ptr<tf::TransformBroadcaster> tranform_broadcaster_;
  std::string frame_id_;
  std::string subframe_id_;
  std::string topic_name_;
  RosDataFormat data_format_;
  int queue_size_;
  std::unique_ptr<PathPublisher> path_publisher_;
  
};

}
