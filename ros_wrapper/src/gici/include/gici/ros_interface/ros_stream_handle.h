/**
* @Function: Handle ROS streams
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#pragma once

#include <iostream>
#include <vector>
#include <unordered_map>
#include <functional>
#include <glog/logging.h>

#include "gici/ros_interface/ros_stream.h"

namespace gici {

class RosStreamHandle {
public:
  using GnssCallback = StreamHandle::GnssCallback;
  using ImuCallback = StreamHandle::ImuCallback;
  using ImageCallback = StreamHandle::ImageCallback;
  using SolutionCallback = EstimatingBase::SolutionCallback;

  RosStreamHandle(ros::NodeHandle& nh, YAML::Node& node);
  ~RosStreamHandle();

  // Set GNSS epoch data callback
  void setGnssCallback(const GnssCallback& gnss_callback, 
                       const std::vector<std::string>& tags) {
    for (size_t i = 0; i < streams_.size(); i++) {
      std::string tag = streams_[i]->getFormatorTag();
      if (std::find(tags.begin(), tags.end(), tag) != tags.end()) {
        streams_[i]->setGnssCallback(gnss_callback, tags);
      }
    }
  }

  // Set IMU epoch data callback
  void setImuCallback(const ImuCallback& imu_callback,
                      const std::vector<std::string>& tags) {
    for (size_t i = 0; i < streams_.size(); i++) {
      std::string tag = streams_[i]->getFormatorTag();
      if (std::find(tags.begin(), tags.end(), tag) != tags.end()) {
        streams_[i]->setImuCallback(imu_callback, tags);
      }
    }
  }

  // Set Image epoch data callback
  void setImageCallback(const ImageCallback& image_callback,
                        const std::vector<std::string>& tags) {
    for (size_t i = 0; i < streams_.size(); i++) {
      std::string tag = streams_[i]->getFormatorTag();
      if (std::find(tags.begin(), tags.end(), tag) != tags.end()) {
        streams_[i]->setImageCallback(image_callback, tags);
      }
    }
  }

  // Set Solution data callback
  void setSolutionCallback(const SolutionCallback& solution_callback,
                        const std::vector<std::string>& tags) {
    for (size_t i = 0; i < streams_.size(); i++) {
      std::string tag = streams_[i]->getFormatorTag();
      if (std::find(tags.begin(), tags.end(), tag) != tags.end()) {
        streams_[i]->setSolutionCallback(solution_callback, tags);
      }
    }
  }

  // Get streamer from given formator tag
  inline std::shared_ptr<RosStream> getStreamFromFormatorTag(std::string tag) {
    for (size_t i = 0; i < streams_.size(); i++) {
      if (streams_[i]->getFormatorTag() == tag) return streams_[i];
    }
    return nullptr;
  }

protected:
  std::vector<std::shared_ptr<RosStream>> streams_;
};

}
