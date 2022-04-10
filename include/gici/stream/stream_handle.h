/**
* @Function: Handle stream data output
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
#include <aslam/common/yaml-serialization.h>
#include <opencv2/opencv.hpp>

#include "gici/stream/streaming.h"
#include "gici/gnss/gnss_types.h"
#include "gici/imu/imu_types.h"
#include "gici/vision/image_types.h"

namespace gici {

class StreamHandle {
public:
  using Ptr = std::shared_ptr<StreamHandle>;

  using GNSSCallback = std::function<void(GNSSMeasurement&)>;
  using IMUCallback = std::function<void(ImuMeasurement&)>;
  using ImageCallback = std::function<void(double, cv::Mat&)>;

  // Behavior of a formator
  struct Behaviors {
    std::vector<std::string> role;
  };

  StreamHandle(YAML::Node& node);
  ~StreamHandle();

  // Set GNSS epoch data callback
  void setGNSSCallback(GNSSCallback& gnss_callback) {
    gnss_callback_ = gnss_callback;
  }

  // Set IMU epoch data callback
  void setIMUCallback(IMUCallback& imu_callback) {
    imu_callback_ = imu_callback;
  }

  // Set Image epoch data callback
  void setImageCallback(ImageCallback& image_callback) {
    image_callback_ = image_callback;
  }

private:
  // Handle GNSS data
  void handleGNSS(const std::string& tag, 
                  const DataFormat::GNSS::Ptr& gnss);

  // Handle IMU data
  void handleIMU(const std::string& tag, 
                 const DataFormat::IMU::Ptr& imu);

  // Handle Image data
  void handleImage(const std::string& tag, 
                   const DataFormat::Image::Ptr& image);

  // Data callback
  void dataCallback(const std::string& tag, const DataFormat::Ptr& data);

protected:
  std::vector<Streaming::Ptr> streamings_;
  std::unordered_map<std::string, Behaviors> behaviors_;
  std::mutex mutex_gnss_, mutex_imu_, mutex_image_;

  // Outside callbacks to handle epoch data
  GNSSCallback gnss_callback_;
  IMUCallback imu_callback_;
  ImageCallback image_callback_;

  // Local GNSS data to select ephemeris and SSR corrections
  DataFormat::GNSS::Ptr gnss_local_;
};

}
