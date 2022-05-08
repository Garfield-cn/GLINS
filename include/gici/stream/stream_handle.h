/**
* @Function: Handle stream data input, log, and output
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
#include <opencv2/opencv.hpp>

#include "gici/stream/streaming.h"
#include "gici/gnss/gnss_types.h"
#include "gici/imu/imu_types.h"
#include "gici/vision/image_types.h"

namespace gici {

class StreamHandle {
public:
  using GnssCallback = std::function<void(GnssMeasurement&)>;
  using ImuCallback = std::function<void(std::string, ImuRole, ImuMeasurement&)>;
  using ImageCallback = std::function<void(double, std::string, CameraRole, cv::Mat&)>;
  using GnssCallbacks = std::vector<std::pair<GnssCallback, std::vector<std::string>>>;
  using ImuCallbacks = std::vector<std::pair<ImuCallback, std::vector<std::string>>>;
  using ImageCallbacks = std::vector<std::pair<ImageCallback, std::vector<std::string>>>;

  // Behavior of a formator
  struct Behaviors {
    std::vector<std::string> role;
  };

  StreamHandle(YAML::Node& node);
  ~StreamHandle();

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

  // Get streamer from given formator tag
  inline std::shared_ptr<Streaming> getStreamFromFormatorTag(std::string tag) {
    for (size_t i = 0; i < streamings_.size(); i++) {
      if (streamings_[i]->hasFormatorTag(tag)) return streamings_[i];
    }
    return nullptr;
  }

private:
  // Handle GNSS data
  void handleGNSS(const std::string& tag, 
                  const std::shared_ptr<DataCluster::GNSS>& gnss);

  // Handle IMU data
  void handleIMU(const std::string& tag, 
                 const std::shared_ptr<DataCluster::IMU>& imu);

  // Handle Image data
  void handleImage(const std::string& tag, 
                   const std::shared_ptr<DataCluster::Image>& image);

  // Data callback
  void dataCallback(const std::string& tag, const std::shared_ptr<DataCluster>& data);

protected:
  std::vector<std::shared_ptr<Streaming>> streamings_;
  std::unordered_map<std::string, Behaviors> behaviors_;
  std::mutex mutex_gnss_, mutex_imu_, mutex_image_;

  // Outside callbacks to handle epoch data
  GnssCallbacks gnss_callbacks_;
  ImuCallbacks imu_callbacks_;
  ImageCallbacks image_callbacks_;

  // Local GNSS data to select ephemeris and SSR corrections
  std::shared_ptr<DataCluster::GNSS> gnss_local_;
};

}
