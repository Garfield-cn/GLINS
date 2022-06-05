/**
* @Function: Estimator thread
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

#include "gici/utility/option.h"
#include "gici/estimate/estimator_types.h"
#include "gici/gnss/gnss_types.h"
#include "gici/imu/imu_types.h"
#include "gici/vision/image_types.h"

namespace gici {

class EstimatingBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using SolutionCallback = std::function<void(std::string, SolutionRole, Solution&)>;
  // We do not need to store tags here because if any client registered this callback, 
  // we will send solution to it without tag check. The tag check is defined to distingush
  // input streams, see EstimateHandle::bindWithStreams.
  using SolutionCallbacks = std::vector<SolutionCallback>;
  // output to ROS
  using FeaturedImageCallback = std::function<void(FramePtr&)>;
  using FeaturedImageCallbacks = std::vector<FeaturedImageCallback>;
  using MapPointCallback = std::function<void(MapPtr&)>;
  using MapPointCallbacks = std::vector<MapPointCallback>;

  EstimatingBase(YAML::Node& node);
  ~EstimatingBase();

  // Start thread
  void start();

  // Stop thread
  void stop();

  // GNSS data callback
  virtual void gnssCallback(GnssMeasurement& data) {
    return;
  }

  // IMU data callback
  virtual void imuCallback(
    std::string tag, ImuRole role, ImuMeasurement& data) {
    return;
  }

  // Image data callback
  virtual void imageCallback(double timestamp, 
    std::string tag, CameraRole role, cv::Mat& image) {
    return;
  }

  // Solution callback from other estimators
  virtual void solutionCallback(
    std::string tag, SolutionRole role, Solution& data) {
    return;
  }

  // Set solution callback
  void setSolutionCallback(SolutionCallback solution_callback) {
    solution_callbacks_.push_back(solution_callback);
  } 

  // Set featured image callback
  void setFeaturedImageCallback(FeaturedImageCallback featured_image_callback) {
    featured_image_callbacks_.push_back(featured_image_callback);
  } 

  // Set map point callback
  void setMapPointCallback(MapPointCallback map_point_callback) {
    map_point_callbacks_.push_back(map_point_callback);
  } 

  // Get tag
  std::string getTag() { return tag_; }

  // Process funtion in every loop
  virtual void process() = 0;

private:
	// Loop processing
	void run();

protected:
	// Thread handles
	std::unique_ptr<std::thread> thread_;
	std::mutex mutex_input_, mutex_process_, mutex_output_;
	bool quit_thread_ = false;
  double loop_duration_;
  // if user setted "loop_duration" as zero, we align the output rate to this input stream
  std::string loop_duration_align_tag_;
  bool aligned_new_data_;
  double aligned_new_timestamp_;

  // Estimator control
  std::string tag_;  // estimator tag
  SolutionRole role_;
  EstimatorType type_;

  // Solutions
  Solution solution_;
  ImuParameters imu_parameters_;
  double publish_timestamp_;
  SolutionCallbacks solution_callbacks_;
  FeaturedImageCallbacks featured_image_callbacks_;
  MapPointCallbacks map_point_callbacks_;
};

}
