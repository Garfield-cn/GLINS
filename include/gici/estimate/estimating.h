/**
* @Function: Handle estimator thread
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
#include "gici/gnss/rtk_estimator.h"
#include "gici/gnss/spp_estimator.h"
#include "gici/fusion/gnss_imu_lc_estimator.h"
#include "gici/fusion/rtk_imu_tc_estimator.h"
#include "gici/vision/image_types.h"

namespace gici {

class Estimating {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<Estimating>;

  using SolutionCallback = std::function<void(Solution&)>;

  Estimating(YAML::Node& node);
  ~Estimating();

  // Start thread
  void start();

  // Stop thread
  void stop();

  // GNSS data callback
  void gnssCallback(GnssMeasurement& data);

  // IMU data callback
  void imuCallback(std::string tag, ImuRole role, ImuMeasurement& data);

  // Image data callback
  void imageCallback(double timestamp, 
    std::string tag, CameraRole role, cv::Mat& image);

  // Set solution callback
  void setSolutionCallback(SolutionCallback solution_callback) {
    solution_callbacks_.push_back(solution_callback);
  } 

private:
  // Process RTK estimator
  void processRtk();

  // Process GNSS and IMU loosely couple estimator
  void processGnssImuLc();

  // Process RTK and IMU tightly couple estimator
  void processRtkImuTc();

  // Get timestamp to publish
  double getPublishTime();

  // Integrate solution to the timestamp we want to publish
  void integrateSolution();

  // Delete one GNSS measurement from front
  inline void popGnssMeasurement() {
    for (auto gnss_measurements : gnss_measurements_) {
      gnss_measurements.second.pop_front();
    }
  }

  // Delete one IMU measurement from front
  inline void popImuMeasurement() {
    for (auto imu_measurements : imu_measurements_) {
      imu_measurements.second.pop_front();
    }
  }

  // Delete one Image measurement from front
  void popImage() {
    for (auto images : images_) {
      images.second.pop_front();
    }
  }

	// Loop processing
	void run();

protected:
	// Thread handles
	std::unique_ptr<std::thread> thread_;
	std::mutex mutex_;
	bool quit_thread_ = false;
  double loop_duration_;
  // if user setted "loop_duration" as zero, we align the output rate to this input stream
  std::string loop_duration_align_tag_;
  bool aligned_new_data_;
  double aligned_new_timestamp_;

  // Estimator control
  std::string tag_;  // estimator tag
  EstimatorType type_;
  std::unique_ptr<RtkEstimator> rtk_estimator_;
  std::unique_ptr<GnssImuLcEstimator> gnss_imu_lc_estimator_;
  std::unique_ptr<RtkImuTcEstimator> rtk_imu_tc_estimator_;

  // Data buffers
  std::map<GnssRole, std::deque<GnssMeasurement>> gnss_measurements_;
  std::map<ImuRole, std::deque<ImuMeasurement>> imu_measurements_;
  std::map<CameraRole, std::deque<FramePtr>> images_;

  // Solutions
  Solution solution_;
  double publish_timestamp_;
  std::vector<SolutionCallback> solution_callbacks_;
};

}
