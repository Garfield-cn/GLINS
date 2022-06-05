/**
* @Function: GNSS/IMU/Camera coupled estimator thread
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

#include "gici/estimate/estimating.h"
#include "gici/vision/feature_handler.h"
#include "gici/fusion/gnss_imu_camera_srr_estimator.h"

namespace gici {

class GnssImuCameraEstimating : public EstimatingBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<GnssImuCameraEstimating>;
  using FrameBundleCallback = std::function<void(const FrameBundlePtr&)>;
  using MapCallback = std::function<void(const MapPtr&)>;

  GnssImuCameraEstimating(YAML::Node& node);
  ~GnssImuCameraEstimating();

  // GNSS data callback
  void gnssCallback(GnssMeasurement& data) override;

  // IMU data callback
  void imuCallback(std::string tag, ImuRole role, ImuMeasurement& data) override;

  // Solution callback from other estimators
  void solutionCallback(std::string tag, SolutionRole role, Solution& data) override;

  // Image data callback
  void imageCallback(double timestamp, 
    std::string tag, CameraRole role, cv::Mat& image) override;

  // Process funtion in every loop
  void process() override;

  // Set frame bundle callback
  void setFrameBundleCallback(const FrameBundleCallback& callback) {
    frame_bundle_callbacks_.push_back(callback);
  }

  // Set map callback
  void setMapCallback(const MapCallback& callback) {
    map_callbacks_.push_back(callback);
  }

private:
  // Process initialization
  bool processInitialize();

  // Process Semi-Tightly couple using GNSS solution, IMU raw, and Image feature
  bool processGnssImuCameraSrr();

  // Process Tightly couple using GNSS raw, IMU raw, and Image feature
  bool processGnssImuCameraTc();

  // Get timestamp to publish
  double getPublishTime();

  // Convert solution to GNSS solution
  GnssSolution convertSolutionToGnssSolution(const Solution& solution);

  // Integrate solution to the timestamp we want to publish
  void integrateSolution();

  // Delete one GNSS measurement from front
  inline void popGnssMeasurement() {
    for (auto& gnss_measurements : gnss_measurements_) {
      if (gnss_measurements.second.size() == 0) continue;
      gnss_measurements.second.pop_front();
    }
  }

  // Delete IMU measurements from front
  inline void popImuMeasurement(double front_timestamp) {
    for (auto& imu_measurements : imu_measurements_) {
      if (imu_measurements.second.size() == 0) continue;
      while (imu_measurements.second.front().timestamp < front_timestamp) {
        imu_measurements.second.pop_front();
      }
    }
  }

  // Delete one solution measurement from front
  inline void popSolutionMeasurement() {
    for (auto& solution : solution_measurements_) {
      if (solution.second.size() == 0) continue;
      solution.second.pop_front();
    }
  }

  // Delete one Image measurement from front
  inline void popImage() {
    for (auto& images : images_) {
      if (images.second.size() == 0) continue;
      images.second.pop_front();
    }
  }

  // Delete one Frame from front
  inline void popFrameBundle() {
    frame_bundles_.pop_front();
  }

  // Update map points of in-windows keyframes
  void updateMap();

  // Camera frontend processing
  void runFrontend();

  // Backend processing
  void runBackend();

protected:
  // Backend thread handles
  std::unique_ptr<std::thread> backend_thread_;

  // Front thread handles
  std::unique_ptr<std::thread> frontend_thread_;
  std::mutex mutex_frontend_;

  // Sensor sequence control
  std::list<SensorType> sensor_sequence_;

  // Estimator control
  std::unique_ptr<GnssImuCameraSrrEstimator> gnss_imu_camera_srr_estimator_;
  std::shared_ptr<GnssImuCameraInitialization> gnss_imu_camera_initializer_;

  // Frontend control
  std::shared_ptr<FeatureHandler> feature_handler_;
  Solution camera_pose_;

  // Data buffers
  std::map<GnssRole, std::deque<GnssMeasurement>> gnss_measurements_;
  std::map<ImuRole, ImuMeasurements> imu_measurements_;
  std::map<SolutionRole, std::deque<Solution>> solution_measurements_;
  std::map<CameraRole, std::deque<std::pair<double, cv::Mat>>> images_;
  std::deque<FrameBundlePtr> frame_bundles_;

  // Solutions
  double backend_processing_timestamp_;
  double integrate_backend_timestamp_;

  // Callbacks
  std::vector<FrameBundleCallback> frame_bundle_callbacks_;
  std::vector<MapCallback> map_callbacks_;
};

}
