/**
* @Function: Multisensor estimation thread handle
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
#include "gici/gnss/spp_estimator.h"

namespace gici {

class MultiSensorEstimating : public EstimatingBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<MultiSensorEstimating>;

  MultiSensorEstimating(const NodeOptionHandlePtr& nodes, size_t i_estimator);
  ~MultiSensorEstimating();

  // Estimator data callback
  void estimatorDataCallback(EstimatorDataCluster& data) override;

  // Process funtion in every loop
  void process() override;

  // Update latest solution
  bool updateSolution() override;

private:
  // Handle time-propagation sensors
  void handleTimePropagationSensors(EstimatorDataCluster& data);

  // Handle non-time-propagation sensors
  void handleNonTimePropagationSensors(EstimatorDataCluster& data);

  // Handle sensors that need frontends
  void handleFrontendSensors(EstimatorDataCluster& data);

  // Put data from addin buffer to measurement buffer
  void putMeasurements();

  // Process estimator
  bool processEstimator();

  // Image frontend processing
  void runImageFrontend();

  // Measurement addin thread
  void runMeasurementAddin();

  // Backend processing
  void runBackend();

  // Estimator type checker
  inline bool estimatorTypeContains(
    SensorType sensor_type, EstimatorType estimator_type) {
    if (sensor_type == SensorType::GNSS) {
      return (estimator_type == EstimatorType::Spp || 
              estimator_type == EstimatorType::Sdgnss ||
              estimator_type == EstimatorType::Dgnss || 
              estimator_type == EstimatorType::Rtk || 
              estimator_type == EstimatorType::Ppp || 
              estimator_type == EstimatorType::GnssImuLc || 
              estimator_type == EstimatorType::RtkImuTc || 
              estimator_type == EstimatorType::GnssImuCameraSrr || 
              estimator_type == EstimatorType::RtkImuCameraRrr);
    }
    else if (sensor_type == SensorType::IMU) {
      return (estimator_type == EstimatorType::GnssImuLc || 
              estimator_type == EstimatorType::RtkImuTc || 
              estimator_type == EstimatorType::GnssImuCameraSrr || 
              estimator_type == EstimatorType::RtkImuCameraRrr);
    }
    else if (sensor_type == SensorType::Camera) {
      return (estimator_type == EstimatorType::GnssImuCameraSrr || 
              estimator_type == EstimatorType::RtkImuCameraRrr);
    }
    else return false;
  }

  // Check if there are more than one non-time-propagation sensors
  inline bool needTimeAlign(EstimatorType estimator_type) {
    return (estimator_type == EstimatorType::GnssImuCameraSrr || 
            estimator_type == EstimatorType::RtkImuCameraRrr);
  }

  // Check legality of estimator data
  inline bool estimatorDataIllegal(const EstimatorDataCluster& data) {
    int num_data_type = 0;
    if (data.gnss) num_data_type++;
    if (data.imu) num_data_type++;
    if (data.image) num_data_type++;
    if (data.frame_bundle) num_data_type++;
    if (data.solution) num_data_type++;
    if (num_data_type > 1) {
      LOG(WARNING) << "Estimator data cluster should contain only one "
                   << "data type at one pack!";
      return true;
    } 
    if (num_data_type == 0) {
      LOG(WARNING) << "Estimator data cluster has no data!";
      return true;
    }
    return false;
  }

  // Check if the data needs a frontend
  inline bool estimatorDataNeedFrontend(const EstimatorDataCluster& data) {
    return (data.image != nullptr);
  }

  // Check if the data will be used for time propagation (only support IMU)
  inline bool estimatorDataIsImu(const EstimatorDataCluster& data) {
    return (data.imu != nullptr);
  }

protected:
  // Measurement data collection thread handles
  std::unique_ptr<std::thread> measurement_thread_;

  // Backend thread handles
  std::unique_ptr<std::thread> backend_thread_;

  // Front thread handles
  std::unique_ptr<std::thread> image_frontend_thread_;

  // Frontend control
  std::shared_ptr<FeatureHandler> feature_handler_;

  // Coordinate
  bool force_initial_global_position_;
  Eigen::Vector3d initial_global_position_;
  std::unique_ptr<SppEstimator> spp_estimator_;

  // Data buffers
  std::deque<EstimatorDataCluster> measurements_;  // non propagate measurements
  int last_backend_pending_num_ = 0;
  // the frontend measurements should be processd by frontend, and the output of frontend will 
  // be inserted into measurements_.
  std::deque<EstimatorDataCluster> image_frontend_measurements_; 
  int last_image_pending_num_ = 0;
  // buffer to temporarily store measurements in case the input stream blocking
  std::deque<EstimatorDataCluster> measurement_addin_buffer_;
  // buffer to align timestamps of different sensor streams
  std::deque<EstimatorDataCluster> measurement_align_buffer_;
  double latest_imu_timestamp_;
  // mutex to lock buffers and processes
  std::mutex mutex_addin_, mutex_input_, mutex_image_input_;
  std::mutex mutex_output_;

  // Solutions
  bool backend_firstly_updated_ = false;
};

}
