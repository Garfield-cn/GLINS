/**
* @Function: GNSS/IMU coupled estimator thread
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
#include "gici/fusion/gnss_imu_lc_estimator.h"
#include "gici/fusion/rtk_imu_tc_estimator.h"

namespace gici {

class GnssImuEstimating : public EstimatingBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<GnssImuEstimating>;

  GnssImuEstimating(YAML::Node& node);
  ~GnssImuEstimating();

  // GNSS data callback
  void gnssCallback(GnssMeasurement& data) override;

  // IMU data callback
  void imuCallback(std::string tag, ImuRole role, ImuMeasurement& data) override;

  // Solution callback from other estimators
  void solutionCallback(std::string tag, SolutionRole role, Solution& data) override;

  // Process funtion in every loop
  void process() override;

private:
  // Process initialization
  bool processInitialize();

  // Process GNSS and IMU loosely couple estimator
  bool processGnssImuLc();

  // Process RTK and IMU tightly couple estimator
  bool processRtkImuTc();

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

  // Backend processing
  void runBackend();

protected:
  // Backend thread handles
  std::unique_ptr<std::thread> backend_thread_;
  bool backend_finished_;

  // Estimator control
  std::unique_ptr<GnssImuLcEstimator> gnss_imu_lc_estimator_;
  std::unique_ptr<RtkImuTcEstimator> rtk_imu_tc_estimator_;
  std::shared_ptr<GnssImuInitialization> gnss_imu_initializer_;
  std::unique_ptr<RtkEstimator> rtk_estimator_;

  // Data buffers
  std::map<GnssRole, std::deque<GnssMeasurement>> gnss_measurements_;
  GnssMeasurement latest_gnss_measurement_ref_;
  std::map<ImuRole, ImuMeasurements> imu_measurements_;
  std::map<SolutionRole, std::deque<Solution>> solution_measurements_;

  // Solutions
  double backend_processing_timestamp_;
  double integrate_backend_timestamp_;
};

}
