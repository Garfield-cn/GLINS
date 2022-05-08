/**
* @Function: GNSS estimator thread
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
#include "gici/gnss/rtk_estimator.h"
#include "gici/gnss/spp_estimator.h"
#include "gici/gnss/dgnss_estimator.h"

namespace gici {

class GnssEstimating : public EstimatingBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<GnssEstimating>;

  GnssEstimating(YAML::Node& node);
  ~GnssEstimating();

  // GNSS data callback
  void gnssCallback(GnssMeasurement& data) override;

  // Process funtion in every loop
  void process() override;

private:
  // Process SPP estimator
  bool processSpp();

  // Process DGNSS estimator
  bool processDgnss();

  // Process RTK estimator
  bool processRtk();
  
  // Convert GNSS solution to solution
  Solution convertGnssSolutionToSolution(const GnssSolution& gnss_solution);

  // Delete one GNSS measurement from front
  inline void popGnssMeasurement() {
    for (auto& gnss_measurements : gnss_measurements_) {
      if (gnss_measurements.second.size() == 0) continue;
      gnss_measurements.second.pop_front();
    }
  }

protected:
  // Estimator control
  std::unique_ptr<SppEstimator> spp_estimator_;
  std::unique_ptr<DgnssEstimator> dgnss_estimator_;
  std::unique_ptr<RtkEstimator> rtk_estimator_;

  // Data buffers
  std::map<GnssRole, std::deque<GnssMeasurement>> gnss_measurements_;
  GnssMeasurement latest_gnss_measurement_ref_;
};

}
