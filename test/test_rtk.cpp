/**
* @Function: Test stream handle
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/stream/stream_handle.h"
#include "gici/utility/signal_handle.h"
#include "gici/utility/spin_control.h"
#include "gici/gnss/gnss_types.h"
#include "gici/gnss/spp_estimator.h"
#include "gici/gnss/rtk_estimator.h"
#include "gici/gnss/gnss_common.h"
#include "gici/gnss/geodetic_coordinate.h"

using namespace gici;

#define SIM_DELAY_PERIOD 0

GNSSMeasurement gnss_measurement_rov_;
GNSSMeasurement gnss_measurement_ref_;
std::deque<GNSSMeasurement> gnss_measurements_ref_;
bool measurement_updated_rov_ = false;
bool measurement_updated_ref_ = false;

void gnssCallback(GNSSMeasurement& data)
{
  if (!measurement_updated_rov_ && data.role == GNSSRole::Rover) {
    gnss_measurement_rov_ = data;
    measurement_updated_rov_ = true;
  } 

  if (data.role == GNSSRole::Reference) {
    gnss_measurements_ref_.push_back(data);
    if (gnss_measurements_ref_.size() > SIM_DELAY_PERIOD + 1) gnss_measurements_ref_.pop_front();
  }

  if (!measurement_updated_ref_ && data.role == GNSSRole::Reference) {
    if (gnss_measurements_ref_.size() == SIM_DELAY_PERIOD + 1) {
      gnss_measurement_ref_ = gnss_measurements_ref_.front();
      measurement_updated_ref_ = true;
    }
  }
}

int main(void)
{
  google::InitGoogleLogging("test");
  // FLAGS_log_dir = log_dir; 
  FLAGS_minloglevel = 0;
  FLAGS_logtostderr = true;
  FLAGS_stderrthreshold = 0;

  YAML::Node config;
  try{
     config = YAML::LoadFile("/home/cc/linux/softwares/gici/option/gic_replay.yaml");
  } catch(YAML::BadFile &e) {
    std::cout<<"read error!"<<std::endl;
    return -1;
  }

  initializeSignalHandles();

  RTKEstimatorOptions estimator_options;
  estimator_options.verbose = true;
  // estimator_options.system_exclude.push_back('C');
  RTKEstimator estimator(estimator_options);

  YAML::Node stream_config = config["stream"];
  StreamHandle stream_handle(stream_config);

  StreamHandle::GNSSCallback gnss_callback = std::bind(gnssCallback, std::placeholders::_1);
  stream_handle.setGNSSCallback(gnss_callback);

  std::ofstream outfile;
  outfile.open("/home/cc/datasets/tmp/log.txt", std::ios::out | std::ios::trunc);
  outfile.close();
  outfile.open("/home/cc/datasets/tmp/log2.txt", std::ios::out | std::ios::trunc);
  outfile.close();

  SpinControl spin(1e-3);
  while (SpinControl::ok()) {

    if (measurement_updated_rov_ && measurement_updated_ref_) {
      // For the first epoch, we add a position prior
      if (estimator.isFirstEpoch() && 
          !SPPEstimator::setCoarsePosition(gnss_measurement_rov_)) {
        measurement_updated_rov_ = false;
        measurement_updated_ref_ = false;
        continue;
      }

      if (estimator.addGNSSMeasurementAndState(
          gnss_measurement_rov_, gnss_measurement_ref_)) {
        estimator.optimize();
        Eigen::Vector3d position = estimator.getPositionEstimate();
        GNSSSolutionStatus status = estimator.getSolutionStatus();

        LOG(INFO) << std::fixed << std::setprecision(9) << gnss_measurement_rov_.timestamp 
                  << " " << std::fixed << position.transpose();

        uint8_t buff[256];
        sol_t sol;
        for (int i = 0; i < 3; i++) sol.rr[i] = position(i);
        sol.time = gnss_common::doubleToGtime(gnss_measurement_rov_.timestamp);
        if (status == GNSSSolutionStatus::Fixed) sol.stat = SOLQ_FIX;
        else if (status == GNSSSolutionStatus::Float) sol.stat = SOLQ_FLOAT;
        else sol.stat = SOLQ_DGPS;
        int size = outnmea_rmc(buff, &sol);
        outnmea_gga(buff + size, &sol);
        outfile.open("/home/cc/datasets/tmp/log.txt", std::ios::out | std::ios::app);
        outfile << buff;
        outfile.close();
      }
      measurement_updated_rov_ = false;
      measurement_updated_ref_ = false;
    }

    spin.sleep();
  }

  return 0;
}