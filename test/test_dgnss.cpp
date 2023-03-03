/**
* @Function: Test stream handle
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/stream/node_handle.h"
#include "gici/utility/signal_handle.h"
#include "gici/utility/spin_control.h"
#include "gici/gnss/gnss_types.h"
#include "gici/gnss/spp_estimator.h"
#include "gici/gnss/dgnss_estimator.h"
#include "gici/gnss/gnss_common.h"

using namespace gici;

GnssMeasurement gnss_measurement_rov_;
GnssMeasurement gnss_measurement_ref_;
bool measurement_updated_rov_ = false;
bool measurement_updated_ref_ = false;

void gnssCallback(GnssMeasurement& data)
{
  // LOG(INFO) << "* Got GNSS data at " << std::fixed << std::setprecision(9) << data.timestamp;

  if (!measurement_updated_rov_ && data.role == GnssRole::Rover) {
    gnss_measurement_rov_ = data;
    measurement_updated_rov_ = true;
  }

  if (!measurement_updated_ref_ && data.role == GnssRole::Reference) {
    gnss_measurement_ref_ = data;
    measurement_updated_ref_ = true;
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

  DgnssEstimatorOptions estimator_options;
  estimator_options.verbose = true;
  // estimator_options.system_exclude.push_back('C');
  DgnssEstimator estimator(estimator_options);

  YAML::Node stream_config = config["stream"];
  NodeHandle stream_handle(stream_config);

  NodeHandle::GnssCallback gnss_callback = std::bind(gnssCallback, std::placeholders::_1);
  stream_handle.setGnssCallback(gnss_callback);

  std::ofstream outfile;
  outfile.open("/home/cc/datasets/tmp/log.txt", std::ios::out | std::ios::trunc);
  outfile.close();

  SpinControl spin(1e-3);
  while (SpinControl::ok()) {

    if (measurement_updated_rov_ && measurement_updated_ref_) {
      
      if (SppEstimator::setCoarsePosition(gnss_measurement_rov_))
      if (estimator.addGnssMeasurementAndState(
          gnss_measurement_rov_, gnss_measurement_ref_)) {
        estimator.optimize();
        Eigen::Vector3d position = estimator.getPositionEstimate();
        LOG(INFO) << std::fixed << position.transpose();

        uint8_t buff[256];
        sol_t sol;
        for (int i = 0; i < 3; i++) sol.rr[i] = position(i);
        sol.time = gnss_common::doubleToGtime(gnss_measurement_rov_.timestamp);
        sol.stat = SOLQ_DGPS;
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