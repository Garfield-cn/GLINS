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

using namespace gici;

GnssMeasurement gnss_measurement_;
bool measurement_updated_ = false;

void gnssCallback(GnssMeasurement& data)
{
  LOG(INFO) << "* Got GNSS data at " << std::fixed << std::setprecision(9) << data.timestamp;
  
  if (data.role != GnssRole::Rover) return;
  if (!measurement_updated_) {
    gnss_measurement_ = data;
    measurement_updated_ = true;
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

  SppEstimatorOptions spp_estimator_options;
  spp_estimator_options.verbose = true;
  // spp_estimator_options.system_exclude.push_back('C');
  SppEstimator estimator(spp_estimator_options);

  YAML::Node stream_config = config["stream"];
  StreamHandle stream_handle(stream_config);

  StreamHandle::GnssCallback gnss_callback = std::bind(gnssCallback, std::placeholders::_1);
  stream_handle.setGnssCallback(gnss_callback);

  std::ofstream outfile;
  outfile.open("/home/cc/datasets/tmp/log.txt", std::ios::out | std::ios::trunc);
  outfile.close();

  SpinControl spin(1e-3);
  while (SpinControl::ok()) {

    if (measurement_updated_) {
      if (estimator.addGnssMeasurementAndState(gnss_measurement_)) {
        estimator.optimize();
        Eigen::Vector3d position = estimator.getPositionEstimate();
        Eigen::Vector2d clock;
        clock(0) = estimator.getClockEstimate('G');
        clock(1) = estimator.getClockEstimate('C');
        LOG(INFO) << std::fixed << position.transpose() << "  " << clock.transpose();
        outfile.open("/home/cc/datasets/tmp/log.txt", std::ios::out | std::ios::app);
        outfile << std::fixed << position.transpose() << std::endl;
        outfile.close();
      }
      measurement_updated_ = false;
    }

    spin.sleep();
  }

  return 0;
}