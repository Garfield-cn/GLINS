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
#include "gici/gnss/dgnss_estimator.h"

using namespace gici;

GNSSMeasurement gnss_measurement_rov_;
GNSSMeasurement gnss_measurement_ref_;
bool measurement_updated_rov_ = false;
bool measurement_updated_ref_ = false;

void gnssCallback(GNSSMeasurement& data)
{
  LOG(INFO) << "* Got GNSS data at " << std::fixed << std::setprecision(9) << data.timestamp;

  if (!measurement_updated_rov_ && data.role == GNSSRole::Rover) {
    gnss_measurement_rov_ = data;
    measurement_updated_rov_ = true;
  }

  if (!measurement_updated_ref_ && data.role == GNSSRole::Reference) {
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

  DGNSSEstimatorOptions estimator_options;
  estimator_options.verbose = true;
  // estimator_options.system_exclude.push_back('C');
  DGNSSEstimator estimator(estimator_options);

  YAML::Node stream_config = config["stream"];
  StreamHandle stream_handle(stream_config);

  StreamHandle::GNSSCallback gnss_callback = std::bind(gnssCallback, std::placeholders::_1);
  stream_handle.setGNSSCallback(gnss_callback);

  std::ofstream outfile;
  outfile.open("/home/cc/datasets/tmp/log.txt", std::ios::out | std::ios::trunc);
  outfile.close();

  SpinControl spin(1e-3);
  while (SpinControl::ok()) {

    if (measurement_updated_rov_ && measurement_updated_ref_) {
      
      if (SPPEstimator::setCoarsePosition(gnss_measurement_rov_))
      if (estimator.addGNSSMeasurementAndState(
          gnss_measurement_rov_, gnss_measurement_ref_)) {
        estimator.optimize();
        Eigen::Vector3d position = estimator.getPositionEstimate();
        LOG(INFO) << std::fixed << position.transpose();
        outfile.open("/home/cc/datasets/tmp/log.txt", std::ios::out | std::ios::app);
        outfile << std::fixed << position.transpose() << std::endl;
        outfile.close();
      }
      measurement_updated_rov_ = false;
      measurement_updated_ref_ = false;
    }

    spin.sleep();
  }

  return 0;
}