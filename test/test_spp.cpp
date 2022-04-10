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

GNSSMeasurement gnss_measurement_;
bool measurement_updated_ = false;

void gnssCallback(GNSSMeasurement& data)
{
  LOG(INFO) << "* Got GNSS data at " << std::fixed << std::setprecision(9) << data.timestamp;
  
  if (data.role != GNSSRole::Rover) return;
  gnss_measurement_ = data;
  measurement_updated_ = true;
}

int main(void)
{
  google::InitGoogleLogging("feature_estimator");
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

  SPPEstimatorOptions spp_estimator_options;
  spp_estimator_options.verbose = true;
  SPPEstimator estimator(spp_estimator_options);

  YAML::Node stream_config = config["stream"];
  StreamHandle stream_handle(stream_config);

  StreamHandle::GNSSCallback gnss_callback = std::bind(gnssCallback, std::placeholders::_1);
  stream_handle.setGNSSCallback(gnss_callback);

  SpinControl spin(1e-3);
  while (SpinControl::ok()) {

    if (measurement_updated_) {
      estimator.addGNSSMeasurementAndState(gnss_measurement_);
      estimator.optimize();
      Eigen::Vector3d position = estimator.getPositionEstimate();
      LOG(INFO) << std::fixed << position.transpose();
      measurement_updated_ = false;
    }

    spin.sleep();
  }

  return 0;
}