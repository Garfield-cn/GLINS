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

void gnssCallback(gici::GNSSMeasurement& data)
{
  LOG(INFO) << "* Got GNSS data at " << std::fixed << std::setprecision(9) << data.timestamp;
  int n_data = 0, n_precise = 0;
  for (auto it_j : data.satellites) {
    n_data++;
    if (it_j.second.sat_type == gici::SatEphType::Precise) n_precise++;
  }
  // LOG(INFO) << "n_data = " << n_data << ", n_precise = " << n_precise;
}

void imuCallback(gici::ImuMeasurement& data)
{
  static int cnt = 0;
  if (cnt++ % 400 == 0)
    LOG(INFO) << "^ Got IMU data at " << std::fixed << std::setprecision(9) << data.timestamp;
}

void imageCallback(double time, cv::Mat& image)
{
  static int cnt = 0;
  if (cnt++ % 10 == 0)
    LOG(INFO) << "$ Got Image data at " << std::fixed << std::setprecision(9) << time;
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

  gici::initializeSignalHandles();

  YAML::Node stream_config = config["stream"];
  gici::StreamHandle stream_handle(stream_config);

  gici::StreamHandle::GNSSCallback gnss_callback = std::bind(gnssCallback, std::placeholders::_1);
  stream_handle.setGNSSCallback(gnss_callback);
  gici::StreamHandle::IMUCallback imu_callback = std::bind(imuCallback, std::placeholders::_1);
  stream_handle.setIMUCallback(imu_callback);
  gici::StreamHandle::ImageCallback image_callback = std::bind(imageCallback, std::placeholders::_1, std::placeholders::_2);
  stream_handle.setImageCallback(image_callback);

  gici::SpinControl spin(1e-3);
  while (gici::SpinControl::ok()) {
    spin.sleep();
  }

  return 0;
}