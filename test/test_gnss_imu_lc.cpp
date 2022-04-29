/**
* @Function: Test
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/stream/stream_handle.h"
#include "gici/utility/signal_handle.h"
#include "gici/utility/spin_control.h"
#include "gici/gnss/gnss_types.h"
#include "gici/gnss/rtk_estimator.h"
#include "gici/gnss/spp_estimator.h"
#include "gici/fusion/gnss_imu_lc_estimator.h"
#include "gici/gnss/gnss_common.h"

using namespace gici;

GnssMeasurement gnss_measurement_rov_;
GnssMeasurement gnss_measurement_ref_;
bool gnss_measurement_updated_rov_ = false;
bool gnss_measurement_updated_ref_ = false;
std::unique_ptr<GnssImuLcEstimator> gnss_imu_lc_estimator_;

void gnssCallback(GnssMeasurement& data)
{
  if (!gnss_measurement_updated_rov_ && data.role == GnssRole::Rover) {
    gnss_measurement_rov_ = data;
    gnss_measurement_updated_rov_ = true;
  }

  if (!gnss_measurement_updated_ref_ && data.role == GnssRole::Reference) {
    gnss_measurement_ref_ = data;
    gnss_measurement_updated_ref_ = true;
  }
}

void imuCallback(ImuMeasurement& data)
{
  gnss_imu_lc_estimator_->addImuMeasurement(data);
}

int main(void)
{
  // google::InitGoogleLogging("test");
  // // FLAGS_log_dir = log_dir; 
  // FLAGS_minloglevel = 0;
  // FLAGS_logtostderr = true;
  // FLAGS_stderrthreshold = 0;

  // YAML::Node config;
  // try{
  //    config = YAML::LoadFile("/home/cc/linux/softwares/gici/option/gic_replay.yaml");
  // } catch(YAML::BadFile &e) {
  //   std::cout<<"read error!"<<std::endl;
  //   return -1;
  // }

  // initializeSignalHandles();

  // RtkEstimatorOptions rtk_estimator_options;
  // RtkEstimator rtk_estimator(rtk_estimator_options);

  // GnssImuLcEstimatorOptions gnss_imu_lc_estimator_options;
  // gnss_imu_lc_estimator_options.verbose = true;
  // gnss_imu_lc_estimator_options.gnss_extrinsic << 0.0, -0.2, -0.1;
  // gnss_imu_lc_estimator_ = std::make_unique<GnssImuLcEstimator>(gnss_imu_lc_estimator_options);

  // YAML::Node stream_config = config["stream"];
  // StreamHandle stream_handle(stream_config);

  // StreamHandle::GnssCallback gnss_callback = std::bind(gnssCallback, std::placeholders::_1);
  // stream_handle.setGnssCallback(gnss_callback);

  // StreamHandle::IMUCallback imu_callback = std::bind(imuCallback, std::placeholders::_1);
  // stream_handle.setIMUCallback(imu_callback);

  // std::ofstream outfile;
  // outfile.open("/home/cc/datasets/tmp/log.txt", std::ios::out | std::ios::trunc);
  // outfile.close();

  // bool gnss_positioning_successed = false;
  // GnssSolution gnss_solution;

  // SpinControl spin(1e-3);
  // while (SpinControl::ok()) {

  //   if (gnss_measurement_updated_rov_ && gnss_measurement_updated_ref_) {
  //     // For the first epoch, we add a position prior
  //     if (rtk_estimator.isFirstEpoch() && 
  //         !SppEstimator::setCoarsePosition(gnss_measurement_rov_)) {
  //       gnss_measurement_updated_rov_ = false;
  //       gnss_measurement_updated_ref_ = false;
  //       continue;
  //     }

  //     if (rtk_estimator.addGnssMeasurementAndState(
  //         gnss_measurement_rov_, gnss_measurement_ref_)) {
  //       rtk_estimator.optimize();
  //       Eigen::Vector3d position = rtk_estimator.getPositionEstimate();

  //       // LOG(INFO) << std::fixed << std::setprecision(9) << gnss_measurement_rov_.timestamp 
  //       //           << " " << std::fixed << position.transpose();

  //       gnss_solution = rtk_estimator.getSolution();
  //       gnss_positioning_successed = true;
  //     }
  //     gnss_measurement_updated_rov_ = false;
  //     gnss_measurement_updated_ref_ = false;
  //   }

  //   if (gnss_positioning_successed) {
  //     if (gnss_imu_lc_estimator_->addGnssMeasurementAndState(gnss_solution)) {
  //       gnss_imu_lc_estimator_->optimize();
  //       Transformation pose = gnss_imu_lc_estimator_->getPoseEstimate();
  //       LOG(INFO) << std::fixed << std::setprecision(9) << gnss_solution.timestamp 
  //                 << " " << std::fixed << pose.getPosition().transpose();
  //       GeoCoordinatePtr& coordinate = gnss_imu_lc_estimator_->getCoordinate();

  //       Eigen::Vector3d position = coordinate->convert(pose.getPosition(), GeoType::ENU, GeoType::ECEF);

  //       uint8_t buff[256];
  //       sol_t sol;
  //       for (int i = 0; i < 3; i++) sol.rr[i] = position(i);
  //       sol.time = gnss_common::doubleToGtime(gnss_solution.timestamp);
  //       int size = outnmea_rmc(buff, &sol);
  //       outnmea_gga(buff + size, &sol);
  //       outfile.open("/home/cc/datasets/tmp/log.txt", std::ios::out | std::ios::app);
  //       outfile << buff;
  //       outfile.close();
  //     }
  //     else {
  //       LOG(INFO) << "Failed to add measurement on LC estimator!";
  //     }
  //     gnss_positioning_successed = false;
  //   }

  //   spin.sleep();
  // }

  return 0;
}