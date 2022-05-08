/**
* @Function: GNSS/IMU/Camera coupled estimator thread
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/fusion/gnss_imu_camera_estimating.h"

#include "gici/gnss/spp_estimator.h"
#include "gici/imu/imu_common.h"
#include "gici/imu/imu_error.h"

// #include <opencv2/opencv.hpp>

namespace gici {

GnssImuCameraEstimating::GnssImuCameraEstimating(YAML::Node& node) : 
  EstimatingBase(node), backend_finished_(true), frontend_finished_(true),
  integrate_backend_timestamp_(0.0)
{
  // instantiate feature handler
  FeatureHandlerOptions frame_handler_options;
  option_tools::loadOptions(node, frame_handler_options);
  feature_handler_.reset(new FeatureHandler(frame_handler_options));

  // instantiate estimator
  if (type_ == EstimatorType::GnssImuCameraStc) {

  }
  else if (type_ == EstimatorType::RtkImuCameraTc) {

  }
}

GnssImuCameraEstimating::~GnssImuCameraEstimating()
{}

// GNSS data callback
void GnssImuCameraEstimating::gnssCallback(GnssMeasurement& data)
{
  gnss_measurements_[data.role].push_back(data);

  // Align timeline
  if (loop_duration_align_tag_ == data.tag) {
    aligned_new_timestamp_ = data.timestamp;
    aligned_new_data_ = true;
  }
}

// IMU data callback
void GnssImuCameraEstimating::imuCallback(
    std::string tag, ImuRole role, ImuMeasurement& data)
{
  imu_measurements_[role].push_back(data);

  // Currently we only support an IMU acts as major role
  if (role == ImuRole::Major) {
    // if (gnss_imu_lc_estimator_) {
    //   gnss_imu_lc_estimator_->addImuMeasurement(data);
    // }
    // if (rtk_imu_tc_estimator_) {
    //   rtk_imu_tc_estimator_->addImuMeasurement(data);
    // }
  }

  // Align timeline
  if (loop_duration_align_tag_ == tag) {
    aligned_new_timestamp_ = data.timestamp;
    aligned_new_data_ = true;
  }
}

// Solution callback from other estimators
void GnssImuCameraEstimating::solutionCallback(
  std::string tag, SolutionRole role, Solution& data)
{
  solution_measurements_[role].push_back(data);

  // Align timeline
  if (loop_duration_align_tag_ == tag) {
    aligned_new_timestamp_ = data.timestamp;
    aligned_new_data_ = true;
  }
}

// Image data callback
void GnssImuCameraEstimating::imageCallback(double timestamp, 
  std::string tag, CameraRole role, cv::Mat& image)
{
  images_[role].push_back(std::make_pair(timestamp, image.clone()));

  // Align timeline
  if (loop_duration_align_tag_ == tag) {
    aligned_new_timestamp_ = timestamp;
    aligned_new_data_ = true;
  }
}

// Process funtion in every loop
void GnssImuCameraEstimating::process()
{
  // Process frontend in a separated thread
  if (frontend_finished_) {
    if (frontend_thread_) frontend_thread_->join();
    frontend_thread_.reset(new std::thread(&GnssImuCameraEstimating::runFrontend, this));
  }

  // Process backend in a separated thread
  if (backend_finished_) {
    if (backend_thread_) backend_thread_->join();
    backend_thread_.reset(new std::thread(&GnssImuCameraEstimating::runBackend, this));
  }

  // Integrate solution to our desired publishing timestamp. Only the estimators with 
  // IMU sensors are applied.
  integrateSolution();
}

// Process Tightly couple using GNSS raw, IMU raw, and Image feature
bool GnssImuCameraEstimating::processGnssImuCameraStc()
{
  return true;
}

// Process RTK and IMU tightly couple estimator
bool GnssImuCameraEstimating::processGnssImuCameraTc()
{
  return true;
}

// Get timestamp to publish
double GnssImuCameraEstimating::getPublishTime() {
  // align to input stream
  if (loop_duration_ == 0) return aligned_new_timestamp_;
  // round current time
  else {
    double current_time = vk::Timer::getCurrentTime();
    int integer = round(current_time);
    double decimal = current_time - static_cast<double>(integer);
    double rounded_decimal = static_cast<double>(
      round(decimal / loop_duration_)) * loop_duration_;
    return (static_cast<double>(integer) + rounded_decimal);
  }
}

// Convert solution to GNSS solution
GnssSolution GnssImuCameraEstimating::convertSolutionToGnssSolution(
  const Solution& solution)
{
  GnssSolution gnss_solution;
  static int32_t id_static = 0;

  gnss_solution.timestamp = solution.timestamp;
  gnss_solution.id = ++id_static;
  gnss_solution.position = solution.backend.coordinate->convert(
    solution.pose.getPosition(), GeoType::ENU, GeoType::ECEF);
  Eigen::Matrix3d R_ecef_enu = solution.backend.coordinate->rotationMatrix(
    GeoType::ENU, GeoType::ECEF);
  gnss_solution.velocity = R_ecef_enu * solution.speed_and_bias.segment<3>(0);
  Eigen::Matrix<double, 6, 6> R_ecef_enu_double;
  R_ecef_enu_double.setZero();
  R_ecef_enu_double.topLeftCorner(3, 3) = R_ecef_enu;
  R_ecef_enu_double.bottomRightCorner(3, 3) = R_ecef_enu;
  Eigen::Matrix<double, 6, 6> covariance_position_velocity;
  covariance_position_velocity.topLeftCorner(3, 3) = 
    solution.covariance.block<3, 3>(0, 0);
  covariance_position_velocity.bottomRightCorner(3, 3) = 
    solution.covariance.block<3, 3>(6, 6);
  covariance_position_velocity.topRightCorner(3, 3) = 
    solution.covariance.block<3, 3>(0, 6);
  covariance_position_velocity.bottomLeftCorner(3, 3) = 
    solution.covariance.block<3, 3>(6, 0);
  gnss_solution.covariance = R_ecef_enu_double * 
    covariance_position_velocity * R_ecef_enu_double.transpose();
  gnss_solution.status = solution.status;
  gnss_solution.num_satellites = solution.num_satellites;
  gnss_solution.differential_age = solution.differential_age;

  return gnss_solution;
}

// Integrate solution to the timestamp we want to publish
void GnssImuCameraEstimating::integrateSolution()
{
  // Check if there is a backend solution
  if (solution_.backend.timestamp == 0.0) return;

  // Check aligned stream
  if (loop_duration_ == 0.0 && !aligned_new_data_) {
    return;
  }
  aligned_new_data_ = false;

  // Interpolate to current time or aligned stream time
  double publish_timestamp = getPublishTime();

  // Integration
  mutex_.lock();
  ImuParameters imu_parameters;
  // if (gnss_imu_lc_estimator_) {
  //   imu_parameters = gnss_imu_lc_estimator_->getImuParameters();
  // }
  // if (rtk_imu_tc_estimator_) {
  //   imu_parameters = rtk_imu_tc_estimator_->getImuParameters();
  // }

  // check if backend updated
  if (integrate_backend_timestamp_ != solution_.backend.timestamp) {
    solution_.timestamp = solution_.backend.timestamp;
    solution_.pose = solution_.backend.pose;
    solution_.speed_and_bias = solution_.backend.speed_and_bias;
    integrate_backend_timestamp_ = solution_.backend.timestamp;
  }

  ImuError::propagation(
    imu_measurements_[ImuRole::Major], imu_parameters, 
    solution_.pose, solution_.speed_and_bias, solution_.timestamp, 
    publish_timestamp, nullptr, nullptr);
  solution_.pose.getRotation().normalize();
  solution_.timestamp = publish_timestamp;

  // If integration period larger than 2 s, we set the status as Dead Reckoning
  if (solution_.timestamp - solution_.backend.timestamp > 2.0) {
    solution_.status = GnssSolutionStatus::DeadReckoning;
  }
  mutex_.unlock();

  // Delete used IMU measurements
  double front_timestamp = 
    std::min(publish_timestamp, backend_processing_timestamp_);
  popImuMeasurement(front_timestamp - 0.1);
}

// Camera frontend processing
void GnssImuCameraEstimating::runFrontend()
{
  // Check if we have new image data
  if (images_.size() == 0) return;
  // currently we only support image with Mono role
  if (!feature_handler_->isFirstFrame() && 
      feature_handler_->getFrameBundle()->getMinTimestampSeconds() >= 
      images_[CameraRole::Mono].back().first) {
    return;
  }

  frontend_finished_ = false;

  double& timestamp = images_[CameraRole::Mono].back().first;
  cv::Mat& image = images_[CameraRole::Mono].back().second;
  if (feature_handler_->addImageBundle({image}, timestamp)) {
    frame_bundles_.push_back(feature_handler_->getFrameBundle());

    // call featured image output (for ROS)
    for (auto featured_image_callback : featured_image_callbacks_) {
      FramePtr frame = frame_bundles_.back()->at(0);
      featured_image_callback(frame);
    }
  }

  frontend_finished_ = true;
}

// Backend processing
void GnssImuCameraEstimating::runBackend()
{
  backend_finished_ = false;

  // if (type_ == EstimatorType::GnssImuLc) {
  //   processGnssImuLc();
  // }
  // else if (type_ == EstimatorType::RtkImuTc) {
  //   processRtkImuTc();
  // }

  backend_finished_ = true;
}

}
