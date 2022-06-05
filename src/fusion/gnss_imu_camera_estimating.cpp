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
#include "gici/utility/spin_control.h"

// #include <opencv2/opencv.hpp>

namespace gici {

GnssImuCameraEstimating::GnssImuCameraEstimating(YAML::Node& node) : 
  EstimatingBase(node), integrate_backend_timestamp_(0.0)
{
  // instantiate feature handler
  FeatureHandlerOptions frame_handler_options;
  if (!node["feature_handler_options"].IsDefined()) {
    LOG(ERROR) << "Unable to load feature_handler_options!";
    return;
  }
  YAML::Node feature_node = node["feature_handler_options"];
  option_tools::loadOptions(feature_node, frame_handler_options);
  feature_handler_.reset(new FeatureHandler(frame_handler_options));

  // instantiate estimator
  YAML::Node estimator_node = node["estimator_options"];
  if (type_ == EstimatorType::GnssImuCameraSrr) {
    GnssImuCameraSrrEstimatorOptions options;
    if (estimator_node.IsDefined()) {
      option_tools::loadOptions(estimator_node, options);
    }
    gnss_imu_camera_srr_estimator_.reset(new GnssImuCameraSrrEstimator(options));
    gnss_imu_camera_initializer_.reset(new GnssImuCameraInitialization(
      options.initialize, gnss_imu_camera_srr_estimator_->getGraph()));
    gnss_imu_camera_srr_estimator_->setFeatureHandler(feature_handler_);
  }
  else if (type_ == EstimatorType::RtkImuCameraRrr) {

  }
  gnss_imu_camera_initializer_->setFeatureHandler(feature_handler_);

  // Initial values
  solution_.timestamp = 0.0;
  solution_.backend.timestamp = 0.0;
}

GnssImuCameraEstimating::~GnssImuCameraEstimating()
{
  if (frontend_thread_) {
    frontend_thread_->join(); frontend_thread_ = nullptr;
  }
  if (backend_thread_) {
    backend_thread_->join(); backend_thread_ = nullptr;
  }
}

// GNSS data callback
void GnssImuCameraEstimating::gnssCallback(GnssMeasurement& data)
{
  mutex_input_.lock();
  gnss_measurements_[data.role].push_back(data);
  sensor_sequence_.push_back(SensorType::GNSS);
  mutex_input_.unlock();

  // Align timeline
  mutex_output_.lock();
  if (loop_duration_align_tag_ == data.tag) {
    aligned_new_timestamp_ = data.timestamp;
    aligned_new_data_ = true;
  }
  mutex_output_.unlock();
}

// IMU data callback
void GnssImuCameraEstimating::imuCallback(
    std::string tag, ImuRole role, ImuMeasurement& data)
{
  mutex_input_.lock();
  imu_measurements_[role].push_back(data);
  mutex_input_.unlock();

  // Currently we only support an IMU acts as major role
  if (role == ImuRole::Major) {
    if (gnss_imu_camera_srr_estimator_) {
      gnss_imu_camera_srr_estimator_->addImuMeasurement(data);
    }
    if (gnss_imu_camera_initializer_) {
      gnss_imu_camera_initializer_->addImuMeasurement(data);
    }
    if (feature_handler_) {
      feature_handler_->addImuMeasurement(data);
    }
  }

  // Align timeline
  mutex_output_.lock();
  if (loop_duration_align_tag_ == tag) {
    aligned_new_timestamp_ = data.timestamp;
    aligned_new_data_ = true;
  }
  mutex_output_.unlock();
}

// Solution callback from other estimators
void GnssImuCameraEstimating::solutionCallback(
  std::string tag, SolutionRole role, Solution& data)
{
  mutex_input_.lock();
  solution_measurements_[role].push_back(data);
  sensor_sequence_.push_back(SensorType::GNSS);
  mutex_input_.unlock();

  // Align timeline
  mutex_output_.lock();
  if (loop_duration_align_tag_ == tag) {
    aligned_new_timestamp_ = data.timestamp;
    aligned_new_data_ = true;
  }
  mutex_output_.unlock();
}

// Image data callback
void GnssImuCameraEstimating::imageCallback(double timestamp, 
  std::string tag, CameraRole role, cv::Mat& image)
{
  mutex_input_.lock();
  images_[role].push_back(std::make_pair(timestamp, image.clone()));
  mutex_input_.unlock();

  // Align timeline
  mutex_output_.lock();
  if (loop_duration_align_tag_ == tag) {
    aligned_new_timestamp_ = timestamp;
    aligned_new_data_ = true;
  }
  mutex_output_.unlock();
}

// Process funtion in every loop
void GnssImuCameraEstimating::process()
{
  // Process frontend in a separated thread
  if (frontend_thread_ == nullptr) {
    frontend_thread_.reset(new std::thread(&GnssImuCameraEstimating::runFrontend, this));
  }

  // Process backend in a separated thread
  if (backend_thread_ == nullptr) {
    backend_thread_.reset(new std::thread(&GnssImuCameraEstimating::runBackend, this)); 
  }

  // kill threads
  if (quit_thread_) {
    if (frontend_thread_) {
      frontend_thread_->join(); frontend_thread_ = nullptr;
    }
    if (backend_thread_) {
      backend_thread_->join(); backend_thread_ = nullptr;
    }
  }

  // Integrate solution to our desired publishing timestamp. Only the estimators with 
  // IMU sensors are applied.
  integrateSolution();
}

// Process initialization
bool GnssImuCameraEstimating::processInitialize()
{
  // Check if we have data to process
  bool has_gnss = false;
  GnssSolution gnss_solution;
  if (gnss_imu_camera_srr_estimator_)
  {
    Solution solution_measurement;
    mutex_input_.lock();
    if (sensor_sequence_.front() == SensorType::GNSS)
    {
      if (solution_measurements_[SolutionRole::Position].size() != 0) {
        solution_measurement = 
          solution_measurements_[SolutionRole::Position].front();
        has_gnss = true;
      }
      else if (solution_measurements_[SolutionRole::PositionAndVelocity].size() != 0) {
        solution_measurement = 
          solution_measurements_[SolutionRole::PositionAndVelocity].front();
        has_gnss = true;
      }
      CHECK(has_gnss);
      sensor_sequence_.pop_front();
      popSolutionMeasurement();
      gnss_solution = convertSolutionToGnssSolution(solution_measurement);
    }
    mutex_input_.unlock();
  }
  else if (0)
  {
    // TODO: for GNSS raw
  }

  mutex_input_.lock();
  if (sensor_sequence_.front() == SensorType::Camera)
  {
    // Throw frames, we do not need them before initialization is finished
    sensor_sequence_.pop_front();
    popFrameBundle();
  }
  mutex_input_.unlock();
  
  if (!has_gnss) return false;

  // Apply initialization
  // set coordinate and gravity
  if (solution_.backend.coordinate == nullptr) {
    solution_.backend.coordinate = std::make_shared<GeoCoordinate>(
      gnss_solution.position, GeoType::ECEF);
    Eigen::Vector3d lla = solution_.backend.coordinate->convert(
      gnss_solution.position, GeoType::ECEF, GeoType::LLA);
    double gravity = earthGravity(lla);

    gnss_imu_camera_initializer_->setCoordinate(solution_.backend.coordinate);
    gnss_imu_camera_initializer_->setGravity(gravity);

    if (gnss_imu_camera_srr_estimator_) {
      gnss_imu_camera_srr_estimator_->setCoordinate(solution_.backend.coordinate);
      gnss_imu_camera_srr_estimator_->setGravity(gravity);
      imu_parameters_ = gnss_imu_camera_srr_estimator_->getImuParameters();
    }
  }

  // add measurement and solve
  feature_handler_->lock();
  bool ret = gnss_imu_camera_initializer_->addGnssMeasurement(gnss_solution);
  feature_handler_->unlock();
  if (!ret) return false;
  gnss_imu_camera_initializer_->initialize();
  if (!gnss_imu_camera_initializer_->finished()) return false;

  // Pass initialization result to estimators
  if (gnss_imu_camera_srr_estimator_) {
    gnss_imu_camera_srr_estimator_->setInitializationResult(gnss_imu_camera_initializer_);
  }
  
  return true;
}

// Process Tightly couple using GNSS raw, IMU raw, and Image feature
bool GnssImuCameraEstimating::processGnssImuCameraSrr()
{
  // Check if we have data to process
  Solution solution_measurement;
  FrameBundlePtr frame_bundle;
  bool has_gnss = false, has_frame = false, has_any = false, is_updated = false;
  mutex_input_.lock();
  if (!has_any && sensor_sequence_.front() == SensorType::GNSS)
  {
    if (solution_measurements_[SolutionRole::Position].size() != 0) {
      solution_measurement = 
        solution_measurements_[SolutionRole::Position].front();
      has_gnss = true;
      has_any = true;
    }
    else if (solution_measurements_[SolutionRole::PositionAndVelocity].size() != 0) {
      solution_measurement = 
        solution_measurements_[SolutionRole::PositionAndVelocity].front();
      has_gnss = true;
      has_any = true;
    }
    CHECK(has_gnss);
    sensor_sequence_.pop_front();
    popSolutionMeasurement();
  }
  mutex_input_.unlock();

  mutex_input_.lock();
  if (!has_any && sensor_sequence_.front() == SensorType::Camera)
  {
    if (frame_bundles_.size() != 0) {
      frame_bundle = frame_bundles_.front();
      has_frame = true;
      has_any = true;
    }
    CHECK(has_frame);
    sensor_sequence_.pop_front();
    popFrameBundle();
  }
  mutex_input_.unlock();
  
  CHECK(!(has_gnss && has_frame));
  if (!has_frame && !has_gnss) return false;
#if 1
  // Process GNSS measurement
  mutex_process_.lock();
  GnssSolution gnss_solution;
  if (has_gnss)
  {
    gnss_solution = convertSolutionToGnssSolution(solution_measurement);
    if (backend_processing_timestamp_ == 0.0) {
      backend_processing_timestamp_ = solution_measurement.timestamp;
    } 
    backend_processing_timestamp_ = std::min(
      backend_processing_timestamp_, solution_measurement.timestamp);

    // add measurement
    if (gnss_imu_camera_srr_estimator_->addGnssMeasurementAndState(gnss_solution)) {
      // solve
      gnss_imu_camera_srr_estimator_->optimize();
      is_updated = true;
    }
  }

  // Process features
  if (has_frame)
  {
    backend_processing_timestamp_ = std::min(
      backend_processing_timestamp_, frame_bundle->getMinTimestampSeconds());

    // add measurement
    feature_handler_->lock();
    bool ret = gnss_imu_camera_srr_estimator_->addImageMeasurementAndState(frame_bundle);
    feature_handler_->unlock();
    if (ret) {
      // solve
      double t = vk::Timer::getCurrentTime();
      gnss_imu_camera_srr_estimator_->optimize();
      LOG(INFO) << "backend_dt = " << vk::Timer::getCurrentTime() - t; 
      is_updated = true;
    }
  }
  mutex_process_.unlock();

  // Update solution
  if (is_updated)
  {
    mutex_output_.lock(); // lock to avoid conflit with integration
    solution_.backend.timestamp = gnss_imu_camera_srr_estimator_->getTimestamp();
    solution_.backend.pose = gnss_imu_camera_srr_estimator_->getPoseEstimate();
    solution_.backend.speed_and_bias = gnss_imu_camera_srr_estimator_->getSpeedAndBias();
    if (has_gnss) {
      solution_.status = gnss_solution.status;
      solution_.num_satellites = gnss_solution.num_satellites;
      solution_.differential_age = gnss_solution.differential_age;
    }
    mutex_output_.unlock();
  }
#else
  if (has_frame) {
    mutex_output_.lock(); // lock to avoid conflit with integration
    solution_.backend.timestamp = frame_bundle->getMinTimestampSeconds();
    solution_.backend.pose = frame_bundle->get_T_W_B();
    solution_.backend.speed_and_bias = gnss_imu_camera_srr_estimator_->getSpeedAndBias();
    mutex_output_.unlock();
  }
#endif
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
  mutex_output_.lock();
  double publish_timestamp = getPublishTime();

  // Integration
  // check if backend updated
  if (integrate_backend_timestamp_ != solution_.backend.timestamp) {
    solution_.timestamp = solution_.backend.timestamp;
    solution_.pose = solution_.backend.pose;
    solution_.speed_and_bias = solution_.backend.speed_and_bias;
    integrate_backend_timestamp_ = solution_.backend.timestamp;
  }

  ImuError::propagation(
    imu_measurements_[ImuRole::Major], imu_parameters_, 
    solution_.pose, solution_.speed_and_bias, solution_.timestamp, 
    publish_timestamp, nullptr, nullptr);
  solution_.pose.getRotation().normalize();
  solution_.timestamp = publish_timestamp;

  // If integration period larger than 2 s, we set the status as Dead Reckoning
  if (solution_.timestamp - solution_.backend.timestamp > 2.0) {
    solution_.status = GnssSolutionStatus::DeadReckoning;
  }
  mutex_output_.unlock();

  // Delete used IMU measurements
  double front_timestamp = 
    std::min(publish_timestamp, backend_processing_timestamp_);
  mutex_input_.lock();
  popImuMeasurement(front_timestamp - 1.0);
  mutex_input_.unlock();
}

// Camera frontend processing
void GnssImuCameraEstimating::runFrontend()
{
  SpinControl spin(loop_duration_ > 0.0 ? loop_duration_ : 1.0e-3);
  while (!quit_thread_ && SpinControl::ok()) {
    // Check if we have new image data
    mutex_input_.lock();
    if (images_[CameraRole::Mono].size() == 0) {
      mutex_input_.unlock(); 
      spin.sleep(); continue;
    }

    // check if timestamp is valid
    if (!feature_handler_->isFirstFrame() && 
        feature_handler_->getFrameBundle()->getMinTimestampSeconds() >= 
        images_[CameraRole::Mono].front().first) {
      LOG(WARNING) << "Current image timestamp less than last added image!";
      popImage(); mutex_input_.unlock(); 
      spin.sleep(); continue;
    }

    // Check pending
    double& timestamp = images_[CameraRole::Mono].front().first;
    if (frame_bundles_.size() > 1) {
      LOG(WARNING) << "Backend pending: " << frame_bundles_.size() 
                  << " frames are waiting!";
    }
    if (images_[CameraRole::Mono].size() > 1) {
      LOG(WARNING) << "Frontend pending: " << images_[CameraRole::Mono].size()
                  << " images are waiting!";
    }

    // Process feature detecting and tracking
    cv::Mat& image = images_[CameraRole::Mono].front().second;
    mutex_input_.unlock();
    // process image
    feature_handler_->lock();
    bool ret = feature_handler_->addImageBundle({image}, timestamp);
    feature_handler_->unlock();
    if (ret) {
      mutex_input_.lock();
      frame_bundles_.push_back(feature_handler_->getFrameBundle());
      sensor_sequence_.push_back(SensorType::Camera);
      FramePtr& frame = frame_bundles_.back()->frames_.at(0);
      mutex_input_.unlock();

      // call featured image output (for ROS)
      for (auto& featured_image_callback : featured_image_callbacks_) {
        featured_image_callback(frame);
      }

      // call map point output (for ROS)
      MapPtr map = feature_handler_->getMap();
      if (map->size() > 0) {
        for (auto& map_point_callback : map_point_callbacks_) {
          map_point_callback(map);
        }
      }
    }

    // Delete used
    mutex_input_.lock();
    popImage();
    mutex_input_.unlock();

    spin.sleep();
  }

}

// Backend processing
void GnssImuCameraEstimating::runBackend()
{
  SpinControl spin(loop_duration_ > 0.0 ? loop_duration_ : 1.0e-3);
  while (!quit_thread_ && SpinControl::ok()) {
    if (!gnss_imu_camera_initializer_->finished()) {
      processInitialize();
    }
    else if (type_ == EstimatorType::GnssImuCameraSrr) {
      processGnssImuCameraSrr();
    }

    spin.sleep();
  }
}

}
