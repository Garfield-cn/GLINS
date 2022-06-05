/**
* @Function: GNSS/IMU coupled estimator thread
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/fusion/gnss_imu_estimating.h"

#include "gici/gnss/spp_estimator.h"
#include "gici/gnss/dgnss_estimator.h"
#include "gici/imu/imu_common.h"
#include "gici/imu/imu_error.h"

namespace gici {

GnssImuEstimating::GnssImuEstimating(YAML::Node& node) : 
  EstimatingBase(node), backend_finished_(true), 
  integrate_backend_timestamp_(0.0)
{
  // instantiate estimator
  YAML::Node estimator_node = node["estimator_options"];
  if (type_ == EstimatorType::GnssImuLc) {
    GnssImuLcEstimatorOptions options;
    if (estimator_node.IsDefined()) {
      option_tools::loadOptions(estimator_node, options);
    }
    gnss_imu_lc_estimator_.reset(new GnssImuLcEstimator(options));
    gnss_imu_initializer_.reset(new GnssImuInitialization(
      options.initialize, gnss_imu_lc_estimator_->getGraph()));
  }
  else if (type_ == EstimatorType::RtkImuTc) {
    RtkImuTcEstimatorOptions options;
    if (estimator_node.IsDefined()) {
      option_tools::loadOptions(estimator_node, options);
    }
    rtk_imu_tc_estimator_.reset(new RtkImuTcEstimator(options));
    gnss_imu_initializer_.reset(new GnssImuInitialization(
      options.initialize, rtk_imu_tc_estimator_->getGraph()));
    // used for initialization
    RtkEstimatorOptions rtk_options;
    rtk_options.max_iteration = options.max_iteration;
    rtk_options.num_threads = options.num_threads;
    rtk_options.max_age = options.max_age;
    rtk_options.window_length = options.window_length;
    rtk_options.use_ambiguity_resolution = options.use_ambiguity_resolution;
    rtk_options.common = options.gnss_common;
    rtk_options.error_parameter = options.gnss_error_parameter;
    rtk_options.ambiguity_resolution = options.ambiguity_resolution;
    rtk_estimator_.reset(new RtkEstimator(rtk_options));
  }

  latest_gnss_measurement_ref_.timestamp = 0.0;
}

GnssImuEstimating::~GnssImuEstimating()
{}

// GNSS data callback
void GnssImuEstimating::gnssCallback(GnssMeasurement& data)
{
  mutex_input_.lock();
  gnss_measurements_[data.role].push_back(data);
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
void GnssImuEstimating::imuCallback(
    std::string tag, ImuRole role, ImuMeasurement& data)
{
  mutex_input_.lock();
  imu_measurements_[role].push_back(data);
  mutex_input_.unlock();

  // Currently we only support an IMU acts as major role
  if (role == ImuRole::Major) {
    if (gnss_imu_lc_estimator_) {
      gnss_imu_lc_estimator_->addImuMeasurement(data);
    }
    if (rtk_imu_tc_estimator_) {
      rtk_imu_tc_estimator_->addImuMeasurement(data);
    }
    if (gnss_imu_initializer_) {
      gnss_imu_initializer_->addImuMeasurement(data);
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
void GnssImuEstimating::solutionCallback(
  std::string tag, SolutionRole role, Solution& data)
{
  mutex_input_.lock();
  solution_measurements_[role].push_back(data);
  mutex_input_.unlock();

  // Align timeline
  mutex_output_.lock();
  if (loop_duration_align_tag_ == tag) {
    aligned_new_timestamp_ = data.timestamp;
    aligned_new_data_ = true;
  }
  mutex_output_.unlock();
}

// Process funtion in every loop
void GnssImuEstimating::process()
{
  // Process backend in a separated thread
  // or it may influence the integration and hence delay the solution publishing
  if (backend_finished_) {
    if (backend_thread_) backend_thread_->join();
    backend_thread_.reset(new std::thread(&GnssImuEstimating::runBackend, this));
  }

  // Integrate solution to our desired publishing timestamp. Only the estimators with 
  // IMU sensors are applied.
  integrateSolution();
}

// Process initialization
bool GnssImuEstimating::processInitialize()
{
  // Check if we have data to process
  GnssSolution gnss_solution;
  if (gnss_imu_lc_estimator_)
  {
    Solution solution_measurement;
    mutex_input_.lock();
    if (solution_measurements_[SolutionRole::Position].size() != 0) {
      solution_measurement = 
        solution_measurements_[SolutionRole::Position].front();
      popSolutionMeasurement();
    }
    else if (solution_measurements_[SolutionRole::PositionAndVelocity].size() != 0) {
      solution_measurement = 
        solution_measurements_[SolutionRole::PositionAndVelocity].front();
      popSolutionMeasurement();
    }
    else {
      mutex_input_.unlock();
      return false;
    }
    mutex_input_.unlock();
    gnss_solution = convertSolutionToGnssSolution(solution_measurement);
  }
  else if (rtk_imu_tc_estimator_)
  {
    if (gnss_measurements_[GnssRole::Rover].size() == 0) return false;
    if (gnss_measurements_[GnssRole::Reference].size() == 0 && 
        latest_gnss_measurement_ref_.timestamp == 0.0) return false;

    // align measurements
    mutex_input_.lock();
    while (gnss_measurements_[GnssRole::Rover].size() > 
          gnss_measurements_[GnssRole::Reference].size() && 
          gnss_measurements_[GnssRole::Rover].size() > 1) {
      gnss_measurements_[GnssRole::Rover].pop_front();
    }
    while (gnss_measurements_[GnssRole::Reference].size() > 
          gnss_measurements_[GnssRole::Rover].size()) {
      gnss_measurements_[GnssRole::Reference].pop_front();
    }

    auto gnss_measurement_rov = gnss_measurements_[GnssRole::Rover].front();
    auto gnss_measurement_ref = latest_gnss_measurement_ref_;
    if (gnss_measurements_[GnssRole::Reference].size() > 0) {
      gnss_measurement_ref = gnss_measurements_[GnssRole::Reference].front();
    }
    latest_gnss_measurement_ref_ = gnss_measurement_ref;

    // Delete used
    popGnssMeasurement();
    mutex_input_.unlock();

    // set coarse position
    if (!DgnssEstimator::setCoarsePosition(
        gnss_measurement_rov, gnss_measurement_ref)) {
      return false; 
    }

    // get RTK solution
    if (!rtk_estimator_->addGnssMeasurementAndState(
        gnss_measurement_rov, gnss_measurement_ref)) {
      return false;
    }
    rtk_estimator_->optimize();
    gnss_solution = rtk_estimator_->getSolution();
  }

  // Apply initialization
  // set coordinate and gravity
  if (solution_.backend.coordinate == nullptr) {
    solution_.backend.coordinate = std::make_shared<GeoCoordinate>(
      gnss_solution.position, GeoType::ECEF);
    Eigen::Vector3d lla = solution_.backend.coordinate->convert(
      gnss_solution.position, GeoType::ECEF, GeoType::LLA);
    double gravity = earthGravity(lla);

    gnss_imu_initializer_->setCoordinate(solution_.backend.coordinate);
    gnss_imu_initializer_->setGravity(gravity);

    if (gnss_imu_lc_estimator_) {
      gnss_imu_lc_estimator_->setCoordinate(solution_.backend.coordinate);
      gnss_imu_lc_estimator_->setGravity(gravity);
      imu_parameters_ = gnss_imu_lc_estimator_->getImuParameters();
    }
    else if (rtk_imu_tc_estimator_) {
      rtk_imu_tc_estimator_->setCoordinate(solution_.backend.coordinate);
      rtk_imu_tc_estimator_->setGravity(gravity);
      imu_parameters_ = rtk_imu_tc_estimator_->getImuParameters();
    }
  }

  // add measurement and solve
  if (!gnss_imu_initializer_->addGnssMeasurement(gnss_solution)) {
    return false;
  }
  if (!gnss_imu_initializer_->finished()) return false;
  
  // Pass initialization result to estimators
  if (gnss_imu_lc_estimator_) {
    gnss_imu_lc_estimator_->setInitializationResult(gnss_imu_initializer_);
  }
  else if (rtk_imu_tc_estimator_) {
    rtk_imu_tc_estimator_->setInitializationResult(gnss_imu_initializer_);
  }
  
  return true;
}

// Process GNSS and IMU loosely couple estimator
bool GnssImuEstimating::processGnssImuLc()
{
  // Check if we have data to process
  Solution solution_measurement;
  mutex_input_.lock();
  if (solution_measurements_[SolutionRole::Position].size() != 0) {
    solution_measurement = 
      solution_measurements_[SolutionRole::Position].front();
    popSolutionMeasurement();
  }
  else if (solution_measurements_[SolutionRole::PositionAndVelocity].size() != 0) {
    solution_measurement = 
      solution_measurements_[SolutionRole::PositionAndVelocity].front();
    popSolutionMeasurement();
  }
  else {
    mutex_input_.unlock();
    return false;
  }
  mutex_input_.unlock();

  // Apply GNSS/IMU loosely couple
  GnssSolution gnss_solution = convertSolutionToGnssSolution(solution_measurement);
  backend_processing_timestamp_ = solution_measurement.timestamp;

  // add measurement
  if (!gnss_imu_lc_estimator_->addGnssMeasurementAndState(gnss_solution)) {
    return false;
  }
  // solve
  gnss_imu_lc_estimator_->optimize();

  // get solution
  mutex_output_.lock(); // lock to avoid conflit with integration
  solution_.backend.timestamp = gnss_imu_lc_estimator_->getTimestamp();
  solution_.backend.pose = gnss_imu_lc_estimator_->getPoseEstimate();
  solution_.backend.speed_and_bias = gnss_imu_lc_estimator_->getSpeedAndBias();
  solution_.status = gnss_solution.status;
  solution_.num_satellites = gnss_solution.num_satellites;
  solution_.differential_age = gnss_solution.differential_age;
  mutex_output_.unlock();

  return true;
}

// Process RTK and IMU tightly couple estimator
bool GnssImuEstimating::processRtkImuTc()
{
  // Check if we have data to process
  if (gnss_measurements_[GnssRole::Rover].size() == 0) return false;
  if (gnss_measurements_[GnssRole::Reference].size() == 0 && 
      latest_gnss_measurement_ref_.timestamp == 0.0) return false;

  // align measurements
  mutex_input_.lock();
  while (gnss_measurements_[GnssRole::Rover].size() > 
         gnss_measurements_[GnssRole::Reference].size() && 
         gnss_measurements_[GnssRole::Rover].size() > 1) {
    gnss_measurements_[GnssRole::Rover].pop_front();
  }
  while (gnss_measurements_[GnssRole::Reference].size() > 
         gnss_measurements_[GnssRole::Rover].size()) {
    gnss_measurements_[GnssRole::Reference].pop_front();
  }

  // Apply RTK and IMU tightly couple
  auto gnss_measurement_rov = gnss_measurements_[GnssRole::Rover].front();
  auto gnss_measurement_ref = latest_gnss_measurement_ref_;
  if (gnss_measurements_[GnssRole::Reference].size() > 0) {
    gnss_measurement_ref = gnss_measurements_[GnssRole::Reference].front();
  }
  latest_gnss_measurement_ref_ = gnss_measurement_ref;
  backend_processing_timestamp_ = gnss_measurement_rov.timestamp;

  // Delete used
  popGnssMeasurement();
  mutex_input_.unlock();

  // set a coarse position to ensure preprocessings for satellites (such as elevation mask)
  if (!SppEstimator::setCoarsePosition(gnss_measurement_rov)) {
    return false; 
  }

  // define coordinate
  if (solution_.backend.coordinate == nullptr) {
    solution_.backend.coordinate = std::make_shared<GeoCoordinate>(
      gnss_measurement_rov.position, GeoType::ECEF);
    rtk_imu_tc_estimator_->setCoordinate(solution_.backend.coordinate);
  }

  // set gravity
  if (rtk_imu_tc_estimator_->isFirstEpoch()) {
    Eigen::Vector3d lla = solution_.backend.coordinate->convert(
      gnss_measurement_rov.position, GeoType::ECEF, GeoType::LLA);
    double gravity = earthGravity(lla);
    rtk_imu_tc_estimator_->setGravity(gravity);
    imu_parameters_ = rtk_imu_tc_estimator_->getImuParameters();
  }

  // add measurement
  if (!rtk_imu_tc_estimator_->addGnssMeasurementAndState(
      gnss_measurement_rov, gnss_measurement_ref)) {
    return false;
  }
  // solve
  rtk_imu_tc_estimator_->optimize();

  // get solution
  mutex_output_.lock(); 
  solution_.backend.timestamp = rtk_imu_tc_estimator_->getTimestamp();
  solution_.backend.pose = rtk_imu_tc_estimator_->getPoseEstimate();
  solution_.backend.speed_and_bias = rtk_imu_tc_estimator_->getSpeedAndBias();
  solution_.status = rtk_imu_tc_estimator_->getSolutionStatus();
  solution_.num_satellites = rtk_imu_tc_estimator_->getNumSatellites();
  solution_.differential_age = rtk_imu_tc_estimator_->getAge();
  mutex_output_.unlock();

  return true;
}

// Get timestamp to publish
double GnssImuEstimating::getPublishTime() {
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
GnssSolution GnssImuEstimating::convertSolutionToGnssSolution(
  const Solution& solution)
{
  GnssSolution gnss_solution;
  static int32_t id_static = 0;

  gnss_solution.timestamp = solution.timestamp;
  gnss_solution.id = ++id_static;
  CHECK(solution.backend.coordinate);
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
void GnssImuEstimating::integrateSolution()
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
  popImuMeasurement(front_timestamp - 0.1);
  mutex_input_.unlock();
}

// Backend processing
void GnssImuEstimating::runBackend()
{
  backend_finished_ = false;

  // Initialize or process
  if (!gnss_imu_initializer_->finished()) {
    processInitialize();
  }
  else if (type_ == EstimatorType::GnssImuLc) {
    processGnssImuLc();
  }
  else if (type_ == EstimatorType::RtkImuTc) {
    processRtkImuTc();
  }

  backend_finished_ = true;
}

}
