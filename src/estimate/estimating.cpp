/**
* @Function: Handle estimator thread
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/estimate/estimating.h"

#include "gici/utility/spin_control.h"

namespace gici {

Estimating::Estimating(YAML::Node& node) : 
  loop_duration_(1e-3), publish_timestamp_(0.0), 
  aligned_new_data_(false)
{
  // Get options
  if (!option_tools::safeGet(node, "tag", &tag_)) {
    LOG(ERROR) << "Unable to load estimator tag!";
  }
  
  std::string type_str;
  if (!option_tools::safeGet(node, "type", &type_str)) {
    LOG(ERROR) << "Unable to load estimator type!";
    return;
  }
  option_tools::convert(type_str, type_);

  if (!option_tools::safeGet(node, "loop_duration", &loop_duration_)) {
    LOG(ERROR) << "Unable to load estimator loop duration!";
  }
  if (loop_duration_ == 0.0) {
    if (!option_tools::safeGet(
        node, "loop_duration_align_tag", &loop_duration_align_tag_)) {
      LOG(ERROR) << "Unable to load estimator loop duration align tag!";
      return;
    }
  }

  // instantiate
  if (type_ == EstimatorType::Rtk) {
    RtkEstimatorOptions options;
    option_tools::loadOptions(node, options);
    rtk_estimator_.reset(new RtkEstimator(options));
  } 
  else if (type_ == EstimatorType::GnssImuLc) {

  }
  else if (type_ == EstimatorType::RtkImuTc) {

  }
}

Estimating::~Estimating()
{}

// Start thread
void Estimating::start()
{
  // Create thread
  quit_thread_ = false;
  thread_.reset(new std::thread(&Estimating::run, this));
}

// Stop thread
void Estimating::stop()
{
  // Kill thread
  if(thread_ != nullptr) {
    quit_thread_ = true;
    thread_->join();
    thread_.reset();
  }
}

// GNSS data callback
void Estimating::gnssCallback(GnssMeasurement& data)
{
  gnss_measurements_[data.role].push_back(data);

  if (loop_duration_align_tag_ == data.tag) {
    aligned_new_timestamp_ = data.timestamp;
    aligned_new_data_ = true;
  }
}

// IMU data callback
void Estimating::imuCallback(
    std::string tag, ImuRole role, ImuMeasurement& data)
{
  imu_measurements_[role].push_back(data);

  if (loop_duration_align_tag_ == tag) {
    aligned_new_timestamp_ = data.timestamp;
    aligned_new_data_ = true;
  }
}

// Image data callback
void Estimating::imageCallback(
  double timestamp, std::string tag, CameraRole role, cv::Mat& image)
{
  // FramePtr frame = std::make_shared<Frame>()

  if (loop_duration_align_tag_ == tag) {
    aligned_new_timestamp_ = timestamp;
    aligned_new_data_ = true;
  }
}

// Process RTK estimator
void Estimating::processRtk()
{
  // Check if we have data to process
  if (gnss_measurements_[GnssRole::Rover].size() == 0) return;
  if (gnss_measurements_[GnssRole::Reference].size() == 0) return;

  // align measurements
  while (gnss_measurements_[GnssRole::Rover].size() > 
         gnss_measurements_[GnssRole::Reference].size()) {
    gnss_measurements_[GnssRole::Rover].pop_front();
  }
  while (gnss_measurements_[GnssRole::Reference].size() > 
         gnss_measurements_[GnssRole::Rover].size()) {
    gnss_measurements_[GnssRole::Reference].pop_front();
  }

  // Apply RTK
  auto& gnss_measurement_rov = gnss_measurements_[GnssRole::Rover].front();
  auto& gnss_measurement_ref = gnss_measurements_[GnssRole::Reference].front();
  // set a coarse position at the first epoch to speed up convergence
  if (rtk_estimator_->isFirstEpoch() && 
      !SppEstimator::setCoarsePosition(gnss_measurement_rov)) {
    popGnssMeasurement();
    return;
  }
  // set coordinate (just for convenience)
  if (rtk_estimator_->isFirstEpoch()) {
    solution_.coordinate = std::make_shared<GeoCoordinate>(
      gnss_measurement_rov.position, GeoType::ECEF);
  }
  // add measurement
  if (!rtk_estimator_->addGnssMeasurementAndState(
        gnss_measurement_rov, gnss_measurement_ref)) {
    popGnssMeasurement();
    return;
  }
  // solve
  rtk_estimator_->optimize();
  // get solution
  solution_.backend_timestamp = gnss_measurement_rov.timestamp;
  solution_.timestamp = gnss_measurement_rov.timestamp;
  solution_.pose = Transformation(solution_.coordinate->convert(
    rtk_estimator_->getPositionEstimate(), GeoType::ECEF, GeoType::ENU), 
    Eigen::Quaterniond::Identity());
  solution_.velocity = Eigen::Vector3d::Zero();
  solution_.integrate_pose = solution_.pose;
  solution_.speed_and_bias.segment<3>(0) = solution_.velocity;
  solution_.status = rtk_estimator_->getSolutionStatus();
  solution_.differential_age = fabs(
      gnss_measurement_rov.timestamp - gnss_measurement_ref.timestamp);
  solution_.num_satellites = rtk_estimator_->getSolution().num_satellites;

  // Delete used
  gnss_measurements_[GnssRole::Rover].pop_front();
  gnss_measurements_[GnssRole::Reference].pop_front();
}

// Process GNSS and IMU loosely couple estimator
void Estimating::processGnssImuLc()
{
  
}

// Process RTK and IMU tightly couple estimator
void Estimating::processRtkImuTc()
{
  
}

// Get timestamp to publish
double Estimating::getPublishTime() {
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

// Integrate solution to the timestamp we want to publish
void Estimating::integrateSolution()
{
  // Interpolate to current time or aligned stream time
  double publish_timestamp = getPublishTime();


}

// Loop processing
void Estimating::run()
{
  // Spin until quit command or global shutdown called 
  SpinControl spin(loop_duration_ > 0.0 ? loop_duration_ : 1.0e-3);
  while (!quit_thread_ && SpinControl::ok()) {
    // Check aligned stream
    if (loop_duration_ == 0.0 && !aligned_new_data_) {
      spin.sleep();
      continue;
    }

    // Process estimator
    if (type_ == EstimatorType::Rtk) {
      processRtk();
    }
    else if (type_ == EstimatorType::GnssImuLc) {
      processGnssImuLc();
      // Integrate solution to our desired publishing timestamp. Only the estimators with 
      // IMU sensors are applied.
      integrateSolution();
    }
    else if (type_ == EstimatorType::RtkImuTc) {
      processRtkImuTc();
      integrateSolution();
    }

    // Publish solution
    if (publish_timestamp_ != solution_.timestamp) {
      LOG(INFO) << solution_.timestamp << ": " << solution_.pose.getPosition().transpose();
      for (auto& solution_callback : solution_callbacks_) {
        solution_callback(solution_);
      }
      publish_timestamp_ = solution_.timestamp;
    }

    spin.sleep();
  }
}

}
