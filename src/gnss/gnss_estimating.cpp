/**
* @Function: GNSS estimator thread
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/gnss/gnss_estimating.h"

namespace gici {

GnssEstimating::GnssEstimating(YAML::Node& node) : 
  EstimatingBase(node)
{
  // instantiate estimator
  YAML::Node estimator_node = node["estimator_options"];
  if (type_ == EstimatorType::Spp) {
    SppEstimatorOptions options;
    if (estimator_node.IsDefined()) {
      option_tools::loadOptions(estimator_node, options);
    }
    spp_estimator_.reset(new SppEstimator(options));
  }
  else if (type_ == EstimatorType::Dgnss) {
    DgnssEstimatorOptions options;
    if (estimator_node.IsDefined()) {
      option_tools::loadOptions(estimator_node, options);
    }
    dgnss_estimator_.reset(new DgnssEstimator(options));
  }
  else if (type_ == EstimatorType::Rtk) {
    RtkEstimatorOptions options;
    if (estimator_node.IsDefined()) {
      option_tools::loadOptions(estimator_node, options);
    }
    rtk_estimator_.reset(new RtkEstimator(options));
  } 

  latest_gnss_measurement_ref_.timestamp = 0.0;
}

GnssEstimating::~GnssEstimating()
{}

// GNSS data callback
void GnssEstimating::gnssCallback(GnssMeasurement& data)
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

// Process funtion in every loop
void GnssEstimating::process()
{
  if (type_ == EstimatorType::Spp) {
    processSpp();
  }
  else if (type_ == EstimatorType::Dgnss) {
    processDgnss();
  }
  else if (type_ == EstimatorType::Rtk) {
    processRtk();
  }
}

// Process SPP estimator
bool GnssEstimating::processSpp()
{
  // Check if we have data to process
  if (gnss_measurements_[GnssRole::Rover].size() == 0) return false;

  // Apply SPP
  mutex_input_.lock();
  auto gnss_measurement = gnss_measurements_[GnssRole::Rover].front();

  // Delete used
  popGnssMeasurement();
  mutex_input_.unlock();

  // set a coarse position to ensure preprocessings for satellites (such as elevation mask)
  if (!SppEstimator::setCoarsePosition(gnss_measurement)) {
    return false; 
  }

  // define coordinate
  if (solution_.backend.coordinate == nullptr) {
    solution_.backend.coordinate = std::make_shared<GeoCoordinate>(
      gnss_measurement.position, GeoType::ECEF);
  }

  // add measurement
  if (!spp_estimator_->addGnssMeasurementAndState(gnss_measurement)) {
    return false;
  }
  // solve
  spp_estimator_->optimize();

  // get solution
  mutex_output_.lock();
  solution_ = convertGnssSolutionToSolution(spp_estimator_->getSolution());
  mutex_output_.unlock();

  return true;
}

// Process DGNSS estimator
bool GnssEstimating::processDgnss()
{
  // Check if we have data to process
  if (gnss_measurements_[GnssRole::Rover].size() == 0) return false;
  if (gnss_measurements_[GnssRole::Reference].size() == 0) return false;

  // align measurements
  mutex_input_.lock();
  while (gnss_measurements_[GnssRole::Rover].size() > 
         gnss_measurements_[GnssRole::Reference].size()) {
    gnss_measurements_[GnssRole::Rover].pop_front();
  }
  while (gnss_measurements_[GnssRole::Reference].size() > 
         gnss_measurements_[GnssRole::Rover].size()) {
    gnss_measurements_[GnssRole::Reference].pop_front();
  }

  // Apply DGNSS
  auto gnss_measurement_rov = gnss_measurements_[GnssRole::Rover].front();
  auto gnss_measurement_ref = gnss_measurements_[GnssRole::Reference].front();

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
  }

  // add measurement
  if (!dgnss_estimator_->addGnssMeasurementAndState(
        gnss_measurement_rov, gnss_measurement_ref)) {
    return false;
  }
  // solve
  dgnss_estimator_->optimize();

  // get solution
  mutex_output_.lock();
  solution_ = convertGnssSolutionToSolution(dgnss_estimator_->getSolution());

  mutex_output_.unlock();

  return true;
}

// Process RTK estimator
bool GnssEstimating::processRtk()
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

  // Apply RTK
  auto gnss_measurement_rov = gnss_measurements_[GnssRole::Rover].front();
  auto gnss_measurement_ref = latest_gnss_measurement_ref_;
  if (gnss_measurements_[GnssRole::Reference].size() > 0) {
    gnss_measurement_ref = gnss_measurements_[GnssRole::Reference].front();
  }
  latest_gnss_measurement_ref_ = gnss_measurement_ref;

  // Delete used
  popGnssMeasurement();
  mutex_input_.unlock();

  // set a coarse position to ensure preprocessings for satellites (such as elevation mask)
  if (!DgnssEstimator::setCoarsePosition(
      gnss_measurement_rov, gnss_measurement_ref)) {
    return false; 
  }

  // define coordinate
  if (solution_.backend.coordinate == nullptr) {
    solution_.backend.coordinate = std::make_shared<GeoCoordinate>(
      gnss_measurement_rov.position, GeoType::ECEF);
  }

  // add measurement
  if (!rtk_estimator_->addGnssMeasurementAndState(
        gnss_measurement_rov, gnss_measurement_ref)) {
    return false;
  }
  // solve
  rtk_estimator_->optimize();

  // get solution
  mutex_output_.lock();
  solution_ = convertGnssSolutionToSolution(rtk_estimator_->getSolution());
  mutex_output_.unlock();

  return true;
}

// Convert GNSS solution to solution
Solution GnssEstimating::convertGnssSolutionToSolution(
  const GnssSolution& gnss_solution)
{
  Solution solution;

  solution.backend.coordinate = solution_.backend.coordinate;

  solution.backend.timestamp = gnss_solution.timestamp;
  solution.backend.pose = Transformation(solution.backend.coordinate->convert(
    gnss_solution.position, GeoType::ECEF, GeoType::ENU), 
    Eigen::Quaterniond::Identity());
  Eigen::Matrix3d R_enu_ecef = 
    solution.backend.coordinate->rotationMatrix(GeoType::ECEF, GeoType::ENU);
  solution.backend.speed_and_bias.setZero();
  solution.backend.speed_and_bias.segment<3>(0) = R_enu_ecef * gnss_solution.velocity;
  Eigen::Matrix<double, 6, 6> R_enu_ecef_double;
  R_enu_ecef_double.setZero();
  R_enu_ecef_double.topLeftCorner(3, 3) = R_enu_ecef;
  R_enu_ecef_double.bottomRightCorner(3, 3) = R_enu_ecef;
  Eigen::Matrix<double, 6, 6> covariance_position_velocity;
  covariance_position_velocity = R_enu_ecef_double * 
    gnss_solution.covariance * R_enu_ecef_double.transpose();
  solution.backend.covariance.setZero();
  solution.backend.covariance.block<3, 3>(0, 0) = 
    covariance_position_velocity.topLeftCorner(3, 3);
  solution.backend.covariance.block<3, 3>(6, 6) = 
    covariance_position_velocity.bottomRightCorner(3, 3);
  solution.backend.covariance.block<3, 3>(0, 6) = 
    covariance_position_velocity.topRightCorner(3, 3);
  solution.backend.covariance.block<3, 3>(6, 0) = 
    covariance_position_velocity.bottomLeftCorner(3, 3);

  // we do apply integration for GNSS only solution
  solution.pose = solution.backend.pose;
  solution.speed_and_bias = solution.backend.speed_and_bias;
  solution.timestamp = gnss_solution.timestamp;
  solution.status = gnss_solution.status;
  solution.num_satellites = gnss_solution.num_satellites;
  solution.differential_age = gnss_solution.differential_age;
  solution.covariance = solution.backend.covariance;

  return solution;
}

}
