/**
* @Function: Pseudorange residual block for ceres backend
*            Parameters are in ECEF coordinate. Observations are single-differenced (between stations).
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/gnss/pseudorange_error_sole_sd.h"

#include "gici/gnss/gnss_common.h"

namespace gici {

// Construct with measurement and information matrix
PseudorangeErrorSoleSD::PseudorangeErrorSoleSD(
                         const GNSSMeasurement& measurement_1,
                         const GNSSMeasurement& measurement_2,
                         const GNSSMeasurementIndex index_1,
                         const GNSSMeasurementIndex index_2,
                         const GNSSErrorParameter& error_parameter)
{
  CHECK(measurement_2.position != Eigen::Vector3d::Zero()) << 
    "The position of reference station is not setted!";

  CHECK(measurement_1.satellites.size() > index_1.satellite_index);
  CHECK(measurement_2.satellites.size() > index_2.satellite_index);
  satellite_1_ = measurement_1.satellites.at(index_1.satellite_index);
  satellite_2_ = measurement_2.satellites.at(index_2.satellite_index);

  auto obs_1 = satellite_1_.observations.find(index_1.code_type);
  auto obs_2 = satellite_2_.observations.find(index_2.code_type);
  CHECK(obs_1 != satellite_1_.observations.end());
  CHECK(obs_2 != satellite_2_.observations.end());
  observation_1_ = obs_1->second;
  observation_2_ = obs_2->second;

  setMeasurement(measurement_1, measurement_2);
  error_parameter_ = error_parameter;
}

// This evaluates the error term and additionally computes the Jacobians.
bool PseudorangeErrorSoleSD::Evaluate(double const* const * parameters,
                                 double* residuals, double** jacobians) const
{
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, nullptr);
}

// This evaluates the error term and additionally computes
// the Jacobians in the minimal internal representation.
bool PseudorangeErrorSoleSD::EvaluateWithMinimalJacobians(
    double const* const * parameters, double* residuals, double** jacobians,
    double** jacobians_minimal) const
{
  // position
  Eigen::Map<const Eigen::Vector3d> t_WR_ECEF(parameters[0]);

  // differenced receiver clock error
  Eigen::Map<const Eigen::Matrix<double, 1, 1>> dclock(parameters[1]);

  // Get estimate derivated measurement
  double timestamp = measurement_1_.timestamp;

  double rho_1 = gnss_common::satelliteToReceiverDistance(satellite_1_.sat_position, t_WR_ECEF);
  double rho_2 = gnss_common::satelliteToReceiverDistance(
    satellite_2_.sat_position, measurement_2_.position);

  double elevation_1 = gnss_common::satelliteElevation(satellite_1_.sat_position, t_WR_ECEF);
  double azimuth_1 = gnss_common::satelliteAzimuth(satellite_1_.sat_position, t_WR_ECEF);

  double dpseudorange_estimate = rho_1 - rho_2 + dclock(0);

  // Compute error
  double dpseudorange = observation_1_.pesudorange - observation_2_.pesudorange;
  Eigen::Matrix<double, 1, 1> error = 
    Eigen::Matrix<double, 1, 1>(dpseudorange - dpseudorange_estimate);

  // weigh it
  Eigen::Map<Eigen::Matrix<double, 1, 1> > weighted_error(residuals);
  Eigen::Vector3d factor(error_parameter_.phase_error_factor);
  double ratio = square(error_parameter_.code_to_phase_ratio);
  double variance = (square(factor(0)) + square(factor(1) / sin(elevation_1))) * ratio * 2.0;
  double square_root_information = sqrt(1.0 / variance);

  weighted_error = square_root_information * error;

  // compute Jacobian
  if (jacobians != nullptr)
  {
    // Position
    if (jacobians[0] != nullptr) {
      Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> J0(jacobians[0]);
      J0.setZero();
      J0 = square_root_information * 
        (-((t_WR_ECEF - satellite_1_.sat_position) / rho_1).transpose());

      if (jacobians_minimal != nullptr && jacobians_minimal[0] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor> >
            J0_minimal_mapped(jacobians_minimal[0]);
        J0_minimal_mapped = J0;
      }
    }
    // Clock
    if (jacobians[1] != nullptr) {
      jacobians[1][0] = square_root_information * (-1.0);
      if (jacobians_minimal != nullptr && jacobians_minimal[1] != nullptr) {
        jacobians_minimal[1][0] = jacobians[1][0];
      }
    }
  }

  return true;
}

}  // namespace gici
