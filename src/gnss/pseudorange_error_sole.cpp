/**
* @Function: Pseudorange residual block for ceres backend
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/gnss/pseudorange_error_sole.h"

#include "gici/gnss/gnss_common.h"

namespace gici {

// Construct with measurement and information matrix
PseudorangeErrorSole::PseudorangeErrorSole(
                       const GNSSMeasurement& measurement,
                       const size_t satellite_index,
                       const int code_type, 
                       const GNSSErrorParameter& error_parameter)
{
  CHECK(measurement.satellites.size() > satellite_index);
  satellite_ = measurement.satellites.at(satellite_index);

  auto obs = satellite_.observations.find(code_type);
  CHECK(obs != satellite_.observations.end());
  observation_ = obs->second;

  setMeasurement(measurement);
  error_parameter_ = error_parameter;
}

// This evaluates the error term and additionally computes the Jacobians.
bool PseudorangeErrorSole::Evaluate(double const* const * parameters,
                                 double* residuals, double** jacobians) const
{
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, nullptr);
}

// This evaluates the error term and additionally computes
// the Jacobians in the minimal internal representation.
bool PseudorangeErrorSole::EvaluateWithMinimalJacobians(
    double const* const * parameters, double* residuals, double** jacobians,
    double** jacobians_minimal) const
{
  // position
  Eigen::Map<const Eigen::Vector3d> t_WR_ECEF(parameters[0]);

  // receiver clock error
  Eigen::Map<const Eigen::Matrix<double, 1, 1>> clock(parameters[1]);

  // Get estimate derivated measurement
  double timestamp = measurement_.timestamp;
  double rho = gnss_common::satelliteToReceiverDistance(satellite_.sat_position, t_WR_ECEF);
  double elevation = gnss_common::satelliteElevation(satellite_.sat_position, t_WR_ECEF);
  double azimuth = gnss_common::satelliteAzimuth(satellite_.sat_position, t_WR_ECEF);
  double troposphere_delay = gnss_common::troposphereSaastamoinen(
    timestamp, t_WR_ECEF, elevation);
  double ionosphere_delay = gnss_common::ionosphereBroadcast(
    timestamp, t_WR_ECEF, azimuth, elevation, measurement_.ionosphere_parameters);

  double pseudorange_estimate = rho + clock(0) - 
    satellite_.sat_clock + troposphere_delay + ionosphere_delay;

  // Compute error
  double pseudorange = observation_.pesudorange;
  Eigen::Matrix<double, 1, 1> error = 
    Eigen::Matrix<double, 1, 1>(pseudorange - pseudorange_estimate);

  // weigh it
  Eigen::Map<Eigen::Matrix<double, 1, 1> > weighted_error(residuals);
  Eigen::Vector3d factor(error_parameter_.phase_error_factor);
  double ratio = error_parameter_.code_to_phase_ratio;
  double variance = (pow(factor(0), 2) + pow(factor(1) / sin(elevation), 2)) * ratio;
  double square_root_information = sqrt(1.0 / variance);

  // check quality
  if (satellite_.sat_type == SatEphType::None) square_root_information = 0.0;

  weighted_error = square_root_information * error;

  // compute Jacobian
  if (jacobians != nullptr)
  {
    // Position
    if (jacobians[0] != nullptr) {
      Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> J0(jacobians[0]);
      J0.setZero();
      J0 = square_root_information * 
        (-((t_WR_ECEF - satellite_.sat_position) / rho).transpose());

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
