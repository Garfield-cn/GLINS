/**
* @Function: Pseudorange residual block for ceres backend
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/gnss/pseudorange_error.h"

#include "gici/gnss/gnss_common.h"
#include "gici/utility/transform.h"
#include "gici/optimizer/pose_local_parameterization.hpp"

namespace gici {

// Construct with measurement and information matrix
template<int... Ns>
PseudorangeError<Ns ...>::PseudorangeError(
                       const GNSSMeasurement& measurement,
                       const GNSSMeasurementIndex index,
                       const GNSSErrorParameter& error_parameter)
{
  CHECK(measurement.satellites.size() > index.satellite_index);
  satellite_ = measurement.satellites.at(index.satellite_index);

  auto obs = satellite_.observations.find(index.code_type);
  CHECK(obs != satellite_.observations.end());
  observation_ = obs->second;

  setMeasurement(measurement);
  error_parameter_ = error_parameter;

  // Check parameter block types
  // Group 1
  if (dims_.kNumParameterBlocks == 2 && 
      dims_.GetDim(0) == 3 && dims_.GetDim(1) == 1) {
    is_estimate_body_ = false;
    is_estimate_atmosphere_ = false;
    parameter_block_group_ = 1;
  }
  // Group 2
  else if (dims_.kNumParameterBlocks == 3 &&
      dims_.GetDim(0) == 6 && dims_.GetDim(1) == 3 &&
      dims_.GetDim(2) == 1) {
    is_estimate_body_ = true;
    is_estimate_atmosphere_ = false;
    parameter_block_group_ = 2;
  }
  // Group 3
  else if (dims_.kNumParameterBlocks == 4 && 
      dims_.GetDim(0) == 3 && dims_.GetDim(1) == 1 &&
      dims_.GetDim(2) == 1 && dims_.GetDim(3) == 1) {
    is_estimate_body_ = false;
    is_estimate_atmosphere_ = true;
    parameter_block_group_ = 3;
  }
  // Group 4
  else if (dims_.kNumParameterBlocks == 5 && 
      dims_.GetDim(0) == 6 && dims_.GetDim(1) == 3 &&
      dims_.GetDim(2) == 1 && dims_.GetDim(3) == 1 &&
      dims_.GetDim(4) == 1) {
    is_estimate_body_ = true;
    is_estimate_atmosphere_ = true;
    parameter_block_group_ = 4;
  }
  else {
    LOG(FATAL) << "PseudorangeError parameter blocks setup invalid!";
  }
}

// This evaluates the error term and additionally computes the Jacobians.
template<int... Ns>
bool PseudorangeError<Ns ...>::Evaluate(double const* const * parameters,
                                 double* residuals, double** jacobians) const
{
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, nullptr);
}

// This evaluates the error term and additionally computes
// the Jacobians in the minimal internal representation.
template<int... Ns>
bool PseudorangeError<Ns ...>::EvaluateWithMinimalJacobians(
    double const* const * parameters, double* residuals, double** jacobians,
    double** jacobians_minimal) const
{
  Eigen::Vector3d t_WR_ECEF, t_WS_W, t_SR_S;
  Eigen::Quaterniond q_WS;
  double clock;
  double troposphere_delay, ionosphere_delay;
  double ephemeris_var, troposphere_var, ionosphere_var;
  double gmf_wet;
  
  // Position and clock
  if (!is_estimate_body_) 
  {
    t_WR_ECEF = Eigen::Map<const Eigen::Vector3d>(parameters[0]);
    clock = parameters[1][0];
  }
  else 
  {
    // pose in ENU frame
    t_WS_W = Eigen::Map<const Eigen::Vector3d>(&parameters[0][0]);
    q_WS = Eigen::Map<const Eigen::Quaterniond>(&parameters[0][3]);

    // relative position
    t_SR_S = Eigen::Map<const Eigen::Vector3d>(parameters[1]);

    // clock
    clock = parameters[2][0];

    // receiver position
    Eigen::Vector3d t_WR_W = t_WS_W + q_WS * t_SR_S;

    if (!coordinate_) {
      LOG(FATAL) << "Coordinate not set!";
    }
    if (!coordinate_->isZeroSetted()) {
      LOG(FATAL) << "Coordinate zero not set!";
    }
    t_WR_ECEF = coordinate_->getECEF(t_WR_W, GeoType::ENU);
  }

  double timestamp = measurement_.timestamp;
  double rho = gnss_common::satelliteToReceiverDistance(satellite_.sat_position, t_WR_ECEF);
  double elevation = gnss_common::satelliteElevation(satellite_.sat_position, t_WR_ECEF);
  double azimuth = gnss_common::satelliteAzimuth(satellite_.sat_position, t_WR_ECEF);

  // Atmosphere
  if (!is_estimate_atmosphere_) 
  {

    // troposphere hydro-static delay
    troposphere_delay = gnss_common::troposphereSaastamoinen(
      timestamp, t_WR_ECEF, elevation);
    troposphere_var = square(
      error_parameter_.troposphere_model_factor * troposphere_delay);
    // troposphere wet delay from augmentation
    if (measurement_.troposphere_wet != 0.0) {
      gnss_common::troposphereGMF(timestamp, t_WR_ECEF, elevation, nullptr, &gmf_wet);
      troposphere_delay += measurement_.troposphere_wet * gmf_wet;
      troposphere_var = square(error_parameter_.troposphere_augment);
    }

    // ionosphere
    if (satellite_.ionosphere != 0.0) {
      ionosphere_delay = satellite_.ionosphere;
      if (satellite_.ionosphere_type == IonoType::Augmentation) {
        ionosphere_var = square(error_parameter_.ionosphere_augment);
      }
      else if (satellite_.ionosphere_type == IonoType::DualFrequency) {
        ionosphere_var = square(error_parameter_.ionosphere_dual_frequency);
      }
    }
    else {
      ionosphere_delay = gnss_common::ionosphereBroadcast(timestamp, t_WR_ECEF, 
          azimuth, elevation, observation_.wavelength, measurement_.ionosphere_parameters);
      ionosphere_var = square(
        error_parameter_.ionosphere_broadcast_factor * ionosphere_delay);
    }

    // ephemeris error
    if (satellite_.sat_type == SatEphType::Broadcast) {
      ephemeris_var = square(error_parameter_.ephemeris_broadcast);
    }
    else if (satellite_.sat_type == SatEphType::Precise) {
      ephemeris_var = square(error_parameter_.ephemeris_precise);
    }
  }
  else
  { 
    // use estimated atomspheric delays
    double troposphere_wet = 0.0;
    if (!is_estimate_body_) {
      troposphere_wet = parameters[2][0];
      ionosphere_delay = parameters[3][0];
    }
    else {
      troposphere_wet = parameters[3][0];
      ionosphere_delay = parameters[4][0];
    }
    troposphere_var = 0.0;
    ionosphere_var = 0.0;

    // troposphere hydro-static delay
    troposphere_delay = gnss_common::troposphereSaastamoinen(
      timestamp, t_WR_ECEF, elevation);
    // troposphere wet delay from augmentation
    gnss_common::troposphereGMF(timestamp, t_WR_ECEF, elevation, nullptr, &gmf_wet);
    troposphere_delay += troposphere_wet * gmf_wet;

    // we do not add ephemeris error in precise positioning case
    // because the precise ephemeris is highly time-correlated, which
    // cannot be treated as white noise.
    ephemeris_var = 0.0;
  }

  // Get estimate derivated measurement
  double pseudorange_estimate = rho + clock - 
    satellite_.sat_clock + troposphere_delay + ionosphere_delay;

  // Compute error
  double pseudorange = observation_.pesudorange;
  Eigen::Matrix<double, 1, 1> error = 
    Eigen::Matrix<double, 1, 1>(pseudorange - pseudorange_estimate);

  // weigh it
  Eigen::Map<Eigen::Matrix<double, 1, 1> > weighted_error(residuals);
  Eigen::Vector3d factor(error_parameter_.phase_error_factor);
  double ratio = square(error_parameter_.code_to_phase_ratio);
  double variance = (square(factor(0)) + square(factor(1) / sin(elevation))) * ratio + 
    ephemeris_var + ionosphere_var + troposphere_var;
  double square_root_information = sqrt(1.0 / variance);
  weighted_error = square_root_information * error;

  // compute Jacobian
  if (jacobians != nullptr)
  {
    // Receiver position in ECEF
    Eigen::Matrix<double, 1, 3> J_t_ECEF = 
      -((t_WR_ECEF - satellite_.sat_position) / rho).transpose();
    
    // Poses
    Eigen::Matrix<double, 1, 6> J_T_WS;
    Eigen::Matrix<double, 1, 3> J_t_SR_S;
    if (is_estimate_body_) {
      // Body position in ENU
      Eigen::Matrix<double, 1, 3> J_t_W = J_t_ECEF * 
        coordinate_->rotationMatrix(GeoType::ECEF, GeoType::ENU);

      // Body rotation in ENU
      Eigen::Matrix<double, 1, 3> J_q_WS = J_t_W * 
        q_WS.toRotationMatrix() * skewSymmetric(t_SR_S);

      // Body pose in ENU
      J_T_WS.setZero();
      J_T_WS.topLeftCorner(1, 3) = J_t_W;
      J_T_WS.topRightCorner(1, 3) = J_q_WS;

      // Relative position 
      J_t_SR_S = J_t_W * q_WS.toRotationMatrix();
    }

    // Clock
    Eigen::Matrix<double, 1, 1> J_clock = -Eigen::MatrixXd::Identity(1, 1);

    // Troposphere
    Eigen::Matrix<double, 1, 1> J_trop = -gmf_wet * Eigen::MatrixXd::Identity(1, 1);

    // Ionosphere
    Eigen::Matrix<double, 1, 1> J_iono = -Eigen::MatrixXd::Identity(1, 1);

    // Group 1
    if (parameter_block_group_ == 1 || parameter_block_group_ == 3) 
    {
      // Position
      if (jacobians[0] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> J0(jacobians[0]);
        J0 = square_root_information * J_t_ECEF;
        
        if (jacobians_minimal != nullptr && jacobians_minimal[0] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor> >
              J0_minimal_mapped(jacobians_minimal[0]);
          J0_minimal_mapped = J0;
        }
      }
      // Clock
      if (jacobians[1] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J1(jacobians[1]);
        J1 = square_root_information * J_clock;

        if (jacobians_minimal != nullptr && jacobians_minimal[1] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor> >
              J1_minimal_mapped(jacobians_minimal[1]);
          J1_minimal_mapped = J1;
        }
      }
      // Troposphere
      if (is_estimate_atmosphere_ && jacobians[2] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J2(jacobians[2]);
        J2 = square_root_information * J_trop;

        if (jacobians_minimal != nullptr && jacobians_minimal[2] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor> >
              J2_minimal_mapped(jacobians_minimal[2]);
          J2_minimal_mapped = J2;
        }
      }
      // Ionosphere
      if (is_estimate_atmosphere_ && jacobians[3] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J3(jacobians[3]);
        J3 = square_root_information * J_iono;

        if (jacobians_minimal != nullptr && jacobians_minimal[3] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor> >
              J3_minimal_mapped(jacobians_minimal[3]);
          J3_minimal_mapped = J3;
        }
      }
    }
    // // Group 2
    if (parameter_block_group_ == 2 || parameter_block_group_ == 4)
    {
      // Pose
      if (jacobians[0] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> J0(jacobians[0]);
        Eigen::Matrix<double, 1, 6, Eigen::RowMajor> J0_minimal;
        J0_minimal = square_root_information * J_T_WS;

        // pseudo inverse of the local parametrization Jacobian:
        Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
        PoseLocalParameterization::liftJacobian(parameters[2], J_lift.data());

        J0 = J0_minimal * J_lift;

        if (jacobians_minimal != nullptr && jacobians_minimal[0] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor> >
              J0_minimal_mapped(jacobians_minimal[0]);
          J0_minimal_mapped = J0_minimal;
        }
      }
      // Relative position
      if (jacobians[1] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> J1(jacobians[1]);
        J1 = J_t_SR_S;

        if (jacobians_minimal != nullptr && jacobians_minimal[1] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor> >
              J1_minimal_mapped(jacobians_minimal[1]);
          J1_minimal_mapped = J1;
        }
      }
      // Clock
      if (jacobians[2] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J2(jacobians[2]);
        J2 = J_clock;

        if (jacobians_minimal != nullptr && jacobians_minimal[2] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor> >
              J2_minimal_mapped(jacobians_minimal[2]);
          J2_minimal_mapped = J2;
        }
      }
      // Troposphere
      if (is_estimate_atmosphere_ && jacobians[3] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J3(jacobians[3]);
        J3 = square_root_information * J_trop;

        if (jacobians_minimal != nullptr && jacobians_minimal[3] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor> >
              J3_minimal_mapped(jacobians_minimal[3]);
          J3_minimal_mapped = J3;
        }
      }
      // Ionosphere
      if (is_estimate_atmosphere_ && jacobians[4] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J4(jacobians[4]);
        J4 = square_root_information * J_iono;

        if (jacobians_minimal != nullptr && jacobians_minimal[4] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor> >
              J4_minimal_mapped(jacobians_minimal[4]);
          J4_minimal_mapped = J4;
        }
      }
    }
  }

  return true;
}

}  // namespace gici
