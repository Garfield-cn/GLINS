/**
* @Function: Pseudorange residual block for ceres backend
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/gnss/pseudorange_error_sd.h"

#include "gici/gnss/gnss_common.h"
#include "gici/utility/transform.h"
#include "gici/optimizer/pose_local_parameterization.hpp"

namespace gici {

// Construct with measurement and information matrix
template<int... Ns>
PseudorangeErrorSD<Ns ...>::PseudorangeErrorSD(
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
    LOG(FATAL) << "PseudorangeErrorSD parameter blocks setup invalid!";
  }
}

// This evaluates the error term and additionally computes the Jacobians.
template<int... Ns>
bool PseudorangeErrorSD<Ns ...>::Evaluate(double const* const * parameters,
                                 double* residuals, double** jacobians) const
{
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, nullptr);
}

// This evaluates the error term and additionally computes
// the Jacobians in the minimal internal representation.
template<int... Ns>
bool PseudorangeErrorSD<Ns ...>::EvaluateWithMinimalJacobians(
    double const* const * parameters, double* residuals, double** jacobians,
    double** jacobians_minimal) const
{
  Eigen::Vector3d t_WR_ECEF, t_WS_W, t_SR_S;
  Eigen::Quaterniond q_WS;
  double dclock;
  double dtroposphere_delay = 0.0, dionosphere_delay = 0.0;
  double gmf_wet;
  
  // Position and clock
  if (!is_estimate_body_) 
  {
    t_WR_ECEF = Eigen::Map<const Eigen::Vector3d>(parameters[0]);
    dclock = parameters[1][0];
  }
  else 
  {
    // pose in ENU frame
    t_WS_W = Eigen::Map<const Eigen::Vector3d>(&parameters[0][0]);
    q_WS = Eigen::Map<const Eigen::Quaterniond>(&parameters[0][3]);

    // relative position
    t_SR_S = Eigen::Map<const Eigen::Vector3d>(parameters[1]);

    // clock
    dclock = parameters[2][0];

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

  double timestamp = measurement_1_.timestamp;

  double rho_1 = gnss_common::satelliteToReceiverDistance(satellite_1_.sat_position, t_WR_ECEF);
  double rho_2 = gnss_common::satelliteToReceiverDistance(
    satellite_2_.sat_position, measurement_2_.position);

  double elevation_1 = gnss_common::satelliteElevation(satellite_1_.sat_position, t_WR_ECEF);
  double elevation_2 = gnss_common::satelliteElevation(
    satellite_2_.sat_position, measurement_2_.position);
  double azimuth_1 = gnss_common::satelliteAzimuth(satellite_1_.sat_position, t_WR_ECEF);

  // Atmosphere
  if (!is_estimate_atmosphere_) 
  {
    // We think all atmosphere delays are eliminated by single-difference
  }
  else
  { 
    // use estimated atomspheric delays
    double dtroposphere_wet = 0.0;
    if (!is_estimate_body_) {
      dtroposphere_wet = parameters[2][0];
      dionosphere_delay = parameters[3][0];
    }
    else {
      dtroposphere_wet = parameters[3][0];
      dionosphere_delay = parameters[4][0];
    }

    // troposphere hydro-static delay
    double troposphere_delay_1 = gnss_common::troposphereSaastamoinen(
      timestamp, t_WR_ECEF, elevation_1);
    double troposphere_delay_2 = gnss_common::troposphereSaastamoinen(
      timestamp, t_WR_ECEF, elevation_2);
    // troposphere wet delay from augmentation
    gnss_common::troposphereGMF(timestamp, t_WR_ECEF, elevation_1, nullptr, &gmf_wet);
    dtroposphere_delay = troposphere_delay_1 - 
      troposphere_delay_2 + dtroposphere_wet * gmf_wet;
  }

  // Get estimate derivated measurement
  double dpseudorange_estimate = rho_1 - rho_2 + dclock
   + dtroposphere_delay + dionosphere_delay;

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
    // Receiver position in ECEF
    Eigen::Matrix<double, 1, 3> J_t_ECEF = 
      -((t_WR_ECEF - satellite_1_.sat_position) / rho_1).transpose();
    
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
