/**
 * @Function: Local LiDAR registration error
 *
 * @Author  : Jiahui Liu
 * @Email   : jh.liu@sjtu.edu.cn
 **/
#include "gici/lidar/local_registration_error.h"

#include "gici/utility/transform.h"
#include "gici/estimate/pose_local_parameterization.h"

namespace gici {

// Construct with measurement and information matrix
LocalRegistrationError::LocalRegistrationError(const Eigen::Vector4d& params, const Point_lidar p,
                                               const Eigen::Matrix<double, 1, 1>& information)
{
  setMeasurement(params, p);
  setInformation(information);
}

// This evaluates the error term and additionally computes the Jacobians.
bool LocalRegistrationError::Evaluate(double const* const* parameters, double* residuals,
                                      double** jacobians) const
{
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, nullptr);
}

// This evaluates the error term and additionally computes
// the Jacobians in the minimal internal representation.
bool LocalRegistrationError::EvaluateWithMinimalJacobians(double const* const* parameters,
                                                          double* residuals, double** jacobians,
                                                          double** jacobians_minimal) const
{
  Eigen::Vector3d t_WR_ECEF, t_WB_i, t_WB_k;
  Eigen::Quaterniond q_WB_i, q_WB_k;

  Eigen::Vector3d n = plane_params_.block<3, 1>(0, 0);

  // Current keyframe pose in ENU frame
  t_WB_i = Eigen::Map<const Eigen::Vector3d>(&parameters[0][0]);
  q_WB_i = Eigen::Map<const Eigen::Quaterniond>(&parameters[0][3]);

  // Last keyframe pose in ENU frame
  t_WB_k = Eigen::Map<const Eigen::Vector3d>(&parameters[1][0]);
  q_WB_k = Eigen::Map<const Eigen::Quaterniond>(&parameters[1][3]);

  // Transform scan point into the previous keyframe
  Eigen::Vector3d p_l;
  p_l.x() = current_point_.x;
  p_l.y() = current_point_.y;
  p_l.z() = current_point_.z;
  Eigen::Vector3d p_w = t_WB_i + q_WB_i * p_l;

  Eigen::Matrix3d R_WB_k = q_WB_k.toRotationMatrix();
  Eigen::Vector3d p_k = -R_WB_k.transpose() * t_WB_k + R_WB_k.transpose() * p_w;

  // Compute error
  double error = n.dot(p_k) + plane_params_(3);

  Eigen::Map<Eigen::Matrix<double, 1, 1>> weighted_error(residuals);
  weighted_error = square_root_information_ * error;

  // Compute Jacobian
  if (jacobians != nullptr) {
    // Jacobians
    Eigen::Matrix<double, 1, 6> J_T_i, J_T_k;
    Eigen::Matrix<double, 1, 3> J_t_i, J_t_k;
    Eigen::Matrix<double, 1, 3> J_q_i, J_q_k;

    J_t_i = n.transpose() * R_WB_k.transpose();
    // Use left-multiplicative perturbations for both body poses
    J_q_i = -n.transpose() * R_WB_k.transpose() * q_WB_i.toRotationMatrix() * skewSymmetric(p_l);

    J_T_i.setZero();
    J_T_i.block<1, 3>(0, 0) = J_t_i;
    J_T_i.block<1, 3>(0, 3) = J_q_i;

    J_t_k = -n.transpose() * R_WB_k.transpose();
    J_q_k = n.transpose() * skewSymmetric(R_WB_k.transpose() * (p_w - t_WB_k));

    J_T_k.setZero();
    J_T_k.block<1, 3>(0, 0) = J_t_k;
    J_T_k.block<1, 3>(0, 3) = J_q_k;

    if (jacobians[0] != nullptr) {
      Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> J0(jacobians[0]);
      Eigen::Matrix<double, 1, 6, Eigen::RowMajor> J0_minimal;
      J0_minimal = J_T_i;

      // Lift the minimal Jacobian to the full pose parameterization
      Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
      PoseLocalParameterization::liftJacobian(parameters[0], J_lift.data());

      J0 = J0_minimal * J_lift;

      if (jacobians_minimal != nullptr && jacobians_minimal[0] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> J0_minimal_mapped(
            jacobians_minimal[0]);
        J0_minimal_mapped = J0_minimal;
      }
    }

    if (jacobians[1] != nullptr) {
      Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> J1(jacobians[1]);
      Eigen::Matrix<double, 1, 6, Eigen::RowMajor> J1_minimal;
      J1_minimal = J_T_k;

      // Lift the minimal Jacobian to the full pose parameterization
      Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
      PoseLocalParameterization::liftJacobian(parameters[1], J_lift.data());

      J1 = J1_minimal * J_lift;

      if (jacobians_minimal != nullptr && jacobians_minimal[1] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> J1_minimal_mapped(
            jacobians_minimal[1]);
        J1_minimal_mapped = J1_minimal;
      }
    }
  }

  return true;
}

}  // namespace gici
