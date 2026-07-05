/**
 * @Function: Global LiDAR registration error
 *
 * @Author  : Jiahui Liu
 * @Email   : jh.liu@sjtu.edu.cn
 **/
#include "gici/lidar/global_registration_error.h"

#include "gici/utility/transform.h"
#include "gici/estimate/pose_local_parameterization.h"

namespace gici {

// Construct with measurement and information matrix
GlobalRegistrationError::GlobalRegistrationError(const Eigen::Vector4d& params, const Point_lidar p,
                                                 const Eigen::Matrix<double, 1, 1>& information)
{
  setMeasurement(params, p);
  setInformation(information);
}

// This evaluates the error term and additionally computes the Jacobians.
bool GlobalRegistrationError::Evaluate(double const* const* parameters, double* residuals,
                                       double** jacobians) const
{
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, nullptr);
}

// This evaluates the error term and additionally computes
// the Jacobians in the minimal internal representation.
bool GlobalRegistrationError::EvaluateWithMinimalJacobians(double const* const* parameters,
                                                           double* residuals, double** jacobians,
                                                           double** jacobians_minimal) const
{
  Eigen::Vector3d t_WR_ECEF, t_WS_W, t_SR_S;
  Eigen::Quaterniond q_WS;

  Eigen::Vector3d n = plane_params_.block<3, 1>(0, 0);

  // Body pose in ENU
  t_WS_W = Eigen::Map<const Eigen::Vector3d>(&parameters[0][0]);
  q_WS = Eigen::Map<const Eigen::Quaterniond>(&parameters[0][3]);

  // Transform scan point from LiDAR frame to world frame
  Eigen::Vector3d p_l;
  p_l.x() = current_point_.x;
  p_l.y() = current_point_.y;
  p_l.z() = current_point_.z;
  Eigen::Vector3d p_w = t_WS_W + q_WS * p_l;

  // Compute error
  double error = n.dot(p_w) + plane_params_(3);

  Eigen::Map<Eigen::Matrix<double, 1, 1>> weighted_error(residuals);
  weighted_error = square_root_information_ * error;

  // Compute Jacobian
  if (jacobians != nullptr) {
    // Jacobians
    Eigen::Matrix<double, 1, 6> J_T_WS;
    Eigen::Matrix<double, 1, 3> J_t;
    Eigen::Matrix<double, 1, 3> J_q;

    J_t = n.transpose();
    // Left-multiplicative rotation perturbation of the transformed LiDAR point in frame W.
    J_q = -n.transpose() * skewSymmetric(q_WS.toRotationMatrix() * p_l);

    J_T_WS.setZero();
    J_T_WS.block<1, 3>(0, 0) = J_t;
    J_T_WS.block<1, 3>(0, 3) = J_q;

    if (jacobians[0] != nullptr) {
      Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> J0(jacobians[0]);
      Eigen::Matrix<double, 1, 6, Eigen::RowMajor> J0_minimal;
      J0_minimal = J_T_WS;

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
  }

  return true;
}

}  // namespace gici
