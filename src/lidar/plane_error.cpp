/**
 * @Function: LiDAR plane landmark error
 *
 * @Author  : Jiahui Liu
 * @Email   : jh.liu@sjtu.edu.cn
 **/
#include "gici/lidar/plane_error.h"

#include "gici/utility/transform.h"
#include "gici/estimate/pose_local_parameterization.h"

namespace gici {

// Construct with measurement and information matrix
PlaneError::PlaneError(const Eigen::Vector3d& normal, const Eigen::Vector3d p,
                       const Eigen::Matrix<double, 1, 1>& information)
{
  setMeasurement(normal, p);
  setInformation(information);
}

// This evaluates the error term and additionally computes the Jacobians.
bool PlaneError::Evaluate(double const* const* parameters, double* residuals,
                          double** jacobians) const
{
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, nullptr);
}

// This evaluates the error term and additionally computes
// the Jacobians in the minimal internal representation.
bool PlaneError::EvaluateWithMinimalJacobians(double const* const* parameters, double* residuals,
                                              double** jacobians, double** jacobians_minimal) const
{
  Eigen::Vector3d t_WR_ECEF, t_WS_W, t_SR_S;
  Eigen::Quaterniond q_WS;

  // Body pose in ENU
  t_WS_W = Eigen::Map<const Eigen::Vector3d>(&parameters[0][0]);
  q_WS = Eigen::Map<const Eigen::Quaterniond>(&parameters[0][3]);

  Eigen::Vector3d p_w = t_WS_W + q_WS * current_point_;

  // Plane center in world coordinates
  Eigen::Map<const Eigen::Vector4d> p_W(&parameters[1][0]);

  // Compute error
  double error = plane_normal_.dot(p_w - p_W.head<3>());

  Eigen::Map<Eigen::Matrix<double, 1, 1>> weighted_error(residuals);
  weighted_error = square_root_information_ * error;

  // Compute Jacobian
  if (jacobians != nullptr) {
    // Jacobians
    Eigen::Matrix<double, 1, 6> J_T_WS;
    Eigen::Matrix<double, 1, 3> J_t;
    Eigen::Matrix<double, 1, 3> J_q;
    Eigen::Matrix<double, 1, 4> J_p;

    J_t = plane_normal_.transpose();
    // Left-multiplicative rotation perturbation in the world frame
    J_q = -plane_normal_.transpose() * skewSymmetric(q_WS.toRotationMatrix() * current_point_);

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

    if (jacobians[1] != nullptr) {
      // Lift the minimal Jacobian to the homogeneous point parameterization
      Eigen::Matrix<double, 3, 4, Eigen::RowMajor> J_lift;
      HomogeneousPointLocalParameterization::liftJacobian(parameters[1], J_lift.data());
      Eigen::Matrix<double, 4, 3, Eigen::RowMajor> J_plus;
      HomogeneousPointLocalParameterization::plusJacobian(parameters[1], J_plus.data());

      Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> J1(jacobians[1]);
      Eigen::Matrix<double, 1, 3, Eigen::RowMajor> J1_minimal =
          -plane_normal_.transpose() * J_lift * J_plus;

      // Convert the minimal Jacobian to the full parameterization
      J1 = J1_minimal * J_lift;

      if (jacobians_minimal != nullptr && jacobians_minimal[1] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> J1_minimal_mapped(
            jacobians_minimal[1]);
        J1_minimal_mapped = J1_minimal;
      }
    }
  }

  return true;
}

}  // namespace gici
