/**
 * @Function: LiDAR plane landmark error
 *
 * @Author  : Jiahui Liu
 * @Email   : jh.liu@sjtu.edu.cn
 **/
#pragma once

#pragma diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// Eigen 3.2.7 uses std::binder1st and std::binder2nd which are deprecated since c++11
// Fix is in 3.3 devel (http://eigen.tuxfamily.org/bz/show_bug.cgi?id=872).
#include <ceres/ceres.h>
#include <Eigen/Core>
#pragma diagnostic pop

#include "gici/estimate/error_interface.h"
#include "gici/lidar/lidar_types.h"
#include "gici/estimate/homogeneous_point_local_parameterization.h"

namespace gici {

// Point-to-plane landmark error
// Parameter blocks: body pose in ENU (7), homogeneous plane center (4)
class PlaneError : public ceres::SizedCostFunction<1 /* number of residuals */,
                                                   7 /* size of first parameter blocks */,
                                                   4 /* size of second parameter */>,
                   public ErrorInterface {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief The base class type.
  typedef ceres::SizedCostFunction<1, 7, 4> base_t;

  /// \brief Number of residuals (1).
  static const int kNumResiduals = 1;

  /// \brief Scalar residual information, in inverse square meters.
  typedef Eigen::Matrix<double, 1, 1> information_t;

  /// \brief Scalar residual covariance, in square meters.
  typedef Eigen::Matrix<double, 1, 1> covariance_t;

  /// \brief Default constructor.
  PlaneError();

  /// \brief Construct with a plane normal, scan point, and information
  /// @param[in] normal Plane normal in the world frame.
  /// @param[in] p Scan point in the LiDAR frame.
  /// @param[in] information Scalar information.
  PlaneError(const Eigen::Vector3d& normal, const Eigen::Vector3d p,
             const Eigen::Matrix<double, 1, 1>& information);

  /// \brief Trivial destructor.
  virtual ~PlaneError()
  {}

  /// \brief Set the plane normal and scan point.
  /// @param[in] normal Plane normal in the world frame.
  /// @param[in] p Scan point in the LiDAR frame.
  void setMeasurement(const Eigen::Vector3d& normal, const Eigen::Vector3d p)
  {
    current_point_ = p;
    plane_normal_ = normal;
  }

  /// \brief Set the information.
  /// @param[in] information The information (weight) matrix.
  void setInformation(const information_t& information)
  {
    information_ = information;
    covariance_ = information.inverse();
    // Perform the Cholesky decomposition to obtain the error weighting
    Eigen::LLT<information_t> lltOfInformation(information_);
    square_root_information_ = lltOfInformation.matrixL().transpose();
    square_root_information_inverse_ = square_root_information_.inverse();
  }

  // error term and Jacobian implementation
  /**
   * @brief This evaluates the error term and additionally computes the Jacobians.
   * @param parameters Pointer to the parameters (see ceres)
   * @param residuals Pointer to the residual vector (see ceres)
   * @param jacobians Pointer to the Jacobians (see ceres)
   * @return Whether the evaluation succeeded.
   */
  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const;

  /**
   * @brief This evaluates the error term and additionally computes
   *        the Jacobians in the minimal internal representation.
   * @param parameters Pointer to the parameters (see ceres)
   * @param residuals Pointer to the residual vector (see ceres)
   * @param jacobians Pointer to the Jacobians (see ceres)
   * @param jacobians_minimal Pointer to the minimal Jacobians (equivalent to jacobians).
   * @return Success of the evaluation.
   */
  bool EvaluateWithMinimalJacobians(double const* const* parameters, double* residuals,
                                    double** jacobians, double** jacobians_minimal) const;

  // sizes
  /// \brief Residual dimension.
  size_t residualDim() const
  {
    return kNumResiduals;
  }

  /// \brief Number of parameter blocks.
  size_t parameterBlocks() const
  {
    return base_t::parameter_block_sizes().size();
  }

  /// \brief Dimension of an individual parameter block.
  size_t parameterBlockDim(size_t parameter_block_idx) const
  {
    return base_t::parameter_block_sizes().at(parameter_block_idx);
  }

  /// @brief Residual block type as string
  virtual ErrorType typeInfo() const
  {
    return ErrorType::kPlaneError;
  }

  // Convert normalized residual to raw residual
  virtual void deNormalizeResidual(double* residuals) const
  {
    Eigen::Map<Eigen::Matrix<double, 1, 1>> Residual(residuals);
    Residual = square_root_information_inverse_ * Residual;
  }

protected:
  Eigen::Vector3d plane_normal_;
  Eigen::Vector3d current_point_;

  // weighting related
  Eigen::Matrix<double, 1, 1> information_;
  Eigen::Matrix<double, 1, 1> square_root_information_;
  information_t square_root_information_inverse_;
  covariance_t covariance_;  ///< The DimxDim covariance matrix.

  // parameter types
  bool is_estimate_body_;
  int parameter_block_group_;
};

}
