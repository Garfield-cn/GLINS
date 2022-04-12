/**
* @Function: Pseudorange residual block for ceres backend
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#pragma once

#pragma diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// Eigen 3.2.7 uses std::binder1st and std::binder2nd which are deprecated since c++11
// Fix is in 3.3 devel (http://eigen.tuxfamily.org/bz/show_bug.cgi?id=872).
#include <ceres/ceres.h>
#include <Eigen/Core>
#pragma diagnostic pop

#include "gici/optimizer/error_interface.hpp"
#include "gici/gnss/geodetic_coordinate.h"
#include "gici/gnss/gnss_types.h"

namespace gici {

// Pesudorange error
// The parameter blocks Ns are indefinite, which enables user to flexibly estimate 
// different groups of parameters, including:
// Group 1: P1. receiver position in ECEF (3), P2. receiver clock (1)
// Group 2: P1. body pose in ENU (7), P2. relative position from body to receiver
//          in body frame (3), P3. receiver clock (1)
// Group 3: Group 1 + P3. troposphere delay (1), P4. ionosphere delay (1)
// Group 4: Group 2 + P4. troposphere delay (1), P5. ionosphere delay (1)
template<int... Ns>
class PseudorangeError :
    public ceres::SizedCostFunction<
    1 /* number of residuals */,
    Ns ... /* parameter blocks */>,
    public ErrorInterface
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief The base class type.
  typedef ceres::SizedCostFunction<1, Ns ...> base_t;

  /// \brief Number of residuals (1).
  static const int kNumResiduals = 1;

  /// \brief The information matrix type (1x1).
  typedef double information_t;

  /// \brief The covariance matrix type (same as information).
  typedef double covariance_t;

  /// \brief Default constructor.
  PseudorangeError();

  /// \brief Construct with measurement and information matrix
  /// @param[in] measurement The measurement.
  /// @param[in] index Index of current satellite.
  /// @param[in] error_parameter To compute GNSS information matrix.
  PseudorangeError(const GNSSMeasurement& measurement,
                   const GNSSMeasurementIndex index,
                   const GNSSErrorParameter& error_parameter);

  /// \brief Trivial destructor.
  virtual ~PseudorangeError() {}

  // setters
  /// \brief Set the measurement.
  /// @param[in] measurement The measurement.
  void setMeasurement(const GNSSMeasurement& measurement)
  {
    measurement_ = measurement;
  }

  // Set coordinate for ENU to ECEF convertion
  void setCoordinate(const GeoCoordinatePtr& coordinate) {
    coordinate_ = coordinate;
  }

  // getters
  /// \brief Get the measurement.
  /// \return The measurement vector.
  const GNSSMeasurement& measurement() const { return measurement_; }

  // error term and Jacobian implementation
  /**
    * @brief This evaluates the error term and additionally computes the Jacobians.
    * @param parameters Pointer to the parameters (see ceres)
    * @param residuals Pointer to the residual vector (see ceres)
    * @param jacobians Pointer to the Jacobians (see ceres)
    * @return success of th evaluation.
    */
  virtual bool Evaluate(double const* const * parameters, double* residuals,
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
  bool EvaluateWithMinimalJacobians(double const* const * parameters,
                                    double* residuals, double** jacobians,
                                    double** jacobians_minimal) const;

  // sizes
  /// \brief Residual dimension.
  size_t residualDim() const { return kNumResiduals; }

  /// \brief Number of parameter blocks.
  size_t parameterBlocks() const { return dims_.kNumParameterBlocks; }

  /// \brief Dimension of an individual parameter block.
  size_t parameterBlockDim(size_t parameter_block_idx) const
  {
    return dims_.GetDim(parameter_block_idx);
  }

  /// @brief Residual block type as string
  virtual ErrorType typeInfo() const
  {
    return ErrorType::kPseudorangeError;
  }

 protected:
  GNSSMeasurement measurement_; ///< The measurement.
  Satellite satellite_;
  Observation observation_;

  // weighting related
  GNSSErrorParameter error_parameter_;

  // Parameter dimensions
  ceres::internal::StaticParameterDims<Ns...> dims_;

  // Geodetic coordinate
  GeoCoordinatePtr coordinate_;

  // parameter types
  bool is_estimate_body_;
  bool is_estimate_atmosphere_;
  int parameter_block_group_;
};

// Explicitly instantiate template classes
template class PseudorangeError<3, 1>;  // Block 1
template class PseudorangeError<7, 3, 1>;  // Block 2
template class PseudorangeError<3, 1, 1, 1>;  // Block 3
template class PseudorangeError<7, 3, 1, 1, 1>;  // Block 4

}  

