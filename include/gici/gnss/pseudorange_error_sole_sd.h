/**
* @Function: Pseudorange residual block for ceres backend
*            Parameters are in ECEF coordinate. Observations are single-differenced (between stations).
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

/// \brief Pseudorange error, position in ECEF coordinate is used as parameter
// to apply single point positioning
class PseudorangeErrorSoleSD :
    public ceres::SizedCostFunction<
    1 /* number of residuals */,
    3, /* size of first parameter: GNSSMeasurement position in ECEF frame */
    1 /* size of second parameter: GNSSMeasurement clock */>,
    public ErrorInterface
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief The base class type.
  typedef ceres::SizedCostFunction<1, 3, 1> base_t;

  /// \brief Number of residuals (1).
  static const int kNumResiduals = 1;

  /// \brief The information matrix type (1x1).
  typedef double information_t;

  /// \brief The covariance matrix type (same as information).
  typedef double covariance_t;

  /// \brief Default constructor.
  PseudorangeErrorSoleSD();

  /// \brief Construct with measurement and information matrix
  /// @param[in] measurement_1 The measurement of rover.
  /// @param[in] measurement_2 The measurement of reference.
  /// @param[in] error_parameter To compute GNSS information matrix.
  PseudorangeErrorSoleSD(const GNSSMeasurement& measurement_1,
                         const GNSSMeasurement& measurement_2,
                         const GNSSMeasurementIndex index_1,
                         const GNSSMeasurementIndex index_2,
                         const GNSSErrorParameter& error_parameter);

  /// \brief Trivial destructor.
  virtual ~PseudorangeErrorSoleSD() {}

  // setters
  /// \brief Set the measurement.
  /// @param[in] measurement The measurement.
  void setMeasurement(const GNSSMeasurement& measurement_1,
                      const GNSSMeasurement& measurement_2)
  {
    measurement_1_ = measurement_1;
    measurement_2_ = measurement_2;
  }

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
  size_t parameterBlocks() const { return parameter_block_sizes().size(); }

  /// \brief Dimension of an individual parameter block.
  size_t parameterBlockDim(size_t parameter_block_idx) const
  {
    return base_t::parameter_block_sizes().at(parameter_block_idx);
  }

  /// @brief Residual block type as string
  virtual ErrorType typeInfo() const
  {
    return ErrorType::kPseudorangeError;
  }

 protected:
  GNSSMeasurement measurement_1_, measurement_2_; ///< The measurement.
  Satellite satellite_1_, satellite_2_;;
  Observation observation_1_, observation_2_;

  // weighting related
  GNSSErrorParameter error_parameter_;
};

}  

