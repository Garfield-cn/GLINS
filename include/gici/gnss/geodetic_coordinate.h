/**
* @Function: Geodetic coordinate for GNSS processing
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#pragma once

#include <Eigen/Core>
#include <memory>

#include "gici/utility/rtklib_safe.h"

namespace gici {

// Geodetic coordinate types
enum class GeoType {
  ECEF,
  LLA,
  ENU,
  NED
};

class GeoCoordinate {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GeoCoordinate(const Eigen::Vector3d& position_zero, 
                const GeoType type);
  GeoCoordinate() : lla_(Eigen::Vector3d::Zero()),
    lla_zero_(Eigen::Vector3d::Zero()), lla_zero_setted_(false) { }
  ~GeoCoordinate() { }

  // Set position
  // The input LLA should be in deg format
  void setPosition(const Eigen::Vector3d& position, 
                   const GeoType type);

  // Set zero position for ENU convertion
  void setZero(const Eigen::Vector3d& position, 
               const GeoType type);

  // Check if zero position was setted
  bool isZeroSetted() { return lla_zero_setted_; }

  // Get position
  Eigen::Vector3d get(const GeoType type);

  // Convert coordinate
  Eigen::Vector3d convert(const Eigen::Vector3d& position,
              const GeoType in_type, const GeoType out_type);

  // Rotation matrix
  Eigen::Matrix3d rotationMatrix(GeoType from, GeoType to);

  // Convert LLA in degree to LLA in rad
  inline static Eigen::Vector3d degToRad(Eigen::Vector3d& deg) {
    Eigen::Vector3d rad = deg;
    rad(0) *= D2R; rad(1) *= D2R;
    return rad;
  }

  // Convert LLA in rad to LLA in degree
  inline static Eigen::Vector3d radToDeg(Eigen::Vector3d& rad) {
    Eigen::Vector3d deg = rad;
    deg(0) *= R2D; deg(1) *= R2D;
    return deg;
  }

private:
  // Get position in LLA coordinate
  Eigen::Vector3d getLLA();

  // Get position in ECEF coordinate
  Eigen::Vector3d getECEF();

  // Get position in ENU coordinate
  Eigen::Vector3d getENU();

  // Get position in NED coordinate
  Eigen::Vector3d getNED();

  // Latitude, Longitude and Altitude stored in rad
  Eigen::Vector3d lla_;
  Eigen::Vector3d lla_zero_;
  bool lla_zero_setted_;
};

using GeoCoordinatePtr = std::shared_ptr<GeoCoordinate>;

} // namespace gici