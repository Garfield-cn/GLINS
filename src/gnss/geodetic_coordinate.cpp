/**
* @Function: Geodetic coordinate for GNSS processing
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/gnss/geodetic_coordinate.h"

#include <glog/logging.h>
#include <Eigen/Geometry>

namespace gici {

GeoCoordinate::GeoCoordinate(const Eigen::Vector3d& position, 
                             const GeoType type) : GeoCoordinate()
{
  setPosition(position, type);
}

// Set position
void GeoCoordinate::setPosition(const Eigen::Vector3d& position, 
                  const GeoType type)
{
  lla_ = convertToLLA(position, type);
}

// Set zero position for ENU convertion
void GeoCoordinate::setZero(const Eigen::Vector3d& position, 
              const GeoType type)
{
  lla_zero_ = convertToLLA(position, type);
  lla_zero_setted_ = true;
}

// Get position
Eigen::Vector3d GeoCoordinate::get(const GeoType type)
{
  if (type == GeoType::LLA) return getLLA();
  if (type == GeoType::ECEF) return getECEF();
  if (type == GeoType::ENU) return getENU();
  if (type == GeoType::NED) return getNED();
  return Eigen::Vector3d::Zero();
}

// Get position in LLA coordinate
Eigen::Vector3d GeoCoordinate::getLLA(void)
{
  return lla_;
}

// Get position in ECEF coordinate
Eigen::Vector3d GeoCoordinate::getECEF(void)
{
  double r[3];
  pos2ecef(lla_.data(), r);
  return Eigen::Vector3d(r);
}

// Get position in ENU coordinate
Eigen::Vector3d GeoCoordinate::getENU(void)
{
  if (!lla_zero_setted_) {
    LOG(ERROR) << "Inital position not setted!";
    return Eigen::Vector3d::Zero();
  }

  double r[3], r0[3], dr[3], enu[3];
  pos2ecef(lla_.data(), r);
  pos2ecef(lla_zero_.data(), r0);
  for (int i = 0; i < 3; i++) dr[i] = r[i] - r0[i];
  ecef2enu(lla_zero_.data(), dr, enu);
  return Eigen::Vector3d(enu);
}

// Get position in NED coordinate
Eigen::Vector3d GeoCoordinate::getNED(void)
{
  Eigen::Vector3d enu = getENU();
  return Eigen::Vector3d(enu(1), enu(0), -enu(2));
}

// ECEF to ENU rotation matrix
Eigen::Matrix3d GeoCoordinate::rotationMatrix(GeoType from, GeoType to)
{
  if (!lla_zero_setted_) {
    LOG(ERROR) << "Inital position not setted!";
    return Eigen::Matrix3d::Identity();
  }

  if (from == GeoType::ECEF) {
    if (to == GeoType::ENU) {
      double E[9];
      xyz2enu(lla_zero_.data(), E);
      return Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::ColMajor>>(E);
    }
    if (to == GeoType::NED) {
      Eigen::Matrix3d R_ECEF_ENU = rotationMatrix(GeoType::ECEF, GeoType::ENU);
      Eigen::Matrix3d R_ENU_NED = rotationMatrix(GeoType::ENU, GeoType::NED);
      return R_ECEF_ENU * R_ENU_NED;
    }
  }

  if (from == GeoType::ENU) {
    if (to == GeoType::ECEF) {
      return rotationMatrix(GeoType::ECEF, GeoType::ENU).transpose();
    }
    if (to == GeoType::NED) {
      Eigen::AngleAxisd x(PI, Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd y(0.0, Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd z(PI / 2.0, Eigen::Vector3d::UnitZ());
      Eigen::Quaterniond q_ENU_NED = z * y * x;
      return q_ENU_NED.toRotationMatrix();
    }
  }

  if (from == GeoType::NED) {
    if (to == GeoType::ECEF) {
      Eigen::Matrix3d R_NED_ENU = rotationMatrix(GeoType::NED, GeoType::ENU);
      Eigen::Matrix3d R_ENU_ECEF = rotationMatrix(GeoType::ENU, GeoType::ECEF);
      return R_NED_ENU * R_ENU_ECEF;
    }
    if (to == GeoType::ENU) {
      return rotationMatrix(GeoType::ENU, GeoType::NED).transpose();
    }
  }

  LOG(ERROR) << "Invalid rotation matrix type!";
  return Eigen::Matrix3d::Identity();
}

// Convert any coordinate to LLA
Eigen::Vector3d GeoCoordinate::convertToLLA(
  const Eigen::Vector3d& position, const GeoType type)
{
  // Convert input coordinate to LLA
  Eigen::Vector3d lla;
  if (type == GeoType::ECEF) {
    double pos[3];
    ecef2pos(position.data(), pos);
    lla = Eigen::Vector3d(pos);
  }
  else if (type == GeoType::LLA) {
    lla = position;
  }
  else {
    LOG(ERROR) << "Input type not supported!";
  }

  return lla;
}

// Convert coordinate
Eigen::Vector3d GeoCoordinate::convet(const Eigen::Vector3d& position,
            const GeoType in_type, const GeoType out_type)
{
  if (out_type == GeoType::ENU) {
    LOG(ERROR) << "Inital position should be specified!";
    return Eigen::Vector3d::Zero();
  }

  GeoCoordinate coord(position, in_type);
  return coord.get(out_type);
}

// Convert coordinate
Eigen::Vector3d GeoCoordinate::convet(const Eigen::Vector3d& position,
            const Eigen::Vector3d& position_zero,
            const GeoType in_type, const GeoType out_type)
{
  GeoCoordinate coord(position, in_type);
  coord.setZero(position, in_type);
  return coord.get(out_type);
}

}
