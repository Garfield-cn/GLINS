/**
* @Function: Geodetic coordinate for GNSS processing
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/gnss/geodetic_coordinate.h"

#include <glog/logging.h>

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
  pos2ecef(degToRad(lla_).data(), r);
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
  pos2ecef(degToRad(lla_).data(), r);
  pos2ecef(degToRad(lla_zero_).data(), r0);
  for (int i = 0; i < 3; i++) dr[i] = r[i] - r0[i];
  ecef2enu(degToRad(lla_zero_).data(), dr, enu);
  return Eigen::Vector3d(enu);
}

// Get position in NED coordinate
Eigen::Vector3d GeoCoordinate::getNED(void)
{
  Eigen::Vector3d enu = getENU();
  return Eigen::Vector3d(enu(1), enu(0), -enu(2));
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
    Eigen::Vector3d rad = Eigen::Vector3d(pos);
    lla = radToDeg(rad);
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
