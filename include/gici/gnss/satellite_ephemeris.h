/**
* @Function: Satellite ephemeris
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#pragma once

#include <iostream>
#include <memory>
#include <Eigen/Core>
#include <rtklib.h>

#include "gici/stream/formator.h"

namespace gici {

// GNSS satellite position type
enum class SatPositionType {
  None,
  Broadcast,
  Precise,
  SBAS
};

// GNSS satellite position and velocity
struct SatPosition {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::string prn;
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  double clock;
  double frequency;
  SatPositionType type;
};

// GNSS satellite ephemeris
class SatEphemeris {
public:
  SatEphemeris();
  ~SatEphemeris();

  // Set new ephemeris
  int set(nav_t *nav);

  // Get satellite position and velocity
  int getSatPosition(double time, 
          const std::vector<std::string>& prns, 
          std::vector<SatPosition>& sat_positions);

protected:
  nav_t nav_;
};

}
