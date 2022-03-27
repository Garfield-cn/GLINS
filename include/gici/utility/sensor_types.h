/**
* @Function: Sensor types
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#ifndef TYPES_H
#define TYPES_H

#include <iostream>
#include <memory>
#include <Eigen/Core>

namespace gici {

// GNSS satellite position type
enum SatPositionType {
  Broadcast,
  Precise,
  SBAS
};

// GNSS satellite position
struct SatPotition {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::string prn;  
  SatPositionType type;
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
};

// GNSS observations
struct SatObservation {
  std::string prn;
  double pseudorange;
  double phase;
  double doppler;
  double SNR; // Signle to noise ratio
  bool LLI;   // Loss of lock indicator
};

}

#endif