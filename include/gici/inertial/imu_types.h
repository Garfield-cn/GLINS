/**
* @Function: IMU types
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#ifndef IMU_TYPES_H
#define IMU_TYPES_H

#include <iostream>
#include <vector>
#include <Eigen/Core>

namespace gici {

namespace INS {

// Role of formator
enum Role {
  NotINS,
  Major, 
  Minor
};

// IMU epoch data
struct Epoch {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double time;
  Eigen::Vector3d acceleration;
  Eigen::Vector3d angular_velocity;
};

}

}

#endif