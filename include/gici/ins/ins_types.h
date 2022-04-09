/**
* @Function: INS types
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Core>

namespace gici {

namespace INS {

// Role of formator
enum class Role {
  None,
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
