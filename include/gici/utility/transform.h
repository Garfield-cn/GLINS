/**
* @Function: Coordinate transform functions
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#pragma once

#include <iostream>
#include <Eigen/Core>

namespace gici {

// Transform coordinate from ECEF to LLA
Eigen::Vector3d ecef2lla(const Eigen::Vector3d& ecef);

// Transform coordinate from LLA to ECEF
Eigen::Vector3d lla2ecef(const Eigen::Vector3d& lla);

}

