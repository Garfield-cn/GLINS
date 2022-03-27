/**
* @Function: Coordinate transform functions
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/utility/transform.h"

#include <rtklib.h>

namespace gici {

// Transform coordinate from ECEF to LLA
Eigen::Vector3d ecef2lla(const Eigen::Vector3d& ecef) 
{
  double r[3], pos[3];
  for (int i = 0; i < 3; i++) r[i] = ecef(i);
  ecef2pos(r, pos);
  Eigen::Vector3d lla;
  for (int i = 0; i < 3; i++) lla(i) = pos[i];
  return lla;
}

// Transform coordinate from LLA to ECEF
Eigen::Vector3d lla2ecef(const Eigen::Vector3d& lla) 
{
  double r[3], pos[3];
  for (int i = 0; i < 3; i++) pos[i] = lla(i);
  pos2ecef(pos, r);
  Eigen::Vector3d ecef;
  for (int i = 0; i < 3; i++) ecef(i) = r[i];
  return ecef;
}

}