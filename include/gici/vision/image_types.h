/**
* @Function: Image types
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#pragma once

#include <iostream>
#include <vector>

#include "gici/utility/svo.h"

namespace gici {

namespace vision {

// Role of formator
enum class Role {
  None,
  Mono, 
  StereoMajor,
  StereoMinor,
  Array
};

}

}
