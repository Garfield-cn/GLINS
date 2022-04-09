/**
* @Function: Image types
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#ifndef IMAGE_TYPES_H
#define IMAGE_TYPES_H

#include <iostream>
#include <vector>
#include <svo/svo.h>

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

// Image epoch data
using Frame = svo::Frame;

}

}

#endif