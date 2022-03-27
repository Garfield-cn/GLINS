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

namespace camera {

// Role of formator
enum Role {
  NotCamera,
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