/**
* @Function: Option tools
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#ifndef OPTION_H
#define OPTION_H

#include <iostream>
#include <glog/logging.h>
#include <aslam/common/yaml-serialization.h>

namespace gici {

namespace option_tools {

enum SensorType {
  GNSS,
  IMU,
  Camera,
  Option
};

// Convert options from yaml type to gici type
template <typename InType, typename OutType>
void convert(const InType& in, OutType& out);

// Get sensor type from options
SensorType sensorType(std::string in);

}

}

#endif