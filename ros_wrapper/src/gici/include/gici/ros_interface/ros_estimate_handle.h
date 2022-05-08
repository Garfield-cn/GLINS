/**
* @Function: Handle estimators
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#pragma once

#include <iostream>
#include <vector>
#include <functional>
#include <glog/logging.h>

#include "gici/estimate/estimate_handle.h"
#include "gici/ros_interface/ros_stream_handle.h"

namespace gici {

class RosEstimateHandle : public EstimateHandle {
public:
  RosEstimateHandle(YAML::Node& node);
  ~RosEstimateHandle();

  // Bind estimators with ROS input and output streams
  void bindWithRosStreams(const std::shared_ptr<RosStreamHandle>& stream_handle);
};

}
