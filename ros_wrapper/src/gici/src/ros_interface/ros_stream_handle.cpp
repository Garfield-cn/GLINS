/**
* @Function: Handle stream data input, log, and output
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/ros_interface/ros_stream_handle.h"

namespace gici {

RosStreamHandle::RosStreamHandle(ros::NodeHandle& nh, YAML::Node& node)
{
  // Initialize streamers
  if (!node["streamers"].IsDefined()) {
    LOG(ERROR) << "Unable to load streamers!";
    return;
  }
  YAML::Node streamer_nodes = node["streamers"];
  for (size_t i = 0; i < streamer_nodes.size(); i++) {
    std::string type_str;
    if (!option_tools::safeGet(streamer_nodes[i]["streamer"], "type", &type_str)) {
      continue;
    }
    StreamerType type;
    option_tools::convert(type_str, type);
    if (type != StreamerType::Ros) continue;
    streams_.push_back(std::make_shared<RosStream>(nh, node, i));
  }
}

RosStreamHandle::~RosStreamHandle()
{}

}