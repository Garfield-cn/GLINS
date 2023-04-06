/**
* @Function: Convert file from GICI IMU pack to rosbag
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "gici/stream/format_image.h"
#include "gici/gnss/gnss_common.h"

using namespace gici;

int main(int argc, char** argv)
{
  // // Get file
  // if (argc != 2) {
  //   std::cerr << "Invalid input variables! Supported variables are: "
  //             << "<path-to-executable> <path-to-config>" << std::endl;
  //   return -1;
  // }
  // std::string config_file_path = argv[1];
  // YAML::Node yaml_node;
  // try {
  //    yaml_node = YAML::LoadFile(config_file_path);
  // } catch (YAML::BadFile &e) {
  //   std::cerr << "Unable to load config file!" << std::endl;
  //   return -1;
  // }

  // // Initialize logging
  // google::InitGoogleLogging("gici_tools");
  // FLAGS_logtostderr = true;

  // // Load options
  // NodeOptionHandlePtr nodes = 
  //   std::make_shared<NodeOptionHandle>(yaml_node);
  // if (!nodes->valid) {
  //   std::cerr << "Invalid base configurations!" << std::endl;
  //   return -1;
  // }

  return 0;
}