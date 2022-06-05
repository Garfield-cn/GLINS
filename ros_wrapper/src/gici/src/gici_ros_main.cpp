/**
* @Function: main function of the ROS wrapper of gici library
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include <ros/ros.h>

#include "gici/ros_interface/ros_stream_handle.h"
#include "gici/ros_interface/ros_estimate_handle.h"
#include "gici/utility/signal_handle.h"

using namespace gici;

// Process streamers and estimators which defined in config.yaml file.
// Usage: rosrun gici ros_gici_main <path-to-config> 
//     or roslaunch gici gici.launch.
// Instructions for basic setting is shown in ../../../../option/example.yaml,
// Instructions for ROS specific settings is shown in option/ros_example.yaml
int main(int argc, char** argv)
{
  std::ofstream outfile;
  outfile.open("/home/cc/datasets/tmp/log.txt", std::ios::out | std::ios::trunc);
  outfile.close();

  // Initialize ROS
  ros::init(argc, argv, "gici");
  ros::NodeHandle nh("~");

  // Get config file
  if (argc != 2) {
    std::cerr << "Invalid input variables! Supported variables are: "
              << "<path-to-executable> <path-to-config>" << std::endl;
    return -1;
  }
  std::string config_file_path = argv[1];
  YAML::Node yaml_node;
  try {
     yaml_node = YAML::LoadFile(config_file_path);
  } catch(YAML::BadFile &e) {
    std::cerr << "Unable to load config file!" << std::endl;
    return -1;
  }

  // Initialize glog for logging
  bool enable_logging = false;
  if (yaml_node["logging"].IsDefined() && 
      option_tools::safeGet(yaml_node["logging"], "enable", &enable_logging) && 
      enable_logging == true) {
    YAML::Node logging_node = yaml_node["logging"];
    google::InitGoogleLogging("gici");
    int min_log_level = 0;
    if (option_tools::safeGet(
        logging_node, "min_log_level", &min_log_level)) {
      FLAGS_minloglevel = min_log_level;
      FLAGS_stderrthreshold = min_log_level;
    }
    option_tools::safeGet(logging_node, "log_to_stderr", &FLAGS_logtostderr);
    option_tools::safeGet(logging_node, "file_directory", &FLAGS_log_dir);
  }

  // Initialize signal handles to catch faults
  initializeSignalHandles();

  // Initialize streamer threads
  if (!yaml_node["stream"].IsDefined()) {
    std::cerr << "Unable to load stream options!" << std::endl;
    return -1;
  }
  YAML::Node stream_node = yaml_node["stream"];
  std::shared_ptr<StreamHandle> stream_handle = std::make_shared<StreamHandle>(stream_node);
  std::shared_ptr<RosStreamHandle> ros_stream_handle = 
    std::make_shared<RosStreamHandle>(nh, stream_node);

  // Initialize estimator threads
  std::shared_ptr<RosEstimateHandle> estimate_handle;
  if (!yaml_node["estimate"].IsDefined()) {
    std::cerr << "Unable to load estimator options. Run in stream-only mode." << std::endl;
  }
  else {
    estimate_handle = std::make_shared<RosEstimateHandle>(yaml_node);
    // bind with streamers
    estimate_handle->bindWithStreams(stream_handle);
    // bind with estimators
    estimate_handle->bindWithEstimators();
    // bind with ROS streams
    estimate_handle->bindWithRosStreams(ros_stream_handle);
  }
  
  // Loop
  ros::spin();

  return 0;
}