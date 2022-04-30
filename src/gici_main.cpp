/**
* @Function: main function of gici library
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/stream/stream_handle.h"
#include "gici/estimate/estimate_handle.h"
#include "gici/utility/signal_handle.h"
#include "gici/utility/spin_control.h"

using namespace gici;

// Process streamers and estimators which defined in config.yaml file.
// You can use this executable by "<path-to-executable> <path-to-config>". 
// For more details on how to configure your config.yaml file, see example.yaml.
int main(int argc, char** argv)
{
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
    std::cerr << "Unable to load config file!"<< std::endl;
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
    std::cerr << "Unable to load stream options!";
    return -1;
  }
  YAML::Node stream_node = yaml_node["stream"];
  StreamHandle::Ptr stream_handle = std::make_shared<StreamHandle>(stream_node);

  // Initialize estimator threads
  EstimateHandle::Ptr estimate_handle;
  if (!yaml_node["estimate"].IsDefined()) {
    std::cerr << "Unable to load estimator options. Run in stream-only mode.";
  }
  else {
    estimate_handle = std::make_shared<EstimateHandle>(yaml_node);
    // bind with streamers
    estimate_handle->bindWithStreams(stream_handle);
  }
  
  // Loop
  SpinControl spin(1e-1);
  while (SpinControl::ok()) {
    spin.sleep();
  }

  return 0;
}