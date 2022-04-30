/**
* @Function: Option tools
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/utility/option.h"

#include <glog/logging.h>

#include "gici/stream/streamer.h"
#include "gici/stream/formator.h"
#include "gici/stream/streaming.h"
#include "gici/gnss/gnss_types.h"
#include "gici/imu/imu_types.h"
#include "gici/vision/image_types.h"
#include "gici/gnss/rtk_estimator.h"
#include "gici/fusion/gnss_imu_lc_estimator.h"
#include "gici/fusion/rtk_imu_tc_estimator.h"
#include "gici/estimate/estimator_types.h"

namespace gici {

namespace option_tools {

// Mapping from in to out
#define MAP_IN_OUT(x, y) if (in == x) { out = y; return; }
#define LOG_INVALId LOG(FATAL) << "Option " << in << " invalid!";

// Convert options from yaml type to gici type
template <typename InType, typename OutType>
void convert(const InType& in, OutType& out)
{
  LOG(FATAL) << "Convertion from " << typeid(in).name() 
         << " to " << typeid(out).name() << " not supported!";
}

template <>
void convert<std::string, StreamerType>
  (const std::string& in, StreamerType& out)
{
  MAP_IN_OUT("serial", StreamerType::Serial);
  MAP_IN_OUT("tcp-client", StreamerType::TcpClient);
  MAP_IN_OUT("tcp-server", StreamerType::TcpServer);
  MAP_IN_OUT("file", StreamerType::File);
  MAP_IN_OUT("ntrip-client", StreamerType::NtripClient);
  MAP_IN_OUT("ntrip-server", StreamerType::NtripServer);
  MAP_IN_OUT("v4l2", StreamerType::V4L2);
  LOG_INVALId;
}

template <>
void convert<std::string, FormatorType>
  (const std::string& in, FormatorType& out)
{
  MAP_IN_OUT("gnss-rtcm-2", FormatorType::RTCM2);
  MAP_IN_OUT("gnss-rtcm-3", FormatorType::RTCM3);
  MAP_IN_OUT("gnss-raw", FormatorType::GnssRaw);
  MAP_IN_OUT("image-v4l2", FormatorType::ImageV4L2);
  MAP_IN_OUT("image-pack", FormatorType::ImagePack);
  MAP_IN_OUT("imu-pack", FormatorType::IMUPack);
  MAP_IN_OUT("option", FormatorType::OptionPack);
  MAP_IN_OUT("nmea", FormatorType::NMEA);
  LOG_INVALId;
}

template <>
void convert<std::string, GnssRawFormats>
  (const std::string& in, GnssRawFormats& out)
{
  MAP_IN_OUT("ublox", GnssRawFormats::Ublox);
  MAP_IN_OUT("septentrio", GnssRawFormats::Septentrio);
  LOG_INVALId;
}

template <>
void convert<std::string, StreamIOType>
  (const std::string& in, StreamIOType& out)
{
  MAP_IN_OUT("input", StreamIOType::Input);
  MAP_IN_OUT("output", StreamIOType::Output);
  MAP_IN_OUT("log", StreamIOType::Log);
  LOG_INVALId;
}

template <>
void convert<std::string, GnssRole>
  (const std::string& in, GnssRole& out)
{
  MAP_IN_OUT("rover", GnssRole::Rover);
  MAP_IN_OUT("reference", GnssRole::Reference);
  MAP_IN_OUT("ephemeris", GnssRole::Ephemeris);
  MAP_IN_OUT("correction", GnssRole::Correction);
  MAP_IN_OUT("heading", GnssRole::Heading);
  LOG_INVALId;
}

template <>
void convert<std::string, ImuRole>
  (const std::string& in, ImuRole& out)
{
  MAP_IN_OUT("major", ImuRole::Major);
  MAP_IN_OUT("minor", ImuRole::Minor);
  LOG_INVALId;
}

template <>
void convert<std::string, CameraRole>
  (const std::string& in, CameraRole& out)
{
  MAP_IN_OUT("mono", CameraRole::Mono);
  MAP_IN_OUT("stereo-major", CameraRole::StereoMajor);
  MAP_IN_OUT("stereo-minor", CameraRole::StereoMinor);
  MAP_IN_OUT("array", CameraRole::Array);
  LOG_INVALId;
}

template <>
void convert<std::string, EstimatorType>
  (const std::string& in, EstimatorType& out)
{
  MAP_IN_OUT("rtk", EstimatorType::Rtk);
  MAP_IN_OUT("gnss_imu_lc", EstimatorType::GnssImuLc);
  MAP_IN_OUT("rtk_imu_tc", EstimatorType::RtkImuTc);
  LOG_INVALId;
}

// Mapping from in to return
#define MAP_IN_RET(x, y) if (in == x) { return y; }

// Get sensor type from options
SensorType sensorType(std::string in)
{
  // From formator role
  MAP_IN_RET("rover", SensorType::GNSS);
  MAP_IN_RET("reference", SensorType::GNSS);
  MAP_IN_RET("ephemeris", SensorType::GNSS);
  MAP_IN_RET("heading", SensorType::GNSS);
  MAP_IN_RET("major", SensorType::IMU);
  MAP_IN_RET("major", SensorType::IMU);
  MAP_IN_RET("mono", SensorType::Camera);
  MAP_IN_RET("stereo-major", SensorType::Camera);
  MAP_IN_RET("stereo-minor", SensorType::Camera);
  MAP_IN_RET("array", SensorType::Camera);
  MAP_IN_RET("option", SensorType::Option);

  // From formator type
  MAP_IN_RET("gnss-rtcm-2", SensorType::GNSS);
  MAP_IN_RET("gnss-rtcm-3", SensorType::GNSS);
  MAP_IN_RET("gnss-raw", SensorType::GNSS);
  MAP_IN_RET("image-v4l2", SensorType::Camera);
  MAP_IN_RET("image-pack", SensorType::Camera);
  MAP_IN_RET("imu-pack", SensorType::IMU);
  MAP_IN_RET("option", SensorType::Option);

  LOG_INVALId;
}

// Load option with info
#define LOAD_COMMON(opt) \
  if (!option_tools::safeGet(node, #opt, &options.opt)) { \
  LOG(INFO) << __FUNCTION__ << ": Unable to load " << #opt \
         << ". Using default instead."; }
// Load option with fatal error
#define LOAD_REQUIRED(opt) \
  if (!option_tools::safeGet(node, #opt, &options.opt)) { \
  LOG(FATAL) << __FUNCTION__ << ": Unable to load " << #opt << "!"; }

// Check sub-options exist
inline bool checkSubOption(
  YAML::Node& node, std::string subname, bool fatal = false)
{
  if (!node[subname].IsDefined()) {
    if (fatal) {
      LOG(FATAL) << "Unable to load " << subname << "!";
    }
    else {
      LOG(INFO) << "Unable to load " << subname << ". Using default instead.";
    }
    return false;
  }
  return true;
}

// split line by pattern
inline std::vector<std::string> split(std::string str, std::string pattern)
{
  std::string::size_type pos;
  std::vector<std::string> result;
  str += pattern;
  int size = str.size();
  for (int i = 0; i < size; i++) {
    pos = str.find(pattern, i);
    if (pos < size) {
      std::string s = str.substr(i, pos - i);
      result.push_back(s);
      i = pos + pattern.size() - 1;
    }
  }
  return result;
}

// delete space
inline void delete_space(std::string& strs)
{
  strs.erase(0, strs.find_first_not_of(' '));
  strs.erase(strs.find_last_not_of(' ') + 1, 
    strs.size() - strs.find_last_not_of(' ') - 1);
}

// delete space for each string in vector
inline void delete_spaces(std::vector<std::string>& strs)
{
  for (size_t i = 0; i < strs.size(); i++) {
    delete_space(strs[i]);
  }
}

// Load options
template <>
void loadOptions<GnssCommonOptions>(
    YAML::Node& node, GnssCommonOptions& options)
{
  LOAD_COMMON(min_elevation);
  LOAD_COMMON(mw_slip_thres);
  LOAD_COMMON(gf_slip_thres);
  LOAD_COMMON(gf_sd_slip_thres);
  LOAD_COMMON(period);

  std::string system_exclude;
  if (option_tools::safeGet(node, "system_exclude", &system_exclude)) {
    std::vector<std::string> strings = split(system_exclude, std::string(","));
    delete_spaces(strings);
    options.system_exclude.clear();
    for (auto str : strings) {
      options.system_exclude.push_back(str[0]);
    }
  }

  std::string satellite_exclude;
  if (option_tools::safeGet(node, "satellite_exclude", &satellite_exclude)) {
    std::vector<std::string> strings = split(satellite_exclude, std::string(","));
    delete_spaces(strings);
    options.satellite_exclude.clear();
    for (auto str : strings) {
      options.satellite_exclude.push_back(str);
    }
  }

  std::string code_exclude;
  if (option_tools::safeGet(node, "code_exclude", &code_exclude)) {
    std::vector<std::string> strings = split(code_exclude, std::string(","));
    delete_spaces(strings);
    // options.code_exclude.clear();
    for (auto str : strings) {
      // TODO
    }
  }
}

template <>
void loadOptions<GnssErrorParameter>(
    YAML::Node& node, GnssErrorParameter& options)
{
  LOAD_COMMON(code_to_phase_ratio);
  LOAD_COMMON(phase_error_factor);
  LOAD_COMMON(ionosphere_broadcast_factor);
  LOAD_COMMON(ionosphere_dual_frequency);
  LOAD_COMMON(ionosphere_augment);
  LOAD_COMMON(troposphere_model_factor);
  LOAD_COMMON(troposphere_augment);
  LOAD_COMMON(doppler_frequency);
  LOAD_COMMON(ephemeris_broadcast);
  LOAD_COMMON(ephemeris_precise);

  std::vector<double> system_error_ratio;
  if (option_tools::safeGet(node, "system_error_ratio", &system_error_ratio)) {
    options.system_error_ratio.at('G') = system_error_ratio[0];
    options.system_error_ratio.at('R') = system_error_ratio[1];
    options.system_error_ratio.at('E') = system_error_ratio[2];
    options.system_error_ratio.at('C') = system_error_ratio[3];
  }
  else {
    LOG(INFO) << "Unable to load system_error_ratio. Using default instead.";
  }
}

template <>
void loadOptions<AmbiguityResolutionOptions>(
    YAML::Node& node, AmbiguityResolutionOptions& options)
{
  LOAD_COMMON(min_percentage_fixation_nl);
  LOAD_COMMON(min_percentage_fixation_wl);
  LOAD_COMMON(min_percentage_fixation_uwl);
  LOAD_COMMON(ratio);
  LOAD_COMMON(norm_phase_residual_reject_thres);
  LOAD_COMMON(min_consistant_fix_as_stable);

  std::string system_exclude;
  if (option_tools::safeGet(node, "system_exclude", &system_exclude)) {
    std::vector<std::string> strings = split(system_exclude, std::string(","));
    delete_spaces(strings);
    options.system_exclude.clear();
    for (auto str : strings) {
      options.system_exclude.push_back(str[0]);
    }
  }
}

template <>
void loadOptions<RtkEstimatorOptions>(
    YAML::Node& node, RtkEstimatorOptions& options)
{
  LOAD_COMMON(max_iteration);
  LOAD_COMMON(num_threads);
  LOAD_COMMON(verbose);
  LOAD_COMMON(max_age);
  LOAD_COMMON(window_length);
  LOAD_COMMON(use_ambiguity_resolution);

  if (checkSubOption(node, "gnss_common_options")) {
    YAML::Node subnode = node["gnss_common_options"];
    loadOptions(subnode, options.common);
  }

  if (checkSubOption(node, "gnss_error_parameter")) {
    YAML::Node subnode = node["gnss_error_parameter"];
    loadOptions(subnode, options.error_parameter);
  }

  if (checkSubOption(node, "ambiguity_resolution_options")) {
    YAML::Node subnode = node["ambiguity_resolution_options"];
    loadOptions(subnode, options.ambiguity_resolution);
  }
}

}

}