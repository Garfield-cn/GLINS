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
#include "gici/gnss/spp_estimator.h"
#include "gici/gnss/dgnss_estimator.h"
#include "gici/fusion/gnss_imu_lc_estimator.h"
#include "gici/fusion/rtk_imu_tc_estimator.h"
#include "gici/estimate/estimator_types.h"
#include "gici/vision/feature_handler.h"

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
  MAP_IN_OUT("ros", StreamerType::Ros);
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
  MAP_IN_OUT("spp", EstimatorType::Spp);
  MAP_IN_OUT("dgnss", EstimatorType::Dgnss);
  MAP_IN_OUT("rtk", EstimatorType::Rtk);
  MAP_IN_OUT("gnss_imu_lc", EstimatorType::GnssImuLc);
  MAP_IN_OUT("rtk_imu_tc", EstimatorType::RtkImuTc);
  MAP_IN_OUT("gnss_imu_camera_stc", EstimatorType::GnssImuCameraStc);
  MAP_IN_OUT("rtk_imu_camera_tc", EstimatorType::RtkImuCameraTc);
  LOG_INVALId;
}

template <>
void convert<std::string, SolutionRole>
  (const std::string& in, SolutionRole& out)
{
  MAP_IN_OUT("position", SolutionRole::Position);
  MAP_IN_OUT("attitude", SolutionRole::Attitude);
  MAP_IN_OUT("velocity", SolutionRole::Velocity);
  MAP_IN_OUT("pose", SolutionRole::Pose);
  MAP_IN_OUT("pose_and_velocity", SolutionRole::PoseAndVelocity);
  MAP_IN_OUT("position_and_velocity", SolutionRole::PositionAndVelocity);
  LOG_INVALId;
}

template <>
void convert<std::string, DetectorType>
  (const std::string& in, DetectorType& out)
{
  MAP_IN_OUT("fast", DetectorType::kFast);
  MAP_IN_OUT("grad", DetectorType::kGrad);
  MAP_IN_OUT("fast_and_grad", DetectorType::kFastGrad);
  MAP_IN_OUT("shitomasi", DetectorType::kShiTomasi);
  MAP_IN_OUT("shitomasi_and_grad", DetectorType::kShiTomasiGrad);
  MAP_IN_OUT("grid_grad", DetectorType::kGridGrad);
  MAP_IN_OUT("all", DetectorType::kAll);
  MAP_IN_OUT("mumford_grad", DetectorType::kGradHuangMumford);
  MAP_IN_OUT("canny", DetectorType::kCanny);
  MAP_IN_OUT("sobel", DetectorType::kSobel);
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
template <typename OptionType>
void loadOptions(YAML::Node& node, OptionType& options)
{
  LOG(FATAL) << "Loading " << typeid(options).name() << " not supported!";
}

template <>
void loadOptions<GnssCommonOptions>(
    YAML::Node& node, GnssCommonOptions& options)
{
  LOAD_COMMON(min_elevation);
  LOAD_COMMON(mw_slip_thres);
  LOAD_COMMON(gf_slip_thres);
  LOAD_COMMON(gf_sd_slip_thres);
  LOAD_COMMON(period);

  std::vector<std::string> system_excludes;
  if (option_tools::safeGet(node, "system_exclude", &system_excludes)) {
    options.system_exclude.clear();
    for (auto system_exclude : system_excludes) {
      options.system_exclude.push_back(system_exclude[0]);
    }
  }

  std::vector<std::string> satellite_excludes;
  if (option_tools::safeGet(node, "satellite_exclude", &satellite_excludes)) {
    options.satellite_exclude.clear();
    for (auto satellite_exclude : satellite_excludes) {
      options.satellite_exclude.push_back(satellite_exclude);
    }
  }

  std::vector<std::string> code_excludes;
  if (option_tools::safeGet(node, "code_exclude", &code_excludes)) {
    options.code_exclude.clear();
    for (auto code_exclude : code_excludes) {
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

  std::vector<std::string> system_excludes;
  if (option_tools::safeGet(node, "system_exclude", &system_excludes)) {
    options.system_exclude.clear();
    for (auto system_exclude : system_excludes) {
      options.system_exclude.push_back(system_exclude[0]);
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

  if (checkSubOption(node, "gnss_common")) {
    YAML::Node subnode = node["gnss_common"];
    loadOptions(subnode, options.common);
  }

  if (checkSubOption(node, "gnss_error_parameter")) {
    YAML::Node subnode = node["gnss_error_parameter"];
    loadOptions(subnode, options.error_parameter);
  }

  if (checkSubOption(node, "ambiguity_resolution")) {
    YAML::Node subnode = node["ambiguity_resolution"];
    loadOptions(subnode, options.ambiguity_resolution);
  }
}

template <>
void loadOptions<SppEstimatorOptions>(
    YAML::Node& node, SppEstimatorOptions& options)
{
  LOAD_COMMON(max_iteration);
  LOAD_COMMON(num_threads);
  LOAD_COMMON(verbose);

  if (checkSubOption(node, "gnss_common")) {
    YAML::Node subnode = node["gnss_common"];
    loadOptions(subnode, options.common);
  }

  if (checkSubOption(node, "gnss_error_parameter")) {
    YAML::Node subnode = node["gnss_error_parameter"];
    loadOptions(subnode, options.error_parameter);
  }
}

template <>
void loadOptions<DgnssEstimatorOptions>(
    YAML::Node& node, DgnssEstimatorOptions& options)
{
  LOAD_COMMON(max_iteration);
  LOAD_COMMON(num_threads);
  LOAD_COMMON(verbose);
  LOAD_COMMON(max_age);

  if (checkSubOption(node, "gnss_common")) {
    YAML::Node subnode = node["gnss_common"];
    loadOptions(subnode, options.common);
  }

  if (checkSubOption(node, "gnss_error_parameter")) {
    YAML::Node subnode = node["gnss_error_parameter"];
    loadOptions(subnode, options.error_parameter);
  }
}

template <>
void loadOptions<ImuParameters>(
    YAML::Node& node, ImuParameters& options)
{
  LOAD_COMMON(a_max);
  LOAD_COMMON(g_max);
  LOAD_COMMON(sigma_g_c);
  LOAD_COMMON(sigma_bg);
  LOAD_COMMON(sigma_a_c);
  LOAD_COMMON(sigma_ba);
  LOAD_COMMON(sigma_gw_c);
  LOAD_COMMON(sigma_aw_c);
  LOAD_COMMON(rate);
  LOAD_COMMON(delay_imu_cam);
}

template <>
void loadOptions<GnssImuInitializationOptions>(
    YAML::Node& node, GnssImuInitializationOptions& options)
{
  LOAD_COMMON(max_iteration);
  LOAD_COMMON(num_threads);
  LOAD_COMMON(verbose);
  LOAD_COMMON(time_window_length_zero_motion);
  LOAD_COMMON(window_length_optimize);
  LOAD_COMMON(gnss_extrinsic_initial_std);
  LOAD_COMMON(min_velocity);

  std::vector<double> gnss_extrinsic;
  if (option_tools::safeGet(node, "gnss_extrinsic", &gnss_extrinsic) && 
      gnss_extrinsic.size() == 3) {
    for (size_t i = 0; i < 3; i++) {
      options.gnss_extrinsic[i] = gnss_extrinsic[i];
    }
  }
  else {
    options.gnss_extrinsic.setZero();
    options.gnss_extrinsic_initial_std = 3.0;
    LOG(INFO) << "Unable to load gnss_extrinsic. Using default instead.";
  }
}

template <>
void loadOptions<GnssImuLcEstimatorOptions>(
    YAML::Node& node, GnssImuLcEstimatorOptions& options)
{
  LOAD_COMMON(max_iteration);
  LOAD_COMMON(num_threads);
  LOAD_COMMON(verbose);
  LOAD_COMMON(window_length);
  LOAD_COMMON(gnss_relative_extrinsic_std);

  if (checkSubOption(node, "imu_parameters")) {
    YAML::Node subnode = node["imu_parameters"];
    loadOptions(subnode, options.imu_parameters);
  }

  if (checkSubOption(node, "initialize")) {
    YAML::Node subnode = node["initialize"];
    loadOptions(subnode, options.initialize);
    options.initialize.gnss_relative_extrinsic_std = 
      options.gnss_relative_extrinsic_std;
    options.initialize.imu_parameters = options.imu_parameters;
  }
}

template <>
void loadOptions<RtkImuTcEstimatorOptions>(
    YAML::Node& node, RtkImuTcEstimatorOptions& options)
{
  LOAD_COMMON(max_iteration);
  LOAD_COMMON(num_threads);
  LOAD_COMMON(verbose);
  LOAD_COMMON(max_age);
  LOAD_COMMON(window_length);
  LOAD_COMMON(use_ambiguity_resolution)
  LOAD_COMMON(gnss_relative_extrinsic_std);

  if (checkSubOption(node, "gnss_common")) {
    YAML::Node subnode = node["gnss_common"];
    loadOptions(subnode, options.gnss_common);
  }

  if (checkSubOption(node, "gnss_error_parameter")) {
    YAML::Node subnode = node["gnss_error_parameter"];
    loadOptions(subnode, options.gnss_error_parameter);
  }

  if (checkSubOption(node, "ambiguity_resolution")) {
    YAML::Node subnode = node["ambiguity_resolution"];
    loadOptions(subnode, options.ambiguity_resolution);
  }

  if (checkSubOption(node, "imu_parameters")) {
    YAML::Node subnode = node["imu_parameters"];
    loadOptions(subnode, options.imu_parameters);
  }

  if (checkSubOption(node, "initialize")) {
    YAML::Node subnode = node["initialize"];
    loadOptions(subnode, options.initialize);
    options.initialize.gnss_relative_extrinsic_std = 
      options.gnss_relative_extrinsic_std;
    options.initialize.imu_parameters = options.imu_parameters;
  }
}

template <>
void loadOptions<DetectorOptions>(
    YAML::Node& node, DetectorOptions& options)
{
  LOAD_COMMON(cell_size);
  LOAD_COMMON(max_level);
  LOAD_COMMON(min_level);
  LOAD_COMMON(border);
  LOAD_COMMON(threshold_primary);
  LOAD_COMMON(sampling_level)
  LOAD_COMMON(level);
  LOAD_COMMON(sec_grid_fineness);
  LOAD_COMMON(threshold_shitomasi);

  std::string detector_type;
  if (option_tools::safeGet(node, "detector_type", &detector_type)) {
    delete_space(detector_type);
    convert(detector_type, options.detector_type);
  }
}

template <>
void loadOptions<FeatureHandlerOptions>(
    YAML::Node& node, FeatureHandlerOptions& options)
{
  LOAD_COMMON(max_n_kfs);
  LOAD_COMMON(max_features_per_frame);
  LOAD_COMMON(kfselect_numkfs_lower_thresh);
  LOAD_COMMON(kfselect_min_dist_metric);
  LOAD_COMMON(kfselect_min_angle);
  LOAD_COMMON(kfselect_min_disparity)
  LOAD_COMMON(max_pyramid_level);
  LOAD_COMMON(min_disparity_new_landmark);

  if (checkSubOption(node, "detector")) {
    YAML::Node subnode = node["detector"];
    loadOptions(subnode, options.detector);
  }

  if (checkSubOption(node, "camera_parameters")) {
    YAML::Node subnode = node["camera_parameters"];
    options.cameras = CameraBundle::loadFromYaml(subnode);
  }
}

}

}