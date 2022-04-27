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
  MAP_IN_OUT("output", StreamIOType::Ouput);
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
void convert<std::string, IMURole>
  (const std::string& in, IMURole& out)
{
  MAP_IN_OUT("major", IMURole::Major);
  MAP_IN_OUT("minor", IMURole::Minor);
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

}

}