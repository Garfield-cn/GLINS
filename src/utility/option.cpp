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
#include "gici/inertial/imu_types.h"
#include "gici/visual/image_types.h"

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
  MAP_IN_OUT("gnss-raw", FormatorType::GNSSRaw);
  MAP_IN_OUT("image-v4l2", FormatorType::ImageV4L2);
  MAP_IN_OUT("image-pack", FormatorType::ImagePack);
  MAP_IN_OUT("imu-pack", FormatorType::IMUPack);
  MAP_IN_OUT("option", FormatorType::OptionPack);
  LOG_INVALId;
}

template <>
void convert<std::string, GNSSRawFormats>
  (const std::string& in, GNSSRawFormats& out)
{
  MAP_IN_OUT("ublox", GNSSRawFormats::Ublox);
  MAP_IN_OUT("septentrio", GNSSRawFormats::Septentrio);
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
void convert<std::string, GNSS::Role>
  (const std::string& in, GNSS::Role& out)
{
  MAP_IN_OUT("rover", GNSS::Role::Rover);
  MAP_IN_OUT("reference", GNSS::Role::Reference);
  MAP_IN_OUT("ephemeris", GNSS::Role::Ephemeris);
  MAP_IN_OUT("correction", GNSS::Role::Correction);
  MAP_IN_OUT("heading", GNSS::Role::Heading);
  LOG_INVALId;
}

template <>
void convert<std::string, INS::Role>
  (const std::string& in, INS::Role& out)
{
  MAP_IN_OUT("major", INS::Role::Major);
  MAP_IN_OUT("minor", INS::Role::Minor);
  LOG_INVALId;
}

template <>
void convert<std::string, camera::Role>
  (const std::string& in, camera::Role& out)
{
  MAP_IN_OUT("mono", camera::Role::Mono);
  MAP_IN_OUT("stereo-major", camera::Role::StereoMajor);
  MAP_IN_OUT("stereo-minor", camera::Role::StereoMinor);
  MAP_IN_OUT("array", camera::Role::Array);
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