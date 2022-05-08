/**
* @Function: Decoding and encoding stream
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#pragma once

#include <iostream>
#include <memory>
#include <glog/logging.h>

#include "gici/stream/format_image.h"
#include "gici/stream/format_imu.h"
#include "gici/utility/option.h"
#include "gici/utility/rtklib_safe.h"
#include "gici/estimate/estimator_types.h"

namespace gici {

// Formator types
enum class FormatorType {
  RTCM2, 
  RTCM3,
  GnssRaw, 
  ImageV4L2,
  ImagePack,  
  IMUPack,
  OptionPack, 
  NMEA
};

// GNSS data types
enum class GnssDataType {
  None = 0,
  Ephemeris = 2,
  Observation = 1,
  AntePos = 5,  // Antenna position
  IonPara = 9,  // Ionosphere parameters
  SSR = 10
};

// Data 
class DataCluster {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DataCluster() {}
  DataCluster(FormatorType type);
  DataCluster(FormatorType type, int _width, int _height);
  ~DataCluster();

  // GNSS data format
  struct GNSS {
    void init();
    void free();

    std::vector<GnssDataType> types;
    obs_t *observation;
    nav_t *ephemeris;
    sta_t *antenna;
  };

  // Image data format
  struct Image {
    void init(int _width, int _height);
    void free();

    double time;
    int width;
    int height;
    uint8_t *image;
  };

  // IMU data format
  struct IMU {
    double time;
    double acceleration[3];
    double angular_velocity[3];
  };

  // Option data format
  struct Option {

  };

  // Parameters
  std::shared_ptr<GNSS> gnss;
  std::shared_ptr<Image> image;
  std::shared_ptr<IMU> imu;
  std::shared_ptr<Option> option;
  std::shared_ptr<Solution> solution;
};

// Formats of FormatorType::GNSS_Raw
enum class GnssRawFormats {
  Ublox = STRFMT_UBX,
  Septentrio = STRFMT_SEPT
};

// Max number of output data buffers for decoders
struct MaxDataSize {
  static const int RTCM2 = 30;
  static const int RTCM3 = RTCM2;
  static const int GnssRaw = RTCM2;
  static const int ImagePack = 2;
  static const int IMUPack = 500;
};

// Tools for RTKLIB types
namespace gnss_common {

// Update observation data
extern void updateObservation(
  obs_t *obs, std::shared_ptr<DataCluster::GNSS>& gnss_data);

// Update ephemeris
extern void updateEphemeris(
  nav_t *nav, int sat, std::shared_ptr<DataCluster::GNSS>& gnss_data);

// Update ion/utc parameters
extern void updateIonAndUTC(
  nav_t *nav, std::shared_ptr<DataCluster::GNSS>& gnss_data);

// Update antenna position
extern void updateAntennaPosition(
  sta_t *sta, std::shared_ptr<DataCluster::GNSS>& gnss_data);

// Update ssr corrections
extern void updateSSR(
  ssr_t *ssr, std::shared_ptr<DataCluster::GNSS>& gnss_data);

// Select data from GNSS stream
// Note that data except for observation are 
// putted in the first place of the vector
extern void updateStreamData(int ret, obs_t *obs, nav_t *nav, 
  sta_t *sta, ssr_t *ssr, int iobs, int sat, 
  std::vector<std::shared_ptr<DataCluster::GNSS>>& gnss_data);

}

// Base class
class FormatorBase {
public:
  FormatorBase() { }
  ~FormatorBase() { }

  // Decode stream to data
  virtual int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) = 0;

  // Encode data to stream
  virtual int encode(
    const std::shared_ptr<DataCluster>& data, uint8_t *buf) = 0;

  // Get formator type
  FormatorType getType() { return type_; }

  // Get data handle
  std::vector<std::shared_ptr<DataCluster>>& getDataHandle() { return data_; }

protected:
  std::vector<std::shared_ptr<DataCluster>> data_;
  FormatorType type_;
};

// RTCM 2
class RTCM2Formator : public FormatorBase {
public:
  struct Config {
    double start_time; 
  };

  RTCM2Formator(Config& config);
  RTCM2Formator(YAML::Node& node);
  ~RTCM2Formator();

  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;

  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

protected:
  rtcm_t rtcm_;
};

// RTCM 3
class RTCM3Formator : public FormatorBase {
public:
  struct Config {
    double start_time; 
  };

  RTCM3Formator(Config& config);
  RTCM3Formator(YAML::Node& node);
  ~RTCM3Formator();

  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;

  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

protected:
  rtcm_t rtcm_;
};

// GNSS raw
class GnssRawFormator : public FormatorBase {
public:
  struct Config {
    double start_time; 
    std::string sub_type;
  };

  GnssRawFormator(Config& config);
  GnssRawFormator(YAML::Node& node);
  ~GnssRawFormator();

  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;

  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

protected:
  raw_t raw_;
  GnssRawFormats format_;
};

// Image V4L2
class ImageV4L2Formator : public FormatorBase {
public:
  struct Config {
    int width;
    int height;
  };

  ImageV4L2Formator(Config& config);
  ImageV4L2Formator(YAML::Node& node);
  ~ImageV4L2Formator();

  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;

  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

protected:
  img_t image_;
};

// Image pack
class ImagePackFormator : public FormatorBase {
public:
  struct Config {
    int width;
    int height;
  };

  ImagePackFormator(Config& config);
  ImagePackFormator(YAML::Node& node);
  ~ImagePackFormator();

  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;

  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

protected:
  img_t image_;
};

// IMU pack
class IMUPackFormator : public FormatorBase {
public:
  struct Config {
    
  };

  IMUPackFormator(Config& config);
  IMUPackFormator(YAML::Node& node);
  ~IMUPackFormator();

  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;

  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

protected:
  imu_t imu_;
};

// Option
class OptionFormator : public FormatorBase {
public:
  struct Config {
    
  };

  OptionFormator(Config& config);
  OptionFormator(YAML::Node& node);
  ~OptionFormator();

  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;

  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

protected:

};

// NMEA (for solution)
class NmeaFormator : public FormatorBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct Config {
    bool use_gga = true;
    bool use_rmc = true;
    bool use_esa = true;
    std::string talker_id = "GN";
  };

  NmeaFormator(Config& config);
  NmeaFormator(YAML::Node& node);
  ~NmeaFormator();

  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;

  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

protected:
  // Encode GNGGA message
  int encodeGGA(const Solution& solution, uint8_t* buf);

  // Encode GNRMC message
  int encodeRMC(const Solution& solution, uint8_t* buf);

  // Encode GNESA (self-defined Extended Speed and Attitude) message
  // Format: $GNESA,tod,Ve,Vn,Vu,Ar,Ap,Ay*checksum
  int encodeESA(const Solution& solution, uint8_t* buf);

  // Convert Solution to sol_t
  void convertSolution(const Solution& solution, sol_t& sol);

  // Configure
  Config config_;
};

// Get formator handle from configure
#define MAKE_FORMATOR(Formator) \
inline std::shared_ptr<FormatorBase> makeFormator( \
  Formator::Config& config) { \
   return std::make_shared<Formator>(config); \
}
MAKE_FORMATOR(RTCM2Formator);
MAKE_FORMATOR(RTCM3Formator);
MAKE_FORMATOR(GnssRawFormator);
MAKE_FORMATOR(ImageV4L2Formator);
MAKE_FORMATOR(ImagePackFormator);
MAKE_FORMATOR(IMUPackFormator);
MAKE_FORMATOR(OptionFormator);
MAKE_FORMATOR(NmeaFormator);

// Get formator handle from yaml
std::shared_ptr<FormatorBase> makeFormator(YAML::Node& node);


}
