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

namespace gici {

// Formator types
enum class FormatorType {
  RTCM2, 
  RTCM3,
  GNSSRaw, 
  ImageV4L2,
  ImagePack,  
  IMUPack,
  OptionPack, 
};

// GNSS data types
enum class GNSSDataType {
  None = 0,
  Ephemeris = 2,
  Observation = 1,
  AntePos = 5,  // Antenna position
  IonPara = 9,  // Ionosphere parameters
  SSR = 10
};

// Data formats
class DataFormat {
public:
  using Ptr = std::shared_ptr<DataFormat>;

  DataFormat(FormatorType type);
  DataFormat(FormatorType type, int _width, int _height);
  ~DataFormat();

  // GNSS data format
  struct GNSS {
    using Ptr = std::shared_ptr<GNSS>;
    void init(void);
    void free(void);

    std::vector<GNSSDataType> types;
    obs_t *observation;
    nav_t *ephemeris;
    sta_t *antenna;
  };

  // Image data format
  struct Image {
    using Ptr = std::shared_ptr<Image>;
    void init(int _width, int _height);
    void free(void);

    double time;
    int width;
    int height;
    uint8_t *image;
  };

  // IMU data format
  struct IMU {
    using Ptr = std::shared_ptr<IMU>;
    double time;
    double acceleration[3];
    double angular_velocity[3];
  };

  // Option data format
  struct Option {
    using Ptr = std::shared_ptr<Option>;

  };

  // Parameters
  GNSS::Ptr gnss;
  Image::Ptr image;
  IMU::Ptr imu;
  Option::Ptr option;
};

// Formats of FormatorType::GNSS_Raw
enum class GNSSRawFormats {
  Ublox = STRFMT_UBX,
  Septentrio = STRFMT_SEPT
};

// Max number of output data buffers for decoders
struct MaxDataSize {
  static const int RTCM2 = 30;
  static const int RTCM3 = RTCM2;
  static const int GNSSRaw = RTCM2;
  static const int ImagePack = 2;
  static const int IMUPack = 500;
};

// Tools for RTKLIB types
namespace gnss_common {

// Update observation data
extern void updateObservation(
  obs_t *obs, DataFormat::GNSS::Ptr& gnss_data);

// Update ephemeris
extern void updateEphemeris(
  nav_t *nav, int sat, DataFormat::GNSS::Ptr& gnss_data);

// Update ion/utc parameters
extern void updateIonAndUTC(
  nav_t *nav, DataFormat::GNSS::Ptr& gnss_data);

// Update antenna position
extern void updateAntennaPosition(
  sta_t *sta, DataFormat::GNSS::Ptr& gnss_data);

// Update ssr corrections
extern void updateSSR(
  ssr_t *ssr, DataFormat::GNSS::Ptr& gnss_data);

// Select data from GNSS stream
// Note that data except for observation are 
// putted in the first place of the vector
extern void updateStreamData(int ret, obs_t *obs, nav_t *nav, 
  sta_t *sta, ssr_t *ssr, int iobs, int sat, 
  std::vector<DataFormat::GNSS::Ptr>& gnss_data);

}

// Base class
class FormatorBase {
public:
  using Ptr = std::shared_ptr<FormatorBase>;

  FormatorBase() { }
  ~FormatorBase() { }

  // Decode stream to data
  virtual int decode(const uint8_t *buf, int size, 
    std::vector<DataFormat::Ptr>& data) = 0;

  // Encode data to stream
  virtual int encode(
    const DataFormat::Ptr& data, uint8_t *buf) = 0;

  // Get formator type
  FormatorType getType(void) { return type_; }

  // Get data handle
  std::vector<DataFormat::Ptr>& getDataHandle(void) { return data_; }

protected:
  std::vector<DataFormat::Ptr> data_;
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
    std::vector<DataFormat::Ptr>& data) override;

  // Encode data to stream
  int encode(const DataFormat::Ptr& data, uint8_t *buf) override;

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
    std::vector<DataFormat::Ptr>& data) override;

  // Encode data to stream
  int encode(const DataFormat::Ptr& data, uint8_t *buf) override;

protected:
  rtcm_t rtcm_;
};

// GNSS raw
class GNSSRawFormator : public FormatorBase {
public:
  struct Config {
    double start_time; 
    std::string sub_type;
  };

  GNSSRawFormator(Config& config);
  GNSSRawFormator(YAML::Node& node);
  ~GNSSRawFormator();

  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<DataFormat::Ptr>& data) override;

  // Encode data to stream
  int encode(const DataFormat::Ptr& data, uint8_t *buf) override;

protected:
  raw_t raw_;
  GNSSRawFormats format_;
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
    std::vector<DataFormat::Ptr>& data) override;

  // Encode data to stream
  int encode(const DataFormat::Ptr& data, uint8_t *buf) override;

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
    std::vector<DataFormat::Ptr>& data) override;

  // Encode data to stream
  int encode(const DataFormat::Ptr& data, uint8_t *buf) override;

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
    std::vector<DataFormat::Ptr>& data) override;

  // Encode data to stream
  int encode(const DataFormat::Ptr& data, uint8_t *buf) override;

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
    std::vector<DataFormat::Ptr>& data) override;

  // Encode data to stream
  int encode(const DataFormat::Ptr& data, uint8_t *buf) override;

protected:

};

// Get formator handle from configure
#define MAKE_FORMATOR(Formator) \
inline FormatorBase::Ptr makeFormator( \
  Formator::Config& config) { \
   return std::make_shared<Formator>(config); \
}
MAKE_FORMATOR(RTCM2Formator);
MAKE_FORMATOR(RTCM3Formator);
MAKE_FORMATOR(GNSSRawFormator);
MAKE_FORMATOR(ImageV4L2Formator);
MAKE_FORMATOR(ImagePackFormator);
MAKE_FORMATOR(IMUPackFormator);
MAKE_FORMATOR(OptionFormator);

// Get formator handle from yaml
FormatorBase::Ptr makeFormator(YAML::Node& node);


}
