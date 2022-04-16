/**
* @Function: GNSS types
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#pragma once

#include <iostream>
#include <vector>
#include <unordered_map>
#include <map>
#include <memory>
#include <Eigen/Core>
#include <glog/logging.h>

namespace gici {

// Role of formator
enum class GNSSRole {
  None,
  Rover,
  Reference,
  Ephemeris,
  Correction,
  Heading,
};

// Satellite ephemeris types
enum class SatEphType {
  None = 0, 
  Broadcast = 1,
  Precise = 2
};

// Ionosphere delay type
enum class IonoType {
  Broadcast,
  DualFrequency,
  Augmentation
};

// GNSS systems
extern std::vector<char> GNSSSystems;

// One code type measurement
struct Observation {
  double wavelength;
  double pseudorange;
  double phaserange;
  double doppler; // in m/s
  double SNR; // Sigal-to-Noise Ratio
  bool LLI; // Loss of Lock Indicator
  bool slip; // Cycle-slip flag
};

// One satellite data
struct Satellite {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	std::string prn;
	std::unordered_map<int, Observation> observations;
  SatEphType sat_type = SatEphType::None;
  Eigen::Vector3d sat_position;
  Eigen::Vector3d sat_velocity;
  double sat_clock;
  double sat_frequency;

  // Biases in meter: dbias = bias(pair.second) - bias(pair.first)
  std::map<std::pair<int, int>, double> DCBs;
  std::map<std::pair<int, int>, double> DPBs;

  double TGDs[6]; // group delay parameters in meter
                  // GPS/QZS:tgd[0]=TGD 
                  // GAL:tgd[0]=BGD_E1E5a,tgd[1]=BGD_E1E5b 
                  // BDS:tgd[0]=TGD_B1I ,tgd[1]=TGD_B2I/B2b,tgd[2]=TGD_B1Cp 
                  //     tgd[3]=TGD_B2ap,tgd[4]=ISC_B1Cd   ,tgd[5]=ISC_B2ad 
                  // GLO:tgd[0]=delay between L1 and L2 (s)

  double ionosphere;  // In 1575.42 MHz frequency
  IonoType ionosphere_type;
  
  // TODO: GPS L5 IFCB and GLONASS IFB
  // Currently, in PPP, we donot use GLONASS code and GPS L5
  // in RTK, we donot use GLONASS code and disable its AR

  // Get satellite system
  inline char getSystem(void) const { return prn[0]; }
};

using Satellites = std::map<std::string, Satellite, std::less<std::string>, 
  Eigen::aligned_allocator<std::pair<const std::string, Satellite>>>;

// Index of observation 
struct GNSSMeasurementIndex {
  GNSSMeasurementIndex(std::string prn, int code_type) : 
    prn(prn), code_type(code_type) {}
  std::string prn;
  int code_type;
};

// Single difference pair
using GNSSMeasurementIndexPairs = 
  std::vector<std::pair<GNSSMeasurementIndex, GNSSMeasurementIndex>>;

// GNSS epoch data
struct GNSSMeasurement {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GNSSMeasurement() : id(++epoch_cnt_) {}

  double timestamp;
  GNSSRole role;
  std::string mount_id;
  int32_t id;  // ID for bundle adjustment
  Satellites satellites;
  Eigen::Vector3d position;  // for reference station
  Eigen::VectorXd ionosphere_parameters;  // GPS broadcast ionosphere parameters
  double troposphere_wet;  // Wet ZTD from augmentation

  inline Satellite& getSat(std::string prn) { 
    auto it = satellites.find(prn);
    CHECK(it != satellites.end());
    return it->second; 
  }
  
  inline Satellite& getSat(GNSSMeasurementIndex index) { 
    return getSat(index.prn);
  }

  inline Observation& getObs(GNSSMeasurementIndex index) {
    Satellite& satellite = getSat(index);
    auto it = satellite.observations.find(index.code_type);
    CHECK(it != satellite.observations.end());
    return it->second;
  }

  static int32_t epoch_cnt_;
};

using GNSSMeasurements = std::vector<GNSSMeasurement, 
  Eigen::aligned_allocator<GNSSMeasurement>>;

// Observation type
enum class GNSSObservationType {
  Pseudorange,
  Phaserange,
  Doppler
};

// GNSS common options
struct GNSSCommonOptions {
  // Usage of satellite systems
  // In default, we use all systems
  std::vector<char> system_exclude;

  // Usage of specific satellite
  // In default, we use all satellites
  std::vector<std::string> satellite_exclude;

  // Usage of code types
  // In default, we use all code types
  std::vector<std::pair<char, int>> code_exclude;

  // Minimum elevation angle (deg)
  double min_elevation = 0.0;

  // Threshold for Melbourne-Wubbena (MW) cycle-slip detection (m)
  double mw_slip_thres = 0.5;

  // Threshold for Geometry-Free (GF) cycle-slip detection (m)
  double gf_slip_thres = 0.05;

  // Threshold for single differenced GF cycle-slip detection (m)
  double gf_sd_slip_thres = 0.05;
};

// GNSS error factors
struct GNSSErrorParameter {
  // code noise = phase noise * ratio
  double code_to_phase_ratio = 100.0;

  // Error factor according to RTKLIB
  double phase_error_factor[3] = {0.003, 0.003, 0.0};

  // Ionosphere model error factor
  double ionosphere_broadcast_factor = 0.5;

  // Dual-frequency ionosphere error
  double ionosphere_dual_frequency = 0.2;

  // Augmentation ionosphere error
  double ionosphere_augment = 0.03;

  // Troposphere model error factor
  double troposphere_model_factor = 0.2;

  // Augmentation troposphere error
  double troposphere_augment = 0.01;

  // Doppler frequency error
  double doppler_frequency = 0.2;

  // Broadcast ephemeris error
  double ephemeris_broadcast = 3.0;

  // Precise ephemeris error
  double ephemeris_precise = 0.1;
};

}
