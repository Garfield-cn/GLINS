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
enum class GnssRole {
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
extern std::vector<char> GnssSystems;

// One code type measurement
struct Observation {
  double wavelength;
  double pseudorange;
  double phaserange;
  double doppler; // in m/s
  double SNR; // Sigal-to-Noise Ratio
  uint8_t LLI; // Loss of Lock Indicator
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

  double ionosphere;  // In 1575.42 MHz frequency
  IonoType ionosphere_type;
  
  // TODO: GPS L5 IFCB and GLONASS IFB
  // Currently, in PPP, we donot use GLONASS code and GPS L5
  // in RTK, we donot use GLONASS code and disable its AR

  // Get satellite system
  inline char getSystem() const { return prn[0]; }
};

using Satellites = std::map<std::string, Satellite, std::less<std::string>, 
  Eigen::aligned_allocator<std::pair<const std::string, Satellite>>>;

// Index of observation 
struct GnssMeasurementIndex {
  GnssMeasurementIndex() {}
  GnssMeasurementIndex(std::string prn, int code_type) : 
    prn(prn), code_type(code_type) {}
  std::string prn;
  int code_type;
};

// Pairs
// single difference pair
struct GnssMeasurementSDIndexPair {
  GnssMeasurementSDIndexPair(
    GnssMeasurementIndex rov, GnssMeasurementIndex ref) : 
    rov(rov), ref(ref) { }
  GnssMeasurementIndex rov;
  GnssMeasurementIndex ref;
};
using GnssMeasurementSDIndexPairs = std::vector<GnssMeasurementSDIndexPair>;
// double difference pair
struct GnssMeasurementDDIndexPair {
  GnssMeasurementDDIndexPair(
    GnssMeasurementIndex rov, GnssMeasurementIndex ref, 
    GnssMeasurementIndex rov_base, GnssMeasurementIndex ref_base) : 
    rov(rov), ref(ref), rov_base(rov_base), ref_base(ref_base) { }
  GnssMeasurementIndex rov;
  GnssMeasurementIndex ref;
  GnssMeasurementIndex rov_base;
  GnssMeasurementIndex ref_base;
};
using GnssMeasurementDDIndexPairs = std::vector<GnssMeasurementDDIndexPair>;

// GNSS epoch data
struct GnssMeasurement {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GnssMeasurement() : id(++epoch_cnt_) {}

  double timestamp;
  GnssRole role;
  std::string tag;
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
  
  inline Satellite& getSat(GnssMeasurementIndex index) { 
    return getSat(index.prn);
  }

  inline Observation& getObs(GnssMeasurementIndex index) {
    Satellite& satellite = getSat(index);
    auto it = satellite.observations.find(index.code_type);
    CHECK(it != satellite.observations.end());
    return it->second;
  }

  inline const Satellite& getSat(std::string prn) const { 
    const auto it = satellites.find(prn);
    CHECK(it != satellites.end());
    return it->second; 
  }

  inline const Satellite& getSat(GnssMeasurementIndex index) const {
    return getSat(index.prn);
  }

  inline const Observation& getObs(GnssMeasurementIndex index) const {
    const Satellite& satellite = getSat(index);
    const auto it = satellite.observations.find(index.code_type);
    CHECK(it != satellite.observations.end());
    return it->second;
  }

  static int32_t epoch_cnt_;
};

using GnssMeasurements = std::vector<GnssMeasurement, 
  Eigen::aligned_allocator<GnssMeasurement>>;

// Observation type
enum class ObservationType {
  Pseudorange,
  Phaserange,
  Doppler
};

// Solution status
enum class GnssSolutionStatus {
  None, 
  Single, 
  DGNSS,
  Float,
  Fixed,
  DeadReckoning
};

// Solutions
struct GnssSolution {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double timestamp;
  int32_t id;  // bundle id for integration
  Eigen::Vector3d position; // in ECEF
  Eigen::Vector3d velocity;
  Eigen::Matrix<double, 6, 6> covariance;
  GnssSolutionStatus status;
  int num_satellites;
  int differential_age;
};

// GNSS common options
struct GnssCommonOptions {
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
  double min_elevation = 7.0;

  // Threshold for Melbourne-Wubbena (MW) cycle-slip detection (m)
  double mw_slip_thres = 0.5;

  // Threshold for Geometry-Free (GF) cycle-slip detection (m)
  double gf_slip_thres = 0.05;

  // Threshold for single differenced GF cycle-slip detection (m)
  double gf_sd_slip_thres = 0.05;

  // Data period
  double period = 1.0;
};

// GNSS error factors
struct GnssErrorParameter {
  // code noise = phase noise * ratio
  double code_to_phase_ratio = 100.0;

  // Error factor according to RTKLIB
  std::vector<double> phase_error_factor{0.005, 0.005, 0.0};

  // System error ratio
  std::map<char, double> system_error_ratio = 
    {{'G', 1.0}, {'R', 5.0}, {'C', 2.0}, {'E', 1.5}};

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
