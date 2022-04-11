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
#include <Eigen/Core>

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

// Troposhere type
enum class TropoType {
  Model,
  Augmentation
};

// One code type measurement
struct Observation {
  double wavelength;
  double pesudorange;
  double phaserange;
  double doppler; // in m/s
  double SNR; // Sigal-to-Noise Ratio
  bool LLI; // Loss of Lock Indicator
  bool slip; // Cycle-slip flag
  double code_bias;
  double phase_bias;
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
  double ionosphere;  // In 1575.42 MHz frequency
  IonoType ionosphere_type;
  
  // TODO: GPS L5 IFCB and GLONASS IFB
  // Currently, in PPP, we donot use GLONASS code and GPS L5
  // in RTK, we donot use GLONASS code and disable its AR

  // Get satellite system
  inline char getSystem(void) const { return prn[0]; }
};

using Satellites = std::vector<Satellite, Eigen::aligned_allocator<Satellite>>;

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
  double troposphere;  // from augmentation
  TropoType troposphere_type;

  static int32_t epoch_cnt_;
};

using GNSSMeasurements = std::vector<GNSSMeasurement, 
  Eigen::aligned_allocator<GNSSMeasurement>>;

// Index of observation 
struct GNSSMeasurementIndex {
  GNSSMeasurementIndex(size_t index, int code) : 
    satellite_index(index), code_type(code) {}
  size_t satellite_index;
  int code_type;
};

// Single difference pair
using GNSSMeasurementIndexPairs = 
  std::vector<std::pair<GNSSMeasurementIndex, GNSSMeasurementIndex>>;

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
  std::vector<int> code_exclude;

  // Minimum elevation angle (deg)
  double min_elevation = 7.0;
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
