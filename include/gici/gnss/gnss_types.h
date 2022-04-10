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
  Broadcast,
  Precise
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
  SatEphType sat_type;
  Eigen::Vector3d sat_position;
  Eigen::Vector3d sat_velocity;
  double sat_clock;
  double sat_frequency;
  double ionosphere;  // from augmentation
  
  // TODO: GPS L5 IFCB and GLONASS IFB
  // Currently, in PPP, we donot use GLONASS code and GPS L5
  // in RTK, we donot use GLONASS code and disable its AR

  // Get satellite system
  char getSystem(void) { return prn[0]; }

  // Compute satellite to receiver distance
  double getRho(const Eigen::Vector3d& xyz);

  // Compute satellite elevation angle
  double getElevation(const Eigen::Vector3d& xyz);

  // Compute satellite azimuth angle
  double getAzimuth(const Eigen::Vector3d& xyz);
};

using Satellites = std::vector<Satellite, Eigen::aligned_allocator<Satellite>>;

// GNSS epoch data
struct GNSSMeasurement {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double timestamp;
  GNSSRole role;
  std::string mount_id;
  Satellites satellites;
  Eigen::Vector3d position;  // for reference station
  Eigen::VectorXd ionosphere_parameters;  // GPS broadcast ionosphere parameters
  double troposphere;  // from augmentation
};

using GNSSMeasurements = std::vector<GNSSMeasurement, 
  Eigen::aligned_allocator<GNSSMeasurement>>;

// GNSS error factors
struct GNSSErrorParameter {
  // code noise = phase noise * ratio
  double code_to_phase_ratio = 100.0;

  // Error factor according to RTKLIB
  double phase_error_factor[3] = {0.003, 0.003, 0.0};

  // Doppler frequency error
  double doppler_frequency = 0.2;
};

}
