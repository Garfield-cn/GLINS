/**
* @Function: GNSS common functions
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#pragma once

#include <iostream>
#include <memory>
#include <map>
#include <Eigen/Core>

#include "gici/utility/rtklib_safe.h"
#include "gici/stream/formator.h"
#include "gici/gnss/gnss_types.h"
#include "gici/utility/common.h"

namespace gici {

namespace gnss_common {

// ----------------------------------------------------------
// Convert char system to int system
int systemConvert(char sys);

// Convert int system to char system
char systemConvert(int sys);

// Convert PRN string to RTKLIB sat
int prnToSat(std::string prn);

// Convert RTKLIB sat to PRN string
std::string satToPrn(int sat);

// Convert gtime to double
double gtimeToDouble(gtime_t time);

// Convert GPS time to UTC time
double gpsTimeToUtc(double gps_time);

// Convert double to gtime
gtime_t doubleToGtime(double time);

// Degree to Rad
inline double degreeToRad(double degree) {
  return degree * D2R;
}

// Rad to Degree
inline double radToDegree(double rad) {
  return rad * R2D;
}

// Get a phase ID
// Note that we use "phase" to represent phase channels, which is not one-to-one 
// correspondence to "wavelength" or "frequency", because the signal chanel like 
// GPS L2C and L2W has the same frequency, but are two different phase channels.
int getPhaseID(char system, int code, double wavelength);

// ----------------------------------------------------------
// Check whether the system is used
bool useSystem(GnssCommonOptions options, const char system);

// Check whether the satellite is used
bool useSatellite(GnssCommonOptions options, const std::string prn);

// Check whether the code type is used
bool useCode(GnssCommonOptions options, char system, const int code_type);

// Check elevation threshold
bool checkElevation(GnssCommonOptions options, 
  const GnssMeasurement& measurement, std::string prn);

// One phase corresponds to muitiple code type, so we need to delete
// duplicated phases
void deleteDuplicatePhases(GnssMeasurement& measurement);

// Check observation valid
bool checkObservationValid(const GnssMeasurement& measurement,
                           const GnssMeasurementIndex& index,
                           const ObservationType type, 
                           const GnssCommonOptions options = GnssCommonOptions());

// Form single difference pseudorange pair
GnssMeasurementSDIndexPairs formPseudorangeSDPair(
                            const GnssMeasurement& measurement_rov, 
                            const GnssMeasurement& measurement_ref,
                            const GnssCommonOptions options = GnssCommonOptions());

// Form single difference phaserange pair
GnssMeasurementSDIndexPairs formPhaserangeSDPair(
                            const GnssMeasurement& measurement_rov, 
                            const GnssMeasurement& measurement_ref,
                            const GnssCommonOptions options = GnssCommonOptions());

// Form double difference pseudorange pair
// we use satellite with highest elevation angle as base satellite
GnssMeasurementDDIndexPairs formPseudorangeDDPair(
                            const GnssMeasurement& measurement_rov, 
                            const GnssMeasurement& measurement_ref,
                            const GnssCommonOptions options = GnssCommonOptions());

// Form double difference phaserange pair
GnssMeasurementDDIndexPairs formPhaserangeDDPair(
                            const GnssMeasurement& measurement_rov, 
                            const GnssMeasurement& measurement_ref,
                            const GnssCommonOptions options = GnssCommonOptions());

// ----------------------------------------------------------
// Saastamoinen troposphere delay model
double troposphereSaastamoinen(double time, 
  const Eigen::Vector3d& ecef, double elevation, double humi = 0.0);

// GMF troposphere delay model
void troposphereGMF(double time, 
  const Eigen::Vector3d& ecef, double elevation, 
  double* gmfh, double* gmfw);

// Broadcast ionosphere model
double ionosphereBroadcast(double time, const Eigen::Vector3d& ecef, 
  double azimuth, double elevation, double wavelength, 
  const Eigen::VectorXd& parameters = Eigen::VectorXd::Zero(8));

// Dual-frequenct ionosphere model
// output ionosphere is in meter at obs_1 frequency
double ionosphereDualFrequency(
  const Observation& obs_1, const Observation& obs_2);

// Convert ionosphere delay to 1575.42 MHz
inline double ionosphereConvertToBase(
  double ionosphere, double wavelength) {
  return ionosphere * square(CLIGHT / FREQ1 / wavelength);
}

// Convert ionosphere delay from 1575.42 MHz to given wavelength
inline double ionosphereConvertFromBase(
  double ionosphere, double wavelength) {
  return ionosphere * square(wavelength / (CLIGHT / FREQ1));
}

// Receiver to satellite distance considering the earth rotation effect
double satelliteToReceiverDistance(
  const Eigen::Vector3d satellite_ecef, const Eigen::Vector3d receiver_ecef);

// Satellite elevation angle
double satelliteElevation(
  const Eigen::Vector3d satellite_ecef, const Eigen::Vector3d receiver_ecef);

// Satellite azimuth angle
double satelliteAzimuth(
  const Eigen::Vector3d satellite_ecef, const Eigen::Vector3d receiver_ecef);

// Melbourne-Wubbena (MW) combination
double combinationMW(const Observation& observation_1,
                     const Observation& observation_2);

// Geometry Free (GF) combination
double combinationGF(const Observation& observation_1,
                     const Observation& observation_2);

// Solve integer ambiguity by LAMBDA
bool solveAmbiguityLambda(const Eigen::VectorXd& float_ambiguities,
                          const Eigen::MatrixXd& covariance, 
                          const double ratio_threshold, 
                          Eigen::VectorXd& fixed_ambiguities);

}

}