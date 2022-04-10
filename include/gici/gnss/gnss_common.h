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

namespace gici {

namespace gnss_common {

// ----------------------------------------------------------
// Convert char system to int system
int sys2sys(char sys);

// Convert int system to char system
char sys2sys(int sys);

// Convert PRN string to RTKLIB sat
int prn2sat(std::string prn);

// Convert RTKLIB sat to PRN string
std::string sat2prn(int sat);

// Convert gtime to double
double gtime2double(gtime_t time);

// Convert double to gtime
gtime_t double2gtime(double time);

// ----------------------------------------------------------
// Saastamoinen troposphere delay model
double troposphereSaastamoinen(double time, 
  const Eigen::Vector3d& ecef, double elevation, double humi = 0.0);

// GMF troposphere delay model
void troposphereGMF(double time, 
  const Eigen::Vector3d& ecef, double elevation, double* gmfh, double* gmfw);

// Broadcast ionosphere model
double ionosphereBroadcast(double time,
  const Eigen::Vector3d& ecef, double azimuth, double elevation, 
  const Eigen::VectorXd& parameters = Eigen::VectorXd::Zero(8));

// Receiver to satellite distance considering the earth rotation effect
double satelliteToReceiverDistance(
  const Eigen::Vector3d satellite_ecef, const Eigen::Vector3d receiver_ecef);

// Satellite elevation angle
double satelliteElevation(
  const Eigen::Vector3d satellite_ecef, const Eigen::Vector3d receiver_ecef);

// Satellite azimuth angle
double satelliteAzimuth(
  const Eigen::Vector3d satellite_ecef, const Eigen::Vector3d receiver_ecef);

}

}