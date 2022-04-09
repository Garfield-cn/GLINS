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
#include <rtklib.h>

#include "gici/stream/formator.h"

namespace gici {

namespace rtklib {

// Convert char system to int system
extern int sys2sys(char sys);

// Convert int system to char system
extern char sys2sys(int sys);

// Convert PRN string to RTKLIB sat
extern int prn2sat(std::string prn);

// Convert RTKLIB sat to PRN string
extern std::string sat2prn(int sat);

// Convert gtime to double
extern double gtime2double(gtime_t time);

// Convert double to gtime
extern gtime_t double2gtime(double time);

}

}