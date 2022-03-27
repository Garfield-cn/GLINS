/**
* @Function: GNSS common functions
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/gnss/gnss_common.h"

#include <glog/logging.h>

namespace gici {

namespace rtklib {

// Convert char system to int system
extern int sys2sys(char sys)
{
  switch (sys) {
  case 'G': return SYS_GPS;
  case 'R': return SYS_GLO;
  case 'E': return SYS_GAL;
  case 'C': return SYS_CMP;
  default: return SYS_NONE;
  }
}

// Convert int system to char system
extern char sys2sys(int sys)
{
  switch (sys) {
  case SYS_GPS: return 'G';
  case SYS_GLO: return 'R';
  case SYS_GAL: return 'E';
  case SYS_CMP: return 'C';
  default: return 0x00;
  }
}

// Convert PRN string to RTKLIB sat
extern int prn2sat(std::string prn)
{
  int system = sys2sys(prn[0]);
  int prnnum = atoi(prn.substr(1, 2).data());
  return satno(system, prnnum);
}

// Convert RTKLIB sat to PRN string
extern std::string sat2prn(int sat)
{
  int prnnum;
  char system = sys2sys(satsys(sat, &prnnum));
  char prnbuf[10];
  sprintf(prnbuf, "%c%02d", system, prnnum);
  return std::string(prnbuf);
}

// Convert gtime to double
extern double gtime2double(gtime_t time)
{
  return (static_cast<double>(time.time) + time.sec);
}

// Convert double to gtime
extern gtime_t double2gtime(double time)
{
  gtime_t gtime;
  gtime.time = floor(time);
  gtime.sec = time - floor(time);
  return gtime;
}

}

}