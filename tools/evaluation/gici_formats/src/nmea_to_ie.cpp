/**
* @Function: Convert file from IE output format to TUM pose format
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "rtklib.h"
#include "nmea_formator.h"

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>

// Convert euler angle to quarternion
Eigen::Quaternion<double> eulerAngleToQuaternion(const Eigen::Vector3d rpy)
{
  // Abbreviations for the various angular functions
  double roll = rpy(0);
  double pitch = rpy(1);
  double yaw = rpy(2);
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  Eigen::Quaternion<double> q;
  q.w() = cr * cp * cy + sr * sp * sy;
  q.x() = sr * cp * cy - cr * sp * sy;
  q.y() = cr * sp * cy + sr * cp * sy;
  q.z() = cr * cp * sy - sr * sp * cy;

  return q;
}

int main(int argc, char ** argv)
{
  char nmea_buf[1024];
	if (argc < 2) {
		return -1;
	} else if (argc == 2) {
		strcpy(nmea_buf, argv[1]);
	}

  FILE *fp_nmea = fopen(nmea_buf, "r");
  char buf[1034];
  sprintf(buf, "%s.ie", nmea_buf);
  FILE *fp_ie = fopen(buf, "w");

  sol_t sol = {0};
  esa_t esa = {0};
  int num_gga = 0;
  bool has_esa = false;
  while (!(fgets(buf, 1034 * sizeof(char), fp_nmea) == NULL))
  {
    std::vector<std::string> strs;
    strs.push_back("");
    for (int i = 0; i < strlen(buf); i++) {
      if (buf[i] == ',') {
        if (strs.back().size() > 0) strs.push_back("");
        continue;
      }
      if (buf[i] == '\r' || buf[i] == '\n') break;
      strs.back().push_back(buf[i]);
    }
    
    if (strs[0].substr(3, 3) == "GGA") {
      decodeGGA(buf, &sol);
      num_gga++;
    }
    else if (strs[0].substr(3, 3) == "RMC") {
      decodeRMC(buf, &sol);
    }
    else if (strs[0].substr(3, 3) == "ESA") {
      decodeESA(buf, &sol, &esa);
      has_esa = true;
    }

    if (num_gga > 1)
    {
      int gpsweek;
      double gpstow, lla[3];
      gpstow = time2gpst(sol.time, &gpsweek);
      ecef2pos(sol.rr, lla);
      double time = (double)sol.time.time + sol.time.sec;
      Eigen::Vector3d att; att.setZero();
      Eigen::Vector3d vel; vel.setZero();
      if (has_esa) {
        if (timediff(esa.time, sol.time) == 0.0) {
          att = Eigen::Map<Eigen::Vector3d>(esa.att);
          vel = Eigen::Map<Eigen::Vector3d>(esa.vel);
          fprintf(fp_ie, "%4d.00000 %9.2lf %14.10lf %14.10lf %12.3lf %9.3lf %9.3lf %9.3lf %14.9lf %14.9lf %14.9lf %1d\r\n", 
            gpsweek, gpstow, lla[0]*R2D, lla[1]*R2D, lla[2], vel[0], vel[1], vel[2],
            -att[2], att[1], att[0], 0);
        }
      }
      else {
        fprintf(fp_ie, "%4d.00000 %9.2lf %14.10lf %14.10lf %12.3lf %9.3lf %9.3lf %9.3lf %14.9lf %14.9lf %14.9lf %1d\r\n", 
          gpsweek, gpstow, lla[0]*R2D, lla[1]*R2D, lla[2], vel[0], vel[1], vel[2],
          -att[2], att[1], att[0], 0);
      }
    }
  }

  fclose(fp_nmea);
  fclose(fp_ie);

  return 0;
}