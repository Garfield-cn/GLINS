/**
* @Function: Convert file from IE output format to NMEA format
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
  char ie_buf[1024];
	if (argc < 2) {
		return -1;
	} else if (argc == 2) {
		strcpy(ie_buf, argv[1]);
	}

  FILE *fp_ie = fopen(ie_buf, "r");

  char buf[1024];
  std::vector<NmeaEpoch> epochs;
  while (!(fgets(buf, 1024 * sizeof(char), fp_ie) == NULL))
  {
    if (!(buf[0] >= '1' && buf[0] <= '9')) continue;
    std::vector<std::string> strs;
    strs.push_back("");
    for (int i = 0; i < strlen(buf); i++) {
      if (buf[i] == ' ') {
        if (strs.back().size() > 0) strs.push_back("");
        continue;
      }
      if (buf[i] == '\r' || buf[i] == '\n') break;
      strs.back().push_back(buf[i]);
    }

    if (strs.size() == 0 || atoi(strs[0].data()) == 0) continue;

    int gpsweek, status;
    double gpstow, lla[3], vel[3], att[3], pos[3];
    gpsweek = atoi(strs[0].data());
    gpstow = atof(strs[1].data());
    lla[1] = atof(strs[3].data()) * D2R;
    lla[0] = atof(strs[4].data()) * D2R;
    lla[2] = atof(strs[5].data());
    att[2] = atof(strs[6].data()) * D2R;
    att[0] = atof(strs[7].data()) * D2R;
    att[1] = atof(strs[8].data()) * D2R;

    pos2ecef(lla, pos);
    att[2] = -att[2];

    sol_t sol;
    sol.time = gpst2time(gpsweek, gpstow);
    sol.stat = SOLQ_FIX;
    pos2ecef(lla, sol.rr);

    NmeaEpoch epoch;
    epoch.sol = sol;
    epoch.esa.time = sol.time;
    for (int i = 0; i < 3; i++) {
      epoch.esa.vel[i] = 0.0;
      epoch.esa.att[i] = att[i];
    }
    epochs.push_back(epoch);
  }

  char nmea_path[1034];
  sprintf(nmea_path, "%s.nmea", ie_buf);
  writeNmeaFile(epochs, nmea_path);

  fclose(fp_ie);

  return 0;
}