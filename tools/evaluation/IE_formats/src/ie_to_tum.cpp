/**
* @Function: Convert file from IE output format to TUM pose format
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "rtklib.h"

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>

const double ref_position[] = {31.0294769301 * D2R, 121.4207972420 * D2R, 19.579};

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
  char buf[1034];
  sprintf(buf, "%s.tum", ie_buf);
  FILE *fp_tum = fopen(buf, "w");
  
  fprintf(fp_tum, "# timestamp tx ty tz qx qy qz qw\r\n");

  while (!(fgets(buf, 1034 * sizeof(char), fp_ie) == NULL))
  {
    if (buf[0] == 'W') continue;
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

    Eigen::Quaterniond q = eulerAngleToQuaternion(Eigen::Map<Eigen::Vector3d>(att));
    Eigen::Vector3d p_enu = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_ecef = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_ref_ecef = Eigen::Vector3d::Zero();
    pos2ecef(lla, p_ecef.data());
    pos2ecef(ref_position, p_ref_ecef.data());
    Eigen::Vector3d dp = p_ecef - p_ref_ecef;
    ecef2enu(ref_position, dp.data(), p_enu.data());
    gtime_t gtime = gpst2time(gpsweek, gpstow);
    double time = (double)gtime.time + gtime.sec;

    fprintf(fp_tum, "%.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf\r\n", 
      time, p_enu(0), p_enu(1), p_enu(2), q.x(), q.y(), q.z(), q.w());
  }

  fclose(fp_ie);
  fclose(fp_tum);

  return 0;
}