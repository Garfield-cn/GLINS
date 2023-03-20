/**
* @Function: Convert pose from IMU coordinate to GNSS antenna coordinate
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "rtklib.h"

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>

const Eigen::Vector3d t_SR_S(0.096, 0.611, 0.061);

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
  char buf[1050];
  sprintf(buf, "%s.tognss.txt", ie_buf);
  FILE *fp_tognss = fopen(buf, "w");

  Eigen::Vector3d ref_position = Eigen::Vector3d::Zero();

  while (!(fgets(buf, 1050 * sizeof(char), fp_ie) == NULL))
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
    if (ref_position == Eigen::Vector3d::Zero()) {
      ref_position = Eigen::Map<Eigen::Vector3d>(lla);
    }
    pos2ecef(ref_position.data(), p_ref_ecef.data());
    Eigen::Vector3d dp = p_ecef - p_ref_ecef;
    ecef2enu(ref_position.data(), dp.data(), p_enu.data());

    p_enu = p_enu + q * t_SR_S;

    enu2ecef(ref_position.data(), p_enu.data(), dp.data());
    p_ecef = p_ref_ecef + dp;
    ecef2pos(p_ecef.data(), lla);

    fprintf(fp_tognss, "%4d.00000 %9.2lf %14.10lf %14.10lf %12.3lf %9.3lf %9.3lf %9.3lf %14.9lf %14.9lf %14.9lf %1d\r\n", 
      gpsweek, gpstow, lla[0]*R2D, lla[1]*R2D, lla[2], vel[0], vel[1], vel[2],
      0.0, 0.0, 0.0, status);
  }

  fclose(fp_ie);
  fclose(fp_tognss);

  return 0;
}