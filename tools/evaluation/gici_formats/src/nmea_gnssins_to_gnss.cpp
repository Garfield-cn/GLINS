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

const Eigen::Vector3d t_SR_S(-0.029, 0.354, -0.042);

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
  char buf[1050];
  sprintf(buf, "%s.tognss.txt", nmea_buf);
  FILE *fp_tognss = fopen(buf, "w");

  sol_t sol = {0};
  esa_t esa = {0};
  int num_gga = 0;
  bool has_esa = false;
  Eigen::Vector3d ref_ecef = Eigen::Vector3d::Zero();
  while (!(fgets(buf, 1050 * sizeof(char), fp_nmea) == NULL))
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
      double time = (double)sol.time.time + sol.time.sec;
      if (has_esa) {
        if (timediff(esa.time, sol.time) == 0.0) {
          Eigen::Quaterniond q = eulerAngleToQuaternion(Eigen::Map<Eigen::Vector3d>(esa.att));
          Eigen::Vector3d p_enu = Eigen::Vector3d::Zero();
          Eigen::Vector3d p_ecef(sol.rr);
          if (ref_ecef == Eigen::Vector3d::Zero()) {
            ref_ecef = Eigen::Map<Eigen::Vector3d>(sol.rr);
          }
          double ref_position[3];
          ecef2pos(ref_ecef.data(), ref_position);
          Eigen::Vector3d dp = p_ecef - ref_ecef;
          ecef2enu(ref_position, dp.data(), p_enu.data());

          p_enu = p_enu.eval() + q * t_SR_S;

          enu2ecef(ref_position, p_enu.data(), dp.data());
          p_ecef = ref_ecef + dp;
          for (int i = 0; i < 3; i++) sol.rr[i] = p_ecef(i);

          char *p = buf;
          p += encodeGGA(&sol, p);
          p += encodeRMC(&sol, p);
          fprintf(fp_tognss, "%s", buf);
        }
      }
      else {
        std::cerr << "Error: Cannot find GNESA message!" << std::endl;
        return -1;
      }
    }
  }

  fclose(fp_nmea);
  fclose(fp_tognss);

  return 0;
}