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
  char nmea_buf[1024];
	if (argc < 2) {
		return -1;
	} else if (argc == 2) {
		strcpy(nmea_buf, argv[1]);
	}

  FILE *fp_nmea = fopen(nmea_buf, "r");
  char buf[1034];
  sprintf(buf, "%s.tum", nmea_buf);
  FILE *fp_tum = fopen(buf, "w");
  
  fprintf(fp_tum, "# timestamp tx ty tz qx qy qz qw\r\n");

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
      Eigen::Quaterniond q; q.setIdentity();
      Eigen::Vector3d p_enu = Eigen::Vector3d::Zero();
      Eigen::Vector3d p_ecef = Eigen::Vector3d::Zero();
      Eigen::Vector3d p_ref_ecef = Eigen::Vector3d::Zero();
      pos2ecef(ref_position, p_ref_ecef.data());
      Eigen::Vector3d dp = Eigen::Map<Eigen::Vector3d>(sol.rr) - p_ref_ecef;
      ecef2enu(ref_position, dp.data(), p_enu.data());
      double time = (double)sol.time.time + sol.time.sec;
      if (has_esa) {
        if (timediff(esa.time, sol.time) == 0.0) {
          q = eulerAngleToQuaternion(Eigen::Map<Eigen::Vector3d>(esa.att));
          fprintf(fp_tum, "%.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf\r\n", 
            time, p_enu(0), p_enu(1), p_enu(2), q.x(), q.y(), q.z(), q.w());
        }
      }
      else {
        fprintf(fp_tum, "%.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf\r\n", 
          time, p_enu(0), p_enu(1), p_enu(2), q.x(), q.y(), q.z(), q.w());
      }
    }
  }

  fclose(fp_nmea);
  fclose(fp_tum);

  return 0;
}