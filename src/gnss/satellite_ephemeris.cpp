/**
* @Function: Satellite ephemeris
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/gnss/satellite_ephemeris.h"

#include <glog/logging.h>

#include "gici/gnss/gnss_common.h"  

namespace gici {

SatEphemeris::SatEphemeris()
{
  if (!(nav_.eph = (eph_t  *)malloc(sizeof(eph_t) * MAXSAT * 2)) ||
    !(nav_.geph = (geph_t *)malloc(sizeof(geph_t) * NSATGLO * 2)) ||
    !(nav_.seph = (seph_t *)malloc(sizeof(seph_t) * NSATSBS * 2)) ) {
    free(nav_.eph); free(nav_.geph); free(nav_.seph);
  }
}

SatEphemeris::~SatEphemeris()
{
  free(nav_.eph); free(nav_.geph); free(nav_.seph);
}

// Get satellite position and velocity
int SatEphemeris::getSatPosition(double time, 
          const std::vector<std::string>& prns, 
          std::vector<SatPosition>& sat_positions)
{
  gtime_t gtime = rtklib::double2gtime(time);
  double *rs, *dts, *var;
  double *rs_ssr, *dts_ssr, *var_ssr;
  int svh[MAXOBS], svh_ssr[MAXOBS], n;
  obs_t obs_tmp; obs_tmp.n = 0;
  if (!(obs_tmp.data = (obsd_t*)malloc(sizeof(obsd_t) * MAXOBS))) {
    LOG(FATAL) << "SatEphemeris::getSatPosition malloc error";
    return 0;
  }
  for (size_t i = 0; i < prns.size(); i++) {
    obs_tmp.data[obs_tmp.n].P[0] = 1.0;
    obs_tmp.data[obs_tmp.n].time = gtime;
    obs_tmp.data[obs_tmp.n].sat = rtklib::prn2sat(prns[i]);
    obs_tmp.n++;
  }
  n = obs_tmp.n;
  rs = mat(6, n); dts = mat(2, n); var = mat(1, n);
  rs_ssr = mat(6, n); dts_ssr = mat(2, n); var_ssr = mat(1, n);
  satposs(gtime, obs_tmp.data, n,
    &nav_, EPHOPT_BRDC, rs, dts, var, svh);
  satposs(gtime, obs_tmp.data, n,
    &nav_, EPHOPT_SSRAPC, rs_ssr, dts_ssr, var_ssr, svh_ssr);

  for (int i = 0; i < obs_tmp.n; i++) {
    SatPosition satpos;
    if (svh_ssr[i] != -1 && rs_ssr[i * 6] != 0 && dts_ssr[i * 2] != 0) {
      satpos.position = Eigen::Map<Eigen::Vector3d>(&rs_ssr[i * 6]);
      satpos.velocity = Eigen::Map<Eigen::Vector3d>(&rs_ssr[3 + i * 6]);
      satpos.clock = dts_ssr[i * 2] * CLIGHT;
      satpos.frequency = dts_ssr[1 + i * 2] * CLIGHT;
      satpos.type = SatPositionType::Precise;
    }
    else if (svh[i] != -1) {
      satpos.position = Eigen::Map<Eigen::Vector3d>(&rs[i * 6]);
      satpos.velocity = Eigen::Map<Eigen::Vector3d>(&rs[3 + i * 6]);
      satpos.clock = dts[i * 2] * CLIGHT;
      satpos.frequency = dts[1 + i * 2] * CLIGHT;
      satpos.type = SatPositionType::Broadcast;
    }
    else {
      satpos.type = SatPositionType::None;
    }
    sat_positions.push_back(satpos);
  }

  return 1;
}


}