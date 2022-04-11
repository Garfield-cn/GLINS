/**
* @Function: GNSS common functions
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/gnss/gnss_common.h"

#include <glog/logging.h>

namespace gici {

namespace gnss_common {

// ----------------------------------------------------------
// Convert char system to int system
int systemConvert(char sys)
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
char systemConvert(int sys)
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
int prnToSat(std::string prn)
{
  int system = systemConvert(prn[0]);
  int prnnum = atoi(prn.substr(1, 2).data());
  return satno(system, prnnum);
}

// Convert RTKLIB sat to PRN string
std::string satToPrn(int sat)
{
  int prnnum;
  char system = systemConvert(satsys(sat, &prnnum));
  char prnbuf[10];
  sprintf(prnbuf, "%c%02d", system, prnnum);
  return std::string(prnbuf);
}

// Convert gtime to double
double gtimeToDouble(gtime_t time)
{
  return (static_cast<double>(time.time) + time.sec);
}

// Convert double to gtime
gtime_t doubleToGtime(double time)
{
  gtime_t gtime;
  gtime.time = floor(time);
  gtime.sec = time - floor(time);
  return gtime;
}

// ----------------------------------------------------------
// Check whether the system is used
bool useSystem(GNSSCommonOptions options, const char system)
{
  auto it = std::find(options.system_exclude.begin(), 
    options.system_exclude.end(), system);
  if (it == options.system_exclude.end()) return true;
  else return false;
}

// Check whether the satellite is used
bool useSatellite(GNSSCommonOptions options, const std::string prn)
{
  auto it = std::find(options.satellite_exclude.begin(), 
    options.satellite_exclude.end(), prn);
  if (it == options.satellite_exclude.end()) return true;
  else return false;
}

// Check whether the code type is used
bool useCode(GNSSCommonOptions options, const int code_type)
{
  auto it = std::find(options.code_exclude.begin(), 
    options.code_exclude.end(), code_type);
  if (it == options.code_exclude.end()) return true;
  else return false;
}

// Check elevation threshold
bool checkElevation(GNSSCommonOptions options, 
  const GNSSMeasurement& measurement, size_t satellite_index)
{
  CHECK(measurement.satellites.size() > satellite_index);

  if (checkZero(measurement.position)) {
    // we do not know whether to use this satellite
    return true;
  }

  if (measurement.satellites[satellite_index].sat_position == 
      Eigen::Vector3d::Zero()) {
    return false;
  }

  double elevation = satelliteElevation(
    measurement.satellites[satellite_index].sat_position, measurement.position);
  if (radToDegree(elevation) > options.min_elevation) return true;
  else return false;
}

// ----------------------------------------------------------
// Saastamoinen troposphere delay model
double troposphereSaastamoinen(double time, 
  const Eigen::Vector3d& ecef, double elevation, double humi)
{
  gtime_t gtime = doubleToGtime(time);
  double azel[2];
  azel[0] = 0.0; azel[1] = elevation;
  Eigen::Vector3d lla;
  ecef2pos(ecef.data(), lla.data());
  return tropmodel(gtime, lla.data(), azel, humi);
}

// GMF troposphere delay model
void troposphereGMF(double time, 
  const Eigen::Vector3d& ecef, double elevation, 
  double* gmfh, double* gmfw)
{
  double dfac[20], P[10][10], aP[55], bP[55], t;

  int i, n, m, nmax, mmax;
  double doy, phh;
  double ah, bh, ch, aw, bw, cw;
  double ahm, aha, awm, awa;
  double c10h, c11h, c0h;
  double a_ht, b_ht, c_ht;
  double sine, beta, gamma, topcon;
  double hs_km, ht_corr, ht_corr_coef;

  gtime_t gtime = doubleToGtime(time);
  Eigen::Vector3d lla;
  ecef2pos(ecef.data(), lla.data());

  //mjulianday mjd;
  //time2mjulianday(gt,&mjd);
  //dmjd=mjd.day+(mjd.ds.sn+mjd.ds.tos)/86400.0;
  const double ep[] = { 2000, 1, 1, 12, 0, 0 };
  double mjd, lat, lon, hgt, zd;

  if (lla[2] < -1000.0 || lla[2] > 20000.0)
  {
    if (gmfw) *gmfw = 0.0;
    return;
  }


  mjd = 51544.5 + (timediff(gtime, epoch2time(ep))) / 86400.0;
  lat = lla[0];
  lon = lla[1];
  hgt = lla[2] - geoidh(lla.data()); /* height in m (mean sea level) */
  zd = PI / 2.0 - elevation;


  //double PI = 3.1415926535897932384626433;
  double TWOPI = 6.283185307179586476925287;


  //reference day is 28 January
  //this is taken from Niell(1996) to be consistent
  //doy=dmjd-44239.0+1-28;
  doy = mjd - 44239.0 - 27;

  static double ah_mean[55] =
  {
    +1.2517e+02,	+8.503e-01,	+6.936e-02,	-6.760e+00, +1.771e-01,
    +1.130e-02,		+5.963e-01,	+1.808e-02, +2.801e-03, -1.414e-03,
    -1.212e+00,		+9.300e-02,	+3.683e-03, +1.095e-03, +4.671e-05,
    +3.959e-01,		-3.867e-02, +5.413e-03, -5.289e-04, +3.229e-04,
    +2.067e-05,		+3.000e-01, +2.031e-02, +5.900e-03, +4.573e-04,
    -7.619e-05,		+2.327e-06, +3.845e-06, +1.182e-01, +1.158e-02,
    +5.445e-03,		+6.219e-05, +4.204e-06, -2.093e-06, +1.540e-07,
    -4.280e-08,		-4.751e-01, -3.490e-02, +1.758e-03, +4.019e-04,
    -2.799e-06,		-1.287e-06, +5.468e-07, +7.580e-08, -6.300e-09,
    -1.160e-01,		+8.301e-03, +8.771e-04, +9.955e-05, -1.718e-06,
    -2.012e-06,		+1.170e-08, +1.790e-08, -1.300e-09, +1.000e-10
  };

  static double bh_mean[55] =
  {
    +0.000e+00,	+0.000e+00, +3.249e-02, +0.000e+00, +3.324e-02,
    +1.850e-02, +0.000e+00, -1.115e-01, +2.519e-02, +4.923e-03,
    +0.000e+00, +2.737e-02, +1.595e-02, -7.332e-04, +1.933e-04,
    +0.000e+00, -4.796e-02, +6.381e-03, -1.599e-04, -3.685e-04,
    +1.815e-05, +0.000e+00, +7.033e-02, +2.426e-03, -1.111e-03,
    -1.357e-04, -7.828e-06, +2.547e-06, +0.000e+00, +5.779e-03,
    +3.133e-03, -5.312e-04, -2.028e-05, +2.323e-07, -9.100e-08,
    -1.650e-08, +0.000e+00, +3.688e-02, -8.638e-04, -8.514e-05,
    -2.828e-05, +5.403e-07, +4.390e-07, +1.350e-08, +1.800e-09,
    +0.000e+00, -2.736e-02, -2.977e-04, +8.113e-05, +2.329e-07,
    +8.451e-07, +4.490e-08, -8.100e-09, -1.500e-09, +2.000e-10
  };

  static double ah_amp[55] =
  {
    -2.738e-01, -2.837e+00, +1.298e-02, -3.588e-01, +2.413e-02,
    +3.427e-02, -7.624e-01, +7.272e-02, +2.160e-02, -3.385e-03,
    +4.424e-01, +3.722e-02, +2.195e-02, -1.503e-03, +2.426e-04,
    +3.013e-01, +5.762e-02, +1.019e-02, -4.476e-04, +6.790e-05,
    +3.227e-05, +3.123e-01, -3.535e-02, +4.840e-03, +3.025e-06,
    -4.363e-05, +2.854e-07, -1.286e-06, -6.725e-01, -3.730e-02,
    +8.964e-04, +1.399e-04, -3.990e-06, +7.431e-06, -2.796e-07,
    -1.601e-07, +4.068e-02, -1.352e-02, +7.282e-04, +9.594e-05,
    +2.070e-06, -9.620e-08, -2.742e-07, -6.370e-08, -6.300e-09,
    +8.625e-02, -5.971e-03, +4.705e-04, +2.335e-05, +4.226e-06,
    +2.475e-07, -8.850e-08, -3.600e-08, -2.900e-09, +0.000e+00
  };

  static double bh_amp[55] =
  {
    +0.000e+00, +0.000e+00, -1.136e-01, +0.000e+00, -1.868e-01,
    -1.399e-02, +0.000e+00, -1.043e-01, +1.175e-02, -2.240e-03,
    +0.000e+00, -3.222e-02, +1.333e-02, -2.647e-03, -2.316e-05,
    +0.000e+00, +5.339e-02, +1.107e-02, -3.116e-03, -1.079e-04,
    -1.299e-05, +0.000e+00, +4.861e-03, +8.891e-03, -6.448e-04,
    -1.279e-05, +6.358e-06, -1.417e-07, +0.000e+00, +3.041e-02,
    +1.150e-03, -8.743e-04, -2.781e-05, +6.367e-07, -1.140e-08,
    -4.200e-08, +0.000e+00, -2.982e-02, -3.000e-03, +1.394e-05,
    -3.290e-05, -1.705e-07, +7.440e-08, +2.720e-08, -6.600e-09,
    +0.000e+00, +1.236e-02, -9.981e-04, -3.792e-05, -1.355e-05,
    +1.162e-06, -1.789e-07, +1.470e-08, -2.400e-09, -4.000e-10
  };

  static double aw_mean[55] =
  {
    +5.640e+01, +1.555e+00, -1.011e+00, -3.975e+00, +3.171e-02,
    +1.065e-01, +6.175e-01, +1.376e-01, +4.229e-02, +3.028e-03,
    +1.688e+00, -1.692e-01, +5.478e-02, +2.473e-02, +6.059e-04,
    +2.278e+00, +6.614e-03, -3.505e-04, -6.697e-03, +8.402e-04,
    +7.033e-04, -3.236e+00, +2.184e-01, -4.611e-02, -1.613e-02,
    -1.604e-03, +5.420e-05, +7.922e-05, -2.711e-01, -4.406e-01,
    -3.376e-02, -2.801e-03, -4.090e-04, -2.056e-05, +6.894e-06,
    +2.317e-06, +1.941e+00, -2.562e-01, +1.598e-02, +5.449e-03,
    +3.544e-04, +1.148e-05, +7.503e-06, -5.667e-07, -3.660e-08,
    +8.683e-01, -5.931e-02, -1.864e-03, -1.277e-04, +2.029e-04,
    +1.269e-05, +1.629e-06, +9.660e-08, -1.015e-07, -5.000e-10
  };

  static double bw_mean[55] =
  {
    +0.000e+00, +0.000e+00, +2.592e-01, +0.000e+00, +2.974e-02,
    -5.471e-01, +0.000e+00, -5.926e-01, -1.030e-01, -1.567e-02,
    +0.000e+00, +1.710e-01, +9.025e-02, +2.689e-02, +2.243e-03,
    +0.000e+00, +3.439e-01, +2.402e-02, +5.410e-03, +1.601e-03,
    +9.669e-05, +0.000e+00, +9.502e-02, -3.063e-02, -1.055e-03,
    -1.067e-04, -1.130e-04, +2.124e-05, +0.000e+00, -3.129e-01,
    +8.463e-03, +2.253e-04, +7.413e-05, -9.376e-05, -1.606e-06,
    +2.060e-06, +0.000e+00, +2.739e-01, +1.167e-03, -2.246e-05,
    -1.287e-04, -2.438e-05, -7.561e-07, +1.158e-06, +4.950e-08,
    +0.000e+00, -1.344e-01, +5.342e-03, +3.775e-04, -6.756e-05,
    -1.686e-06, -1.184e-06, +2.768e-07, +2.730e-08, +5.700e-09
  };

  static double aw_amp[55] =
  {
    +1.023e-01, -2.695e+00, +3.417e-01, -1.405e-01, +3.175e-01,
    +2.116e-01, +3.536e+00, -1.505e-01, -1.660e-02, +2.967e-02,
    +3.819e-01, -1.695e-01, -7.444e-02, +7.409e-03, -6.262e-03,
    -1.836e+00, -1.759e-02, -6.256e-02, -2.371e-03, +7.947e-04,
    +1.501e-04, -8.603e-01, -1.360e-01, -3.629e-02, -3.706e-03,
    -2.976e-04, +1.857e-05, +3.021e-05, +2.248e+00, -1.178e-01,
    +1.255e-02, +1.134e-03, -2.161e-04, -5.817e-06, +8.836e-07,
    -1.769e-07, +7.313e-01, -1.188e-01, +1.145e-02, +1.011e-03,
    +1.083e-04, +2.570e-06, -2.140e-06, -5.710e-08, +2.000e-08,
    -1.632e+00, -6.948e-03, -3.893e-03, +8.592e-04, +7.577e-05,
    +4.539e-06, -3.852e-07, -2.213e-07, -1.370e-08, +5.800e-09
  };

  static double bw_amp[55] =
  {
    +0.000e+00, +0.000e+00, -8.865e-02, +0.000e+00, -4.309e-01,
    +6.340e-02, +0.000e+00, +1.162e-01, +6.176e-02, -4.234e-03,
    +0.000e+00, +2.530e-01, +4.017e-02, -6.204e-03, +4.977e-03,
    +0.000e+00, -1.737e-01, -5.638e-03, +1.488e-04, +4.857e-04,
    -1.809e-04, +0.000e+00, -1.514e-01, -1.685e-02, +5.333e-03,
    -7.611e-05, +2.394e-05, +8.195e-06, +0.000e+00, +9.326e-02,
    -1.275e-02, -3.071e-04, +5.374e-05, -3.391e-05, -7.436e-06,
    +6.747e-07, +0.000e+00, -8.637e-02, -3.807e-03, -6.833e-04,
    -3.861e-05, -2.268e-05, +1.454e-06, +3.860e-07, -1.068e-07,
    +0.000e+00, -2.658e-02, -1.947e-03, +7.131e-04, -3.506e-05,
    +1.885e-07, +5.792e-07, +3.990e-08, +2.000e-08, -5.700e-09
  };

  //parameter t
  t = sin(lat);

  //degree n and order m
  nmax = 9;
  mmax = 9;

  //determine nmax!(faktorielle) moved by 1
  dfac[0] = 1;
  for (i = 1; i <= 2 * nmax + 1; i++)
    dfac[i] = dfac[i - 1] * i;


  int j;
  int ir;
  int k;
  double sum;
  // determine Legendre functions (Heiskanen and Moritz, Physical Geodesy, 1967, eq. 1-62)
  for (i = 0; i <= nmax; i++)
  {
    for (j = 0; j <= std::min(i, mmax); j++)
    {
      ir = int((i - j) / 2);
      sum = 0.0;

      for (k = 0; k <= ir; k++)
      {
        sum = sum + pow(-1.0, k) * dfac[2 * i - 2 * k] / dfac[k] / 
          dfac[i - k] / dfac[i - j - 2 * k] * pow(t, i - j - 2 * k);
      }

      //Legender functions moved by 1
      P[i][j] = 1.0 / pow(2.0, i) * sqrt(pow(1 - t * t, j)) * sum;
    }
  }

  //spherical harmonics
  i = 0;
  double dt;
  for (n = 0; n <= 9; n++)
  {
    for (m = 0; m <= n; m++)
    {
      i = i + 1;
      dt = m * lon;
      aP[i - 1] = P[n][m] * cos(dt);
      bP[i - 1] = P[n][m] * sin(dt);
    }
  }

  //hydrostatic
  bh = 0.0029;
  c0h = 0.062;

  if (lat < 0)  	//southern hemisphere
  {
    phh = PI;
    c11h = 0.007;
    c10h = 0.002;
  }
  else  	//northern hemisphere
  {
    phh = 0;
    c11h = 0.005;
    c10h = 0.001;
  }

  ch = c0h + ((cos(doy / 365.25 * TWOPI + 
    phh) + 1.0) * c11h / 2.0 + c10h) * (1.0 - cos(lat));

  ahm = 0.0;
  aha = 0.0;
  for (i = 1; i <= 55; i++)
  {
    ahm = ahm + (ah_mean[i - 1] * aP[i - 1] + bh_mean[i - 1] * bP[i - 1]) * 1.0e-5;
    aha = aha + (ah_amp[i - 1] * aP[i - 1] + bh_amp[i - 1] * bP[i - 1]) * 1.0e-5;
  }

  ah = ahm + aha * cos(doy / 365.25 * TWOPI);

  sine = sin(elevation);
  beta = bh / (sine + ch);
  gamma = ah / (sine + beta);
  topcon = (1.0 + ah / (1.0 + bh / (1.0 + ch)));
  *gmfh = topcon / (sine + gamma);

  a_ht = 2.53e-5;	//2.53 from http://maia.usno.navy.mil/conv2010/chapter9/GMF.F

  b_ht = 5.49e-3;
  c_ht = 1.14e-3;
  hs_km = hgt / 1000.0;

  beta = b_ht / (sine + c_ht);
  gamma = a_ht / (sine + beta);
  topcon = (1.0 + a_ht / (1.0 + b_ht / (1.0 + c_ht)));
  ht_corr_coef = 1.0 / sine - topcon / (sine + gamma);
  ht_corr = ht_corr_coef * hs_km;
  *gmfh = *gmfh + ht_corr;

  //wet
  bw = 0.00146;
  cw = 0.04391;

  awm = 0.0;
  awa = 0.0;
  for (i = 1; i <= 55; i++)
  {
    awm = awm + (aw_mean[i - 1] * aP[i - 1] + bw_mean[i - 1] * bP[i - 1]) * 1e-5;
    awa = awa + (aw_amp[i - 1] * aP[i - 1] + bw_amp[i - 1] * bP[i - 1]) * 1e-5;
  }
  aw = awm + awa * cos(doy / 365.25 * TWOPI);

  beta = bw / (sine + cw);
  gamma = aw / (sine + beta);
  topcon = (1.0 + aw / (1.0 + bw / (1.0 + cw)));
  *gmfw = topcon / (sine + gamma);
}

// Broadcast ionosphere model
double ionosphereBroadcast(double time, const Eigen::Vector3d& ecef, 
  double azimuth, double elevation, double wavelength, 
  const Eigen::VectorXd& parameters)
{
  gtime_t gtime = doubleToGtime(time);
  double azel[2];
  azel[0] = azimuth; azel[1] = elevation;
  Eigen::Vector3d lla;
  ecef2pos(ecef.data(), lla.data());
  Eigen::VectorXd parameters_local = parameters;
  if (parameters.size() < 8) parameters_local = Eigen::VectorXd::Zero(8);
  double ion_l1 = ionmodel(gtime, parameters_local.data(), lla.data(), azel);
  return ion_l1 * square(wavelength / (CLIGHT / FREQ1));
}

// Dual-frequenct ionosphere model
double ionosphereDualFrequency(
  const Observation& obs_1, const Observation& obs_2)
{
  return (obs_1.pesudorange - obs_2.pesudorange) / 
         (1 - square(obs_2.wavelength / obs_1.wavelength));
}

// Compute Receiver to satellite distance considering the earth rotation effect
double satelliteToReceiverDistance(
  const Eigen::Vector3d satellite_ecef, const Eigen::Vector3d receiver_ecef)
{
  // Correct the satellite position
  double rho0 = (satellite_ecef - receiver_ecef).norm();
  double dPhi = OMGE * rho0 / CLIGHT;

  Eigen::Vector3d xRec;

  xRec(0) = receiver_ecef(0) * cos(dPhi) - receiver_ecef(1) * sin(dPhi);
  xRec(1) = receiver_ecef(1) * cos(dPhi) + receiver_ecef(0) * sin(dPhi);
  xRec(2) = receiver_ecef(2);

  // Compute the distance
  return (satellite_ecef - xRec).norm();
}

// Satellite elevation angle
double satelliteElevation(
  const Eigen::Vector3d satellite_ecef, const Eigen::Vector3d receiver_ecef)
{
  Eigen::Vector3d rr = satellite_ecef - receiver_ecef;
  double rho0 = rr.norm();

  double enu[3], pos[3];
  ecef2pos(receiver_ecef.data(), pos);
  ecef2enu(pos, rr.data(), enu);

  double el = acos(sqrt(enu[0]*enu[0] + enu[1]*enu[1]) / rho0);
  if (enu[2] < 0) el *= -1.0;
  return el;
}

// Satellite azimuth angle
double satelliteAzimuth(
  const Eigen::Vector3d satellite_ecef, const Eigen::Vector3d receiver_ecef)
{
  Eigen::Vector3d rr = satellite_ecef - receiver_ecef;
  double rho0 = rr.norm();

  double enu[3], pos[3];
  ecef2pos(receiver_ecef.data(), pos);
  ecef2enu(pos, rr.data(), enu);

  return atan2(enu[0], enu[1]);
}

}

}