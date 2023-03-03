/**
* @Function: Estimator types
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/estimate/estimator_types.h"

namespace gici {

// Static variables
std::multimap<BackendId, State> State::overlaps;

// Convert from estimator type to string
std::string estimatorTypeToString(const EstimatorType& type)
{
  if (type == EstimatorType::None) return "None";
  if (type == EstimatorType::Spp) return "SPP";
  if (type == EstimatorType::Sdgnss) return "SDGNSS";
  if (type == EstimatorType::Dgnss) return "DGNSS";
  if (type == EstimatorType::Rtk) return "RTK";
  if (type == EstimatorType::Ppp) return "PPP";
  if (type == EstimatorType::GnssImuLc) return "GNSS/IMU LC";
  if (type == EstimatorType::RtkImuTc) return "RTK/IMU TC";
  if (type == EstimatorType::GnssImuCameraSrr) return "GNSS/IMU/Camera SSR";
  if (type == EstimatorType::RtkImuCameraRrr) return "RTK/IMU/Camera RRR";
  return "";
}

} // namespace gici
