/**
* @Function: Ceres iteration callback for debuging
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/estimate/ceres_iteration_callback.h"

#include <glog/logging.h>

namespace gici {

ceres::CallbackReturnType CeresDebugCallback::handle(
  const ceres::IterationSummary& summary) 
{
  // if (summary.cost > 1e4) 
  // {
  //   LOG(ERROR) << "Not good!";
  // }

  return ceres::SOLVER_CONTINUE;
}

}  // namespace gici
