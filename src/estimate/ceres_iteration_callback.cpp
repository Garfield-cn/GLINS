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
  return ceres::SOLVER_CONTINUE;
}

}  // namespace gici
