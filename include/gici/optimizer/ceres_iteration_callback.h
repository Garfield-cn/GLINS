/**
* @Function: Ceres iteration callback for debuging
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#pragma once

#include <ceres/iteration_callback.h>

namespace gici {

class CeresDebugCallback : public ceres::IterationCallback {
 public:
  explicit CeresDebugCallback() {}

  ~CeresDebugCallback() {}

  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) {
    // add breakpoint here
    return ceres::SOLVER_CONTINUE;
  }
};

}  // namespace gici
