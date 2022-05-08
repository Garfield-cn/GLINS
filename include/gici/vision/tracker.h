/**
* @Function: Feature tracking using LK optical flow
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#pragma once

#include "gici/utility/svo.h"

namespace gici {

// Track features by LK optical flow
void trackFeaturesPyrLK(const FramePtr& ref_frame,
      const FramePtr& cur_frame, OccupandyGrid2D& grid,
      bool use_pose_prediction = true);

}