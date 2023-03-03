/**
* @Function: GNSS types
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/gnss/gnss_types.h"

#include "gici/gnss/gnss_common.h"

namespace gici {

// Static variable
int32_t GnssMeasurement::epoch_cnt_ = 0;
std::vector<char> gnss_systems{'G', 'R', 'E', 'C'};

}