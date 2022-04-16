/**
* @Function: GNSS relative constant errors for ceres backend
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#pragma once

#include "gici/optimizer/relative_const_error.h"

namespace gici {

using RelativePositionError = RelativeConstError<3, ErrorType::kRelativePositionError>;
using RelativeAmbiguityError = RelativeConstError<1, ErrorType::kRelativeAmbiguityError>;

} 
