/**
* @Function: GNSS relative constant errors for ceres backend
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#pragma once

#include "gici/estimate/const_error.h"

namespace gici {

using ClockError = ConstError<1, ErrorType::kClockError>;
using FrequencyError = ConstError<1, ErrorType::kFrequencyError>;
using TroposphereError = ConstError<1, ErrorType::kTroposphereError>;
using IonosphereError = ConstError<1, ErrorType::kIonosphereError>;
using SingleAmbiguityError = ConstError<1, ErrorType::kAmbiguityError>;

} 
