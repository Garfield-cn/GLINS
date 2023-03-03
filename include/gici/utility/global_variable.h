/**
* @Function: Global variables
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#pragma once

#include <iostream>

namespace gici {

namespace global {

// Tell cost function block not to compute residual weighting (used for debug)
extern bool __cost_function_no_residual_weighting__;

// For debug
extern int __debug_print__;

}

}
