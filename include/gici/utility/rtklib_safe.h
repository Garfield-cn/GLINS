/**
* @Function: Un-define RTKLIB types to avoid conflicts
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#ifndef RTKLIB_SAFE
#define RTKLIB_SAFE

#include <rtklib.h>

#ifdef __cplusplus
extern "C" {
#endif

#undef lock
#undef unlock

#ifdef __cplusplus
}
#endif

#endif
