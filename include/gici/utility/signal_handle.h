/**
* @Function: Handle exception signals
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#ifndef SIGNAL_HANDLE_H
#define SIGNAL_HANDLE_H

#include <signal.h>

namespace gici {

// Initialize all signal handles
extern void initializeSignalHandles(void);

}

#endif
