/**
* @Function: Handle error signals
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/utility/signal_handle.h"

#include <glog/logging.h>

namespace gici {

// Handle pipe exception from TCP/IP
static void handlePipe(int sig)
{
  LOG(ERROR) << "Received a pipe exception!";
}

// Initialize all signal handles
extern void initializeSignalHandles()
{
  struct sigaction sa;
  sa.sa_handler = handlePipe;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  sigaction(SIGPIPE, &sa, NULL);
}

}