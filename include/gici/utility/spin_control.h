/**
* @Function: Tick control for spinning
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#ifndef SPIN_CONTROL_H
#define SPIN_CONTROL_H

#include <iostream>
#include <vikit/timer.h>

namespace gici {

// Spin control
class SpinControl {
public:
  SpinControl(double duration);
  ~SpinControl(void) { }

  // Sleep to ensure spinning rate and restart timer
  void sleep(void);

  // Check global status
  static bool ok(void) { return ok_; }

  // Shutdown all spin controllers
  static void kill(void) { ok_ = false; }

private:
  // Tick controllers
  vk::Timer timer_;
  double duration_;

  // Whether enable spin in all objects
  static bool ok_;
};

}

#endif