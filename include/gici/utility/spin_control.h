/**
* @Function: Tick control for spinning
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#pragma once

#include <iostream>
#include <vikit/timer.h>

namespace gici {

// Spin control
class SpinControl {
public:
  SpinControl(double duration);
  ~SpinControl() { }

  // Sleep to ensure spinning rate and restart timer
  void sleep();

  // (Re)set loop duration
  void setDuration(double duration) { duration_ = duration; }

  // Check global status
  static bool ok() { return ok_; }

  // Shutdown all spin controllers
  static void kill() { ok_ = false; }

private:
  // Tick controllers
  vk::Timer timer_;
  double duration_;

  // Whether enable spin in all objects
  static bool ok_;
};

}
