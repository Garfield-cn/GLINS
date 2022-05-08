/**
* @Function: Estimator thread
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/estimate/estimating.h"

#include "gici/utility/spin_control.h"
#include "gici/imu/imu_common.h"
#include "gici/imu/imu_error.h"

namespace gici {

EstimatingBase::EstimatingBase(YAML::Node& node) : 
  loop_duration_(1e-3), publish_timestamp_(0.0), 
  aligned_new_data_(false)
{
  // Get options
  if (!option_tools::safeGet(node, "tag", &tag_)) {
    LOG(ERROR) << "Unable to load estimator tag!";
  }
  
  std::string type_str;
  if (!option_tools::safeGet(node, "type", &type_str)) {
    LOG(ERROR) << "Unable to load estimator type!";
    return;
  }
  option_tools::convert(type_str, type_);

  if (!option_tools::safeGet(node, "loop_duration", &loop_duration_)) {
    LOG(ERROR) << "Unable to load estimator loop duration!";
  }
  if (loop_duration_ == 0.0) {
    if (!option_tools::safeGet(
        node, "loop_duration_align_tag", &loop_duration_align_tag_)) {
      LOG(ERROR) << "Unable to load estimator loop duration align tag!";
      return;
    }
  }

  std::string role_str;
  if (!option_tools::safeGet(node, "role", &role_str)) {
    LOG(INFO) << "Unable to load estimator role!";
  }
  if (role_str.size() == 0) role_ = SolutionRole::None;
  else option_tools::convert(role_str, role_);

  solution_.backend.timestamp = 0.0;
  solution_.timestamp = 0.0;
}

EstimatingBase::~EstimatingBase()
{}

// Start thread
void EstimatingBase::start()
{
  // Create thread
  quit_thread_ = false;
  thread_.reset(new std::thread(&EstimatingBase::run, this));
}

// Stop thread
void EstimatingBase::stop()
{
  // Kill thread
  if(thread_ != nullptr) {
    quit_thread_ = true;
    thread_->join();
    thread_.reset();
  }
}

// Loop processing
void EstimatingBase::run()
{
  // Spin until quit command or global shutdown called 
  SpinControl spin(loop_duration_ > 0.0 ? loop_duration_ : 1.0e-3);
  while (!quit_thread_ && SpinControl::ok()) {
    // Process funtion in every loop
    process();

    // Publish solution
    mutex_.lock();
    if (publish_timestamp_ != solution_.timestamp) {
      for (auto& solution_callback : solution_callbacks_) {
        solution_callback(tag_, role_, solution_);
      }
      publish_timestamp_ = solution_.timestamp;
    }
    mutex_.unlock();

    spin.sleep();
  }
}

}
