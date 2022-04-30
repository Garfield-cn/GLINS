/**
* @Function: Handle estimators
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#pragma once

#include <iostream>
#include <vector>
#include <functional>
#include <glog/logging.h>

#include "gici/estimate/estimating.h"
#include "gici/stream/stream_handle.h"

namespace gici {

class EstimateHandle {
public:
  using Ptr = std::shared_ptr<EstimateHandle>;

  EstimateHandle(YAML::Node& node);
  ~EstimateHandle();

  // Bind estimators with input and output streams
  void bindWithStreams(const StreamHandle::Ptr& stream_handle);

protected:
  // estimating threads
  std::vector<Estimating::Ptr> estimatings_;
  std::vector<std::vector<std::string>> input_formator_tags_;
  std::vector<std::vector<std::string>> output_formator_tags_;
};

}
