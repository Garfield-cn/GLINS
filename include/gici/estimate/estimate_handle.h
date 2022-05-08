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
  EstimateHandle(YAML::Node& node);
  ~EstimateHandle();

  // Bind estimators with input and output streams
  void bindWithStreams(const std::shared_ptr<StreamHandle>& stream_handle);

  // Bind esimators with other estimators
  void bindWithEstimators();

private: 
  // Make estimating thread from yaml
  std::shared_ptr<EstimatingBase> makeEstimaing(YAML::Node& node);

protected:
  // estimating threads
  std::vector<std::shared_ptr<EstimatingBase>> estimatings_;
  std::vector<std::vector<std::string>> input_formator_tags_;
  std::vector<std::vector<std::string>> input_estimator_tags_;
  std::vector<std::vector<std::string>> output_formator_tags_;
};

}
