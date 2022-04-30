/**
* @Function: Handle estimators
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/estimate/estimate_handle.h"

namespace gici {

EstimateHandle::EstimateHandle(YAML::Node& node)
{
  // Initialize estimators
  if (!node["estimate"].IsDefined()) {
    LOG(ERROR) << "Unable to load estimators!";
    return;
  }
  YAML::Node estimator_nodes = node["estimate"];
  for (auto it : estimator_nodes) {
    if (!it["estimator"].IsDefined()) {
      LOG(ERROR) << "Unable to load estimator!";
      continue;
    }
    // add estimator
    YAML::Node estimator_node = it["estimator"];
    estimatings_.push_back(std::make_shared<Estimating>(estimator_node));

    // Match input and output streams
    std::vector<std::string> input_formator_tags;
    if (!option_tools::safeGet(
        it["estimator"], "input_formator_tags", &input_formator_tags)) {
      LOG(ERROR) << "Unable to load estimator input formator tags!";
      continue;
    }
    input_formator_tags_.push_back(input_formator_tags);

    std::vector<std::string> output_formator_tags;
    if (!option_tools::safeGet(
        it["estimator"], "output_formator_tags", &output_formator_tags)) {
      LOG(ERROR) << "Unable to load estimator output formator tags!";
      continue;
    }
    output_formator_tags_.push_back(output_formator_tags);
  }

  // Start estimators
  for (size_t i = 0; i < estimatings_.size(); i++) {
    estimatings_[i]->start();
  }
}

EstimateHandle::~EstimateHandle()
{
  // Stop estimators
  for (size_t i = 0; i < estimatings_.size(); i++) {
    estimatings_[i]->stop();
  }
}

// Bind estimators with input and output streams
void EstimateHandle::bindWithStreams(const StreamHandle::Ptr& stream_handle)
{
  for (size_t i = 0; i < estimatings_.size(); i++) {
    auto& estimating = estimatings_[i];
    auto& input_streamer_tags = input_formator_tags_[i];
    auto& output_formator_tags = output_formator_tags_[i];

    // Bind with input streams
    // we do not care what type is the stream here, because we can distinguish them by tags
    StreamHandle::GnssCallback gnss_callback = 
      std::bind(&Estimating::gnssCallback, estimating.get(), std::placeholders::_1);
    stream_handle->setGnssCallback(gnss_callback, input_streamer_tags);

    StreamHandle::ImuCallback imu_callback = 
      std::bind(&Estimating::imuCallback, estimating.get(), 
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    stream_handle->setImuCallback(imu_callback, input_streamer_tags);

    StreamHandle::ImageCallback image_callback = 
      std::bind(&Estimating::imageCallback, estimating.get(), 
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4);
    stream_handle->setImageCallback(image_callback, input_streamer_tags);

    // Bind with output streams
    for (size_t j = 0; j < output_formator_tags.size(); j++) {
      std::string tag = output_formator_tags[j];
      Streaming::Ptr stream = stream_handle->getStreamFromFormatorTag(tag);
      Estimating::SolutionCallback solution_callback = std::bind(
        &Streaming::solutionOutputCallback, stream.get(), std::placeholders::_1);
      estimatings_[i]->setSolutionCallback(solution_callback);
    }
  }
}

}