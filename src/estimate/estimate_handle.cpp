/**
* @Function: Handle estimators
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/estimate/estimate_handle.h"

#include "gici/gnss/gnss_estimating.h"
#include "gici/fusion/gnss_imu_estimating.h"
#include "gici/fusion/gnss_imu_camera_estimating.h"

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
    std::shared_ptr<EstimatingBase> estimating = makeEstimaing(estimator_node);
    if (estimating == nullptr) {
      LOG(ERROR) << "Failed to make estimating thread! Maybe there are some "
                 << "problems on your options.";
      continue;
    }
    estimatings_.push_back(estimating);

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

    // Match estimators
    std::vector<std::string> input_estimator_tags;
    if (!option_tools::safeGet(
        it["estimator"], "input_estimator_tags", &input_estimator_tags)) {
      LOG(INFO) << "Unable to load estimator input estimator tags!";
      continue;
    }
    input_estimator_tags_.push_back(input_estimator_tags);
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
void EstimateHandle::bindWithStreams(const std::shared_ptr<StreamHandle>& stream_handle)
{
  for (size_t i = 0; i < estimatings_.size(); i++) 
  {
    auto& estimating = estimatings_[i];
    auto& input_formator_tags = input_formator_tags_[i];
    auto& output_formator_tags = output_formator_tags_[i];

    // Bind with input streams
    // we do not care what type is the stream here, because we can distinguish them by tags
    StreamHandle::GnssCallback gnss_callback = 
      std::bind(&EstimatingBase::gnssCallback, estimating.get(), std::placeholders::_1);
    stream_handle->setGnssCallback(gnss_callback, input_formator_tags);

    StreamHandle::ImuCallback imu_callback = 
      std::bind(&EstimatingBase::imuCallback, estimating.get(), 
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    stream_handle->setImuCallback(imu_callback, input_formator_tags);

    StreamHandle::ImageCallback image_callback = 
      std::bind(&EstimatingBase::imageCallback, estimating.get(), 
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4);
    stream_handle->setImageCallback(image_callback, input_formator_tags);

    // Bind with output streams
    for (size_t j = 0; j < output_formator_tags.size(); j++) {
      std::string tag = output_formator_tags[j];
      std::shared_ptr<Streaming> stream = stream_handle->getStreamFromFormatorTag(tag);
      if (stream == nullptr) continue;
      EstimatingBase::SolutionCallback solution_callback = std::bind(
        &Streaming::solutionOutputCallback, stream.get(), 
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
      estimatings_[i]->setSolutionCallback(solution_callback);
    }
  }
}

// Bind esimators with other estimators
void EstimateHandle::bindWithEstimators()
{
  for (size_t i = 0; i < estimatings_.size(); i++) 
  {
    auto& input_estimator_tags = input_estimator_tags_[i];
    for (size_t j = 0; j < input_estimator_tags.size(); j++) 
    {
      for (size_t k = 0; k < estimatings_.size(); k++) {
        std::string tag = estimatings_[k]->getTag();
        if (tag != input_estimator_tags[j]) continue;
        EstimatingBase::SolutionCallback solution_callback = std::bind(
          &EstimatingBase::solutionCallback, estimatings_[i].get(), 
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        estimatings_[k]->setSolutionCallback(solution_callback);
      }
    }

  }
}

// Make estimating thread from yaml
std::shared_ptr<EstimatingBase> EstimateHandle::makeEstimaing(YAML::Node& node)
{
  std::string type_str;
  EstimatorType type;
  if (!option_tools::safeGet(node, "type", &type_str)) {
    LOG(ERROR) << "Unable to load estimator type!";
    return nullptr;
  }
  option_tools::convert(type_str, type);

  if (type == EstimatorType::Spp || 
      type == EstimatorType::Dgnss || 
      type == EstimatorType::Rtk) {
    return std::make_shared<GnssEstimating>(node);
  }
  else if (type == EstimatorType::GnssImuLc ||
           type == EstimatorType::RtkImuTc) {
    return std::make_shared<GnssImuEstimating>(node);
  }
  else if (type == EstimatorType::GnssImuCameraSrr || 
           type == EstimatorType::RtkImuCameraRrr) {
    return std::make_shared<GnssImuCameraEstimating>(node);
  }
  else return nullptr;
}

}