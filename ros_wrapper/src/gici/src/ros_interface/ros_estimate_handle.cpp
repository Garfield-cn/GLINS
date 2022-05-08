/**
* @Function: Handle estimators
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/ros_interface/ros_estimate_handle.h"

#include "gici/gnss/gnss_estimating.h"
#include "gici/fusion/gnss_imu_estimating.h"
#include "gici/fusion/gnss_imu_camera_estimating.h"

namespace gici {

RosEstimateHandle::RosEstimateHandle(YAML::Node& node) : 
  EstimateHandle(node)
{}

RosEstimateHandle::~RosEstimateHandle()
{}

  // Bind estimators with ROS input and output streams
void RosEstimateHandle::bindWithRosStreams(
  const std::shared_ptr<RosStreamHandle>& stream_handle)
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
      std::shared_ptr<RosStream> stream = stream_handle->getStreamFromFormatorTag(tag);
      if (stream == nullptr) continue;
      
      // we bind the stream to all type of callbacks. The callback implementation will 
      // check the input data type to filter un-matched data out.
      // solution
      EstimatingBase::SolutionCallback solution_callback = std::bind(
        &RosStream::solutionOutputCallback, stream.get(), 
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
      estimatings_[i]->setSolutionCallback(solution_callback);

      // featured image
      EstimatingBase::FeaturedImageCallback featured_image_callback = std::bind(
        &RosStream::featuredImageCallback, stream.get(), std::placeholders::_1);
      estimatings_[i]->setFeaturedImageCallback(featured_image_callback);

      // map points
      EstimatingBase::MapPointCallback map_point_callback = std::bind(
        &RosStream::mapPointCallback, stream.get(), std::placeholders::_1);
      estimatings_[i]->setMapPointCallback(map_point_callback);
    }
  }
}

}