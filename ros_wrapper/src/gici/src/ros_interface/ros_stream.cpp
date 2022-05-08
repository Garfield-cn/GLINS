/**
* @Function: Handle ROS stream publish and subscribe
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/ros_interface/ros_stream.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "gici/utility/spin_control.h"

namespace gici {

RosStream::RosStream(
  ros::NodeHandle& nh, YAML::Node& node, int istreamer) : 
  nh_(nh), frame_id_("World")
{
  // Get streamer option
  if (!node["streamers"][istreamer].IsDefined() ||
    !node["streamers"][istreamer]["streamer"].IsDefined()) {
    LOG(ERROR) << "Streamer " << istreamer << " not defined!";
    return;
  }
  YAML::Node streamer_node = node["streamers"][istreamer]["streamer"];
  if (!option_tools::safeGet(streamer_node, "tag", &tag_)) {
    LOG(ERROR) << "Unable to load streamer tag!";
    return;
  }
  if (!option_tools::safeGet(streamer_node, "formator_tag", &formator_tag_)) {
    LOG(ERROR) << "Unable to load formator tag for ROS stream!";
    return;
  }
  if (!option_tools::safeGet(streamer_node, "topic_name", &topic_name_)) {
    LOG(ERROR) << "Unable to load ROS topic name!";
    return;
  }
  if (!option_tools::safeGet(streamer_node, "queue_size", &queue_size_)) {
    LOG(INFO) << "Unable to load ROS topic queue size. Using default instead";
    queue_size_ = 10;
  }
  std::string type_str;
  if (!option_tools::safeGet(streamer_node, "io", &type_str)) {
    LOG(ERROR) << "Unable to load ROS streamer I/O type!";
    return;
  }
  option_tools::convert(type_str, io_type_);
  if (io_type_ != StreamIOType::Input || io_type_ != StreamIOType::Output) {
    LOG(ERROR) << "Invalid IO type for ROS streamer!";
  }
  // for input stream, we must specify its role, so that we can let estimator know
  // how to handle it
  if (io_type_ == StreamIOType::Input &&
     !option_tools::safeGet(streamer_node, "formator_role", &roles_)) {
    LOG(ERROR) << "Unable to load formator role for ROS stream!";
    return;
  }
  // initialize ros topic
  std::string data_format;
  if (!option_tools::safeGet(streamer_node, "format", &data_format)) {
    LOG(ERROR) << "Unable to load ROS topic format!";
    return;
  }
  if (data_format == "image") {
    data_format_ = RosDataFormat::Image;
    if (io_type_ == StreamIOType::Input) {
      subscriber_ = nh_.subscribe<sensor_msgs::Image>(
        topic_name_, queue_size_, boost::bind(&RosStream::imageCallback, this, _1));
    }
    else {
      publisher_ = nh_.advertise<sensor_msgs::Image>(topic_name_, queue_size_);
    }
  }
  else if (data_format == "imu") {
    data_format_ = RosDataFormat::Imu;
    if (io_type_ == StreamIOType::Input) {
      subscriber_ = nh_.subscribe<sensor_msgs::Imu>(
        topic_name_, queue_size_, boost::bind(&RosStream::imuCallback, this, _1));
    }
    else {
      LOG(ERROR) << "Set IMU topic as output is disabled!";
    }
  }
  else if (data_format == "gnss_raw") {
    data_format_ = RosDataFormat::GnssRaw;
    if (io_type_ == StreamIOType::Input) {
      // TODO
    }
    else {
      LOG(ERROR) << "Set GNSS Raw topic as output is disabled!";
    }
  }
  else if (data_format == "pose_stamped") {
    data_format_ = RosDataFormat::PoseStamped;
    if (io_type_ == StreamIOType::Input) {
      LOG(ERROR) << "Set pose stamped topic as input is disabled! If you want to input pose" 
                 << ", use \"pose_with_covariance_stamped\" instead.";
      return;
    }
    else {
      publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(topic_name_, queue_size_);
      // publish transform if sub-frame is specified
      if (option_tools::safeGet(streamer_node, "subframe_id", &subframe_id_)) {
        tranform_broadcaster_ = std::make_unique<tf::TransformBroadcaster>();
      }
    }
  }
  else if (data_format == "pose_with_covariance_stamped") {
    data_format_ = RosDataFormat::PoseWithCovarianceStamped;
    if (io_type_ == StreamIOType::Input) {
      subscriber_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
        topic_name_, queue_size_, boost::bind(&RosStream::poseCallback, this, _1));
    }
    else {
      publisher_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        topic_name_, queue_size_);
      // publish transform if sub-frame is specified
      if (option_tools::safeGet(streamer_node, "subframe_id", &subframe_id_)) {
        tranform_broadcaster_ = std::make_unique<tf::TransformBroadcaster>();
      }
    }
  }
  else if (data_format == "marker") {
    data_format_ = RosDataFormat::Marker;
    if (io_type_ == StreamIOType::Input) {
      LOG(ERROR) << "Set marker topic as input is disabled!";
      return;
    }
    else {
      publisher_ = nh_.advertise<visualization_msgs::Marker>(topic_name_, queue_size_);
    }
  }
  else if (data_format == "path") {
    data_format_ = RosDataFormat::Path;
    if (io_type_ == StreamIOType::Input) {
      LOG(ERROR) << "Set path topic as input is disabled!";
      return;
    }
    else {
      publisher_ = nh_.advertise<nav_msgs::Path>(topic_name_, queue_size_);
    }
    path_publisher_ = std::make_unique<PathPublisher>();
  }
}

RosStream::~RosStream()
{}

// Send solution data to ROS topic
void RosStream::solutionOutputCallback(
  std::string tag, SolutionRole role, Solution& solution)
{
  if (data_format_ == RosDataFormat::PoseStamped) {
    if (tranform_broadcaster_) {
      publishPoseWithTransform(publisher_, *tranform_broadcaster_, solution.pose, 
        ros::Time(solution.timestamp), frame_id_, subframe_id_);  
    }
    else {
      publishPoseStamped(publisher_, solution.pose, 
        ros::Time(solution.timestamp), frame_id_);
    }
  }
  else if (data_format_ == RosDataFormat::PoseWithCovarianceStamped) {
    if (tranform_broadcaster_) {
      publishPoseWithCovarianceAndTransform(publisher_, *tranform_broadcaster_, 
        solution.pose, solution.covariance.block<6, 6>(0, 0), 
        ros::Time(solution.timestamp), frame_id_, subframe_id_);
    }
    else {
      publishPoseWithCovarianceStamped(publisher_, 
        solution.pose, solution.covariance.block<6, 6>(0, 0), 
        ros::Time(solution.timestamp), frame_id_);
    }
  }
  else if (data_format_ == RosDataFormat::Path) {
    CHECK_NOTNULL(path_publisher_);
    path_publisher_->addPoseAndPublish(publisher_, solution.pose, 
      ros::Time(solution.timestamp), frame_id_);
  }
}

// Send featured image to ROS topic
void RosStream::featuredImageCallback(FramePtr& frame)
{
  if (data_format_ == RosDataFormat::Image) {
    publishFeaturedImage(publisher_, frame, ros::Time(frame->getTimestampSec()));
  }
}

// Send features as marker to ROS topic
void RosStream::mapPointCallback(MapPtr& map)
{
  if (data_format_ == RosDataFormat::Marker) {
    publishFeatures(publisher_, map, 
      ros::Time(map->getLastKeyframe()->getTimestampSec()), frame_id_);
  }
}

// ROS callbacks
void RosStream::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // Note that we set MONO8 as default, if the input image is colored image, 
  // you should add a convertion here
  cv_bridge::CvImageConstPtr ptr = 
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  cv::Mat image = ptr->image;
  
  // Call Image processor
  for (auto it_image_callback : image_callbacks_) {
    std::vector<std::string>& tags = it_image_callback.second;
    ImageCallback& callback = it_image_callback.first;
    auto it_tag = std::find(tags.begin(), tags.end(), formator_tag_);
    if (it_tag == tags.end()) continue;
    CameraRole role;
    option_tools::convert(roles_[0], role);
    callback(msg->header.stamp.toSec(), formator_tag_, role, image);
  }
}

void RosStream::imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
  // Convert IMU data
  ImuMeasurement epoch;
  epoch.timestamp = msg->header.stamp.toSec();
  epoch.linear_acceleration.x() = msg->linear_acceleration.x;
  epoch.linear_acceleration.y() = msg->linear_acceleration.y;
  epoch.linear_acceleration.z() = msg->linear_acceleration.z;
  epoch.angular_velocity.x() = msg->angular_velocity.x;
  epoch.angular_velocity.y() = msg->angular_velocity.y;
  epoch.angular_velocity.z() = msg->angular_velocity.z;
  
  // Call INS processor
  for (auto it_imu_callback : imu_callbacks_) {
    std::vector<std::string>& tags = it_imu_callback.second;
    ImuCallback& callback = it_imu_callback.first;
    auto it_tag = std::find(tags.begin(), tags.end(), formator_tag_);
    if (it_tag == tags.end()) continue;
    ImuRole role;
    option_tools::convert(roles_[0], role);
    callback(formator_tag_, role, epoch);
  }
}

// void RosStream::gnssRawCallback()
// {

// }

void RosStream::poseCallback(
  const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  // Convert data
  Solution solution;
  solution.timestamp = msg->header.stamp.toSec();
  Eigen::Vector3d p(msg->pose.pose.position.x, 
    msg->pose.pose.position.y, msg->pose.pose.position.z);
  Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, 
    msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  solution.pose = Transformation(p, q);
  for (size_t i = 0; i < 6; i++) {
    for (size_t j = 0; j < 6; j++) {
      solution.covariance(i, j) = msg->pose.covariance[i * 6 + j];
    }
  }

  // Call loosely couple estimators
  for (auto it_solution_callback : solution_callbacks_) {
    std::vector<std::string>& tags = it_solution_callback.second;
    SolutionCallback& callback = it_solution_callback.first;
    auto it_tag = std::find(tags.begin(), tags.end(), formator_tag_);
    if (it_tag == tags.end()) continue;
    SolutionRole role;
    option_tools::convert(roles_[0], role);
    callback(formator_tag_, role, solution);
  }
}

}
