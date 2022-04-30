/**
* @Function: test
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include <iostream>
#include <stdio.h>
#include <cmath>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Core>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <yaml-cpp/yaml.h>

#include "gici/stream/stream_handle.h"
#include "gici/utility/signal_handle.h"
#include "gici/utility/spin_control.h"
#include "gici/gnss/gnss_types.h"
#include "gici/gnss/rtk_estimator.h"
#include "gici/gnss/spp_estimator.h"
#include "gici/fusion/gnss_imu_lc_estimator.h"
#include "gici/gnss/gnss_common.h"
#include "gici/imu/imu_common.h"
#include "gici/ros_interface/publisher.h"

using namespace gici;

ros::Publisher gnss_pose_pub_;
ros::Publisher gnss_path_pub_;
ros::Publisher imu_raw_pub_;
ros::Publisher pose_pub_;
ros::Publisher path_pub_;
PathPublisher gnss_path_publisher_;
PathPublisher path_publisher_;

GnssMeasurement gnss_measurement_rov_;
GnssMeasurement gnss_measurement_ref_;
bool gnss_measurement_updated_rov_ = false;
bool gnss_measurement_updated_ref_ = false;
std::list<GnssSolution> gnss_solutions_;
std::unique_ptr<RtkEstimator> rtk_estimator_;
std::unique_ptr<GnssImuLcEstimator> gnss_imu_lc_estimator_;
GeoCoordinatePtr coordinate_;

void gnssCallback(GnssMeasurement& data)
{
  if (!gnss_measurement_updated_rov_ && data.role == GnssRole::Rover) {
    gnss_measurement_rov_ = data;
    gnss_measurement_updated_rov_ = true;
  }

  if (!gnss_measurement_updated_ref_ && data.role == GnssRole::Reference) {
    gnss_measurement_ref_ = data;
    gnss_measurement_updated_ref_ = true;
  }

  if (gnss_measurement_updated_rov_ && gnss_measurement_updated_ref_) {
    // For the first epoch, we add a position prior
    if (rtk_estimator_->isFirstEpoch() && 
        !SppEstimator::setCoarsePosition(gnss_measurement_rov_)) {
      gnss_measurement_updated_rov_ = false;
      gnss_measurement_updated_ref_ = false;
      return;
    }

    if (rtk_estimator_->addGnssMeasurementAndState(
        gnss_measurement_rov_, gnss_measurement_ref_)) {
      rtk_estimator_->optimize();

      gnss_solutions_.push_back(rtk_estimator_->getSolution());

      // modify covariance
      // if (gnss_solutions_.back().status == GnssSolutionStatus::Fixed) {
      //   gnss_solutions_.back().covariance.topLeftCorner(3, 3) = 
      //     Eigen::Matrix3d::Identity() * square(0.05);
      // }

      if (coordinate_ == nullptr) {
        coordinate_.reset(new GeoCoordinate(gnss_solutions_.back().position, GeoType::ECEF));
        gnss_imu_lc_estimator_->setCoordinate(coordinate_);
      }

      // // publish gnss pose
      // Eigen::Vector3d gnss_position = coordinate_->convert(
      //     gnss_solutions_.back().position, GeoType::ECEF, GeoType::ENU);
      // Transformation gnss_pose = Transformation(gnss_position, Eigen::Quaterniond::Identity());
      // ros::Time ros_time(gnss_solutions_.back().timestamp);
      // publishPoseStamped(gnss_pose_pub_, gnss_pose, ros_time, "World");

      // // publish GNSS path
      // gnss_path_publisher_.addPoseAndPublish(gnss_path_pub_, gnss_pose, ros_time, "World");
    }
    gnss_measurement_updated_rov_ = false;
    gnss_measurement_updated_ref_ = false;
  }
}

void imuCallback(ImuMeasurement& data)
{
  gnss_imu_lc_estimator_->addImuMeasurement(data);

  sensor_msgs::Imu imu_msg;
  imu_msg.header.stamp = ros::Time(data.timestamp);
  imu_msg.linear_acceleration.x = data.linear_acceleration(0);
  imu_msg.linear_acceleration.y = data.linear_acceleration(1);
  imu_msg.linear_acceleration.z = data.linear_acceleration(2);
  imu_msg.angular_velocity.x = data.angular_velocity(0);
  imu_msg.angular_velocity.y = data.angular_velocity(1);
  imu_msg.angular_velocity.z = data.angular_velocity(2);
  imu_raw_pub_.publish(imu_msg);
}

int main(int argc, char** argv)
{
  // ros interface
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;

  // ROS publishers
	gnss_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/gnss_pose", 10);
	pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/pose", 10);
	path_pub_ = nh.advertise<nav_msgs::Path>("/path", 1000);
	gnss_path_pub_ = nh.advertise<nav_msgs::Path>("/gnss_path", 1000);
  imu_raw_pub_ = nh.advertise<sensor_msgs::Imu>("/imu_raw", 100);

	// It should be declared after ros::init
	tf::TransformBroadcaster body_broadcaster;

  google::InitGoogleLogging("test");
  // FLAGS_log_dir = log_dir; 
  FLAGS_minloglevel = 0;
  FLAGS_logtostderr = true;
  FLAGS_stderrthreshold = 0;

  YAML::Node config;
  try{
     config = YAML::LoadFile("/home/cc/linux/softwares/gici/option/gic_replay.yaml");
  } catch(YAML::BadFile &e) {
    std::cout<<"read error!"<<std::endl;
    return -1;
  }

	initializeSignalHandles();

  RtkEstimatorOptions rtk_estimator_options;
  // rtk_estimator_options.use_ambiguity_resolution = false;
  rtk_estimator_ = std::make_unique<RtkEstimator>(rtk_estimator_options);

  Eigen::Vector3d lla(31, 121, 0);
  lla = GeoCoordinate::degToRad(lla);
  double gravity = earthGravity(lla);

  GnssImuLcEstimatorOptions gnss_imu_lc_estimator_options;
  gnss_imu_lc_estimator_options.verbose = true;
  gnss_imu_lc_estimator_options.window_length = 3;
  gnss_imu_lc_estimator_options.max_iteration = 5;
  gnss_imu_lc_estimator_options.imu_parameters.g = gravity;
  gnss_imu_lc_estimator_options.imu_parameters.sigma_g_c = 1.0e-4;
  gnss_imu_lc_estimator_options.imu_parameters.sigma_a_c = 2.0e-3;
  gnss_imu_lc_estimator_options.imu_parameters.sigma_gw_c = 2.1e-5;
  gnss_imu_lc_estimator_options.imu_parameters.sigma_aw_c = 8.4e-4;
  GnssImuInitializationOptions gnss_imu_initialize_options;
  gnss_imu_initialize_options.gnss_extrinsic << 0.1, -0.4, -0.4;
  gnss_imu_initialize_options.gnss_extrinsic_initial_std = 0.3;
  gnss_imu_initialize_options.verbose = true;
  gnss_imu_initialize_options.window_length_optimize = 20;
  gnss_imu_initialize_options.max_iteration = 100;
  gnss_imu_initialize_options.min_velocity = 2.0;
  gnss_imu_initialize_options.imu_parameters = gnss_imu_lc_estimator_options.imu_parameters;
  gnss_imu_lc_estimator_ = std::make_unique<GnssImuLcEstimator>(
    gnss_imu_lc_estimator_options, gnss_imu_initialize_options);

  YAML::Node stream_config = config["stream"];
  StreamHandle stream_handle(stream_config);

  StreamHandle::GnssCallback gnss_callback = std::bind(gnssCallback, std::placeholders::_1);
  stream_handle.setGnssCallback(gnss_callback);

  StreamHandle::ImuCallback imu_callback = std::bind(imuCallback, std::placeholders::_1);
  stream_handle.setImuCallback(imu_callback);

  std::ofstream outfile;
  outfile.open("/home/cc/datasets/tmp/log.txt", std::ios::out | std::ios::trunc);
  outfile.close();

	Transformation pose;

  ros::Rate loop_rate(10);
	while (ros::ok())
	{	
    // enable ros topic handlers
    ros::spinOnce();

    ros::Time ros_time = ros::Time::now();

    if (gnss_solutions_.size() > 0) {
      if (gnss_imu_lc_estimator_->addGnssMeasurementAndState(gnss_solutions_.front())) {
        gnss_imu_lc_estimator_->optimize();

        Transformation cur_pose = gnss_imu_lc_estimator_->getPoseEstimate();

        LOG(INFO) << std::fixed << std::setprecision(9) << gnss_solutions_.front().timestamp 
                  << " " << std::fixed << cur_pose.getPosition().transpose();

        if (gnss_solutions_.size() > 1) {
          // // publish body pose and transform
          // publishPoseWithTransform(pose_pub_, body_broadcaster, cur_pose, ros_time, "Body", "World");

          // // publish path
          // path_publisher_.addPoseAndPublish(path_pub_, cur_pose, ros_time, "World");
        }

        // publish gnss pose
        Eigen::Vector3d gnss_position = coordinate_->convert(
            gnss_solutions_.front().position, GeoType::ECEF, GeoType::ENU);
        Transformation gnss_pose = Transformation(gnss_position, Eigen::Quaterniond::Identity());
        publishPoseStamped(gnss_pose_pub_, gnss_pose, ros_time, "World");

        // publish GNSS path
        gnss_path_publisher_.addPoseAndPublish(gnss_path_pub_, gnss_pose, ros_time, "World");

        uint8_t buff[256];
        sol_t sol;
        Eigen::Vector3d position = coordinate_->convert(cur_pose.getPosition(), GeoType::ENU, GeoType::ECEF);
        for (int i = 0; i < 3; i++) sol.rr[i] = position(i);
        sol.time = gnss_common::doubleToGtime(gnss_measurement_rov_.timestamp);
        sol.time = utc2gpst(sol.time);
        GnssSolutionStatus status = gnss_solutions_.front().status;
        if (status == GnssSolutionStatus::Fixed) sol.stat = SOLQ_FIX;
        else if (status == GnssSolutionStatus::Float) sol.stat = SOLQ_FLOAT;
        else sol.stat = SOLQ_DGPS;
        int size = outnmea_rmc(buff, &sol);
        outnmea_gga(buff + size, &sol);
        outfile.open("/home/cc/datasets/tmp/log.txt", std::ios::out | std::ios::app);
        outfile << buff;
        outfile.close();
      }
      else {
        LOG(INFO) << "Failed to add measurement on LC estimator!";
      }
      gnss_solutions_.pop_front();
      LOG(INFO) << "GNSS solutions pendding " << gnss_solutions_.size();
    }
    
    if (gnss_solutions_.size() == 0) {
      // publish body pose and transform
      pose = gnss_imu_lc_estimator_->getPoseIntegrated();
      publishPoseWithTransform(pose_pub_, body_broadcaster, pose, ros_time, "World", "Imu");

      // publish path
      path_publisher_.addPoseAndPublish(path_pub_, pose, ros_time, "World");
    }

    if (gnss_imu_lc_estimator_->getTimeIntegrated() > 1643523550.0) break;

		loop_rate.sleep();
	}

	return 0;
}

