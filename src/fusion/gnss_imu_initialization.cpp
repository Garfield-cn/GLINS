/**
* @Function: GNSS/IMU initialization
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/fusion/gnss_imu_initialization.h"
#include "gici/gnss/position_error.h"
#include "gici/gnss/gnss_parameter_blocks.h"
#include "gici/gnss/gnss_relative_errors.h"
#include "gici/imu/imu_common.h"
#include "gici/imu/imu_error.hpp"
#include "gici/utility/common.h"
#include "gici/optimizer/pose_parameter_block.hpp"
#include "gici/optimizer/speed_and_bias_parameter_block.hpp"
#include "gici/imu/speed_and_bias_error.hpp"
#include "gici/optimizer/pose_error.hpp"
#include "gici/imu/yaw_error.h"
#include "gici/imu/roll_and_pitch_error.h"

namespace gici {


// The default constructor
GnssImuInitialization::GnssImuInitialization(
  const GnssImuInitializationOptions& options) :
  options_(options), graph_ptr_(std::make_shared<Graph>()),
  cauchy_loss_function_ptr_(new ceres::CauchyLoss(1)),
  huber_loss_function_ptr_(new ceres::HuberLoss(1)), 
  zero_motion_finished_(false), finished_(false), coordinate_(nullptr)
{}

GnssImuInitialization::GnssImuInitialization(
    const GnssImuInitializationOptions& options, 
    const std::shared_ptr<Graph>& graph_ptr) :
  options_(options), graph_ptr_(graph_ptr),
  cauchy_loss_function_ptr_(new ceres::CauchyLoss(1)),
  huber_loss_function_ptr_(new ceres::HuberLoss(1)), 
  zero_motion_finished_(false), finished_(false), coordinate_(nullptr)
{}

// The default destructor
GnssImuInitialization::~GnssImuInitialization()
{}

// Add GNSS measurements 
bool GnssImuInitialization::addGnssMeasurement(const GnssSolution& gnss_solution)
{
  // Check if we have already finished initialization
  if (finished_) return true;

  // Store measurements
  gnss_solutions_.push_back(gnss_solution);
  if (gnss_solutions_.size() > options_.window_length_optimize) {
    gnss_solutions_.pop_front();
  }
  // we do not have enough data now
  else return false;

  // Check if we have finished the first step
  if (!zero_motion_finished_) return false;

  // Determine initial yaw angle and improve the accuracy of other intial values
  // Check if we can start step 2 initialization
  // we need 1/3 of the motions in the window has velocity larger than min_velocity
  int valid_cnt = 0;
  bool has_velocity = false;
  if (checkZero(gnss_solutions_.front().velocity) && 
      checkZero(gnss_solutions_.back().velocity)) {
    // we do not have velocity measurement
    for (size_t i = 1; i < gnss_solutions_.size(); i++) {
      if (gnss_solutions_[i].status != gnss_solutions_[i - 1].status) {
        // avoid crazy jump
        continue;
      }
      double dt = gnss_solutions_[i].timestamp - gnss_solutions_[i - 1].timestamp;
      Eigen::Vector3d velocity = 
        (gnss_solutions_[i].position - gnss_solutions_[i - 1].position) / dt;
      if (velocity.norm() > options_.min_velocity) {
        valid_cnt++;
      }
    }
  }
  else {
    has_velocity = true;
    for (size_t i = 0; i < gnss_solutions_.size(); i++) {
      if (gnss_solutions_[i].velocity.norm() > options_.min_velocity) {
        valid_cnt++;
      }
    }
  }
  // if (valid_cnt < options_.window_length_optimize / 2 || valid_cnt < 1) {
  if (valid_cnt < 1) {
    LOG(INFO) << "Insufficient data. Current data length is " << options_.window_length_optimize
              << ". Number of epochs with velocity larger than " << options_.min_velocity
              << " is " << valid_cnt;
    return false;
  }

  // check coordinate
  if (coordinate_ == nullptr) {
    LOG(ERROR) << "Coordinate not setted!";
    return false;
  }

  // Start initialization
  LOG(INFO) << "Start initialization.";
  for (size_t i = 0; i < gnss_solutions_.size(); i++) 
  {
    auto& cur_gnss = gnss_solutions_[i];
    Eigen::Vector3d cur_position = coordinate_->convert(
      cur_gnss.position, GeoType::ECEF, GeoType::ENU);

    // Get prior
    double timestamp = cur_gnss.timestamp;
    Eigen::Vector3d t_SR_S = options_.gnss_extrinsic;

    // get initial pose
    bool has_pose = false;
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    if (!has_velocity && i != 0 && 
        gnss_solutions_[i].status == gnss_solutions_[i - 1].status) {
      double dt = gnss_solutions_[i].timestamp - gnss_solutions_[i - 1].timestamp;
      Eigen::Vector3d p_0 = coordinate_->convert(
        gnss_solutions_[i - 1].position, GeoType::ECEF, GeoType::ENU);
      Eigen::Vector3d p_1 = coordinate_->convert(
        gnss_solutions_[i].position, GeoType::ECEF, GeoType::ENU);
      velocity = (p_1 - p_0) / dt;
    }
    else if (has_velocity) {
      velocity = gnss_solutions_[i].velocity;
    }
    bool found_angular_velocity = false;
    Eigen::Vector3d angular_velocity;
    for (auto imu : imu_measurements_) {
      if (checkEqual(imu.timestamp, timestamp, 0.05)) {
        angular_velocity = imu.angular_velocity;
        found_angular_velocity = true;
        break;
      }
    }
    if (velocity.norm() > options_.min_velocity && 
        found_angular_velocity && fabs(angular_velocity.norm()) < 0.1) {
      // initialize using velocity
      double yaw;
      initYawFromVelocity(velocity, yaw);

      Eigen::Vector3d ypr_0 = T_WS_0_.getEigenQuaternion().matrix().eulerAngles(2,1,0);
      Eigen::Quaterniond quat_coarse = 
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * 
        Eigen::AngleAxisd(ypr_0(1), Eigen::Vector3d::UnitY()) * 
        Eigen::AngleAxisd(ypr_0(2), Eigen::Vector3d::UnitX());
      T_WS_0_ = Transformation(T_WS_0_.getPosition(), quat_coarse);
    }

    // Add parameter blocks in current timestamp
    // Pose block
    BackendId pose_id = createGnssPoseId(cur_gnss.id);
    std::shared_ptr<PoseParameterBlock> pose_parameter_block = 
      std::make_shared<PoseParameterBlock>(T_WS_0_, pose_id.asInteger());
    if (!graph_ptr_->addParameterBlock(pose_parameter_block, Graph::Pose6d)) {
      return false;
    }

    // Speed and bias block
    BackendId speed_and_bias_id = changeIdType(pose_id, IdType::ImuStates);
    std::shared_ptr<SpeedAndBiasParameterBlock> speed_and_bias_parameter_block = 
      std::make_shared<SpeedAndBiasParameterBlock>(
      speed_and_bias_0_, speed_and_bias_id.asInteger());
    if (!graph_ptr_->addParameterBlock(speed_and_bias_parameter_block)) {
      return false;
    }

    // Relative position between GNSS and IMU
    BackendId gnss_extrinsic_id = changeIdType(pose_id, IdType::gExtrinsics);
    std::shared_ptr<PositionParameterBlock> gnss_extrinsic_parameter_block = 
      std::make_shared<PositionParameterBlock>(t_SR_S, gnss_extrinsic_id.asInteger());
    if (!graph_ptr_->addParameterBlock(gnss_extrinsic_parameter_block)) {
      return false;
    }

    // Add GNSS error
    std::shared_ptr<PositionError<7, 3>> position_error = 
      std::make_shared<PositionError<7, 3>>(gnss_solutions_[i].position, 
      gnss_solutions_[i].covariance.topLeftCorner(3, 3).inverse());
    position_error->setCoordinate(coordinate_);
    graph_ptr_->addResidualBlock(position_error, 
      // cauchy_loss_function_ptr_ ? cauchy_loss_function_ptr_.get() : nullptr,
      huber_loss_function_ptr_ ? huber_loss_function_ptr_.get() : nullptr,
      // nullptr, 
      graph_ptr_->parameterBlockPtr(pose_id.asInteger()),
      graph_ptr_->parameterBlockPtr(gnss_extrinsic_id.asInteger()));

    // Add IMU error
    double last_timestamp;
    BackendId last_pose_id;
    if (i != 0) {
      last_timestamp = gnss_solutions_[i - 1].timestamp;
      last_pose_id = createGnssPoseId(gnss_solutions_[i - 1].id);

      BackendId last_speed_and_bias_id = changeIdType(last_pose_id, IdType::ImuStates);
      BackendId speed_and_bias_id = changeIdType(pose_id, IdType::ImuStates);
      std::shared_ptr<ImuError> imu_error =
        std::make_shared<ImuError>(imu_measurements_, options_.imu_parameters,
                                  last_timestamp, timestamp);
      graph_ptr_->addResidualBlock(imu_error, nullptr, 
        graph_ptr_->parameterBlockPtr(last_pose_id.asInteger()), 
        graph_ptr_->parameterBlockPtr(last_speed_and_bias_id.asInteger()), 
        graph_ptr_->parameterBlockPtr(pose_id.asInteger()), 
        graph_ptr_->parameterBlockPtr(speed_and_bias_id.asInteger()));
    }

    // GNSS extrinsic
    if (i != 0) {
      BackendId last_gnss_extrinsic_id = changeIdType(last_pose_id, IdType::gExtrinsics);
      Eigen::Matrix3d relative_extrinsic_information = 
        Eigen::Matrix3d::Identity() * square(1.0 / options_.gnss_relative_extrinsic_std);
      std::shared_ptr<RelativePositionError> relative_extrinsic_error = 
        std::make_shared<RelativePositionError>(relative_extrinsic_information);
      graph_ptr_->addResidualBlock(relative_extrinsic_error, nullptr,
        graph_ptr_->parameterBlockPtr(last_gnss_extrinsic_id.asInteger()),
        graph_ptr_->parameterBlockPtr(gnss_extrinsic_id.asInteger()));
    }

    // Initial errors
    if (i == 0) {
      // Initial GNSS extrinsic error
      std::shared_ptr<PositionError<3>> extrinsic_error = 
        std::make_shared<PositionError<3>>(options_.gnss_extrinsic, 
        Eigen::Matrix3d::Identity() * square(1.0 / options_.gnss_extrinsic_initial_std));
      graph_ptr_->addResidualBlock(extrinsic_error, nullptr,
        graph_ptr_->parameterBlockPtr(gnss_extrinsic_id.asInteger()));

      // Initial bias error
      std::shared_ptr<SpeedAndBiasError> speed_and_bias_error = 
        std::make_shared<SpeedAndBiasError>(speed_and_bias_0_, 
        square(options_.min_velocity * 2.0), 
        square(options_.imu_parameters.sigma_bg), 
        square(options_.imu_parameters.sigma_ba));
      graph_ptr_->addResidualBlock(speed_and_bias_error, nullptr,
        graph_ptr_->parameterBlockPtr(speed_and_bias_id.asInteger()));
    }
  }

  // Optimize
  graph_ptr_->options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  graph_ptr_->options.trust_region_strategy_type = ceres::DOGLEG;
  graph_ptr_->options.num_threads = options_.num_threads;
  graph_ptr_->options.max_num_iterations = options_.max_iteration;

  // For debug
  // debug_callback_.reset(new CeresDebugCallback());
  // graph_ptr_->options.callbacks.push_back(debug_callback_.get());

  if (options_.verbose) {
    // graph_ptr_->options.logging_type = ceres::LoggingType::SILENT;
    graph_ptr_->options.minimizer_progress_to_stdout = true;
  }
  else {
    graph_ptr_->options.logging_type = ceres::LoggingType::SILENT;
    graph_ptr_->options.minimizer_progress_to_stdout = false;
  }

  // call solver
  graph_ptr_->solve();

  // flag
  finished_ = true;

  if (options_.verbose) {
    LOG(INFO) << graph_ptr_->summary.BriefReport();
  }

  return true;
}

// Add IMU measurement
void GnssImuInitialization::addImuMeasurement(const ImuMeasurement& imu_measurement)
{
  if (finished_) return;

  if (imu_measurements_.size() != 0 && 
      imu_measurements_.back().timestamp > imu_measurement.timestamp) {
    LOG(WARNING) << "Received IMU with previous timestamp!";
  }
  else {
    imu_measurements_.push_back(imu_measurement);
  }

  // Under zero motion, we determine intial pitch, roll, and anguler rate bias
  double timestamp_end = imu_measurements_.back().timestamp;
  double timestamp_start = timestamp_end - options_.time_window_length_zero_motion;
  if (!zero_motion_finished_ && 
      timestamp_start >= imu_measurements_.front().timestamp && 
      initPoseAndBiases(imu_measurements_, options_.imu_parameters.g, 
      T_WS_0_, speed_and_bias_0_)) {
    zero_motion_finished_ = true;
  }

  while (imu_measurements_.front().timestamp < gnss_solutions_.front().timestamp - 0.1) {
    imu_measurements_.pop_front();
  }
}

// Get initialization result
bool GnssImuInitialization::getResult(
                 Transformation& T_WS, Eigen::Matrix<double, 7, 7>& cov_T_WS,
                 SpeedAndBias& speed_and_bias, 
                 Eigen::Matrix<double, 9, 9>& cov_speed_and_bias,
                 Eigen::Vector3d& t_SR_S, Eigen::Matrix3d& cov_t_SR_S, 
                 bool compute_covariance)
{
  if (!finished_) return false;

  // pose
  BackendId pose_id = createGnssPoseId(gnss_solutions_.back().id);
  CHECK(graph_ptr_->parameterBlockExists(pose_id.asInteger()));
  std::shared_ptr<PoseParameterBlock> pose_ptr = 
    std::dynamic_pointer_cast<PoseParameterBlock>(
    graph_ptr_->parameterBlockPtr(pose_id.asInteger()));
  CHECK(pose_ptr != nullptr);
  T_WS = pose_ptr->estimate();

  // speed and bias
  BackendId speed_and_bias_id = changeIdType(pose_id, IdType::ImuStates);
  CHECK(graph_ptr_->parameterBlockExists(speed_and_bias_id.asInteger()));
  std::shared_ptr<SpeedAndBiasParameterBlock> speed_and_bias_ptr = 
    std::dynamic_pointer_cast<SpeedAndBiasParameterBlock>(
    graph_ptr_->parameterBlockPtr(speed_and_bias_id.asInteger()));
  CHECK(speed_and_bias_ptr != nullptr);
  speed_and_bias = speed_and_bias_ptr->estimate();

  // extrinsic
  BackendId gnss_extrinsic_id = changeIdType(pose_id, IdType::gExtrinsics);
  CHECK(graph_ptr_->parameterBlockExists(gnss_extrinsic_id.asInteger()));
  std::shared_ptr<PositionParameterBlock> gnss_extrinsic_ptr = 
    std::dynamic_pointer_cast<PositionParameterBlock>(
    graph_ptr_->parameterBlockPtr(gnss_extrinsic_id.asInteger()));
  CHECK(gnss_extrinsic_ptr != nullptr);
  t_SR_S = gnss_extrinsic_ptr->estimate();

  if (compute_covariance)
  {
    // pose covariance
    Eigen::MatrixXd cov_T_WS_0;
    graph_ptr_->computeCovariance({pose_id.asInteger()}, cov_T_WS_0);
    CHECK_EQ(cov_T_WS_0.cols(), cov_T_WS.cols());
    cov_T_WS = cov_T_WS_0;

    // speed and bias covariance
    Eigen::MatrixXd cov_speed_and_bias_0;
    graph_ptr_->computeCovariance(
      {speed_and_bias_id.asInteger()}, cov_speed_and_bias_0);
    CHECK_EQ(cov_speed_and_bias_0.cols(), cov_speed_and_bias.cols());
    cov_speed_and_bias = cov_speed_and_bias_0;

    // extrinsic covariance
    Eigen::MatrixXd cov_t_SR_S_0;
    graph_ptr_->computeCovariance(
      {gnss_extrinsic_id.asInteger()}, cov_t_SR_S_0);
    CHECK_EQ(cov_t_SR_S_0.cols(), cov_t_SR_S.cols());
    cov_t_SR_S = cov_t_SR_S_0;
  }

  {
    std::ofstream outfile;
    outfile.open("/home/cc/datasets/tmp/log.txt", std::ios::out | std::ios::trunc);
    for (size_t i = 0; i < gnss_solutions_.size(); i++) {
      BackendId pose_id = createGnssPoseId(gnss_solutions_[i].id);
      CHECK(graph_ptr_->parameterBlockExists(pose_id.asInteger()));
      std::shared_ptr<PoseParameterBlock> pose_ptr = 
        std::dynamic_pointer_cast<PoseParameterBlock>(
        graph_ptr_->parameterBlockPtr(pose_id.asInteger()));
      CHECK(pose_ptr != nullptr);
      T_WS = pose_ptr->estimate();

      outfile << std::fixed << std::setw(8) << std::setprecision(4) 
        << T_WS.getPosition().transpose() << " " << coordinate_->convert(
        gnss_solutions_[i].position, GeoType::ECEF, GeoType::ENU).transpose() << " "
        << T_WS.getRotation().vector().transpose() << std::endl;
    }
    outfile.close();
  }

  // {
  //   std::ofstream outfile;
  //   outfile.open("/home/cc/datasets/tmp/log2.txt", std::ios::out | std::ios::trunc);
  //   outfile << graph_ptr_->summary.BriefReport() << std::endl;
  //   for (auto residual_map : graph_ptr_->residual_block_id_to_residual_block_spec_map_) {
  //     auto residual = residual_map.first;
  //     auto& error_interface_ptr = residual_map.second.error_interface_ptr;
  //     size_t size = residual_map.second.error_interface_ptr->residualDim();

  //     Eigen::VectorXd Residuals = Eigen::VectorXd::Zero(size);
  //     graph_ptr_->problem_->EvaluateResidualBlock(
  //         residual, false, nullptr, Residuals.data(), nullptr);
  //     // if (Residuals.maxCoeff() > 1.0 || Residuals.minCoeff() < -1.0)
  //     // if (size > 2)
  //       outfile << "Residual " << static_cast<int>(error_interface_ptr->typeInfo()) << ": " 
  //               << residual << ": " << Residuals.transpose() << std::endl;

  //       // auto parameters = graph_ptr_->parameters(residual);
  //       // for (Graph::ParameterBlockSpec &parameter : parameters) {
  //       //   outfile << "    Parameter: " << parameter.rov << std::endl;
  //       // }
  //   }
  //   outfile.close();
  // }

  // std::cout << speed_and_bias.transpose() << std::endl;
  // std::cout << t_SR_S.transpose() << std::endl;

  // // LOG(FATAL) << "end";

  return true;
}

// Marginalize the used measurements to a given window size 
bool GnssImuInitialization::marginalization(const int window_length,
            const std::shared_ptr<MarginalizationError>& marginalization_ptr,
            std::deque<GnssSolution>& left_gnss_solutions, 
            ceres::ResidualBlockId& marginalization_residual_id)
{
  if (!finished_) return false;

  // check if we need apply marginalization
  if (gnss_solutions_.size() < window_length) {
    left_gnss_solutions = gnss_solutions_;
    return true;
  }

  // these will keep track of what we want to marginalize out.
  std::vector<uint64_t> parameter_blocks_to_be_marginalized;
  std::vector<bool> keep_parameter_blocks;

  // Fill marginalization terms
  left_gnss_solutions.clear();
  std::deque<GnssSolution>::reverse_iterator it_gnss = gnss_solutions_.rbegin();
  for (int size = 0; it_gnss != gnss_solutions_.rend(); it_gnss++, size++)
  {
    GnssSolution& gnss_solution = *it_gnss;
    if (size < window_length - 1) {
      left_gnss_solutions.push_front(gnss_solution);
      continue;
    }

    // Pose parameter
    BackendId pose_id = createGnssPoseId(gnss_solution.id);
    if (graph_ptr_->parameterBlockExists(pose_id.asInteger())) {
      parameter_blocks_to_be_marginalized.push_back(pose_id.asInteger());
      keep_parameter_blocks.push_back(false);

      // Get all residuals connected to this state.
      Graph::ResidualBlockCollection residuals = 
        graph_ptr_->residuals(pose_id.asInteger());
      for (size_t r = 0; r < residuals.size(); ++r) {
        marginalization_ptr->addResidualBlock(
              residuals[r].residual_block_id);
      }
    }

    // Speed and bias parameter
    BackendId speed_and_bias_id = changeIdType(pose_id, IdType::ImuStates);
    if (graph_ptr_->parameterBlockExists(speed_and_bias_id.asInteger())) {
      parameter_blocks_to_be_marginalized.push_back(speed_and_bias_id.asInteger());
      keep_parameter_blocks.push_back(false);

      // Get all residuals connected to this state.
      Graph::ResidualBlockCollection residuals = 
        graph_ptr_->residuals(speed_and_bias_id.asInteger());
      for (size_t r = 0; r < residuals.size(); ++r) {
        marginalization_ptr->addResidualBlock(
              residuals[r].residual_block_id);
      }
    }

    // GNSS Extrinsic parameter
    BackendId gnss_extrinsic_id = changeIdType(pose_id, IdType::gExtrinsics);
    if (graph_ptr_->parameterBlockExists(gnss_extrinsic_id.asInteger())) {
      parameter_blocks_to_be_marginalized.push_back(gnss_extrinsic_id.asInteger());
      keep_parameter_blocks.push_back(false);

      // Get all residuals connected to this state.
      Graph::ResidualBlockCollection residuals = 
        graph_ptr_->residuals(gnss_extrinsic_id.asInteger());
      for (size_t r = 0; r < residuals.size(); ++r) {
        marginalization_ptr->addResidualBlock(
              residuals[r].residual_block_id);
      }
    }
  }

  // Apply marginalization
  marginalization_ptr->marginalizeOut(parameter_blocks_to_be_marginalized,
                           keep_parameter_blocks);

  // update error computation
  if(parameter_blocks_to_be_marginalized.size() > 0) {
    marginalization_ptr->updateErrorComputation();
  }                              

  if (marginalization_ptr)
  {
    std::vector<std::shared_ptr<ParameterBlock> > parameter_block_ptrs;
    marginalization_ptr->getParameterBlockPtrs(parameter_block_ptrs);
    marginalization_residual_id = graph_ptr_->addResidualBlock(
          marginalization_ptr, nullptr, parameter_block_ptrs);
    CHECK(marginalization_residual_id)
        << "could not add marginalization error";
    if (!marginalization_residual_id)
    {
      return false;
    }
  }

  return true;
}


}