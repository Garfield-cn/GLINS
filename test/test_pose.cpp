/**
* @Function: Test
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/estimate/graph.h"
#include "gici/estimate/pose_parameter_block.h"
#include "gici/estimate/pose_error.h"
#include "gici/imu/yaw_error.h"
#include "gici/imu/roll_and_pitch_error.h"
#include "gici/estimate/estimator_types.h"

using namespace gici;

int main(void)
{
  google::InitGoogleLogging("test");
  // FLAGS_log_dir = log_dir; 
  FLAGS_minloglevel = 0;
  FLAGS_logtostderr = true;
  FLAGS_stderrthreshold = 0;

  Eigen::Quaterniond q_pr(0.678, -0.031, -0.734, 0.0); q_pr.normalize();
  Eigen::Quaterniond q_y(0.47, 0.0, 0.0, 0.88); q_y.normalize();

  Eigen::Vector3d ypr0 = q_pr.matrix().eulerAngles(2,1,0);
  Eigen::Vector3d ypr1 = q_y.matrix().eulerAngles(2,1,0); // 2.156
  Eigen::Quaterniond quatA = Eigen::AngleAxisd(ypr1(0), Eigen::Vector3d::UnitZ()) * 
                             Eigen::AngleAxisd(ypr0(1), Eigen::Vector3d::UnitY()) * 
                             Eigen::AngleAxisd(ypr0(2), Eigen::Vector3d::UnitX());

  Eigen::Quaterniond quaty = Eigen::AngleAxisd(ypr1(0), Eigen::Vector3d::UnitZ()) * 
                             Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) * 
                             Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
  Eigen::Quaterniond quatp = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) * 
                             Eigen::AngleAxisd(ypr0(1), Eigen::Vector3d::UnitY()) * 
                             Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
  Eigen::Quaterniond quatr = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) * 
                             Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) * 
                             Eigen::AngleAxisd(ypr0(2), Eigen::Vector3d::UnitX());    
  Eigen::Quaterniond quatpr = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) * 
                             Eigen::AngleAxisd(ypr0(1), Eigen::Vector3d::UnitY()) * 
                             Eigen::AngleAxisd(ypr0(2), Eigen::Vector3d::UnitX());    
  Eigen::Quaterniond quatpr_inv = quatpr.inverse();
  Eigen::Quaterniond quatB = quaty * quatp * quatr;
  Eigen::Quaterniond quatC = quaty * quatpr;
  
  std::shared_ptr<Graph> graph_ptr_ = std::make_shared<Graph>();
  
  BackendId pose_id = createGnssPoseId(1);
  std::shared_ptr<PoseParameterBlock> pose_parameter_block = 
    std::make_shared<PoseParameterBlock>(
    Transformation(Eigen::Vector3d::Zero(), q_pr), pose_id.asInteger());
  if (!graph_ptr_->addParameterBlock(pose_parameter_block, Graph::Pose6d)) {
    return false;
  }

  {
    // Transformation T_WS_y(Eigen::Vector3d::Zero(), quatA);
    // Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
    // information(5, 5) = 1.0; 
    // std::shared_ptr<PoseError> yaw_pose_error = 
    //   std::make_shared<PoseError>(T_WS_y, information);
    // graph_ptr_->addResidualBlock(yaw_pose_error, nullptr,
    //   graph_ptr_->parameterBlockPtr(pose_id.asInteger()));
    std::shared_ptr<YawError> yaw_error = 
      std::make_shared<YawError>(ypr1(0), 1.0e2);
    graph_ptr_->addResidualBlock(yaw_error, nullptr,
      graph_ptr_->parameterBlockPtr(pose_id.asInteger()));
  }

  {
    // Transformation T_WS_rp = Transformation(Eigen::Vector3d::Zero(), q_pr);
    // Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity() * 1e-12;
    // information(3, 3) = 1.0e2; 
    // information(4, 4) = 1.0e2;
    // std::shared_ptr<PoseError> pose_error = 
    //   std::make_shared<PoseError>(T_WS_rp, information);
    // graph_ptr_->addResidualBlock(pose_error, nullptr,
    //   graph_ptr_->parameterBlockPtr(pose_id.asInteger()));
    Eigen::Vector2d roll_pitch(ypr0(2), ypr0(1));
    Eigen::Matrix2d information = Eigen::Matrix2d::Identity() * 1e2;
    std::shared_ptr<RollAndPitchError> roll_and_pitch_error = 
      std::make_shared<RollAndPitchError>(roll_pitch, information);
    graph_ptr_->addResidualBlock(roll_and_pitch_error, nullptr,
      graph_ptr_->parameterBlockPtr(pose_id.asInteger()));
  }

  {
    Transformation T_WS_position = Transformation(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
    information.topLeftCorner(3, 3) = Eigen::Matrix3d::Identity() * 1.0e2;
    std::shared_ptr<PoseError> pose_error = 
      std::make_shared<PoseError>(T_WS_position, information);
    graph_ptr_->addResidualBlock(pose_error, nullptr,
      graph_ptr_->parameterBlockPtr(pose_id.asInteger()));
  }

  graph_ptr_->options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
  graph_ptr_->options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  graph_ptr_->options.max_num_iterations = 10;
  graph_ptr_->options.logging_type = ceres::LoggingType::SILENT;
  graph_ptr_->options.minimizer_progress_to_stdout = true;

  graph_ptr_->solve();

  LOG(INFO) << graph_ptr_->summary.BriefReport();

  CHECK(graph_ptr_->parameterBlockExists(pose_id.asInteger()));
  std::shared_ptr<PoseParameterBlock> pose_ptr = 
    std::dynamic_pointer_cast<PoseParameterBlock>(
    graph_ptr_->parameterBlockPtr(pose_id.asInteger()));
  CHECK(pose_ptr != nullptr);
  Transformation T_WS = pose_ptr->estimate();

  Eigen::MatrixXd cov_T_WS;
  graph_ptr_->computeCovariance({pose_id.asInteger()}, cov_T_WS);
  Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
  Eigen::Matrix<double, 7, 1> parameters;
  parameters << T_WS.getPosition().x(), T_WS.getPosition().y(), T_WS.getPosition().z(),
    T_WS.getRotation().x(), T_WS.getRotation().y(), T_WS.getRotation().z(), T_WS.getRotation().w();
  PoseLocalParameterization::liftJacobian(parameters.data(), J_lift.data());
  Eigen::MatrixXd cov_T_WS_minimal = J_lift * cov_T_WS * J_lift.transpose();
  std::cout << cov_T_WS_minimal.bottomRightCorner(3, 3) << std::endl;

  return 0;
}