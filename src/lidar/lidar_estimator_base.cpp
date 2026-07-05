/**
 * @Function: Base class for LiDAR estimators
 *
 * @Author  : Jiahui Liu
 * @Email   : jh.liu@sjtu.edu.cn
 **/
#include "gici/lidar/lidar_estimator_base.h"
#include "gici/estimate/pose_parameter_block.h"

namespace gici {

// The default constructor
LidarEstimatorBase::LidarEstimatorBase(const LidarEstimatorBaseOptions& options,
                                       const EstimatorBaseOptions& base_options) :
  EstimatorBase(base_options),
  lidar_base_options_(options),
  scan_to_map_W_(new Cloud),
  local_map_(new Cloud)
{
  loss_function_ = new ceres::CauchyLoss(2 * sqrt(1 / options.var));
}

// The default destructor
LidarEstimatorBase::~LidarEstimatorBase()
{}

// Add the LiDAR-to-body extrinsic parameter block to the graph
BackendId LidarEstimatorBase::addLidarExtrinsicsParameterBlock(const int32_t id,
                                                               const Transformation& T_BL_prior,
                                                               const bool if_estimate_extrinsics)
{
  BackendId pose_id = createLidarPoseId(id);
  BackendId lidar_extrinsic_id = changeIdType(pose_id, IdType::lExtrinsics);
  std::shared_ptr<PoseParameterBlock> lidar_extrinsic_parameter_block =
      std::make_shared<PoseParameterBlock>(T_BL_prior, lidar_extrinsic_id.asInteger());
  CHECK(graph_->addParameterBlock(lidar_extrinsic_parameter_block));

  // Keep the prior fixed unless online LiDAR/IMU calibration is enabled
  if (!if_estimate_extrinsics) {
    graph_->setParameterBlockConstant(lidar_extrinsic_id.asInteger());
  }
  return lidar_extrinsic_id;
}

void LidarEstimatorBase::addLidarResidualMarginBlocks(const State& state)
{
  CHECK(graph_->parameterBlockExists(state.id_in_graph.asInteger()));
  Graph::ResidualBlockCollection residuals = graph_->residuals(state.id_in_graph.asInteger());
  for (size_t r = 0; r < residuals.size(); ++r) {
    if (residuals[r].error_interface_ptr->typeInfo() != ErrorType::kRegistrationError &&
        residuals[r].error_interface_ptr->typeInfo() != ErrorType::kPlaneError)
      continue;
    marginalization_error_->addResidualBlock(residuals[r].residual_block_id);
  }
}

void LidarEstimatorBase::erasePlaneErrorResidualBlock(ceres::ResidualBlockId residual_block_id)
{
  const Graph::ParameterBlockCollection parameters = graph_->parameters(residual_block_id);
  const BackendId id(parameters.at(1).first);
  CHECK(id.type() == IdType::lLandmark);
  // Remove observation from landmark map
  MapPlane& map_plane = landmarks_map_.at(id);
  for (auto it = map_plane.observations.begin(); it != map_plane.observations.end();) {
    if (it->second == uint64_t(residual_block_id)) {
      it = map_plane.observations.erase(it);
    } else {
      ++it;
    }
  }
  // Remove residual block
  graph_->removeResidualBlock(residual_block_id);
}

// Erase landmark observations at a given state
void LidarEstimatorBase::erasePlaneErrorResidualBlocks(const State& state)
{
  Graph::ResidualBlockCollection residuals = graph_->residuals(state.id_in_graph.asInteger());
  for (size_t r = 0; r < residuals.size(); ++r) {
    if (residuals[r].error_interface_ptr->typeInfo() != ErrorType::kPlaneError) continue;

    erasePlaneErrorResidualBlock(residuals[r].residual_block_id);
  }
}

void LidarEstimatorBase::eraseRegistrationErrorResidualBlocks(const State& state)
{
  Graph::ResidualBlockCollection residuals = graph_->residuals(state.id_in_graph.asInteger());
  for (size_t r = 0; r < residuals.size(); ++r) {
    if (residuals[r].error_interface_ptr->typeInfo() != ErrorType::kRegistrationError) continue;

    graph_->removeResidualBlock(residuals[r].residual_block_id);
  }
}

void LidarEstimatorBase::selectKeyFrame(const ScanPtr& scan, const Transformation T)
{
  if (last_keyframe_ == nullptr) {
    scan->is_keyframe = true;
    last_keyframe_ = scan;
    last_keyframe_T_ = T;
  }

  const double a =
      Quaternion::log(T.getRotation() * last_keyframe_T_.getRotation().inverse()).norm() * 180 /
      M_PI;
  const double d = (T.getPosition() - last_keyframe_T_.getPosition()).norm();
  if (!(a < 10 && d < 10)) {
    scan->is_keyframe = true;
    last_keyframe_ = scan;
    last_keyframe_T_ = T;
  }

  const double dt = scan->timefinal - last_keyframe_->timefinal;
  if (dt > lidar_base_options_.kfselect_min_dt) {
    scan->is_keyframe = true;
    last_keyframe_ = scan;
    last_keyframe_T_ = T;
  }
}

void LidarEstimatorBase::addGlobalRegistrationErrorResidualBlocks(const State& cur_state,
                                                                  const ScanPtr& scan,
                                                                  bool need_tf_body)
{
  Eigen::Vector4d plane_params;
  if (need_tf_body) {
    // Transform from LiDAR to body using the extrinsic prior
    Cloud_ptr cloud_b(new Cloud);
    Cloud_ptr cloud_w(new Cloud);

    pcl::transformPointCloud(*scan->cloud_ptr, *cloud_b, transTomat(lidar_base_options_.T_B_L),
                             true);
    pcl::transformPointCloud(*scan->cloud_ptr, *cloud_w,
                             transTomat(getPoseEstimate(cur_state) * lidar_base_options_.T_B_L),
                             true);

    // Coarse search, and add residual blocks
    for (size_t i = 0; i < scan->feature_indices.size(); i++) {
      auto idx = scan->feature_indices[i];
      if (!tree_handler_->getGlobalTree()->search(cloud_w->points[idx], plane_params)) {
        continue;
      }

      if (!addGlobalRegistrationErrorResidualBlocks(cur_state, cloud_b->points[idx],
                                                    plane_params)) {
        continue;
      }
    }
  }
}

void LidarEstimatorBase::addPlaneParameterBlocksWithResiduals(const State& cur_state,
                                                              const ScanPtr& scan)
{
  Eigen::Vector4d plane_params;
  // Transform from LiDAR to body using the extrinsic prior
  Cloud_ptr cloud_b(new Cloud);
  Cloud_ptr cloud_w(new Cloud);
  Cloud_ptr residual_cloud(new Cloud);

  pcl::transformPointCloud(*scan->cloud_ptr, *cloud_b, transTomat(lidar_base_options_.T_B_L), true);
  pcl::transformPointCloud(*scan->cloud_ptr, *cloud_w,
                           transTomat(getPoseEstimate(cur_state) * lidar_base_options_.T_B_L),
                           true);

  // Search landmarks
  VoxelPlanePtr searched_plane = nullptr;
  residual_cloud->clear();
  for (size_t idx = 0; idx < cloud_w->size(); idx++) {
    ObsId cur_id(cur_state.is_keyframe, scan->seq, idx);
    // Skip this point if not searched
    if (!tree_handler_->searchLandmarks(cloud_w->points[idx], searched_plane, cur_id)) {
      continue;
    }

    // If current searched landmark is not in the estimator, add it
    if (!isLandmarkInEstimator(createLidarLandmarkId(searched_plane->landmark_id_))) {
      // add the landmark
      BackendId landmark_backend_id = createLidarLandmarkId(searched_plane->landmark_id_);
      std::shared_ptr<HomogeneousPointParameterBlock> point_parameter_block =
          std::make_shared<HomogeneousPointParameterBlock>(searched_plane->center_,
                                                           landmark_backend_id.asInteger());

      CHECK(graph_->addParameterBlock(point_parameter_block, Graph::HomogeneousPoint));

      Eigen::Vector4d center;
      center << searched_plane->center_, 1;
      Eigen::Matrix3d information =
          Eigen::Matrix3d::Identity() * 1.0 / lidar_base_options_.landmark_var;
      // create error term
      std::shared_ptr<HomogeneousPointError> hp_error =
          std::make_shared<HomogeneousPointError>(center, information);
      graph_->addResidualBlock(hp_error, nullptr,
                               graph_->parameterBlockPtr(landmark_backend_id.asInteger()));

      // add landmark to map
      landmarks_map_.emplace_hint(landmarks_map_.end(), landmark_backend_id,
                                  MapPlane(searched_plane));

      searched_plane->is_in_graph_ = true;
    }

    // Add one observation residual after ensuring the landmark is in the graph
    addPlaneErrorResidualBlocks(cur_state, cloud_b->points[idx], searched_plane, cur_id);

    residual_cloud->push_back(cloud_w->points[idx]);
  }

  tree_handler_->setResidual(residual_cloud);
}

void LidarEstimatorBase::addPlaneResidualBlocks(const State& cur_state, const ScanPtr& scan)
{
  Cloud_ptr cloud_b(new Cloud);
  Cloud_ptr cloud_w(new Cloud);

  pcl::transformPointCloud(*scan->cloud_ptr, *cloud_b, transTomat(lidar_base_options_.T_B_L), true);
  pcl::transformPointCloud(*scan->cloud_ptr, *cloud_w,
                           transTomat(getPoseEstimate(cur_state) * lidar_base_options_.T_B_L),
                           true);

  // Search landmarks
  VoxelPlanePtr plane;
  for (size_t idx = 0; idx < cloud_w->size(); idx++) {
    ObsId cur_id(cur_state.is_keyframe, scan->seq, idx);
    // Skip this point if not searched
    if (!tree_handler_->searchLandmarks(cloud_w->points[idx], plane, cur_id)) {
      continue;
    }

    // If current searched landmark is in the estimator, add residual blocks
    if (isLandmarkInEstimator(createLidarLandmarkId(plane->landmark_id_))) {
      // Add residual blocks
      addPlaneErrorResidualBlocks(cur_state, cloud_b->points[idx], plane, cur_id);
    }
  }
}

void LidarEstimatorBase::addRegistrationErrorResidualBlocks(const State& cur_state,
                                                            const ScanPtr& scan)
{
  Eigen::Vector4d plane_params;
  // Transform from LiDAR to body using the extrinsic prior
  Cloud_ptr cloud_b(new Cloud);
  Cloud_ptr cloud_w(new Cloud);

  Cloud_ptr residual_cloud(new Cloud);

  pcl::transformPointCloud(*scan->cloud_ptr, *cloud_b, transTomat(lidar_base_options_.T_B_L), true);
  pcl::transformPointCloud(*scan->cloud_ptr, *cloud_w,
                           transTomat(getPoseEstimate(cur_state) * lidar_base_options_.T_B_L),
                           true);

  // Coarse search, and add residual blocks
  VoxelPlane searched_plane;
  residual_cloud->clear();
  for (size_t idx = 0; idx < cloud_w->size(); idx++) {
    ObsId cur_id(cur_state.is_keyframe, scan->seq, idx);
    if (!tree_handler_->searchVoxelMap(cloud_w->points[idx], searched_plane, cur_id)) {
      continue;
    }

    residual_cloud->push_back(cloud_w->points[idx]);

    plane_params << searched_plane.normal_, searched_plane.d_;
    // Add residual blocks
    addGlobalRegistrationErrorResidualBlocks(cur_state, cloud_b->points[idx], plane_params);
  }
}

ceres::ResidualBlockId LidarEstimatorBase::addPlaneErrorResidualBlocks(const State& state,
                                                                       const Point_lidar p,
                                                                       VoxelPlanePtr& plane,
                                                                       const ObsId& obsid)
{
  Eigen::Matrix<double, 1, 1> information = Eigen::Matrix<double, 1, 1>::Identity();
  information *= 1 / (lidar_base_options_.var);
  Eigen::Vector3d point(p.x, p.y, p.z);
  std::shared_ptr<PlaneError> planeError =
      std::make_shared<PlaneError>(plane->normal_, point, information);

  const BackendId landmark_backend_id = createLidarLandmarkId(plane->landmark_id_);

  if (!isLandmarkInEstimator(landmark_backend_id)) {
    return nullptr;
  }

  ceres::ResidualBlockId ret_val = graph_->addResidualBlock(
      planeError, loss_function_, graph_->parameterBlockPtr(state.id_in_graph.asInteger()),
      graph_->parameterBlockPtr(landmark_backend_id.asInteger()));

  // Store observation for later marginalization
  landmarks_map_.at(landmark_backend_id)
      .observations.insert(std::pair<ObsId, uint64_t>(obsid, reinterpret_cast<uint64_t>(ret_val)));
  landmarks_map_.at(landmark_backend_id).num_observations_historical++;

  return ret_val;
}

ceres::ResidualBlockId LidarEstimatorBase::addGlobalRegistrationErrorResidualBlocks(
    const State& state, const Point_lidar p, const Eigen::Vector4d params)
{
  Eigen::Matrix<double, 1, 1> information = Eigen::Matrix<double, 1, 1>::Identity();
  information *= 1 / (0.01);
  std::shared_ptr<GlobalRegistrationError> registrationError =
      std::make_shared<GlobalRegistrationError>(params, p, information);

  ceres::ResidualBlockId ret_val = graph_->addResidualBlock(
      registrationError, nullptr, graph_->parameterBlockPtr(state.id_in_graph.asInteger()));

  return ret_val;
}

void LidarEstimatorBase::addLocalRegistrationErrorResidualBlocks(State& cur_state,
                                                                 State& last_state,
                                                                 const Cloud_ptr cloud_l,
                                                                 bool is_neighbor)
{
  Eigen::Vector4d plane_params;

  // Transform from LiDAR to body using the extrinsic prior
  Cloud_ptr cloud_b(new Cloud);
  Cloud_ptr cloud_w(new Cloud);

  pcl::transformPointCloud(*cloud_l, *cloud_b, transTomat(lidar_base_options_.T_B_L), true);
  pcl::transformPointCloud(*cloud_l, *cloud_w,
                           transTomat(getPoseEstimate(last_state).inverse() *
                                      getPoseEstimate(cur_state) * lidar_base_options_.T_B_L),
                           true);

  // Coarse search, and add residual blocks
  for (size_t idx = 0; idx < cloud_w->size(); idx++) {
    if (!tree_handler_->getLocalTree()->search(cloud_w->points[idx], plane_params)) {
      continue;
    };

    if (!addLocalRegistrationErrorResidualBlocks(cur_state, last_state, cloud_b->points[idx],
                                                 plane_params)) {
      continue;
    }
  }
}

ceres::ResidualBlockId LidarEstimatorBase::addLocalRegistrationErrorResidualBlocks(
    const State& cur_state, const State& last_state, const Point_lidar p,
    const Eigen::Vector4d params)
{
  Eigen::Matrix<double, 1, 1> information = Eigen::Matrix<double, 1, 1>::Identity();
  information *= 1 / lidar_base_options_.var;
  std::shared_ptr<LocalRegistrationError> registrationError =
      std::make_shared<LocalRegistrationError>(params, p, information);

  ceres::ResidualBlockId ret_val =
      graph_->addResidualBlock(registrationError, loss_function_,
                               graph_->parameterBlockPtr(cur_state.id_in_graph.asInteger()),
                               graph_->parameterBlockPtr(last_state.id_in_graph.asInteger()));

  return ret_val;
}

Transformation LidarEstimatorBase::getExtrinsicEstimate()
{
  std::shared_ptr<PoseParameterBlock> block_ptr = std::static_pointer_cast<PoseParameterBlock>(
      graph_->parameterBlockPtr(lidar_extrinsics_id_.asInteger()));
  CHECK(block_ptr != nullptr);
  Transformation T_BL = block_ptr->estimate();

  return T_BL;
}

// Reject excessive residuals between state and landmarks
bool LidarEstimatorBase::rejectExcessiveResiduals(const State& state)
{
  if (!state.is_keyframe) return false;

  // Find all residuals associated with this state
  Graph::ResidualBlockCollection residuals = graph_->residuals(state.id_in_graph.asInteger());

  // Group residuals by landmark ID
  std::map<BackendId, std::vector<std::pair<ceres::ResidualBlockId, double>>> landmark_residuals;

  // First pass: Evaluate all residuals and group them by landmark
  for (auto& residual : residuals) {
    // Skip if not a plane error
    if (residual.error_interface_ptr->typeInfo() != ErrorType::kPlaneError) continue;

    // Get the landmark ID from the residual block
    const Graph::ParameterBlockCollection parameters =
        graph_->parameters(residual.residual_block_id);

    // Plane error uses pose at index 0 and landmark at index 1
    if (parameters.size() < 2) continue;

    const BackendId landmark_id(parameters.at(1).first);
    if (landmark_id.type() != IdType::lLandmark) continue;

    // Evaluate the residual to get its error value
    double residual_value[1];
    graph_->problem()->EvaluateResidualBlock(residual.residual_block_id, false, nullptr,
                                             residual_value, nullptr);

    double error = fabs(*residual_value);

    // Group residual by landmark
    landmark_residuals[landmark_id].push_back(std::make_pair(residual.residual_block_id, error));
  }

  // Second pass: Keep only the residual with smallest error for each landmark
  int removed_count = 0;
  for (auto& [landmark_id, residual_list] : landmark_residuals) {
    if (residual_list.size() <= 1) continue;  // Nothing to do if only one residual

    // Sort residuals by error value (ascending)
    std::sort(
        residual_list.begin(), residual_list.end(),
        [](const std::pair<ceres::ResidualBlockId, double>& a,
           const std::pair<ceres::ResidualBlockId, double>& b) { return a.second < b.second; });

    // Keep the first one (smallest error), remove the rest
    for (size_t i = 1; i < residual_list.size(); ++i) {
      ceres::ResidualBlockId to_remove = residual_list[i].first;

      // Remove observation from landmark map
      auto& map_plane = landmarks_map_.at(landmark_id);
      for (auto it = map_plane.observations.begin(); it != map_plane.observations.end();) {
        if (it->second == reinterpret_cast<uint64_t>(to_remove)) {
          it = map_plane.observations.erase(it);
        } else {
          ++it;
        }
      }

      // Remove from the graph
      graph_->removeResidualBlock(to_remove);
      removed_count++;
    }
  }

  return removed_count > 0;
}

void LidarEstimatorBase::addLandmarkParameterMarginBlocksWithResiduals(const State& state,
                                                                       bool keep)
{
  // Only apply landmark marginalization for keyframes
  if (!state.is_keyframe) return;

  const BackendId& margin_keyframe_id = state.id_in_graph;
  for (PlaneMap::iterator pit = landmarks_map_.begin(); pit != landmarks_map_.end();) {
    Graph::ResidualBlockCollection residuals = graph_->residuals(pit->first.asInteger());
    CHECK(residuals.size() != 0) << ": " << pit->second.observations.size();

    // Check whether the landmark is affected by marginalization
    bool at_pose_to_margin = false;
    size_t num_at_other_poses = 0;
    for (size_t r = 0; r < residuals.size(); ++r) {
      if (residuals[r].error_interface_ptr->typeInfo() != ErrorType::kPlaneError) continue;

      BackendId pose_id(graph_->parameters(residuals[r].residual_block_id).at(0).first);
      bool is_keyframe = false;
      for (auto s : states_) {
        if (s.is_keyframe && (s.id == pose_id)) {
          is_keyframe = true;
          break;
        }
      }
      // Skip an already erased state
      if (pose_id == margin_keyframe_id) is_keyframe = true;

      // if the landmark is visible in the frames to marginalize
      if (pose_id == margin_keyframe_id) at_pose_to_margin = true;

      // the landmark is still visible in keyframes of the sliding window
      if (is_keyframe && (pose_id != margin_keyframe_id)) {
        num_at_other_poses++;
      }
    }

    // Skip landmarks unaffected by marginalization
    if (!at_pose_to_margin) {
      pit++;
      continue;
    }

    // Collect residuals to marginalize
    bool margin_parameter = false;
    for (size_t r = 0; r < residuals.size(); ++r) {
      if (residuals[r].error_interface_ptr->typeInfo() != ErrorType::kPlaneError) continue;

      BackendId pose_id(graph_->parameters(residuals[r].residual_block_id).at(0).first);

      // can be observed by at least two keyframes, margin the observation
      // at current keyframe
      if (at_pose_to_margin && num_at_other_poses >= 2) {
        if (pose_id == margin_keyframe_id) {
          marginalization_error_->addResidualBlock(residuals[r].residual_block_id);
        }
      }
      // cannot be observed by at least two other keyframes, marginalize the landmark
      // and its keyframe observations, and erase non-keyframe observations.
      else if (at_pose_to_margin && num_at_other_poses < 2) {
        // Add information to the pending marginalization item
        margin_parameter = true;
        // Check whether the observation belongs to a keyframe
        bool is_keyframe = false;
        for (auto s : states_) {
          if (s.is_keyframe && (s.id == pose_id)) {
            is_keyframe = true;
            break;
          }
        }
        // Skip an already erased state
        if (pose_id == margin_keyframe_id) is_keyframe = true;
        // Marginalize keyframe observation
        if (is_keyframe) {
          marginalization_error_->addResidualBlock(residuals[r].residual_block_id);
        }
        // Erase non-keyframe observation
        else {
          erasePlaneErrorResidualBlock(residuals[r].residual_block_id);
        }
      }
    }

    // Update landmark parameter blocks
    if (margin_parameter) {
      marginalization_parameter_ids_.push_back(pit->first);
      marginalization_keep_parameter_blocks_.push_back(keep);
      pit->second.plane->is_in_graph_ = false;
      pit = landmarks_map_.erase(pit);
      continue;
    }

    pit++;
  }
}

void LidarEstimatorBase::updateCloudMap(const State& state)
{
  scan_to_map_W_.reset(new Cloud());
  pcl::transformPointCloud(*state.current_scan_l, *scan_to_map_W_,
                           transTomat(getPoseEstimate(state) * lidar_base_options_.T_B_L), true);

  tree_handler_->mapGrow(scan_to_map_W_);
  *local_map_ += *scan_to_map_W_;

  if (tree_handler_->getMapSize() > 5e4) {
    local_map_->clear();
    for (auto s : states_) {
      if (s.id.type() == IdType::lPose && s.is_keyframe) {
        Cloud_ptr scan_w(new Cloud);
        pcl::transformPointCloud(*s.current_scan_l, *scan_w,
                                 transTomat(getPoseEstimate(s) * lidar_base_options_.T_B_L), true);
        *local_map_ += *scan_w;
      }
    }

    tree_handler_->mapBuild(local_map_);
  }
}

void LidarEstimatorBase::updateLandmarks()
{
  for (auto& id_and_map_point : landmarks_map_) {
    // Update landmark coordinates
    id_and_map_point.second.hom_coordinates =
        std::static_pointer_cast<HomogeneousPointParameterBlock>(
            graph_->parameterBlockPtr(id_and_map_point.first.asInteger()))
            ->estimate();
    CHECK(id_and_map_point.second.hom_coordinates(3) == 1.0);
    // Synchronize optimized landmark centers with the frontend
    id_and_map_point.second.plane->center_ = id_and_map_point.second.hom_coordinates.head<3>();
  }
}

void LidarEstimatorBase::eraseEmptyLandmarks()
{
  for (PlaneMap::iterator pit = landmarks_map_.begin(); pit != landmarks_map_.end();) {
    if (pit->second.observations.size() == 0) {
      graph_->removeParameterBlock(pit->first.asInteger());
      pit->second.plane->is_in_graph_ = false;
      pit = landmarks_map_.erase(pit);
    } else
      pit++;
  }
}

}
