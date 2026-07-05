/**
 * @Function: Base class for LiDAR estimators
 *
 * @Author  : Jiahui Liu
 * @Email   : jh.liu@sjtu.edu.cn
 **/
#pragma once

#include "gici/estimate/estimator_base.h"
#include "gici/utility/common.h"
#include "gici/lidar/tree_handler.h"
#include "gici/lidar/global_registration_error.h"
#include "gici/lidar/plane_error.h"
#include "gici/lidar/local_registration_error.h"
#include "gici/vision/homogeneous_point_error.h"

namespace gici {

// Common LiDAR estimator options
struct LidarEstimatorBaseOptions {
  // Minimum usable LiDAR range, in meters
  double blind = 2.0;
  // LiDAR-to-body extrinsic transformation T_B_L
  Transformation T_B_L;
  // Point-to-plane residual variance, in square meters
  double var;
  // Plane landmark position variance, in square meters
  double landmark_var;
  // Minimum keyframe interval, in seconds
  double kfselect_min_dt = 0.2;
};

// Plane landmarks indexed by backend parameter ID
typedef std::map<BackendId, MapPlane, std::less<BackendId>,
                 Eigen::aligned_allocator<std::pair<const BackendId, MapPlane>>>
    PlaneMap;

// Estimator
class LidarEstimatorBase : public virtual EstimatorBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LidarEstimatorBase(const LidarEstimatorBaseOptions& options,
                     const EstimatorBaseOptions& base_options);
  ~LidarEstimatorBase();

  // Set tree handler
  inline void setTreeHandler(const std::shared_ptr<TreeHandler>& tree_handler)
  {
    tree_handler_ = tree_handler;
  }

  // Add scan measurement and state
  virtual bool addLidarMeasurementAndState(const ScanPtr& scan)
  {
    return false;
  }

  inline void tflidarpoint(Point_lidar& p, Transformation T)
  {
    const auto& R = T.getRotationMatrix();
    const auto& t = T.getPosition();

    double x = p.x, y = p.y, z = p.z;
    p.x = R(0, 0) * x + R(0, 1) * y + R(0, 2) * z + t.x();
    p.y = R(1, 0) * x + R(1, 1) * y + R(1, 2) * z + t.y();
    p.z = R(2, 0) * x + R(2, 1) * y + R(2, 2) * z + t.z();
  }

  inline Eigen::Matrix4d transTomat(Transformation T)
  {
    Eigen::Matrix4d M;
    M.setIdentity();
    M.block<3, 3>(0, 0) = T.getRotationMatrix();
    M.block<3, 1>(0, 3) = T.getPosition();
    return M;
  }

protected:
  // Add the LiDAR extrinsic parameter block to the graph
  BackendId addLidarExtrinsicsParameterBlock(const int32_t id, const Transformation& T_BL_prior,
                                             const bool if_estimate_extrinsics);

  void addLidarResidualMarginBlocks(const State& state);

  void erasePlaneErrorResidualBlock(ceres::ResidualBlockId residual_block_id);

  void erasePlaneErrorResidualBlocks(const State& state);

  void eraseRegistrationErrorResidualBlocks(const State& state);

  void selectKeyFrame(const ScanPtr& scan, const Transformation T);

  void addGlobalRegistrationErrorResidualBlocks(const State& cur_state, const ScanPtr& scan,
                                                bool need_tf_body);

  void addPlaneParameterBlocksWithResiduals(const State& cur_state, const ScanPtr& scan);

  void addPlaneResidualBlocks(const State& cur_state, const ScanPtr& scan);

  void addRegistrationErrorResidualBlocks(const State& cur_state, const ScanPtr& scan);

  ceres::ResidualBlockId addPlaneErrorResidualBlocks(const State& state, const Point_lidar p,
                                                     VoxelPlanePtr& plane, const ObsId& obsid);

  void addLocalRegistrationErrorResidualBlocks(State& cur_state, State& last_state,
                                               const Cloud_ptr cloud, bool is_neighbor);

  ceres::ResidualBlockId addGlobalRegistrationErrorResidualBlocks(const State& state,
                                                                  const Point_lidar p,
                                                                  const Eigen::Vector4d params);

  ceres::ResidualBlockId addLocalRegistrationErrorResidualBlocks(const State& cur_state,
                                                                 const State& last_state,
                                                                 const Point_lidar p,
                                                                 const Eigen::Vector4d params);

  Transformation getExtrinsicEstimate();

  bool rejectExcessiveResiduals(const State& state);

  void addLandmarkParameterMarginBlocksWithResiduals(const State& state, bool keep = false);

  void updateCloudMap(const State& state);

  void updateLandmarks();

  void eraseEmptyLandmarks();

  // Get current scan
  inline ScanPtr& curScan()
  {
    return getCurrent(scans_);
  }

  // Count size of the keyframe states
  inline size_t sizeOfLidarkeyframeStates()
  {
    return static_cast<size_t>(std::count_if(
        states_.begin(), states_.end(), [](State& state) { return state.valid() && state.is_keyframe; }));
  }

  // Check if a landmark is in estimator
  inline bool isLandmarkInEstimator(BackendId id) const
  {
    return landmarks_map_.find(id) != landmarks_map_.end();
  }

protected:
  // Options
  LidarEstimatorBaseOptions lidar_base_options_;

  // Measurements
  std::deque<ScanPtr> scans_;

  // States
  BackendId lidar_extrinsics_id_;

  // Tree handler
  std::shared_ptr<TreeHandler> tree_handler_;

  // Keyframe
  ScanPtr last_keyframe_;
  Transformation last_keyframe_T_;

  // Current scan transformed to the world frame for map insertion
  Cloud_ptr scan_to_map_W_;

  PlaneMap landmarks_map_;

  // Local map
  Cloud_ptr local_map_;

  ceres::LossFunction* loss_function_;
};

}
