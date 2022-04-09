/**
* @Function: SVO library
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#pragma once

#include <svo/svo.h>
#include <svo/tracker/feature_tracking_utils.h>
#include <svo/ceres_backend/estimator.hpp>
#include <svo/ceres_backend/ceres_iteration_callback.hpp>
#include <svo/ceres_backend/marginalization_error.hpp>
#include <svo/ceres_backend/pose_error.hpp>
#include <svo/ceres_backend/pose_parameter_block.hpp>
#include <svo/ceres_backend/reprojection_error.hpp>
#include <svo/ceres_backend/homogeneous_point_error.hpp>
#include <svo/ceres_backend/relative_pose_error.hpp>
#include <svo/outlier_rejection.hpp>
#include <svo/img_align/sparse_img_align.h>
#include <svo/direct/feature_detection_utils.h>
#include <svo/direct/feature_alignment.h>
#include <svo/direct/patch_warp.h>
#include <svo/direct/patch_score.h>
#include <svo/direct/patch_utils.h>

// We do not directly apply (using namespace svo) here to 
// avoid some naming conflit when reforming some features
// of svo.
namespace gici {

// Common
namespace initialization_utils = svo::initialization_utils;
namespace frame_utils = svo::frame_utils;
namespace feature_tracking_utils = svo::feature_tracking_utils;
namespace warp = svo::warp;
namespace patch_utils = svo::patch_utils;
namespace feature_alignment = svo::feature_alignment;
namespace feature_detection_utils = svo::feature_detection_utils;
namespace ceres_backend = svo::ceres_backend;
namespace feature_detection_utils = svo::feature_detection_utils;
namespace reprojector_utils = svo::reprojector_utils;

using Frame = svo::Frame;
using FramePtr = svo::FramePtr;
using Keypoint = svo::Keypoint;
using DetectorOptions = svo::DetectorOptions;
using ReprojectorOptions = svo::ReprojectorOptions;
using InitializationOptions = svo::InitializationOptions;
using FeatureTrackerOptions = svo::FeatureTrackerOptions;
using SparseImgAlignOptions = svo::SparseImgAlignOptions;
using FrameBundle = svo::FrameBundle;
using FrameBundlePtr = svo::FrameBundlePtr;
using Map = svo::Map;
using MapPtr = svo::MapPtr;
using Camera = svo::Camera;
using CameraPtr = svo::CameraPtr;
using CameraBundle = svo::CameraBundle;
using CameraBundlePtr = svo::CameraBundlePtr;
using Transformation = svo::Transformation;
using InitResult = svo::InitResult;
using Bearings = svo::Bearings;
using Positions = svo::Positions;
using Point = svo::Point;
using PointPtr = svo::PointPtr;
using InitializerPtr = svo::InitializerPtr;
using FeatureType = svo::FeatureType;
using BearingVector = svo::BearingVector;
using FeatureWrapper = svo::FeatureWrapper;
using FloatType = svo::FloatType;
using GradientVector = svo::GradientVector;
using SeedState = svo::SeedState;
using DetectorPtr = svo::DetectorPtr;
using DetectorType = svo::DetectorType;
using AbstractDetector = svo::AbstractDetector;
using Keypoints = svo::Keypoints;
using Scores = svo::Scores;
using Levels = svo::Levels;
using Gradients = svo::Gradients;
using FeatureTypes = svo::FeatureTypes;
using OccupandyGrid2D = svo::OccupandyGrid2D;
using States = svo::States;
using MarginalizationTiming = svo::MarginalizationTiming;
using ExtrinsicsEstimationParametersVec = svo::ExtrinsicsEstimationParametersVec;
using FrameBundleConstPtr = svo::FrameBundleConstPtr;
using BackendId = svo::BackendId;
using Point = svo::Point;
using PointPtr = svo::PointPtr;
using BundleId = svo::BundleId;
using MapPoint = svo::MapPoint;
using PointMap = svo::PointMap;
using IdType = svo::IdType;
using Position = svo::Position;
using KeypointIdentifier = svo::KeypointIdentifier;
using MapPointVector = svo::MapPointVector;
using PerformanceMonitorPtr = svo::PerformanceMonitorPtr;
using OutlierRejection = svo::OutlierRejection;
using ExtrinsicsEstimationParameters = svo::ExtrinsicsEstimationParameters;
using EnumClassHash = svo::EnumClassHash;
using Quaternion = svo::Quaternion;
using BundleAdjustmentType = svo::BundleAdjustmentType;
using SparseImgAlignBasePtr = svo::SparseImgAlignBasePtr;
using ReprojectorPtr = svo::ReprojectorPtr;
using Reprojector = svo::Reprojector;
using DetectorPtr = svo::DetectorPtr;
using SparseImgAlign = svo::SparseImgAlign;
using SeedRef = svo::SeedRef;
using GradientVector = svo::GradientVector;

inline BackendId createLandmarkId(int track_id)
{
  return BackendId(static_cast<uint64_t>(track_id) |
                   (static_cast<uint64_t>(IdType::Landmark) << 56));
}

inline BackendId createNFrameId(int32_t bundle_id)
{
  CHECK_GE(bundle_id, 0);
  return BackendId((static_cast<uint64_t>(bundle_id) << 16) |
                   (static_cast<uint64_t>(IdType::NFrame) << 56));
}

inline BackendId createExtrinsicsId(uint8_t camera_index,
                                    int32_t bundle_id){
  return BackendId((static_cast<uint64_t>(
                      static_cast<uint32_t>(bundle_id)) << 16) |
                   (static_cast<uint64_t>(camera_index) << 48) |
                   (static_cast<uint64_t>(IdType::Extrinsics) << 56));
}

inline BackendId createImuStateId(int32_t bundle_id)
{
  return BackendId((static_cast<uint64_t>(
                      static_cast<uint32_t>(bundle_id)) << 16) |
                   (static_cast<uint64_t>(IdType::ImuStates) << 56));
}

}
