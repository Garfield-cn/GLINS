/**
* @Function: SVO library
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#pragma once

#include <svo/svo.h>
#include <svo/tracker/feature_tracking_utils.h>
#include <svo/outlier_rejection.hpp>
#include <svo/img_align/sparse_img_align.h>
#include <svo/direct/feature_detection_utils.h>
#include <svo/direct/feature_alignment.h>
#include <svo/direct/patch_warp.h>
#include <svo/direct/patch_score.h>
#include <svo/direct/patch_utils.h>

// We do not directly apply (using namespace gici) here to 
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
using Point = svo::Point;
using PointPtr = svo::PointPtr;
using BundleId = svo::BundleId;
using Position = svo::Position;
using KeypointIdentifier = svo::KeypointIdentifier;
using PerformanceMonitorPtr = svo::PerformanceMonitorPtr;
using OutlierRejection = svo::OutlierRejection;
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
using CameraConstPtr = svo::CameraConstPtr;

}
