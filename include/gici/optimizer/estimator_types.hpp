/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *  Copyright (c) 2016, ETH Zurich, Wyss Zurich, Zurich Eye
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Jul 6, 2016
 *      Author: Zurich Eye
 *      Modified: Cheng Chi
 *********************************************************************************/

#pragma once

#include <map>
#include <vector>

#pragma diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// Eigen 3.2.7 uses std::binder1st and std::binder2nd which are deprecated since c++11
// Fix is in 3.3 devel (http://eigen.tuxfamily.org/bz/show_bug.cgi?id=872).
#include <Eigen/Core>
#pragma diagnostic pop

#include <glog/logging.h>

#include "gici/utility/svo.h"

namespace gici {

// -----------------------------------------------------------------------------
// IDs
enum class IdType : uint8_t
{
  cNFrame = 0,
  cLandmark = 1,
  ImuStates = 2,
  cExtrinsics = 3,
  gPosition = 4,
  gClock = 5, 
  gFrequency = 6,
  gTroposphere = 7,
  gExtrinsics = 8,
  gAmbiguity = 9,
  gIonosphere = 10
};

//! The Backend ID for multiple types.
//! Memory layout for types {Frame, IMU state, GNSS position}:
//! Byte 0: IdType
//! Byte 1: zero
//! Byte 2-5: BundleID
//! Byte 6-7: zero
//!
//! For Extrinsics
//! Byte 0: IdType
//! Byte 1: CameraIdx
//! Byte 2-5: BundleID
//! Byte 6-7: zero
//!
//! For Landmarks
//! Byte 0: IdType
//! Byte 1-3: zero
//! Byte 4-7: 32 bit Track ID
//!
//! For GNSS clock
//! Byte 0: IdType
//! Byte 1: GNSS system
//! Byte 2-5: BundleID
//! Byte 6-7: zero
class BackendId
{
public:
  BackendId() = default;
  explicit BackendId(uint64_t id) : id_(id) {}

  uint64_t asInteger() const
  {
    return id_;
  }

  IdType type() const
  {
    // The first byte represents the type.
    return static_cast<IdType>(id_ >> 56);
  }

  int32_t bundleId() const
  {
    CHECK(type() != IdType::cLandmark)
        << "Landmarks do not have a bundle ID.";
    // The bundle ID is byte 2 -> 6 in id.
    return static_cast<int32_t>((id_ >> 16) & 0xFFFFFFFF);
  }

  uint32_t trackId() const
  {
    CHECK(type() == IdType::cLandmark);
    // In case of a landmark, the last 4 bytes are the track ID.
    return static_cast<uint32_t>(id_ & 0xFFFFFFFF);
  }

  uint16_t nFrameHandle() const
  {
    CHECK(type() == IdType::cNFrame ||
                type() == IdType::ImuStates ||
                type() == IdType::cExtrinsics);
    // In case of an NFrame, the last 2 bytes are the handle.
    return static_cast<uint16_t>(id_ & 0xFFFF);
  }

  uint8_t cameraIndex() const
  {
    CHECK(type() == IdType::cExtrinsics);
   // The second byte is the camara index.
    return static_cast<uint8_t>((id_ >> 48) & 0x00000FF);
  }

  bool valid() const
  {
    return id_ != 0;
  }

private:
  uint64_t id_{0};
};

// Factories
inline BackendId createLandmarkId(int track_id)
{
  return BackendId(static_cast<uint64_t>(track_id) |
                   (static_cast<uint64_t>(IdType::cLandmark) << 56));
}

inline BackendId createNFrameId(int32_t bundle_id)
{
  CHECK_GE(bundle_id, 0);
  return BackendId((static_cast<uint64_t>(bundle_id) << 16) |
                   (static_cast<uint64_t>(IdType::cNFrame) << 56));
}

inline BackendId createGNSSPositionId(int32_t bundle_id)
{
  CHECK_GE(bundle_id, 0);
  return BackendId((static_cast<uint64_t>(bundle_id) << 16) |
                   (static_cast<uint64_t>(IdType::gPosition) << 56));
}

inline BackendId createGNSSClockId(char system,
                                   int32_t bundle_id)
{
  CHECK_GE(bundle_id, 0);
  return BackendId((static_cast<uint64_t>(
                      static_cast<uint32_t>(bundle_id)) << 16) |
                   (static_cast<uint64_t>(system) << 48) |
                   (static_cast<uint64_t>(IdType::cExtrinsics) << 56));
}

inline BackendId createExtrinsicsId(uint8_t camera_index,
                                    int32_t bundle_id){
  return BackendId((static_cast<uint64_t>(
                      static_cast<uint32_t>(bundle_id)) << 16) |
                   (static_cast<uint64_t>(camera_index) << 48) |
                   (static_cast<uint64_t>(IdType::cExtrinsics) << 56));
}

inline BackendId createImuStateId(int32_t bundle_id)
{
  return BackendId((static_cast<uint64_t>(
                      static_cast<uint32_t>(bundle_id)) << 16) |
                   (static_cast<uint64_t>(IdType::ImuStates) << 56));
}

inline BackendId changeIdType(BackendId id, IdType type, size_t cam_index = 0)
{
  CHECK(id.type() != IdType::cLandmark);
  CHECK(type != IdType::cLandmark);
  CHECK(cam_index == 0 || type == IdType::cExtrinsics);
  // Last 6 bytes remain the same.
  return BackendId((id.asInteger() & 0xFFFFFFFFFFFF) |
                   (static_cast<uint64_t>(cam_index) << 48) |
                   (static_cast<uint64_t>(type) << 56));
}

inline BackendId changeIdType(BackendId id, IdType type, const char system)
{
  CHECK(id.type() != IdType::gPosition);
  // Last 6 bytes remain the same.
  return BackendId((id.asInteger() & 0xFFFFFFFFFFFF) |
                   (static_cast<uint64_t>(system) << 48) |
                   (static_cast<uint64_t>(type) << 56));
}

// Comparison operator for use in maps.
inline bool operator<(const BackendId& lhs, const BackendId& rhs)
{
  return lhs.asInteger() < rhs.asInteger();
}

inline bool operator==(const BackendId& lhs, const BackendId& rhs)
{
  return lhs.asInteger() == rhs.asInteger();
}

inline bool operator!=(const BackendId& lhs, const BackendId& rhs)
{
  return lhs.asInteger() != rhs.asInteger();
}

inline bool operator>=(const BackendId& lhs, const BackendId& rhs)
{
  return lhs.asInteger() >= rhs.asInteger();
}

inline std::ostream& operator<<(std::ostream& out, const BackendId& id)
{
  out << std::hex << id.asInteger() << std::dec;
  return out;
}

//------------------------------------------------------------------------------
/**
 * @brief A type to store information about a point in the world graph.
 */
struct MapPoint
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief Default constructor. Point is nullptr.
  MapPoint()
      : point(nullptr), fixed_position(false)
  {}
  /**
   * @brief Constructor.
   * @param point     Pointer to underlying gici::Point
   */
  MapPoint(const PointPtr& point)
    : point(point), fixed_position(false)
  {
    hom_coordinates << point->pos(), 1;
  }

  Eigen::Vector4d hom_coordinates; ///< Continuosly updates position of point


  //! Pointer to the point. The position is not updated inside backend
  //! because of possible multithreading conflicts.
  PointPtr point;

  //! Observations of this point. The uint64_t's are the casted
  //! ceres::ResidualBlockId values of the reprojection error residual block.
  std::map<KeypointIdentifier, uint64_t> observations;

  //! Is the point position fixed by a loop closure?
  bool fixed_position;
};

typedef std::vector<MapPoint, Eigen::aligned_allocator<MapPoint> > MapPointVector;
typedef std::map<BackendId, MapPoint, std::less<BackendId>,
  Eigen::aligned_allocator<std::pair<const BackendId, MapPoint>> > PointMap;

} // namespace gici
