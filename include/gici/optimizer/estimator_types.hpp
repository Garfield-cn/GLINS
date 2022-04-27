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
#include "gici/utility/rtklib_safe.h"

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
  gVelocity = 5,
  gPose = 6, 
  gClock = 7, 
  gFrequency = 8,
  gTroposphere = 9,
  gExtrinsics = 10,
  gAmbiguity = 11,
  gIonosphere = 12
};

// The BackendID for multiple types
// bit 0-5:   IdType
// bit 6-13:  CameraIdx, GNSS system
// bit 14-21: GNSS PRN number
// bit 22-49: BundleID
// bit 50-55: GNSS PhaseID
// bit 32-63: Landmark ID
#define BITS_IDTYPE 0, 5
#define BITS_CAMERA_IDX 6, 13
#define BITS_GNSS_SYSTEM 6, 13
#define BITS_GNSS_PRN 14, 21
#define BITS_BUNDLEID 22, 49
#define BITS_GNSS_PHASEID 50, 55
#define BITS_LANDMARKID 32, 63
class BackendId
{
public:
  BackendId() = default;
  explicit BackendId(uint64_t id) : id_(id) {}

  // Get bits
  inline static uint32_t getBits(uint64_t id, int start, int end) {
    int length = end - start + 1;
    uint8_t buffer[8];
    for (int i = 0; i < 8; i++) {
      buffer[i] = (id >> (8 * (7 - i))) & 0xFF;
    }
    return getbitu(buffer, start, length);
  }

  // Set bits
  template<typename T> 
  inline static uint64_t setBits(T data, int start, int end) {
    int length = end - start + 1;
    uint32_t bits = static_cast<uint32_t>(data);
    uint64_t id = 0;
    uint8_t buffer[8];
    memset(buffer, 0, sizeof(uint8_t) * 8);
    setbitu(buffer, start, length, bits);
    for (int i = 0; i < 8; i++) {
      uint64_t byte = static_cast<uint64_t>(buffer[i]) << (8 * (7 - i));
      id += byte;
    }
    return id;
  }

  // Reset some bits
  template<typename T> 
  inline static uint64_t resetBits(
    uint64_t id, T data, int start, int end) {
    int length = end - start + 1;
    uint32_t bits = static_cast<uint32_t>(data);
    uint64_t out_id = 0;
    uint8_t buffer[8];
    for (int i = 0; i < 8; i++) {
      buffer[i] = (id >> (8 * (7 - i))) & 0xFF;
    }
    setbitu(buffer, start, length, bits);
    for (int i = 0; i < 8; i++) {
      uint64_t byte = static_cast<uint64_t>(buffer[i]) << (8 * (7 - i));
      out_id |= byte;
    }
    return out_id;
  }

  inline uint64_t asInteger() const {
    return id_;
  }

  inline IdType type() const {
    return static_cast<IdType>(getBits(id_, BITS_IDTYPE));
  }

  inline int32_t bundleId() const {
    CHECK(type() != IdType::cLandmark)
        << "Landmarks do not have a bundle ID.";
    return static_cast<int32_t>(getBits(id_, BITS_BUNDLEID));
  }

  inline uint32_t trackId() const {
    CHECK(type() == IdType::cLandmark);
    return static_cast<uint32_t>(getBits(id_, BITS_LANDMARKID));
  }

  inline uint8_t cameraIndex() const {
    CHECK(type() == IdType::cExtrinsics);
    return static_cast<uint8_t>(getBits(id_, BITS_CAMERA_IDX));
  }

  inline char gSystem() const {
    return static_cast<char>(getBits(id_, BITS_GNSS_SYSTEM));
  }

  inline int gPrnNumber() const {
    return static_cast<int>(getBits(id_, BITS_GNSS_PRN));
  }

  inline std::string gPrn() const {
    char system = gSystem();
    int prn_number = gPrnNumber();
    char prn_buf[4];
    sprintf(prn_buf, "%c%02d", system, prn_number);
    return std::string(prn_buf);
  }

  inline int gPhaseId() const {
    return static_cast<int>(getBits(id_, BITS_GNSS_PHASEID));
  }

  inline bool valid() const {
    return id_ != 0;
  }

private:
  uint64_t id_{0};
};

// Factories
inline BackendId createLandmarkId(int track_id)
{
  return BackendId(
    BackendId::setBits(track_id, BITS_LANDMARKID) |
    BackendId::setBits(IdType::cLandmark, BITS_IDTYPE));
}

inline BackendId createNFrameId(int32_t bundle_id)
{
  CHECK_GE(bundle_id, 0);
  return BackendId(
    BackendId::setBits(bundle_id, BITS_BUNDLEID) |
    BackendId::setBits(IdType::cNFrame, BITS_IDTYPE));
}

inline BackendId createGnssPositionId(int32_t bundle_id)
{
  CHECK_GE(bundle_id, 0);
  return BackendId(
    BackendId::setBits(bundle_id, BITS_BUNDLEID) |
    BackendId::setBits(IdType::gPosition, BITS_IDTYPE));
}

inline BackendId createGnssVelocityId(int32_t bundle_id)
{
  CHECK_GE(bundle_id, 0);
  return BackendId(
    BackendId::setBits(bundle_id, BITS_BUNDLEID) |
    BackendId::setBits(IdType::gVelocity, BITS_IDTYPE));
}

inline BackendId createGnssPoseId(int32_t bundle_id)
{
  CHECK_GE(bundle_id, 0);
  return BackendId(
    BackendId::setBits(bundle_id, BITS_BUNDLEID) |
    BackendId::setBits(IdType::gPose, BITS_IDTYPE));
}

inline BackendId createGnssClockId(char system,
                                   int32_t bundle_id)
{
  CHECK_GE(bundle_id, 0);
  return BackendId(
    BackendId::setBits(bundle_id, BITS_BUNDLEID) |
    BackendId::setBits(system, BITS_GNSS_SYSTEM) |
    BackendId::setBits(IdType::gClock, BITS_IDTYPE));
}

inline BackendId createGnssAmbiguityId(std::string prn,
                  int phase_id, int32_t bundle_id)
{
  CHECK_GE(bundle_id, 0);
  CHECK_GE(phase_id, 0);
  char system = prn[0];
  int prn_number = atoi(prn.substr(1, 2).data());
  return BackendId(
    BackendId::setBits(bundle_id, BITS_BUNDLEID) |
    BackendId::setBits(system, BITS_GNSS_SYSTEM) |
    BackendId::setBits(prn_number, BITS_GNSS_PRN) |
    BackendId::setBits(phase_id, BITS_GNSS_PHASEID) |
    BackendId::setBits(IdType::gAmbiguity, BITS_IDTYPE));
}

inline BackendId createExtrinsicsId(uint8_t camera_index,
                                    int32_t bundle_id){
  return BackendId(
    BackendId::setBits(bundle_id, BITS_BUNDLEID) | 
    BackendId::setBits(camera_index, BITS_CAMERA_IDX) |
    BackendId::setBits(IdType::cExtrinsics, BITS_IDTYPE));
}

inline BackendId createImuStateId(int32_t bundle_id)
{
  return BackendId(
    BackendId::setBits(bundle_id, BITS_BUNDLEID) | 
    BackendId::setBits(IdType::ImuStates, BITS_IDTYPE));
}

inline BackendId changeIdType(BackendId id, IdType type, size_t cam_index = 0)
{
  CHECK(id.type() != IdType::cLandmark);
  CHECK(type != IdType::cLandmark);
  CHECK(cam_index == 0 || type == IdType::cExtrinsics || type == IdType::gVelocity ||
        type == IdType::gExtrinsics);
  uint64_t out = id.asInteger();
  out = BackendId::resetBits(out, cam_index, BITS_CAMERA_IDX);
  out = BackendId::resetBits(out, type, BITS_IDTYPE);
  return BackendId(out);
}

inline BackendId changeIdType(BackendId id, IdType type, const char system)
{
  CHECK(id.type() != IdType::gClock);
  CHECK(type == IdType::gClock);
  uint64_t out = id.asInteger();
  out = BackendId::resetBits(out, system, BITS_GNSS_SYSTEM);
  out = BackendId::resetBits(out, type, BITS_IDTYPE);
  return BackendId(out);
}

inline bool sameAmbiguity(const BackendId& lhs, const BackendId& rhs)
{
  CHECK(BackendId::getBits(lhs.asInteger(), BITS_IDTYPE) == 
        static_cast<uint32_t>(IdType::gAmbiguity));
  CHECK(BackendId::getBits(rhs.asInteger(), BITS_IDTYPE) == 
        static_cast<uint32_t>(IdType::gAmbiguity));
  
  uint32_t sys_lhs = BackendId::getBits(lhs.asInteger(), BITS_GNSS_SYSTEM);
  uint32_t sys_rhs = BackendId::getBits(rhs.asInteger(), BITS_GNSS_SYSTEM);
  if (sys_lhs != sys_rhs) return false;

  uint32_t prn_lhs = BackendId::getBits(lhs.asInteger(), BITS_GNSS_PRN);
  uint32_t prn_rhs = BackendId::getBits(rhs.asInteger(), BITS_GNSS_PRN);
  if (prn_lhs != prn_rhs) return false;

  uint32_t phase_lhs = BackendId::getBits(lhs.asInteger(), BITS_GNSS_PHASEID);
  uint32_t phase_rhs = BackendId::getBits(rhs.asInteger(), BITS_GNSS_PHASEID);
  if (phase_lhs != phase_rhs) return false;

  return true;
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

} // namespace gici
