/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

// This file contains codec dependent definitions that are needed in
// order to compile the WebRTC codebase, even if this codec is not used.

#ifndef MODULES_VIDEO_CODING_CODECS_H265_INCLUDE_H265_GLOBALS_H_
#define MODULES_VIDEO_CODING_CODECS_H265_INCLUDE_H265_GLOBALS_H_

#include <algorithm>
#include <string>

#include "modules/video_coding/codecs/interface/common_constants.h"
#include "rtc_base/checks.h"

namespace webrtc {

// The packetization types that we support: single, aggregated, and fragmented.
enum H265PacketizationTypes {
    kH265SingleNalu,  // This packet contains a single NAL unit.
    kH265AP,       // This packet contains AP (single time
                    // aggregation) packets. If this packet has an
                    // associated NAL unit type, it'll be for the
                    // first such aggregated packet.
    kH265FU,         // This packet contains a FU (fragmentation
                    // unit) packet, meaning it is a part of a frame
                    // that was too large to fit into a single packet.
};

// Packetization modes are defined in RFC 6184 section 6
// Due to the structure containing this being initialized with zeroes
// in some places, and mode 1 being default, mode 1 needs to have the value
// zero. https://crbug.com/webrtc/6803
enum class H265PacketizationMode {
  NonInterleaved = 0,  // Mode 1 - STAP-A, FU-A is allowed
  SingleNalUnit        // Mode 0 - only single NALU allowed
};

// This function is declared inline because it is not clear which
// .cc file it should belong to.
// TODO(hta): Refactor. https://bugs.webrtc.org/6842
// TODO(jonasolsson): Use absl::string_view instead when that's available.
inline std::string ToString(H265PacketizationMode mode) {
  if (mode == H265PacketizationMode::NonInterleaved) {
    return "NonInterleaved";
  } else if (mode == H265PacketizationMode::SingleNalUnit) {
    return "SingleNalUnit";
  }
  RTC_DCHECK_NOTREACHED();
  return "";
}

struct H265NaluInfo {
  uint8_t type;
  int vps_id;
  int sps_id;
  int pps_id;

  friend bool operator==(const H265NaluInfo& lhs, const H265NaluInfo& rhs) {
    return lhs.type == rhs.type && lhs.sps_id == rhs.sps_id &&
           lhs.pps_id == rhs.pps_id;
  }

  friend bool operator!=(const H265NaluInfo& lhs, const H265NaluInfo& rhs) {
    return !(lhs == rhs);
  }
};

const size_t kMaxH265NalusPerPacket = 10;

struct RTPVideoHeaderH265 {
  // The NAL unit type. If this is a header for a
  // fragmented packet, it's the NAL unit type of
  // the original data. If this is the header for an
  // aggregated packet, it's the NAL unit type of
  // the first NAL unit in the packet.
  uint8_t nalu_type;
  // The packetization type of this buffer - single, aggregated or fragmented.
  H265PacketizationTypes packetization_type;
  H265NaluInfo nalus[kMaxH265NalusPerPacket];
  size_t nalus_length;
  // The packetization mode of this transport. Packetization mode
  // determines which packetization types are allowed when packetizing.
  H265PacketizationMode packetization_mode;

  friend bool operator==(const RTPVideoHeaderH265& lhs,
                         const RTPVideoHeaderH265& rhs) {
    return lhs.nalu_type == rhs.nalu_type &&
           lhs.packetization_type == rhs.packetization_type &&
           std::equal(lhs.nalus, lhs.nalus + lhs.nalus_length, rhs.nalus,
                      rhs.nalus + rhs.nalus_length) &&
           lhs.packetization_mode == rhs.packetization_mode;
  }

  friend bool operator!=(const RTPVideoHeaderH265& lhs,
                         const RTPVideoHeaderH265& rhs) {
    return !(lhs == rhs);
  }
};

}  // namespace webrtc

#endif  // MODULES_VIDEO_CODING_CODECS_H265_INCLUDE_H265_GLOBALS_H_
