/*
 *  Copyright (c) 2014 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/rtp_rtcp/source/rtp_format_h265.h"

#include <string.h>

#include <cstddef>
#include <cstdint>
#include <iterator>
#include <memory>
#include <utility>
#include <vector>

#include "absl/types/optional.h"
#include "absl/types/variant.h"
#include "common_video/h265/h265_common.h"
#include "common_video/h265/h265_pps_parser.h"
#include "common_video/h265/h265_sps_parser.h"
#include "common_video/h265/h265_vps_parser.h"
#include "modules/rtp_rtcp/source/byte_io.h"
#include "modules/rtp_rtcp/source/rtp_packet_to_send.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"

namespace webrtc {
namespace {

// Unlike H.264, HEVC NAL header is 2-bytes.
static const size_t kH265NalHeaderSize = 2;
static const size_t kH265FuHeaderSize = 1;
static const size_t kH265LengthFieldSize = 2;

}  // namespace

RtpPacketizerH265::RtpPacketizerH265(rtc::ArrayView<const uint8_t> payload,
                                     PayloadSizeLimits limits,
                                     H265PacketizationMode packetization_mode)
    : limits_(limits), num_packets_left_(0) {
  // Guard against uninitialized memory in packetization_mode.
  RTC_CHECK(packetization_mode == H265PacketizationMode::NonInterleaved ||
            packetization_mode == H265PacketizationMode::SingleNalUnit);

  for (const auto& nalu : H265::FindNaluIndices(payload.data(), payload.size())) {
    input_fragments_.push_back(payload.subview(nalu.payload_start_offset, nalu.payload_size));
  }

  if (!GeneratePackets(packetization_mode)) {
    // If failed to generate all the packets, discard already generated
    // packets in case the caller would ignore return value and still try to
    // call NextPacket().
    num_packets_left_ = 0;
    while (!packets_.empty()) {
      packets_.pop();
    }
  }
}

RtpPacketizerH265::~RtpPacketizerH265() = default;

size_t RtpPacketizerH265::NumPackets() const {
  return num_packets_left_;
}

bool RtpPacketizerH265::GeneratePackets(H265PacketizationMode packetization_mode) {
  for (size_t i = 0; i < input_fragments_.size();) {
    switch (packetization_mode) {
      case H265PacketizationMode::SingleNalUnit:
        if (!PacketizeSingleNalu(i))
          return false;
        ++i;
        break;
      case H265PacketizationMode::NonInterleaved:
        int fragment_len = input_fragments_[i].size();
        int single_packet_capacity = limits_.max_payload_len;
        if (input_fragments_.size() == 1)
          single_packet_capacity -= limits_.single_packet_reduction_len;
        else if (i == 0)
          single_packet_capacity -= limits_.first_packet_reduction_len;
        else if (i + 1 == input_fragments_.size())
          single_packet_capacity -= limits_.last_packet_reduction_len;

        if (fragment_len > single_packet_capacity) {
          if (!PacketizeFU(i)) // Fragmentation Unit
            return false;
          ++i;
        } else {
          i = PacketizeAP(i); // Aggregation Packet
        }
        break;
    }
  }
  return true;
}

bool RtpPacketizerH265::PacketizeFU(size_t fragment_index) {
  // Fragment payload into packets (FU-A).
  rtc::ArrayView<const uint8_t> fragment = input_fragments_[fragment_index];
    int nal_type = H265::ParseNaluType(fragment[0]);
    RTC_LOG(LS_INFO) << "H265 nal_type PacketizeFU: " << nal_type;

  PayloadSizeLimits limits = limits_;
  // Leave room for the FU-A header.
  limits.max_payload_len -= kH265NalHeaderSize + kH265FuHeaderSize;
  // Update single/first/last packet reductions unless it is single/first/last
  // fragment.
  if (input_fragments_.size() != 1) {
    // if this fragment is put into a single packet, it might still be the
    // first or the last packet in the whole sequence of packets.
    if (fragment_index == input_fragments_.size() - 1) {
      limits.single_packet_reduction_len = limits_.last_packet_reduction_len;
    } else if (fragment_index == 0) {
      limits.single_packet_reduction_len = limits_.first_packet_reduction_len;
    } else {
      limits.single_packet_reduction_len = 0;
    }
  }
  if (fragment_index != 0)
    limits.first_packet_reduction_len = 0;
  if (fragment_index != input_fragments_.size() - 1)
    limits.last_packet_reduction_len = 0;

  // Strip out the original header.
  size_t payload_left = fragment.size() - kH265NalHeaderSize;
  int offset = kH265NalHeaderSize;

  std::vector<int> payload_sizes = SplitAboutEqually(payload_left, limits);
  if (payload_sizes.empty())
    return false;
    
  for (size_t i = 0; i < payload_sizes.size(); ++i) {
    int packet_length = payload_sizes[i];
    RTC_CHECK_GT(packet_length, 0);
    uint16_t header = (fragment[0] << 8) | fragment[1];
    packets_.push(PacketUnit(fragment.subview(offset, packet_length),
                             /*first_fragment=*/i == 0,
                             /*last_fragment=*/i == payload_sizes.size() - 1,
                             false, header));
    offset += packet_length;
    payload_left -= packet_length;
  }
  num_packets_left_ += payload_sizes.size();
  RTC_CHECK_EQ(0, payload_left);
  return true;
}

size_t RtpPacketizerH265::PacketizeAP(size_t fragment_index) {
  // Aggregate fragments into one packet (AP).
  size_t payload_size_left = limits_.max_payload_len;
  int aggregated_fragments = 0;
  size_t fragment_headers_length = 0;
  rtc::ArrayView<const uint8_t> fragment = input_fragments_[fragment_index];
    int nal_type = H265::ParseNaluType(fragment[0]);
    RTC_LOG(LS_INFO) << "H265 nal_type PacketizeAP: " << nal_type;
  RTC_CHECK_GE(payload_size_left, fragment.size());
  ++num_packets_left_;

  const bool has_first_fragment = fragment_index == 0;
  auto payload_size_needed = [&] {
    size_t fragment_size = fragment.size() + fragment_headers_length;
    bool has_last_fragment = fragment_index == input_fragments_.size() - 1;
    if (has_first_fragment && has_last_fragment) {
      return fragment_size + limits_.single_packet_reduction_len;
    } else if (has_first_fragment) {
      return fragment_size + limits_.first_packet_reduction_len;
    } else if (has_last_fragment) {
      return fragment_size + limits_.last_packet_reduction_len;
    } else {
      return fragment_size;
    }
  };

    uint16_t header = (fragment[0] << 8) | fragment[1];
    uint8_t payload_hdr_h = header >> 8;
    
    nal_type = H265::ParseNaluType(payload_hdr_h);
    RTC_LOG(LS_INFO) << "H265 nal_type PacketizeAP: " << nal_type;
  while (payload_size_left >= payload_size_needed()) {
    RTC_CHECK_GT(fragment.size(), 0);
    packets_.push(PacketUnit(fragment, aggregated_fragments == 0, false, true, header));
    payload_size_left -= fragment.size();
    payload_size_left -= fragment_headers_length;

    fragment_headers_length = kH265LengthFieldSize;
    // If we are going to try to aggregate more fragments into this packet
    // we need to add the STAP-A NALU header and a length field for the first
    // NALU of this packet.
    if (aggregated_fragments == 0)
      fragment_headers_length += kH265NalHeaderSize + kH265LengthFieldSize;
    ++aggregated_fragments;

    // Next fragment.
    ++fragment_index;
    if (fragment_index == input_fragments_.size())
      break;
    fragment = input_fragments_[fragment_index];
  }
  RTC_CHECK_GT(aggregated_fragments, 0);
  packets_.back().last_fragment = true;
  return fragment_index;
}

bool RtpPacketizerH265::PacketizeSingleNalu(size_t fragment_index) {
  // Add a single NALU to the queue, no aggregation.
  size_t payload_size_left = limits_.max_payload_len;
  if (input_fragments_.size() == 1)
    payload_size_left -= limits_.single_packet_reduction_len;
  else if (fragment_index == 0)
    payload_size_left -= limits_.first_packet_reduction_len;
  else if (fragment_index + 1 == input_fragments_.size())
    payload_size_left -= limits_.last_packet_reduction_len;
  rtc::ArrayView<const uint8_t> fragment = input_fragments_[fragment_index];
  if (payload_size_left < fragment.size()) {
    RTC_LOG(LS_ERROR) << "Failed to fit a fragment to packet in SingleNalu "
                         "packetization mode. Payload size left "
                      << payload_size_left << ", fragment length "
                      << fragment.size() << ", packet capacity "
                      << limits_.max_payload_len;
    return false;
  }
  RTC_CHECK_GT(fragment.size(), 0u);
  packets_.push(PacketUnit(fragment, true /* first */, true /* last */, false /* aggregated */, fragment[0]));
  ++num_packets_left_;
  return true;
}

bool RtpPacketizerH265::NextPacket(RtpPacketToSend* rtp_packet) {
  RTC_DCHECK(rtp_packet);
  if (packets_.empty()) {
    return false;
  }

  PacketUnit packet = packets_.front();
  if (packet.first_fragment && packet.last_fragment) {
    // Single NAL unit packet.
    size_t bytes_to_send = packet.source_fragment.size();
    uint8_t* buffer = rtp_packet->AllocatePayload(bytes_to_send);
    memcpy(buffer, packet.source_fragment.data(), bytes_to_send);
    packets_.pop();
    input_fragments_.pop_front();
  } else if (packet.aggregated) {
    NextAggregatePacket(rtp_packet);
  } else {
    NextFragmentPacket(rtp_packet);
  }
  rtp_packet->SetMarker(packets_.empty());
  --num_packets_left_;
  return true;
}

void RtpPacketizerH265::NextAggregatePacket(RtpPacketToSend* rtp_packet) {
  // Reserve maximum available payload, set actual payload size later.
  size_t payload_capacity = rtp_packet->FreeCapacity();
  RTC_CHECK_GE(payload_capacity, kH265NalHeaderSize);
  uint8_t* buffer = rtp_packet->AllocatePayload(payload_capacity);
  RTC_DCHECK(buffer);
  PacketUnit* packet = &packets_.front();
  RTC_CHECK(packet->first_fragment);
    
//    +---------------+---------------+
//    |0|1|2|3|4|5|6|7|0|1|2|3|4|5|6|7|
//    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//    |F|   Type    |  LayerId  | TID |
//    +-------------+-----------------+
//    Figure 1: The Structure of the HEVC NAL Unit Header
    
//     0                   1                   2                   3
//     0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
//    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//    |                          RTP Header                           |
//    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//    |       PayloadHdr (Type=48)    |           NALU 1 Size         |
//    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//    |           NALU 1 HDR          |                               |
//    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+           NALU 1 Data         |
//    |        ...                                                    |
//    |                                                               |
//    +               +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//    |       . . .   |            NALU 2 Size       |   NALU 2 HDR   |
//    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//    |   NALU 2 HDR  |                                               |
//    +-+-+-+-+-+-+-+-+                 NALU 2 Data                   |
//    |                    ...                                        |
//    |                               +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//    |                               :...OPTIONAL RTP padding        |
//    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//    Figure 7: An Example of an AP Packet Containing Two Aggregation Units without the DONL and DOND Fields
    
    uint8_t payload_hdr_h = packet->header >> 8;
    uint8_t payload_hdr_l = packet->header & 0xFF;
    uint8_t layer_id_h = payload_hdr_h & kH265LayerIDHMask;
    
    int nal_type = H265::ParseNaluType(payload_hdr_h);
    RTC_LOG(LS_INFO) << "H265 nal_type AP: " << nal_type;
    
    payload_hdr_h = (payload_hdr_h & kH265TypeMaskN) | (H265::NaluType::kAP << 1) | layer_id_h;
    
    buffer[0] = payload_hdr_h;
    buffer[1] = payload_hdr_l;
    
  size_t index = kH265NalHeaderSize;
  bool is_last_fragment = packet->last_fragment;
  while (packet->aggregated) {
    rtc::ArrayView<const uint8_t> fragment = packet->source_fragment;
    RTC_CHECK_LE(index + kH265LengthFieldSize + fragment.size(), payload_capacity);
    // Add NAL unit length field.
    ByteWriter<uint16_t>::WriteBigEndian(&buffer[index], fragment.size());
    index += kH265LengthFieldSize;
    // Add NAL unit.
    memcpy(&buffer[index], fragment.data(), fragment.size());
    index += fragment.size();
    packets_.pop();
    input_fragments_.pop_front();
    if (is_last_fragment)
      break;
    packet = &packets_.front();
    is_last_fragment = packet->last_fragment;
  }
  RTC_CHECK(is_last_fragment);
  rtp_packet->SetPayloadSize(index);
}

void RtpPacketizerH265::NextFragmentPacket(RtpPacketToSend* rtp_packet) {
  PacketUnit* packet = &packets_.front();
  // NAL unit fragmented over multiple packets (FU).
  // We do not send original NALU header, so it will be replaced by the
  // FU indicator header of the first packet.
   
//    +---------------+---------------+
//    |0|1|2|3|4|5|6|7|0|1|2|3|4|5|6|7|
//    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//    |F|   Type    |  LayerId  | TID |
//    +-------------+-----------------+
//    Figure 1: The Structure of the HEVC NAL Unit Header
    
//    +---------------+
//    |0|1|2|3|4|5|6|7|
//    +-+-+-+-+-+-+-+-+
//    |S|E|  FuType   |
//    +---------------+
//    Figure 10: The Structure of FU Header
    
//     0                   1                   2                   3
//     0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
//    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//    |     PayloadHdr (Type=49)      |    FU header  |   DONL (cond) |
//    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//    |   DONL (cond) |                                               |
//    |-+-+-+-+-+-+-+-+                                               |
//    |                          FU payload                           |
//    |                                                               |
//    |                               +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//    |                               :...OPTIONAL RTP padding        |
//    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//                    Figure 9: The Structure of an FU
    
    uint8_t payload_hdr_h = packet->header >> 8;  // 1-bit F, 6-bit type, 1-bit layerID highest-bit
    uint8_t payload_hdr_l = packet->header & 0xFF;
    uint8_t layer_id_h = payload_hdr_h & kH265LayerIDHMask;
    uint8_t fu_header = 0; // S | E |6 bit type.
    fu_header |= (packet->first_fragment ? kH265SBit : 0);
    fu_header |= (packet->last_fragment ? kH265EBit : 0);
    int type = (payload_hdr_h & kH265TypeMask) >> 1;
    RTC_LOG(LS_INFO) << "H265 nal_type fu: " << type;
    fu_header |= type;
    payload_hdr_h = (payload_hdr_h & kH265TypeMaskN) | (H265::NaluType::kFU << 1) | layer_id_h;
    
  rtc::ArrayView<const uint8_t> fragment = packet->source_fragment;
  uint8_t* buffer = rtp_packet->AllocatePayload(kH265NalHeaderSize + kH265FuHeaderSize + fragment.size());
  buffer[0] = payload_hdr_h;
  buffer[1] = payload_hdr_l;
  buffer[2] = fu_header;
  memcpy(buffer + kH265NalHeaderSize + kH265FuHeaderSize, fragment.data(), fragment.size());
  if (packet->last_fragment)
    input_fragments_.pop_front();
  packets_.pop();
}

}  // namespace webrtc
