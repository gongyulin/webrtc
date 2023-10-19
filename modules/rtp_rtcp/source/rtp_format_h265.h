/*
 *  Copyright (c) 2014 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_RTP_RTCP_SOURCE_RTP_FORMAT_H265_H_
#define MODULES_RTP_RTCP_SOURCE_RTP_FORMAT_H265_H_

#include <stddef.h>
#include <stdint.h>

#include <deque>
#include <memory>
#include <queue>

#include "api/array_view.h"
#include "modules/rtp_rtcp/source/rtp_format.h"
#include "modules/rtp_rtcp/source/rtp_packet_to_send.h"
#include "modules/video_coding/codecs/h265/include/h265_globals.h"
#include "rtc_base/buffer.h"

namespace webrtc {

// Bit masks for NAL (F, NRI, Type) indicators.
//    +---------------+---------------+
//    |0|1|2|3|4|5|6|7|0|1|2|3|4|5|6|7|
//    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//    |F|   Type    |  LayerId  | TID |
//    +-------------+-----------------+
//    Figure 1: The Structure of the HEVC NAL Unit Header
constexpr uint8_t kH265FBit = 0x80;
constexpr uint8_t kH265TypeMask = 0x7E;
constexpr uint8_t kH265LayerIDHMask = 0x01;
constexpr uint8_t kH265LayerIDLMask = 0xF8;
constexpr uint8_t kH265TIDMask = 0x07;
constexpr uint8_t kH265TypeMaskN = 0x81;
constexpr uint8_t kH265TypeMaskInFuHeader = 0x3F;

// Bit masks for FU headers.
//    +---------------+
//    |0|1|2|3|4|5|6|7|
//    +-+-+-+-+-+-+-+-+
//    |S|E|  FuType   |
//    +---------------+
//    Figure 10: The Structure of FU Header
constexpr uint8_t kH265SBit = 0x80;
constexpr uint8_t kH265EBit = 0x40;
constexpr uint8_t kH265FuTypeBit = 0x3F;

class RtpPacketizerH265 : public RtpPacketizer {
 public:
  // Initialize with payload from encoder.
  // The payload_data must be exactly one encoded H265 frame.
  RtpPacketizerH265(rtc::ArrayView<const uint8_t> payload,
                    PayloadSizeLimits limits,
                    H265PacketizationMode packetization_mode);

  ~RtpPacketizerH265() override;

  RtpPacketizerH265(const RtpPacketizerH265&) = delete;
  RtpPacketizerH265& operator=(const RtpPacketizerH265&) = delete;

  size_t NumPackets() const override;

  // Get the next payload with H265 payload header.
  // Write payload and set marker bit of the `packet`.
  // Returns true on success, false otherwise.
  bool NextPacket(RtpPacketToSend* rtp_packet) override;

 private:
  // A packet unit (H265 packet), to be put into an RTP packet:
  // If a NAL unit is too large for an RTP packet, this packet unit will
  // represent a FU-A packet of a single fragment of the NAL unit.
  // If a NAL unit is small enough to fit within a single RTP packet, this
  // packet unit may represent a single NAL unit or a STAP-A packet, of which
  // there may be multiple in a single RTP packet (if so, aggregated = true).
  struct PacketUnit {
    PacketUnit(rtc::ArrayView<const uint8_t> source_fragment,
               bool first_fragment,
               bool last_fragment,
               bool aggregated,
               uint16_t header)
        : source_fragment(source_fragment),
          first_fragment(first_fragment),
          last_fragment(last_fragment),
          aggregated(aggregated),
          header(header) {
              
          }

    rtc::ArrayView<const uint8_t> source_fragment;
    bool first_fragment;
    bool last_fragment;
    bool aggregated;
    uint16_t header;
  };

  bool GeneratePackets(H265PacketizationMode packetization_mode);
  bool PacketizeFU(size_t fragment_index);
  size_t PacketizeAP(size_t fragment_index);
  bool PacketizeSingleNalu(size_t fragment_index);

  void NextAggregatePacket(RtpPacketToSend* rtp_packet);
  void NextFragmentPacket(RtpPacketToSend* rtp_packet);

  const PayloadSizeLimits limits_;
  size_t num_packets_left_;
  std::deque<rtc::ArrayView<const uint8_t>> input_fragments_;
  std::queue<PacketUnit> packets_;
};
}  // namespace webrtc
#endif  // MODULES_RTP_RTCP_SOURCE_RTP_FORMAT_H265_H_
