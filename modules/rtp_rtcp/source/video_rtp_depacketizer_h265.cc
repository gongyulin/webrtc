/*
 *  Copyright (c) 2020 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/rtp_rtcp/source/video_rtp_depacketizer_h265.h"

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "absl/types/optional.h"
#include "absl/types/variant.h"
#include "common_video/h265/h265_common.h"
#include "common_video/h265/h265_pps_parser.h"
#include "common_video/h265/h265_sps_parser.h"
#include "common_video/h265/h265_vps_parser.h"
#include "common_video/h265/h265_bitstream_parser.h"
#include "modules/rtp_rtcp/source/byte_io.h"
#include "modules/rtp_rtcp/source/rtp_format_h265.h"
#include "modules/rtp_rtcp/source/video_rtp_depacketizer.h"
#include "rtc_base/checks.h"
#include "rtc_base/copy_on_write_buffer.h"
#include "rtc_base/logging.h"

namespace webrtc {
namespace {

constexpr size_t kH265NalHeaderSize = 2;
constexpr size_t kH265FuHeaderSize = 1;
constexpr size_t kH265LengthFieldSize = 2;
constexpr size_t kH265APHeaderSize = kH265NalHeaderSize + kH265LengthFieldSize;

// TODO(pbos): Avoid parsing this here as well as inside the jitter buffer.
bool ParseAPStartOffsets(const uint8_t* nalu_ptr,
                            size_t length_remaining,
                            std::vector<size_t>* offsets) {
  size_t offset = 0;
  while (length_remaining > 0) {
    // Buffer doesn't contain room for additional nalu length.
    if (length_remaining < sizeof(uint16_t))
      return false;
    uint16_t nalu_size = ByteReader<uint16_t>::ReadBigEndian(nalu_ptr);
    nalu_ptr += sizeof(uint16_t);
    length_remaining -= sizeof(uint16_t);
    if (nalu_size > length_remaining)
      return false;
    nalu_ptr += nalu_size;
    length_remaining -= nalu_size;

    offsets->push_back(offset + kH265APHeaderSize);
    offset += kH265LengthFieldSize + nalu_size;
  }
  return true;
}

absl::optional<VideoRtpDepacketizer::ParsedRtpPayload> ProcessAPOrSingleNalu(
    rtc::CopyOnWriteBuffer rtp_payload) {
  const uint8_t* const payload_data = rtp_payload.cdata();
  absl::optional<VideoRtpDepacketizer::ParsedRtpPayload> parsed_payload(
      absl::in_place);
//  bool modified_buffer = false;
  parsed_payload->video_payload = rtp_payload;
  parsed_payload->video_header.width = 0;
  parsed_payload->video_header.height = 0;
  parsed_payload->video_header.codec = kVideoCodecH265;
  parsed_payload->video_header.simulcastIdx = 0;
  parsed_payload->video_header.is_first_packet_in_frame = true;
  auto& H265_header = parsed_payload->video_header.video_type_header
                          .emplace<RTPVideoHeaderH265>();

  const uint8_t* nalu_start = payload_data + kH265NalHeaderSize;
  const size_t nalu_length = rtp_payload.size() - kH265NalHeaderSize;
  int nal_type = (payload_data[0] & kH265TypeMask) >> 1;
  std::vector<size_t> nalu_start_offsets;
  if (nal_type == H265::NaluType::kAP) {
    // Skip the AP header (AP NAL type + length).
    if (rtp_payload.size() <= kH265APHeaderSize) {
      RTC_LOG(LS_ERROR) << "StapA header truncated.";
      return absl::nullopt;
    }

    if (!ParseAPStartOffsets(nalu_start, nalu_length, &nalu_start_offsets)) {
      RTC_LOG(LS_ERROR) << "StapA packet with incorrect NALU packet lengths.";
      return absl::nullopt;
    }

    H265_header.packetization_type = kH265AP;
    // nal_type = (payload_data[kAPHeaderSize] & kH265TypeMask) >> 1;
  } else {
    H265_header.packetization_type = kH265SingleNalu;
    nalu_start_offsets.push_back(0);
  }
  H265_header.nalu_type = nal_type;
  parsed_payload->video_header.frame_type = VideoFrameType::kVideoFrameDelta;

  nalu_start_offsets.push_back(rtp_payload.size() +
                               kH265LengthFieldSize);  // End offset.
  for (size_t i = 0; i < nalu_start_offsets.size() - 1; ++i) {
    size_t start_offset = nalu_start_offsets[i];
    // End offset is actually start offset for next unit, excluding length field
    // so remove that from this units length.
    size_t end_offset = nalu_start_offsets[i + 1] - kH265LengthFieldSize;
    if (end_offset - start_offset < kH265APHeaderSize) {
      RTC_LOG(LS_ERROR) << "AP packet too short";
      return absl::nullopt;
    }

    H265NaluInfo nalu;
    int nalType = (payload_data[start_offset] & kH265TypeMask) >> 1;
    nalu.type = nalType;
    nalu.vps_id = -1;
    nalu.sps_id = -1;
    nalu.pps_id = -1;
    start_offset += kH265NalHeaderSize;
      
    RTC_LOG(LS_INFO) << "H265 nal_type: " << nalu.type;
    switch (nalu.type) {
        case H265::NaluType::kVps: {
            absl::optional<H265VpsParser::VpsState> vps =
              H265VpsParser::ParseVps(&payload_data[start_offset], end_offset - start_offset);
            if (vps) {
                nalu.vps_id = vps->id;
            } else {
                RTC_LOG(LS_WARNING) << "Failed to parse VPS id from VPS slice.";
            }
            break;
        }
      case H265::NaluType::kSps: {
        // Check if VUI is present in SPS and if it needs to be modified to
        // avoid
        // excessive decoder latency.

        // Copy any previous data first (likely just the first header).
        rtc::Buffer output_buffer;
        if (start_offset)
          output_buffer.AppendData(payload_data, start_offset);

        absl::optional<H265SpsParser::SpsState> sps = H265SpsParser::ParseSps(&payload_data[start_offset], end_offset - start_offset);
        if (sps) {
          parsed_payload->video_header.width = sps->width;
          parsed_payload->video_header.height = sps->height;
          nalu.sps_id = sps->sps_id;
          nalu.vps_id = sps->vps_id;
        } else {
          RTC_LOG(LS_WARNING) << "Failed to parse SPS id from SPS slice.";
        }
        parsed_payload->video_header.frame_type =
            VideoFrameType::kVideoFrameKey;
        break;
      }
      case H265::NaluType::kPps: {
        uint32_t pps_id;
        uint32_t sps_id;
        if (H265PpsParser::ParsePpsIds(&payload_data[start_offset],
                                   end_offset - start_offset, &pps_id,
                                   &sps_id)) {
          nalu.pps_id = pps_id;
          nalu.sps_id = sps_id;
        } else {
          RTC_LOG(LS_WARNING)
              << "Failed to parse PPS id and SPS id from PPS slice.";
        }
        break;
      }
      case H265::NaluType::kIdrNLp:
      case H265::NaluType::kIdrWRadl:
      case H265::NaluType::kCra:
        parsed_payload->video_header.frame_type = VideoFrameType::kVideoFrameKey;
        [[fallthrough]];
      case H265::NaluType::kTrailN:
      case H265::NaluType::kTrailR:
        {
            absl::optional<uint32_t> pps_id = H265BitstreamParser::ParsePpsIdFromSliceSegmentLayerRbsp(&payload_data[start_offset], end_offset - start_offset, nalu.type);
            if (pps_id) {
                nalu.pps_id = *pps_id;
            } else {
                RTC_LOG(LS_WARNING) << "Failed to parse PPS id from slice of type: "
                << static_cast<int>(nalu.type);
            }
            break;
        }
      // Slices below don't contain VPS SPS or PPS ids.
        case H265::NaluType::kAud:
        case H265::NaluType::kTsaN:
        case H265::NaluType::kTsaR:
        case H265::NaluType::kStsaN:
        case H265::NaluType::kStsaR:
        case H265::NaluType::kRadlN:
        case H265::NaluType::kRadlR:
        case H265::NaluType::kBlaWLp:
        case H265::NaluType::kBlaWRadl:
        case H265::NaluType::kPrefixSei:
        case H265::NaluType::kSuffixSei:
            break;
        case H265::NaluType::kAP:
        case H265::NaluType::kFU:
        RTC_LOG(LS_WARNING) << "Unexpected AP or FU received.";
        return absl::nullopt;
    }

    if (H265_header.nalus_length == kMaxNalusPerPacket) {
      RTC_LOG(LS_WARNING)
          << "Received packet containing more than " << kMaxNalusPerPacket
          << " NAL units. Will not keep track sps and pps ids for all of them.";
    } else {
      H265_header.nalus[H265_header.nalus_length++] = nalu;
    }
  }

  return parsed_payload;
}

absl::optional<VideoRtpDepacketizer::ParsedRtpPayload> ParseFuNalu(
    rtc::CopyOnWriteBuffer rtp_payload) {
  if (rtp_payload.size() < kH265NalHeaderSize + kH265FuHeaderSize) {
    RTC_LOG(LS_ERROR) << "FU-A NAL units truncated.";
    return absl::nullopt;
  }
  absl::optional<VideoRtpDepacketizer::ParsedRtpPayload> parsed_payload(
      absl::in_place);
    
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

    uint8_t f = rtp_payload.cdata()[0] & kH265FBit;
    uint8_t layer_id_h = rtp_payload.cdata()[0] & kH265LayerIDHMask;
    uint8_t layer_id_l_unshifted = rtp_payload.cdata()[1] & kH265LayerIDLMask;
    uint8_t tid = rtp_payload.cdata()[1] & kH265TIDMask;
    int original_nal_type = rtp_payload.cdata()[2] & kH265TypeMaskInFuHeader;
    bool first_fragment = rtp_payload.cdata()[2] & kH265SBit;
    
    RTC_LOG(LS_INFO) << "H265 nal_type original_nal_type: " << original_nal_type;
    
  H265NaluInfo nalu;
  nalu.type = original_nal_type;
  nalu.vps_id = -1;
  nalu.sps_id = -1;
  nalu.pps_id = -1;
  if (first_fragment) {
    absl::optional<uint32_t> pps_id = H265BitstreamParser::ParsePpsIdFromSliceSegmentLayerRbsp(rtp_payload.cdata() + kH265NalHeaderSize + kH265FuHeaderSize, rtp_payload.size() - kH265FuHeaderSize, nalu.type);
    if (pps_id) {
      nalu.pps_id = *pps_id;
    } else {
      RTC_LOG(LS_WARNING)
          << "Failed to parse PPS from first fragment of FU NAL "
             "unit with original type: "
          << static_cast<int>(nalu.type);
    }
      
    rtp_payload = rtp_payload.Slice(kH265FuHeaderSize, rtp_payload.size() - kH265FuHeaderSize);
    rtp_payload.MutableData()[0] = f | original_nal_type << 1 | layer_id_h;
    rtp_payload.MutableData()[1] = layer_id_l_unshifted | tid;
    parsed_payload->video_payload = std::move(rtp_payload);
  } else {
    parsed_payload->video_payload = rtp_payload.Slice(kH265NalHeaderSize + kH265FuHeaderSize, rtp_payload.size() - kH265NalHeaderSize - kH265FuHeaderSize);
    parsed_payload->video_payload = std::move(rtp_payload);
  }

  if (original_nal_type == H265::NaluType::kIdrNLp ||
      original_nal_type == H265::NaluType::kIdrWRadl ||
      original_nal_type == H265::NaluType::kCra) {
    parsed_payload->video_header.frame_type = VideoFrameType::kVideoFrameKey;
  } else {
    parsed_payload->video_header.frame_type = VideoFrameType::kVideoFrameDelta;
  }
  parsed_payload->video_header.width = 0;
  parsed_payload->video_header.height = 0;
  parsed_payload->video_header.codec = kVideoCodecH265;
  parsed_payload->video_header.simulcastIdx = 0;
  parsed_payload->video_header.is_first_packet_in_frame = first_fragment;
  auto& H265_header = parsed_payload->video_header.video_type_header.emplace<RTPVideoHeaderH265>();
  H265_header.packetization_type = kH265FU;
  H265_header.nalu_type = original_nal_type;
  if (first_fragment) {
    H265_header.nalus[H265_header.nalus_length] = nalu;
    H265_header.nalus_length = 1;
  }
  return parsed_payload;
}

}  // namespace

absl::optional<VideoRtpDepacketizer::ParsedRtpPayload>
VideoRtpDepacketizerH265::Parse(rtc::CopyOnWriteBuffer rtp_payload) {
  if (rtp_payload.size() == 0) {
    RTC_LOG(LS_ERROR) << "Empty payload.";
    return absl::nullopt;
  }

  int nal_type = (rtp_payload.cdata()[0] & kH265TypeMask) >> 1;

  if (nal_type == H265::NaluType::kFU) {
    // Fragmented NAL units (FU).
    return ParseFuNalu(std::move(rtp_payload));
  } else {
    // We handle AP and single NALU's the same way here. The jitter buffer
    // will depacketize the AP into NAL units later.
    return ProcessAPOrSingleNalu(std::move(rtp_payload));
  }
}

}  // namespace webrtc
