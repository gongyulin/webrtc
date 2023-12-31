/*
 *  Copyright 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#import "RTCCodecSpecificInfoH265+Private.h"

#import "RTCH265ProfileLevelId.h"

// H265 specific settings.
@implementation RTC_OBJC_TYPE (RTCCodecSpecificInfoH265)

@synthesize packetizationMode = _packetizationMode;

- (webrtc::CodecSpecificInfo)nativeCodecSpecificInfo {
  webrtc::CodecSpecificInfo codecSpecificInfo;
  codecSpecificInfo.codecType = webrtc::kVideoCodecH265;
  codecSpecificInfo.codecSpecific.H265.packetization_mode =
      (webrtc::H265PacketizationMode)_packetizationMode;

  return codecSpecificInfo;
}

@end
