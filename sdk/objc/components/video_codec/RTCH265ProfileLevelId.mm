/*
 *  Copyright 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 *
 */

#import "RTCH265ProfileLevelId.h"

#import "helpers/NSString+StdString.h"
//#if defined(WEBRTC_IOS)
//#import "UIDevice+H264Profile.h"
//#endif

//#include "api/video_codecs/H264_profile_level_id.h"UIDevice+H264Profile
#include "api/video_codecs/h265_profile_tier_level.h"
#include "media/base/media_constants.h"

namespace {

//NSString *MaxSupportedProfileLevelConstrainedHigh();
//NSString *MaxSupportedProfileLevelConstrainedBaseline();

}  // namespace

NSString *const kRTCVideoCodecH265Name = @(cricket::kH265CodecName);
//NSString *const kRTCLevel31ConstrainedHigh = @"640c1f";
//NSString *const kRTCLevel31ConstrainedBaseline = @"42e01f";
//NSString *const kRTCMaxSupportedH265ProfileLevelConstrainedHigh =
//    MaxSupportedProfileLevelConstrainedHigh();
//NSString *const kRTCMaxSupportedH265ProfileLevelConstrainedBaseline =
//    MaxSupportedProfileLevelConstrainedBaseline();

namespace {

//#if defined(WEBRTC_IOS)
//
//NSString *MaxSupportedLevelForProfile(webrtc::H265Profile profile) {
//  const absl::optional<webrtc::H265ProfileLevelId> profileLevelId =
//      [UIDevice maxSupportedH265Profile];
//  if (profileLevelId && profileLevelId->profile >= profile) {
//    const absl::optional<std::string> profileString =
//        H265ProfileLevelIdToString(webrtc::H265ProfileLevelId(profile, profileLevelId->level));
//    if (profileString) {
//      return [NSString stringForStdString:*profileString];
//    }
//  }
//  return nil;
//}
//#endif

//NSString *MaxSupportedProfileLevelConstrainedBaseline() {
//#if defined(WEBRTC_IOS)
//  NSString *profile = MaxSupportedLevelForProfile(webrtc::H265Profile::kProfileConstrainedBaseline);
//  if (profile != nil) {
//    return profile;
//  }
//#endif
//  return kRTCLevel31ConstrainedBaseline;
//}

//NSString *MaxSupportedProfileLevelConstrainedHigh() {
//#if defined(WEBRTC_IOS)
//  NSString *profile = MaxSupportedLevelForProfile(webrtc::H265Profile::kProfileConstrainedHigh);
//  if (profile != nil) {
//    return profile;
//  }
//#endif
//  return kRTCLevel31ConstrainedHigh;
//}

}  // namespace

@interface RTC_OBJC_TYPE (RTCH265ProfileLevelId)
()

    @property(nonatomic, assign) RTCH265Profile profile;
@property(nonatomic, assign) RTCH265Level level;
@property(nonatomic, strong) NSString *hexString;

@end

@implementation RTC_OBJC_TYPE (RTCH265ProfileLevelId)

@synthesize profile = _profile;
@synthesize level = _level;
@synthesize hexString = _hexString;

- (instancetype)initWithHexString:(NSString *)hexString {
  if (self = [super init]) {
    self.hexString = hexString;

//    absl::optional<webrtc::H265ProfileLevelId> profile_level_id =
//        webrtc::ParseH265ProfileLevelId([hexString cStringUsingEncoding:NSUTF8StringEncoding]);
//    if (profile_level_id.has_value()) {
//      self.profile = static_cast<RTCH265Profile>(profile_level_id->profile);
//      self.level = static_cast<RTCH265Level>(profile_level_id->level);
//    }
  }
  return self;
}

- (instancetype)initWithProfile:(RTCH265Profile)profile level:(RTCH265Level)level {
  if (self = [super init]) {
    self.profile = profile;
    self.level = level;

//    absl::optional<std::string> hex_string =
//        webrtc::H265ProfileLevelIdToString(webrtc::H265ProfileLevelId(
//            static_cast<webrtc::H265Profile>(profile), static_cast<webrtc::H265Level>(level)));
//    self.hexString =
//        [NSString stringWithCString:hex_string.value_or("").c_str() encoding:NSUTF8StringEncoding];
  }
  return self;
}

@end
