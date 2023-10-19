/*
 *  Copyright 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#import <Foundation/Foundation.h>

#import "RTCMacros.h"

RTC_OBJC_EXPORT extern NSString *const kRTCVideoCodecH265Name;
//RTC_OBJC_EXPORT extern NSString *const kRTCLevel31ConstrainedHigh;
//RTC_OBJC_EXPORT extern NSString *const kRTCLevel31ConstrainedBaseline;
//RTC_OBJC_EXPORT extern NSString *const kRTCMaxSupportedH265ProfileLevelConstrainedHigh;
//RTC_OBJC_EXPORT extern NSString *const kRTCMaxSupportedH265ProfileLevelConstrainedBaseline;

/** H265 Profiles and levels. */
typedef NS_ENUM(NSUInteger, RTCH265Profile) {
  RTCH265ProfileConstrainedBaseline,
  RTCH265ProfileBaseline,
  RTCH265ProfileMain,
  RTCH265ProfileConstrainedHigh,
  RTCH265ProfileHigh,
};

typedef NS_ENUM(NSUInteger, RTCH265Level) {
  RTCH265Level1_b = 0,
  RTCH265Level1 = 10,
  RTCH265Level1_1 = 11,
  RTCH265Level1_2 = 12,
  RTCH265Level1_3 = 13,
  RTCH265Level2 = 20,
  RTCH265Level2_1 = 21,
  RTCH265Level2_2 = 22,
  RTCH265Level3 = 30,
  RTCH265Level3_1 = 31,
  RTCH265Level3_2 = 32,
  RTCH265Level4 = 40,
  RTCH265Level4_1 = 41,
  RTCH265Level4_2 = 42,
  RTCH265Level5 = 50,
  RTCH265Level5_1 = 51,
  RTCH265Level5_2 = 52
};

RTC_OBJC_EXPORT
@interface RTC_OBJC_TYPE (RTCH265ProfileLevelId) : NSObject

@property(nonatomic, readonly) RTCH265Profile profile;
@property(nonatomic, readonly) RTCH265Level level;
@property(nonatomic, readonly) NSString *hexString;

- (instancetype)initWithHexString:(NSString *)hexString;
- (instancetype)initWithProfile:(RTCH265Profile)profile level:(RTCH265Level)level;

@end
