// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
// trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

#pragma once

// UrlabRclCore is the single UE-agnostic seam that owns every rcl / rosidl call
// in the project. This header exposes only opaque handles and POD/C-friendly
// free functions: no UE types, no rcl types, no rosidl types, no STL in any
// signature. It is includable from UE module code and from the standalone ROS
// workspace harness alike; its only include is <stdint.h>.
//
// Return convention: functions returning int use 0 = ok, negative = a mapped
// rcl_ret_t. Create* functions return a handle pointer, null on failure.
// UrlabRcl_LastError() returns the rcutils error string from the most recent
// failed call, for logging. Every Create* has a matching Destroy that takes its
// handle; all destroy/shutdown functions are null-safe.
//
// Time is sim time in nanoseconds (int64_t); the core splits it into sec/nsec.
// Quaternions are xyzw ordered.
//
// Threading: the core takes no locks. Create and destroy all handles from one
// thread, serialize calls per handle, and note that UrlabRcl_SpinSome runs
// subscription callbacks on its caller's thread.

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Opaque handles. Definitions live only in UrlabRclCore.cpp.
struct UrlabRclContext;         // rcl init options + context + one node
struct UrlabRclJointStatePub;
struct UrlabRclImuPub;
struct UrlabRclTfPub;
struct UrlabRclTwistStampedPub;
struct UrlabRclClockPub;
struct UrlabRclImagePub;
struct UrlabRclCtrlPub;
struct UrlabRclCtrlSub;
struct UrlabRclTwistSub;

// --- Context ---------------------------------------------------------------
// DomainId -1 = use the ROS_DOMAIN_ID environment variable. Returns null on
// failure (query UrlabRcl_LastError for the reason).
struct UrlabRclContext* UrlabRcl_Init(const char* NodeName,
    const char* NodeNamespace, int32_t DomainId);
void UrlabRcl_Shutdown(struct UrlabRclContext* Ctx);   // fini node/context, reverse order
const char* UrlabRcl_DistroName();                     // compile-time pin, for the facts file
const char* UrlabRcl_LastError();

// --- Publishers ------------------------------------------------------------
// Joint-name arrays are copied at create time (sized to the model); message
// structs are preallocated per handle.
struct UrlabRclJointStatePub* UrlabRcl_CreateJointStatePub(struct UrlabRclContext* Ctx,
    const char* Topic, const char** JointNames, int32_t JointCount);
int UrlabRcl_PublishJointState(struct UrlabRclJointStatePub* Pub,
    const double* Positions, const double* Velocities, const double* Efforts,
    int32_t Count, int64_t SimTimeNs);                 // Velocities/Efforts may be null
void UrlabRcl_DestroyJointStatePub(struct UrlabRclJointStatePub* Pub);

struct UrlabRclImuPub* UrlabRcl_CreateImuPub(struct UrlabRclContext* Ctx,
    const char* Topic, const char* FrameId);
int UrlabRcl_PublishImu(struct UrlabRclImuPub* Pub, const double AngularVel[3],
    const double LinearAccel[3], const double OrientationXyzw[4],
    int64_t SimTimeNs);                                // any array may be null (unpaired gyro)
void UrlabRcl_DestroyImuPub(struct UrlabRclImuPub* Pub);

struct UrlabRclTfPub* UrlabRcl_CreateTfPub(struct UrlabRclContext* Ctx, int32_t bStatic);
    // bStatic != 0: /tf_static with transient-local QoS; else /tf
int UrlabRcl_PublishTf(struct UrlabRclTfPub* Pub, const char** ParentFrameIds,
    const char** ChildFrameIds, const double* TranslationsXyz /* 3*Count */,
    const double* RotationsXyzw /* 4*Count */, int32_t Count,
    int64_t SimTimeNs);
void UrlabRcl_DestroyTfPub(struct UrlabRclTfPub* Pub);

struct UrlabRclTwistStampedPub* UrlabRcl_CreateTwistStampedPub(struct UrlabRclContext* Ctx,
    const char* Topic, const char* FrameId);
int UrlabRcl_PublishTwistStamped(struct UrlabRclTwistStampedPub* Pub,
    const double Linear[3], const double Angular[3], int64_t SimTimeNs);
void UrlabRcl_DestroyTwistStampedPub(struct UrlabRclTwistStampedPub* Pub);

struct UrlabRclClockPub* UrlabRcl_CreateClockPub(struct UrlabRclContext* Ctx); // topic /clock
int UrlabRcl_PublishClock(struct UrlabRclClockPub* Pub, int64_t SimTimeNs);
void UrlabRcl_DestroyClockPub(struct UrlabRclClockPub* Pub);

struct UrlabRclImagePub* UrlabRcl_CreateImagePub(struct UrlabRclContext* Ctx,
    const char* Topic, const char* FrameId, int32_t Width, int32_t Height,
    const char* Encoding);                             // ROS encoding string, e.g. "rgb8"/"bgra8"
int UrlabRcl_PublishImage(struct UrlabRclImagePub* Pub, const uint8_t* Data,
    int32_t StrideBytes, int64_t SimTimeNs);
void UrlabRcl_DestroyImagePub(struct UrlabRclImagePub* Pub);

// Control injection: the publish counterpart to the /<art>/cmd_ctrl
// subscription below (std_msgs/Float64MultiArray). It exists so control can be
// injected in-process for loopback and tests without a second ROS node.
struct UrlabRclCtrlPub* UrlabRcl_CreateCtrlPub(struct UrlabRclContext* Ctx,
    const char* Topic);
int UrlabRcl_PublishCtrl(struct UrlabRclCtrlPub* Pub, const double* Values,
    int32_t Count);
void UrlabRcl_DestroyCtrlPub(struct UrlabRclCtrlPub* Pub);

// --- Subscriptions ---------------------------------------------------------
// Callbacks fire inside UrlabRcl_SpinSome on its caller's thread; the core does
// no queuing beyond what the rmw layer holds.
typedef void (*UrlabRclCtrlCallback)(const double* Values, int32_t Count,
    void* User);
struct UrlabRclCtrlSub* UrlabRcl_CreateCtrlSub(struct UrlabRclContext* Ctx,
    const char* Topic, UrlabRclCtrlCallback Callback, void* User);
    // std_msgs/Float64MultiArray, the /<art>/cmd_ctrl shape
void UrlabRcl_DestroyCtrlSub(struct UrlabRclCtrlSub* Sub);

typedef void (*UrlabRclTwistCallback)(const double Linear[3],
    const double Angular[3], void* User);
struct UrlabRclTwistSub* UrlabRcl_CreateTwistSub(struct UrlabRclContext* Ctx,
    const char* Topic, UrlabRclTwistCallback Callback, void* User);
    // geometry_msgs/Twist, the /<art>/cmd_vel shape
void UrlabRcl_DestroyTwistSub(struct UrlabRclTwistSub* Sub);

int UrlabRcl_SpinSome(struct UrlabRclContext* Ctx, int64_t TimeoutNs);

// --- Zero-copy (only Clock is loanable in our message set) -----------------
// CanLoan wraps rcl_publisher_can_loan_messages. The loaned publish borrows,
// fills in place, publishes, and falls back to the plain publish when loaning
// is unavailable.
int UrlabRcl_ClockCanLoan(struct UrlabRclClockPub* Pub);   // 1 = loanable, 0 = not
int UrlabRcl_PublishClockLoaned(struct UrlabRclClockPub* Pub, int64_t SimTimeNs);

#ifdef __cplusplus
}  // extern "C"
#endif
