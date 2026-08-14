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

#include "CoreMinimal.h"

struct FMjArticulationState;

// Pure IR -> state-estimation math shared by the Odometry and Pose providers and
// the CameraInfo intrinsic derivation. No rcl dependency, so it compiles and is
// unit-tested in every configuration (the providers call it; the tests assert its
// correctness without a ROS runtime).
namespace MjRosStateEstimation
{
/**
 * The ground-truth pose and body-frame twist of a free-base articulation,
 * derived from its free joint. MuJoCo stores a free joint as
 * qpos = [world position(3), world orientation wxyz(4)] and
 * qvel = [WORLD linear velocity(3), BODY angular velocity(3)] - the linear and
 * angular halves live in DIFFERENT frames. nav_msgs/Odometry reports the twist
 * in the child (base) frame, so the linear half is rotated world->body here and
 * the angular half is passed through unchanged.
 */
struct FMjFreeBaseState
{
	bool bValid = false;
	int32 BaseBodyIndex = INDEX_NONE;     // index into Art.Bodies, or INDEX_NONE
	double Position[3] = {0.0, 0.0, 0.0}; // world position (m)
	double OrientationXyzw[4] = {0.0, 0.0, 0.0, 1.0};
	double LinearBody[3] = {0.0, 0.0, 0.0};  // linear velocity in the base frame
	double AngularBody[3] = {0.0, 0.0, 0.0}; // angular velocity in the base frame
};

/** Rotate a vector from the world frame into a body frame given the body
 *  orientation quaternion in MuJoCo wxyz order (i.e. apply the conjugate).
 *  Pure quaternion algebra, independent of coordinate handedness. */
URLABROS_API void RotateWorldToBody(const double QuatWxyz[4], const double VWorld[3],
	double OutVBody[3]);

/** Fill Out from the articulation's free joint (first joint of type Free with a
 *  full 7-DOF qpos / 6-DOF qvel). Returns false when the art has no free base
 *  (a fixed-base arm), in which case no odometry / base pose is published.
 *  The base body is the one whose world position matches the free-joint qpos;
 *  BaseBodyIndex is INDEX_NONE when no body matches (caller falls back to a
 *  conventional base link name). */
URLABROS_API bool ComputeFreeBaseState(const FMjArticulationState& Art,
	FMjFreeBaseState& Out);

/** Standard pinhole intrinsics K (row-major 3x3) from a vertical field of view
 *  and image size: fy = (H/2) / tan(fovy/2), fx = fy (square pixels; the
 *  horizontal FOV follows from the width), principal point at the image centre
 *  (cx = W/2, cy = H/2). K[0]=fx, K[2]=cx, K[4]=fy, K[5]=cy, K[8]=1, rest 0. */
URLABROS_API void PinholeKFromFovy(double FovyDegrees, int32 Width, int32 Height,
	double OutK9[9]);
} // namespace MjRosStateEstimation
