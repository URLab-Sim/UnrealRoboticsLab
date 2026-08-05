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

#include "Transport/RosStateEstimation.h"
#include "State/MjStateTypes.h"

namespace MjRosStateEstimation
{
namespace
{
// Rotate V by the quaternion (w, x, y, z) using the standard vector form
// v' = v + 2w(u x v) + 2u x (u x v), u = (x, y, z).
void RotateByQuat(double w, double x, double y, double z, const double V[3], double O[3])
{
	const double tx = 2.0 * (y * V[2] - z * V[1]);
	const double ty = 2.0 * (z * V[0] - x * V[2]);
	const double tz = 2.0 * (x * V[1] - y * V[0]);
	O[0] = V[0] + w * tx + (y * tz - z * ty);
	O[1] = V[1] + w * ty + (z * tx - x * tz);
	O[2] = V[2] + w * tz + (x * ty - y * tx);
}
}  // namespace

void RotateWorldToBody(const double QuatWxyz[4], const double VWorld[3], double OutVBody[3])
{
	// world->body is rotation by the conjugate (negate the vector part).
	RotateByQuat(QuatWxyz[0], -QuatWxyz[1], -QuatWxyz[2], -QuatWxyz[3], VWorld, OutVBody);
}

bool ComputeFreeBaseState(const FMjArticulationState& Art, FMjFreeBaseState& Out)
{
	Out = FMjFreeBaseState();

	const FMjJointState* Free = nullptr;
	for (const FMjJointState& Joint : Art.Joints)
	{
		if (Joint.Type == EMjJointType::free && Joint.QPos.Num() >= 7 && Joint.QVel.Num() >= 6)
		{
			Free = &Joint;
			break;
		}
	}
	if (!Free)
	{
		return false;
	}

	// Pose straight from the free-joint qpos: [0..2] world position, [3..6] world
	// orientation wxyz. ROS carries xyzw, so reorder the quaternion.
	for (int32 i = 0; i < 3; ++i)
	{
		Out.Position[i] = Free->QPos[i];
	}
	const double QuatWxyz[4] = { Free->QPos[3], Free->QPos[4], Free->QPos[5], Free->QPos[6] };
	Out.OrientationXyzw[0] = QuatWxyz[1];
	Out.OrientationXyzw[1] = QuatWxyz[2];
	Out.OrientationXyzw[2] = QuatWxyz[3];
	Out.OrientationXyzw[3] = QuatWxyz[0];

	// Twist: qvel[0..2] is WORLD linear (rotate into the base frame); qvel[3..5] is
	// already BODY angular (pass through).
	const double LinearWorld[3] = { Free->QVel[0], Free->QVel[1], Free->QVel[2] };
	RotateWorldToBody(QuatWxyz, LinearWorld, Out.LinearBody);
	Out.AngularBody[0] = Free->QVel[3];
	Out.AngularBody[1] = Free->QVel[4];
	Out.AngularBody[2] = Free->QVel[5];

	// The base body is the free-jointed root, whose world position equals the free
	// joint qpos exactly (same mjData source). Match on that.
	int32 BestIndex = INDEX_NONE;
	double BestSq = 1.0e-12;  // tight: an exact copy, not a nearest-neighbour search
	for (int32 i = 0; i < Art.Bodies.Num(); ++i)
	{
		const FMjBodyState& Body = Art.Bodies[i];
		const double dx = Body.Xpos[0] - Out.Position[0];
		const double dy = Body.Xpos[1] - Out.Position[1];
		const double dz = Body.Xpos[2] - Out.Position[2];
		const double Sq = dx * dx + dy * dy + dz * dz;
		if (Sq <= BestSq)
		{
			BestSq = Sq;
			BestIndex = i;
		}
	}
	Out.BaseBodyIndex = BestIndex;

	Out.bValid = true;
	return true;
}

void PinholeKFromFovy(double FovyDegrees, int32 Width, int32 Height, double OutK9[9])
{
	for (int32 i = 0; i < 9; ++i)
	{
		OutK9[i] = 0.0;
	}
	const double W = Width > 0 ? static_cast<double>(Width) : 1.0;
	const double H = Height > 0 ? static_cast<double>(Height) : 1.0;
	// MuJoCo's default camera fovy is 45 degrees; an unset / non-positive fovy
	// would otherwise collapse the focal length to infinity.
	const double FovyRad = FMath::DegreesToRadians(FovyDegrees > 0.0 ? FovyDegrees : 45.0);
	const double Fy = (H * 0.5) / FMath::Tan(FovyRad * 0.5);
	const double Fx = Fy;  // square pixels; horizontal FOV emerges from the width
	OutK9[0] = Fx;
	OutK9[2] = W * 0.5;    // cx
	OutK9[4] = Fy;
	OutK9[5] = H * 0.5;    // cy
	OutK9[8] = 1.0;
}
}  // namespace MjRosStateEstimation
