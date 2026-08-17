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
#include "mujoco/mujoco.h"

/**
 * One contact's debug-visualization data, captured post-step for the
 * mjVIS_CONTACTPOINT / mjVIS_CONTACTFORCE overlays. Filled only while a
 * consumer has contact viz enabled (see UMjPhysicsEngine::bContactVizWanted),
 * so the per-contact mj_contactForce call stays off the hot path when unused.
 */
struct FMjContactViz
{
	mjtNum Pos[3];   // contact point, world frame
	mjtNum Frame[9]; // contact frame, row-major; row 0 is the contact normal
	mjtNum Force[6]; // mj_contactForce result: force xyz then torque xyz, contact frame
	mjtNum Dist;     // signed penetration distance (< 0 penetrating)
};

/**
 * Single-frame snapshot of MuJoCo state for game-thread consumers.
 *
 * Produced once per step on the physics thread inside
 * UMjPhysicsEngine::PushRenderState while CallbackMutex is held.
 * Consumed on the game thread via UMjPhysicsEngine::WithRenderState,
 * which holds RenderStateMutex for the entire visitor body so every
 * consumer in one UE frame observes the same coherent physics frame.
 *
 * All arrays are sized once on model load when nbody / ngeom /
 * nsite / ncam / nq / nv / nu / nsensordata / ntree / nflexvert
 * change; steady-state push reuses the existing storage.
 *
 * Lifetime is bound to the owning UMjPhysicsEngine.
 */
struct FMjRenderSnapshot
{
	// --- Body kinematics (visual update + APIs) -----------------------

	/** Body world position. 3 * nbody (mjtNum). */
	TArray<mjtNum> XPos;

	/** Body world rotation (unit quaternion, mjtNum order w, x, y, z). 4 * nbody. */
	TArray<mjtNum> XQuat;

	/** Body spatial velocity (angular xyz, linear xyz, body frame). 6 * nbody. */
	TArray<mjtNum> CVel;

	/** Applied wrench on each body (linear xyz, angular xyz). 6 * nbody. */
	TArray<mjtNum> XfrcApplied;

	// --- Geoms / sites (debug viz, on-demand transform queries) -------

	TArray<mjtNum> GeomXPos; // 3 * ngeom
	TArray<mjtNum> GeomXMat; // 9 * ngeom
	TArray<mjtNum> SiteXPos; // 3 * nsite
	TArray<mjtNum> SiteXMat; // 9 * nsite

	// --- Cameras (tracking modes, RPC camera lookups) -----------------

	TArray<mjtNum> CamXPos; // 3 * ncam
	TArray<mjtNum> CamXMat; // 9 * ncam

	// --- Debug-overlay extras (COM / inertia / contacts) --------------

	/** Body subtree centre of mass (d->subtree_com), world frame. 3 * nbody. mjVIS_COM. */
	TArray<mjtNum> SubtreeCom;

	/** Body inertial-frame world position (d->xipos). 3 * nbody. mjVIS_INERTIA. */
	TArray<mjtNum> XiPos;

	/** Body inertial-frame world orientation (d->ximat), row-major. 9 * nbody. mjVIS_INERTIA. */
	TArray<mjtNum> XiMat;

	/** Per-contact debug data. Empty unless a contact-viz consumer asked for it
	 *  (UMjPhysicsEngine::SetContactVizWanted); sized to d->ncon when filled. */
	TArray<FMjContactViz> Contacts;

	// --- Joint / actuator / sensor state ------------------------------

	TArray<mjtNum> QPos;          // nq
	TArray<mjtNum> QVel;          // nv
	TArray<mjtNum> QAcc;          // nv
	TArray<mjtNum> ActuatorForce; // nu
	TArray<mjtNum> SensorData;    // nsensordata

	/** Joint world anchor and axis, as the joint accessors report them. */
	TArray<mjtNum> JntXAnchor; // 3 * njnt
	TArray<mjtNum> JntXAxis;   // 3 * njnt

	/** The rest of the actuator state the actuator accessors report. */
	TArray<mjtNum> Ctrl;             // nu
	TArray<mjtNum> ActuatorLength;   // nu
	TArray<mjtNum> ActuatorVelocity; // nu
	TArray<mjtNum> Act;              // na

	/** Tendon state, as the tendon accessors report it. */
	TArray<mjtNum> TenLength;   // ntendon
	TArray<mjtNum> TenVelocity; // ntendon

	// --- Tendon wrap geometry (mjVIS_TENDON overlay) ------------------
	//
	// The world-frame wrap path each spatial tendon threads through, mirroring
	// what MuJoCo's own renderer draws. wrap_xpos is declared nwrap x 6, i.e.
	// 2*nwrap points of xyz, so WrapXPos is 6 * nwrap mjtNum. WrapObj marks each
	// point's wrap object (-2 = pulley, skipped when connecting). TenWrapAdr /
	// TenWrapNum slice the point list per tendon (in units of points, so index
	// j addresses WrapXPos[3*j]). Empty unless a tendon-viz consumer asked for
	// it, keeping wrap capture off the hot path when unused.

	TArray<mjtNum> WrapXPos;   // 6 * nwrap (2*nwrap points, xyz each), world frame
	TArray<int32> WrapObj;     // 2 * nwrap; -2 marks a pulley point
	TArray<int32> TenWrapAdr;  // ntendon; first wrap-point index of the tendon
	TArray<int32> TenWrapNum;  // ntendon; number of wrap points in the tendon

	// --- Sleep state (IsAwake + debug viz) ----------------------------
	//
	// Stored as int because MuJoCo's body_awake is `int*` carrying
	// mjtSleepState values (0 = asleep, 1 = awake). tree_asleep stores
	// a signed sleep-cycle index (<0 = awake). tree_awake is 0/1.

	TArray<int32> BodyAwake;  // nbody
	TArray<int32> TreeAsleep; // ntree
	TArray<int32> TreeAwake;  // ntree

	// --- Flex deformable state (UMjFlexcomp) --------------------------

	TArray<mjtNum> FlexvertXPos; // 3 * nflexvert

	// --- Metadata -----------------------------------------------------

	/** MuJoCo simulation time at the moment of the snapshot. */
	mjtNum SimTime = 0.0;

	/** Monotonic frame counter. Incremented every successful push.
	 *  Consumers can skip on a stale snapshot via equality test. */
	uint64 FrameId = 0;
};
