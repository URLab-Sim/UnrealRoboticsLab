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

struct mjModel_;
typedef mjModel_ mjModel;

/**
 * Configuration for the mjModel -> URDF exporter. MuJoCo has no joint effort or
 * velocity limits of its own, so the exporter derives what it can from the
 * actuators and falls back to these defaults; consumers such as urdfdom and
 * MoveIt TOTG reject a revolute/prismatic joint whose limit lacks a positive
 * velocity, so the defaults must be non-zero.
 */
struct FUrdfExportConfig
{
	double DefaultEffort = 100.0;
	double DefaultVelocityAngular = 3.14;   // rad/s, for hinge joints
	double DefaultVelocityLinear = 1.0;     // m/s, for slide joints
	/** Per-joint (compiled joint name) velocity override, highest priority. */
	TMap<FString, double> VelocityOverrides;
	/** Widen exported position limits by this (rad / m) so MuJoCo's soft-limit
	 *  overshoot stays inside the URDF hard limits. Without it a planner (MoveIt)
	 *  rejects the sim's resting state as out of bounds on tight joints (the
	 *  Franka's joint4). Small enough to be physically negligible. */
	double LimitMargin = 0.02;
	/** When true, append a fixed "tool0" link at the leaf body of the
	 *  articulation tree so MoveIt can attach an end-effector. */
	bool bAppendTool0 = true;
};

/** One emitted geom's frame data, kept for the forward-kinematics self-test.
 *  The link/collision classification and shape params live only in the XML. */
struct FUrdfGeomFrame
{
	int32 MjGeomId = -1;
	FVector LocalPos = FVector::ZeroVector;   // origin xyz after the jnt_pos frame-shift, metres
	FVector LocalRpy = FVector::ZeroVector;   // origin rpy (extrinsic XYZ), radians
};

/** One URDF link. Dummy links (multi-joint bodies) carry no geoms. */
struct FUrdfLink
{
	FString Name;
	TArray<FUrdfGeomFrame> Geoms;
};

/** One URDF joint. Origins are parent-link -> child-link at the URDF zero pose. */
struct FUrdfJoint
{
	FString Name;
	FString Parent;
	FString Child;
	FString Type;   // revolute / continuous / prismatic / fixed
	FVector OriginPos = FVector::ZeroVector;
	FVector OriginRpy = FVector::ZeroVector;
	FVector Axis = FVector::ZeroVector;
	bool bHasLimit = false;   // lower/upper present (revolute + prismatic)
	double Lower = 0.0;
	double Upper = 0.0;
	double Effort = 0.0;
	double Velocity = 0.0;
	int32 MjJointId = -1;     // -1 for synthesized fixed joints and dummy chains
};

/** Structured result of a build plus the serialized XML. */
struct FUrdfModel
{
	FString RobotName;
	TArray<FUrdfLink> Links;
	TArray<FUrdfJoint> Joints;
	/** Compiled mesh ids referenced by the model, deduplicated (one STL each). */
	TArray<int32> MeshIds;
	/** Human-readable warnings for every dropped or defaulted construct. */
	TArray<FString> Warnings;
	/** The full URDF spec. */
	FString Xml;
};

/**
 * In-process exporter from a compiled `mjModel` to URDF + binary STL meshes.
 *
 * This is a direct port of the Python prototype validated field-by-field against
 * the mujoco_menagerie Franka (see docs/plan_ros_urdf_port_spec.md). It is a
 * pure mjModel reader with no external-transport dependency, so it always
 * compiles as part of the core module.
 *
 * Frame conventions (validated to ~1e-9 m against mjModel geom_xpos):
 *  - URDF joint limits are `jnt_range - qpos0`, so URDF q=0 is the MuJoCo
 *    reference pose (qpos == qpos0). The matched joint_states shift lives in
 *    the state publisher's FillJointState.
 *  - A single-joint body with `jnt_pos != 0` has its link frame moved to the
 *    joint anchor; every body-local origin is re-expressed accordingly.
 *  - A body with k>1 joints becomes k-1 zero-inertia dummy links chained by the
 *    first k-1 joints, with the real link carried by the last.
 *
 * Dropped constructs (ball/free joints, tendons, equalities, unsupported geom
 * types) are logged into FUrdfModel::Warnings.
 */
class URLAB_API FUrdfExporter
{
public:
	/** Body ids (i > 0) belonging to an art, in ascending (topological) order.
	 *  When ArtRawName is empty every non-world body is returned (whole model).
	 *  Otherwise the art's bodies are those whose compiled name equals ArtRawName
	 *  or begins with "<ArtRawName>_", matching FMjCanonicalName::PartSegment. */
	static TArray<int32> BodyIdsForArt(const mjModel* M, const FString& ArtRawName);

	/** Build the structured model + XML for the given body set. MeshUriDir is the
	 *  directory the emitted `file://` mesh URIs point at; no files are written. */
	static FUrdfModel Build(const mjModel* M, const FString& RobotName,
		const FString& ArtRawName, const TArray<int32>& BodyIds,
		const FString& MeshUriDir, const FUrdfExportConfig& Cfg);

	/** Full export to disk: build, write every referenced mesh as binary STL to
	 *  OutDir/meshes, and dump OutDir/model.urdf. Returns the model (Xml filled).
	 *  Returns an empty model (no links) on a null mjModel. */
	static FUrdfModel ExportToDir(const mjModel* M, const FString& RobotName,
		const FString& ArtRawName, const FString& OutDir, const FUrdfExportConfig& Cfg);

	/** Serialize one compiled mesh id to a binary STL file at Dir/<name>.stl.
	 *  OutFilename receives the bare "<name>.stl". Returns false on write failure. */
	static bool WriteMeshStl(const mjModel* M, int32 MeshId, const FString& Dir,
		FString& OutFilename);

	/** Sanitized compiled mesh name (no extension), matching the STL file stem. */
	static FString MeshBaseName(const mjModel* M, int32 MeshId);
};
