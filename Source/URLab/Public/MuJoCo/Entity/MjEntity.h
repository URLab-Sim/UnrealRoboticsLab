// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"
#include "State/MjStateTypes.h" // EMjSensorSemantic

struct mjModel_;
typedef mjModel_ mjModel;

/**
 * The addressing unit: a stable name bound to the slice of MuJoCo ids (in the one compiled model)
 * that a caller treats as one thing. A robot is an entity that owns actuators; a prop is an entity
 * that owns none. It is a view built from the compiled mjModel, not a second copy of it: no
 * UObjects, ids only.
 */
struct URLAB_API FMjEntity
{
	/** Stable public name (compiled: the participant prefix; raw: root / subtree name). */
	FName Name;

	/**
	 * The name the wire keys this entity by: the sanitized ActorId when set, else the sanitized
	 * Name. Distinct from Name (which is the raw compiled-prefix stem) because addressing must
	 * survive Unreal's volatile actor renaming. Falls back to Name when nothing supplied it.
	 */
	FName PublicName;

	/** Bridge-owned identifier echoed in the handshake, or empty. */
	FString ActorId;

	/** Root body id (for root-link state + free-base detection). */
	int32 RootBodyId = -1;

	/** True if the root body owns a single mjJNT_FREE joint. */
	bool bFreeBase = false;

	/** The id slices this entity owns (any may be empty). */
	TArray<int32> BodyIds;
	TArray<int32> JointIds;
	TArray<int32> ActuatorIds;
	TArray<int32> SensorIds;

	/** Parallel to SensorIds: each sensor's ROS semantic, computed at build. */
	TArray<EMjSensorSemantic> SensorSemantics;

	// Metadata computed once at install and stored alongside the id slices (sensor semantics
	// parallel to SensorIds, camera canonical names, ...) is filled here by the builder.
};

/** How to partition the live model into entities. */
struct URLAB_API FMjEntityPartition
{
	/** Name prefixes (compiled participants). Empty => a single root entity over the whole model. */
	TArray<FString> Prefixes;

	/**
	 * Optional per-prefix actor facts the model cannot supply, parallel to Prefixes: the wire public
	 * segment and the bridge ActorId. When absent for a prefix, the entity's PublicName falls back to
	 * its Name and ActorId stays empty.
	 */
	TArray<FName> PublicNames;
	TArray<FString> ActorIds;

	/** Raw path: split each top-level body subtree into its own entity. */
	bool bBodySubtreeSplit = false;
};

namespace MjEntityBuilder
{
	/** Build the entity partition from the live compiled model. Identical for compiled and raw. */
	URLAB_API TArray<FMjEntity> Build(const mjModel* Model, const FMjEntityPartition& How);
}

/**
 * Structure-version signal: bumped whenever the entity partition is rebuilt (recompile / reinstall).
 * Consumers such as the ROS re-subscribe path poll it to learn the model shape changed.
 */
struct URLAB_API FMjEntityStructureVersion
{
	uint64 Version = 0;
	void Bump() { ++Version; }
	uint64 Get() const { return Version; }
};
