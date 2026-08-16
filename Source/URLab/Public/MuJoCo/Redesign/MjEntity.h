// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"

struct mjModel_;
typedef mjModel_ mjModel;

/**
 * The ONE addressing unit: a stable name bound to the slice of MuJoCo ids (in the single compiled
 * model) that a caller treats as one thing. EVERYTHING is an FMjEntity -- a robot is one that owns
 * actuators, a prop is one that owns none. Absorbs the old props-only FMjEntityRecord. Built FROM
 * the compiled mjModel (a view), never a hand-maintained second copy. No UObjects; ids only.
 */
struct URLAB_API FMjEntity
{
	/** Stable public name (compiled: the participant prefix; raw: root / subtree name). */
	FName Name;

	/** Root body id (for root-link state + free-base detection). */
	int32 RootBodyId = -1;

	/** True if the root body owns a single mjJNT_FREE joint. */
	bool bFreeBase = false;

	/** The id slices this entity owns (any may be empty). */
	TArray<int32> BodyIds;
	TArray<int32> JointIds;
	TArray<int32> ActuatorIds;
	TArray<int32> SensorIds;

	/**
	 * Build-time ENRICHMENT (the "30%"): metadata computed once at install and stored HERE, not on
	 * an actor -- sensor semantics parallel to SensorIds, camera canonical names, etc. Filled by the
	 * builder; keeps ROS/camera/handshake off the deleted shadow actor.
	 */
};

/** How to partition the live model into entities. */
struct URLAB_API FMjEntityPartition
{
	/** Name prefixes (compiled participants). Empty => a single root entity over the whole model. */
	TArray<FString> Prefixes;

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
 * Consumers (ROS re-subscribe) poll it to know the model shape changed. Re-homed here from the
 * deleted MjStateCollector producer-cache rebuild.
 */
struct URLAB_API FMjEntityStructureVersion
{
	uint64 Version = 0;
	void Bump() { ++Version; }
	uint64 Get() const { return Version; }
};
