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
#include "State/MjStateTypes.h"
#include <atomic>

class AAMjManager;
class UMjTwistController;
struct mjModel_;
struct mjData_;
typedef mjModel_ mjModel;
typedef mjData_ mjData;

/**
 * The one concrete collector that builds the per-step IR. No producer interface,
 * no registration: it has exactly three fixed steps.
 *
 * - Producer cache (game thread): per entity, the wire public name and the id
 *   slices it owns, plus weak ptrs to its actor-side channels (twist controller,
 *   interface producers); rebuilt on the same triggers as the old broadcast cache
 *   (registry change / recompile). Bumps StructureVersion.
 * - Per-step build (physics thread, post-step): reset the persistent snapshot,
 *   stamp time/step/clock, describe each cached entity's bodies / joints /
 *   actuators / sensors straight from its ids plus the twist controller, then fill
 *   Entities from the manager's entity cache indexing mjData directly.
 * - On-demand build (RPC step replies): the same Collect(), called under the
 *   engine's CallbackMutex by the dispatcher.
 *
 * Collect() reuses one persistent snapshot and returns a reference to it. Every
 * caller holds the engine's CallbackMutex (the post-step fan-out runs inside it;
 * the reply paths acquire it explicitly), so builds never overlap and the single
 * buffer is safe.
 */
class URLAB_API FMjStateCollector
{
public:
	void Init(AAMjManager* InManager);

	/** Mark the producer cache stale (registry change / recompile). A rebuild is
	 *  scheduled on the game thread; safe to call from any thread. */
	void MarkProducerCacheDirty();

	/** Rebuild the producer cache from the engine's entity partition. Game thread
	 *  only (resolves actor-side channels). Bumps StructureVersion. */
	void RebuildProducerCacheGameThread();

	/** Build the IR for the current step and return the persistent snapshot. */
	const FMjStateSnapshot& Collect(mjModel* m, mjData* d, int64 StepIdx);

	uint32 GetStructureVersion() const { return StructureVersion; }

private:
	struct FCachedEntity
	{
		/** Raw compiled-prefix stem (== the owning actor's GetName()); the prefix
		 *  the element-key strip removes. */
		FName Name;
		/** The wire key this entity's block is stamped with. */
		FName PublicName;
		/** The id slices this entity owns, in ascending mj-id order. */
		TArray<int32> BodyIds;
		TArray<int32> JointIds;
		TArray<int32> ActuatorIds;
		TArray<int32> SensorIds;
		/** Parallel to SensorIds: each sensor's semantic, resolved at build. */
		TArray<EMjSensorSemantic> SensorSemantics;
		TWeakObjectPtr<UMjTwistController> TwistCtrl; // UActorComponent; called separately
		/** IMjStateProducer implementers registered under this entity's actor (e.g.
		 *  user channel components). Any UObject; the collector Casts to the interface. */
		TArray<TWeakObjectPtr<UObject>> InterfaceProducers;
	};

	// A non-robot geom resolved once on the game thread (shape is static); the
	// physics thread only reads the parent body's live pose each step.
	struct FCachedWorldGeom
	{
		FName Name;
		EMjWorldGeomShape Shape = EMjWorldGeomShape::Box;
		double Size[3] = {0.0, 0.0, 0.0};
		int32 BodyId = 0;
		double LocalPos[3] = {0.0, 0.0, 0.0};
		double LocalQuat[4] = {1.0, 0.0, 0.0, 0.0};
		bool bStatic = true;
		TSharedPtr<const FMjWorldMesh> Mesh; // set when Shape == Mesh
	};

	TWeakObjectPtr<AAMjManager> Manager;
	TArray<FCachedEntity> Cache;             // built game thread, read physics thread
	TArray<FCachedWorldGeom> WorldGeomCache; // built game thread, read physics thread
	/** Registered IMjStateProducers not owned by any articulation; their
	 *  DescribeSceneState fills the snapshot's scene-scoped blocks. */
	TArray<TWeakObjectPtr<UObject>> SceneProducers;
	FCriticalSection CacheMutex;
	std::atomic<bool> bCacheValid{false};
	std::atomic<bool> bRebuildScheduled{false};
	FMjStateSnapshot Snapshot; // persistent; Reset() keeps top-level capacity
	uint32 StructureVersion = 0;

	/** Post a game-thread rebuild if one is not already queued. */
	void RequestGameThreadRebuild();
};
