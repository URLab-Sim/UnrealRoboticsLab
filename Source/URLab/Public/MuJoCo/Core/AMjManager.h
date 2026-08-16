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

#include "MuJoCo/Convert/MjQuickConvertComponent.h"
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "Bridge/RpcDispatcher.h"
#include "Bridge/BridgeServer.h"
#include "Transport/SnapshotPublisher.h"
#include "State/MjStateCollector.h"
#include "State/MjStateProducer.h"
#include "State/MjStateConsumer.h"
#include <atomic>
#include "AMjManager.generated.h"

// Forward declarations
class AMjEntity;
class AMjHeightfieldActor;
class UMjPhysicsEngine;
class UMjDebugVisualizer;
class UMjNetworkManager;
class UMjInputHandler;
class UMjPerturbation;
struct FSpecRef;

class UMjCompiler;
class UMjFlag;
class UMjModel;
class UMjOption;
class UMjSimulationState;
class UMjBody;
class UMjUserChannelComponent;
struct FMjUserChannel;
enum class EMjUserChannelKind : uint8;

/**
 * @struct FMjUserInputChannelInfo
 * @brief One declared user-input channel and its scope, enumerated for the
 *        transports that create per-channel input subscriptions (ROS) or route
 *        writes to it (the set_user_channels RPC op). ArtSegment is the canonical
 *        art segment for art scope, or empty for scene scope.
 */
struct FMjUserInputChannelInfo
{
	FString ArtSegment;
	FName Channel;
	EMjUserChannelKind Kind;
};

/**
 * @struct FMjEntityRecord
 * @brief Cached non-articulation entity metadata: a UMjBody whose owner is
 *        not an AMjArticulation (props, free-jointed scene objects, ...).
 *        Built once per compile and consumed by
 *        UURLabZmqPublishTransport for "scene/<name>/state" PUB topics and by
 *        the step server for the `entities` block in step replies.
 *        Articulations have their own typed cache; this struct is for
 *        everything else dynamic in the world.
 */
struct FMjEntityRecord
{
	/** UMjBody MjID after compile. */
	int32 MjId = -1;
	/** Compiled name (the same string mj_id2name returns). */
	FString Name;
	/** True if the body owns a single mjJNT_FREE joint (qpos[7]/qvel[6]). */
	bool bHasFreeBase = false;
	/** Weak ref kept for diagnostics; consumers should index by MjId. */
	TWeakObjectPtr<UMjBody> BodyComp;
};

/**
 * @class AAMjManager
 * @brief Thin coordinator actor for the MuJoCo simulation within Unreal Engine.
 *
 * Owns subsystem components and delegates to them:
 * - UMjPhysicsEngine: simulation lifecycle, model/data, options, async loop
 * - UMjDebugVisualizer: debug drawing, collision wireframes
 * - UMjNetworkManager: ZMQ components, camera streaming
 * - UMjInputHandler: keyboard hotkeys
 *
 * External code accesses subsystem state via Manager->PhysicsEngine->X, etc.
 */
UCLASS()
class URLAB_API AAMjManager : public AActor
{
	GENERATED_BODY()

public:
	AAMjManager();

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo")
	UMjPhysicsEngine* PhysicsEngine;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo")
	UMjDebugVisualizer* DebugVisualizer;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo")
	UMjNetworkManager* NetworkManager;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo")
	UMjInputHandler* InputHandler;

	/** Mouse-driven body perturbation (simulate-style Ctrl+LMB/RMB drag). */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo")
	UMjPerturbation* Perturbation;

	/**
	 * The scene spec's root, and the manager's transform root.
	 *
	 * A level is one MuJoCo scene, so it is one MJCF spec; the sections
	 * below are its children, and every articulation in the level is attached
	 * into it at write time rather than being merged into it.
	 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|Scene")
	TObjectPtr<UMjModel> SceneSpec;

	/**
	 * The scene's `<option>`, as the spec element it is.
	 *
	 * The scene is one MuJoCo spec assembled from the level, and its
	 * top-level sections belong to the manager rather than to any articulation:
	 * MuJoCo takes the scene's option block whole and discards an attached
	 * spec's own copy, so there is exactly one authority and this is it.
	 * Fields hold MJCF's values in MJCF's units.
	 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|Scene")
	TObjectPtr<UMjOption> SceneOption;

	/** The scene's `<option><flag>`. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|Scene")
	TObjectPtr<UMjFlag> SceneFlags;

	/**
	 * The scene's `<compiler>`, which carries the attach conflict policy.
	 *
	 * `merge` rather than MuJoCo's `warning` default, because the attach target
	 * is a scene nobody authored: under `warning` the parent keeps every field,
	 * so an imported model's `<option>` is discarded against defaults.
	 * Per-articulation `AttachConflict` overrides it.
	 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|Scene")
	TObjectPtr<UMjCompiler> SceneCompiler;

	/** A handle on the scene spec the manager's sections belong to. */
	FSpecRef GetSceneSpec() const;

	/** Set in BeginPlay, cleared in EndPlay. Use GetManager() from Blueprints. */
	static AAMjManager* Instance;

	UFUNCTION(BlueprintPure, Category = "MuJoCo|Global", meta = (DisplayName = "Get MuJoCo Manager"))
	static AAMjManager* GetManager();

	/** The physics engine a game-thread accessor should talk to: the singleton
	 *  manager's when one is live, otherwise the first manager in the calling
	 *  object's world (test worlds never run BeginPlay, so Instance is null
	 *  there). Components resolve per call rather than caching the engine. */
	static UMjPhysicsEngine* ResolveEngine(const UObject* WorldCtx);

	// --- State Control (delegates to PhysicsEngine) ---

	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Control")
	void SetPaused(bool bPaused);

	UFUNCTION(BlueprintPure, Category = "MuJoCo|Status")
	bool IsRunning() const;

	UFUNCTION(BlueprintPure, Category = "MuJoCo|Status")
	bool IsInitialized() const;

	// --- Articulation Access ---

	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Global")
	AMjArticulation* GetArticulation(const FString& ActorName) const;

	/**
	 * The runtime face of a compiled entity, by its stable name (the participant-prefix stem). Returns
	 * the AMjEntity that answers Joint / Actuator / Geom for it, spawning a thin one on first request
	 * and reusing it after; null when no compiled entity carries that name.
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Global")
	AMjEntity* GetEntity(FName EntityName);

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "MuJoCo|Global")
	// By value: forwards the engine's locked snapshot (see UMjPhysicsEngine::GetAllArticulations).
	TArray<AMjArticulation*> GetAllArticulations() const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "MuJoCo|Global")
	TArray<UMjQuickConvertComponent*> GetAllQuickComponents() const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "MuJoCo|Global")
	TArray<AMjHeightfieldActor*> GetAllHeightfields() const;

	/** Non-articulation entity table, rebuilt on every compile; empty until the first. */
	const TArray<FMjEntityRecord>& GetEntities() const { return EntityCache; }

	/** Refresh the entity cache. Called once per compile, on the game thread. */
	void BuildEntityCache();

	/** Rebuild the caches the state IR reads (entity table + producer cache) and
	 *  (re)bind the collector to this manager. Run after every compile / recompile
	 *  on the game thread. */
	void RefreshStateCaches();

	/** The per-step state-IR collector. Owned by the manager; used by the
	 *  post-step snapshot fan-out and by the RPC step/reset/forward replies. */
	FMjStateCollector& GetStateCollector() { return StateCollector; }

	/** Export a URDF + binary STL meshes for every articulation from the compiled
	 *  mjModel, dumping each to <ProjectSaved>/URLab/UrdfExport/<art>/ and caching
	 *  the URDF text for the /<art>/robot_description publisher. Runs on the
	 *  game thread; auto-invoked from RefreshStateCaches (every compile) and
	 *  callable directly as the manual re-export trigger. No-op without a model. */
	void ExportRobotDescriptions();

	/** Cached URDF specs keyed by canonical art segment, filled by
	 *  ExportRobotDescriptions and read by the state publish transport. */
	const TMap<FName, FString>& GetRobotDescriptions() const { return RobotDescriptions; }

	UFUNCTION(BlueprintPure, Category = "MuJoCo|Status")
	float GetSimTime() const;

	UFUNCTION(BlueprintPure, Category = "MuJoCo|Status")
	float GetTimestep() const;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|UI")
	bool bAutoCreateSimulateWidget = true;

	// --- Remote Stepping ---

	/**
	 * @brief Step mode for the simulation.
	 *
	 * Auto (default) lets the Python client promote to Direct or Puppet on hello.
	 * Pinning to Live / Direct / Puppet locks the engine and rejects
	 * mode-switch RPCs with error("mode_locked_by_server").
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Remote Stepping")
	EStepMode StepMode = EStepMode::Auto;

	/**
	 * @brief Deterministic seed written to m->opt.seed before compile.
	 *
	 * Also writable at runtime via the reset RPC for episode-level reseeding.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Remote Stepping")
	int32 Seed = 0;

	/**
	 * @brief Single source of truth for "publishers should pause".
	 *
	 * FURLabRpcDispatcher flips this to true on entering Direct/Puppet so the
	 * sensor broadcaster, control subscriber, and camera workers all idle.
	 * Flipped back to false on exit.
	 */
	std::atomic<bool> bPublishersPaused{false};

	/** Post-step render-snapshot id / sim time last applied to the actors.
	 *  Written on the game thread in ApplyLatestRenderState; read (atomically)
	 *  by cameras when stamping readbacks. */
	std::atomic<uint64> LastAppliedRenderFrameId{0};
	std::atomic<double> LastAppliedRenderSimTime{0.0};

	/** Owns the FURLabRpcDispatcher + transports. Created in BeginPlay, destroyed in EndPlay. */
	UPROPERTY()
	TObjectPtr<UURLabBridgeServer> BridgeServer;

	FURLabRpcDispatcher* GetStepDispatcher() const
	{
		return BridgeServer ? BridgeServer->GetDispatcher() : nullptr;
	}

	/** Snapshot publishers share one per-step state_full payload — each
	 *  wire transport (ZMQ PUB, SHM ring) avoids re-serialising the same
	 *  data. OwnerObj may be any UObject; parameter name avoids shadowing
	 *  AActor::Owner. */
	void RegisterSnapshotPublisher(IMjSnapshotPublisher* Publisher,
		class UObject* OwnerObj);
	void UnregisterSnapshotPublisher(IMjSnapshotPublisher* Publisher);

	/** Register a typed consumer of the per-step state IR. FanOutStateSnapshot
	 *  calls ConsumeState on every registered consumer once per step, in all step
	 *  modes (unlike the byte fan-out, which the Direct/Puppet pause suppresses).
	 *  OwnerObj keeps the registration alive only while the owner is valid. This
	 *  is the transport-agnostic seam an out-of-core encoder registers against so
	 *  the manager never names a concrete consumer type. */
	void RegisterStateConsumer(IMjStateConsumer* Consumer, class UObject* OwnerObj);
	void UnregisterStateConsumer(IMjStateConsumer* Consumer);

	/** Register an IMjStateProducer the collector cannot discover by walking
	 *  articulations (scene-level actors, user channel components). Marks the
	 *  producer cache dirty so scope is re-resolved. Game thread. */
	void RegisterStateProducer(TScriptInterface<IMjStateProducer> Producer);
	void UnregisterStateProducer(TScriptInterface<IMjStateProducer> Producer);

	/** Copy the registered state producers out under the registry lock. Called by
	 *  the collector's game-thread cache rebuild. */
	void GetStateProducers(TArray<TWeakObjectPtr<UObject>>& Out) const;

	/** Route an inbound user-channel value to the declaring component. ArtOrNone is
	 *  the canonical art segment for art scope, or None/empty for scene scope. The
	 *  transport (the set_user_channels RPC op, or a ROS subscription) builds the
	 *  value; the component validates it against the declared kind and stores it.
	 *  Returns true when a declaring component accepted the write. Thread-safe;
	 *  callable from any transport thread. This is the input mirror of the state
	 *  consumer seam. */
	bool ApplyUserChannelInput(FName ArtOrNone, FName Channel, const FMjUserChannel& Value);

	/** Enumerate every declared user-input channel across registered components,
	 *  with its scope. Used by ROS to create one subscription per input channel and
	 *  rebuild the set on a StructureVersion change. Thread-safe. */
	void GetUserInputChannels(TArray<FMjUserInputChannelInfo>& Out) const;

	/** Build the per-step IR, encode the canonical `state_full` msgpack, and fan
	 *  the bytes to every registered snapshot publisher. Bound to the physics
	 *  post-step callback; gated by bPublishersPaused (byte fan-out only). */
	void FanOutStateSnapshot(struct mjModel_* m, struct mjData_* d);

	/** Bound to Tab key. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|UI")
	void ToggleSimulateWidget();

	UPROPERTY()
	UUserWidget* SimulateWidget = nullptr;

protected:
	/** O(1) articulation lookup, rebuilt on every compile. Key = actor name. */
	TMap<FString, AMjArticulation*> m_ArticulationMap;

	TArray<FMjEntityRecord> EntityCache;

	/** Builds the per-step state IR consumed by the msgpack encoder. */
	FMjStateCollector StateCollector;

	/** Per-art URDF specs, keyed by canonical art segment. */
	TMap<FName, FString> RobotDescriptions;

public:
	/** Manager-owned UObject publish transports
	 *  (UURLabShmPublishTransport + UURLabZmqPublishTransport).
	 *  UPROPERTY(Transient) keeps GC alive across PIE without
	 *  serialising. EndPlay calls TransportShutdown on each before
	 *  clearing. Public so the dispatcher can iterate to find specific
	 *  publishers (e.g. SHM state path for hello reply). */
	UPROPERTY(Transient)
	TArray<TObjectPtr<class UURLabPublishTransport>> ManagerOwnedPublishTransports;

	/** Manager-owned UObject subscribe transports
	 *  (UURLabZmqSubscribeTransport). Same lifetime contract as the publish
	 *  array; same per-step iteration contract via PreStep callback. */
	UPROPERTY(Transient)
	TArray<TObjectPtr<class UURLabSubscribeTransport>> ManagerOwnedSubscribeTransports;

	/** Read-only viewer role: when set, this process is a VIEWER driven by a
	 *  remote owner's viewer bus. It owns no bridge/publishers; this transport
	 *  is the sole input. Null on a normal owner. */
	UPROPERTY(Transient)
	TObjectPtr<class UURLabViewerSubscribeTransport> ViewerTransport;

	/** True when this process was started as a viewer (StateSourceEndpoint set). */
	bool bIsViewerRole = false;

	// Owner-side viewer bus (direct/live): the "viewer" ({t,qpos,qvel}) and "geoms"
	// (per-geom transforms) topics, fanned out through the agnostic publish
	// abstraction (ZMQ now; SHM/ROS/gRPC via new UURLabPublishTransport impls),
	// on their own endpoint separate from the state bus. Empty unless
	// bBroadcastViewers was set on an owner. Written from FanOutStateSnapshot
	// (physics thread); TransportShutdown'd + cleared in EndPlay.
	UPROPERTY(Transient)
	TArray<TObjectPtr<class UURLabPublishTransport>> ViewerBusTransports;
	/** Fan a topic's payload out to every viewer-bus publish transport. */
	void PublishOnViewerBus(const FString& Topic, const TArray<uint8>& Payload);
	/** Encode {t,qpos,qvel} from (m,d) and PUB it on the viewer bus. */
	void PublishViewerFrame(struct mjModel_* m, struct mjData_* d);
	/** Encode per-geom world transforms {f,xpos,xquat} from (m,d) and PUB them on
	 *  the same bus under the "geoms" topic, so a fast-path renderer can mirror
	 *  this owner with no physics. */
	void PublishGeomFrame(struct mjModel_* m, struct mjData_* d);
	/** Monotonic frame id for the geoms bus. */
	uint64 GeomBroadcastFrame = 0;

protected:
	struct FRegisteredSnapshotPublisher
	{
		TWeakObjectPtr<UObject> Owner;
		IMjSnapshotPublisher* Publisher = nullptr;
	};
	/** Snapshot publishers registered by their owning components. Read on
	 *  the physics async thread, mutated on the game thread (BeginPlay /
	 *  EndPlay) -- protect with SnapshotPublishersMutex. */
	TArray<FRegisteredSnapshotPublisher> SnapshotPublishers;
	mutable FCriticalSection SnapshotPublishersMutex;

	struct FRegisteredStateConsumer
	{
		TWeakObjectPtr<UObject> Owner;
		IMjStateConsumer* Consumer = nullptr;
	};
	/** Typed state consumers registered by their owning transports. Read on the
	 *  physics async thread (fan-out), mutated on the game thread; guarded by
	 *  StateConsumersMutex. */
	TArray<FRegisteredStateConsumer> StateConsumers;
	mutable FCriticalSection StateConsumersMutex;

	/** IMjStateProducers registered by owners the collector cannot walk to.
	 *  Read on the game thread (collector rebuild), mutated on the game thread
	 *  (BeginPlay / EndPlay); guarded by StateProducersMutex for safety. */
	TArray<TWeakObjectPtr<UObject>> StateProducers;
	mutable FCriticalSection StateProducersMutex;

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

public:
	virtual void Tick(float DeltaTime) override;

	/** Pull the latest physics render snapshot and push it onto the UE
	 *  actor/component transforms. Normally driven once per frame from Tick.
	 *  Records the applied snapshot's FrameId / SimTime so cameras can tag
	 *  their readbacks with the post-step state they show. Game thread only. */
	void ApplyLatestRenderState();

	/** Render-snapshot id last applied to the actors (post-step state id that
	 *  the currently-rendered scene reflects). Cameras stamp readbacks with
	 *  this; the bridge associates an image with a step by frame_id. */
	uint64 GetLastAppliedFrameId() const
	{
		return LastAppliedRenderFrameId.load(std::memory_order_acquire);
	}

	/** MuJoCo sim time of the snapshot last applied to the actors. */
	double GetLastAppliedSimTime() const
	{
		return LastAppliedRenderSimTime.load(std::memory_order_acquire);
	}

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Mujoco Physics|Objects")
	TArray<UMjQuickConvertComponent*> m_MujocoComponents;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Mujoco Physics|Objects")
	TArray<AMjArticulation*> m_articulations;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Mujoco Physics|Objects")
	TArray<AMjHeightfieldActor*> m_heightfieldActors;

	void Compile();

	// --- Replay ---

	UFUNCTION(CallInEditor, Category = "MuJoCo|Replay")
	void StartRecording();

	UFUNCTION(CallInEditor, Category = "MuJoCo|Replay")
	void StopRecording();

	UFUNCTION(CallInEditor, Category = "MuJoCo|Replay")
	void StartReplay();

	UFUNCTION(CallInEditor, Category = "MuJoCo|Replay")
	void StopReplay();

	// --- Delegating Methods (thin wrappers around PhysicsEngine) ---

	UFUNCTION(BlueprintCallable, CallInEditor, Category = "MuJoCo|Control")
	void ResetSimulation();

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "MuJoCo|Status")
	FString GetLastCompileError() const;

	/** Pauses the async loop, steps N times synchronously, then restores pause state. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Control")
	void StepSync(int32 NumSteps);

	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Control")
	bool CompileModel();

	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Snapshot")
	UMjSimulationState* CaptureSnapshot();

	/** Restore is scheduled for the next physics step (not applied immediately). */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Snapshot")
	void RestoreSnapshot(UMjSimulationState* Snapshot);
};
