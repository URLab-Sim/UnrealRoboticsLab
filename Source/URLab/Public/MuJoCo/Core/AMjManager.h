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
#include "MuJoCo/Entity/MjPoseSource.h"
#include "MuJoCo/Core/MjSimClock.h"
#include "Bridge/BridgeServer.h"
#include "State/MjStateCollector.h"
#include <atomic>
#include "AMjManager.generated.h"

// Forward declarations
class AMjEntity;
class AMjEntityPawn;
class AMjHeightfieldActor;
class UMjCamera;
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
class UMjAppearanceStore;
class UMjUserChannelComponent;
struct FMjUserChannel;
enum class EMjUserChannelKind : uint8;

class FURLabRpcDispatcher;
class IMjSnapshotPublisher;
class IMjStateConsumer;
class IMjStateProducer;

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
class URLAB_API AAMjManager : public AActor, public IMjSimClock
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

	/**
	 * Possess the interactive pawn for the named entity with the local player controller, replacing the
	 * retired articulation's possess button. Only an entity that opted in has a pawn -- one is spawned
	 * and configured off its authoring articulation at the post-compile handoff -- so a name with no
	 * pawn (a prop, a scene body, an entity that never authored possess) is a clean no-op returning
	 * false. The pawn's spring-arm camera re-homes onto the render view's root body for the entity so it
	 * tracks the physics, exactly as the articulation's camera hung off RootBody. Game thread.
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Possession")
	bool PossessEntity(FName EntityName);

	/** Release the possessed entity pawn and return the controller to the pawn it held before. Game thread. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Possession")
	void UnpossessEntity();

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "MuJoCo|Global")
	// By value: forwards the engine's locked snapshot (see UMjPhysicsEngine::GetAllArticulations).
	TArray<AMjArticulation*> GetAllArticulations() const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "MuJoCo|Global")
	TArray<UMjQuickConvertComponent*> GetAllQuickComponents() const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "MuJoCo|Global")
	TArray<AMjHeightfieldActor*> GetAllHeightfields() const;

	/**
	 * Every camera the RPC surface can serve, in one place: the manager's global cameras
	 * plus the compiled render view's re-homed body-fixed cameras. When the render view
	 * carries cameras it owns the model cameras, so the transitional articulation-mounted
	 * cameras are skipped -- enumerating both would collide on the shared canonical name
	 * and double the GPU capture. Falls back to the articulation cameras only while the
	 * view has none. Works with the articulations alive or gone.
	 */
	void CollectCameras(TArray<UMjCamera*>& Out) const;

	/**
	 * The visual domain-randomization channel: name-keyed geom appearance overrides
	 * that re-drive live material instances off the mjModel. Lazily created on first
	 * request and owned here (so overrides outlive any one render scene and a
	 * render-server swap can re-apply them). Never null.
	 */
	UMjAppearanceStore* GetAppearanceStore();

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

	// --- Capabilities (composable, orthogonal to the pose source) ---

	/**
	 * @brief Whether this instance publishes camera frames (its model cameras and/or its own view).
	 *
	 * A composable capability, not a mode: it turns the "render server" role on or off independently
	 * of the pose source, and is advertised in the handshake so peers know this instance streams.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Capabilities")
	bool bStreamCameras = true;

	/**
	 * @brief Whether this instance accepts interactive input (xfrc / wrench / drag) into its sim.
	 *
	 * A composable capability, not a mode: it turns the "viewer accepts input" role on or off. When
	 * off, forwarded perturbation input is rejected. Advertised in the handshake.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Capabilities")
	bool bAcceptInput = true;

	/** True if this instance currently has the given capability enabled. */
	UFUNCTION(BlueprintPure, Category = "MuJoCo|Capabilities")
	bool HasCapability(EMjCapability Capability) const;

	// --- Remote Stepping ---

	/**
	 * @brief Project pose-source pin for the simulation.
	 *
	 * Honoured only when bPinStepMode is set: it locks the engine to this pose
	 * source and rejects mode-switch RPCs with error("mode_locked_by_server").
	 * When bPinStepMode is false (the default) the client is free to promote to
	 * Stepped / StatePushed on hello and this value is ignored (the session
	 * resolves to FreeRun until the client picks).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Remote Stepping")
	EMjStepMode StepMode = EMjStepMode::FreeRun;

	/**
	 * @brief Pin StepMode as the server-locked pose source.
	 *
	 * False (default) is the old "Auto" policy: the client picks. True pins the
	 * engine to StepMode and rejects set_mode.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Remote Stepping")
	bool bPinStepMode = false;

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

	/** The entity pawn currently possessed via PossessEntity, or invalid when none. */
	TWeakObjectPtr<AMjEntityPawn> PossessedEntityPawn;

	/** The pawn the controller held before the first PossessEntity, restored on release. */
	TWeakObjectPtr<APawn> PrePossessPawn;

	/** Name-keyed geom appearance overrides (visual DR). Lazily created; see
	 *  GetAppearanceStore. Transient so it is GC-rooted across PIE without
	 *  serialising the overrides. */
	UPROPERTY(Transient)
	TObjectPtr<UMjAppearanceStore> AppearanceStore;

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

	// Owner-side viewer bus (direct/live): the "render" (per-body transforms +
	// optional debug tier) topic, fanned out through the agnostic publish
	// abstraction (ZMQ now; SHM/ROS/gRPC via new UURLabPublishTransport impls),
	// on their own endpoint separate from the state bus. Empty unless
	// bBroadcastViewers was set on an owner. Written from FanOutStateSnapshot
	// (physics thread); TransportShutdown'd + cleared in EndPlay. (The qpos render
	// tier -- the old "viewer" {t,qpos,qvel} topic -- was removed in Phase 3.2.)
	UPROPERTY(Transient)
	TArray<TObjectPtr<class UURLabPublishTransport>> ViewerBusTransports;
	/** Fan a topic's payload out to every viewer-bus publish transport. */
	void PublishOnViewerBus(const FString& Topic, const TArray<uint8>& Payload);

	/** Debug-tier subscription capabilities (source-of-truth §8.2). Gate the
	 *  optional fields appended to a render frame; a subscriber that requests
	 *  neither cap pays zero extra bytes. Phase 2.3 establishes this seam only --
	 *  the debug fields themselves (contacts / subtree_com / ctrl / act /
	 *  wrap_xpos / xfrc_applied / eq / sensor / light) are computed + serialized
	 *  in Phase 9.1. */
	struct FMjRenderDebugCaps
	{
		bool bStreamContacts = false;  // contacts[] (capped at MaxContacts)
		bool bStreamOverlay = false;   // derived-decor bundle (§8.2)
		int32 MaxContacts = 0;         // subscriber-set contact cap; 0 => none
	};

	/** Build the always-present render tier (§8.1: per-body + camera world
	 *  transforms + frame id) into a fresh JSON object. Shared by every tier
	 *  encoding (ZMQ topic / gRPC format / SHM ring). */
	TSharedPtr<class FJsonObject> BuildRenderFrame(struct mjModel_* m, struct mjData_* d);
	/** Capability-gated, count-capped seam that appends the optional debug tier
	 *  (§8.2) onto a render frame, computed straight from (m,d) after the step.
	 *  Serializes contacts[] (<= Caps.MaxContacts) under StreamContacts, and the
	 *  derived-decor bundle (xfrc_applied / subtree_com / ctrl / act / wrap paths
	 *  / eq_active + anchors / sensordata / light xpos+xdir) under StreamOverlay.
	 *  A subscriber that requested neither cap pays zero extra bytes. */
	void AppendRenderDebugFields(TSharedPtr<class FJsonObject>& Frame,
		struct mjModel_* m, struct mjData_* d, const FMjRenderDebugCaps& Caps);
	/** Parse the debug-tier subscription capabilities off a subscribe request
	 *  (msgpack payload) into an FMjRenderDebugCaps: `contacts` (bool) toggles
	 *  StreamContacts, `overlay` (bool) toggles StreamOverlay, and
	 *  `maxcontacts` (int, alias `max_contacts`) sets the contact cap. Unknown /
	 *  missing keys leave the corresponding cap off. */
	static FMjRenderDebugCaps ParseRenderDebugCaps(const TArray<uint8>& SubscribePayload);
	/** The debug-tier caps a render subscriber negotiated (source-of-truth §8.2).
	 *  Defaults to none so a lean mirror pays zero extra bytes; a subscribe request
	 *  that asks for contacts/overlay populates it via ParseRenderDebugCaps and it
	 *  gates what PublishRenderFrame appends. */
	FMjRenderDebugCaps ActiveRenderDebugCaps;
	/** Guards ActiveRenderDebugCaps: written on a gRPC server thread (a mirror's
	 *  subscribe, via ApplyRenderDebugCapsFromSubscribe) and read on the physics
	 *  thread (PublishRenderFrame). Matches the FCriticalSection pattern used by the
	 *  snapshot/state consumer registries and the gRPC render cache. */
	mutable FCriticalSection RenderDebugCapsLock;
	/** Fold a render subscriber's negotiated debug caps -- parsed from its RAW
	 *  subscribe payload ({contacts, overlay, maxcontacts}) -- into
	 *  ActiveRenderDebugCaps, thread-safely. Bound to
	 *  FMjExternalTransportProvider::OnRenderDebugCapsRequested so a UE gRPC owner
	 *  honors what a mirror asks for (the reverse leg of the render sink). Caps are
	 *  UNIONed (OR'd), never cleared: with >1 subscriber the owner streams the
	 *  superset any mirror requested (caps are per-owner, §8.2/9.1). A lean mirror's
	 *  payload parses to no caps, so a session with only lean subscribers keeps the
	 *  caps off and pays zero extra bytes. */
	void ApplyRenderDebugCapsFromSubscribe(const TArray<uint8>& SubscribePayload);
	/** Encode the render tier (+ optional debug fields) from (m,d) and PUB it on
	 *  the viewer bus under the "render" topic, so a fast-path renderer can mirror
	 *  this owner with no physics. */
	void PublishRenderFrame(struct mjModel_* m, struct mjData_* d);
	/** Monotonic frame id for the render bus. */
	uint64 GeomBroadcastFrame = 0;
	/** Binds FMjExternalTransportProvider::OnRenderDebugCapsRequested (in BeginPlay,
	 *  on the owning Instance) -> ApplyRenderDebugCapsFromSubscribe, so a mirror's
	 *  gRPC subscribe reaches this owner's caps. Removed in EndPlay so the gRPC
	 *  thread never invokes a lambda capturing a destroyed manager. */
	FDelegateHandle RenderDebugCapsHandle;

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

	/** The one lightweight render view for the compiled play path: an AMjRenderer
	 *  seeded from the borrowed compiled model, drawing the scene geometry from the
	 *  render snapshot. Spawned at BeginPlay in game worlds only, never in automation. */
	TObjectPtr<class AMjRenderer> CompiledRenderView;
	TObjectPtr<class UMjOverlayRenderer> OverlayRenderer;
	const struct mjModel_* CompiledViewModel = nullptr;

	/** Build the compiled render view + overlay once the engine has a model. */
	void BuildRuntimeView();
	/** Drive the compiled render view from one render snapshot (game thread, under
	 *  the render-state lock). */
	void DriveCompiledRenderView(const struct FMjRenderSnapshot& Snap);

	/** The one runtime debug-overlay drive: mjModel + snapshot driven via UMjOverlayRenderer, in every
	 *  runtime mode (compiled view / mirror / raw). The articulation debug-draw is edit-time only. */
	void DriveOverlays(const struct FMjRenderSnapshot& Snap);

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

public:
	virtual void Tick(float DeltaTime) override;

	/** Pull the latest physics render snapshot and push it onto the UE
	 *  actor/component transforms. Normally driven once per frame from Tick.
	 *  Records the applied snapshot's FrameId / SimTime so cameras can tag
	 *  their readbacks with the post-step state they show. Game thread only. */
	void ApplyLatestRenderState();

	/** The compiled play render view (approach B), or null in editor / automation. */
	class AMjRenderer* GetCompiledRenderView() const { return CompiledRenderView; }

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

	// --- IMjSimClock: cameras read the applied render state through this, so the
	// capture/delay pipeline never depends on the AAMjManager singleton. ---
	virtual uint64 GetAppliedFrameId() const override { return GetLastAppliedFrameId(); }
	virtual double GetAppliedSimTime() const override { return GetLastAppliedSimTime(); }

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
