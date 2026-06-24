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

#include "MuJoCo/Components/QuickConvert/MjQuickConvertComponent.h"
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "Bridge/RpcDispatcher.h"
#include "Bridge/BridgeServer.h"
#include "Transport/SnapshotPublisher.h"
#include <atomic>
#include "AMjManager.generated.h"

// Forward declarations
class AMjHeightfieldActor;
class UMjPhysicsEngine;
class UMjDebugVisualizer;
class UMjNetworkManager;
class UMjInputHandler;
class UMjPerturbation;
class UMjSimulationState;
class UMjBody;

/**
 * @struct FMjEntityRecord
 * @brief Cached non-articulation entity metadata: a UMjBody whose owner is
 *        not an AMjArticulation (props, free-jointed scene objects, ...).
 *        Built once at session start (PostCompile) and consumed by
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

	/** Set in BeginPlay, cleared in EndPlay. Use GetManager() from Blueprints. */
	static AAMjManager* Instance;

	UFUNCTION(BlueprintPure, Category = "MuJoCo|Global", meta = (DisplayName = "Get MuJoCo Manager"))
	static AAMjManager* GetManager();

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

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "MuJoCo|Global")
	TArray<AMjArticulation*> GetAllArticulations() const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "MuJoCo|Global")
	TArray<UMjQuickConvertComponent*> GetAllQuickComponents() const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "MuJoCo|Global")
	TArray<AMjHeightfieldActor*> GetAllHeightfields() const;

	/** Non-articulation entity table built in PostCompile; empty until then. */
	const TArray<FMjEntityRecord>& GetEntities() const { return EntityCache; }

	/** Refresh the entity cache; called from PostCompile. */
	void BuildEntityCache();

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

	/**
	 * @brief The resolved, authoritative step mode the physics loop runs under.
	 *
	 * `StepMode` above is the *configured* value and may be `Auto`, which the
	 * dispatcher resolves to a concrete mode (Auto starts Live). The physics
	 * loop must pace off the resolved mode, not the configured one — reading
	 * `StepMode == Live` directly leaves `Auto` (the default) pacing as if it
	 * were Direct, blocking on the step-request timeout at ~10 Hz. The
	 * dispatcher mirrors its `ActiveStepMode` here whenever it changes; defaults
	 * to Live so a bridge-less PIE session runs real-time.
	 */
	std::atomic<EStepMode> EffectiveStepMode{EStepMode::Live};

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

	/** Bound to Tab key. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|UI")
	void ToggleSimulateWidget();

	UPROPERTY()
	UUserWidget* SimulateWidget = nullptr;

protected:
	/** O(1) articulation lookup built in PostCompile. Key = actor name. */
	TMap<FString, AMjArticulation*> m_ArticulationMap;

	TArray<FMjEntityRecord> EntityCache;

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

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

public:
	virtual void Tick(float DeltaTime) override;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Mujoco Physics|Objects")
	TArray<UMjQuickConvertComponent*> m_MujocoComponents;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Mujoco Physics|Objects")
	TArray<AMjArticulation*> m_articulations;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Mujoco Physics|Objects")
	TArray<AMjHeightfieldActor*> m_heightfieldActors;

	void Compile();
	void PreCompile();
	void PostCompile();

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
