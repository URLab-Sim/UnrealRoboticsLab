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
#include "Components/ActorComponent.h"
#include "mujoco/mujoco.h"
#include "MuJoCo/Core/MjRenderSnapshot.h"
#include "MuJoCo/Spec/MjBinding.h"
#include "MuJoCo/Spec/MjSceneAssembly.h"
#include "MuJoCo/Spec/MjSceneSpec.h"
#include <functional>
#include <atomic>
#include "MjPhysicsEngine.generated.h"

// Forward declarations
class AMjArticulation;
class AMjHeightfieldActor;
class UMjQuickConvertComponent;
class UMjSimulationState;

// Shared enum: control source for articulations and physics engine
UENUM(BlueprintType)
enum class EControlSource : uint8
{
	ZMQ UMETA(DisplayName = "External (ZMQ)"),
	UI UMETA(DisplayName = "Internal (UI)")
};

/**
 * @enum EStepMode
 * @brief Controls how the physics engine advances the simulation.
 *
 * - Live: physics thread advances at the model's timestep on its own. Publishers
 *   stream state, control subscriber writes ctrl. Live / streaming workflows.
 * - Direct: physics thread blocks on a step-request queue fed by UURLabZmqRpcTransport.
 *   RPC writes ctrl, calls mj_step n times, returns observations. Deterministic
 *   RL training where UE owns the integrator.
 * - Puppet: physics thread blocks on a push-state queue fed by UURLabZmqRpcTransport.
 *   RPC writes qpos/qvel, calls mj_forward, returns observations. MJX / Jax-owned
 *   rollouts where the client owns the integrator.
 * - Auto: starts Live, hello RPC promotes to Direct or Puppet on first
 *   client connection, demotes back when client disconnects.
 */
UENUM(BlueprintType)
enum class EStepMode : uint8
{
	Live UMETA(DisplayName = "Live (streaming)"),
	Direct UMETA(DisplayName = "Direct (RPC step)"),
	Puppet UMETA(DisplayName = "Puppet (RPC push-state)"),
	Auto UMETA(DisplayName = "Auto (client picks)")
};

/**
 * The MJCF a compiled scene was built from, with everything else it needs.
 *
 * A scene is not always one spec: it may reference participant models by
 * VFS name, and a client cannot reload it without them. So the payload is the
 * scene text plus the specs and asset files that go in the VFS alongside
 * it, which is also exactly what the compiler was given.
 */
struct URLAB_API FMjCompiledScene
{
	/** The scene spec's MJCF text. */
	FString Xml;

	/** ParticipantXml the scene references, keyed by the VFS name it references them under. */
	TMap<FString, FString> ParticipantXml;

	/** The asset files mounted alongside the scene, keyed by mounted name. */
	TMap<FString, FString> AssetFiles;

	bool IsValid() const { return !Xml.IsEmpty(); }
};

/**
 * @class UMjPhysicsEngine
 * @brief Core physics engine component that owns the MuJoCo simulation lifecycle.
 *
 * Manages mjSpec compilation, mjModel/mjData ownership, the async physics loop,
 * and callback registration for pre/post step hooks.
 */
UCLASS(ClassGroup = (MuJoCo), meta = (BlueprintSpawnableComponent))
class URLAB_API UMjPhysicsEngine : public UActorComponent
{
	GENERATED_BODY()

public:
	UMjPhysicsEngine();
	virtual void BeginDestroy() override;
#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

	// --- MuJoCo Core Pointers ---

	mjModel* m_model = nullptr;
	mjData* m_data = nullptr;

	mjModel* GetModel() const { return m_model; }
	mjData* GetData() const { return m_data; }

	// --- Thread Synchronization ---

	mutable FCriticalSection CallbackMutex;
	std::atomic<bool> bShouldStopTask{false};
	TFuture<void> AsyncPhysicsFuture;

	/** True for exactly the lifetime of the async worker lambda. The direct-mode
	 *  step body reads this to decide whether to submit to the worker or run the
	 *  handler inline: unlike AsyncPhysicsFuture.IsValid() it is cleared the
	 *  instant the worker returns, so a joined-but-not-yet-reset future can't be
	 *  mistaken for a live worker. */
	std::atomic<bool> bWorkerRunning{false};

	/** Wakes the async physics worker when a step request lands in
	 *  direct/puppet mode. Dispatcher Triggers on enqueue; worker
	 *  Waits on this in lieu of the real-time spin pacer when the
	 *  client owns the clock. Allocated when the worker starts,
	 *  returned to the pool on shutdown. */
	FEvent* StepRequestEvent = nullptr;

	// --- Worker shadow state (lock-free reads on the physics thread) ---
	//
	// The physics worker must not read UPROPERTYs (torn cross-thread) or
	// reach into the owning actor for the step mode. These mirror the
	// authoritative values: SetPaused / SetSimSpeed / SetStepMode
	// (plus PostEditChangeProperty for details-panel edits) keep them in
	// sync, and RunMujocoAsync seeds them when the worker starts.
	std::atomic<bool> bPausedAtomic{true};
	std::atomic<float> SimSpeedAtomic{100.0f};
	std::atomic<EStepMode> ResolvedStepMode{EStepMode::Live};

	/** Set by the game thread each time it consumes the render snapshot; the
	 *  live-mode worker publishes a new snapshot only when it is set, so the
	 *  full-state copy runs at the consumer's frame rate rather than the
	 *  physics rate. Direct/puppet publish every step (frame association). */
	std::atomic<bool> bSnapshotWanted{true};

	/** Worker-thread only. Reset at the top of each worker iteration; a step
	 *  handler that publishes the render snapshot itself (direct mode captures
	 *  its exact frame id for the reply) sets this so the loop tail does not
	 *  publish again and bump the id past what the step reported. */
	bool bRenderStatePublishedThisStep = false;

	// --- Step Callbacks ---

	/** If bound, replaces mj_step (direct/puppet stepping and replay).
	 *  Returns true iff it advanced sim state this call (dequeued work and
	 *  stepped); false on an idle wake with nothing to do, so the worker
	 *  loop can skip the render-snapshot publish instead of inflating
	 *  FrameId on unchanged state. A handler that steps owns its own
	 *  OnPostStep notification (per sub-step for direct mode). */
	using FMujocoStepCallback = std::function<bool(mjModel*, mjData*)>;
	FMujocoStepCallback CustomStepHandler;

	/** Called after mj_step (or custom step); used for recording. */
	using FMujocoPostStepCallback = std::function<void(mjModel*, mjData*)>;
	FMujocoPostStepCallback OnPostStep;

	void SetCustomStepHandler(FMujocoStepCallback Handler);
	void ClearCustomStepHandler();

	// --- Pending State (Thread-Safe Atomics) ---

	std::atomic<bool> bPendingReset{false};
	std::atomic<bool> bPendingRestore{false};
	TArray<double> PendingStateVector;
	int32 PendingStateMask = 0;

	// --- Simulation Options ---

	/** 100 = realtime, 50 = half speed. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Options", meta = (ClampMin = "5", ClampMax = "100"))
	float SimSpeedPercent = 100.0f;

	/**
	 * Worker threads for MuJoCo's per-step thread pool (mju_threadpool).
	 * 0 = off (single-threaded, the default). >=1 parallelises collision
	 * detection and island constraint solving inside mj_step; the benefit
	 * scales with contact count / number of constraint islands, so it helps
	 * large scenes and is usually a no-op for a single small articulation.
	 * Clamped at runtime to the machine's logical CPU core count
	 * (MaxWorkerThreads(), cross-platform). Applied to the live mjData on
	 * change (idempotent) so it can be set at edit time or at runtime via the
	 * set_sim_options RPC.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Options", meta = (ClampMin = "0", UIMax = "32"))
	int32 NumWorkerThreads = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Status")
	bool bIsPaused = true;

	/** Saves compiled scene XML and MJB to Saved/URLab/ on each compile. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Debug")
	bool bSaveDebugXml = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Runtime")
	EControlSource ControlSource;

	// --- Registered Scene Objects ---

	UPROPERTY()
	TArray<UMjQuickConvertComponent*> m_MujocoComponents;

	UPROPERTY()
	TArray<AMjArticulation*> m_articulations;

	UPROPERTY()
	TArray<AMjHeightfieldActor*> m_heightfieldActors;

	/** O(1) articulation lookup. Key = actor name. */
	TMap<FString, AMjArticulation*> m_ArticulationMap;

	/** Error string from the most recent Compile(); empty on success. */
	FString m_LastCompileError;

	/** Absolute paths of every mesh / texture asset the last compile mounted.
	 *  Collected by the spec's own asset pass, which is the same pass the
	 *  compile fed its VFS from. Used by the bridge handshake (opt-in) to ship
	 *  the model and its assets to a remote client. */
	UPROPERTY()
	TMap<FString, FString> ActiveAssetFiles;

	// --- Compilation ---

	/**
	 * Compile the level and install the result as the live model.
	 *
	 * A thin spelling of `InstallCompiledSpec` that reports rather than
	 * returns, because that is what every existing caller wants and the reason
	 * the two stood side by side has gone: the spec is what compiles now.
	 */
	void Compile();

	void ApplyOptions();

	/**
	 * Project the level into a scene assembly: the manager's sections as the
	 * scene root, and one participant per articulation and per scene
	 * contributor, ordered by prefix.
	 *
	 * A projection rather than a stored membership list, so there is no second
	 * source of truth to reconcile against the actors actually present. Not
	 * const: a contributor authors its spec from Unreal content here, which
	 * is the one point at which a sampled terrain or a converted mesh becomes
	 * MJCF, and it has to happen against the level as it is now.
	 */
	void BuildSceneAssembly(FSceneAssembly& Out);

	/**
	 * Every object in the world that contributes a spec to the scene:
	 * heightfield actors, quick-convert components, anything else implementing
	 * the interface. Actors and components both, because the two consumers that
	 * exist are one of each.
	 */
	TArray<UObject*> GatherSceneContributors() const;

	/**
	 * Compile the level's specs and install the result as the live model.
	 *
	 * The spec route from end to end: the specs are built and composed, the
	 * model becomes the engine's, mjData is made for it, every element that
	 * survived the compile is told the id it received, and each articulation's
	 * control slots are sized to the scene it compiled into. After this a caller
	 * can read `d->sensordata` at an element's `BoundId` and get an answer.
	 *
	 * Transactional. Everything that can fail -- building the specs, composing
	 * them, compiling, making the data -- happens before anything running is
	 * touched, so a compile that fails costs the caller a diagnostic and leaves
	 * the session stepping exactly the model it was already stepping.
	 */
	bool InstallCompiledSpec(FString& OutError);

	/**
	 * Drop the installed model, its data, and the scene they came from.
	 *
	 * The model belongs to the compiled scene rather than to this component, so
	 * it cannot be freed on its own: the specs it was compiled from have to
	 * outlive it, and only the scene knows that order. Callers that used to
	 * delete the two pointers call this instead.
	 */
	void ReleaseCompiledScene();

	/**
	 * The MJCF of the scene the engine currently holds compiled.
	 *
	 * The single source of the bridge handshake's `mjcf_compiled` and of the
	 * render farm's upload: neither reaches into the engine's compile state for
	 * the text, so where that text comes from stays the engine's business.
	 * Returns false with the reason in `OutError` when there is nothing to give.
	 */
	bool BuildCompiledScene(FMjCompiledScene& Out, FString& OutError) const;

	/** (Re)build or free MuJoCo's per-step worker thread pool on the live
	 *  mjData from NumWorkerThreads. Idempotent; safe at setup or runtime. */
	void ApplyThreadPool();

	/** Max useful worker threads = logical CPU cores (cross-platform).
	 *  Used to clamp NumWorkerThreads. */
	static int32 MaxWorkerThreads();

	// --- Runtime ---

	void RunMujocoAsync();
	void SetPaused(bool bPaused);

	/** Set the real-time speed target (percent). Writes the UPROPERTY (for
	 *  the details panel) and the worker's lock-free shadow. */
	void SetSimSpeed(float Percent);

	/** Single entry point for the runtime step mode. Stores the resolved mode
	 *  the worker honours (pacing + whether it runs the UE controller pass;
	 *  Auto resolves to Live) and unpauses the worker for client-driven modes
	 *  (direct/puppet) so the async loop calls the step handler and drains the
	 *  request queue. The RPC dispatcher is the runtime owner; call on every
	 *  mode change. */
	void SetStepMode(EStepMode Mode);

	/** The resolved step mode the physics worker is currently pacing off
	 *  (Auto already collapsed to Live). This is the authoritative value the
	 *  loop reads, so it is what regression coverage for the live 10 Hz lock
	 *  should assert. */
	EStepMode GetStepMode() const { return ResolvedStepMode.load(std::memory_order_acquire); }

	bool IsRunning() const;
	bool IsInitialized() const;
	float GetSimTime() const;
	float GetTimestep() const;
	void ResetSimulation();
	void StepSync(int32 NumSteps);
	bool CompileModel();
	UMjSimulationState* CaptureSnapshot();
	/** Restore is scheduled for the next physics step. */
	void RestoreSnapshot(UMjSimulationState* Snapshot);
	void SetControlSource(EControlSource NewSource);
	EControlSource GetControlSource() const;
	AMjArticulation* GetArticulation(const FString& ActorName) const;
	/** The live articulation registry. Registration happens in bulk at compile
	 *  time while the worker thread is stopped and joined, so the
	 *  array is immutable for the duration of a play session. The returned
	 *  reference is therefore stable to read on the game thread, but it is NOT a
	 *  synchronised snapshot: it must not be retained across a recompile, and
	 *  callers on other threads that need a stable copy must take one themselves.
	 *  The physics worker iterates the underlying array directly, not through
	 *  this accessor. */
	const TArray<AMjArticulation*>& GetAllArticulations() const;

	/** Register an articulation into the registry the physics worker iterates
	 *  (ApplyControls). Takes CallbackMutex so bulk registration can't tear the
	 *  array or the name map out from under a step; in practice registration
	 *  runs at compile time with the worker joined. */
	void RegisterArticulation(AMjArticulation* Articulation);
	TArray<UMjQuickConvertComponent*> GetAllQuickComponents() const;
	TArray<AMjHeightfieldActor*> GetAllHeightfields() const;
	FString GetLastCompileError() const;

	// --- Callback Registration ---

	// TFunction (not std::function): TArray-safe move semantics. Storing
	// std::function in TArray here corrupted captures across reallocation.
	using FPhysicsCallback = TFunction<void(mjModel*, mjData*)>;

	void RegisterPreStepCallback(FPhysicsCallback Callback);
	void RegisterPostStepCallback(FPhysicsCallback Callback);
	void ClearCallbacks();

	// --- RenderState pump (MuJoCo -> UE) -------------------------------

	/**
	 * @brief Copies the live mjData into the engine-owned render
	 * snapshot. Bumps RenderSnapshot.FrameId. Must be called by the
	 * thread that just finished mj_step while it holds CallbackMutex.
	 *
	 * Lock ordering: CallbackMutex (outer) -> RenderStateMutex (inner).
	 * RenderStateMutex is taken inside this method only; callers must
	 * not hold it.
	 */
	void PushRenderState();

	/**
	 * @brief Runs the visitor against the latest render snapshot
	 * under RenderStateMutex. Holds the lock for the duration of the
	 * visitor, so the visitor must complete promptly and must not
	 * acquire CallbackMutex (or any lock that the producer takes
	 * under CallbackMutex).
	 */
	void WithRenderState(TFunctionRef<void(const FMjRenderSnapshot&)> Visitor) const;

	/** Current render-snapshot frame id (monotonic, bumped each PushRenderState
	 *  i.e. each step's post-step state). Returned in step replies so a client
	 *  can fetch the matching camera frame by id. Thread-safe. */
	uint64 GetRenderFrameId();

	// --- Command channel (UE -> MuJoCo) --------------------------------
	//
	// Game-thread writers (mocap, wrench, sleep) enqueue under
	// CommandMutex; the stepping thread drains the queue inside
	// CallbackMutex immediately before mj_step. This keeps every
	// mjData mutation on the physics thread and avoids lock-free
	// races on xfrc_applied / mocap_pos / body_awake.
	//
	// Last-write-wins per body per drain: re-submitting in one game
	// frame collapses to the final value applied for the next step.

	/** World-frame mocap pose for a mocap body, in MuJoCo coordinates. */
	void SubmitMocapPose(int32 BodyId, const double Pos[3], const double Quat[4]);

	/** Set xfrc_applied[body] = [tx, ty, tz, fx, fy, fz] (MuJoCo order). */
	void SubmitWrench(int32 BodyId, const double Xfrc[6]);

	/** Zero xfrc_applied for the body on the next drain. */
	void SubmitClearForce(int32 BodyId);

	/** Wake the body and its kinematic tree synchronously. Takes
	 *  CallbackMutex briefly so the write is observable to the caller
	 *  immediately (matches the user-facing one-shot contract). */
	void ApplyWakeBody(int32 BodyId);

	/** Put the body and its kinematic tree to sleep synchronously.
	 *  Takes CallbackMutex briefly (see ApplyWakeBody). */
	void ApplySleepBody(int32 BodyId);

	/** Sleep state of a body. Takes CallbackMutex briefly so it pairs with the
	 *  synchronous ApplyWakeBody / ApplySleepBody contract. Bodies of an
	 *  uncompiled or unbound model read as awake. */
	bool IsBodyAwake(int32 BodyId) const;

	// --- Synchronous live-model / live-data edits ----------------------
	//
	// Runtime effects a Blueprint caller expects to observe on the very
	// next read, so they take CallbackMutex and write through rather than
	// queueing. The spec field remains the authority: these reach the
	// compiled model only, and a recompile reconciles from the spec.

	/** Write d->qpos for a 1-DOF joint's first slot. */
	void ApplyJointPosition(int32 JointId, double Value);

	/** Write d->qvel for a 1-DOF joint's first slot. */
	void ApplyJointVelocity(int32 JointId, double Value);

	/** Write the sliding-friction coefficient of a geom (m->geom_friction[0]). */
	void ApplyGeomFriction(int32 GeomId, double Slide);

	/** Write up to 6 gear entries of an actuator (m->actuator_gear). */
	void ApplyActuatorGear(int32 ActuatorId, TConstArrayView<double> Gear);

private:
	TArray<FPhysicsCallback> PreStepCallbacks;
	TArray<FPhysicsCallback> PostStepCallbacks;

	// --- RenderState plumbing ------------------------------------------

	/** The MJCF the installed model was compiled from, and the specs that
	 *  MJCF references by VFS name. Kept because a client cannot reload a scene
	 *  from the root text alone, and because the model itself cannot be turned
	 *  back into the text that produced it. */
	FString CompiledXml;
	TMap<FString, FString> ParticipantXml;

	/** The binding of the model currently installed.
	 *
	 *  The next install needs it: an element's simulation state lives at an
	 *  address only the binding of the compile that produced it can resolve, and
	 *  by the time the new binding exists the old model is gone. Read only
	 *  between the join and the teardown, while `m_model` is still the model it
	 *  was built against. */
	FMjBinding InstalledBinding;

#if URLAB_MJ_GEN
	/**
	 * The compiled scene the installed model belongs to.
	 *
	 * It owns the model, the composed scene spec and every participant spec, in
	 * that destruction order. `m_model` is an alias into it and never an owner,
	 * which is why replacing this is what retires a model. Held behind a pointer
	 * so a worker that could not be joined can be left holding the model it is
	 * inside rather than having it freed underneath it.
	 */
	TUniquePtr<urlab::spec::FMjCompiledScene> InstalledScene;
#endif

	/** Write the compiled scene and its MJB to Saved/URLab. Honours bSaveDebugXml. */
	void SaveDebugArtifacts() const;

	/** Guards RenderSnapshot. Inner to CallbackMutex on the producer
	 *  path; held alone on the consumer (game thread) path. */
	mutable FCriticalSection RenderStateMutex;

	/** Engine-owned snapshot buffer. Sized on model load. */
	FMjRenderSnapshot RenderSnapshot;

	// --- Command channel plumbing --------------------------------------

	struct FMocapPose
	{
		double Pos[3];
		double Quat[4];
	};

	struct FWrench
	{
		double Xfrc[6];
	};

	struct FCommandQueue
	{
		TMap<int32, FMocapPose> MocapPoses;
		TMap<int32, FWrench> WrenchSets;
		TSet<int32> WrenchClears;

		bool IsEmpty() const
		{
			return MocapPoses.Num() == 0
				&& WrenchSets.Num() == 0
				&& WrenchClears.Num() == 0;
		}
	};

	/** Independent of CallbackMutex / RenderStateMutex. Held only
	 *  briefly by Submit* and by the drain swap. */
	FCriticalSection CommandMutex;
	FCommandQueue PendingCommands;

	/** Drains PendingCommands into m_data. Must be called by the
	 *  stepping thread while it already holds CallbackMutex. Returns true
	 *  iff it applied at least one command, so the caller can treat a mocap
	 *  / wrench edit as a state advance (publish it) even while paused. */
	bool DrainCommands();
};

/**
 * One value from the engine's latest published snapshot, or zero when the
 * index is outside what the snapshot holds.
 *
 * The Blueprint-facing accessors read here rather than from live mjData. The
 * physics thread publishes one coherent copy per step, so two questions asked
 * in the same frame cannot be answered from two different steps; the price is
 * that the answer is the state at the end of the last completed step rather
 * than whatever the integrator is part way through writing.
 */
template <typename FPickArray>
double MjSnapshotValue(const UMjPhysicsEngine& Engine, int32 Index, FPickArray&& PickArray)
{
	double Value = 0.0;
	Engine.WithRenderState([&](const FMjRenderSnapshot& Snapshot) {
		const auto& Array = PickArray(Snapshot);
		if (Array.IsValidIndex(Index))
		{
			Value = static_cast<double>(Array[Index]);
		}
	});
	return Value;
}

/**
 * `Count` values from `Index` of one of the snapshot's arrays, into `Out`.
 *
 * False leaves `Out` untouched: the range is not in the snapshot, which is
 * what a caller sees before the first publish or across a recompile.
 */
template <typename FPickArray>
bool MjSnapshotRange(const UMjPhysicsEngine& Engine, int32 Index, int32 Count, double* Out,
	FPickArray&& PickArray)
{
	bool bRead = false;
	Engine.WithRenderState([&](const FMjRenderSnapshot& Snapshot) {
		const auto& Array = PickArray(Snapshot);
		if (Count <= 0 || Index < 0 || Index + Count > Array.Num())
		{
			return;
		}
		for (int32 I = 0; I < Count; ++I)
		{
			Out[I] = static_cast<double>(Array[Index + I]);
		}
		bRead = true;
	});
	return bRead;
}
