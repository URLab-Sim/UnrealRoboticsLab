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

#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "State/MjCanonicalName.h"
#include "MuJoCo/Convert/MjQuickConvertComponent.h"
#include "MuJoCo/Convert/AMjHeightfieldActor.h"
#include "MuJoCo/Core/MjSimulationState.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjSceneOptions.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjSceneContributor.h"
#include "MuJoCo/Utils/URLabAxisConv.h"
#include "EngineUtils.h"
#include "Kismet/GameplayStatics.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformMisc.h"
#include "Async/Future.h"
#include "Misc/Paths.h"
#include "XmlFile.h"
#include "Internationalization/Regex.h"
#include "Utils/URLabLogging.h"
#include <atomic>
#if WITH_EDITOR
#include "Misc/MessageDialog.h"
#endif

// Installed as mju_user_error / mju_user_warning so MuJoCo's fatal-error
// path logs via UE_LOG instead of calling exit(1). Without this, any MuJoCo
// internal invariant violation (e.g. "mj_sleep: found sleeping tree N in
// island M" on flex × free-body + SLEEP + MULTICCD) terminates the process:
// exit() unwinds every live FRHIBreadcrumbEventManual's TOptional across
// threads and trips the !Node assertion, killing the editor. Messages are
// deduplicated + throttled so pathological per-step errors can't flood the
// log at step rate.
static FCriticalSection GMujocoLogMutex;
static TMap<FString, int64> GMujocoMsgHistory; // message text -> next step count at which to log
static std::atomic<int64> GMujocoMsgStepCounter{0};

static void URLab_LogMujocoMessage(const TCHAR* Severity, const char* Msg, ELogVerbosity::Type Verbosity)
{
	const FString Text = Msg ? FString(UTF8_TO_TCHAR(Msg)) : FString(TEXT("(null)"));
	const int64 Step = GMujocoMsgStepCounter.fetch_add(1, std::memory_order_relaxed);

	int64 FirstHitStep = -1;
	int64 HitCountSinceLastLog = 0;
	{
		FScopeLock Lock(&GMujocoLogMutex);
		int64* NextLog = GMujocoMsgHistory.Find(Text);
		if (!NextLog)
		{
			// First occurrence — log it and start counting future hits.
			GMujocoMsgHistory.Add(Text, Step + 500); // log again after 500 more messages
			FirstHitStep = Step;
		}
		else if (Step >= *NextLog)
		{
			HitCountSinceLastLog = 500; // approx — we don't track exact
			*NextLog = Step + 500;
			FirstHitStep = Step;
		}
	}

	if (FirstHitStep >= 0)
	{
		if (HitCountSinceLastLog > 0)
		{
			UE_LOG(LogURLab, Warning, TEXT("[MuJoCo %s x~%lld] %s"), Severity, (long long)HitCountSinceLastLog, *Text);
		}
		else
		{
			if (Verbosity == ELogVerbosity::Error)
			{
				UE_LOG(LogURLab, Error, TEXT("[MuJoCo %s] %s"), Severity, *Text);
			}
			else
			{
				UE_LOG(LogURLab, Warning, TEXT("[MuJoCo %s] %s"), Severity, *Text);
			}
		}
	}
}

static void URLab_OnMujocoError(const char* Msg)
{
	URLab_LogMujocoMessage(TEXT("fatal"), Msg, ELogVerbosity::Error);
}

static void URLab_OnMujocoWarning(const char* Msg)
{
	URLab_LogMujocoMessage(TEXT("warn"), Msg, ELogVerbosity::Warning);
}

static bool GMujocoCallbacksInstalled = false;
static void URLab_InstallMujocoCallbacks()
{
	if (GMujocoCallbacksInstalled)
		return;

#if PLATFORM_WINDOWS
	// mujoco.dll is delay-loaded on Windows (URLab.Build.cs adds it via
	// PublicDelayLoadDLLs), and the linker refuses to bind data symbols
	// through a delayed import. Resolve the two mju_user_* function
	// pointers manually via GetDllExport.
	void* Handle = FPlatformProcess::GetDllHandle(TEXT("mujoco.dll"));
	if (!Handle)
	{
		UE_LOG(LogURLab, Warning, TEXT("[URLab] Could not resolve mujoco.dll to install error callbacks"));
		return;
	}

	using ErrorFnPtr = void (*)(const char*);
	// GetDllExport(hMod, "mju_user_error") returns the address of the
	// exported variable itself — i.e. an ErrorFnPtr*.
	ErrorFnPtr* PErr = reinterpret_cast<ErrorFnPtr*>(FPlatformProcess::GetDllExport(Handle, TEXT("mju_user_error")));
	ErrorFnPtr* PWarn = reinterpret_cast<ErrorFnPtr*>(FPlatformProcess::GetDllExport(Handle, TEXT("mju_user_warning")));
	if (PErr)
	{
		*PErr = &URLab_OnMujocoError;
	}
	if (PWarn)
	{
		*PWarn = &URLab_OnMujocoWarning;
	}
	UE_LOG(LogURLab, Log, TEXT("[URLab] MuJoCo error callbacks installed (err=%p warn=%p)"), (void*)PErr, (void*)PWarn);
#else
	// On Linux/macOS the lib is linked directly (no delay-load), so
	// mju_user_error / mju_user_warning are resolvable as ordinary BSS
	// data symbols at link time. Direct assignment is enough — no
	// GetDllHandle / GetDllExport, no hardcoded SONAME literal needed.
	mju_user_error = &URLab_OnMujocoError;
	mju_user_warning = &URLab_OnMujocoWarning;
	UE_LOG(LogURLab, Log, TEXT("[URLab] MuJoCo error callbacks installed (direct)"));
#endif

	GMujocoCallbacksInstalled = true;
}

UMjPhysicsEngine::UMjPhysicsEngine()
{
	PrimaryComponentTick.bCanEverTick = false;

	ControlSource = EControlSource::ZMQ;

	// Auto-reset so each Trigger arms exactly one Wait; coalesces bursts.
	StepRequestEvent = FPlatformProcess::GetSynchEventFromPool(false);

	URLab_InstallMujocoCallbacks();
}

void UMjPhysicsEngine::BeginDestroy()
{
	// Stop and JOIN the async worker before tearing anything down. The
	// worker captures `this` and dereferences m_model / m_data /
	// m_articulations every iteration, and it may be parked on
	// StepRequestEvent — returning that event to the pool (below) while
	// the worker still waits on it is a use-after-free. Wait() outside any
	// lock the worker takes so it can reach its bShouldStopTask check.
	bShouldStopTask = true;
	if (StepRequestEvent)
		StepRequestEvent->Trigger();

	// Bounded join. BeginDestroy runs on the GC path, so an unbounded wait on a
	// wedged mj_step would hang garbage collection (and with it the editor). If
	// the worker does not exit in time we leak its sync event and MuJoCo state
	// rather than block forever or free memory the still-running worker reads.
	bool bWorkerExited = true;
	if (AsyncPhysicsFuture.IsValid())
	{
		constexpr double kBeginDestroyWaitSec = 3.0;
		bWorkerExited = AsyncPhysicsFuture.WaitFor(FTimespan::FromSeconds(kBeginDestroyWaitSec));
		if (!bWorkerExited)
		{
			UE_LOG(LogURLab, Warning,
				TEXT("Physics async worker still running at BeginDestroy after %.1fs; ")
					TEXT("leaking its sync event to avoid a use-after-free in the stuck step."),
				kBeginDestroyWaitSec);
		}
	}

	// Only recycle the event once the worker has provably stopped waiting on it.
	if (bWorkerExited && StepRequestEvent)
	{
		FPlatformProcess::ReturnSynchEventToPool(StepRequestEvent);
		StepRequestEvent = nullptr;
	}
	Super::BeginDestroy();
}

#if WITH_EDITOR
void UMjPhysicsEngine::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);
	// Keep the worker's lock-free shadows in step with details-panel edits
	// of bIsPaused / SimSpeedPercent.
	bPausedAtomic.store(bIsPaused, std::memory_order_release);
	SimSpeedAtomic.store(SimSpeedPercent, std::memory_order_release);
}
#endif

void UMjPhysicsEngine::Compile()
{
	FString Error;
	if (InstallCompiledSpec(Error))
	{
		return;
	}

	UE_LOG(LogURLab, Error, TEXT("Model compilation failed: %s"), *Error);
#if WITH_EDITOR
	FMessageDialog::Open(EAppMsgType::Ok,
		FText::Format(NSLOCTEXT("URLab", "CompileError", "MuJoCo compile failed:\n\n{0}"),
			FText::FromString(Error)));
#endif
}

int32 UMjPhysicsEngine::MaxWorkerThreads()
{
	// Cross-platform (Windows/Linux/Mac) logical-core count — the ceiling on
	// useful MuJoCo worker threads. Floor of 1 so callers always get a sane cap.
	return FMath::Max(1, FPlatformMisc::NumberOfCoresIncludingHyperthreads());
}

void UMjPhysicsEngine::ApplyThreadPool()
{
	if (!m_data)
		return;

	// mju_threadpool is idempotent: same worker count is a no-op, a different
	// count rebuilds the pool, and 0 frees it. So this is safe to call at
	// mjData creation and again at runtime (e.g. from the set_sim_options RPC).
	// The pool is owned by mjData and auto-freed in mj_deleteData.
	const int32 Cap = MaxWorkerThreads();
	const int32 N = FMath::Clamp(NumWorkerThreads, 0, Cap);
	if (NumWorkerThreads > Cap)
	{
		UE_LOG(LogURLab, Warning,
			TEXT("NumWorkerThreads=%d exceeds %d available cores; capping to %d."),
			NumWorkerThreads, Cap, Cap);
	}
	// mju_threadpool is the only public threadpool entry point; mju_numThread
	// lives in MuJoCo's internal engine_thread.h, so don't depend on it here.
	mju_threadpool(m_data, N);
	UE_LOG(LogURLab, Log,
		TEXT("MuJoCo thread pool: %s (%d worker(s) requested)."),
		N > 0 ? TEXT("enabled") : TEXT("disabled"), N);
}

namespace
{
/** The actor a contributor's elements live on: itself, or the one it sits on. */
AActor* ActorOfContributor(UObject* Object)
{
	if (AActor* Actor = Cast<AActor>(Object))
	{
		return Actor;
	}
	if (UActorComponent* Component = Cast<UActorComponent>(Object))
	{
		return Component->GetOwner();
	}
	return nullptr;
}

/** One participant's placement, in MuJoCo's frame, from an Unreal transform. */
void AddParticipant(FSceneAssembly& Scene, const FSpecRef& Spec, const FString& Prefix,
	const FTransform& Placement)
{
	if (!Spec.IsValid())
	{
		return;
	}
	double MjPos[3] = {0.0, 0.0, 0.0};
	double MjQuat[4] = {1.0, 0.0, 0.0, 0.0};
	URLabAxisConv::UePositionToMj(Placement.GetLocation(), MjPos);
	URLabAxisConv::UeQuatToMj(Placement.GetRotation(), MjQuat);
	Scene.Add(Spec, Prefix, FVector(MjPos[0], MjPos[1], MjPos[2]),
		FQuat(MjQuat[1], MjQuat[2], MjQuat[3], MjQuat[0]));
}
}  // namespace

TArray<UObject*> UMjPhysicsEngine::GatherSceneContributors() const
{
	TArray<UObject*> Out;
	UWorld* World = GetWorld();
	if (World == nullptr)
	{
		return Out;
	}
	for (TActorIterator<AActor> It(World); It; ++It)
	{
		AActor* Actor = *It;
		if (Actor == nullptr)
		{
			continue;
		}
		if (Actor->Implements<UMjSceneContributor>())
		{
			Out.Add(Actor);
		}
		for (UActorComponent* Component : Actor->GetComponents())
		{
			if (Component != nullptr && Component->Implements<UMjSceneContributor>())
			{
				Out.Add(Component);
			}
		}
	}
	return Out;
}

void UMjPhysicsEngine::BuildSceneAssembly(FSceneAssembly& Out)
{
	const AAMjManager* Manager = Cast<AAMjManager>(GetOwner());
	if (Manager == nullptr)
	{
		return;
	}
	Out.SetSceneRoot(Manager->GetSceneSpec());

	UWorld* World = GetWorld();
	if (World == nullptr)
	{
		return;
	}
	TArray<AActor*> Actors;
	UGameplayStatics::GetAllActorsOfClass(World, AMjArticulation::StaticClass(), Actors);
	for (AActor* Actor : Actors)
	{
		AMjArticulation* Articulation = Cast<AMjArticulation>(Actor);
		if (Articulation == nullptr)
		{
			continue;
		}
		// The prefix is the articulation's own, and it is what every compiled
		// name of this participant carries, so the binding composes with it.
		AddParticipant(Out, Articulation->GetSpec(), Articulation->GetName() + TEXT("_"),
			Articulation->GetActorTransform());
	}

	// Heightfields and converted actors take the same route as an articulation:
	// author the spec, then attach it under a prefix. The only difference is
	// that the spec is written from Unreal content a moment before it is
	// read, which is why this is the point the sampling and the conversion run.
	for (UObject* Object : GatherSceneContributors())
	{
		IMjSceneContributor* Contributor = Cast<IMjSceneContributor>(Object);
		if (Contributor == nullptr)
		{
			continue;
		}
		Contributor->AuthorSceneSpec();
		AddParticipant(Out, Contributor->GetSceneSpec(), Contributor->GetScenePrefix(),
			Contributor->GetScenePlacement());
	}
}

FMjCompiled UMjPhysicsEngine::CompileSceneSpec()
{
	FSceneAssembly Scene;
	BuildSceneAssembly(Scene);
	return MjCompileScene(Scene);
}

namespace
{
/** Every MuJoCo spec node on `Actor`, whether or not it is in the tree. */
void ForEachSpecNode(AActor& Actor, TFunctionRef<void(UMjNodeComponent&)> Visit)
{
	TArray<UMjNodeComponent*> Nodes;
	Actor.GetComponents(Nodes);
	for (UMjNodeComponent* Node : Nodes)
	{
		if (Node != nullptr)
		{
			Visit(*Node);
		}
	}
}

// --- Recompile state migration ------------------------------------------- //
//
// A structural edit during play recompiles the scene, and a recompile is a new
// mjModel with new addresses: a joint that gained a sibling moves in qpos, so
// reading the old address back would be reading someone else's state. The
// element that owns the state has not changed, though, so the state can follow
// it -- keyed on the node's creation serial, which is minted once per element
// and never reissued.
//
// The rules follow the ProtoSpec bridge's `Recompile` (protospec/lib/compile),
// which is the same migration one level down:
//
//   * a surviving element's state is written back at its NEW address,
//   * a deleted element's state is dropped with it,
//   * a new element starts at the model's own defaults (qpos0 and zeros),
//   * an element whose shape changed -- a hinge become a ball, an actuator that
//     gained an activation -- starts at the defaults too, because there is no
//     meaning to carrying three numbers into a slot that now holds four,
//   * and `time` continues rather than restarting, so a recompile is an edit to
//     a running simulation rather than a new one.

struct FMjJointState
{
	TArray<double> QPos;
	TArray<double> QVel;
};

struct FMjActuatorState
{
	double Ctrl = 0.0;
	TArray<double> Act;
};

struct FMjMocapState
{
	double Pos[3] = {0.0, 0.0, 0.0};
	double Quat[4] = {1.0, 0.0, 0.0, 0.0};
};

/** Everything one model's mjData holds that belongs to a spec element. */
struct FMjMigratedState
{
	TMap<uint64, FMjJointState> Joints;
	TMap<uint64, FMjActuatorState> Actuators;
	TMap<uint64, FMjMocapState> Mocaps;
	double Time = 0.0;
	bool bHasState = false;
};

/** How many qpos entries joint `JointId` owns, from the next joint's address. */
int32 QPosWidth(const mjModel& Model, int32 JointId)
{
	const int32 Start = Model.jnt_qposadr[JointId];
	const int32 End = (JointId + 1 < Model.njnt) ? Model.jnt_qposadr[JointId + 1] : Model.nq;
	return End - Start;
}

/** How many dofs joint `JointId` owns. */
int32 DofWidth(const mjModel& Model, int32 JointId)
{
	const int32 Start = Model.jnt_dofadr[JointId];
	const int32 End = (JointId + 1 < Model.njnt) ? Model.jnt_dofadr[JointId + 1] : Model.nv;
	return End - Start;
}

/** Read the live state out of `Data`, keyed by element rather than by address. */
FMjMigratedState StashState(const mjModel* Model, const mjData* Data, const FMjBinding& Binding)
{
	FMjMigratedState Out;
	if (Model == nullptr || Data == nullptr)
	{
		return Out;
	}
	Out.bHasState = true;
	Out.Time = Data->time;

	for (const FMjBinding::FEntry& Entry : Binding.GetEntries())
	{
		if (Entry.Id < 0 || Entry.Serial == 0)
		{
			continue;
		}
		if (Entry.ObjType == mjOBJ_JOINT)
		{
			if (Entry.Id >= Model->njnt)
			{
				continue;
			}
			FMjJointState State;
			const int32 QAdr = Model->jnt_qposadr[Entry.Id];
			const int32 VAdr = Model->jnt_dofadr[Entry.Id];
			State.QPos.Append(Data->qpos + QAdr, QPosWidth(*Model, Entry.Id));
			State.QVel.Append(Data->qvel + VAdr, DofWidth(*Model, Entry.Id));
			Out.Joints.Add(Entry.Serial, MoveTemp(State));
		}
		else if (Entry.ObjType == mjOBJ_ACTUATOR)
		{
			if (Entry.Id >= Model->nu)
			{
				continue;
			}
			FMjActuatorState State;
			State.Ctrl = Data->ctrl[Entry.Id];
			const int32 ActAdr = Model->actuator_actadr[Entry.Id];
			const int32 ActNum = Model->actuator_actnum[Entry.Id];
			if (ActAdr >= 0 && ActNum > 0)
			{
				State.Act.Append(Data->act + ActAdr, ActNum);
			}
			Out.Actuators.Add(Entry.Serial, MoveTemp(State));
		}
		else if (Entry.ObjType == mjOBJ_BODY)
		{
			if (Entry.Id >= Model->nbody)
			{
				continue;
			}
			const int32 MocapId = Model->body_mocapid[Entry.Id];
			if (MocapId < 0)
			{
				continue;
			}
			FMjMocapState State;
			FMemory::Memcpy(State.Pos, Data->mocap_pos + 3 * MocapId, sizeof(State.Pos));
			FMemory::Memcpy(State.Quat, Data->mocap_quat + 4 * MocapId, sizeof(State.Quat));
			Out.Mocaps.Add(Entry.Serial, State);
		}
	}
	return Out;
}

/**
 * Write `Stash` back at the addresses `Binding` gives in `Model`.
 *
 * `Data` must already hold the new model's defaults: everything this does not
 * write is what a surviving element did not have, or what a new element starts
 * with, and both of those answers are the model's own.
 */
void RestoreState(const mjModel* Model, mjData* Data, const FMjBinding& Binding,
	const FMjMigratedState& Stash)
{
	if (Model == nullptr || Data == nullptr || !Stash.bHasState)
	{
		return;
	}

	for (const FMjBinding::FEntry& Entry : Binding.GetEntries())
	{
		if (Entry.Id < 0 || Entry.Serial == 0)
		{
			continue;
		}
		if (Entry.ObjType == mjOBJ_JOINT)
		{
			const FMjJointState* State = Stash.Joints.Find(Entry.Serial);
			if (State == nullptr || Entry.Id >= Model->njnt)
			{
				continue;
			}
			const int32 QAdr = Model->jnt_qposadr[Entry.Id];
			const int32 VAdr = Model->jnt_dofadr[Entry.Id];
			if (State->QPos.Num() == QPosWidth(*Model, Entry.Id))
			{
				FMemory::Memcpy(Data->qpos + QAdr, State->QPos.GetData(),
					State->QPos.Num() * sizeof(double));
			}
			if (State->QVel.Num() == DofWidth(*Model, Entry.Id))
			{
				FMemory::Memcpy(Data->qvel + VAdr, State->QVel.GetData(),
					State->QVel.Num() * sizeof(double));
			}
		}
		else if (Entry.ObjType == mjOBJ_ACTUATOR)
		{
			const FMjActuatorState* State = Stash.Actuators.Find(Entry.Serial);
			if (State == nullptr || Entry.Id >= Model->nu)
			{
				continue;
			}
			Data->ctrl[Entry.Id] = State->Ctrl;
			const int32 ActAdr = Model->actuator_actadr[Entry.Id];
			const int32 ActNum = Model->actuator_actnum[Entry.Id];
			if (ActAdr >= 0 && State->Act.Num() == ActNum)
			{
				FMemory::Memcpy(Data->act + ActAdr, State->Act.GetData(),
					State->Act.Num() * sizeof(double));
			}
		}
		else if (Entry.ObjType == mjOBJ_BODY)
		{
			const FMjMocapState* State = Stash.Mocaps.Find(Entry.Serial);
			if (State == nullptr || Entry.Id >= Model->nbody)
			{
				continue;
			}
			const int32 MocapId = Model->body_mocapid[Entry.Id];
			if (MocapId < 0)
			{
				continue;
			}
			FMemory::Memcpy(Data->mocap_pos + 3 * MocapId, State->Pos, sizeof(State->Pos));
			FMemory::Memcpy(Data->mocap_quat + 4 * MocapId, State->Quat, sizeof(State->Quat));
		}
	}

	Data->time = Stash.Time;
}
}  // namespace

bool UMjPhysicsEngine::InstallCompiledSpec(FString& OutError)
{
	OutError.Reset();

	UWorld* World = GetWorld();
	if (World == nullptr)
	{
		OutError = TEXT("no world to compile");
		return false;
	}

	TArray<AActor*> Actors;
	UGameplayStatics::GetAllActorsOfClass(World, AMjArticulation::StaticClass(), Actors);

	// Compile before tearing anything down, so a compile that fails costs the
	// caller a diagnostic rather than the model it was running. The assembly is
	// held rather than discarded because the ship-list has to come off the same
	// projection the compiler was given, not a second walk of the level. This
	// also re-authors every contributor's spec, so the contributor list is
	// taken afterwards and describes the level the compiler was shown.
	FSceneAssembly Scene;
	BuildSceneAssembly(Scene);
	const TArray<UObject*> Contributors = GatherSceneContributors();
	FMjCompiled Compiled = MjCompileScene(Scene);
	if (!Compiled.IsOk())
	{
		TArray<FString> Reasons;
		for (const FMjSpecDiagnostic& Diagnostic : Compiled.Errors)
		{
			Reasons.Add(Diagnostic.ToString());
		}
		OutError = Reasons.Num() > 0 ? FString::Join(Reasons, TEXT("; ")) : TEXT("the scene spec did not compile");
		m_LastCompileError = OutError;
		return false;
	}

	// Stop and JOIN the worker before the old model goes away: it can be
	// between its stop check and its CallbackMutex acquire, and the join is
	// what keeps it from resuming against the new one.
	bShouldStopTask = true;
	if (StepRequestEvent != nullptr)
	{
		StepRequestEvent->Trigger();
	}
	if (AsyncPhysicsFuture.IsValid())
	{
		AsyncPhysicsFuture.Wait();
	}

	// Read the running state out while the model that gives it meaning is still
	// here. After the teardown below there is no way back to it: the addresses
	// are the old model's and the old model is freed.
	const FMjMigratedState Stash = StashState(m_model, m_data, InstalledBinding);

	// Every id from the previous compile is now meaningless. Forgetting them
	// before the model goes away means no element can be read against a model
	// it never bound to, even for the length of this function.
	for (AActor* Actor : Actors)
	{
		if (Actor == nullptr)
		{
			continue;
		}
		ForEachSpecNode(*Actor, [](UMjNodeComponent& Node) { Node.Unbind(); });
		if (AMjArticulation* Articulation = Cast<AMjArticulation>(Actor))
		{
			Articulation->ClearControlSlots();
			Articulation->ClearElementIndex();
		}
	}
	if (AAMjManager* Manager = Cast<AAMjManager>(GetOwner()))
	{
		ForEachSpecNode(*Manager, [](UMjNodeComponent& Node) { Node.Unbind(); });
	}
	for (UObject* Object : Contributors)
	{
		if (AActor* Owner = ActorOfContributor(Object))
		{
			ForEachSpecNode(*Owner, [](UMjNodeComponent& Node) { Node.Unbind(); });
		}
	}

	{
		FScopeLock Lock(&CallbackMutex);
		if (m_data != nullptr)
		{
			mj_deleteData(m_data);
			m_data = nullptr;
		}
		if (m_model != nullptr)
		{
			mj_deleteModel(m_model);
			m_model = nullptr;
		}
	}

	// The ids it holds address a model that no longer exists. Dropped here
	// rather than on the way out so that every path from this point on -- the
	// failure returns included -- leaves nothing behind that a later install
	// could mistake for the state of the model it is replacing.
	InstalledBinding = FMjBinding{};

	CompiledXml = MoveTemp(Compiled.Xml);
	ParticipantXml = MoveTemp(Compiled.ParticipantXml);
	ActiveAssetFiles = Scene.CollectAssetFiles();

	m_model = Compiled.Release();
	m_data = mj_makeData(m_model);
	if (m_data == nullptr)
	{
		OutError = TEXT("mj_makeData returned null");
		m_LastCompileError = OutError;
		return false;
	}
	m_LastCompileError.Empty();

	// One pass over the binding does both jobs: it tells each element its id,
	// and it collects the actuator ids per articulation. The ids are the
	// scene's, so an articulation's are neither zero-based nor contiguous, and
	// there is nowhere else they could be recovered from.
	TMap<AMjArticulation*, TArray<int32>> ActuatorIdsByArticulation;
	for (const FMjBinding::FEntry& Entry : Compiled.Binding.GetEntries())
	{
		if (Entry.Node == nullptr || Entry.Id < 0)
		{
			continue;
		}
		UMjNodeComponent* Node = const_cast<UMjNodeComponent*>(Entry.Node);
		Node->BindTo(Entry.Id);

		AMjArticulation* Articulation = Cast<AMjArticulation>(Node->GetOwner());
		if (Articulation == nullptr)
		{
			continue;
		}
		Articulation->IndexBoundElement(*Node, Entry.ObjType, Entry.Id);
		if (Entry.ObjType == mjOBJ_ACTUATOR)
		{
			ActuatorIdsByArticulation.FindOrAdd(Articulation).Add(Entry.Id);
		}
	}

	m_articulations.Empty();
	m_ArticulationMap.Empty();
	for (AActor* Actor : Actors)
	{
		AMjArticulation* Articulation = Cast<AMjArticulation>(Actor);
		if (Articulation == nullptr)
		{
			continue;
		}
		// Sized for every articulation, including ones with no actuators of
		// their own: the slots are indexed by scene id and an articulation
		// that had none still has to answer a read without going out of range.
		Articulation->ResetControlSlots(m_model->nu,
			ActuatorIdsByArticulation.FindRef(Articulation));
		Articulation->BindController(m_model, m_data);
		RegisterArticulation(Articulation);
	}

	// The contributor registries are what the per-frame render pass and the
	// debug visualiser iterate, so they are rebuilt from the same list the
	// compile was given rather than from a second walk of the level.
	m_MujocoComponents.Empty();
	m_heightfieldActors.Empty();
	for (UObject* Object : Contributors)
	{
		if (IMjSceneContributor* Contributor = Cast<IMjSceneContributor>(Object))
		{
			Contributor->OnSceneBound();
		}
		if (UMjQuickConvertComponent* Quick = Cast<UMjQuickConvertComponent>(Object))
		{
			m_MujocoComponents.AddUnique(Quick);
		}
		else if (AMjHeightfieldActor* Heightfield = Cast<AMjHeightfieldActor>(Object))
		{
			m_heightfieldActors.AddUnique(Heightfield);
		}
	}

	ApplyThreadPool();
	ApplyOptions();

	if (bSaveDebugXml)
	{
		SaveDebugArtifacts();
	}

	// One step and back, so the derived quantities a paused scene is inspected
	// through -- contacts, constraints, sensor readings -- are all populated
	// before anybody looks at them. The reset is what leaves mjData holding the
	// new model's own defaults, which is exactly the floor the migration writes
	// on top of, so it has to happen first.
	mj_step(m_model, m_data);
	mj_resetData(m_model, m_data);
	RestoreState(m_model, m_data, Compiled.Binding, Stash);
	mj_forward(m_model, m_data);

	// Held for the next install, which cannot resolve these addresses once this
	// model is gone.
	InstalledBinding = Compiled.Binding;
	return true;
}

void UMjPhysicsEngine::SaveDebugArtifacts() const
{
	if (m_model == nullptr || CompiledXml.IsEmpty())
	{
		return;
	}

	const FString CacheDir = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("URLab"));
	IFileManager::Get().MakeDirectory(*CacheDir, true);

	const FString MjbPath = FPaths::Combine(CacheDir, TEXT("scene_compiled.mjb"));
	mj_saveModel(m_model, TCHAR_TO_UTF8(*MjbPath), nullptr, mj_sizeModel(m_model));

	// The text as the compiler was given it, rather than a re-serialisation of
	// the model: this is the artifact that reproduces the compile.
	FFileHelper::SaveStringToFile(CompiledXml, *FPaths::Combine(CacheDir, TEXT("scene_compiled.xml")));
	for (const TPair<FString, FString>& Spec : ParticipantXml)
	{
		FFileHelper::SaveStringToFile(Spec.Value, *FPaths::Combine(CacheDir, Spec.Key));
	}
}

bool UMjPhysicsEngine::BuildCompiledScene(FMjCompiledScene& Out, FString& OutError) const
{
	Out = FMjCompiledScene();
	OutError.Reset();

	// The ship-list is what the compiler was given and is available whether or
	// not the text is, so it is filled first: a caller that only wants the
	// assets gets them even when the MJCF cannot be produced.
	Out.AssetFiles = ActiveAssetFiles;

	if (CompiledXml.IsEmpty())
	{
		OutError = TEXT("no compiled scene");
		return false;
	}

	// The text the compiler was handed, not a re-serialisation of the model:
	// re-serialising loses the participant structure a client needs to reload,
	// and there is no reason to reconstruct what was written a moment ago.
	Out.Xml = CompiledXml;
	Out.ParticipantXml = ParticipantXml;
	return true;
}

void UMjPhysicsEngine::ApplyOptions()
{
	if (!m_model)
		return;

	// The scene's <option> is the manager's, because the scene is one spec
	// and the manager holds its top-level sections.
	if (const AAMjManager* Manager = Cast<AAMjManager>(GetOwner()))
	{
		MjApplyOptionToModel(Manager->SceneOption, Manager->SceneFlags, m_model);
	}

	UE_LOG(LogURLab, Log, TEXT("Applied manager option overrides (timestep=%.4f from model)"),
		m_model->opt.timestep);
}

void UMjPhysicsEngine::RunMujocoAsync()
{
	if (!m_model || !m_data)
	{
		UE_LOG(LogURLab, Error, TEXT("Skipping RunMuJoCoAsync: m_model or m_data is invalid."));
		return;
	}

	bShouldStopTask = false;

	// Seed the worker's lock-free shadow state from the current config so it
	// never reads UPROPERTYs or the owning actor from the physics thread.
	bPausedAtomic.store(bIsPaused, std::memory_order_release);
	SimSpeedAtomic.store(SimSpeedPercent, std::memory_order_release);

	// Seed the resolved step mode from the RPC dispatcher, which is the runtime
	// owner of the mode: a client hello promotes Live -> Direct/Puppet after
	// startup, so the manager's configured StepMode is only the initial default.
	// This path also runs on recompile (CompileModel restarts the worker); reading
	// the configured StepMode here would revert a mid-session recompile back to
	// Live while the Direct/Puppet handler stays installed, so the worker would
	// pace real-time with the wrong controller pass. SetStepMode re-applies the
	// engine-side effects of the strategy's OnEnter (pacing + unpausing client
	// modes); the installed CustomStepHandler and the publisher-pause flag both
	// survive the recompile, so the full invariant is restored. Fall back to the
	// configured mode only before a dispatcher exists.
	if (AAMjManager* Mgr = Cast<AAMjManager>(GetOwner()))
	{
		FURLabRpcDispatcher* Disp = Mgr->GetStepDispatcher();
		SetStepMode(Disp ? Disp->GetActiveStepMode() : Mgr->StepMode);
		// A recompile rebuilt m_model/m_data under a live session; re-run the
		// active strategy's OnEnter so its step handler is reinstalled onto the
		// fresh engine and the pause / pacing invariants are restored.
		if (Disp)
			Disp->ReapplyActiveStepMode();
	}

	AsyncPhysicsFuture = Async(EAsyncExecution::Thread, [this]() {
		bWorkerRunning.store(true, std::memory_order_release);
		FPlatformProcess::Sleep(0.0f);

		while (true)
		{
			const double LoopStartTime = FPlatformTime::Seconds();

			if (bShouldStopTask)
				break;

			// Runtime-resolved step mode, owned by the RPC dispatcher and
			// seeded from config. Drives the controller pass and the pacer
			// without a per-iteration actor cast.
			const EStepMode Mode = ResolvedStepMode.load(std::memory_order_acquire);

			// Real-time pacer interval. Read from the model under CallbackMutex
			// below (a concurrent CompileModel frees m_model, so reading it
			// outside the lock races the delete); re-read per iteration so
			// set_sim_options retunes the pacer live. Defaulted so pacing stays
			// sane if the model is momentarily absent.
			float TargetInterval = 0.002f;

			{
				FScopeLock Lock(&CallbackMutex);

				if (!m_model || !m_data || bShouldStopTask)
					break;

				TargetInterval = (float)m_model->opt.timestep;

				// Did mjData actually change this iteration? Only then do we
				// publish a render snapshot (which bumps FrameId and drives
				// state-change camera capture). Paused iterations and idle
				// direct/puppet wakes leave this false.
				bool bAdvanced = false;

				// A step handler that self-publishes (direct mode) sets this so
				// the tail below doesn't publish a second time and advance the
				// id past what the step reported.
				bRenderStatePublishedThisStep = false;

				if (bPendingReset)
				{
					mj_resetData(m_model, m_data);
					mj_forward(m_model, m_data);
					bPendingReset = false;
					bAdvanced = true;

					// Zero all actuator control values so stale commands
					// don't persist after reset.
					for (AMjArticulation* Art : m_articulations)
					{
						if (!Art)
							continue;
						for (const int32 Id : Art->GetOwnedActuatorIds())
						{
							Art->ClearStagedControl(Id);
						}
					}

					// Snapshot the registry into weak refs under CallbackMutex.
					// The broadcast runs later on the game thread and must not
					// capture a raw `this` (the engine may be torn down before it
					// runs) nor iterate the worker-owned m_articulations array
					// off the worker thread.
					TArray<TWeakObjectPtr<AMjArticulation>> ResetTargets;
					ResetTargets.Reserve(m_articulations.Num());
					for (AMjArticulation* Art : m_articulations)
					{
						if (Art)
							ResetTargets.Add(Art);
					}
					AsyncTask(ENamedThreads::GameThread, [ResetTargets = MoveTemp(ResetTargets)]() {
						for (const TWeakObjectPtr<AMjArticulation>& Target : ResetTargets)
						{
							if (AMjArticulation* Art = Target.Get())
								Art->OnSimulationReset.Broadcast();
						}
					});
				}

				if (bPendingRestore)
				{
					TArray<double> RestoreState;
					int32 RestoreMask = 0;
					{
						// Swap the pending vector out under CommandMutex so a
						// concurrent RestoreSnapshot can't tear it mid-read.
						FScopeLock CmdLock(&CommandMutex);
						if (bPendingRestore)
						{
							RestoreState = MoveTemp(PendingStateVector);
							RestoreMask = PendingStateMask;
							bPendingRestore = false;
						}
					}
					if (RestoreState.Num() > 0)
					{
						mj_setState(m_model, m_data, RestoreState.GetData(), RestoreMask);
						mj_forward(m_model, m_data);
						bAdvanced = true;
					}
				}

				for (const FPhysicsCallback& Cb : PreStepCallbacks)
				{
					Cb(m_model, m_data);
				}

				// Puppet mode: client pushes qpos/qvel/ctrl directly, so
				// ApplyControls (NetworkValue → d->ctrl) would clobber the
				// snapshot. Skip the controller pass.
				const bool bSkipApplyControls = (Mode == EStepMode::Puppet);
				if (!bSkipApplyControls)
				{
					for (AMjArticulation* Art : m_articulations)
					{
						if (Art)
							Art->ApplyControls(m_model, m_data, /*bSkipController=*/false);
					}
				}

				// A mocap/wrench edit mutates m_data even while paused, so it
				// counts as an advance (publish it).
				bAdvanced |= DrainCommands();

				if (!bPausedAtomic.load(std::memory_order_acquire))
				{
					if (CustomStepHandler)
					{
						// Direct/puppet/replay handler. It owns its own
						// OnPostStep notification and returns true iff it
						// dequeued work and stepped this call.
						bAdvanced |= CustomStepHandler(m_model, m_data);
					}
					else
					{
						mj_step(m_model, m_data);
						// Live/streaming path has no custom handler, so the loop
						// owns the single post-step notification here. Handlers
						// call OnPostStep themselves, so the loop must not — that
						// would double-fire recorders in direct mode.
						if (OnPostStep)
							OnPostStep(m_model, m_data);
						bAdvanced = true;
					}
				}

				// Streaming publishers / debug capture, left unconditional:
				// they broadcast on their own channels and the puppet inline
				// push path (RPC thread) doesn't route through this loop, so
				// gating them on bAdvanced here would change puppet-mode
				// streaming cadence.
				for (const FPhysicsCallback& Cb : PostStepCallbacks)
				{
					Cb(m_model, m_data);
				}

				// Publish a coherent render snapshot for game-thread consumers,
				// inside the same CallbackMutex scope so it reflects the m_data
				// just stepped. Gated on bAdvanced: bumping FrameId on an
				// unchanged frame re-triggers state-change camera capture on
				// identical pixels and inflates FrameId at the idle wake rate.
				// In live mode the game thread consumes at frame rate, so
				// publish only when it asked (bSnapshotWanted) instead of
				// copying the full snapshot every physics step; direct/puppet
				// publish every step because the client associates frames by id.
				if (bAdvanced && !bRenderStatePublishedThisStep)
				{
					const bool bWantPublish = (Mode != EStepMode::Live)
						|| bSnapshotWanted.exchange(false, std::memory_order_acq_rel);
					if (bWantPublish)
					{
						PushRenderState();
					}
				}
			} // FScopeLock released here

			// End-of-iteration pacing.
			//
			// Live mode: UE owns the clock. Pace to TargetInterval so the loop
			//   runs at real-time physics rate. Sleep off the bulk of the wait
			//   (relies on UE's ~1ms process timer resolution) and spin only the
			//   final sub-millisecond for accuracy, rather than spinning the
			//   whole interval and pinning a CPU core.
			// Direct / Puppet: the client owns the clock. Block on
			//   StepRequestEvent (signalled by the dispatcher on enqueue)
			//   so we drain commands at the rate Python sends them rather
			//   than capping at 1 / timestep Hz. Short timeout keeps the
			//   bShouldStopTask check responsive on shutdown.
			// Pace off the resolved mode, not the configured StepMode (which
			// defaults to Auto). Auto resolves to Live, so a freshly-started
			// live session runs real-time instead of blocking at ~10 Hz.
			const bool bUseRealTimePacing = (Mode == EStepMode::Live);
			if (bUseRealTimePacing)
			{
				const float SpeedFactor = FMath::Clamp(SimSpeedAtomic.load(std::memory_order_acquire), 5.0f, 100.0f) / 100.0f;
				const double TargetTime = LoopStartTime + (TargetInterval / SpeedFactor);
				const double Remaining = TargetTime - FPlatformTime::Seconds();
				if (Remaining > 0.0015)
				{
					FPlatformProcess::SleepNoStats((float)(Remaining - 0.0005));
				}
				while (FPlatformTime::Seconds() < TargetTime)
				{
					FPlatformProcess::YieldThread();
				}
			}
			else if (StepRequestEvent)
			{
				// 100ms timeout: cap shutdown latency without polling hot.
				StepRequestEvent->Wait(100);
			}
		}

		bWorkerRunning.store(false, std::memory_order_release);
	});
}

void UMjPhysicsEngine::SetControlSource(EControlSource NewSource)
{
	ControlSource = NewSource;
}

EControlSource UMjPhysicsEngine::GetControlSource() const
{
	return ControlSource;
}

void UMjPhysicsEngine::SetPaused(bool bPaused)
{
	bIsPaused = bPaused;
	bPausedAtomic.store(bPaused, std::memory_order_release);
}

void UMjPhysicsEngine::SetSimSpeed(float Percent)
{
	SimSpeedPercent = Percent;
	SimSpeedAtomic.store(Percent, std::memory_order_release);
}

void UMjPhysicsEngine::SetStepMode(EStepMode Mode)
{
	const EStepMode Resolved = (Mode == EStepMode::Auto) ? EStepMode::Live : Mode;
	ResolvedStepMode.store(Resolved, std::memory_order_release);
	// Client-driven modes need the worker unpaused so the async loop calls the
	// step handler and drains the request queue; the engine otherwise defaults
	// to paused until the editor UI unpauses.
	if (Resolved != EStepMode::Live && bIsPaused)
	{
		SetPaused(false);
	}
}

bool UMjPhysicsEngine::IsRunning() const
{
	return IsInitialized() && !bIsPaused;
}

bool UMjPhysicsEngine::IsInitialized() const
{
	return (m_model != nullptr && m_data != nullptr);
}

FString UMjPhysicsEngine::GetLastCompileError() const
{
	return m_LastCompileError;
}

void UMjPhysicsEngine::StepSync(int32 NumSteps)
{
	if (!IsInitialized())
		return;

	const bool bWasPaused = bIsPaused;
	SetPaused(true);

	FScopeLock Lock(&CallbackMutex);

	for (int32 i = 0; i < NumSteps; ++i)
	{
		DrainCommands();
		mj_step(m_model, m_data);
	}

	// Publish a render snapshot for the sync step path too. Keeps the
	// render flow uniform across async and sync stepping (RPC, replay
	// scrub, custom step handlers).
	PushRenderState();

	SetPaused(bWasPaused);
}

bool UMjPhysicsEngine::CompileModel()
{
	// The install does its own stop-and-join and its own teardown of the model
	// it is replacing, so all that is left here is restarting the worker against
	// whatever it produced.
	m_MujocoComponents.Empty();
	m_articulations.Empty();
	m_heightfieldActors.Empty();

	Compile();

	if (!IsInitialized())
	{
		return false;
	}

	RunMujocoAsync();
	return true;
}

AMjArticulation* UMjPhysicsEngine::GetArticulation(const FString& ActorName) const
{
	if (const AMjArticulation* const* Found = m_ArticulationMap.Find(ActorName))
		return const_cast<AMjArticulation*>(*Found);
	// Resolve by UE object name, the user-supplied ActorId, or the canonical public
	// segment (ArtSegment) so a caller can address an art by its ROS/topic name
	// ("franka") as well as its raw UE name.
	for (AMjArticulation* Art : m_articulations)
	{
		if (!Art)
			continue;
		if (Art->GetName() == ActorName || Art->ActorId == ActorName
			|| FMjCanonicalName::ArtSegment(Art).ToString() == ActorName)
			return Art;
	}
	return nullptr;
}

void UMjPhysicsEngine::RegisterArticulation(AMjArticulation* Articulation)
{
	if (!Articulation)
		return;
	FScopeLock Lock(&CallbackMutex);
	m_articulations.AddUnique(Articulation);
	m_ArticulationMap.Add(Articulation->GetName(), Articulation);
}

const TArray<AMjArticulation*>& UMjPhysicsEngine::GetAllArticulations() const
{
	return m_articulations;
}

TArray<UMjQuickConvertComponent*> UMjPhysicsEngine::GetAllQuickComponents() const
{
	return m_MujocoComponents;
}

TArray<AMjHeightfieldActor*> UMjPhysicsEngine::GetAllHeightfields() const
{
	return m_heightfieldActors;
}

float UMjPhysicsEngine::GetSimTime() const
{
	if (m_data)
		return (float)m_data->time;
	return 0.0f;
}

float UMjPhysicsEngine::GetTimestep() const
{
	return m_model ? (float)m_model->opt.timestep : 0.002f;
}

void UMjPhysicsEngine::ResetSimulation()
{
	bPendingReset = true;
	UE_LOG(LogURLab, Log, TEXT("MuJoCo PhysicsEngine: Reset requested."));
}

void UMjPhysicsEngine::SetCustomStepHandler(FMujocoStepCallback Handler)
{
	FScopeLock Lock(&CallbackMutex);
	CustomStepHandler = Handler;
}

void UMjPhysicsEngine::ClearCustomStepHandler()
{
	FScopeLock Lock(&CallbackMutex);
	CustomStepHandler = nullptr;
}

UMjSimulationState* UMjPhysicsEngine::CaptureSnapshot()
{
	check(IsInGameThread()); // NewObject must run on the game thread
	if (!m_model || !m_data)
		return nullptr;

	UMjSimulationState* NewSnapshot = NewObject<UMjSimulationState>(GetOwner());

	const uint32 Mask = mjSTATE_INTEGRATION;
	const int nState = mj_stateSize(m_model, Mask);
	NewSnapshot->StateVector.SetNum(nState);
	NewSnapshot->StateMask = (int32)Mask;

	{
		// Read the live state under the step lock so the capture can't tear
		// against the physics worker mid-step.
		FScopeLock Lock(&CallbackMutex);
		NewSnapshot->SimTime = (float)m_data->time;
		mj_getState(m_model, m_data, NewSnapshot->StateVector.GetData(), Mask);
	}

	UE_LOG(LogURLab, Log, TEXT("MuJoCo PhysicsEngine: Snapshot captured at t=%f (Size: %d)"), NewSnapshot->SimTime, nState);
	return NewSnapshot;
}

void UMjPhysicsEngine::RestoreSnapshot(UMjSimulationState* Snapshot)
{
	if (!Snapshot)
		return;

	{
		// Match the worker's guarded swap so two restores (or a restore vs the
		// worker's read) can't tear the vector.
		FScopeLock Lock(&CommandMutex);
		PendingStateVector = Snapshot->StateVector;
		PendingStateMask = Snapshot->StateMask;
		bPendingRestore = true;
	}

	UE_LOG(LogURLab, Log, TEXT("MuJoCo PhysicsEngine: Restore requested for snapshot t=%f"), Snapshot->SimTime);
}

void UMjPhysicsEngine::RegisterPreStepCallback(FPhysicsCallback Callback)
{
	// Lock matches the iteration in RunMujocoAsync — TArray realloc during
	// a concurrent Add would invalidate the buffer the physics thread holds.
	FScopeLock Lock(&CallbackMutex);
	PreStepCallbacks.Add(MoveTemp(Callback));
}

void UMjPhysicsEngine::RegisterPostStepCallback(FPhysicsCallback Callback)
{
	FScopeLock Lock(&CallbackMutex);
	PostStepCallbacks.Add(MoveTemp(Callback));
}

void UMjPhysicsEngine::ClearCallbacks()
{
	FScopeLock Lock(&CallbackMutex);
	PreStepCallbacks.Empty();
	PostStepCallbacks.Empty();
}

// =============================================================================
// RenderState pump
//
// Producer (PushRenderState) runs on the stepping thread inside the
// CallbackMutex region right after mj_step + OnPostStep, then briefly
// takes RenderStateMutex to publish a fresh frame. Consumers
// (WithRenderState) take RenderStateMutex for the duration of their
// visitor, so every consumer in one UE frame sees the same coherent
// physics frame.
//
// Lock ordering invariant: CallbackMutex (outer) -> RenderStateMutex
// (inner). The consumer never takes CallbackMutex.
// =============================================================================

namespace
{
template <typename T>
static void ResizeIfDifferent(TArray<T>& Array, int32 RequiredNum)
{
	if (Array.Num() != RequiredNum)
	{
		Array.SetNumUninitialized(RequiredNum);
	}
}

template <typename T>
static void CopyArray(TArray<T>& Dst, const void* Src, int32 Count)
{
	// T is deduced from Dst only. Src is type-erased so MuJoCo's
	// raw `int*` / `mjtNum*` pointers don't have to match T's
	// typedef chain exactly; sizeof(T) drives the byte count.
	if (!Src || Count <= 0)
	{
		Dst.Reset();
		return;
	}
	ResizeIfDifferent(Dst, Count);
	FMemory::Memcpy(Dst.GetData(), Src, sizeof(T) * static_cast<SIZE_T>(Count));
}
} // namespace

void UMjPhysicsEngine::PushRenderState()
{
	if (!m_model || !m_data)
	{
		return;
	}

	FScopeLock Lock(&RenderStateMutex);

	const int32 NBody = m_model->nbody;
	const int32 NGeom = m_model->ngeom;
	const int32 NSite = m_model->nsite;
	const int32 NCam = m_model->ncam;
	const int32 NQ = m_model->nq;
	const int32 NV = m_model->nv;
	const int32 NU = m_model->nu;
	const int32 NSensorData = m_model->nsensordata;
	const int32 NTree = m_model->ntree;
	const int32 NFlexvert = m_model->nflexvert;

	// Body kinematics.
	CopyArray(RenderSnapshot.XPos, m_data->xpos, NBody * 3);
	CopyArray(RenderSnapshot.XQuat, m_data->xquat, NBody * 4);
	CopyArray(RenderSnapshot.CVel, m_data->cvel, NBody * 6);
	CopyArray(RenderSnapshot.XfrcApplied, m_data->xfrc_applied, NBody * 6);

	// Geoms / sites / cameras.
	CopyArray(RenderSnapshot.GeomXPos, m_data->geom_xpos, NGeom * 3);
	CopyArray(RenderSnapshot.GeomXMat, m_data->geom_xmat, NGeom * 9);
	CopyArray(RenderSnapshot.SiteXPos, m_data->site_xpos, NSite * 3);
	CopyArray(RenderSnapshot.SiteXMat, m_data->site_xmat, NSite * 9);
	CopyArray(RenderSnapshot.CamXPos, m_data->cam_xpos, NCam * 3);
	CopyArray(RenderSnapshot.CamXMat, m_data->cam_xmat, NCam * 9);

	// Joint / actuator / sensor state.
	CopyArray(RenderSnapshot.QPos, m_data->qpos, NQ);
	CopyArray(RenderSnapshot.QVel, m_data->qvel, NV);
	CopyArray(RenderSnapshot.QAcc, m_data->qacc, NV);
	CopyArray(RenderSnapshot.ActuatorForce, m_data->actuator_force, NU);
	CopyArray(RenderSnapshot.SensorData, m_data->sensordata, NSensorData);

	// Sleep state.
	CopyArray(RenderSnapshot.BodyAwake, m_data->body_awake, NBody);
	CopyArray(RenderSnapshot.TreeAsleep, m_data->tree_asleep, NTree);
	CopyArray(RenderSnapshot.TreeAwake, m_data->tree_awake, NTree);

	// Flex deformable state.
	CopyArray(RenderSnapshot.FlexvertXPos, m_data->flexvert_xpos, NFlexvert * 3);

	// Metadata.
	RenderSnapshot.SimTime = m_data->time;
	++RenderSnapshot.FrameId;
}

void UMjPhysicsEngine::WithRenderState(
	TFunctionRef<void(const FMjRenderSnapshot&)> Visitor)
{
	FScopeLock Lock(&RenderStateMutex);
	Visitor(RenderSnapshot);
}

uint64 UMjPhysicsEngine::GetRenderFrameId()
{
	FScopeLock Lock(&RenderStateMutex);
	return RenderSnapshot.FrameId;
}

// =============================================================================
// Command channel (UE -> MuJoCo)
//
// Game-thread writers enqueue under CommandMutex; the stepping thread drains
// inside CallbackMutex right before mj_step. Last-write-wins per body per
// drain.
// =============================================================================

void UMjPhysicsEngine::SubmitMocapPose(int32 BodyId, const double Pos[3], const double Quat[4])
{
	if (BodyId < 0)
		return;
	FScopeLock Lock(&CommandMutex);
	FMocapPose& Slot = PendingCommands.MocapPoses.FindOrAdd(BodyId);
	FMemory::Memcpy(Slot.Pos, Pos, sizeof(Slot.Pos));
	FMemory::Memcpy(Slot.Quat, Quat, sizeof(Slot.Quat));
}

void UMjPhysicsEngine::SubmitWrench(int32 BodyId, const double Xfrc[6])
{
	if (BodyId < 0)
		return;
	FScopeLock Lock(&CommandMutex);
	FWrench& Slot = PendingCommands.WrenchSets.FindOrAdd(BodyId);
	FMemory::Memcpy(Slot.Xfrc, Xfrc, sizeof(Slot.Xfrc));
	PendingCommands.WrenchClears.Remove(BodyId);
}

void UMjPhysicsEngine::SubmitClearForce(int32 BodyId)
{
	if (BodyId < 0)
		return;
	FScopeLock Lock(&CommandMutex);
	PendingCommands.WrenchSets.Remove(BodyId);
	PendingCommands.WrenchClears.Add(BodyId);
}

void UMjPhysicsEngine::ApplyWakeBody(int32 BodyId)
{
	if (!m_model || !m_data || BodyId < 0 || BodyId >= m_model->nbody)
		return;
	FScopeLock Lock(&CallbackMutex);
	m_data->body_awake[BodyId] = 1;
	const int32 TreeId = m_model->body_treeid[BodyId];
	if (TreeId >= 0 && TreeId < m_model->ntree)
	{
		m_data->tree_asleep[TreeId] = -1;
		m_data->tree_awake[TreeId] = 1;
	}
}

void UMjPhysicsEngine::ApplySleepBody(int32 BodyId)
{
	if (!m_model || !m_data || BodyId < 0 || BodyId >= m_model->nbody)
		return;
	FScopeLock Lock(&CallbackMutex);
	m_data->body_awake[BodyId] = 0;
	const int32 TreeId = m_model->body_treeid[BodyId];
	if (TreeId >= 0 && TreeId < m_model->ntree)
	{
		if (m_data->tree_asleep[TreeId] < 0)
			m_data->tree_asleep[TreeId] = 0;
		m_data->tree_awake[TreeId] = 0;
	}
}

bool UMjPhysicsEngine::IsBodyAwake(int32 BodyId) const
{
	if (!m_model || !m_data || BodyId < 0 || BodyId >= m_model->nbody)
		return true;
	FScopeLock Lock(&CallbackMutex);
	return m_data->body_awake[BodyId] != 0;
}

void UMjPhysicsEngine::ApplyJointPosition(int32 JointId, double Value)
{
	if (!m_model || !m_data || JointId < 0 || JointId >= m_model->njnt)
		return;
	const int32 QposAdr = m_model->jnt_qposadr[JointId];
	if (QposAdr < 0 || QposAdr >= m_model->nq)
		return;
	FScopeLock Lock(&CallbackMutex);
	m_data->qpos[QposAdr] = Value;
}

void UMjPhysicsEngine::ApplyJointVelocity(int32 JointId, double Value)
{
	if (!m_model || !m_data || JointId < 0 || JointId >= m_model->njnt)
		return;
	const int32 DofAdr = m_model->jnt_dofadr[JointId];
	if (DofAdr < 0 || DofAdr >= m_model->nv)
		return;
	FScopeLock Lock(&CallbackMutex);
	m_data->qvel[DofAdr] = Value;
}

void UMjPhysicsEngine::ApplyGeomFriction(int32 GeomId, double Slide)
{
	if (!m_model || GeomId < 0 || GeomId >= m_model->ngeom)
		return;
	FScopeLock Lock(&CallbackMutex);
	m_model->geom_friction[GeomId * 3] = Slide;
}

void UMjPhysicsEngine::ApplyActuatorGear(int32 ActuatorId, TConstArrayView<double> Gear)
{
	if (!m_model || ActuatorId < 0 || ActuatorId >= m_model->nu)
		return;
	const int32 Num = FMath::Min(Gear.Num(), 6);
	FScopeLock Lock(&CallbackMutex);
	for (int32 i = 0; i < Num; ++i)
		m_model->actuator_gear[ActuatorId * 6 + i] = Gear[i];
}

bool UMjPhysicsEngine::DrainCommands()
{
	if (!m_model || !m_data)
		return false;

	FCommandQueue Local;
	{
		FScopeLock Lock(&CommandMutex);
		if (PendingCommands.IsEmpty())
			return false;
		Local = MoveTemp(PendingCommands);
		PendingCommands = FCommandQueue();
	}

	const int32 NBody = m_model->nbody;

	for (const TPair<int32, FMocapPose>& Pair : Local.MocapPoses)
	{
		const int32 BodyId = Pair.Key;
		if (BodyId < 0 || BodyId >= NBody)
			continue;
		const int32 MocapId = m_model->body_mocapid[BodyId];
		if (MocapId < 0)
			continue;
		FMemory::Memcpy(m_data->mocap_pos + 3 * MocapId, Pair.Value.Pos, sizeof(Pair.Value.Pos));
		FMemory::Memcpy(m_data->mocap_quat + 4 * MocapId, Pair.Value.Quat, sizeof(Pair.Value.Quat));
	}

	for (const TPair<int32, FWrench>& Pair : Local.WrenchSets)
	{
		const int32 BodyId = Pair.Key;
		if (BodyId < 0 || BodyId >= NBody)
			continue;
		FMemory::Memcpy(m_data->xfrc_applied + 6 * BodyId, Pair.Value.Xfrc, sizeof(Pair.Value.Xfrc));
	}
	for (int32 BodyId : Local.WrenchClears)
	{
		if (BodyId < 0 || BodyId >= NBody)
			continue;
		FMemory::Memzero(m_data->xfrc_applied + 6 * BodyId, sizeof(double) * 6);
	}

	return true;
}
