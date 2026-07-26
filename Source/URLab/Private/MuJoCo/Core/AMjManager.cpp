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

#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Core/MjRenderSnapshot.h"
#include "MuJoCo/Core/MjDebugVisualizer.h"
#include "MuJoCo/Components/Bodies/MjBody.h"
#include "EngineUtils.h"
#include "Transport/NetworkManager.h"
#include "MuJoCo/Input/MjInputHandler.h"
#include "MuJoCo/Input/MjPerturbation.h"
#include "Replay/MjReplayManager.h"
#include "mujoco/mujoco.h"

#include "Kismet/GameplayStatics.h"
#include "Blueprint/UserWidget.h"
#include "Transport/ZmqPublishTransport.h"
#include "Transport/ZmqSubscribeTransport.h"
#include "Transport/ZmqRpcTransport.h"
#include "Bridge/RpcDispatcher.h"
#include "Bridge/BridgeServerConfig.h"
#include "Bridge/BridgeServerConfigUtils.h"
#include "State/MjMsgpackEncoder.h"
#include "State/MjStateTypes.h"
#include "State/MjCanonicalName.h"
#include "Ros/UrdfExporter.h"
#if defined(URLAB_WITH_ROS2) && URLAB_WITH_ROS2
#include "Transport/RosPublishTransport.h"
#endif
#include "Transport/ShmPublishTransport.h"
#include "Transport/ShmRpcTransport.h"
#include "MuJoCo/Core/MjSimulationState.h"
#include "Utils/URLabLogging.h"
#include "Bridge/BridgeServerProvider.h"
#if WITH_EDITOR
#include "Misc/MessageDialog.h"
#endif

AAMjManager* AAMjManager::Instance = nullptr;

AAMjManager::AAMjManager()
{
	PrimaryActorTick.bCanEverTick = true;

	PhysicsEngine = CreateDefaultSubobject<UMjPhysicsEngine>(TEXT("PhysicsEngine"));
	DebugVisualizer = CreateDefaultSubobject<UMjDebugVisualizer>(TEXT("DebugVisualizer"));
	NetworkManager = CreateDefaultSubobject<UMjNetworkManager>(TEXT("NetworkManager"));
	InputHandler = CreateDefaultSubobject<UMjInputHandler>(TEXT("InputHandler"));
	Perturbation = CreateDefaultSubobject<UMjPerturbation>(TEXT("Perturbation"));
}

// --- Forwarding shims: PreCompile, PostCompile, Compile, ApplyOptions ---
// Actual implementations live in UMjPhysicsEngine.

void AAMjManager::PreCompile()
{
	if (PhysicsEngine)
		PhysicsEngine->PreCompile();
}

void AAMjManager::PostCompile()
{
	if (PhysicsEngine)
		PhysicsEngine->PostCompile();
	RefreshStateCaches();
}

void AAMjManager::RefreshStateCaches()
{
	BuildEntityCache();
	// Rebuild the state-IR producer cache off the same trigger as the entity
	// cache (initial compile + every recompile). Both run on the game thread.
	StateCollector.Init(this);
	StateCollector.RebuildProducerCacheGameThread();
	// Re-export the URDF(s) so the ROS robot_description matches the fresh model.
	ExportRobotDescriptions();
}

void AAMjManager::ExportRobotDescriptions()
{
	RobotDescriptions.Reset();
	if (!PhysicsEngine || !PhysicsEngine->m_model)
		return;

	const mjModel* Model = PhysicsEngine->m_model;
	const FString RootDir = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("URLab"), TEXT("RosExport"));
	const FUrdfExportConfig Cfg;

	for (AMjArticulation* Art : GetAllArticulations())
	{
		if (!Art)
			continue;
		const FString RawName = Art->GetName();
		const FName Segment = FMjCanonicalName::ArtSegment(Art);
		const FString SegmentStr = Segment.ToString();
		const FString OutDir = FPaths::Combine(RootDir, SegmentStr);

		const FUrdfModel Urdf = FUrdfExporter::ExportToDir(
			Model, SegmentStr, RawName, OutDir, Cfg);
		if (Urdf.Links.Num() == 0)
			continue;

		RobotDescriptions.Add(Segment, Urdf.Xml);
		for (const FString& W : Urdf.Warnings)
			UE_LOG(LogURLab, Warning, TEXT("[URDF] %s: %s"), *SegmentStr, *W);
		UE_LOG(LogURLab, Log,
			TEXT("[URDF] %s: %d links, %d joints, %d meshes -> %s"),
			*SegmentStr, Urdf.Links.Num(), Urdf.Joints.Num(), Urdf.MeshIds.Num(),
			*FPaths::Combine(OutDir, TEXT("model.urdf")));
	}
}

void AAMjManager::BuildEntityCache()
{
	EntityCache.Reset();
	if (!PhysicsEngine || !PhysicsEngine->m_model)
		return;
	UWorld* World = GetWorld();
	if (!World)
		return;

	mjModel* m = PhysicsEngine->m_model;

	TSet<AMjArticulation*> ArticSet;
	for (AMjArticulation* A : PhysicsEngine->m_articulations)
		if (A)
			ArticSet.Add(A);

	for (TActorIterator<AActor> It(World); It; ++It)
	{
		AActor* Actor = *It;
		if (!Actor)
			continue;
		if (AMjArticulation* AsArt = Cast<AMjArticulation>(Actor))
		{
			if (ArticSet.Contains(AsArt))
				continue;
		}
		TArray<UMjBody*> Bodies;
		Actor->GetComponents<UMjBody>(Bodies);
		for (UMjBody* B : Bodies)
		{
			if (!B || B->bIsDefault)
				continue;
			int32 Id = B->GetMjID();
			if (Id < 0 || Id >= m->nbody)
				continue;

			FMjEntityRecord Rec;
			Rec.MjId = Id;
			Rec.Name = B->GetMjName();
			Rec.BodyComp = B;
			if (m->body_jntnum && m->body_jntadr)
			{
				int FirstJnt = m->body_jntadr[Id];
				int NumJnt = m->body_jntnum[Id];
				Rec.bHasFreeBase = (FirstJnt >= 0 && NumJnt > 0 && FirstJnt < m->njnt && m->jnt_type[FirstJnt] == mjJNT_FREE);
			}
			EntityCache.Add(Rec);
		}
	}
}

void AAMjManager::Compile()
{
	if (!PhysicsEngine)
		return;

	PhysicsEngine->Compile();

	// Sync discovery lists from PhysicsEngine
	m_MujocoComponents = PhysicsEngine->m_MujocoComponents;
	m_articulations = PhysicsEngine->m_articulations;
	m_heightfieldActors = PhysicsEngine->m_heightfieldActors;
	m_ArticulationMap = PhysicsEngine->m_ArticulationMap;
}

void AAMjManager::BeginPlay()
{
	Super::BeginPlay();

	if (Instance != nullptr && Instance != this)
	{
		UE_LOG(LogURLab, Error, TEXT("[AAMjManager] Multiple AAMjManager actors detected in level. Only one is supported — this instance (%s) will be ignored."), *GetName());
		return;
	}
	Instance = this;

	// Auto-create ReplayManager BEFORE the dispatcher / ZMQ components so the
	// dispatcher's Init can cache its pointer. (Transport worker threads later
	// read this cache to avoid TActorIterator, which asserts IsInGameThread.)
	{
		AMjReplayManager* ExistingReplay = Cast<AMjReplayManager>(
			UGameplayStatics::GetActorOfClass(GetWorld(), AMjReplayManager::StaticClass()));
		if (!ExistingReplay)
		{
			FActorSpawnParameters SpawnParams;
			AMjReplayManager* ReplayMgr = GetWorld()->SpawnActor<AMjReplayManager>(SpawnParams);
			if (ReplayMgr)
			{
				UE_LOG(LogURLab, Log, TEXT("[AAMjManager] Auto-created AMjReplayManager"));
			}
		}
	}

	// Resolve a bridge server. In editor builds the URLabEditor module
	// installs a resolver via URLabBridgeProvider that hands back the
	// subsystem's server (lifetime spans PIE sessions). Cooked builds
	// have no resolver, fall through to creating our own and marking it
	// as owned-by-manager so EndPlay tears it down.
	UURLabBridgeServer* ResolvedServer = URLabBridgeProvider::ResolveEditorServer();
	if (ResolvedServer)
	{
		BridgeServer = ResolvedServer;
		// Don't claim ownership — subsystem controls lifetime.
	}
	else
	{
		// Cooked path: no editor subsystem. Manager owns its bridge and
		// brings up both RPC transports inline. Bridge owns all RPC
		// transports; manager only owns publish/subscribe streams.
		FURLabBridgeServerConfig CookedConfig;
		URLabBridgeServerConfigUtils::LoadFromIni(CookedConfig);
		URLabBridgeServerConfigUtils::ApplyEnvAndCommandLineOverrides(CookedConfig);

		BridgeServer = NewObject<UURLabBridgeServer>(this, TEXT("BridgeServer"));
		BridgeServer->SetOwnedByManager(true);
		BridgeServer->SetInstanceConfig(CookedConfig);
		const FString StepEndpoint = FString::Printf(TEXT("tcp://%s:%d"),
			*CookedConfig.BindAddress, CookedConfig.StepPort);
		BridgeServer->Start(StepEndpoint);
		BridgeServer->EnsureShmBound(CookedConfig.InstanceId);
	}
	BridgeServer->RegisterManager(this);

	// State PUB binds the configured address + port so farm instances don't
	// collide; single-editor defaults reproduce tcp://0.0.0.0:5555.
	const FURLabBridgeServerConfig& NetConfig = BridgeServer->GetInstanceConfig();
	const FString StateEndpoint = FString::Printf(TEXT("tcp://%s:%d"),
		*NetConfig.BindAddress, NetConfig.StatePort);

	// Auto-create the streaming transports (PIE-only producers — they
	// tap PhysicsEngine pre/post-step callbacks). Bridge-style UObject
	// lifecycle: NewObject + SetOwningManager + TransportInit. Manager
	// owns streaming; bridge owns RPC.
	{
		UE_LOG(LogURLab, Log, TEXT("[AAMjManager] Creating manager-owned streaming transports"));

		UURLabZmqPublishTransport* Broadcaster = NewObject<UURLabZmqPublishTransport>(
			this, TEXT("AutoZmqBroadcaster"));
		if (Broadcaster)
		{
			Broadcaster->ZmqEndpoint = StateEndpoint;
			Broadcaster->SetOwningManager(this);
			if (Broadcaster->TransportInit())
			{
				ManagerOwnedPublishTransports.Add(Broadcaster);
				UE_LOG(LogURLab, Log,
					TEXT("[AAMjManager] Created UURLabZmqPublishTransport (%s)"), *StateEndpoint);
			}
		}

		UURLabShmPublishTransport* SmPub = NewObject<UURLabShmPublishTransport>(
			this, TEXT("AutoSmSnapshotPublisher"));
		if (SmPub)
		{
			SmPub->SetOwningManager(this);
			if (SmPub->TransportInit())
			{
				ManagerOwnedPublishTransports.Add(SmPub);
				UE_LOG(LogURLab, Log,
					TEXT("[AAMjManager] Created UURLabShmPublishTransport (state.shm)"));
			}
		}

		UURLabZmqSubscribeTransport* Subscriber = NewObject<UURLabZmqSubscribeTransport>(
			this, TEXT("AutoZmqSubscriber"));
		if (Subscriber)
		{
			Subscriber->SetOwningManager(this);
			if (Subscriber->TransportInit())
			{
				ManagerOwnedSubscribeTransports.Add(Subscriber);
				UE_LOG(LogURLab, Log,
					TEXT("[AAMjManager] Created UURLabZmqSubscribeTransport (tcp://127.0.0.1:5556)"));
			}
		}
	}

	// Compile via PhysicsEngine (also discovers ZMQ components in PreCompile)
	Compile();
	// The engine runs its own PostCompile (component PostSetup) inside Compile();
	// the manager's PostCompile shim is not on that path, so build the entity +
	// producer caches the state IR reads here, after the articulation lists sync.
	RefreshStateCaches();
	if (NetworkManager)
		NetworkManager->UpdateCameraStreamingState();

	// Register ONE PreStep + ONE PostStep callback that walks both
	// manager-owned transport arrays.
	if (PhysicsEngine)
	{
		TWeakObjectPtr<AAMjManager> WeakSelf(this);
		PhysicsEngine->RegisterPreStepCallback(
			[WeakSelf](mjModel* m, mjData* d) {
				AAMjManager* Self = WeakSelf.Get();
				if (!Self)
					return;
				for (const TObjectPtr<UURLabSubscribeTransport>& T : Self->ManagerOwnedSubscribeTransports)
				{
					if (T)
						T->PreStep(m, d);
				}
				for (const TObjectPtr<UURLabPublishTransport>& T : Self->ManagerOwnedPublishTransports)
				{
					if (T)
						T->PreStep(m, d);
				}
			});
		PhysicsEngine->RegisterPostStepCallback(
			[WeakSelf](mjModel* m, mjData* d) {
				AAMjManager* Self = WeakSelf.Get();
				if (!Self)
					return;
				for (const TObjectPtr<UURLabPublishTransport>& T : Self->ManagerOwnedPublishTransports)
				{
					if (T)
						T->PostStep(m, d);
				}
				for (const TObjectPtr<UURLabSubscribeTransport>& T : Self->ManagerOwnedSubscribeTransports)
				{
					if (T)
						T->PostStep(m, d);
				}
			});
	}

	if (PhysicsEngine)
	{
		// Register debug data capture as a post-step callback. Fires whenever any
		// debug overlay needs fresh mjData — contact forces (key 1), body shader
		// overlays (Island / Segmentation modes), or tendon/muscle rendering.
		PhysicsEngine->RegisterPostStepCallback([this](mjModel* m, mjData* d) {
			if (!DebugVisualizer)
				return;
			const bool bNeedsCapture =
				DebugVisualizer->bShowDebug || DebugVisualizer->DebugShaderMode != EMjDebugShaderMode::Off || DebugVisualizer->bGlobalDrawTendons;
			if (bNeedsCapture)
			{
				DebugVisualizer->CaptureDebugData();
			}
		});

		// Build the state IR once per physics step, encode it to the canonical
		// msgpack `state_full` snapshot, and fan the bytes out to every
		// IMjSnapshotPublisher (ZMQ PUB, SHM ring, ...). This runs inside the
		// engine's CallbackMutex, so the collector's persistent snapshot buffer
		// never races an on-demand Collect from an RPC step reply.
		TWeakObjectPtr<AAMjManager> WeakSelf(this);
		PhysicsEngine->RegisterPostStepCallback(
			[WeakSelf](mjModel* m, mjData* d) {
				if (AAMjManager* Self = WeakSelf.Get())
					Self->FanOutStateSnapshot(m, d);
			});

		PhysicsEngine->RunMujocoAsync();
	}

	// Auto-create simulate widget AFTER Compile so articulations are registered
	if (bAutoCreateSimulateWidget && !SimulateWidget)
	{
		static const TCHAR* WidgetBPPath = TEXT("/UnrealRoboticsLab/UI/WBP_MjSimulate.WBP_MjSimulate_C");
		UClass* WidgetClass = LoadClass<UUserWidget>(nullptr, WidgetBPPath);
		if (WidgetClass)
		{
			APlayerController* PC = GetWorld()->GetFirstPlayerController();
			if (PC)
			{
				UUserWidget* Widget = CreateWidget<UUserWidget>(PC, WidgetClass);
				if (Widget)
				{
					Widget->AddToViewport(0);
					SimulateWidget = Widget;
					UE_LOG(LogURLab, Log, TEXT("[AAMjManager] Auto-created MjSimulate widget (press Tab to show)"));
				}
			}
		}
		else
		{
			UE_LOG(LogURLab, Warning, TEXT("[AAMjManager] Could not load WBP_MjSimulate Blueprint class. Widget not created."));
		}
	}
}

void AAMjManager::ToggleSimulateWidget()
{
	if (SimulateWidget)
	{
		bool bIsVisible = SimulateWidget->IsVisible();
		SimulateWidget->SetVisibility(bIsVisible ? ESlateVisibility::Collapsed : ESlateVisibility::Visible);
	}
}

void AAMjManager::RegisterSnapshotPublisher(IMjSnapshotPublisher* Publisher,
	UObject* OwnerObj)
{
	if (!Publisher || !OwnerObj)
		return;
	FScopeLock Lock(&SnapshotPublishersMutex);
	for (const FRegisteredSnapshotPublisher& R : SnapshotPublishers)
	{
		if (R.Publisher == Publisher)
			return; // already registered
	}
	SnapshotPublishers.Add({OwnerObj, Publisher});
}

void AAMjManager::UnregisterSnapshotPublisher(IMjSnapshotPublisher* Publisher)
{
	if (!Publisher)
		return;
	FScopeLock Lock(&SnapshotPublishersMutex);
	SnapshotPublishers.RemoveAll([Publisher](const FRegisteredSnapshotPublisher& R) {
		return R.Publisher == Publisher;
	});
}

void AAMjManager::FanOutStateSnapshot(mjModel* m, mjData* d)
{
	// Build the state IR once per physics step, encode it to the canonical
	// msgpack `state_full` snapshot, and fan the bytes out to every
	// IMjSnapshotPublisher (ZMQ PUB, SHM ring, ...). The same IR feeds the ROS
	// publisher, which is its own encoder. Runs inside the engine's
	// CallbackMutex, so the collector's persistent snapshot buffer never races
	// an on-demand Collect from an RPC step reply.
#if defined(URLAB_WITH_ROS2) && URLAB_WITH_ROS2
	UURLabRosPublishTransport* RosPublisher = nullptr;
	for (const TObjectPtr<UURLabPublishTransport>& Transport : ManagerOwnedPublishTransports)
	{
		if (UURLabRosPublishTransport* Ros = Cast<UURLabRosPublishTransport>(Transport))
		{
			RosPublisher = Ros;
			break;
		}
	}
#endif

	TArray<IMjSnapshotPublisher*> Pubs;
	{
		FScopeLock Lock(&SnapshotPublishersMutex);
		Pubs.Reserve(SnapshotPublishers.Num());
		for (const FRegisteredSnapshotPublisher& R : SnapshotPublishers)
		{
			if (R.Publisher && R.Owner.IsValid())
				Pubs.Add(R.Publisher);
		}
	}

	// bPublishersPaused gates the msgpack byte fan-out only: it is set on
	// Direct / Puppet mode entry so the step reply is the sole delivery to the
	// stepping client (no double-write). ROS is a distinct consumer, so it
	// receives the IR every step in all modes regardless of the pause.
	const bool bByteFanOut = Pubs.Num() > 0
		&& !bPublishersPaused.load(std::memory_order_acquire);

	bool bNeedSnapshot = bByteFanOut;
#if defined(URLAB_WITH_ROS2) && URLAB_WITH_ROS2
	bNeedSnapshot = bNeedSnapshot || (RosPublisher != nullptr);
#endif
	if (!bNeedSnapshot)
		return;

	FURLabRpcDispatcher* Disp = GetStepDispatcher();
	const int64 StepIdx = Disp ? Disp->GetStepCounter() : 0;
	const FMjStateSnapshot& Snap = StateCollector.Collect(m, d, StepIdx);

#if defined(URLAB_WITH_ROS2) && URLAB_WITH_ROS2
	if (RosPublisher)
		RosPublisher->PublishState(Snap);
#endif

	if (!bByteFanOut)
		return;

	TArray<uint8> Buf = FMjMsgpackEncoder::EncodeSnapshotBytes(
		Snap, FURLabRpcDispatcher::EObservationLevel::Standard);
	if (Buf.Num() == 0)
		return;
	for (IMjSnapshotPublisher* Pub : Pubs)
		Pub->PublishSnapshot(Buf);
}

void AAMjManager::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	// Stop the physics async thread BEFORE Super::EndPlay so PostStep
	// callbacks don't race into resources child components tear down.
	// Bounded wait: a pathological mj_step can take many seconds; an
	// unbounded Wait() would freeze PIE-stop. On timeout we detach and
	// leak m_model/m_data to avoid use-after-free in the still-running
	// step (one-time per session).
	bool bAsyncExited = true;
	if (PhysicsEngine)
	{
		PhysicsEngine->bShouldStopTask = true;
		// Wake the worker if parked on the step-request event so it
		// observes bShouldStopTask without burning the Wait timeout.
		if (PhysicsEngine->StepRequestEvent)
			PhysicsEngine->StepRequestEvent->Trigger();
		if (PhysicsEngine->AsyncPhysicsFuture.IsValid())
		{
			constexpr double kShutdownTimeoutSec = 3.0;
			bAsyncExited = PhysicsEngine->AsyncPhysicsFuture.WaitFor(
				FTimespan::FromSeconds(kShutdownTimeoutSec));
			if (!bAsyncExited)
			{
				UE_LOG(LogURLab, Warning,
					TEXT("Physics async thread did not exit within %.1fs — detaching. ")
						TEXT("mj_step is likely stuck; MuJoCo resources will leak for this session ")
							TEXT("to avoid a use-after-free in the still-running step."),
					kShutdownTimeoutSec);
			}
		}
		// Clearing callbacks takes CallbackMutex, which a wedged worker holds for
		// its whole iteration. Only safe once the worker has provably exited;
		// on the detach path the callbacks leak with the rest of the accepted
		// leak rather than deadlock PIE-stop.
		if (bAsyncExited)
			PhysicsEngine->ClearCallbacks();
	}

	// Manager-owned transports aren't UActorComponents, so EndPlay
	// doesn't propagate to them; explicit TransportShutdown required.
	for (TObjectPtr<UURLabSubscribeTransport>& T : ManagerOwnedSubscribeTransports)
	{
		if (T)
			T->TransportShutdown();
	}
	ManagerOwnedSubscribeTransports.Reset();
	for (TObjectPtr<UURLabPublishTransport>& T : ManagerOwnedPublishTransports)
	{
		if (T)
			T->TransportShutdown();
	}
	ManagerOwnedPublishTransports.Reset();

	Super::EndPlay(EndPlayReason);
	if (Instance == this)
		Instance = nullptr;

	// Manager-owned servers also tear down the dispatcher itself;
	// subsystem-owned servers stay alive across PIE cycles.
	if (BridgeServer)
	{
		BridgeServer->UnregisterManager(this);
		if (BridgeServer->IsOwnedByManager())
		{
			BridgeServer->Stop();
		}
		BridgeServer = nullptr;
	}

	// Clear tracked actors to prevent dangling pointers on level restart
	m_heightfieldActors.Empty();
	m_articulations.Empty();
	m_MujocoComponents.Empty();

	// Only touch MuJoCo resources if the async thread actually exited — a
	// detached thread may still be executing mj_step and reading these.
	if (PhysicsEngine && bAsyncExited)
	{
		if (PhysicsEngine->m_data)
		{
			mj_deleteData(PhysicsEngine->m_data);
			PhysicsEngine->m_data = nullptr;
		}
		if (PhysicsEngine->m_model)
		{
			mj_deleteModel(PhysicsEngine->m_model);
			PhysicsEngine->m_model = nullptr;
		}
		if (PhysicsEngine->m_spec)
		{
			mj_deleteVFS(&PhysicsEngine->m_vfs);
			mj_deleteSpec(PhysicsEngine->m_spec);
			PhysicsEngine->m_spec = nullptr;
		}

		PhysicsEngine->m_heightfieldActors.Empty();
		PhysicsEngine->m_articulations.Empty();
		PhysicsEngine->m_MujocoComponents.Empty();
	}
}

void AAMjManager::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (!PhysicsEngine || !PhysicsEngine->IsInitialized())
	{
		return;
	}

	ApplyLatestRenderState();
}

void AAMjManager::ApplyLatestRenderState()
{
	if (!PhysicsEngine || !PhysicsEngine->IsInitialized())
	{
		return;
	}

	// Ask the live-mode worker to publish a fresh snapshot; it copies the
	// full state only when a consumer (this tick) has requested one.
	PhysicsEngine->bSnapshotWanted.store(true, std::memory_order_release);

	const TArray<AMjArticulation*>& Arts = PhysicsEngine->GetAllArticulations();
	const TArray<UMjQuickConvertComponent*> Quicks = PhysicsEngine->GetAllQuickComponents();

	PhysicsEngine->WithRenderState([&](const FMjRenderSnapshot& Snap) {
		for (AMjArticulation* Art : Arts)
		{
			if (Art)
			{
				Art->ApplyRenderState(Snap);
			}
		}
		for (UMjQuickConvertComponent* Quick : Quicks)
		{
			if (Quick)
			{
				Quick->ApplyRenderState(Snap);
			}
		}
		// Record which post-step state the actors now reflect so cameras can
		// tag their readbacks with it (frame_id association for the bridge).
		LastAppliedRenderFrameId.store(Snap.FrameId, std::memory_order_release);
		LastAppliedRenderSimTime.store(Snap.SimTime, std::memory_order_release);
	});
}

AAMjManager* AAMjManager::GetManager()
{
	return Instance;
}

void AAMjManager::SetPaused(bool bPaused)
{
	if (PhysicsEngine)
		PhysicsEngine->SetPaused(bPaused);
}

bool AAMjManager::IsRunning() const
{
	return PhysicsEngine ? PhysicsEngine->IsRunning() : false;
}

bool AAMjManager::IsInitialized() const
{
	return PhysicsEngine ? PhysicsEngine->IsInitialized() : false;
}

FString AAMjManager::GetLastCompileError() const
{
	return PhysicsEngine ? PhysicsEngine->GetLastCompileError() : FString();
}

void AAMjManager::StepSync(int32 NumSteps)
{
	if (PhysicsEngine)
		PhysicsEngine->StepSync(NumSteps);
}

bool AAMjManager::CompileModel()
{
	if (!PhysicsEngine)
		return false;

	bool Result = PhysicsEngine->CompileModel();

	// Re-sync discovery lists after recompile
	m_MujocoComponents = PhysicsEngine->m_MujocoComponents;
	m_articulations = PhysicsEngine->m_articulations;
	m_heightfieldActors = PhysicsEngine->m_heightfieldActors;
	m_ArticulationMap = PhysicsEngine->m_ArticulationMap;

	// Component ids/views were re-bound; rebuild the state-IR caches so the
	// collector's weak ptrs and entity table match the fresh model.
	RefreshStateCaches();

	return Result;
}

AMjArticulation* AAMjManager::GetArticulation(const FString& ActorName) const
{
	return PhysicsEngine ? PhysicsEngine->GetArticulation(ActorName) : nullptr;
}

const TArray<AMjArticulation*>& AAMjManager::GetAllArticulations() const
{
	return PhysicsEngine ? PhysicsEngine->GetAllArticulations() : m_articulations;
}

TArray<UMjQuickConvertComponent*> AAMjManager::GetAllQuickComponents() const
{
	return PhysicsEngine ? PhysicsEngine->GetAllQuickComponents() : m_MujocoComponents;
}

TArray<AMjHeightfieldActor*> AAMjManager::GetAllHeightfields() const
{
	return PhysicsEngine ? PhysicsEngine->GetAllHeightfields() : m_heightfieldActors;
}

float AAMjManager::GetSimTime() const
{
	return PhysicsEngine ? PhysicsEngine->GetSimTime() : 0.0f;
}

float AAMjManager::GetTimestep() const
{
	return PhysicsEngine ? PhysicsEngine->GetTimestep() : 0.002f;
}

// --- Recording / replay ---

void AAMjManager::StartRecording()
{
	AMjReplayManager* ReplayMgr = Cast<AMjReplayManager>(UGameplayStatics::GetActorOfClass(GetWorld(), AMjReplayManager::StaticClass()));
	if (ReplayMgr)
	{
		ReplayMgr->StartRecording();
		UE_LOG(LogURLab, Log, TEXT("StartRecording called on ReplayManager."));
	}
	else
	{
		UE_LOG(LogURLab, Warning, TEXT("ReplayManager not found in scene!"));
	}
}

void AAMjManager::StopRecording()
{
	AMjReplayManager* ReplayMgr = Cast<AMjReplayManager>(UGameplayStatics::GetActorOfClass(GetWorld(), AMjReplayManager::StaticClass()));
	if (ReplayMgr)
	{
		ReplayMgr->StopRecording();
		UE_LOG(LogURLab, Log, TEXT("StopRecording called on ReplayManager."));
	}
	// OnPostStep callback stays registered; the replay manager gates
	// recording with bIsRecording so it can be restarted without re-registering.
}

void AAMjManager::StartReplay()
{
	AMjReplayManager* ReplayMgr = Cast<AMjReplayManager>(UGameplayStatics::GetActorOfClass(GetWorld(), AMjReplayManager::StaticClass()));
	if (ReplayMgr)
	{
		ReplayMgr->StartReplay();
		UE_LOG(LogURLab, Log, TEXT("StartReplay called on ReplayManager."));
	}
}

void AAMjManager::StopReplay()
{
	AMjReplayManager* ReplayMgr = Cast<AMjReplayManager>(UGameplayStatics::GetActorOfClass(GetWorld(), AMjReplayManager::StaticClass()));
	if (ReplayMgr)
	{
		ReplayMgr->StopReplay();
		UE_LOG(LogURLab, Log, TEXT("StopReplay called on ReplayManager."));
	}
}

void AAMjManager::ResetSimulation()
{
	if (PhysicsEngine)
	{
		PhysicsEngine->ResetSimulation();
	}
	UE_LOG(LogURLab, Log, TEXT("MuJoCo Manager: Reset requested."));
}

UMjSimulationState* AAMjManager::CaptureSnapshot()
{
	return PhysicsEngine ? PhysicsEngine->CaptureSnapshot() : nullptr;
}

void AAMjManager::RestoreSnapshot(UMjSimulationState* Snapshot)
{
	if (PhysicsEngine)
		PhysicsEngine->RestoreSnapshot(Snapshot);
}