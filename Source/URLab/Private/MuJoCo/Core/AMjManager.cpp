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
#include "MuJoCo/Entity/MjAppearanceStore.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Core/MjRenderSnapshot.h"
#include "MuJoCo/Core/MjDebugVisualizer.h"
#include "MuJoCo/Elements/MjBody.h"
#include "MuJoCo/Elements/MjCamera.h"
#include "MuJoCo/Entity/MjEntityActor.h"
#include "MuJoCo/Entity/MjEntityHandoff.h"
#include "MuJoCo/Entity/MjEntityLogicComponent.h"
#include "MuJoCo/Entity/MjEntityPawn.h"
#include "MuJoCo/Fast/MjbScene.h"
#include "MuJoCo/Entity/MjOverlayRenderer.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Gen/Elements/Options/MjCompiler.gen.h"
#include "MuJoCo/Gen/Elements/Options/MjFlag.gen.h"
#include "MuJoCo/Gen/Elements/MjModel.gen.h"
#include "MuJoCo/Gen/Elements/Options/MjOption.gen.h"
#include "EngineUtils.h"
#include "Transport/NetworkManager.h"
#include "MuJoCo/Input/MjInputHandler.h"
#include "MuJoCo/Input/MjPerturbation.h"
#include "Replay/MjReplayManager.h"
#include "mujoco/mujoco.h"

#include "Kismet/GameplayStatics.h"
#include "GameFramework/PlayerController.h"
#include "Blueprint/UserWidget.h"
#include "Transport/ZmqPublishTransport.h"
#include "Transport/ZmqSubscribeTransport.h"
#include "Transport/ViewerSubscribeTransport.h"
#include "Bridge/MsgpackHelpers.h"
#include "zmq.h"
#include "Bridge/RpcDispatcher.h"
#include "Bridge/BridgeServerConfig.h"
#include "Bridge/BridgeServerConfigUtils.h"
#include "State/MjMsgpackEncoder.h"
#include "State/MjStateTypes.h"
#include "State/MjCanonicalName.h"
#include "State/MjObservationLevel.h"
#include "UserChannels/MjUserChannelComponent.h"
#include "Urdf/UrdfExporter.h"
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

	// The scene is one MuJoCo spec and this is its root. Sections hang off
	// it rather than off the actor directly, because a spec with two roots
	// is not a spec -- and the writer walks exactly this tree.
	SceneSpec = CreateDefaultSubobject<UMjModel>(TEXT("SceneSpec"));
	SceneOption = CreateDefaultSubobject<UMjOption>(TEXT("SceneOption"));
	SceneFlags = CreateDefaultSubobject<UMjFlag>(TEXT("SceneFlags"));
	SceneCompiler = CreateDefaultSubobject<UMjCompiler>(TEXT("SceneCompiler"));
	SceneOption->SetupAttachment(SceneSpec);
	SceneFlags->SetupAttachment(SceneOption);
	SceneCompiler->SetupAttachment(SceneSpec);
	if (RootComponent == nullptr)
	{
		RootComponent = SceneSpec;
	}

	// URLab's two departures from MuJoCo's own defaults, authored rather than
	// hard-coded so they show up in the details panel and in written MJCF.
	// Everything else stays unset, which is what leaves MuJoCo's default in
	// place: a value written here is written into every scene, and MuJoCo is
	// free to change its own mind between versions.
	SceneOption->Integrator = EMjIntegrator::implicitfast;
	SceneCompiler->Conflict = EMjConflict::merge;
}

FSpecRef AAMjManager::GetSceneSpec() const
{
	return FSpecRef::OverActor(const_cast<AAMjManager&>(*this));
}

void AAMjManager::RefreshStateCaches()
{
	BuildEntityCache();
	// Rebuild the state-IR producer cache off the same trigger as the entity
	// cache (initial compile + every recompile). Both run on the game thread.
	StateCollector.Init(this);
	StateCollector.RebuildProducerCacheGameThread();
	// Re-export the URDF(s) so the robot_description matches the fresh model.
	ExportRobotDescriptions();
}

void AAMjManager::ExportRobotDescriptions()
{
	RobotDescriptions.Reset();
	if (!PhysicsEngine || !PhysicsEngine->m_model)
		return;

	const mjModel* Model = PhysicsEngine->m_model;
	const FString RootDir = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("URLab"), TEXT("UrdfExport"));
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
			if (B == nullptr)
				continue;
			const int32 Id = B->GetBoundId().Get(-1);
			if (Id < 0 || Id >= m->nbody)
				continue;

			FMjEntityRecord Rec;
			Rec.MjId = Id;
			Rec.Name = B->MjName.Get(B->GetName());
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

	RefreshStateCaches();
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

	// Viewer role decision: a non-empty StateSourceEndpoint makes this process a
	// read-only VIEWER. A viewer owns no bridge server and binds no owner ports
	// (so it can share a host with the owner); it only subscribes to the owner's
	// viewer bus and renders. Resolve config up front so the whole owner block
	// below can be skipped.
	FURLabBridgeServerConfig ViewerCfg;
	URLabBridgeServerConfigUtils::LoadFromIni(ViewerCfg);
	URLabBridgeServerConfigUtils::ApplyEnvAndCommandLineOverrides(ViewerCfg);
	bIsViewerRole = !ViewerCfg.StateSourceEndpoint.IsEmpty();

	if (!bIsViewerRole)
	{
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

	// Bind external (ROS) transports if their module is present. No-op when
	// URLabRos is not loaded (its factory hooks are unbound), so non-ROS builds
	// and non-ROS users are unaffected. This is what starts in-process ROS
	// publishing/services for a live session.
	BridgeServer->EnsureExternalTransportsBound();

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

	// Owner viewer bus: publish the viewer/geoms topics through the agnostic
	// publish abstraction (used by a direct/live UE owner) so lightweight viewers
	// and fast-path renderers can subscribe over any transport. In puppet the
	// Python client is the owner and broadcasts on its own step, so this stays off.
	if (ViewerCfg.bBroadcastViewers)
	{
		const FString Ep = FString::Printf(TEXT("tcp://%s:%d"),
			*ViewerCfg.BindAddress, ViewerCfg.ViewerPort);
		UURLabZmqPublishTransport* ViewerZmq = NewObject<UURLabZmqPublishTransport>(
			this, TEXT("ViewerBusZmqPublisher"));
		if (ViewerZmq)
		{
			ViewerZmq->ZmqEndpoint = Ep;
			ViewerZmq->SetOwningManager(this);
			if (ViewerZmq->TransportInit())
			{
				ViewerBusTransports.Add(ViewerZmq);
				UE_LOG(LogURLab, Log, TEXT("[AAMjManager] viewer bus publisher bound on %s"), *Ep);
			}
			else
			{
				UE_LOG(LogURLab, Error, TEXT("[AAMjManager] viewer bus publisher bind failed on %s"), *Ep);
			}
		}
	}
	} // end owner-only setup (a viewer skips the bridge + owner transports)

	Compile();

	// Viewer role: no owner physics. Pause the engine so it never self-steps,
	// then subscribe to the owner's viewer bus; the transport's worker thread
	// applies each received {qpos,qvel} and pushes a render snapshot.
	if (bIsViewerRole && PhysicsEngine)
	{
		PhysicsEngine->SetPaused(true);
		ViewerTransport = NewObject<UURLabViewerSubscribeTransport>(this, TEXT("ViewerSub"));
		ViewerTransport->SourceEndpoint = ViewerCfg.StateSourceEndpoint;
		ViewerTransport->Topic = TEXT("viewer");
		ViewerTransport->SetOwningManager(this);
		if (ViewerTransport->TransportInit())
		{
			UE_LOG(LogURLab, Log, TEXT("[AAMjManager] Viewer role: subscribing to %s"),
				*ViewerCfg.StateSourceEndpoint);
		}
		else
		{
			UE_LOG(LogURLab, Error,
				TEXT("[AAMjManager] Viewer transport failed to start (%s); the scene will be static."),
				*ViewerCfg.StateSourceEndpoint);
		}
	}
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

	// The compiled scene renders through one lightweight view at play, not the
	// authoring mesh tree. Built here (BeginPlay, game worlds only) so it is absent
	// in automation worlds, which drive rendering without dispatching BeginPlay.
	BuildRuntimeView();
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

void AAMjManager::RegisterStateProducer(TScriptInterface<IMjStateProducer> Producer)
{
	UObject* Obj = Producer.GetObject();
	if (!Obj)
		return;
	{
		FScopeLock Lock(&StateProducersMutex);
		for (const TWeakObjectPtr<UObject>& P : StateProducers)
		{
			if (P.Get() == Obj)
				return; // already registered
		}
		StateProducers.Add(Obj);
	}
	StateCollector.MarkProducerCacheDirty();
}

void AAMjManager::UnregisterStateProducer(TScriptInterface<IMjStateProducer> Producer)
{
	UObject* Obj = Producer.GetObject();
	if (!Obj)
		return;
	{
		FScopeLock Lock(&StateProducersMutex);
		StateProducers.RemoveAll([Obj](const TWeakObjectPtr<UObject>& P) {
			return P.Get() == Obj;
		});
	}
	StateCollector.MarkProducerCacheDirty();
}

void AAMjManager::GetStateProducers(TArray<TWeakObjectPtr<UObject>>& Out) const
{
	FScopeLock Lock(&StateProducersMutex);
	Out = StateProducers;
}

namespace
{
// The canonical art segment a user-channel component contributes under, or empty
// for scene scope. Mirrors the collector's producer scope resolution.
FString UserChannelScopeSegment(const UMjUserChannelComponent* Comp)
{
	if (!Comp)
		return FString();
	AActor* Owner = Comp->GetOwner();
	if (const AMjArticulation* Art = Cast<AMjArticulation>(Owner))
		return FMjCanonicalName::ArtSegment(Art).ToString();
	return FString();
}
} // namespace

bool AAMjManager::ApplyUserChannelInput(FName ArtOrNone, FName Channel,
	const FMjUserChannel& Value)
{
	const FString TargetScope = ArtOrNone.IsNone() ? FString() : ArtOrNone.ToString();

	TArray<TWeakObjectPtr<UObject>> Producers;
	GetStateProducers(Producers);

	bool bApplied = false;
	for (const TWeakObjectPtr<UObject>& Weak : Producers)
	{
		UMjUserChannelComponent* Comp = Cast<UMjUserChannelComponent>(Weak.Get());
		if (!Comp)
			continue;
		if (UserChannelScopeSegment(Comp) != TargetScope)
			continue;
		EMjUserChannelKind Declared;
		if (!Comp->GetDeclaredInputKind(Channel, Declared))
			continue;
		if (Comp->ApplyInput(Channel, Value))
			bApplied = true;
	}
	return bApplied;
}

void AAMjManager::GetUserInputChannels(TArray<FMjUserInputChannelInfo>& Out) const
{
	TArray<TWeakObjectPtr<UObject>> Producers;
	GetStateProducers(Producers);

	for (const TWeakObjectPtr<UObject>& Weak : Producers)
	{
		UMjUserChannelComponent* Comp = Cast<UMjUserChannelComponent>(Weak.Get());
		if (!Comp)
			continue;
		const FString Scope = UserChannelScopeSegment(Comp);
		TArray<TPair<FName, EMjUserChannelKind>> Declared;
		Comp->GetDeclaredInputChannels(Declared);
		for (const TPair<FName, EMjUserChannelKind>& Pair : Declared)
		{
			FMjUserInputChannelInfo Info;
			Info.ArtSegment = Scope;
			Info.Channel = Pair.Key;
			Info.Kind = Pair.Value;
			Out.Add(MoveTemp(Info));
		}
	}
}

void AAMjManager::RegisterStateConsumer(IMjStateConsumer* Consumer, UObject* OwnerObj)
{
	if (!Consumer || !OwnerObj)
		return;
	FScopeLock Lock(&StateConsumersMutex);
	for (const FRegisteredStateConsumer& R : StateConsumers)
	{
		if (R.Consumer == Consumer)
			return; // already registered
	}
	StateConsumers.Add({OwnerObj, Consumer});
}

void AAMjManager::UnregisterStateConsumer(IMjStateConsumer* Consumer)
{
	if (!Consumer)
		return;
	FScopeLock Lock(&StateConsumersMutex);
	StateConsumers.RemoveAll([Consumer](const FRegisteredStateConsumer& R) {
		return R.Consumer == Consumer;
	});
}

void AAMjManager::PublishOnViewerBus(const FString& Topic, const TArray<uint8>& Payload)
{
	for (const TObjectPtr<UURLabPublishTransport>& Pub : ViewerBusTransports)
	{
		if (Pub)
		{
			Pub->Publish(Topic, Payload);
		}
	}
}

void AAMjManager::PublishViewerFrame(mjModel* m, mjData* d)
{
	if (ViewerBusTransports.Num() == 0 || !m || !d)
		return;
	TSharedPtr<FJsonObject> Obj = MakeShared<FJsonObject>();
	Obj->SetNumberField(TEXT("t"), d->time);
	TArray<TSharedPtr<FJsonValue>> QPos;
	QPos.Reserve(m->nq);
	for (int i = 0; i < m->nq; ++i)
		QPos.Add(MakeShared<FJsonValueNumber>(d->qpos[i]));
	Obj->SetArrayField(TEXT("qpos"), QPos);
	TArray<TSharedPtr<FJsonValue>> QVel;
	QVel.Reserve(m->nv);
	for (int i = 0; i < m->nv; ++i)
		QVel.Add(MakeShared<FJsonValueNumber>(d->qvel[i]));
	Obj->SetArrayField(TEXT("qvel"), QVel);

	TArray<uint8> Buf;
	FURLabMsgpackUtil::PackJsonObject(Obj, Buf);
	if (Buf.Num() == 0)
		return;
	PublishOnViewerBus(TEXT("viewer"), Buf);
}

void AAMjManager::PublishGeomFrame(mjModel* m, mjData* d)
{
	if (ViewerBusTransports.Num() == 0 || !m || !d)
		return;
	const int NGeom = m->ngeom;
	TSharedPtr<FJsonObject> Obj = MakeShared<FJsonObject>();
	Obj->SetNumberField(TEXT("f"), static_cast<double>(GeomBroadcastFrame++));

	TArray<TSharedPtr<FJsonValue>> XPos;
	XPos.Reserve(3 * NGeom);
	for (int i = 0; i < 3 * NGeom; ++i)
		XPos.Add(MakeShared<FJsonValueNumber>(d->geom_xpos[i]));
	Obj->SetArrayField(TEXT("xpos"), XPos);

	// MuJoCo stores geom orientation as a 3x3 (geom_xmat); the wire carries wxyz.
	TArray<TSharedPtr<FJsonValue>> XQuat;
	XQuat.Reserve(4 * NGeom);
	double q[4];
	for (int g = 0; g < NGeom; ++g)
	{
		mju_mat2Quat(q, d->geom_xmat + 9 * g);
		for (int k = 0; k < 4; ++k)
			XQuat.Add(MakeShared<FJsonValueNumber>(q[k]));
	}
	Obj->SetArrayField(TEXT("xquat"), XQuat);

	// Per-camera world transforms, so a render-server renderer's cameras track.
	if (m->ncam > 0)
	{
		TArray<TSharedPtr<FJsonValue>> CxPos;
		CxPos.Reserve(3 * m->ncam);
		for (int i = 0; i < 3 * m->ncam; ++i)
			CxPos.Add(MakeShared<FJsonValueNumber>(d->cam_xpos[i]));
		Obj->SetArrayField(TEXT("cxpos"), CxPos);
		TArray<TSharedPtr<FJsonValue>> CxQuat;
		CxQuat.Reserve(4 * m->ncam);
		for (int c = 0; c < m->ncam; ++c)
		{
			mju_mat2Quat(q, d->cam_xmat + 9 * c);
			for (int k = 0; k < 4; ++k)
				CxQuat.Add(MakeShared<FJsonValueNumber>(q[k]));
		}
		Obj->SetArrayField(TEXT("cxquat"), CxQuat);
	}

	TArray<uint8> Buf;
	FURLabMsgpackUtil::PackJsonObject(Obj, Buf);
	if (Buf.Num() == 0)
		return;
	PublishOnViewerBus(TEXT("geoms"), Buf);
}

void AAMjManager::FanOutStateSnapshot(mjModel* m, mjData* d)
{
	// Owner viewer bus: raw {t,qpos,qvel} to any subscribed viewers, every step,
	// independent of the state_full byte fan-out (which pauses in direct/puppet).
	PublishViewerFrame(m, d);
	// Per-geom world transforms on the same bus, for fast-path renderers.
	PublishGeomFrame(m, d);

	// Build the state IR once per physics step, encode it to the canonical
	// msgpack `state_full` snapshot, and fan the bytes out to every
	// IMjSnapshotPublisher (ZMQ PUB, SHM ring, ...). Registered IMjStateConsumers
	// receive the same typed IR and run their own encoders. Runs inside the
	// engine's CallbackMutex, so the collector's persistent snapshot buffer never
	// races an on-demand Collect from an RPC step reply.
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

	TArray<IMjStateConsumer*> Consumers;
	{
		FScopeLock Lock(&StateConsumersMutex);
		Consumers.Reserve(StateConsumers.Num());
		for (const FRegisteredStateConsumer& R : StateConsumers)
		{
			if (R.Consumer && R.Owner.IsValid())
				Consumers.Add(R.Consumer);
		}
	}

	// bPublishersPaused gates the msgpack byte fan-out only: it is set on
	// Direct / Puppet mode entry so the step reply is the sole delivery to the
	// stepping client (no double-write). A typed consumer is a distinct sink, so
	// it receives the IR every step in all modes regardless of the pause.
	const bool bByteFanOut = Pubs.Num() > 0
						  && !bPublishersPaused.load(std::memory_order_acquire);

	if (!bByteFanOut && Consumers.Num() == 0)
		return;

	FURLabRpcDispatcher* Disp = GetStepDispatcher();
	const int64 StepIdx = Disp ? Disp->GetStepCounter() : 0;
	const FMjStateSnapshot& Snap = StateCollector.Collect(m, d, StepIdx);

	for (IMjStateConsumer* Consumer : Consumers)
		Consumer->ConsumeState(Snap);

	if (!bByteFanOut)
		return;

	TArray<uint8> Buf = FMjMsgpackEncoder::EncodeSnapshotBytes(
		Snap, EObservationLevel::Standard);
	if (Buf.Num() == 0)
		return;
	for (IMjSnapshotPublisher* Pub : Pubs)
		Pub->PublishSnapshot(Buf);
}

void AAMjManager::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	// Retire the compiled render view before the engine frees its borrowed model.
	if (CompiledRenderView)
	{
		CompiledRenderView->Destroy();
		CompiledRenderView = nullptr;
	}
	CompiledViewModel = nullptr;
	if (OverlayRenderer)
	{
		OverlayRenderer->SetModel(nullptr);
	}

	// Stop the viewer input FIRST: its worker thread applies into the engine
	// under CallbackMutex, so it must be joined before the engine (and its
	// model/data) are torn down below.
	if (ViewerTransport)
	{
		ViewerTransport->TransportShutdown();
		ViewerTransport = nullptr;
	}

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

	// Owner viewer bus: the physics worker (its only writer) has stopped above,
	// so the publish transports can be torn down without racing a send.
	for (TObjectPtr<UURLabPublishTransport>& T : ViewerBusTransports)
	{
		if (T)
			T->TransportShutdown();
	}
	ViewerBusTransports.Reset();

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
		// The model belongs to the compiled scene, which also owns the specs it
		// was compiled from and knows the order they have to go in, so the two
		// pointers cannot be freed on their own.
		PhysicsEngine->ReleaseCompiledScene();
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

	PhysicsEngine->WithRenderState([&](const FMjRenderSnapshot& Snap) {
		for (AMjArticulation* Art : Arts)
		{
			if (Art)
			{
				Art->ApplyRenderState(Snap);
			}
		}
		DriveCompiledRenderView(Snap);
		// Record which post-step state the actors now reflect so cameras can
		// tag their readbacks with it (frame_id association for the bridge).
		LastAppliedRenderFrameId.store(Snap.FrameId, std::memory_order_release);
		LastAppliedRenderSimTime.store(Snap.SimTime, std::memory_order_release);
	});
}

void AAMjManager::BuildRuntimeView()
{
	UWorld* World = GetWorld();
	if (GIsAutomationTesting || !World || !World->IsGameWorld() || bIsViewerRole || !PhysicsEngine)
	{
		return;
	}
	mjModel* Model = PhysicsEngine->GetModel();
	if (!Model || PhysicsEngine->IsRawModelInstalled())
	{
		return;
	}
	if (CompiledRenderView && CompiledViewModel == Model)
	{
		return;
	}
	if (CompiledRenderView)
	{
		CompiledRenderView->Destroy();
		CompiledRenderView = nullptr;
	}

	const TArray<AMjArticulation*> Arts = PhysicsEngine->GetAllArticulations();
	const TArray<UMjQuickConvertComponent*> Quicks = PhysicsEngine->GetAllQuickComponents();

	FActorSpawnParameters Params;
	Params.Owner = this;
	Params.ObjectFlags |= RF_Transient;
	AMjbScene* View = World->SpawnActorDeferred<AMjbScene>(
		AMjbScene::StaticClass(), FTransform::Identity, this, nullptr,
		ESpawnActorCollisionHandlingMethod::AlwaysSpawn);
	if (!View)
	{
		return;
	}
	View->MarkExternallyDriven();
	UGameplayStatics::FinishSpawningActor(View, FTransform::Identity);
	View->BuildFromCompiledModel(Model, Arts, Quicks, /*bBuildCameras=*/true);
	CompiledRenderView = View;
	CompiledViewModel = Model;

	// The view now draws each converted prop from the compiled model, so the
	// source actors' own meshes must not double-draw beside it at play.
	for (UMjQuickConvertComponent* Quick : Quicks)
	{
		if (Quick)
		{
			Quick->SetSourceMeshesHiddenInGame(true);
		}
	}

	if (!OverlayRenderer)
	{
		OverlayRenderer = NewObject<UMjOverlayRenderer>(this, TEXT("CompiledOverlayRenderer"));
		OverlayRenderer->SetupAttachment(GetRootComponent());
		OverlayRenderer->RegisterComponent();
	}
	OverlayRenderer->SetModel(Model);

	// The render view now carries every pose, camera, and overlay the articulations used to
	// drive, so the authoring actors are retired. Re-home each art's authored logic and possess
	// config onto its thin runtime entity first, then destroy the arts. UnregisterArticulation
	// drops each from both the registry array and the non-UPROPERTY name map under CallbackMutex,
	// so no consumer resolves a stale actor through GetArticulation.
	for (AMjArticulation* Art : Arts)
	{
		if (!Art)
			continue;
		const FName EntityName(*Art->GetName());
		MjEntityHandoff::TransferPossessConfig(Art, EntityName);
		TArray<UMjEntityLogicComponent*> Logic;
		Art->GetComponents<UMjEntityLogicComponent>(Logic);
		if (Logic.Num() > 0)
		{
			if (AMjEntity* E = GetEntity(EntityName))
				MjEntityHandoff::TransferAuthoredLogic(Art, E);
		}
	}
	for (AMjArticulation* Art : Arts)
	{
		if (!Art)
			continue;
		PhysicsEngine->UnregisterArticulation(Art);
		Art->Destroy();
	}

	// The handoff spawned each entity's pawn, which now hosts the twist controller the collector
	// reads; rebuild the producer cache off the pawns so a possessed entity's twist publishes.
	StateCollector.MarkProducerCacheDirty();
}

void AAMjManager::DriveCompiledRenderView(const FMjRenderSnapshot& Snap)
{
	if (!CompiledRenderView)
	{
		return;
	}
	if (Snap.XPos.Num() > 0 && Snap.XQuat.Num() > 0)
	{
		CompiledRenderView->ApplyBodyTransforms(Snap.XPos.GetData(), Snap.XQuat.GetData());
	}
	// Re-homed body-fixed cameras track the stepped state too. Guard on the snapshot
	// carrying a full cam_xpos/cam_xmat block for every view camera.
	const int32 NCam = CompiledRenderView->NumCameras();
	if (NCam > 0 && Snap.CamXPos.Num() >= NCam * 3 && Snap.CamXMat.Num() >= NCam * 9)
	{
		CompiledRenderView->ApplyCameraPosesFromMat(Snap.CamXPos.GetData(), Snap.CamXMat.GetData());
	}

	// Debug wireframe overlays (collision hulls / joints / sites) draw from the
	// snapshot; intent is the manager's global toggles OR any entity's own flags.
	if (OverlayRenderer && DebugVisualizer)
	{
		bool bCollision = DebugVisualizer->bGlobalDrawDebugCollision;
		bool bJoints = DebugVisualizer->bGlobalDrawDebugJoints;
		bool bSites = false;
		if (PhysicsEngine)
		{
			for (const FMjEntity& E : PhysicsEngine->GetEntityPartition())
			{
				bCollision |= E.Overlay.bDrawDebugCollision;
				bJoints |= E.Overlay.bDrawDebugJoints;
				bSites |= E.Overlay.bDrawDebugSites;
			}
		}
		if (bCollision || bJoints || bSites)
		{
			if (OverlayRenderer->Flags.VisFlags.Num() < mjNVISFLAG)
			{
				OverlayRenderer->Flags.VisFlags.SetNumZeroed(mjNVISFLAG);
			}
			OverlayRenderer->Flags.VisFlags[mjVIS_CONVEXHULL] = bCollision ? 1 : 0;
			OverlayRenderer->Flags.VisFlags[mjVIS_JOINT] = bJoints ? 1 : 0;
			OverlayRenderer->bDrawSites = bSites;
			OverlayRenderer->DrawOverlays(Snap);
		}
	}
}

AAMjManager* AAMjManager::GetManager()
{
	return Instance;
}

UMjPhysicsEngine* AAMjManager::ResolveEngine(const UObject* WorldCtx)
{
	if (AAMjManager* Manager = GetManager())
	{
		if (Manager->PhysicsEngine)
			return Manager->PhysicsEngine;
	}
	if (WorldCtx)
	{
		if (UWorld* World = WorldCtx->GetWorld())
		{
			for (TActorIterator<AAMjManager> It(World); It; ++It)
			{
				if (It->PhysicsEngine)
					return It->PhysicsEngine;
			}
		}
	}
	return nullptr;
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

UMjAppearanceStore* AAMjManager::GetAppearanceStore()
{
	if (AppearanceStore == nullptr)
	{
		AppearanceStore = NewObject<UMjAppearanceStore>(this);
		AppearanceStore->Init(this);
	}
	return AppearanceStore;
}

AMjEntity* AAMjManager::GetEntity(FName EntityName)
{
	if (PhysicsEngine == nullptr)
	{
		return nullptr;
	}

	bool bKnown = false;
	for (const FMjEntity& E : PhysicsEngine->GetEntityPartition())
	{
		if (E.Name == EntityName)
		{
			bKnown = true;
			break;
		}
	}
	if (!bKnown)
	{
		return nullptr;
	}

	UWorld* World = GetWorld();
	if (World == nullptr)
	{
		return nullptr;
	}

	// Reuse a face already spawned for this name; a thin AMjEntity persists in the world once created.
	for (TActorIterator<AMjEntity> It(World); It; ++It)
	{
		if (It->GetEntityName() == EntityName)
		{
			return *It;
		}
	}

	FActorSpawnParameters SpawnParams;
	SpawnParams.ObjectFlags |= RF_Transient;
	AMjEntity* Entity = World->SpawnActor<AMjEntity>(SpawnParams);
	if (Entity != nullptr)
	{
		Entity->SetEntityName(EntityName);
	}
	return Entity;
}

bool AAMjManager::PossessEntity(FName EntityName)
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return false;
	}
	APlayerController* PC = UGameplayStatics::GetPlayerController(World, 0);
	if (!PC)
	{
		return false;
	}

	AMjEntityPawn* Pawn = nullptr;
	for (TActorIterator<AMjEntityPawn> It(World); It; ++It)
	{
		if (It->OwnerEntityName == EntityName)
		{
			Pawn = *It;
			break;
		}
	}
	if (!Pawn)
	{
		return false;
	}

	// Re-home the possess camera onto the render view's root body for this entity, so it follows the
	// stepped physics rather than the fixed placement the pawn spawned at -- the articulation hung its
	// spring-arm off RootBody for the same reason.
	if (CompiledRenderView && PhysicsEngine)
	{
		for (const FMjEntity& E : PhysicsEngine->GetEntityPartition())
		{
			if (E.Name == EntityName)
			{
				if (USceneComponent* Body = CompiledRenderView->GetBodyRootComponent(E.RootBodyId))
				{
					Pawn->SetTrackedComponent(Body);
				}
				break;
			}
		}
	}

	if (PossessedEntityPawn.Get() == Pawn)
	{
		return true;
	}
	if (!PossessedEntityPawn.IsValid())
	{
		PrePossessPawn = PC->GetPawn();
	}
	PC->Possess(Pawn);
	PossessedEntityPawn = Pawn;
	return true;
}

void AAMjManager::UnpossessEntity()
{
	if (!PossessedEntityPawn.IsValid())
	{
		return;
	}
	UWorld* World = GetWorld();
	APlayerController* PC = World ? UGameplayStatics::GetPlayerController(World, 0) : nullptr;
	if (PC)
	{
		PC->UnPossess();
		if (PrePossessPawn.IsValid())
		{
			PC->Possess(PrePossessPawn.Get());
		}
	}
	PossessedEntityPawn = nullptr;
	PrePossessPawn = nullptr;
}

TArray<AMjArticulation*> AAMjManager::GetAllArticulations() const
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

void AAMjManager::CollectCameras(TArray<UMjCamera*>& Out) const
{
	// Global (manager-owned) cameras: attached directly to the manager actor, not to any
	// articulation or the render view. Always included.
	{
		TArray<UMjCamera*> GlobalCameras;
		GetComponents<UMjCamera>(GlobalCameras);
		for (UMjCamera* Cam : GlobalCameras)
		{
			if (Cam)
			{
				Out.Add(Cam);
			}
		}
	}

	// The compiled render view's re-homed body-fixed cameras. When present they are the
	// canonical owners of the model cameras, so the articulation-mounted cameras below are
	// skipped: enumerating both would collide on the shared canonical name and double the
	// GPU capture.
	const int32 NView = CompiledRenderView ? CompiledRenderView->NumCameras() : 0;
	if (NView > 0)
	{
		for (int32 i = 0; i < NView; ++i)
		{
			if (UMjCamera* Cam = CompiledRenderView->GetCamera(i))
			{
				Out.Add(Cam);
			}
		}
		return;
	}

	// Transitional: no view cameras yet, so enumerate the articulation-mounted cameras.
	for (AMjArticulation* Art : GetAllArticulations())
	{
		if (!Art)
		{
			continue;
		}
		TArray<UMjCamera*> Cameras;
		Art->GetComponents<UMjCamera>(Cameras);
		for (UMjCamera* Cam : Cameras)
		{
			if (Cam)
			{
				Out.Add(Cam);
			}
		}
	}
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