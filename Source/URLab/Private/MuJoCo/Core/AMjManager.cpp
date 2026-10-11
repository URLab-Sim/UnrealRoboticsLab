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
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Authoring/MjAuthoringCompiler.h"
#include "MuJoCo/Entity/MjAppearanceStore.h"
#include "MuJoCo/Entity/MjSceneVisuals.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Elements/MjBody.h"
#include "Render/RenderCameraComponent.h"
#include "MuJoCo/Render/MjRendererBoot.h"
#include "MuJoCo/Runtime/MjSimulationWorldSubsystem.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Gen/Elements/Options/MjCompiler.gen.h"
#include "MuJoCo/Gen/Elements/Options/MjFlag.gen.h"
#include "MuJoCo/Gen/Elements/MjModel.gen.h"
#include "MuJoCo/Gen/Elements/Options/MjOption.gen.h"
#include "EngineUtils.h"

#include "Blueprint/UserWidget.h"
#include "Utils/URLabLogging.h"
#include "Bridge/BridgeServer.h"
#include "Bridge/BridgeSubsystem.h"
// Not editor-only: the by-path asset names are read on the cooked runtime path too
// (WidgetBPPath below). Guarding this include compiled in the editor and broke the
// cook, which is the one build that proves the paths cook at all.
#include "MuJoCo/Core/MjContentPaths.h"
#if WITH_EDITOR
#include "Misc/MessageDialog.h"
#endif

AAMjManager* AAMjManager::Instance = nullptr;

AAMjManager::AAMjManager()
{
	PrimaryActorTick.bCanEverTick = false;

	// This actor is source data. The installed simulation is owned by the world
	// subsystem, so a game-world manager never constructs the legacy engine.

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

void AAMjManager::Compile()
{
	FMjAuthoredSimulationPackage Package = FMjAuthoringCompiler::Compile(*this);
	AuthoringCompileError = Package.Error;
	for (const FString& Warning : Package.Warnings)
		UE_LOG(LogURLab, Warning, TEXT("Authored scene: %s"), *Warning);
	for (const FString& Info : Package.Infos)
		UE_LOG(LogURLab, Log, TEXT("Authored scene: %s"), *Info);
	if (!Package.IsValid())
	{
		UE_LOG(LogURLab, Error, TEXT("Authored scene compilation failed: %s"),
			*AuthoringCompileError);
	}
}

void AAMjManager::BeginPlay()
{
	// Compile source objects before component BeginPlay can run any old runtime
	// hook. The resolved boot plan is the sole workflow decision: viewer and
	// render-server worlds do not acquire a simulation because a manager was
	// placed in their map.
	const FMjRendererBootPlan Boot = FMjRendererBoot::FromCommandLine();
	if (!Boot.AllowsAuthoredSimulationSource())
	{
		Super::BeginPlay();
		return;
	}

	UWorld* World = GetWorld();
	UMjSimulationWorldSubsystem* RuntimeOwner =
		World ? World->GetSubsystem<UMjSimulationWorldSubsystem>() : nullptr;
	FString StartError;
	if (!RuntimeOwner || !RuntimeOwner->ActivateRuntime(StartError))
	{
		AuthoringCompileError = RuntimeOwner ? StartError
			: TEXT("authored scene has no simulation world subsystem");
		UE_LOG(LogURLab, Error, TEXT("[AAMjManager] authored handoff unavailable: %s"),
			*AuthoringCompileError);
		Super::BeginPlay();
		return;
	}
	if (!RuntimeOwner->ConfigureLocalClockMode(Boot.ClockMode, StartError))
	{
		AuthoringCompileError = StartError;
		UE_LOG(LogURLab, Error, TEXT("[AAMjManager] authored clock configuration failed: %s"),
			*AuthoringCompileError);
		Super::BeginPlay();
		return;
	}
	if (FMjRendererBoot::WantsBridge(Boot, Boot.Drive, /*bHasBus=*/false))
	{
		if (!RuntimeOwner->AttachBridge(StartError))
		{
			AuthoringCompileError = StartError;
			UE_LOG(LogURLab, Error, TEXT("[AAMjManager] authored bridge attachment failed: %s"),
				*AuthoringCompileError);
			Super::BeginPlay();
			return;
		}
		if (UURLabBridgeServer* Bridge = UURLabBridgeSubsystem::Find(World))
		{
			Bridge->SetGrants(FMjRendererBoot::ResolvePermissionPolicy(
				Boot, EMjPermissionProfile::Simulation));
		}
	}
	if (Boot.HasNativeSimulationSource())
	{
		AuthoringCompileError = TEXT("competing simulation sources: the world contains an authored scene root and -URLabSimulationSource configures a native source");
		UE_LOG(LogURLab, Error, TEXT("[AAMjManager] authored handoff rejected: %s"),
			*AuthoringCompileError);
		Super::BeginPlay();
		return;
	}

	FMjAuthoredSimulationPackage Package = FMjAuthoringCompiler::Compile(*this);
	AuthoringCompileError = Package.Error;
	for (const FString& Warning : Package.Warnings)
		UE_LOG(LogURLab, Warning, TEXT("Authored scene: %s"), *Warning);
	for (const FString& Info : Package.Infos)
		UE_LOG(LogURLab, Log, TEXT("Authored scene: %s"), *Info);
	if (!Package.IsValid())
	{
		UE_LOG(LogURLab, Error, TEXT("[AAMjManager] authored handoff compilation failed: %s"),
			*AuthoringCompileError);
		Super::BeginPlay();
		return;
	}

	const FMjSimulationWorldInstallResult Installed = RuntimeOwner->InstallCandidate(
		MoveTemp(Package.Candidate), Package.Presentation);
	if (!Installed.bCommitted)
	{
		AuthoringCompileError = Installed.Error;
		UE_LOG(LogURLab, Error, TEXT("[AAMjManager] authored handoff rejected: %s"),
			*AuthoringCompileError);
		Super::BeginPlay();
		return;
	}
	AuthoringCompileError = Installed.Error;
	if (!Installed.Error.IsEmpty())
	{
		UE_LOG(LogURLab, Error,
			TEXT("[AAMjManager] authored model committed but boot configuration was incomplete: %s"),
			*Installed.Error);
	}

	// The subsystem has synchronously copied every renderer input it needs.
	// Authoring actors can now retire without affecting simulation or rendering.
	Super::BeginPlay();
	for (TActorIterator<AActor> It(World); It; ++It)
	{
		AActor* Actor = *It;
		if (!Actor || Actor == this) continue;
		TArray<UMjQuickConvertComponent*> QuickComponents;
		Actor->GetComponents(QuickComponents);
		for (UMjQuickConvertComponent* Quick : QuickComponents)
			if (Quick) Quick->SetSourceMeshesHiddenInGame(true);
	}
	TArray<AMjArticulation*> Articulations;
	for (TActorIterator<AMjArticulation> It(World); It; ++It)
		if (*It) Articulations.Add(*It);
	for (AMjArticulation* Articulation : Articulations)
		Articulation->Destroy();
	Destroy();
}

void AAMjManager::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	// Runtime and neutral-scene lifetime belong to UMjSimulationWorldSubsystem.
	// Retiring this authoring actor must not detach or shut either one down.
	Super::EndPlay(EndPlayReason);
	if (Instance == this) Instance = nullptr;
}

AAMjManager* AAMjManager::GetManager()
{
	return Instance;
}

bool AAMjManager::HasCapability(EMjCapability Capability) const
{
	switch (Capability)
	{
		case EMjCapability::CameraFrames:
			return bStreamCameraFrames;
		case EMjCapability::CameraRpc:
			return bAllowCameraRpc;
		case EMjCapability::AcceptInput:
			return bAcceptInput;
		case EMjCapability::AcceptModelLoad:
			return bAcceptModelLoad;
		default:
			return false;
	}
}

FString AAMjManager::GetLastCompileError() const
{
	return AuthoringCompileError;
}

bool AAMjManager::CompileModel()
{
	Compile();
	return AuthoringCompileError.IsEmpty();
}

UMjAppearanceStore* AAMjManager::GetAppearanceStore()
{
	// The store belongs to the WORLD, not to this manager: appearance is a render
	// property and the no-manager render paths drive the very same components, so
	// two stores would fight and only one would be re-driven after a rebuild. Bind
	// ourselves into it so a geom name still resolves to a compiled physics id.
	if (UMjSceneVisualsSubsystem* Visuals = UMjSceneVisualsSubsystem::Get(GetWorld()))
	{
		UMjAppearanceStore* Store = Visuals->Appearance();
		Store->Init(this);
		return Store;
	}
	// No world subsystem (a manager outside a live world, e.g. in a test): keep a
	// private store so callers never get null.
	if (AppearanceStore == nullptr)
	{
		AppearanceStore = NewObject<UMjAppearanceStore>(this);
		AppearanceStore->Init(this);
	}
	return AppearanceStore;
}

void AAMjManager::CollectCameras(TArray<URenderCameraComponent*>& Out) const
{
	// Global (manager-owned) cameras: attached directly to the manager actor, not to any
	// articulation or the render view. Always included.
	{
		TArray<URenderCameraComponent*> GlobalCameras;
		GetComponents<URenderCameraComponent>(GlobalCameras);
		for (URenderCameraComponent* Cam : GlobalCameras)
		{
			if (Cam)
			{
				Out.Add(Cam);
			}
		}
	}

}
