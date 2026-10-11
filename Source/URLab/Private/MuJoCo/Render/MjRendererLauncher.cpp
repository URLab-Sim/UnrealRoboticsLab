// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "MuJoCo/Render/MjRendererLauncher.h"

#include "MuJoCo/Render/MjRendererBoot.h"
#include "MuJoCo/Render/DroneViewerPawn.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjModelSourcePayload.h"
#include "MuJoCo/Runtime/MjSimulationWorldSubsystem.h"
#include "Bridge/BridgeServer.h"
#include "Bridge/BridgeSubsystem.h"
#include "Utils/URLabLogging.h"
#include "Workflows/RenderServerWorldSubsystem.h"
#include "Workflows/ViewerWorldSubsystem.h"

#include "Engine/World.h"
#include "GameFramework/Pawn.h"
#include "EngineUtils.h"


void UMjRendererLauncher::OnWorldBeginPlay(UWorld& InWorld)
{
	Super::OnWorldBeginPlay(InWorld);
	if (!InWorld.IsGameWorld())
	{
		return;
	}

	// Resolve the workflow before applying any render-world setup. A simulation world
	// starts only its world service and must retain the authored game's pawn state.
	FMjRendererBootPlan Plan = FMjRendererBoot::FromCommandLine();

	// Headless render server: the stock GameModeBase auto-spawns a visible ADefaultPawn
	// (a ~1 m sphere) at the world origin when the level has no PlayerStart, and that
	// sphere shows up in the MuJoCo cameras (the grey "dome" over the scene). Nothing
	// here needs a player, so hide every auto-spawned pawn.
	if (Plan.Action != EMjBootAction::Simulation)
	{
		for (TActorIterator<APawn> PawnIt(&InWorld); PawnIt; ++PawnIt)
		{
			PawnIt->SetActorHiddenInGame(true);
		}
	}

	if (Plan.Action == EMjBootAction::Simulation)
	{
		UMjSimulationWorldSubsystem* Simulation =
			InWorld.GetSubsystem<UMjSimulationWorldSubsystem>();
		FString Error;
		if (!Simulation || !Simulation->ActivateRuntime(Error))
		{
			UE_LOG(LogURLab, Error, TEXT("[URLabSimulation] runtime activation failed: %s"),
				Error.IsEmpty() ? TEXT("simulation world subsystem unavailable") : *Error);
			return;
		}
		if (!Simulation->ConfigureLocalClockMode(Plan.ClockMode, Error))
		{
			UE_LOG(LogURLab, Error, TEXT("[URLabSimulation] local clock configuration failed: %s"),
				*Error);
			return;
		}
		if (FMjRendererBoot::WantsBridge(Plan, Plan.Drive, /*bHasBus=*/false))
		{
			if (!Simulation->AttachBridge(Error))
			{
				UE_LOG(LogURLab, Error, TEXT("[URLabSimulation] bridge attachment failed: %s"),
					*Error);
				return;
			}
			if (UURLabBridgeServer* Bridge = UURLabBridgeSubsystem::Find(&InWorld))
			{
				Bridge->SetGrants(FMjRendererBoot::ResolvePermissionPolicy(
					Plan, EMjPermissionProfile::Simulation));
			}
		}
		if (Plan.SimulationSourceManifest.IsEmpty())
		{
			UE_LOG(LogURLab, Display, TEXT("[URLabSimulation] service started model-free"));
			return;
		}
		for (TActorIterator<AAMjManager> It(&InWorld); It; ++It)
		{
			UE_LOG(LogURLab, Error,
				TEXT("[URLabSimulation] refusing competing native manifest and authored manager scene"));
			return;
		}

		FMjModelSourcePayload Source;
		if (!FMjModelSourceLoader::LoadManifest(
				Plan.SimulationSourceManifest, Source, Error))
		{
			UE_LOG(LogURLab, Error,
				TEXT("[URLabSimulation] source manifest '%s' was refused: %s"),
				*Plan.SimulationSourceManifest, *Error);
			return;
		}
		FMjSimulationPresentation Presentation;
		Presentation.SceneOptions.SceneTransform =
			FTransform(FQuat::Identity, Plan.Scene.Origin);
		Presentation.CameraMaxHeight = Plan.Scene.CamMaxHeight;
		const FMjSimulationWorldInstallResult Installed =
			Simulation->InstallSource(MoveTemp(Source), Presentation);
		if (!Installed.bCommitted)
		{
			UE_LOG(LogURLab, Error,
				TEXT("[URLabSimulation] source manifest '%s' was refused: %s"),
				*Plan.SimulationSourceManifest, *Installed.Error);
			return;
		}
		if (!Installed.Error.IsEmpty())
		{
			UE_LOG(LogURLab, Error,
				TEXT("[URLabSimulation] source installed but boot configuration was incomplete: %s"),
				*Installed.Error);
		}
		UE_LOG(LogURLab, Display,
			TEXT("[URLabSimulation] source installed: generation=%llu epoch=%llu revision=%s"),
			static_cast<unsigned long long>(Installed.Generation),
			static_cast<unsigned long long>(Installed.ClockEpoch), *Installed.SceneRevision);
		return;
	}

	// VR / spectator viewer: fly a free-fly drone camera around the sim rather than
	// framing it with a static camera. Spawned after the default pawn is hidden above;
	// the helper defers + retries internally until the PlayerController exists, and is
	// idempotent, so a later browser join with the same cap never spawns a second drone.
	if (Plan.Caps.Has(EMjCapability::FreeFlyPawn))
	{
		ADroneViewerPawn::SpawnAndPossess(&InWorld);
	}

	switch (Plan.Action)
	{
		case EMjBootAction::Viewer:
		{
			auto* Bridge=UURLabBridgeSubsystem::EnsureFor(&InWorld);
			if(Bridge) Bridge->SetGrants(FMjRendererBoot::ResolvePermissionPolicy(
				Plan, EMjPermissionProfile::Render));
			auto* Workflow=InWorld.GetSubsystem<UURLabViewerWorldSubsystem>();FString Error;
			if(!Workflow||!Workflow->Activate(Error)||!Workflow->AttachBridge(Error))
			{
				if(Workflow)Workflow->ShutdownWorkflow();
				UE_LOG(LogURLab,Error,TEXT("[URLabViewer] workflow activation failed: %s"),*Error);
				return;
			}
			if(Bridge) Bridge->EnsureExternalTransportsBound();
			ViewerWorkflow=Workflow;
			UE_LOG(LogURLab,Display,TEXT("[URLabViewer] typed viewer service started; awaiting explicit owner/source/bind"));
			return;
		}
		case EMjBootAction::Build:
		{
			// A request service is valid before its first scene arrives. Stand up only its
			// world-owned command/data surface; the first explicit render load will install
			// the scene. A placeholder mjModel would make bootstrap look model-free while
			// silently consuming the sole renderer slot with fabricated scene identity.
			if (Plan.Drive != EMjDrive::Request)
			{
				UE_LOG(LogURLab, Error,
					TEXT("[URLabRender] retired hybrid renderer boot was refused"));
				return;
			}
			if (!FMjRendererBoot::WantsBridge(Plan, Plan.Drive, /*bHasBus=*/false))
			{
				UE_LOG(LogURLab, Display,
					TEXT("[URLabRender] model-free request service has no enabled bridge"));
				return;
			}
			UURLabRenderServerWorldSubsystem* Workflow =
				InWorld.GetSubsystem<UURLabRenderServerWorldSubsystem>();
			FString Error;
			if (!Workflow || !Workflow->Activate(Error))
			{
				UE_LOG(LogURLab, Error,
					TEXT("[URLabRender] request workflow activation failed: %s"),
					Error.IsEmpty() ? TEXT("render workflow subsystem unavailable") : *Error);
				return;
			}
			UURLabBridgeServer* Bridge = UURLabBridgeSubsystem::EnsureFor(&InWorld);
			if (!Bridge)
			{
				Workflow->ShutdownWorkflow();
				UE_LOG(LogURLab, Error, TEXT("[URLabRender] request bridge activation failed"));
				return;
			}
			Bridge->SetGrants(FMjRendererBoot::ResolvePermissionPolicy(
				Plan, EMjPermissionProfile::Render));
			if (!Workflow->AttachBridge(Error))
			{
				Workflow->ShutdownWorkflow();
				UE_LOG(LogURLab, Error,
					TEXT("[URLabRender] request service attachment failed: %s"), *Error);
				return;
			}
			Bridge->EnsureExternalTransportsBound();
			RenderWorkflow = Workflow;
			UE_LOG(LogURLab, Display,
				TEXT("[URLabRender] typed request service started; awaiting explicit source install"));
			return;
		}

		case EMjBootAction::Fail:
		{
			// The flags name a shape nothing can build. Say so: this used to be silence in
			// the packaged binary, which looked exactly like a successful headless boot.
			UE_LOG(LogURLab, Error, TEXT("[MjRenderer] launch refused: %s"), *Plan.Error);
			return;
		}

		case EMjBootAction::None:
		case EMjBootAction::Simulation: // handled before any render-workflow state above
		default:
			return;   // a normal map boot
	}
}

void UMjRendererLauncher::Deinitialize()
{
	if(auto* Workflow=ViewerWorkflow.Get()){Workflow->DetachBridge();Workflow->ShutdownWorkflow();}
	ViewerWorkflow.Reset();
	if (UURLabRenderServerWorldSubsystem* Workflow = RenderWorkflow.Get())
	{
		Workflow->DetachBridge();
		Workflow->ShutdownWorkflow();
	}
	RenderWorkflow.Reset();
	Super::Deinitialize();
}
