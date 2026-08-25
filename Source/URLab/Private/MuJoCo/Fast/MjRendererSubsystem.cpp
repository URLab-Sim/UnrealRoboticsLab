// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "MuJoCo/Fast/MjRendererSubsystem.h"

#include "MuJoCo/Fast/MjRenderer.h"
#include "MuJoCo/Fast/MjRendererDriverClient.h"
#include "MuJoCo/Fast/DroneViewerPawn.h"
#include "UI/SMjRendererBrowser.h"
#include "UI/SMjRendererHud.h"
#include "Utils/URLabLogging.h"

#include "Engine/World.h"
#include "Engine/GameInstance.h"
#include "Engine/GameViewportClient.h"
#include "GameFramework/PlayerController.h"
#include "Kismet/GameplayStatics.h"
#include "TimerManager.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "AssetRegistry/IAssetRegistry.h"

void UMjRendererSubsystem::RefreshDrivers()
{
	FString Err;
	URLabFastPath::DiscoverDrivers(Drivers, Err);
	if (!Err.IsEmpty())
	{
		UE_LOG(LogURLab, Warning, TEXT("[MjRenderer] driver discovery: %s"), *Err);
	}
}

void UMjRendererSubsystem::RefreshLevels()
{
	Levels.Reset();
	// The bare-plane choice: an empty engine map + the launcher's own light rig.
	FMjRendererLevelChoice Plane;
	Plane.Name = TEXT("Bare Plane");
	Levels.Add(MoveTemp(Plane));

	IAssetRegistry& AR =
		FModuleManager::LoadModuleChecked<FAssetRegistryModule>("AssetRegistry").Get();
	// In editor -game the registry scans /Game lazily, so a fresh boot would list no
	// project maps. Force /Game to be scanned so authored levels appear. (In a cooked
	// build the registry is already fully populated, so this is a cheap no-op.)
	AR.ScanPathsSynchronous({ TEXT("/Game") });
	TArray<FAssetData> Worlds;
	AR.GetAssetsByClass(UWorld::StaticClass()->GetClassPathName(), Worlds);
	for (const FAssetData& A : Worlds)
	{
		const FString Pkg = A.PackageName.ToString();
		if (!Pkg.StartsWith(TEXT("/Game/")))
		{
			continue; // project maps only; skip engine / plugin maps
		}
		FMjRendererLevelChoice C;
		C.Name = A.AssetName.ToString();
		C.Path = Pkg;
		Levels.Add(MoveTemp(C));
	}
}

bool UMjRendererSubsystem::JoinDriver(const FMjDriverInfo& Driver, const FString& LevelPath,
	const FVector& Origin, bool bCameras, bool bVr, bool bInput, FString& OutError)
{
	OutError.Empty();
	TArray<uint8> Mjb;
	FString Bus;
	if (!FMjRendererDriverClient::FetchModel(Driver.Control, Mjb, Bus, OutError))
	{
		UE_LOG(LogURLab, Error, TEXT("[MjRenderer] fetch from driver %s failed: %s"),
			*Driver.Control, *OutError);
		return false;
	}

	// Prefer the bus the Driver reports in the handshake; fall back to its registry
	// advertisement.
	PendingBus = Bus.IsEmpty() ? Driver.Bus : Bus;
	PendingMjb = MoveTemp(Mjb);
	PendingControl = Driver.Control;
	PendingOrigin = Origin;
	bPendingBaseLevel = !LevelPath.IsEmpty();
	bPendingCameras = bCameras;
	bPendingVr = bVr;
	bPendingInput = bInput;
	bJoinPending = true;

	HideBrowser();
	const FString Target = LevelPath.IsEmpty() ? TEXT("/Engine/Maps/Entry") : LevelPath;
	UE_LOG(LogURLab, Log,
		TEXT("[MjRenderer] joining driver %s (%d bytes, bus %s) -> level %s origin (%s) caps(cameras=%d vr=%d input=%d)"),
		*Driver.Control, PendingMjb.Num(), *PendingBus, *Target, *Origin.ToString(),
		bCameras ? 1 : 0, bVr ? 1 : 0, bInput ? 1 : 0);
	UGameplayStatics::OpenLevel(this, FName(*Target));
	return true;
}

bool UMjRendererSubsystem::ConsumePendingJoin(UWorld* World)
{
	if (!bJoinPending || !World)
	{
		return false;
	}
	ActiveRenderer = AMjRenderer::SpawnRenderer(World, PendingMjb, FString(), PendingBus, PendingOrigin,
		/*bDirect=*/false, bPendingBaseLevel, bPendingCameras);

	// Interact cap: point the mirror's perturb channel at the driver's control
	// endpoint so Ctrl-drag forwards fastpath_perturb intents (the mirror has no
	// physics; the owner applies the wrench, gated on its own accept_input
	// capability). SendPerturbation/ProcessMirrorPerturbationInput are guarded on
	// an empty OwnerControlEndpoint, so leaving it unset keeps the join read-only.
	if (bPendingInput && !PendingControl.IsEmpty())
	{
		if (AMjRenderer* Scene = ActiveRenderer.Get())
		{
			Scene->OwnerControlEndpoint = PendingControl;
		}
	}

	// VR cap: swap the static framing camera for the possessed free-fly drone.
	// Same shared entry the -URLabCaps=vr launch path uses (and idempotent, so a
	// join on a vr-flagged boot never spawns a second drone). Possession
	// re-targets the view onto the drone after SpawnRenderer's framing camera.
	if (bPendingVr)
	{
		ADroneViewerPawn::SpawnAndPossess(World);
	}

	bJoinPending = false;
	PendingMjb.Empty();
	PendingControl.Empty();
	UE_LOG(LogURLab, Log, TEXT("[MjRenderer] pending join spawned into %s"), *World->GetMapName());
	ShowHud(); // return-to-browser + live origin tuning
	return true;
}

void UMjRendererSubsystem::BeginAutoJoin(const FString& SceneFilter, const FString& LevelPath,
	const FVector& Origin, bool bCameras, bool bVr, bool bInput)
{
	AutoScene = SceneFilter;
	AutoLevel = LevelPath;
	AutoOrigin = Origin;
	bAutoCameras = bCameras;
	bAutoVr = bVr;
	bAutoInput = bInput;
	AutoJoinTries = 0;
	UGameInstance* GI = GetGameInstance();
	UWorld* W = GI ? GI->GetWorld() : nullptr;
	if (!W)
	{
		return;
	}
	UE_LOG(LogURLab, Log, TEXT("[MjRenderer] auto-join: polling for driver%s%s"),
		SceneFilter.IsEmpty() ? TEXT("") : TEXT(" matching "), *SceneFilter);
	W->GetTimerManager().SetTimer(AutoJoinTimer, this,
		&UMjRendererSubsystem::AutoJoinPoll, 1.0f, /*bLoop=*/true, /*FirstDelay=*/0.5f);
}

void UMjRendererSubsystem::AutoJoinPoll()
{
	RefreshDrivers();
	const FMjDriverInfo* Pick = nullptr;
	for (const FMjDriverInfo& D : Drivers)
	{
		if (AutoScene.IsEmpty() || D.Scene.Contains(AutoScene))
		{
			Pick = &D;
			break;
		}
	}
	UWorld* W = GetGameInstance() ? GetGameInstance()->GetWorld() : nullptr;
	if (Pick)
	{
		FString Err;
		const bool bOk = JoinDriver(*Pick, AutoLevel, AutoOrigin, bAutoCameras, bAutoVr, bAutoInput, Err);
		if (bOk && W)
		{
			W->GetTimerManager().ClearTimer(AutoJoinTimer); // JoinDriver OpenLevels away
		}
		else if (!bOk)
		{
			UE_LOG(LogURLab, Warning, TEXT("[MjRenderer] auto-join: fetch failed, retrying (%s)"), *Err);
		}
	}
	else if (++AutoJoinTries > 30) // ~30s
	{
		UE_LOG(LogURLab, Warning, TEXT("[MjRenderer] auto-join: no driver found, giving up"));
		if (W)
		{
			W->GetTimerManager().ClearTimer(AutoJoinTimer);
		}
	}
}

void UMjRendererSubsystem::ShowBrowser()
{
	if (BrowserWidget.IsValid())
	{
		return;
	}
	UGameInstance* GI = GetGameInstance();
	UGameViewportClient* VP = GI ? GI->GetGameViewportClient() : nullptr;
	if (!VP)
	{
		return;
	}
	// Remember the map the browser lives on so ReturnToBrowser can come back to it.
	if (UWorld* W = GI->GetWorld())
	{
		HomeMap = FName(*W->GetOutermost()->GetName());
	}
	RefreshLevels();
	RefreshDrivers();

	BrowserWidget = SNew(SMjRendererBrowser).Subsystem(this);
	VP->AddViewportWidgetContent(BrowserWidget.ToSharedRef(), /*ZOrder=*/100);
	if (APlayerController* PC = GI->GetFirstLocalPlayerController())
	{
		PC->bShowMouseCursor = true;
		FInputModeUIOnly Mode;
		Mode.SetWidgetToFocus(BrowserWidget);
		Mode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
		PC->SetInputMode(Mode);
	}
	UE_LOG(LogURLab, Log, TEXT("[MjRenderer] server browser shown"));
}

void UMjRendererSubsystem::HideBrowser()
{
	if (!BrowserWidget.IsValid())
	{
		return;
	}
	UGameInstance* GI = GetGameInstance();
	if (UGameViewportClient* VP = GI ? GI->GetGameViewportClient() : nullptr)
	{
		VP->RemoveViewportWidgetContent(BrowserWidget.ToSharedRef());
	}
	BrowserWidget.Reset();
	if (GI)
	{
		if (APlayerController* PC = GI->GetFirstLocalPlayerController())
		{
			PC->bShowMouseCursor = false;
			PC->SetInputMode(FInputModeGameOnly());
		}
	}
}

void UMjRendererSubsystem::ShowHud()
{
	if (HudWidget.IsValid())
	{
		return;
	}
	UGameInstance* GI = GetGameInstance();
	UGameViewportClient* VP = GI ? GI->GetGameViewportClient() : nullptr;
	if (!VP)
	{
		return;
	}
	HudWidget = SNew(SMjRendererHud).Subsystem(this);
	VP->AddViewportWidgetContent(HudWidget.ToSharedRef(), /*ZOrder=*/90);
	// Game+UI so the render is visible AND the HUD buttons take clicks.
	if (APlayerController* PC = GI->GetFirstLocalPlayerController())
	{
		PC->bShowMouseCursor = true;
		FInputModeGameAndUI Mode;
		Mode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
		Mode.SetHideCursorDuringCapture(false);
		PC->SetInputMode(Mode);
	}
}

void UMjRendererSubsystem::HideHud()
{
	if (!HudWidget.IsValid())
	{
		return;
	}
	UGameInstance* GI = GetGameInstance();
	if (UGameViewportClient* VP = GI ? GI->GetGameViewportClient() : nullptr)
	{
		VP->RemoveViewportWidgetContent(HudWidget.ToSharedRef());
	}
	HudWidget.Reset();
}

void UMjRendererSubsystem::ReturnToBrowser()
{
	HideHud();
	ActiveRenderer.Reset();
	bJoinPending = false;
	// OpenLevel back to the browser's home map; its BeginPlay re-shows the browser
	// (no MJB, no pending, -URLabSourceFind=browse still on the command line).
	if (!HomeMap.IsNone())
	{
		UGameplayStatics::OpenLevel(this, HomeMap);
	}
	else
	{
		ShowBrowser(); // fallback: overlay on the current level
	}
}

void UMjRendererSubsystem::NudgeOrigin(const FVector& Delta)
{
	if (AMjRenderer* S = ActiveRenderer.Get())
	{
		S->SceneOrigin += Delta;
	}
}

void UMjRendererSubsystem::SetOrigin(const FVector& NewOrigin)
{
	if (AMjRenderer* S = ActiveRenderer.Get())
	{
		S->SceneOrigin = NewOrigin;
	}
}

FVector UMjRendererSubsystem::GetOrigin() const
{
	const AMjRenderer* S = ActiveRenderer.Get();
	return S ? S->SceneOrigin : FVector::ZeroVector;
}

bool UMjRendererSubsystem::HasActiveRenderer() const
{
	return ActiveRenderer.IsValid();
}
