// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "MuJoCo/Fast/MjbFastPathLauncher.h"

#include "MuJoCo/Fast/MjbScene.h"
#include "Utils/URLabLogging.h"

#include "Camera/CameraActor.h"
#include "GameFramework/PlayerController.h"
#include "Engine/World.h"
#include "Engine/DirectionalLight.h"
#include "Engine/SkyLight.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/SkyLightComponent.h"
#include "EngineUtils.h"
#include "Kismet/GameplayStatics.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

void UMjbFastPathLauncher::OnWorldBeginPlay(UWorld& InWorld)
{
	Super::OnWorldBeginPlay(InWorld);
	if (!InWorld.IsGameWorld())
	{
		return;
	}

	FString Mjb;
	if (!FParse::Value(FCommandLine::Get(), TEXT("URLabFastMjb="), Mjb) || Mjb.IsEmpty())
	{
		return; // not a fast-path renderer launch
	}

	// In PIE the editor-world preview (built by LaunchFastPathSync) is duplicated
	// into this world and its BeginPlay already rebuilds + streams. Don't spawn a
	// second scene on top of it. This launcher is the pure -game / packaged path,
	// where no editor preview exists.
	for (TActorIterator<AMjbScene> It(&InWorld); It; ++It)
	{
		UE_LOG(LogURLab, Log,
			TEXT("[MjbFastPath] a fast-path scene already exists in this world; launcher skipping"));
		return;
	}
	FString Bus;
	FParse::Value(FCommandLine::Get(), TEXT("URLabFastBus="), Bus);

	// Direct: step this MJB in-process through the shared engine (a full sim a
	// Python client can drive over RPC), instead of mirroring an owner's bus.
	const bool bDirect = FParse::Param(FCommandLine::Get(), TEXT("URLabFastDirect"));

	// Deferred spawn so the actor's fields are set BEFORE its BeginPlay runs: then
	// BeginPlay owns the whole build (geometry + camera streaming + Direct/bus
	// connect) itself. A plain SpawnActor runs BeginPlay immediately with empty
	// fields, which logs a spurious "no MJB" error and leaves camera streaming
	// unstarted (BeginPlay is the only caller of StartCameraStreaming).
	AMjbScene* Scene = InWorld.SpawnActorDeferred<AMjbScene>(
		AMjbScene::StaticClass(), FTransform::Identity);
	if (!Scene)
	{
		UE_LOG(LogURLab, Error, TEXT("[MjbFastPath] failed to spawn AMjbScene"));
		return;
	}
	Scene->RunMode = bDirect ? EMjbRunMode::Direct : EMjbRunMode::Puppet;
	// Local dev sweep only when there is neither an owner bus nor Direct stepping.
	Scene->bTestSweep = Bus.IsEmpty() && !bDirect;
	Scene->MjbFilePath = Mjb;
	Scene->BusEndpoint = Bus;
	// Render-server cameras are opt-in (capture is not free).
	Scene->bEnableCameraStreaming = FParse::Param(FCommandLine::Get(), TEXT("URLabFastCameras"));
	UGameplayStatics::FinishSpawningActor(Scene, FTransform::Identity);
	UE_LOG(LogURLab, Log, TEXT("[MjbFastPath] launched: mjb=%s mode=%s bus=%s"),
		*Mjb, bDirect ? TEXT("direct") : TEXT("puppet"),
		Bus.IsEmpty() ? TEXT("(none)") : *Bus);

	// The fast-path renderer's usual home is an empty boot map with no lighting, so
	// a showcase would render black however well the geometry built. Give the scene
	// its own light rig -- a key directional sun plus a sky light for ambient fill --
	// so the MJB is visible on any map. Movable so no bake is needed at runtime.
	{
		// Key directional sun (movable, so no bake) -- lights the MJB.
		const FTransform SunXf(FRotator(-46.0, -60.0, 0.0), FVector::ZeroVector);
		if (ADirectionalLight* Sun =
				InWorld.SpawnActor<ADirectionalLight>(ADirectionalLight::StaticClass(), SunXf))
		{
			if (ULightComponent* L = Sun->GetLightComponent())
			{
				L->SetMobility(EComponentMobility::Movable);
			}
		}
		// A second, dimmer fill from the opposite side so shadowed faces are not
		// pure black (an empty map has no sky to bounce ambient off).
		const FTransform FillXf(FRotator(-18.0, 120.0, 0.0), FVector::ZeroVector);
		if (ADirectionalLight* Fill =
				InWorld.SpawnActor<ADirectionalLight>(ADirectionalLight::StaticClass(), FillXf))
		{
			if (ULightComponent* L = Fill->GetLightComponent())
			{
				L->SetMobility(EComponentMobility::Movable);
				L->SetIntensity(0.4f * L->Intensity);
				L->SetLightColor(FLinearColor(0.7f, 0.75f, 0.9f));
				L->SetCastShadows(false);
			}
		}
		// Sky light for gentle ambient fill (captured; harmless if the scene is dark).
		if (ASkyLight* Sky = InWorld.SpawnActor<ASkyLight>(ASkyLight::StaticClass()))
		{
			if (USkyLightComponent* SkyComp = Sky->GetLightComponent())
			{
				SkyComp->SetMobility(EComponentMobility::Movable);
			}
		}
	}

	// Frame the scene with a simple view camera (robot ~1 m tall at the origin).
	if (APlayerController* PC = InWorld.GetFirstPlayerController())
	{
		const FTransform View(FRotator(-18.0, 0.0, 0.0), FVector(-450.0, 0.0, 190.0));
		ACameraActor* Cam = InWorld.SpawnActor<ACameraActor>(ACameraActor::StaticClass(), View);
		if (Cam)
		{
			PC->SetViewTargetWithBlend(Cam);
		}
	}
}
