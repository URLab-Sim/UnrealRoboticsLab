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
#include "EngineUtils.h"
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

	AMjbScene* Scene = InWorld.SpawnActor<AMjbScene>();
	if (!Scene)
	{
		UE_LOG(LogURLab, Error, TEXT("[MjbFastPath] failed to spawn AMjbScene"));
		return;
	}
	// SpawnActor already ran the actor's BeginPlay with empty fields, so build +
	// connect explicitly now that the flags are set.
	Scene->bTestSweep = Bus.IsEmpty(); // no owner -> local dev sweep so it moves
	Scene->MjbFilePath = Mjb;
	Scene->BusEndpoint = Bus;
	Scene->LoadAndBuild();
	if (!Bus.IsEmpty())
	{
		Scene->ConnectBus();
	}
	UE_LOG(LogURLab, Log, TEXT("[MjbFastPath] launched: mjb=%s bus=%s"),
		*Mjb, Bus.IsEmpty() ? TEXT("(none, sweep)") : *Bus);

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
