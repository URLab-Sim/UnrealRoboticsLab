// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "MjbFastPathEditorLauncher.h"

#include "MjLevelOps.h"
#include "URLabEditorLogging.h"

#include "Editor.h"
#include "Engine/World.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

void UMjbFastPathEditorLauncher::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	FString Mjb;
	if (!FParse::Value(FCommandLine::Get(), TEXT("URLabFastMjb="), Mjb) || Mjb.IsEmpty())
	{
		return; // not a fast-path renderer launch
	}

	// Defer: the editor world does not exist yet at subsystem init, and the
	// initial map is still opening. Poll until both have settled, then launch.
	TickHandle = FTSTicker::GetCoreTicker().AddTicker(
		FTickerDelegate::CreateUObject(this, &UMjbFastPathEditorLauncher::TryLaunch), 0.25f);
}

void UMjbFastPathEditorLauncher::Deinitialize()
{
	if (TickHandle.IsValid())
	{
		FTSTicker::GetCoreTicker().RemoveTicker(TickHandle);
		TickHandle.Reset();
	}
	Super::Deinitialize();
}

bool UMjbFastPathEditorLauncher::TryLaunch(float DeltaTime)
{
	Waited += DeltaTime;

	const bool bWorldReady = GEditor && GEditor->GetEditorWorldContext().World() != nullptr;
	if (!bWorldReady)
	{
		return Waited < 30.0f; // keep waiting, give up after 30s
	}
	if (Waited < 0.75f)
	{
		return true; // let the initial map finish opening before we switch levels
	}

	FString Mjb, Bus;
	FParse::Value(FCommandLine::Get(), TEXT("URLabFastMjb="), Mjb);
	FParse::Value(FCommandLine::Get(), TEXT("URLabFastBus="), Bus);

	FString Err;
	if (!URLabLevelOps::LaunchFastPathSync(Mjb, Bus, /*bFreshLevel=*/true, Err))
	{
		UE_LOG(LogURLabEditor, Error, TEXT("[MjbFastPath] editor launch failed: %s"), *Err);
	}
	else
	{
		UE_LOG(LogURLabEditor, Log, TEXT("[MjbFastPath] editor launch complete (mjb=%s bus=%s)"),
			*Mjb, Bus.IsEmpty() ? TEXT("(none)") : *Bus);
	}

	TickHandle.Reset();
	return false; // one-shot
}
