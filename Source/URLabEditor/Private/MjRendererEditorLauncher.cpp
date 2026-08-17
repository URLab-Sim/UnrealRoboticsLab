// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "MjRendererEditorLauncher.h"

#include "MjLevelOps.h"
#include "URLabEditorLogging.h"

#include "Editor.h"
#include "Engine/World.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

void UMjRendererEditorLauncher::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	// A fast-path renderer launch is requested by any of: an owner control
	// endpoint to connect to, a discovery flag, or a direct MJB file path.
	const TCHAR* Cmd = FCommandLine::Get();
	FString Ignored;
	const bool bRequested =
		(FParse::Value(Cmd, TEXT("URLabFastConnect="), Ignored) && !Ignored.IsEmpty()) ||
		FParse::Param(Cmd, TEXT("URLabFastDiscover")) ||
		(FParse::Value(Cmd, TEXT("URLabFastMjb="), Ignored) && !Ignored.IsEmpty());
	if (!bRequested)
	{
		return; // not a fast-path renderer launch
	}

	// Defer: the editor world does not exist yet at subsystem init, and the
	// initial map is still opening. Poll until both have settled, then launch.
	TickHandle = FTSTicker::GetCoreTicker().AddTicker(
		FTickerDelegate::CreateUObject(this, &UMjRendererEditorLauncher::TryLaunch), 0.25f);
}

void UMjRendererEditorLauncher::Deinitialize()
{
	if (TickHandle.IsValid())
	{
		FTSTicker::GetCoreTicker().RemoveTicker(TickHandle);
		TickHandle.Reset();
	}
	Super::Deinitialize();
}

bool UMjRendererEditorLauncher::TryLaunch(float DeltaTime)
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

	const TCHAR* Cmd = FCommandLine::Get();
	FString Err;
	bool bOk = false;

	FString Control;
	if (FParse::Value(Cmd, TEXT("URLabFastConnect="), Control) && !Control.IsEmpty())
	{
		// Connect to a named Driver: pull its MJB + bus over the control channel.
		bOk = URLabLevelOps::LaunchFastPathFromDriverSync(Control, /*bFreshLevel=*/true, Err);
	}
	else if (FParse::Param(Cmd, TEXT("URLabFastDiscover")))
	{
		// Auto-discover: connect to the first advertised Driver found.
		TArray<URLabLevelOps::FMjDriverInfo> Owners;
		FString DiscErr;
		if (!URLabLevelOps::DiscoverFastPathOwners(Owners, DiscErr) || Owners.Num() == 0)
		{
			Err = Owners.Num() == 0 ? TEXT("no fast-path owners advertised") : DiscErr;
		}
		else
		{
			UE_LOG(LogURLabEditor, Log, TEXT("[MjRenderer] discovered %d owner(s); connecting to '%s' (%s)"),
				Owners.Num(), *Owners[0].Scene, *Owners[0].Control);
			bOk = URLabLevelOps::LaunchFastPathFromDriverSync(Owners[0].Control, /*bFreshLevel=*/true, Err);
		}
	}
	else
	{
		// Direct MJB file path (Driver-less or hand-specified bus).
		FString Mjb, Bus;
		FParse::Value(Cmd, TEXT("URLabFastMjb="), Mjb);
		FParse::Value(Cmd, TEXT("URLabFastBus="), Bus);
		bOk = URLabLevelOps::LaunchFastPathSync(Mjb, Bus, /*bFreshLevel=*/true, Err);
	}

	if (bOk)
	{
		UE_LOG(LogURLabEditor, Log, TEXT("[MjRenderer] editor launch complete"));
	}
	else
	{
		UE_LOG(LogURLabEditor, Error, TEXT("[MjRenderer] editor launch failed: %s"), *Err);
	}

	TickHandle.Reset();
	return false; // one-shot
}
