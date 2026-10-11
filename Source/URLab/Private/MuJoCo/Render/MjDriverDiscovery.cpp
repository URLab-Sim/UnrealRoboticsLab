// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "MuJoCo/Render/MjDriverDiscovery.h"

#include "Bridge/InstanceRegistry.h"

#include "HAL/FileManager.h"
#include "HAL/PlatformProcess.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/DateTime.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Dom/JsonObject.h"

namespace URLabRender
{
const TCHAR* const OwnerRole = TEXT("render_owner");

bool IsOwnerEntry(const TSharedPtr<FJsonObject>& Obj)
{
	// Only render drivers (role or grant). Skips ordinary bridge instances,
	// which now write role "bridge" into the same directory. Mirrors the Python
	// predicate pool.is_owner_entry so both repos agree on what is an owner.
	// `capabilities` is the GRANTS array on both writers; the build/protocol
	// feature list lives under `features` and is deliberately not consulted
	// here, so this predicate can never be satisfied by a build feature.
	if (!Obj.IsValid())
	{
		return false;
	}
	FString Role;
	if (Obj->TryGetStringField(TEXT("role"), Role) && Role == OwnerRole)
	{
		return true;
	}
	const TArray<TSharedPtr<FJsonValue>>* Caps = nullptr;
	if (Obj->TryGetArrayField(TEXT("capabilities"), Caps) && Caps)
	{
		for (const TSharedPtr<FJsonValue>& V : *Caps)
		{
			if (V.IsValid() && V->AsString() == OwnerRole)
			{
				return true;
			}
		}
	}
	return false;
}

bool DiscoverDrivers(TArray<FMjDriverInfo>& OutDrivers, FString& OutError)
{
	OutDrivers.Reset();
	OutError.Empty();

	const FString Dir = FURLabInstanceRegistry::ResolveRegistryDir();
	IFileManager& FM = IFileManager::Get();
	if (!FM.DirectoryExists(*Dir))
	{
		return true; // no registry yet -> no drivers, not an error
	}

	// Entries older than this are treated as dead (the driver heartbeats ~10s).
	constexpr double kTtlSeconds = 30.0;
	const FDateTime Now = FDateTime::UtcNow();

	TArray<FString> Files;
	FM.FindFiles(Files, *(Dir / TEXT("*.json")), /*Files=*/true, /*Directories=*/false);
	for (const FString& Name : Files)
	{
		const FString Path = Dir / Name;
		if ((Now - FM.GetTimeStamp(*Path)).GetTotalSeconds() > kTtlSeconds)
		{
			continue; // stale
		}
		FString Json;
		if (!FFileHelper::LoadFileToString(Json, *Path))
		{
			continue;
		}
		TSharedPtr<FJsonObject> Obj;
		const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Json);
		if (!FJsonSerializer::Deserialize(Reader, Obj) || !Obj.IsValid())
		{
			continue;
		}

		// Only render drivers (role or capability); the shared predicate keeps
		// this rule identical to the Python reader. Skips ordinary bridge
		// instances that share the same registry directory.
		if (!IsOwnerEntry(Obj))
		{
			continue;
		}

		FMjDriverInfo Info;
		Obj->TryGetStringField(TEXT("instance_id"), Info.InstanceId);
		Obj->TryGetStringField(TEXT("scene"), Info.Scene);
		Obj->TryGetStringField(TEXT("host"), Info.Host);
		Obj->TryGetStringField(TEXT("control"), Info.Control);
		Obj->TryGetStringField(TEXT("bus"), Info.Bus);
		// TryGetNumberField, not GetIntegerField: a foreign entry missing the key
		// should leave the default rather than log a read error per poll.
		Obj->TryGetNumberField(TEXT("ngeom"), Info.Ngeom);
		Obj->TryGetNumberField(TEXT("pid"), Info.Pid);
		if (Info.Control.IsEmpty())
		{
			continue;
		}
		// A fresh-mtime entry left behind by a crashed/killed driver still has a
		// valid heartbeat timestamp, so the TTL check alone lets it through. Prune
		// it here too: dead pid OR stale mtime both mean "not a live driver".
		if (!FPlatformProcess::IsApplicationRunning(static_cast<uint32>(Info.Pid)))
		{
			continue; // dead pid
		}
		OutDrivers.Add(MoveTemp(Info));
	}
	return true;
}
} // namespace URLabRender
