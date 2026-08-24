// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "MuJoCo/Fast/MjDriverDiscovery.h"

#include "Bridge/InstanceRegistry.h"

#include "HAL/FileManager.h"
#include "HAL/PlatformProcess.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/DateTime.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Dom/JsonObject.h"

namespace URLabFastPath
{
const TCHAR* const OwnerRole = TEXT("fastpath_owner");

bool IsOwnerEntry(const TSharedPtr<FJsonObject>& Obj)
{
	// Only fast-path drivers (role or capability). Skips ordinary bridge
	// instances that share the same registry directory. Mirrors the Python
	// predicate pool.is_owner_entry so both repos agree on what is an owner.
	if (!Obj.IsValid())
	{
		return false;
	}
	if (Obj->GetStringField(TEXT("role")) == OwnerRole)
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

		// Only fast-path drivers (role or capability); the shared predicate keeps
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
		Info.Ngeom = static_cast<int32>(Obj->GetIntegerField(TEXT("ngeom")));
		Info.Pid = static_cast<int32>(Obj->GetIntegerField(TEXT("pid")));
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
} // namespace URLabFastPath
