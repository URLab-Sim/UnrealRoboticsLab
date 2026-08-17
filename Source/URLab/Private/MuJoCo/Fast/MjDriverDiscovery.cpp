// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "MuJoCo/Fast/MjDriverDiscovery.h"

#include "Bridge/InstanceRegistry.h"

#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/DateTime.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Dom/JsonObject.h"

namespace URLabFastPath
{
bool DiscoverOwners(TArray<FMjDriverInfo>& OutOwners, FString& OutError)
{
	OutOwners.Reset();
	OutError.Empty();

	const FString Dir = FURLabInstanceRegistry::ResolveRegistryDir();
	IFileManager& FM = IFileManager::Get();
	if (!FM.DirectoryExists(*Dir))
	{
		return true; // no registry yet -> no owners, not an error
	}

	// Entries older than this are treated as dead (the owner heartbeats ~10s).
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

		// Only fast-path owners (role or capability). Skips ordinary bridge
		// instances that share the same registry directory.
		bool bIsOwner = Obj->GetStringField(TEXT("role")) == TEXT("fastpath_owner");
		const TArray<TSharedPtr<FJsonValue>>* Caps = nullptr;
		if (!bIsOwner && Obj->TryGetArrayField(TEXT("capabilities"), Caps) && Caps)
		{
			for (const TSharedPtr<FJsonValue>& V : *Caps)
			{
				if (V.IsValid() && V->AsString() == TEXT("fastpath_owner"))
				{
					bIsOwner = true;
					break;
				}
			}
		}
		if (!bIsOwner)
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
		if (!Info.Control.IsEmpty())
		{
			OutOwners.Add(MoveTemp(Info));
		}
	}
	return true;
}
} // namespace URLabFastPath
