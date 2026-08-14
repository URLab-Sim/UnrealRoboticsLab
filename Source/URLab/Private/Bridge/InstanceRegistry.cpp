// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "Bridge/InstanceRegistry.h"
#include "Bridge/BridgeServerConfig.h"

#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/DateTime.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformMisc.h"

namespace
{
/** Instance id used for the file name and the JSON, falling back to the SHM
 *  default so a single-editor entry stays readable. */
FString EffectiveInstanceId(const FURLabBridgeServerConfig& Cfg)
{
	return Cfg.InstanceId.IsEmpty() ? FString(TEXT("live")) : Cfg.InstanceId;
}
} // namespace

const TArray<FString>& FURLabInstanceRegistry::Capabilities()
{
	static const TArray<FString> Caps = {
		TEXT("render_sync"),
		TEXT("render_async"),
		TEXT("shm_rpc"),
		TEXT("model_upload"),
		TEXT("content_cache"),
	};
	return Caps;
}

FString FURLabInstanceRegistry::ResolveRegistryDir()
{
	const FString Override = FPlatformMisc::GetEnvironmentVariable(TEXT("URLAB_REGISTRY_DIR"));
	if (!Override.IsEmpty())
		return Override;

#if PLATFORM_WINDOWS
	FString Base = FPlatformMisc::GetEnvironmentVariable(TEXT("LOCALAPPDATA"));
	if (Base.IsEmpty())
		Base = FPlatformProcess::UserSettingsDir();
#else
	FString Base = FPlatformMisc::GetEnvironmentVariable(TEXT("XDG_CACHE_HOME"));
	if (Base.IsEmpty())
		Base = FPaths::Combine(FPlatformMisc::GetEnvironmentVariable(TEXT("HOME")), TEXT(".cache"));
#endif
	return FPaths::Combine(Base, TEXT("URLab"), TEXT("registry"));
}

FString FURLabInstanceRegistry::ResolveEntryPath(const FURLabBridgeServerConfig& Cfg)
{
	const FString FileName = FString::Printf(TEXT("%s_%u.json"),
		*EffectiveInstanceId(Cfg), FPlatformProcess::GetCurrentProcessId());
	return FPaths::Combine(ResolveRegistryDir(), FileName);
}

void FURLabInstanceRegistry::WriteEntry(const FURLabBridgeServerConfig& Cfg,
	const FString& UrlabVersion, bool bManagerPresent, bool bBusy)
{
	TSharedPtr<FJsonObject> Entry = MakeShared<FJsonObject>();
	Entry->SetStringField(TEXT("instance_id"), EffectiveInstanceId(Cfg));
	Entry->SetNumberField(TEXT("index"), Cfg.InstanceIndex);
	Entry->SetNumberField(TEXT("pid"), static_cast<double>(FPlatformProcess::GetCurrentProcessId()));
	Entry->SetStringField(TEXT("host"), FPlatformProcess::ComputerName());
	Entry->SetNumberField(TEXT("step_port"), Cfg.StepPort);
	Entry->SetNumberField(TEXT("state_port"), Cfg.StatePort);
	Entry->SetNumberField(TEXT("cam_base_port"), Cfg.CamBasePort);
	Entry->SetBoolField(TEXT("manager_present"), bManagerPresent);
	Entry->SetBoolField(TEXT("busy"), bBusy);
	Entry->SetStringField(TEXT("urlab_version"), UrlabVersion);

	TArray<TSharedPtr<FJsonValue>> Caps;
	for (const FString& Cap : Capabilities())
		Caps.Add(MakeShared<FJsonValueString>(Cap));

	// A viewer-broadcasting instance is also a fast-path owner: it serves its MJB
	// over the control channel (fastpath_hello on the step port) and publishes the
	// geoms transform bus on the viewer port. Advertise that so a fast-path
	// renderer's server browser can discover and connect to it, exactly like a
	// Python owner. Off unless this instance broadcasts.
	if (Cfg.bBroadcastViewers)
	{
		Caps.Add(MakeShared<FJsonValueString>(TEXT("fastpath_owner")));
		const FString Host = FPlatformProcess::ComputerName();
		Entry->SetStringField(TEXT("role"), TEXT("fastpath_owner"));
		Entry->SetStringField(TEXT("control"),
			FString::Printf(TEXT("tcp://%s:%d"), *Host, Cfg.StepPort));
		Entry->SetStringField(TEXT("bus"),
			FString::Printf(TEXT("tcp://%s:%d"), *Host, Cfg.ViewerPort));
		Entry->SetNumberField(TEXT("viewer_port"), Cfg.ViewerPort);
		Entry->SetStringField(TEXT("scene"), EffectiveInstanceId(Cfg));
	}
	Entry->SetArrayField(TEXT("capabilities"), Caps);

	Entry->SetStringField(TEXT("registry_written_at"), FDateTime::UtcNow().ToIso8601());

	FString Serialized;
	TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&Serialized);
	FJsonSerializer::Serialize(Entry.ToSharedRef(), Writer);

	const FString Path = ResolveEntryPath(Cfg);
	IFileManager::Get().MakeDirectory(*FPaths::GetPath(Path), /*Tree=*/true);
	FFileHelper::SaveStringToFile(Serialized, *Path);
}

void FURLabInstanceRegistry::RefreshEntry(const FURLabBridgeServerConfig& Cfg,
	const FString& UrlabVersion, bool bManagerPresent, bool bBusy)
{
	WriteEntry(Cfg, UrlabVersion, bManagerPresent, bBusy);
}

void FURLabInstanceRegistry::RemoveEntry(const FURLabBridgeServerConfig& Cfg)
{
	IFileManager::Get().Delete(*ResolveEntryPath(Cfg), /*RequireExists=*/false, /*EvenReadOnly=*/true);
}
