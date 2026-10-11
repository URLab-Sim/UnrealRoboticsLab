// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "Bridge/InstanceRegistry.h"
#include "Bridge/BridgeServerConfig.h"
#include "MuJoCo/Render/MjDriverDiscovery.h"

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
/** Role of an instance that publishes nothing for a viewer to join. Written
 *  explicitly rather than omitted: a missing key used to mean "not an owner"
 *  to some readers and "invent the owner role" to others. */
const TCHAR* const BridgeRole = TEXT("bridge");

void AddString(TArray<TSharedPtr<FJsonValue>>& Out, const FString& Value)
{
	Out.Add(MakeShared<FJsonValueString>(Value));
}
} // namespace

FString FURLabInstanceRegistry::ResolveInstanceId(const FURLabBridgeServerConfig& Cfg)
{
	// Falls back to the SHM default so a single-editor entry stays readable.
	return Cfg.InstanceId.IsEmpty() ? FString(TEXT("live")) : Cfg.InstanceId;
}

const TArray<FString>& FURLabInstanceRegistry::Features()
{
	static const TArray<FString> Feats = {
		TEXT("render_sync"),
		TEXT("render_async"),
		TEXT("model_upload"),
		TEXT("content_cache"),
	};
	return Feats;
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
		*ResolveInstanceId(Cfg), FPlatformProcess::GetCurrentProcessId());
	return FPaths::Combine(ResolveRegistryDir(), FileName);
}

void FURLabInstanceRegistry::WriteEntry(const FURLabBridgeServerConfig& Cfg,
	const FURLabRegistryLiveState& Live)
{
	// One entry shape, whatever this instance is configured to do. Every key
	// below is written unconditionally, because the alternative -- keys that
	// appear only when bBroadcastOnRenderBus is set -- made a single file two
	// different shapes and pushed the difference onto four readers, each of
	// which invented its own answer for the missing halves.
	const FString Host = FPlatformProcess::ComputerName();
	const bool bOwner = Cfg.bBroadcastOnRenderBus;

	TSharedPtr<FJsonObject> Entry = MakeShared<FJsonObject>();
	Entry->SetStringField(TEXT("instance_id"), ResolveInstanceId(Cfg));
	Entry->SetNumberField(TEXT("index"), Cfg.InstanceIndex);
	Entry->SetNumberField(TEXT("pid"), static_cast<double>(FPlatformProcess::GetCurrentProcessId()));
	Entry->SetStringField(TEXT("host"), Host);

	// A broadcasting instance is also a render owner: it serves its
	// model over the typed render service and
	// publishes the geoms transform bus on the render bus port. Anything else is a
	// plain bridge, which the role-filtered readers skip.
	Entry->SetStringField(TEXT("role"), bOwner ? URLabRender::OwnerRole : BridgeRole);

	// capabilities[] are GRANTS: what a peer may ask of this instance. They are
	// the same vocabulary the Python owner writes, so a pool filtering on a grant
	// matches an Unreal owner. features[] are the protocol/build features of this
	// binary. The two used to share the `capabilities` key while holding disjoint
	// vocabularies, which made every cross-repo capability filter silently
	// unsatisfiable.
	TArray<TSharedPtr<FJsonValue>> Caps;
	if (bOwner)
		AddString(Caps, URLabRender::OwnerRole);
	for (const FString& Token : Live.GrantTokens)
		AddString(Caps, Token);
	Entry->SetArrayField(TEXT("capabilities"), Caps);

	TArray<TSharedPtr<FJsonValue>> Feats;
	for (const FString& Feature : Features())
		AddString(Feats, Feature);
	Entry->SetArrayField(TEXT("features"), Feats);

	Entry->SetStringField(TEXT("scene"), ResolveInstanceId(Cfg));
	Entry->SetNumberField(TEXT("ngeom"), Live.Ngeom);

	// The canonical port block, always this instance's real configured ports.
	Entry->SetNumberField(TEXT("step_port"), Cfg.StepPort);
	Entry->SetNumberField(TEXT("state_port"), Cfg.StatePort);
	Entry->SetNumberField(TEXT("cam_base_port"), Cfg.CamBasePort);

	// Endpoints carry a scheme so a reader never has to guess one. A
	// scheme-less host:port from this writer is routed to ZMQ by the client's
	// transport factory, which is how a gRPC endpoint used to end up dialled
	// over ZMQ.
	Entry->SetStringField(TEXT("control"),
		FString::Printf(TEXT("tcp://%s:%d"), *Host, Cfg.StepPort));

	TArray<TSharedPtr<FJsonValue>> Transports;
	AddString(Transports, TEXT("zmq"));

	// The transform bus only exists while this instance broadcasts; null (not a
	// plausible-looking endpoint) when it does not.
	if (bOwner)
	{
		Entry->SetStringField(TEXT("bus"),
			FString::Printf(TEXT("tcp://%s:%d"), *Host, Cfg.RenderBusPort));
	}
	else
	{
		Entry->SetField(TEXT("bus"), MakeShared<FJsonValueNull>());
	}

	// gRPC is advertised only when a dm_env_rpc server is actually listening in
	// this process (the caller checks the bound transport, not the configured
	// port). Advertising the well-known port whenever the instance broadcast
	// meant a discovering session read an endpoint from a build with no gRPC
	// server at all, dialled it, and failed at connect time rather than at
	// discovery time. Never the bind wildcard: the host is this machine's name.
	if (!Live.GrpcEndpoint.IsEmpty())
	{
		AddString(Transports, TEXT("grpc"));
		Entry->SetStringField(TEXT("grpc"), Live.GrpcEndpoint);
	}
	else
	{
		Entry->SetField(TEXT("grpc"), MakeShared<FJsonValueNull>());
	}
	Entry->SetArrayField(TEXT("transports"), Transports);

	Entry->SetBoolField(TEXT("manager_present"), Live.bManagerPresent);
	Entry->SetBoolField(TEXT("busy"), Live.bBusy);
	Entry->SetStringField(TEXT("urlab_version"), Live.UrlabVersion);

	// ISO-8601 UTC on both writers. The readers used to carry a hand-kept pair
	// of helpers whose only job was to accept the other writer's shape.
	Entry->SetStringField(TEXT("registry_written_at"), FDateTime::UtcNow().ToIso8601());

	FString Serialized;
	TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&Serialized);
	FJsonSerializer::Serialize(Entry.ToSharedRef(), Writer);

	const FString Path = ResolveEntryPath(Cfg);
	IFileManager& FM = IFileManager::Get();
	FM.MakeDirectory(*FPaths::GetPath(Path), /*Tree=*/true);

	// Write beside the entry, then rename over it. A rename within a directory
	// is atomic, so a reader globbing this directory mid-heartbeat sees either
	// the previous entry or the new one and never a half-written file. Readers
	// swallow a parse failure and skip the entry, so overwriting in place made
	// a live instance vanish from discovery for a whole poll. The temp name
	// does not end in .json, so no reader's glob picks it up. Matches the
	// Python writer, which has always used mkstemp + os.replace.
	const FString TempPath = FString::Printf(TEXT("%s.%u.tmp"),
		*Path, FPlatformProcess::GetCurrentProcessId());
	if (!FFileHelper::SaveStringToFile(Serialized, *TempPath))
	{
		return;
	}
	if (!FM.Move(*Path, *TempPath, /*bReplace=*/true, /*bEvenIfReadOnly=*/true))
	{
		FM.Delete(*TempPath, /*RequireExists=*/false, /*EvenReadOnly=*/true);
	}
}

void FURLabInstanceRegistry::RemoveEntry(const FURLabBridgeServerConfig& Cfg)
{
	IFileManager::Get().Delete(*ResolveEntryPath(Cfg), /*RequireExists=*/false, /*EvenReadOnly=*/true);
}
