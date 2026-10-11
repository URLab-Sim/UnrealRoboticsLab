// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc.

// ============================================================================
// MjDiscoveryFeatureTests.cpp
//
// DISCOVERY / REGISTRY / BROWSER-JOIN behaviour: one entry schema, one writer per
// repository, one filter, one liveness rule. Exercises the in-process
// writer/reader pair with no live external owner:
//
//   FURLabInstanceRegistry::WriteEntry   (InstanceRegistry.cpp)
//   URLabRender::DiscoverDrivers        (MjDriverDiscovery.cpp)
//   URLabRender::IsOwnerEntry           (the single exported role/cap predicate)
//
// Isolation: every test points URLAB_REGISTRY_DIR at a unique scratch directory
// (honoured by ResolveRegistryDir), so the process's real registry is untouched
// and the reader only ever sees the entries a test wrote.
//
// PRINCIPLE: assert INTENDED behaviour. Several of these were written as
// bug-sitting "expected fail" tests against a writer that emitted no real geom
// count, advertised a gRPC endpoint nothing was listening on, and a reader that
// never pruned a dead pid. All three are fixed, so they now assert the fix.
// ============================================================================

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "Bridge/InstanceRegistry.h"
#include "Bridge/BridgeServerConfig.h"
#include "MuJoCo/Render/MjDriverDiscovery.h"

#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformMisc.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/Guid.h"
#include "Misc/DateTime.h"
#include "Misc/Timespan.h"

namespace
{
constexpr const TCHAR* kRegistryEnv = TEXT("URLAB_REGISTRY_DIR");

/**
 * RAII: redirect the registry directory at a fresh, empty scratch dir for the
 * lifetime of one test, restoring the prior URLAB_REGISTRY_DIR on scope exit and
 * deleting the scratch tree. Both WriteEntry and DiscoverDrivers resolve through
 * ResolveRegistryDir(), which reads this env var first, so this fully isolates the
 * writer/reader round-trip.
 */
struct FScopedRegistryDir
{
	FString Dir;
	FString Saved;
	bool bHadSaved = false;

	FScopedRegistryDir()
	{
		Saved = FPlatformMisc::GetEnvironmentVariable(kRegistryEnv);
		bHadSaved = !Saved.IsEmpty();
		Dir = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("URLabTest"),
			FString::Printf(TEXT("registry_%s"), *FGuid::NewGuid().ToString(EGuidFormats::Digits)));
		IFileManager::Get().MakeDirectory(*Dir, /*Tree=*/true);
		FPlatformMisc::SetEnvironmentVar(kRegistryEnv, *Dir);
	}

	~FScopedRegistryDir()
	{
		if (bHadSaved)
		{
			FPlatformMisc::SetEnvironmentVar(kRegistryEnv, *Saved);
		}
		else
		{
			// No pre-existing override: clear it so later tests see the default path.
			FPlatformMisc::SetEnvironmentVar(kRegistryEnv, TEXT(""));
		}
		IFileManager::Get().DeleteDirectory(*Dir, /*RequireExists=*/false, /*Tree=*/true);
	}
};

/** A broadcasting owner config: publish is on (bBroadcastOnRenderBus), so WriteEntry
 *  stamps role=render_owner plus the bus a viewer subscribes to. */
FURLabBridgeServerConfig MakeOwnerConfig(const FString& Id)
{
	FURLabBridgeServerConfig Cfg;
	Cfg.InstanceId = Id;
	Cfg.InstanceIndex = 0;
	Cfg.StepPort = 6100;
	Cfg.StatePort = 6101;
	Cfg.CamBasePort = 6102;
	Cfg.RenderBusPort = 6110;
	Cfg.bBroadcastOnRenderBus = true;
	return Cfg;
}

/** The live-process half of an entry. The registry takes it as a struct rather
 *  than defaulted parameters precisely so a call site cannot forget the geom
 *  count, which is what used to make every UE owner advertise ngeom 0. */
FURLabRegistryLiveState MakeLiveState(int32 Ngeom = 0, const FString& GrpcEndpoint = FString())
{
	FURLabRegistryLiveState Live;
	Live.UrlabVersion = TEXT("test-0.6.0");
	Live.bManagerPresent = false;
	Live.bBusy = false;
	Live.Ngeom = Ngeom;
	Live.GrpcEndpoint = GrpcEndpoint;
	return Live;
}

/** Read one entry file back as parsed JSON, so a test can inspect exactly what the
 *  WRITER emitted (independent of what the reader chooses to surface). */
bool LoadEntryJson(const FURLabBridgeServerConfig& Cfg, TSharedPtr<FJsonObject>& OutObj)
{
	const FString Path = FURLabInstanceRegistry::ResolveEntryPath(Cfg);
	FString Json;
	if (!FFileHelper::LoadFileToString(Json, *Path))
	{
		return false;
	}
	const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Json);
	return FJsonSerializer::Deserialize(Reader, OutObj) && OutObj.IsValid();
}

/** Hand-write a raw owner entry (bypassing WriteEntry) so a test can control the
 *  pid and mtime — needed to exercise the reader's pruning rules directly. */
FString WriteRawOwnerEntry(const FString& InstanceId, int32 Pid, const FString& Control)
{
	TSharedPtr<FJsonObject> Obj = MakeShared<FJsonObject>();
	Obj->SetStringField(TEXT("instance_id"), InstanceId);
	Obj->SetStringField(TEXT("role"), TEXT("render_owner"));
	Obj->SetStringField(TEXT("host"), TEXT("testhost"));
	Obj->SetStringField(TEXT("scene"), InstanceId);
	Obj->SetStringField(TEXT("control"), Control);
	Obj->SetStringField(TEXT("bus"), TEXT("tcp://testhost:6110"));
	Obj->SetNumberField(TEXT("pid"), static_cast<double>(Pid));
	Obj->SetNumberField(TEXT("ngeom"), 7);
	TArray<TSharedPtr<FJsonValue>> Caps;
	Caps.Add(MakeShared<FJsonValueString>(TEXT("render_owner")));
	Obj->SetArrayField(TEXT("capabilities"), Caps);
	Obj->SetStringField(TEXT("registry_written_at"), FDateTime::UtcNow().ToIso8601());

	FString Serialized;
	const TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&Serialized);
	FJsonSerializer::Serialize(Obj.ToSharedRef(), Writer);

	const FString Path = FPaths::Combine(
		FURLabInstanceRegistry::ResolveRegistryDir(),
		FString::Printf(TEXT("%s_%d.json"), *InstanceId, Pid));
	FFileHelper::SaveStringToFile(Serialized, *Path);
	return Path;
}
} // namespace

// ---------------------------------------------------------------------------
// URLab.Discovery.WriteEntryRoundTripsJoinerFields
//
// The core round-trip: a published owner written by WriteEntry is discovered by
// DiscoverDrivers with every joiner-needed field intact, and a lean
// (non-publishing) instance sharing the same directory is correctly filtered out.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjDiscoveryRoundTrip,
	"URLab.Discovery.WriteEntryRoundTripsJoinerFields",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjDiscoveryRoundTrip::RunTest(const FString& Parameters)
{
	FScopedRegistryDir Scoped;

	const FURLabBridgeServerConfig Owner = MakeOwnerConfig(TEXT("round_trip_owner"));
	FURLabInstanceRegistry::WriteEntry(Owner, MakeLiveState());

	// A lean instance (publish off) must NOT be advertised as a joinable driver.
	// It writes the SAME keys -- one entry shape -- and is filtered on its role,
	// not on which halves of the schema it happened to fill in.
	FURLabBridgeServerConfig Lean;
	Lean.InstanceId = TEXT("round_trip_lean");
	Lean.bBroadcastOnRenderBus = false;
	FURLabInstanceRegistry::WriteEntry(Lean, MakeLiveState());

	TArray<FMjDriverInfo> Drivers;
	FString Err;
	const bool bOk = URLabRender::DiscoverDrivers(Drivers, Err);
	TestTrue(TEXT("DiscoverDrivers succeeds"), bOk);
	TestEqual(TEXT("only the published owner is discovered (lean instance filtered out)"),
		Drivers.Num(), 1);
	if (Drivers.Num() != 1)
	{
		return false;
	}

	const FMjDriverInfo& D = Drivers[0];
	const FString Host = FPlatformProcess::ComputerName();
	TestEqual(TEXT("instance_id round-trips"), D.InstanceId, FString(TEXT("round_trip_owner")));
	TestEqual(TEXT("host round-trips"), D.Host, Host);
	TestEqual(TEXT("scene round-trips"), D.Scene, FString(TEXT("round_trip_owner")));
	TestEqual(TEXT("control endpoint (tcp://host:step_port) round-trips"),
		D.Control, FString::Printf(TEXT("tcp://%s:%d"), *Host, Owner.StepPort));
	TestEqual(TEXT("bus endpoint (tcp://host:render_bus_port) round-trips"),
		D.Bus, FString::Printf(TEXT("tcp://%s:%d"), *Host, Owner.RenderBusPort));
	TestEqual(TEXT("pid round-trips to the writing process"),
		D.Pid, static_cast<int32>(FPlatformProcess::GetCurrentProcessId()));

	return true;
}

// ---------------------------------------------------------------------------
// URLab.Discovery.WriteEntryEmitsNgeom
//
// The writer emits the model's REAL geom count and it survives the round-trip to
// the reader. `ngeom` used to be a defaulted parameter no call site passed, so
// the key was always 0 and every browser listed a UE owner as empty while the
// same process's render_hello reported the truth.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjDiscoveryEmitsNgeom,
	"URLab.Discovery.WriteEntryEmitsNgeom",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjDiscoveryEmitsNgeom::RunTest(const FString& Parameters)
{
	FScopedRegistryDir Scoped;

	const FURLabBridgeServerConfig Owner = MakeOwnerConfig(TEXT("ngeom_owner"));
	FURLabInstanceRegistry::WriteEntry(Owner, MakeLiveState(/*Ngeom=*/42));

	TSharedPtr<FJsonObject> Raw;
	TestTrue(TEXT("owner entry file is readable"), LoadEntryJson(Owner, Raw));
	if (!Raw.IsValid())
	{
		return false;
	}
	TestTrue(TEXT("writer emits `ngeom`"), Raw->HasField(TEXT("ngeom")));
	TestEqual(TEXT("`ngeom` carries the value it was given, not 0"),
		static_cast<int32>(Raw->GetIntegerField(TEXT("ngeom"))), 42);

	TArray<FMjDriverInfo> Drivers;
	FString Err;
	TestTrue(TEXT("DiscoverDrivers succeeds"), URLabRender::DiscoverDrivers(Drivers, Err));
	TestEqual(TEXT("the owner is discovered"), Drivers.Num(), 1);
	if (Drivers.Num() == 1)
	{
		TestEqual(TEXT("ngeom round-trips to the reader"), Drivers[0].Ngeom, 42);
	}
	return true;
}

// ---------------------------------------------------------------------------
// URLab.Discovery.WriteEntryEmitsGrpcEndpoint
//
// `grpc` is advertised only when a dm_env_rpc server is actually listening, and
// when it is, it is dialable: a scheme, and this machine's name rather than the
// bind wildcard. The writer used to publish the well-known port from every
// broadcasting instance, including builds with no gRPC module at all, so a
// discovering session failed at connect time instead of at discovery time; and
// it published a scheme-less host:port, which the client transport factory
// routes to ZMQ.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjDiscoveryEmitsGrpc,
	"URLab.Discovery.WriteEntryEmitsGrpcEndpoint",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjDiscoveryEmitsGrpc::RunTest(const FString& Parameters)
{
	FScopedRegistryDir Scoped;

	// No gRPC server listening: null, and `grpc` is absent from `transports`.
	const FURLabBridgeServerConfig Silent = MakeOwnerConfig(TEXT("grpc_silent"));
	FURLabInstanceRegistry::WriteEntry(Silent, MakeLiveState());
	{
		TSharedPtr<FJsonObject> Raw;
		TestTrue(TEXT("silent owner entry file is readable"), LoadEntryJson(Silent, Raw));
		if (!Raw.IsValid())
		{
			return false;
		}
		TestTrue(TEXT("no listening gRPC server -> `grpc` is null"),
			Raw->HasTypedField<EJson::Null>(TEXT("grpc")));
		const TArray<TSharedPtr<FJsonValue>>* Transports = nullptr;
		TestTrue(TEXT("`transports` is written"),
			Raw->TryGetArrayField(TEXT("transports"), Transports) && Transports != nullptr);
		if (Transports)
		{
			bool bHasGrpc = false;
			for (const TSharedPtr<FJsonValue>& V : *Transports)
			{
				bHasGrpc |= (V.IsValid() && V->AsString() == TEXT("grpc"));
			}
			TestFalse(TEXT("`transports` does not claim grpc"), bHasGrpc);
		}
	}

	// A listening gRPC server: the endpoint is advertised verbatim, with scheme.
	const FURLabBridgeServerConfig Serving = MakeOwnerConfig(TEXT("grpc_serving"));
	const FString Endpoint = FString::Printf(TEXT("grpc://%s:50051"),
		FPlatformProcess::ComputerName());
	FURLabInstanceRegistry::WriteEntry(Serving, MakeLiveState(/*Ngeom=*/0, Endpoint));
	{
		TSharedPtr<FJsonObject> Raw;
		TestTrue(TEXT("serving owner entry file is readable"), LoadEntryJson(Serving, Raw));
		if (!Raw.IsValid())
		{
			return false;
		}
		FString Grpc;
		TestTrue(TEXT("`grpc` is a string when a server is listening"),
			Raw->TryGetStringField(TEXT("grpc"), Grpc));
		TestEqual(TEXT("`grpc` is the dialable endpoint, scheme included"), Grpc, Endpoint);
		TestTrue(TEXT("`grpc` carries a scheme so it is not dialled over ZMQ"),
			Grpc.StartsWith(TEXT("grpc://")));
		TestFalse(TEXT("`grpc` never advertises the bind wildcard"),
			Grpc.Contains(TEXT("0.0.0.0")));
	}

	return true;
}

// ---------------------------------------------------------------------------
// URLab.Discovery.IsOwnerEntryClassifies
//
// IsOwnerEntry is the ONE exported (URLAB_API) role/grant predicate every reader
// shares (it mirrors Python pool.is_owner_entry). The retired editor browser no longer
// has a second discovery adapter, leaving one filter and one liveness rule.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjDiscoveryIsOwnerEntry,
	"URLab.Discovery.IsOwnerEntryClassifies",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjDiscoveryIsOwnerEntry::RunTest(const FString& Parameters)
{
	// role == render_owner -> owner
	{
		TSharedPtr<FJsonObject> Obj = MakeShared<FJsonObject>();
		Obj->SetStringField(TEXT("role"), TEXT("render_owner"));
		TestTrue(TEXT("role=render_owner classifies as owner"),
			URLabRender::IsOwnerEntry(Obj));
	}
	// capabilities (grants) contains render_owner -> owner
	{
		TSharedPtr<FJsonObject> Obj = MakeShared<FJsonObject>();
		TArray<TSharedPtr<FJsonValue>> Caps;
		Caps.Add(MakeShared<FJsonValueString>(TEXT("camera_rpc")));
		Caps.Add(MakeShared<FJsonValueString>(TEXT("render_owner")));
		Obj->SetArrayField(TEXT("capabilities"), Caps);
		TestTrue(TEXT("capabilities[]=render_owner classifies as owner"),
			URLabRender::IsOwnerEntry(Obj));
	}
	// ordinary bridge instance (neither role nor grant) -> not an owner. Its
	// build features live under `features`, which this predicate never reads --
	// the two arrays used to be one, and that is what made capability filters
	// across the two repositories unsatisfiable.
	{
		TSharedPtr<FJsonObject> Obj = MakeShared<FJsonObject>();
		Obj->SetStringField(TEXT("role"), TEXT("bridge"));
		Obj->SetArrayField(TEXT("capabilities"), TArray<TSharedPtr<FJsonValue>>());
		TArray<TSharedPtr<FJsonValue>> Feats;
		Feats.Add(MakeShared<FJsonValueString>(TEXT("render_sync")));
		Feats.Add(MakeShared<FJsonValueString>(TEXT("model_upload")));
		Obj->SetArrayField(TEXT("features"), Feats);
		TestFalse(TEXT("ordinary bridge instance is not an owner"),
			URLabRender::IsOwnerEntry(Obj));
	}
	// null object -> not an owner (documented contract)
	{
		TSharedPtr<FJsonObject> Null;
		TestFalse(TEXT("null entry is not an owner"),
			URLabRender::IsOwnerEntry(Null));
	}
	return true;
}

// ---------------------------------------------------------------------------
// URLab.Discovery.PrunesStaleMtimeEntries
//
// DiscoverDrivers drops entries whose file mtime is older than the 30 s heartbeat
// TTL. A fresh owner is discovered; a byte-identical owner backdated well past
// the TTL is pruned. The other half of the one liveness rule.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjDiscoveryPrunesStale,
	"URLab.Discovery.PrunesStaleMtimeEntries",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjDiscoveryPrunesStale::RunTest(const FString& Parameters)
{
	FScopedRegistryDir Scoped;

	// Use a guaranteed-live pid for both fixtures so this test isolates the mtime
	// TTL path from pid-liveness pruning (which is covered by PrunesDeadPidEntries).
	const int32 LivePid = static_cast<int32>(FPlatformProcess::GetCurrentProcessId());
	WriteRawOwnerEntry(TEXT("fresh_owner"), LivePid, TEXT("tcp://testhost:6100"));
	const FString StalePath =
		WriteRawOwnerEntry(TEXT("stale_owner"), LivePid, TEXT("tcp://testhost:6200"));

	// Backdate the stale entry two minutes past the 30 s TTL.
	IFileManager::Get().SetTimeStamp(*StalePath,
		FDateTime::UtcNow() - FTimespan::FromSeconds(120.0));

	TArray<FMjDriverInfo> Drivers;
	FString Err;
	TestTrue(TEXT("DiscoverDrivers succeeds"),
		URLabRender::DiscoverDrivers(Drivers, Err));
	TestEqual(TEXT("only the fresh owner survives the mtime TTL"), Drivers.Num(), 1);
	if (Drivers.Num() == 1)
	{
		TestEqual(TEXT("survivor is the fresh entry"),
			Drivers[0].InstanceId, FString(TEXT("fresh_owner")));
	}
	return true;
}

// ---------------------------------------------------------------------------
// URLab.Discovery.PrunesDeadPidEntries
//
// An entry whose mtime is fresh but whose PID is dead is not advertised as a
// joinable driver. Half of the one liveness rule: fresh mtime AND running pid,
// applied by every reader in both repositories.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjDiscoveryPrunesDeadPid,
	"URLab.Discovery.PrunesDeadPidEntries",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjDiscoveryPrunesDeadPid::RunTest(const FString& Parameters)
{
	FScopedRegistryDir Scoped;

	// Fresh mtime, but a PID that cannot correspond to a live process.
	constexpr int32 DeadPid = 0x7FFFFFFE;
	WriteRawOwnerEntry(TEXT("dead_owner"), DeadPid, TEXT("tcp://testhost:6300"));

	TArray<FMjDriverInfo> Drivers;
	FString Err;
	TestTrue(TEXT("DiscoverDrivers succeeds"),
		URLabRender::DiscoverDrivers(Drivers, Err));

	TestEqual(TEXT("a fresh-mtime entry with a dead PID is pruned from discovery"),
		Drivers.Num(), 0);

	return true;
}
