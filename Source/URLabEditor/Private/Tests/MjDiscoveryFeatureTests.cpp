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
// DISCOVERY / REGISTRY / BROWSER-JOIN behaviour built since v0.6.0-beta, grounded
// in source-of-truth §12 (one entry schema, one writer per repo, one filter) and
// §17. Exercises the in-process writer/reader pair with no live external owner:
//
//   FURLabInstanceRegistry::WriteEntry   (InstanceRegistry.cpp)
//   URLabFastPath::DiscoverDrivers        (MjDriverDiscovery.cpp)
//   URLabFastPath::IsOwnerEntry           (the single exported role/cap predicate)
//
// Isolation: every test points URLAB_REGISTRY_DIR at a unique scratch directory
// (honoured by ResolveRegistryDir), so the process's real registry is untouched
// and the reader only ever sees the entries a test wrote.
//
// PRINCIPLE (task brief): assert INTENDED behaviour. The bug-sitting tests below
// are marked "EXPECTED FAIL (addendum §A5/§A6/§B)" and FAIL against today's code
// on purpose — the failure is the signal the latent bug is still live.
// ============================================================================

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "Bridge/InstanceRegistry.h"
#include "Bridge/BridgeServerConfig.h"
#include "MuJoCo/Fast/MjDriverDiscovery.h"

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

/** A broadcasting owner config: publish is on (bBroadcastViewers), so WriteEntry
 *  stamps role=fastpath_owner + control/bus/scene — the joinable shape a browser
 *  needs (source-of-truth §5 `publish` writes the registry role). */
FURLabBridgeServerConfig MakeOwnerConfig(const FString& Id)
{
	FURLabBridgeServerConfig Cfg;
	Cfg.InstanceId = Id;
	Cfg.InstanceIndex = 0;
	Cfg.StepPort = 6100;
	Cfg.StatePort = 6101;
	Cfg.CamBasePort = 6102;
	Cfg.ViewerPort = 6110;
	Cfg.bBroadcastViewers = true;
	return Cfg;
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
	Obj->SetStringField(TEXT("role"), TEXT("fastpath_owner"));
	Obj->SetStringField(TEXT("host"), TEXT("testhost"));
	Obj->SetStringField(TEXT("scene"), InstanceId);
	Obj->SetStringField(TEXT("control"), Control);
	Obj->SetStringField(TEXT("bus"), TEXT("tcp://testhost:6110"));
	Obj->SetNumberField(TEXT("pid"), static_cast<double>(Pid));
	Obj->SetNumberField(TEXT("ngeom"), 7);
	TArray<TSharedPtr<FJsonValue>> Caps;
	Caps.Add(MakeShared<FJsonValueString>(TEXT("fastpath_owner")));
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
// DiscoverDrivers with every joiner-needed field intact (§12 schema), and a lean
// (non-publishing) instance sharing the same directory is correctly filtered out.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjDiscoveryRoundTrip,
	"URLab.Discovery.WriteEntryRoundTripsJoinerFields",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjDiscoveryRoundTrip::RunTest(const FString& Parameters)
{
	FScopedRegistryDir Scoped;

	const FURLabBridgeServerConfig Owner = MakeOwnerConfig(TEXT("round_trip_owner"));
	FURLabInstanceRegistry::WriteEntry(Owner, TEXT("test-0.6.0"),
		/*bManagerPresent=*/false, /*bBusy=*/false);

	// A lean instance (publish off) must NOT be advertised as a joinable driver.
	FURLabBridgeServerConfig Lean;
	Lean.InstanceId = TEXT("round_trip_lean");
	Lean.bBroadcastViewers = false;
	FURLabInstanceRegistry::WriteEntry(Lean, TEXT("test-0.6.0"), false, false);

	TArray<FMjDriverInfo> Drivers;
	FString Err;
	const bool bOk = URLabFastPath::DiscoverDrivers(Drivers, Err);
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
	TestEqual(TEXT("bus endpoint (tcp://host:viewer_port) round-trips"),
		D.Bus, FString::Printf(TEXT("tcp://%s:%d"), *Host, Owner.ViewerPort));
	TestEqual(TEXT("pid round-trips to the writing process"),
		D.Pid, static_cast<int32>(FPlatformProcess::GetCurrentProcessId()));

	return true;
}

// ---------------------------------------------------------------------------
// URLab.Discovery.WriteEntryEmitsNgeom     [EXPECTED FAIL — addendum §A5]
//
// DiscoverDrivers reads `ngeom` (MjDriverDiscovery.cpp:100) but WriteEntry never
// emits it, so a UE-discovered UE owner always reports Ngeom=0 (GetIntegerField
// returns 0 on a missing key — silent). Intended: the writer emits the model's
// geom count and it survives the round-trip. Fails today: the key is absent.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjDiscoveryEmitsNgeom,
	"URLab.Discovery.WriteEntryEmitsNgeom",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjDiscoveryEmitsNgeom::RunTest(const FString& Parameters)
{
	FScopedRegistryDir Scoped;

	const FURLabBridgeServerConfig Owner = MakeOwnerConfig(TEXT("ngeom_owner"));
	FURLabInstanceRegistry::WriteEntry(Owner, TEXT("test-0.6.0"), false, false);

	TSharedPtr<FJsonObject> Raw;
	TestTrue(TEXT("owner entry file is readable"), LoadEntryJson(Owner, Raw));
	if (!Raw.IsValid())
	{
		return false;
	}

	// EXPECTED FAIL (addendum §A5): WriteEntry (InstanceRegistry.cpp) emits no
	// `ngeom` field, so the reader silently reports 0. The writer should emit it.
	TestTrue(TEXT("EXPECTED FAIL (addendum §A5): WriteEntry must emit `ngeom` that "
		"MjDriverDiscovery reads; today the key is absent and owners report Ngeom=0"),
		Raw->HasField(TEXT("ngeom")));

	return true;
}

// ---------------------------------------------------------------------------
// URLab.Discovery.WriteEntryEmitsGrpcEndpoint     [EXPECTED FAIL — addendum §A6]
//
// WriteEntry hardcodes `grpc: null` (InstanceRegistry.cpp:113) even for a
// published owner; a gRPC (dm_env_rpc) mirror-joiner therefore can never discover
// a UE owner's dm_env endpoint. Intended: when the instance serves dm_env_rpc,
// the registry advertises a non-null grpc endpoint. Fails today: always null.
// (Root cause noted in the addendum: FURLabBridgeServerConfig carries no dm_env /
// grpc port field yet, so WriteEntry has nothing to emit — the fix is additive.)
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjDiscoveryEmitsGrpc,
	"URLab.Discovery.WriteEntryEmitsGrpcEndpoint",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjDiscoveryEmitsGrpc::RunTest(const FString& Parameters)
{
	FScopedRegistryDir Scoped;

	const FURLabBridgeServerConfig Owner = MakeOwnerConfig(TEXT("grpc_owner"));
	FURLabInstanceRegistry::WriteEntry(Owner, TEXT("test-0.6.0"), false, false);

	TSharedPtr<FJsonObject> Raw;
	TestTrue(TEXT("owner entry file is readable"), LoadEntryJson(Owner, Raw));
	if (!Raw.IsValid())
	{
		return false;
	}

	const bool bGrpcIsNull = Raw->HasTypedField<EJson::Null>(TEXT("grpc"));
	// EXPECTED FAIL (addendum §A6): a published owner that serves dm_env_rpc should
	// advertise a non-null grpc endpoint so gRPC mirror-joiners can discover it.
	TestFalse(TEXT("EXPECTED FAIL (addendum §A6): registry `grpc` must be a real "
		"endpoint when the instance serves dm_env_rpc; today it is hardcoded null"),
		bGrpcIsNull);

	return true;
}

// ---------------------------------------------------------------------------
// URLab.Discovery.IsOwnerEntryClassifies
//
// IsOwnerEntry is the ONE exported (URLAB_API) role/capability predicate that
// every reader must share (source-of-truth §12; mirrors Python pool.is_owner_entry).
// NOTE: the editor re-inlines this same rule at MjLevelOps.cpp:749 instead of
// calling this exported helper (cleanup_audit_addendum §B) — the two copies must
// stay byte-identical, which is exactly the drift risk a single predicate removes.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjDiscoveryIsOwnerEntry,
	"URLab.Discovery.IsOwnerEntryClassifies",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjDiscoveryIsOwnerEntry::RunTest(const FString& Parameters)
{
	// role == fastpath_owner -> owner
	{
		TSharedPtr<FJsonObject> Obj = MakeShared<FJsonObject>();
		Obj->SetStringField(TEXT("role"), TEXT("fastpath_owner"));
		TestTrue(TEXT("role=fastpath_owner classifies as owner"),
			URLabFastPath::IsOwnerEntry(Obj));
	}
	// capabilities contains fastpath_owner -> owner
	{
		TSharedPtr<FJsonObject> Obj = MakeShared<FJsonObject>();
		TArray<TSharedPtr<FJsonValue>> Caps;
		Caps.Add(MakeShared<FJsonValueString>(TEXT("render_sync")));
		Caps.Add(MakeShared<FJsonValueString>(TEXT("fastpath_owner")));
		Obj->SetArrayField(TEXT("capabilities"), Caps);
		TestTrue(TEXT("capabilities[]=fastpath_owner classifies as owner"),
			URLabFastPath::IsOwnerEntry(Obj));
	}
	// ordinary bridge instance (neither role nor cap) -> not an owner
	{
		TSharedPtr<FJsonObject> Obj = MakeShared<FJsonObject>();
		Obj->SetStringField(TEXT("role"), TEXT("bridge"));
		TArray<TSharedPtr<FJsonValue>> Caps;
		Caps.Add(MakeShared<FJsonValueString>(TEXT("render_sync")));
		Caps.Add(MakeShared<FJsonValueString>(TEXT("shm_rpc")));
		Obj->SetArrayField(TEXT("capabilities"), Caps);
		TestFalse(TEXT("ordinary bridge instance is not an owner"),
			URLabFastPath::IsOwnerEntry(Obj));
	}
	// null object -> not an owner (documented contract)
	{
		TSharedPtr<FJsonObject> Null;
		TestFalse(TEXT("null entry is not an owner"),
			URLabFastPath::IsOwnerEntry(Null));
	}
	return true;
}

// ---------------------------------------------------------------------------
// URLab.Discovery.PrunesStaleMtimeEntries
//
// DiscoverDrivers drops entries whose file mtime is older than the 30 s heartbeat
// TTL (MjDriverDiscovery.cpp:62-72). A fresh owner is discovered; a byte-identical
// owner backdated well past the TTL is pruned.
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
		URLabFastPath::DiscoverDrivers(Drivers, Err));
	TestEqual(TEXT("only the fresh owner survives the mtime TTL"), Drivers.Num(), 1);
	if (Drivers.Num() == 1)
	{
		TestEqual(TEXT("survivor is the fresh entry"),
			Drivers[0].InstanceId, FString(TEXT("fresh_owner")));
	}
	return true;
}

// ---------------------------------------------------------------------------
// URLab.Discovery.PrunesDeadPidEntries     [EXPECTED FAIL — addendum §B]
//
// An entry whose mtime is fresh but whose PID is dead should not be advertised as
// a joinable driver. FMjDriverInfo::Pid is currently write-only in the reader path
// (cleanup_audit_addendum §B: "editor doc claims a dead-pid skip that isn't
// implemented") — DiscoverDrivers performs only the mtime TTL check, so a
// crash-orphaned entry inside the TTL window is still returned.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjDiscoveryPrunesDeadPid,
	"URLab.Discovery.PrunesDeadPidEntries",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjDiscoveryPrunesDeadPid::RunTest(const FString& Parameters)
{
	FScopedRegistryDir Scoped;

	// Fresh mtime, but a PID that cannot correspond to a live process. Because
	// dead-pid pruning is unimplemented the specific value is immaterial.
	constexpr int32 DeadPid = 0x7FFFFFFE;
	WriteRawOwnerEntry(TEXT("dead_owner"), DeadPid, TEXT("tcp://testhost:6300"));

	TArray<FMjDriverInfo> Drivers;
	FString Err;
	TestTrue(TEXT("DiscoverDrivers succeeds"),
		URLabFastPath::DiscoverDrivers(Drivers, Err));

	// EXPECTED FAIL (addendum §B): the dead-pid entry should be pruned; today it is
	// returned because only the mtime TTL is checked.
	TestEqual(TEXT("EXPECTED FAIL (addendum §B): a fresh-mtime entry with a dead PID "
		"must be pruned from discovery"), Drivers.Num(), 0);

	return true;
}
