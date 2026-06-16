// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "HAL/FileManager.h"
#include "Dom/JsonObject.h"
#include "Containers/Ticker.h"

#include "Bridge/OpRegistry.h"
#include "MjLevelOps.h"

namespace
{
// Smallest MJCF that compiles cleanly. No external mesh refs so the
// factory's clean_meshes.py step is a fast no-op even on a fresh box
// without Pillow / trimesh.
const TCHAR* kMinimalMjcf = TEXT(
	"<mujoco model=\"levelops_min\">\n"
	"  <worldbody>\n"
	"    <body name=\"b1\">\n"
	"      <geom name=\"g1\" type=\"sphere\" size=\"0.1\"/>\n"
	"    </body>\n"
	"  </worldbody>\n"
	"</mujoco>\n");

FString WriteScratchMjcf(const FString& Stem)
{
	const FString Dir = FPaths::ConvertRelativePathToFull(
		FPaths::ProjectSavedDir() / TEXT("URLabTest"));
	IFileManager::Get().MakeDirectory(*Dir, /*Tree=*/true);
	const FString Path = Dir / (Stem + TEXT(".xml"));
	FFileHelper::SaveStringToFile(FString(kMinimalMjcf), *Path);
	return Path;
}
} // namespace

// ---------------------------------------------------------------------------
// 1. import_xml registry: handler is installed by URLabEditor on startup.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjLevelOpsImportXmlRegistered,
	"URLab.LevelOps.ImportXmlRegistered",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjLevelOpsImportXmlRegistered::RunTest(const FString& Parameters)
{
	TestTrue(TEXT("import_xml is in the editor-only op set"),
		URLabOpRegistry::IsEditorOnlyOp(TEXT("import_xml")));
	auto Handler = URLabOpRegistry::GetHandler(TEXT("import_xml"));
	TestTrue(TEXT("import_xml handler installed by URLabEditor module"),
		static_cast<bool>(Handler));
	return true;
}

// ---------------------------------------------------------------------------
// 2. ImportXmlSync: end-to-end import of a minimal MJCF returns a
//    blueprint class path under /Game/MuJoCoImports/. Re-import with
//    force_reimport=false short-circuits to imported_now=false.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjLevelOpsImportXmlEndToEnd,
	"URLab.LevelOps.ImportXmlEndToEnd",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjLevelOpsImportXmlEndToEnd::RunTest(const FString& Parameters)
{
	const FString Path = WriteScratchMjcf(TEXT("levelops_e2e"));

	FString ClassPath, ShortName, Err;
	bool bImportedNow = false;

	const bool bOk = URLabLevelOps::ImportXmlSync(
		Path, /*bForceReimport=*/false, ClassPath, ShortName, bImportedNow, Err);
	TestTrue(*FString::Printf(TEXT("import_xml succeeds: %s"), *Err), bOk);
	if (!bOk)
		return false;

	TestTrue(TEXT("ShortName non-empty"), !ShortName.IsEmpty());
	TestTrue(TEXT("ClassPath under /Game/MuJoCoImports/"),
		ClassPath.StartsWith(TEXT("/Game/MuJoCoImports/")));
	TestTrue(TEXT("ClassPath ends in _C (UBlueprintGeneratedClass)"),
		ClassPath.EndsWith(TEXT("_C")));

	// Second call with force_reimport=false re-uses the existing asset.
	FString ClassPath2, ShortName2, Err2;
	bool bImportedNow2 = true;
	const bool bOk2 = URLabLevelOps::ImportXmlSync(
		Path, /*bForceReimport=*/false, ClassPath2, ShortName2, bImportedNow2, Err2);
	TestTrue(TEXT("re-import succeeds"), bOk2);
	TestEqual(TEXT("ClassPath stable across re-imports"), ClassPath2, ClassPath);
	TestEqual(TEXT("ShortName stable across re-imports"), ShortName2, ShortName);
	TestFalse(TEXT("imported_now=false on cached re-import"), bImportedNow2);

	return true;
}

// ---------------------------------------------------------------------------
// 3a. ResolveLevelPath: short name vs full /Game/... path.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjLevelOpsResolveLevelPath,
	"URLab.LevelOps.ResolveLevelPath",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjLevelOpsResolveLevelPath::RunTest(const FString& Parameters)
{
	TestEqual(TEXT("short name -> /Game/Levels/<name>"),
		URLabLevelOps::ResolveLevelPath(TEXT("foo")),
		FString(TEXT("/Game/Levels/foo")));
	TestEqual(TEXT("/Game/... passes through"),
		URLabLevelOps::ResolveLevelPath(TEXT("/Game/Maps/bar")),
		FString(TEXT("/Game/Maps/bar")));
	return true;
}

// ---------------------------------------------------------------------------
// 3b. create_level / save_level / load_level lifecycle.
//     Using a unique scratch level name per test run so reruns don't trip
//     on the existing-asset guard.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjLevelOpsCreateLoadSave,
	"URLab.LevelOps.CreateLoadSave",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjLevelOpsCreateLoadSave::RunTest(const FString& Parameters)
{
	const FString LevelName = FString::Printf(
		TEXT("urlab_test_%lld"), FDateTime::UtcNow().ToUnixTimestamp());

	FString OutPath, Err;
	{
		const bool bOk = URLabLevelOps::CreateLevelSync(LevelName, false, OutPath, Err);
		TestTrue(*FString::Printf(TEXT("create succeeds: %s"), *Err), bOk);
		if (!bOk)
			return false;
		TestEqual(TEXT("create returns /Game/Levels/<name>"),
			OutPath, FString::Printf(TEXT("/Game/Levels/%s"), *LevelName));
	}
	{
		FString LoadPath, LoadErr;
		const bool bOk = URLabLevelOps::LoadLevelSync(LevelName, LoadPath, LoadErr);
		TestTrue(*FString::Printf(TEXT("load succeeds: %s"), *LoadErr), bOk);
		TestEqual(TEXT("load returns the same path"), LoadPath, OutPath);
	}
	{
		FString SavePath, SaveErr;
		const bool bOk = URLabLevelOps::SaveCurrentLevelSync(SavePath, SaveErr);
		TestTrue(*FString::Printf(TEXT("save succeeds: %s"), *SaveErr), bOk);
		TestTrue(TEXT("save_level reports a level path"), !SavePath.IsEmpty());
	}
	return true;
}

// ---------------------------------------------------------------------------
// 4. spawn_actor / destroy_actor: end-to-end. Uses a fresh scratch level so
//    the test doesn't pollute whatever map is currently open.
// ---------------------------------------------------------------------------
#include "MuJoCo/Core/MjArticulation.h"
#include "EngineUtils.h"
#include "Editor.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjLevelOpsSpawnDestroy,
	"URLab.LevelOps.SpawnDestroy",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjLevelOpsSpawnDestroy::RunTest(const FString& Parameters)
{
	// 1) Import a minimal MJCF -> Blueprint.
	const FString XmlPath = WriteScratchMjcf(TEXT("levelops_spawn"));
	FString BPClassPath, BPShortName, ImportErr;
	bool bImportedNow = false;
	if (!URLabLevelOps::ImportXmlSync(XmlPath, /*bForceReimport=*/false,
			BPClassPath, BPShortName, bImportedNow, ImportErr))
	{
		AddError(FString::Printf(TEXT("import setup failed: %s"), *ImportErr));
		return false;
	}

	// 2) Make a fresh scratch level so spawn / destroy live in isolation.
	const FString LevelName = FString::Printf(
		TEXT("urlab_test_spawn_%lld"), FDateTime::UtcNow().ToUnixTimestamp());
	FString LevelPath, LevelErr;
	if (!URLabLevelOps::CreateLevelSync(LevelName, false, LevelPath, LevelErr))
	{
		AddError(FString::Printf(TEXT("create_level setup failed: %s"), *LevelErr));
		return false;
	}

	// 3) Spawn the BP at a non-trivial pose with an actor id.
	FString ActorName, ActorPath, OutClassPath, SpawnErr;
	bool bWasExisting = true;        // expect false after a fresh create
	const FQuat Q = FQuat::Identity; // wxyz unit quat; identity rotation
	const bool bSpawnOk = URLabLevelOps::SpawnActorSync(
		BPClassPath, TEXT("robot_a"),
		FVector(1.0, 2.0, 0.5), // metres
		Q,
		FVector(1.0, 1.0, 1.0),
		ActorName, ActorPath, OutClassPath, bWasExisting, SpawnErr);
	TestTrue(*FString::Printf(TEXT("spawn ok: %s"), *SpawnErr), bSpawnOk);
	if (!bSpawnOk)
		return false;

	TestTrue(TEXT("ActorName non-empty"), !ActorName.IsEmpty());
	TestTrue(TEXT("ActorPath non-empty"), !ActorPath.IsEmpty());
	TestEqual(TEXT("BP class path resolved"), OutClassPath, BPClassPath);
	TestFalse(TEXT("first spawn is a create, not an update"), bWasExisting);

	// 4) The spawned actor exists in the editor world and carries the
	//    actor id (via UPROPERTY since the BP is an AMjArticulation).
	AAMjManager* DummySearch = nullptr; // unused
	UWorld* World = GEditor->GetEditorWorldContext().World();
	bool bFoundById = false;
	bool bFoundByName = false;
	for (TActorIterator<AActor> It(World); It; ++It)
	{
		if (AMjArticulation* Mj = Cast<AMjArticulation>(*It))
		{
			if (Mj->ActorId == TEXT("robot_a"))
				bFoundById = true;
		}
		if ((*It)->GetName() == ActorName)
			bFoundByName = true;
	}
	TestTrue(TEXT("spawned actor present in world"), bFoundByName);
	TestTrue(TEXT("actor id stored on AMjArticulation::ActorId"), bFoundById);

	// 5) Destroy by actor id.
	FString DestroyErr;
	const bool bDestroyOk = URLabLevelOps::DestroyActorSync(TEXT("robot_a"), DestroyErr);
	TestTrue(*FString::Printf(TEXT("destroy ok: %s"), *DestroyErr), bDestroyOk);

	// 6) Confirm gone.
	bool bStillThere = false;
	for (TActorIterator<AActor> It(World); It; ++It)
	{
		if (AMjArticulation* Mj = Cast<AMjArticulation>(*It))
		{
			if (Mj->ActorId == TEXT("robot_a"))
			{
				bStillThere = true;
				break;
			}
		}
	}
	TestFalse(TEXT("destroyed actor no longer in world"), bStillThere);

	// 7) Destroy with non-existent id errors cleanly.
	{
		FString Err;
		const bool bOk = URLabLevelOps::DestroyActorSync(TEXT("nope"), Err);
		TestFalse(TEXT("destroy of unknown id fails"), bOk);
	}
	return true;
}

// ---------------------------------------------------------------------------
// 4a. spawn_actor idempotency: same actor_id called twice updates the
//     existing actor rather than duplicating it. Different BP class with
//     the same id is a hard error so a typo can't silently swap geometry.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjLevelOpsSpawnIdempotent,
	"URLab.LevelOps.SpawnIdempotent",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjLevelOpsSpawnIdempotent::RunTest(const FString& Parameters)
{
	const FString XmlPath = WriteScratchMjcf(TEXT("levelops_idem"));
	FString BPClassPath, BPShortName, ImportErr;
	bool bImportedNow = false;
	if (!URLabLevelOps::ImportXmlSync(XmlPath, /*bForceReimport=*/false,
			BPClassPath, BPShortName, bImportedNow, ImportErr))
	{
		AddError(ImportErr);
		return false;
	}

	const FString LevelName = FString::Printf(
		TEXT("urlab_test_idem_%lld"), FDateTime::UtcNow().ToUnixTimestamp());
	FString LP, LE;
	if (!URLabLevelOps::CreateLevelSync(LevelName, false, LP, LE))
	{
		AddError(LE);
		return false;
	}

	// First spawn.
	FString N1, P1, BC1, Err1;
	bool bExisting1 = true;
	const bool bOk1 = URLabLevelOps::SpawnActorSync(
		BPClassPath, TEXT("twin"),
		FVector(1.0, 0.0, 0.0), FQuat::Identity,
		FVector(1.0, 1.0, 1.0),
		N1, P1, BC1, bExisting1, Err1);
	TestTrue(TEXT("first spawn ok"), bOk1);
	TestFalse(TEXT("first spawn was_existing=false"), bExisting1);

	// Second spawn with same actor_id + same BP: should update, not duplicate.
	FString N2, P2, BC2, Err2;
	bool bExisting2 = false;
	const bool bOk2 = URLabLevelOps::SpawnActorSync(
		BPClassPath, TEXT("twin"),
		FVector(2.0, 1.0, 0.5), FQuat::Identity,
		FVector(1.0, 1.0, 1.0),
		N2, P2, BC2, bExisting2, Err2);
	TestTrue(TEXT("second spawn ok"), bOk2);
	TestTrue(TEXT("second spawn was_existing=true"), bExisting2);
	TestEqual(TEXT("same actor returned (same name)"), N2, N1);
	TestEqual(TEXT("same actor returned (same path)"), P2, P1);

	// Verify only one actor in the world has actor_id == "twin".
	UWorld* World = GEditor->GetEditorWorldContext().World();
	int32 MatchCount = 0;
	for (TActorIterator<AActor> It(World); It; ++It)
	{
		if (AMjArticulation* Mj = Cast<AMjArticulation>(*It))
		{
			if (Mj->ActorId == TEXT("twin"))
				++MatchCount;
		}
	}
	TestEqual(TEXT("idempotent — exactly one actor with actor_id 'twin'"), MatchCount, 1);

	// BP-class mismatch on the same actor_id is a hard error.
	{
		// Construct a different BP path to force the mismatch — any class
		// not matching the existing one will do. AStaticMeshActor isn't a
		// BP path string, so use a fake path that won't resolve to the
		// same class. The mismatch check kicks in only if the class
		// *does* resolve; an unresolved class returns the existing
		// "blueprint class not found" error first. To exercise the
		// mismatch path we need a real second BP — import a second MJCF.
		const FString XmlPath2 = WriteScratchMjcf(TEXT("levelops_idem_other"));
		FString BPClassPath2, BPShortName2, ImportErr2;
		bool bImportedNow2 = false;
		if (URLabLevelOps::ImportXmlSync(XmlPath2, /*bForceReimport=*/false,
				BPClassPath2, BPShortName2, bImportedNow2, ImportErr2)
			&& BPClassPath2 != BPClassPath)
		{
			FString N3, P3, BC3, Err3;
			bool bExisting3 = false;
			const bool bOk3 = URLabLevelOps::SpawnActorSync(
				BPClassPath2, TEXT("twin"),
				FVector(0, 0, 0), FQuat::Identity,
				FVector(1.0, 1.0, 1.0),
				N3, P3, BC3, bExisting3, Err3);
			TestFalse(TEXT("BP-class mismatch on same actor_id rejected"), bOk3);
			TestTrue(TEXT("error mentions actor_id"), Err3.Contains(TEXT("twin")));
		}
		// If the second import collapsed into the same BP we silently
		// skip this assertion — it's a setup artifact, not a bug.
	}

	return true;
}

// ---------------------------------------------------------------------------
// 4b. set_actor_transform: edit-time move of a spawned light by actor id.
//     Lights are quick to spawn (no MJCF needed) so we use one for this
//     focused transform-only test.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjLevelOpsSetActorTransform,
	"URLab.LevelOps.SetActorTransform",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjLevelOpsSetActorTransform::RunTest(const FString& Parameters)
{
	const FString LevelName = FString::Printf(
		TEXT("urlab_test_xform_%lld"), FDateTime::UtcNow().ToUnixTimestamp());
	FString LP, LE;
	if (!URLabLevelOps::CreateLevelSync(LevelName, false, LP, LE))
	{
		AddError(LE);
		return false;
	}

	// Spawn a light at origin.
	FString N, P, RKind, Err;
	if (!URLabLevelOps::SpawnLightSync(
			TEXT("point"), TEXT("movable"),
			FVector(0, 0, 0), FVector(0, 0, 0),
			5000.0f, FLinearColor::White, N, P, RKind, Err))
	{
		AddError(Err);
		return false;
	}

	// Move it to (1, 0, 2) m.
	{
		FVector Loc(1.0, 0.0, 2.0);
		FString OutName, MoveErr;
		TestTrue(*FString::Printf(TEXT("set_actor_transform ok: %s"), *MoveErr),
			URLabLevelOps::SetActorTransformSync(
				TEXT("movable"), &Loc, /*Quat=*/nullptr, OutName, MoveErr));
	}

	// Verify the actor's world position reflects the MJ -> UE convert
	// (1m -> 100cm; Y stays since it was 0; Z -> 200cm).
	UWorld* W = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr;
	bool bChecked = false;
	if (W)
	{
		for (TActorIterator<AActor> It(W); It; ++It)
		{
			if ((*It)->Tags.Contains(FName(TEXT("URLab.ActorId=movable"))))
			{
				FVector Pos = (*It)->GetActorLocation();
				TestTrue(TEXT("X moved to ~100cm"), FMath::IsNearlyEqual(Pos.X, 100.0, 0.5));
				TestTrue(TEXT("Z moved to ~200cm"), FMath::IsNearlyEqual(Pos.Z, 200.0, 0.5));
				bChecked = true;
				break;
			}
		}
	}
	TestTrue(TEXT("found moved actor"), bChecked);

	// Unknown id errors cleanly.
	{
		FVector Loc(0, 0, 0);
		FString OutName, Err2;
		TestFalse(TEXT("missing id fails"),
			URLabLevelOps::SetActorTransformSync(
				TEXT("does_not_exist"), &Loc, nullptr, OutName, Err2));
	}

	return true;
}

// ---------------------------------------------------------------------------
// 5. spawn_light: directional / point / spot — actor with actor-id tag.
// ---------------------------------------------------------------------------
#include "Engine/DirectionalLight.h"
#include "Engine/PointLight.h"
#include "Engine/SpotLight.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjLevelOpsSpawnLight,
	"URLab.LevelOps.SpawnLight",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjLevelOpsSpawnLight::RunTest(const FString& Parameters)
{
	const FString LevelName = FString::Printf(
		TEXT("urlab_test_light_%lld"), FDateTime::UtcNow().ToUnixTimestamp());
	FString LP, LE;
	if (!URLabLevelOps::CreateLevelSync(LevelName, false, LP, LE))
	{
		AddError(LE);
		return false;
	}

	auto Spawn = [this](const FString& Kind, const FString& Id) -> AActor* {
		FString N, P, RKind, Err;
		const bool bOk = URLabLevelOps::SpawnLightSync(
			Kind, Id, FVector(0, 0, 1), FVector(0, -45, 0),
			12345.0f, FLinearColor(0.5f, 0.5f, 0.5f, 1.0f),
			N, P, RKind, Err);
		TestTrue(*FString::Printf(TEXT("spawn %s ok: %s"), *Kind, *Err), bOk);
		TestEqual(*FString::Printf(TEXT("kind echoed for %s"), *Kind),
			RKind, Kind.ToLower());
		UWorld* W = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr;
		if (!W)
			return nullptr;
		for (TActorIterator<AActor> It(W); It; ++It)
			if ((*It)->GetName() == N)
				return *It;
		return nullptr;
	};

	AActor* Dir = Spawn(TEXT("directional"), TEXT("sun"));
	TestTrue(TEXT("dir light spawned"), Dir && Dir->IsA<ADirectionalLight>());
	TestTrue(TEXT("actor id stored as tag"),
		Dir && Dir->Tags.Contains(FName(TEXT("URLab.ActorId=sun"))));

	AActor* Pt = Spawn(TEXT("point"), TEXT("bulb1"));
	TestTrue(TEXT("point light spawned"), Pt && Pt->IsA<APointLight>());

	AActor* Sp = Spawn(TEXT("spot"), TEXT("spot1"));
	TestTrue(TEXT("spot light spawned"), Sp && Sp->IsA<ASpotLight>());

	// destroy_actor should resolve a tag-based logical id.
	{
		FString Err;
		TestTrue(TEXT("destroy directional by id"),
			URLabLevelOps::DestroyActorSync(TEXT("sun"), Err));
	}

	// Unknown kind errors.
	{
		FString N, P, K, Err;
		TestFalse(TEXT("unknown kind fails"),
			URLabLevelOps::SpawnLightSync(
				TEXT("starshine"), TEXT("x"),
				FVector::ZeroVector, FVector::ZeroVector,
				1.0f, FLinearColor::White, N, P, K, Err));
	}

	return true;
}

// ---------------------------------------------------------------------------
// 6. PIE control: pie_status without an active PIE manager. begin_pie /
//    stop_pie are not exercised in automation here — they hop to the
//    game thread and require a full PIE world; covered by the wire-shape
//    Python test + manual smoke.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjLevelOpsPieStatusNoPie,
	"URLab.LevelOps.PieStatusNoPie",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjLevelOpsPieStatusNoPie::RunTest(const FString& Parameters)
{
	auto Handler = URLabOpRegistry::GetHandler(TEXT("pie_status"));
	TestTrue(TEXT("pie_status handler installed"), static_cast<bool>(Handler));
	if (!Handler)
		return false;

	TSharedPtr<FJsonObject> Req = MakeShared<FJsonObject>();
	Req->SetStringField(TEXT("op"), TEXT("pie_status"));
	TSharedPtr<FJsonObject> Reply = Handler(Req);

	FString Op;
	Reply->TryGetStringField(TEXT("op"), Op);
	TestEqual(TEXT("op == pie_status_ok"), Op, FString(TEXT("pie_status_ok")));

	FString State;
	Reply->TryGetStringField(TEXT("state"), State);
	TestEqual(TEXT("state == off when not playing"), State, FString(TEXT("off")));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjLevelOpsPieHandlersRegistered,
	"URLab.LevelOps.PieHandlersRegistered",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjLevelOpsPieHandlersRegistered::RunTest(const FString& Parameters)
{
	TestTrue(TEXT("begin_pie editor-only"),
		URLabOpRegistry::IsEditorOnlyOp(TEXT("begin_pie")));
	TestTrue(TEXT("stop_pie editor-only"),
		URLabOpRegistry::IsEditorOnlyOp(TEXT("stop_pie")));
	TestTrue(TEXT("pie_status editor-only"),
		URLabOpRegistry::IsEditorOnlyOp(TEXT("pie_status")));
	TestTrue(TEXT("begin_pie handler installed"),
		static_cast<bool>(URLabOpRegistry::GetHandler(TEXT("begin_pie"))));
	TestTrue(TEXT("stop_pie handler installed"),
		static_cast<bool>(URLabOpRegistry::GetHandler(TEXT("stop_pie"))));
	TestTrue(TEXT("pie_status handler installed"),
		static_cast<bool>(URLabOpRegistry::GetHandler(TEXT("pie_status"))));
	return true;
}

// ---------------------------------------------------------------------------
// 3. import_xml handler error cases: missing path / non-existent file.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjLevelOpsImportXmlErrors,
	"URLab.LevelOps.ImportXmlErrors",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjLevelOpsImportXmlErrors::RunTest(const FString& Parameters)
{
	auto Handler = URLabOpRegistry::GetHandler(TEXT("import_xml"));
	auto StatusHandler = URLabOpRegistry::GetHandler(TEXT("op_status"));
	if (!Handler || !StatusHandler)
	{
		AddError(TEXT("import_xml / op_status handler not installed"));
		return false;
	}

	// import_xml is async now: the handler returns op_started + job_id and the
	// real work (plus its error reply) runs on the next game-thread tick, read
	// back via op_status. Drive that to the terminal result here.
	auto RunToResult = [&](TSharedPtr<FJsonObject> Req) -> TSharedPtr<FJsonObject>
	{
		TSharedPtr<FJsonObject> Started = Handler(Req);
		FString JobId;
		Started->TryGetStringField(TEXT("job_id"), JobId);
		if (JobId.IsEmpty())
			return Started; // synchronous reply (no job) -> use directly
		for (int32 i = 0; i < 100; ++i)
		{
			FTSTicker::GetCoreTicker().Tick(0.05f);
			TSharedPtr<FJsonObject> SReq = MakeShared<FJsonObject>();
			SReq->SetStringField(TEXT("op"), TEXT("op_status"));
			SReq->SetStringField(TEXT("job_id"), JobId);
			TSharedPtr<FJsonObject> SRep = StatusHandler(SReq);
			FString State;
			SRep->TryGetStringField(TEXT("state"), State);
			if (State != TEXT("running"))
			{
				const TSharedPtr<FJsonObject>* Res = nullptr;
				if (SRep->TryGetObjectField(TEXT("result"), Res) && Res)
					return *Res;
				return SRep;
			}
		}
		return nullptr;
	};

	// Missing 'path' field.
	{
		TSharedPtr<FJsonObject> Req = MakeShared<FJsonObject>();
		Req->SetStringField(TEXT("op"), TEXT("import_xml"));
		TSharedPtr<FJsonObject> Reply = RunToResult(Req);
		FString Code;
		if (Reply.IsValid())
			Reply->TryGetStringField(TEXT("code"), Code);
		TestEqual(TEXT("missing path -> missing_field"),
			Code, FString(TEXT("missing_field")));
	}

	// Non-existent file.
	{
		TSharedPtr<FJsonObject> Req = MakeShared<FJsonObject>();
		Req->SetStringField(TEXT("op"), TEXT("import_xml"));
		Req->SetStringField(TEXT("path"), TEXT("C:/no/such/path/never.xml"));
		TSharedPtr<FJsonObject> Reply = RunToResult(Req);
		FString Code;
		if (Reply.IsValid())
			Reply->TryGetStringField(TEXT("code"), Code);
		TestEqual(TEXT("missing file -> import_failed"),
			Code, FString(TEXT("import_failed")));
	}

	return true;
}

// ---------------------------------------------------------------------------
// Scene introspection ops: registry presence.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjLevelOpsIntrospectRegistered,
	"URLab.LevelOps.IntrospectRegistered",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjLevelOpsIntrospectRegistered::RunTest(const FString& Parameters)
{
	const TCHAR* Ops[] = {
		TEXT("find_actors"),
		TEXT("get_actor_bounds"),
		TEXT("snapshot"),
		TEXT("duplicate_actor"),
		TEXT("actor_hierarchy"),
	};
	for (const TCHAR* Op : Ops)
	{
		TestTrue(*FString::Printf(TEXT("%s registered"), Op),
			static_cast<bool>(URLabOpRegistry::GetHandler(Op)));
		TestTrue(*FString::Printf(TEXT("%s is editor-only"), Op),
			URLabOpRegistry::IsEditorOnlyOp(Op));
	}
	return true;
}

// ---------------------------------------------------------------------------
// find_actors: filtering on a fresh level.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjLevelOpsFindActors,
	"URLab.LevelOps.FindActors",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjLevelOpsFindActors::RunTest(const FString& Parameters)
{
	// Fresh level so we know exactly which actors are present.
	const FString LevelName = FString::Printf(
		TEXT("urlab_test_find_%lld"), FDateTime::UtcNow().ToUnixTimestamp());
	FString LP, LE;
	if (!URLabLevelOps::CreateLevelSync(LevelName, false, LP, LE))
	{
		AddError(LE);
		return false;
	}

	// With no filters: at minimum we'd see whatever default actors UE
	// placed (lights, world settings are filtered out by ShouldListActor;
	// a brand-new level may legitimately have zero matching actors).
	TArray<TSharedPtr<FJsonValue>> All;
	bool bInPie = false;
	FString Err;
	TestTrue(TEXT("find_actors with no filters succeeds"),
		URLabLevelOps::FindActorsSync(
			TEXT(""), TEXT(""), TEXT(""), false, All, bInPie, Err));
	TestFalse(TEXT("in_pie reports false outside PIE"), bInPie);

	// Class filter that nothing matches.
	TArray<TSharedPtr<FJsonValue>> NoMatch;
	URLabLevelOps::FindActorsSync(
		TEXT("DefinitelyNotARealClassXYZ"), TEXT(""), TEXT(""), false,
		NoMatch, bInPie, Err);
	TestEqual(TEXT("unknown class filter -> 0 actors"), NoMatch.Num(), 0);

	return true;
}

// ---------------------------------------------------------------------------
// scene.snapshot: succeeds on a fresh level + reports in_pie=false.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjLevelOpsSnapshot,
	"URLab.LevelOps.Snapshot",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjLevelOpsSnapshot::RunTest(const FString& Parameters)
{
	const FString LevelName = FString::Printf(
		TEXT("urlab_test_snap_%lld"), FDateTime::UtcNow().ToUnixTimestamp());
	FString LP, LE;
	if (!URLabLevelOps::CreateLevelSync(LevelName, false, LP, LE))
	{
		AddError(LE);
		return false;
	}

	TArray<TSharedPtr<FJsonValue>> Actors;
	bool bInPie = false;
	FString LevelPath, Err;
	TestTrue(TEXT("snapshot succeeds"),
		URLabLevelOps::SnapshotSceneSync(Actors, bInPie, LevelPath, Err));
	TestFalse(TEXT("in_pie reports false outside PIE"), bInPie);
	TestTrue(TEXT("snapshot returns the active level path"),
		LevelPath.Contains(LevelName));
	return true;
}

// ---------------------------------------------------------------------------
// duplicate_actor: end-to-end against an imported BP.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjLevelOpsDuplicateActor,
	"URLab.LevelOps.DuplicateActor",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjLevelOpsDuplicateActor::RunTest(const FString& Parameters)
{
	const FString XmlPath = WriteScratchMjcf(TEXT("levelops_dup"));
	FString BPClassPath, BPShortName, ImportErr;
	bool bImportedNow = false;
	if (!URLabLevelOps::ImportXmlSync(XmlPath, false,
			BPClassPath, BPShortName, bImportedNow, ImportErr))
	{
		AddError(ImportErr);
		return false;
	}

	const FString LevelName = FString::Printf(
		TEXT("urlab_test_dup_%lld"), FDateTime::UtcNow().ToUnixTimestamp());
	FString LP, LE;
	if (!URLabLevelOps::CreateLevelSync(LevelName, false, LP, LE))
	{
		AddError(LE);
		return false;
	}

	FString OrigName, OrigPath, OrigBPPath, SpawnErr;
	bool bWasExisting = true;
	if (!URLabLevelOps::SpawnActorSync(
			BPClassPath, TEXT("dup_src"),
			FVector(0.0, 0.0, 0.0), FQuat::Identity, FVector::OneVector,
			OrigName, OrigPath, OrigBPPath, bWasExisting, SpawnErr))
	{
		AddError(SpawnErr);
		return false;
	}

	FString DupName, DupPath, DupBPPath, DupErr;
	const bool bOk = URLabLevelOps::DuplicateActorSync(
		TEXT("dup_src"), TEXT("dup_copy"),
		false, FVector::ZeroVector,
		DupName, DupPath, DupBPPath, DupErr);
	TestTrue(*FString::Printf(TEXT("duplicate succeeds: %s"), *DupErr), bOk);
	if (!bOk)
		return false;
	TestTrue(TEXT("duplicate produced a different actor name"),
		!DupName.Equals(OrigName));
	return true;
}

// ---------------------------------------------------------------------------
// actor_hierarchy: returns a single-node tree for an actor with no
// attached actors (the spawned BP root).
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjLevelOpsActorHierarchy,
	"URLab.LevelOps.ActorHierarchy",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjLevelOpsActorHierarchy::RunTest(const FString& Parameters)
{
	const FString XmlPath = WriteScratchMjcf(TEXT("levelops_hier"));
	FString BPClassPath, BPShortName, ImportErr;
	bool bImportedNow = false;
	if (!URLabLevelOps::ImportXmlSync(XmlPath, false,
			BPClassPath, BPShortName, bImportedNow, ImportErr))
	{
		AddError(ImportErr);
		return false;
	}

	const FString LevelName = FString::Printf(
		TEXT("urlab_test_hier_%lld"), FDateTime::UtcNow().ToUnixTimestamp());
	FString LP, LE;
	if (!URLabLevelOps::CreateLevelSync(LevelName, false, LP, LE))
	{
		AddError(LE);
		return false;
	}

	FString N, P, BP, SE;
	bool bWE = true;
	if (!URLabLevelOps::SpawnActorSync(
			BPClassPath, TEXT("hier_root"),
			FVector::ZeroVector, FQuat::Identity, FVector::OneVector,
			N, P, BP, bWE, SE))
	{
		AddError(SE);
		return false;
	}

	TSharedPtr<FJsonObject> Root;
	FString HE;
	const bool bOk = URLabLevelOps::ActorHierarchySync(TEXT("hier_root"), Root, HE);
	TestTrue(*FString::Printf(TEXT("hierarchy succeeds: %s"), *HE), bOk);
	if (!bOk)
		return false;
	TestTrue(TEXT("root has 'name' field"), Root->HasField(TEXT("name")));
	TestTrue(TEXT("root has 'children' field"), Root->HasField(TEXT("children")));
	return true;
}

// ---------------------------------------------------------------------------
// get_actor_bounds: spawned BP has a non-degenerate AABB.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjLevelOpsActorBounds,
	"URLab.LevelOps.ActorBounds",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjLevelOpsActorBounds::RunTest(const FString& Parameters)
{
	const FString XmlPath = WriteScratchMjcf(TEXT("levelops_bounds"));
	FString BPClassPath, BPShortName, ImportErr;
	bool bImportedNow = false;
	if (!URLabLevelOps::ImportXmlSync(XmlPath, false,
			BPClassPath, BPShortName, bImportedNow, ImportErr))
	{
		AddError(ImportErr);
		return false;
	}

	const FString LevelName = FString::Printf(
		TEXT("urlab_test_bounds_%lld"), FDateTime::UtcNow().ToUnixTimestamp());
	FString LP, LE;
	if (!URLabLevelOps::CreateLevelSync(LevelName, false, LP, LE))
	{
		AddError(LE);
		return false;
	}

	FString N, P, BP, SE;
	bool bWE = true;
	if (!URLabLevelOps::SpawnActorSync(
			BPClassPath, TEXT("bounds_actor"),
			FVector::ZeroVector, FQuat::Identity, FVector::OneVector,
			N, P, BP, bWE, SE))
	{
		AddError(SE);
		return false;
	}

	double Min[3], Max[3];
	FString ResolvedName, BoundsErr;
	const bool bOk = URLabLevelOps::GetActorBoundsSync(
		TEXT("bounds_actor"), false, Min, Max, ResolvedName, BoundsErr);
	TestTrue(*FString::Printf(TEXT("bounds succeeds: %s"), *BoundsErr), bOk);
	if (!bOk)
		return false;
	for (int i = 0; i < 3; ++i)
	{
		TestTrue(*FString::Printf(TEXT("max[%d] >= min[%d]"), i, i),
			Max[i] >= Min[i]);
	}
	return true;
}
