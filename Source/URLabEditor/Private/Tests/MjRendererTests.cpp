// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
//
// Fast-path (MJB) scene builder tests. Verifies that a version-matched MJB
// loads and builds the expected body actors + geom components without physics.

#include "Misc/AutomationTest.h"
#include "Misc/Paths.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "Editor.h"
#include "Engine/World.h"

#include "MuJoCo/Fast/MjRenderer.h"

#if WITH_EDITOR

// A generated primitives fixture ships beside the repo (see mjb_test/). The test
// is skipped gracefully when it is absent, so CI without the fixture stays green.
static const TCHAR* kPrimitivesMjb =
	TEXT("/home/buzz/Documents/urlab_debug/mjb_test/primitives.mjb");

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRendererBuildsFromMjb,
	"URLab.Fast.MjRendererBuildsFromMjb",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjRendererBuildsFromMjb::RunTest(const FString& Parameters)
{
	if (!FPaths::FileExists(kPrimitivesMjb))
	{
		AddInfo(FString::Printf(TEXT("fixture %s absent; skipping"), kPrimitivesMjb));
		return true;
	}
	UWorld* World = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr;
	if (!TestNotNull(TEXT("editor world"), World))
	{
		return false;
	}

	AMjRenderer* Scene = World->SpawnActor<AMjRenderer>();
	if (!TestNotNull(TEXT("spawned AMjRenderer"), Scene))
	{
		return false;
	}
	Scene->MjbFilePath = kPrimitivesMjb;

	const int32 NGeom = Scene->LoadAndBuild();

	// primitives.mjb: 7 geoms (floor + box/sphere/capsule/cylinder/ellipsoid +
	// one hidden rgba=0 box), 2 bodies (world + rig).
	TestEqual(TEXT("ngeom reported by the loaded MJB"), NGeom, 7);
	TestEqual(TEXT("one body actor per MuJoCo body (world + rig)"), Scene->NumBodyActors(), 2);
	TestEqual(TEXT("built geoms skip the hidden rgba=0 proxy"), Scene->NumBuiltGeoms(), 6);

	Scene->Destroy();
	return !HasAnyErrors();
}

static const TCHAR* kPandaMjb =
	TEXT("/home/buzz/Documents/urlab_debug/mjb_test/panda.mjb");

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRendererBuildsMeshModel,
	"URLab.Fast.MjRendererBuildsMeshModel",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjRendererBuildsMeshModel::RunTest(const FString& Parameters)
{
	if (!FPaths::FileExists(kPandaMjb))
	{
		AddInfo(FString::Printf(TEXT("fixture %s absent; skipping"), kPandaMjb));
		return true;
	}
	UWorld* World = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr;
	if (!TestNotNull(TEXT("editor world"), World))
	{
		return false;
	}
	AMjRenderer* Scene = World->SpawnActor<AMjRenderer>();
	if (!TestNotNull(TEXT("spawned AMjRenderer"), Scene))
	{
		return false;
	}
	Scene->MjbFilePath = kPandaMjb;

	const int32 NGeom = Scene->LoadAndBuild();

	// panda.mjb: 12 bodies, 81 geoms (many are triangle meshes). We don't pin the
	// exact built count (some are transparent collision proxies), only that the
	// mesh path builds a substantial scene without error.
	TestEqual(TEXT("panda ngeom"), NGeom, 81);
	TestEqual(TEXT("one actor per body"), Scene->NumBodyActors(), 12);
	TestTrue(TEXT("mesh geoms built (>30 comps)"), Scene->NumBuiltGeoms() > 30);

	Scene->Destroy();
	return !HasAnyErrors();
}

// Live integration: a running run_fastpath_demo.py broadcasts per-geom
// transforms on tcp://127.0.0.1:5561 and writes /tmp/urlab_fastpath.mjb. This
// spawns the fast-path scene from that MJB, connects to the bus, and confirms a
// frame is received. Skips (informational) when no broadcaster is present, so
// it never blocks CI; run the demo script first to exercise it for real.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRendererReceivesBus,
	"URLab.Fast.MjRendererReceivesBus",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjRendererReceivesBus::RunTest(const FString& Parameters)
{
	const TCHAR* DemoMjb = TEXT("/tmp/urlab_fastpath.mjb");
	const TCHAR* Bus = TEXT("tcp://127.0.0.1:5561");
	if (!FPaths::FileExists(DemoMjb))
	{
		AddInfo(TEXT("no demo MJB (/tmp/urlab_fastpath.mjb); run run_fastpath_demo.py first — skipping"));
		return true;
	}
	UWorld* World = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr;
	if (!TestNotNull(TEXT("editor world"), World))
	{
		return false;
	}
	AMjRenderer* Scene = World->SpawnActor<AMjRenderer>();
	if (!TestNotNull(TEXT("spawned AMjRenderer"), Scene))
	{
		return false;
	}
	Scene->MjbFilePath = DemoMjb;
	const int32 NGeom = Scene->LoadAndBuild();
	Scene->BusEndpoint = Bus;
	Scene->ConnectBus();

	const double Start = FPlatformTime::Seconds();
	while (!Scene->HasReceivedFrame() && (FPlatformTime::Seconds() - Start) < 5.0)
	{
		FPlatformProcess::Sleep(0.05f);
	}
	// The MJB fixture exists, so a broadcaster is expected: a pass means the
	// fast-path scene loaded the model, connected to the bus, and applied a
	// live per-geom transform frame end to end.
	TestTrue(FString::Printf(TEXT("built %d geoms from the demo MJB"), NGeom), NGeom > 0);
	TestTrue(TEXT("connected to the bus and received a live transform frame"),
		Scene->HasReceivedFrame());

	Scene->Destroy();
	return !HasAnyErrors();
}

#endif // WITH_EDITOR
