// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
//
// Fast-path (MJB) scene builder tests. Verifies that a version-matched MJB
// loads and builds the expected body actors + geom components without physics.

#include "Misc/AutomationTest.h"
#include "Misc/Paths.h"
#include "Editor.h"
#include "Engine/World.h"

#include "MuJoCo/Fast/MjbScene.h"

#if WITH_EDITOR

// A generated primitives fixture ships beside the repo (see mjb_test/). The test
// is skipped gracefully when it is absent, so CI without the fixture stays green.
static const TCHAR* kPrimitivesMjb =
	TEXT("/home/buzz/Documents/urlab_debug/mjb_test/primitives.mjb");

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjbSceneBuildsFromMjb,
	"URLab.Fast.MjbSceneBuildsFromMjb",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjbSceneBuildsFromMjb::RunTest(const FString& Parameters)
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

	AMjbScene* Scene = World->SpawnActor<AMjbScene>();
	if (!TestNotNull(TEXT("spawned AMjbScene"), Scene))
	{
		return false;
	}
	Scene->bTestSweep = false;
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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjbSceneBuildsMeshModel,
	"URLab.Fast.MjbSceneBuildsMeshModel",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjbSceneBuildsMeshModel::RunTest(const FString& Parameters)
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
	AMjbScene* Scene = World->SpawnActor<AMjbScene>();
	if (!TestNotNull(TEXT("spawned AMjbScene"), Scene))
	{
		return false;
	}
	Scene->bTestSweep = false;
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

#endif // WITH_EDITOR
