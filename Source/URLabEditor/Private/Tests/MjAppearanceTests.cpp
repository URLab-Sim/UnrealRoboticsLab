// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
//
// Visual domain-randomization store tests. The visual result needs a live GPU, so
// these assert the plumbing -- that the store holds an override, resolves a geom
// name to a found/missing verdict, and applies without crashing -- not pixels.

#include "Misc/AutomationTest.h"
#include "Misc/Paths.h"
#include "Editor.h"
#include "Engine/World.h"
#include "UObject/Package.h"

#include "MuJoCo/Entity/MjAppearanceStore.h"
#include "MuJoCo/Entity/MjGeomAppearance.h"
#include "MuJoCo/Spec/MjAssetResolve.h"
#include "MuJoCo/Fast/MjbScene.h"

#if WITH_EDITOR

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjAppearanceStoreHoldsOverride,
	"URLab.Appearance.StoreHoldsOverride",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjAppearanceStoreHoldsOverride::RunTest(const FString& Parameters)
{
	UMjAppearanceStore* Store = NewObject<UMjAppearanceStore>(GetTransientPackage());
	if (!TestNotNull(TEXT("appearance store"), Store))
	{
		return false;
	}
	Store->Init(nullptr);

	const FName GeomName(TEXT("can_body"));

	FMjGeomAppearance Override;
	Override.BaseColor = FLinearColor(0.2f, 0.6f, 0.9f, 1.0f);
	Override.Metallic = 0.75f;
	Override.TextureBindings.Add(EMjMaterialRole::Rgb, FName(TEXT("sprite_can")));

	// No world is bound, so nothing live is driven -- but the override is still
	// stored, which is the point.
	const int32 Applied = Store->SetOverride(GeomName, Override);
	TestEqual(TEXT("no live components driven with no world"), Applied, 0);
	TestEqual(TEXT("one override held"), Store->Num(), 1);
	TestTrue(TEXT("override present by name"), Store->HasOverride(GeomName));

	const FMjGeomAppearance* Held = Store->FindOverride(GeomName);
	if (!TestNotNull(TEXT("override read back"), Held))
	{
		return false;
	}
	TestTrue(TEXT("base colour set"), Held->BaseColor.IsSet());
	TestEqual(TEXT("base colour green channel"), Held->BaseColor.GetValue().G, 0.6f);
	TestTrue(TEXT("metallic set"), Held->Metallic.IsSet());
	TestEqual(TEXT("metallic value"), Held->Metallic.GetValue(), 0.75f);
	TestFalse(TEXT("roughness left for the base pass"), Held->Roughness.IsSet());
	TestTrue(TEXT("texture binding held"), Held->TextureBindings.Contains(EMjMaterialRole::Rgb));

	// Resolution of a name with nothing live is a clean miss, not a crash.
	const UMjAppearanceStore::FResolution Res = Store->ResolveGeom(GeomName);
	TestFalse(TEXT("nothing live resolves as not found"), Res.IsFound());
	TestEqual(TEXT("no authoring components"), Res.AuthoringComponents, 0);
	TestEqual(TEXT("no fastpath components"), Res.FastpathComponents, 0);

	// Clearing a held override succeeds; clearing an absent one reports -1.
	TestTrue(TEXT("clearing a held override returns >= 0"), Store->ClearOverride(GeomName) >= 0);
	TestFalse(TEXT("override gone after clear"), Store->HasOverride(GeomName));
	TestEqual(TEXT("clearing an absent override reports -1"),
		Store->ClearOverride(FName(TEXT("never_set"))), -1);

	return !HasAnyErrors();
}

// Fast-path apply walk. Needs a version-matched MJB fixture (shared with the
// MjbScene tests); skips gracefully when absent so CI stays green. Exercises the
// store's world walk into AMjbScene::ApplyAppearanceOverride / NumGeomsNamed and
// asserts the found-vs-missing plumbing on a real built scene.
static const TCHAR* kPrimitivesMjb =
	TEXT("/home/buzz/Documents/urlab_debug/mjb_test/primitives.mjb");

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjAppearanceStoreWalksFastPath,
	"URLab.Appearance.StoreWalksFastPath",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjAppearanceStoreWalksFastPath::RunTest(const FString& Parameters)
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
	Scene->LoadAndBuild();

	// Outer the store to the scene so its world (and the built AMjbScene) is walked.
	UMjAppearanceStore* Store = NewObject<UMjAppearanceStore>(Scene);
	Store->Init(nullptr);

	const FName Missing(TEXT("no_such_geom"));
	const UMjAppearanceStore::FResolution Res = Store->ResolveGeom(Missing);
	TestEqual(TEXT("a missing geom finds no fastpath components"), Res.FastpathComponents, 0);

	FMjGeomAppearance Override;
	Override.BaseColor = FLinearColor::Red;
	// Applying to a missing geom drives nothing but must not crash, and the override
	// is still stored for a later rebuild.
	const int32 Applied = Store->SetOverride(Missing, Override);
	TestEqual(TEXT("missing geom drives nothing"), Applied, 0);
	TestTrue(TEXT("override still stored"), Store->HasOverride(Missing));

	Scene->Destroy();
	return !HasAnyErrors();
}

#endif // WITH_EDITOR
