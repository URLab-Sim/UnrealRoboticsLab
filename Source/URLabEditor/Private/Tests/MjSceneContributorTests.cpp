// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// Unreal content becoming MJCF: the inverse of import, held to the same bar.
//
// A heightfield samples the level; a quick-convert component turns an actor's
// meshes into collision geometry. Both author a spec that the scene attaches as
// an ordinary participant, which means the thing to assert is not that they ran
// but that what they authored reached the compiled model with the right numbers
// in it.
//
// So each assertion here reads mjModel, not the spec. The elevation is
// checked against `hfield_data` cell by cell, because the row order is the one
// thing about a heightfield that is silently wrong rather than loudly wrong: a
// flipped field still compiles, still collides, and still looks like terrain.
// The converted mesh is checked against the compiled geom's bounding radius and
// world pose, and then checked again after a recompile, because the hull it
// names lives in a generated file and the question a recompile asks is whether
// those bytes are still there.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Components/BoxComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/Engine.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "HAL/FileManager.h"
#include "PhysicsEngine/BodySetup.h"

#include "MuJoCo/Convert/AMjHeightfieldActor.h"
#include "MuJoCo/Convert/MjQuickConvertComponent.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Spec/MjNodeComponent.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace MjSceneContributorTests
{

/** A throwaway world with a manager in it, torn down on scope exit. */
struct FContributorScene
{
	UWorld* World = nullptr;
	AAMjManager* Manager = nullptr;
	FString LastError;

	bool Open()
	{
		World = UWorld::CreateWorld(EWorldType::Game, false);
		if (World == nullptr)
		{
			LastError = TEXT("CreateWorld failed");
			return false;
		}
		FWorldContext& Context = GEngine->CreateNewWorldContext(EWorldType::Game);
		Context.SetCurrentWorld(World);

		FActorSpawnParameters Params;
		Manager = World->SpawnActor<AAMjManager>(Params);
		if (Manager == nullptr)
		{
			LastError = TEXT("SpawnActor AAMjManager failed");
			return false;
		}
		return true;
	}

	bool Install() { return Manager->PhysicsEngine->InstallCompiledSpec(LastError); }

	mjModel* Model() const { return Manager != nullptr ? Manager->PhysicsEngine->GetModel() : nullptr; }
	mjData* Data() const { return Manager != nullptr ? Manager->PhysicsEngine->GetData() : nullptr; }

	/**
	 * A blocking slab, for the height rays to find.
	 *
	 * A plain actor rather than anything MuJoCo knows about: the sampler filters
	 * out articulations, heightfields and converted actors so that a simulated
	 * body can never become terrain, and a slab that got filtered would leave
	 * every ray missing and the test asserting on flat ground.
	 */
	AActor* AddSlab(const FVector& Centre, const FVector& Extent)
	{
		FActorSpawnParameters Params;
		AActor* Actor = World->SpawnActor<AActor>(Params);
		if (Actor == nullptr)
		{
			return nullptr;
		}
		UBoxComponent* Box = NewObject<UBoxComponent>(Actor);
		Actor->SetRootComponent(Box);
		Box->SetBoxExtent(Extent);
		Box->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
		Box->SetCollisionObjectType(ECC_WorldStatic);
		Box->SetCollisionResponseToAllChannels(ECR_Block);
		Box->RegisterComponent();
		Actor->SetActorLocation(Centre);
		return Actor;
	}

	~FContributorScene()
	{
		if (Manager != nullptr)
		{
			Manager->PhysicsEngine->bShouldStopTask = true;
		}
		if (World != nullptr)
		{
			World->DestroyWorld(false);
			GEngine->DestroyWorldContext(World);
		}
	}
};

/** The engine's unit cube: 100 uu on a side, so half a metre either way. */
UStaticMesh* UnitCube()
{
	return LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
}
} // namespace MjSceneContributorTests

// ---------------------------------------------------------------------------
// Heightfield
// ---------------------------------------------------------------------------

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjHeightfieldReachesModelTest,
	"URLab.Convert.HeightfieldReachesTheCompiledModel",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjHeightfieldReachesModelTest::RunTest(const FString& Parameters)
{
	using namespace MjSceneContributorTests;

	FContributorScene Scene;
	if (!TestTrue(TEXT("world opens"), Scene.Open()))
	{
		AddError(Scene.LastError);
		return false;
	}

	// Three slabs, so the sampled field varies along both axes: everything sits
	// at -400, the half at X > 100 rises to -350, and the half at Y > 100 rises
	// to -300 over the top of it. A field that came back flipped or transposed
	// disagrees with this in a way a single ramp would hide.
	Scene.AddSlab(FVector(0, 0, -450), FVector(1200, 1200, 50));
	Scene.AddSlab(FVector(650, 0, -400), FVector(550, 1200, 50));
	Scene.AddSlab(FVector(0, 650, -375), FVector(1200, 550, 75));

	FActorSpawnParameters Params;
	AMjHeightfieldActor* Terrain = Scene.World->SpawnActor<AMjHeightfieldActor>(Params);
	if (!TestNotNull(TEXT("heightfield actor spawns"), Terrain))
	{
		return false;
	}
	constexpr int32 Resolution = 8;
	Terrain->Resolution = Resolution;
	Terrain->BaseThickness = 0.1f;
	Terrain->HFieldName = TEXT("terrain");
	// The disk cache is keyed on bounds, resolution and base thickness, none of
	// which name the world; a previous run's samples would otherwise answer for
	// this one's slabs.
	Terrain->bForceRecache = true;

	if (!TestTrue(TEXT("the scene compiles"), Scene.Install()))
	{
		AddError(Scene.LastError);
		return false;
	}

	mjModel* Model = Scene.Model();
	if (!TestNotNull(TEXT("model exists"), Model))
	{
		return false;
	}

	TestEqual(TEXT("one heightfield in the model"), (int32)Model->nhfield, 1);
	if (Model->nhfield != 1)
	{
		return false;
	}
	TestEqual(TEXT("hfield rows"), Model->hfield_nrow[0], Resolution);
	TestEqual(TEXT("hfield columns"), Model->hfield_ncol[0], Resolution);

	// The box is 1000 uu half-extent each way, so the sampled region is 20 m
	// across and the hfield's half sizes are 10 m. The elevation range is the
	// 100 uu between the lowest and highest sample, which is 1 m, and the base
	// is the configured tenth of it.
	TestEqual(TEXT("hfield half size X"), (double)Model->hfield_size[0], 10.0, 1e-6);
	TestEqual(TEXT("hfield half size Y"), (double)Model->hfield_size[1], 10.0, 1e-6);
	TestEqual(TEXT("hfield elevation range"), (double)Model->hfield_size[2], 1.0, 1e-6);
	TestEqual(TEXT("hfield base thickness"), (double)Model->hfield_size[3], 0.1, 1e-6);

	const FString Prefix = Terrain->GetScenePrefix();
	const int32 NamedGeomId = mj_name2id(Model, mjOBJ_GEOM,
		TCHAR_TO_UTF8(*(Prefix + TEXT("terrain_geom"))));
	TestTrue(TEXT("the hfield geom compiled under the actor's prefix"), NamedGeomId >= 0);
	if (NamedGeomId < 0)
	{
		return false;
	}
	TestEqual(TEXT("the geom is a heightfield"), (int32)Model->geom_type[NamedGeomId],
		(int32)mjGEOM_HFIELD);
	TestEqual(TEXT("the geom points at the compiled hfield"), Model->geom_dataid[NamedGeomId], 0);

	// The samples are taken in world space, so the participant attaches at
	// identity and the geom sits at the centre of the region at the lowest
	// height measured: -400 uu is -4 m, and MuJoCo's Y is Unreal's negated.
	TestEqual(TEXT("geom X"), (double)Model->geom_pos[3 * NamedGeomId + 0], 0.0, 1e-6);
	TestEqual(TEXT("geom Y"), (double)Model->geom_pos[3 * NamedGeomId + 1], 0.0, 1e-6);
	TestEqual(TEXT("geom Z"), (double)Model->geom_pos[3 * NamedGeomId + 2], -4.0, 1e-6);

	TestTrue(TEXT("the element bound to the compiled hfield"),
		Terrain->GetHfieldElement() != nullptr
			&& Terrain->GetHfieldElement()->GetBoundId().Get(-1) == 0);

	// Storage row 0 is minimum MuJoCo Y, which is maximum Unreal Y, so the
	// slabs' relief reads back top row first. Column order is unchanged by the
	// crossing: column 0 is minimum X in both frames.
	const float* Elevation = Model->hfield_data + Model->hfield_adr[0];
	bool bElevationMatches = true;
	for (int32 Row = 0; Row < Resolution; ++Row)
	{
		// Storage rows run from maximum Unreal Y down to minimum.
		const double UnrealY = 1000.0 - Row * (2000.0 / (Resolution - 1));
		for (int32 Col = 0; Col < Resolution; ++Col)
		{
			const double UnrealX = -1000.0 + Col * (2000.0 / (Resolution - 1));
			const double Expected = UnrealY > 100.0 ? 1.0 : (UnrealX > 100.0 ? 0.5 : 0.0);
			const float Actual = Elevation[Row * Resolution + Col];
			if (FMath::Abs(Actual - Expected) > 1e-4)
			{
				AddError(FString::Printf(
					TEXT("elevation[%d][%d] (Unreal X=%.0f Y=%.0f) is %.4f, expected %.4f"), Row, Col,
					UnrealX, UnrealY, Actual, Expected));
				bElevationMatches = false;
			}
		}
	}
	TestTrue(TEXT("the sampled relief reached the model in the right order"), bElevationMatches);

	// Taken by value: installing the next compile deletes this model, and the
	// comparison below is against what this one held.
	const TArray<float> FirstElevation(Elevation, Resolution * Resolution);

	// A recompile re-authors the spec from the same actor. The samples come
	// off the disk cache the first pass wrote, which is the only place they live
	// between compiles.
	Terrain->bForceRecache = false;
	if (!TestTrue(TEXT("the scene recompiles"), Scene.Install()))
	{
		AddError(Scene.LastError);
		return false;
	}
	mjModel* Recompiled = Scene.Model();
	TestEqual(TEXT("still one heightfield after a recompile"), (int32)Recompiled->nhfield, 1);
	TestTrue(TEXT("still one hfield geom after a recompile"),
		mj_name2id(Recompiled, mjOBJ_GEOM, TCHAR_TO_UTF8(*(Prefix + TEXT("terrain_geom")))) >= 0);
	TestEqual(TEXT("the elevation range survived the recompile"), (double)Recompiled->hfield_size[2], 1.0,
		1e-6);

	const float* Again = Recompiled->hfield_data + Recompiled->hfield_adr[0];
	bool bStable = true;
	for (int32 Cell = 0; Cell < Resolution * Resolution; ++Cell)
	{
		bStable = bStable && FMath::Abs(Again[Cell] - FirstElevation[Cell]) < 1e-6;
	}
	TestTrue(TEXT("the cached samples recompiled to the same field"), bStable);

	return true;
}

// ---------------------------------------------------------------------------
// Quick convert
// ---------------------------------------------------------------------------

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjQuickConvertBecomesGeomTest,
	"URLab.Convert.QuickConvertBecomesAGeom",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjQuickConvertBecomesGeomTest::RunTest(const FString& Parameters)
{
	using namespace MjSceneContributorTests;

	UStaticMesh* Cube = UnitCube();
	if (!TestNotNull(TEXT("the engine's unit cube loads"), Cube))
	{
		return false;
	}

	FContributorScene Scene;
	if (!TestTrue(TEXT("world opens"), Scene.Open()))
	{
		AddError(Scene.LastError);
		return false;
	}

	FActorSpawnParameters Params;
	AActor* Prop = Scene.World->SpawnActor<AActor>(Params);
	if (!TestNotNull(TEXT("prop actor spawns"), Prop))
	{
		return false;
	}
	UStaticMeshComponent* Smc = NewObject<UStaticMeshComponent>(Prop);
	Prop->SetRootComponent(Smc);
	Smc->SetStaticMesh(Cube);
	Smc->RegisterComponent();
	Prop->SetActorLocation(FVector(300.0, 400.0, 500.0));

	UMjQuickConvertComponent* Convert = NewObject<UMjQuickConvertComponent>(Prop);
	// Static, so the geom's compiled pose is the actor's placement and nothing
	// else: a free joint would put the body wherever the first forward left it.
	Convert->Static = true;
	Convert->friction = FVector3d(1.2, 0.006, 0.0002);
	Convert->RegisterComponent();
	Prop->AddInstanceComponent(Convert);

	if (!TestTrue(TEXT("the scene compiles"), Scene.Install()))
	{
		AddError(Scene.LastError);
		return false;
	}

	mjModel* Model = Scene.Model();
	if (!TestNotNull(TEXT("model exists"), Model))
	{
		return false;
	}

	const FString Prefix = Convert->GetScenePrefix();
	const int32 BodyId = mj_name2id(Model, mjOBJ_BODY, TCHAR_TO_UTF8(*(Prefix + TEXT("body"))));
	TestTrue(TEXT("the converted body compiled under the actor's prefix"), BodyId > 0);
	if (BodyId <= 0)
	{
		return false;
	}
	TestEqual(TEXT("the component reports the compiled body id"), Convert->GetMjBodyId(), BodyId);
	TestEqual(TEXT("the body carries exactly one geom"), (int32)Model->body_geomnum[BodyId], 1);
	if (Model->body_geomnum[BodyId] != 1)
	{
		return false;
	}
	TestEqual(TEXT("a static conversion adds no degrees of freedom"), (int32)Model->nq, 0);

	const int32 GeomId = Model->body_geomadr[BodyId];
	TestEqual(TEXT("the geom is a mesh"), (int32)Model->geom_type[GeomId], (int32)mjGEOM_MESH);
	TestTrue(TEXT("the mesh asset compiled"), Model->nmesh >= 1);
	TestTrue(TEXT("the geom names a mesh"), Model->geom_dataid[GeomId] >= 0);

	// The engine cube is 100 uu on a side, so half a metre either way and a
	// bounding radius of the half diagonal. This is what proves the OBJ that was
	// written carries the shape and the units, rather than merely existing.
	TestEqual(TEXT("the geom has the cube's bounding radius"), (double)Model->geom_rbound[GeomId],
		FMath::Sqrt(3.0) * 0.5, 1e-3);

	TestEqual(TEXT("friction reached the compiled geom"), (double)Model->geom_friction[3 * GeomId + 0], 1.2,
		1e-9);
	TestEqual(TEXT("torsional friction reached the compiled geom"),
		(double)Model->geom_friction[3 * GeomId + 1], 0.006, 1e-9);

	mjData* Data = Scene.Data();
	mj_forward(Model, Data);
	// Unreal (300, 400, 500) cm is MuJoCo (3, -4, 5) m.
	TestEqual(TEXT("geom world X"), Data->geom_xpos[3 * GeomId + 0], 3.0, 1e-6);
	TestEqual(TEXT("geom world Y"), Data->geom_xpos[3 * GeomId + 1], -4.0, 1e-6);
	TestEqual(TEXT("geom world Z"), Data->geom_xpos[3 * GeomId + 2], 5.0, 1e-6);

	TestEqual(TEXT("the debug id map names the compiled geom"), Convert->m_geomName2ID.Num(), 1);
	for (const TPair<FString, int>& Entry : Convert->m_geomName2ID)
	{
		TestEqual(TEXT("the mapped id is the compiled one"), Entry.Value, GeomId);
	}

	return true;
}

// ---------------------------------------------------------------------------
// What the conversion settings mean in the compiled model
// ---------------------------------------------------------------------------

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjQuickConvertMobilityTest, "URLab.Convert.QuickConvertMobility",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjQuickConvertMobilityTest::RunTest(const FString& Parameters)
{
	using namespace MjSceneContributorTests;

	UStaticMesh* Cube = UnitCube();
	if (!TestNotNull(TEXT("the engine's unit cube loads"), Cube))
	{
		return false;
	}

	// Three settings, three worlds, because the three answers are structural:
	// a free joint, a mocap body, and neither.
	struct FCase
	{
		const TCHAR* Name;
		bool bStatic;
		bool bDrivenByUnreal;
		int32 ExpectedNq;
		int32 ExpectedMocap;
	};
	const FCase Cases[] = {
		{     TEXT("the default conversion is free to move"), false, false, 7, 0},
		{ TEXT("a static conversion is welded to the world"),  true, false, 0, 0},
		{TEXT("an Unreal-driven conversion is a mocap body"), false,  true, 0, 1},
	};

	for (const FCase& Case : Cases)
	{
		FContributorScene Scene;
		if (!TestTrue(TEXT("world opens"), Scene.Open()))
		{
			AddError(Scene.LastError);
			return false;
		}

		FActorSpawnParameters Params;
		AActor* Prop = Scene.World->SpawnActor<AActor>(Params);
		UStaticMeshComponent* Smc = NewObject<UStaticMeshComponent>(Prop);
		Prop->SetRootComponent(Smc);
		Smc->SetStaticMesh(Cube);
		Smc->RegisterComponent();

		UMjQuickConvertComponent* Convert = NewObject<UMjQuickConvertComponent>(Prop);
		Convert->Static = Case.bStatic;
		Convert->bDrivenByUnreal = Case.bDrivenByUnreal;
		Convert->RegisterComponent();
		Prop->AddInstanceComponent(Convert);

		if (!TestTrue(TEXT("the scene compiles"), Scene.Install()))
		{
			AddError(Scene.LastError);
			return false;
		}
		TestEqual(Case.Name, (int32)Scene.Model()->nq, Case.ExpectedNq);
		TestEqual(Case.Name, (int32)Scene.Model()->nmocap, Case.ExpectedMocap);
	}

	return true;
}

// ---------------------------------------------------------------------------
// The generated hull, across a recompile
// ---------------------------------------------------------------------------

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjGeneratedMeshSurvivesRecompileTest,
	"URLab.Convert.GeneratedMeshSurvivesRecompile",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjGeneratedMeshSurvivesRecompileTest::RunTest(const FString& Parameters)
{
	using namespace MjSceneContributorTests;

	UStaticMesh* Cube = UnitCube();
	if (!TestNotNull(TEXT("the engine's unit cube loads"), Cube))
	{
		return false;
	}

	FContributorScene Scene;
	if (!TestTrue(TEXT("world opens"), Scene.Open()))
	{
		AddError(Scene.LastError);
		return false;
	}

	FActorSpawnParameters Params;
	AActor* Prop = Scene.World->SpawnActor<AActor>(Params);
	UStaticMeshComponent* Smc = NewObject<UStaticMeshComponent>(Prop);
	Prop->SetRootComponent(Smc);
	Smc->SetStaticMesh(Cube);
	Smc->RegisterComponent();

	UMjQuickConvertComponent* Convert = NewObject<UMjQuickConvertComponent>(Prop);
	Convert->Static = true;
	Convert->RegisterComponent();
	Prop->AddInstanceComponent(Convert);

	if (!TestTrue(TEXT("the scene compiles"), Scene.Install()))
	{
		AddError(Scene.LastError);
		return false;
	}

	const int32 FirstMeshCount = (int32)Scene.Model()->nmesh;
	const int32 FirstVertCount = (int32)Scene.Model()->nmeshvert;
	TestTrue(TEXT("a mesh was generated"), FirstMeshCount >= 1);

	// The hull has no source asset, so one was written. It is the only place its
	// bytes live between compiles, which is the whole question this test asks.
	const TMap<FString, FString> AssetFiles = Scene.Manager->PhysicsEngine->ActiveAssetFiles;
	TestTrue(TEXT("the generated hull is in the compile's asset list"), AssetFiles.Num() >= 1);
	bool bOnDisk = AssetFiles.Num() > 0;
	for (const TPair<FString, FString>& Asset : AssetFiles)
	{
		const FString& Path = Asset.Value;
		bOnDisk = bOnDisk && FPaths::FileExists(Path);
		TestTrue(FString::Printf(TEXT("the hull exists on disk: %s"), *Path), FPaths::FileExists(Path));
	}
	if (!bOnDisk)
	{
		return false;
	}

	// Second compile: the spec is re-authored from the same actor, the
	// export finds its own content hash and skips, and the bytes are re-read
	// from disk. Nothing about the compiled mesh may move.
	if (!TestTrue(TEXT("the scene recompiles"), Scene.Install()))
	{
		AddError(Scene.LastError);
		return false;
	}
	TestEqual(TEXT("the same number of meshes after a recompile"), (int32)Scene.Model()->nmesh, FirstMeshCount);
	TestEqual(TEXT("the same mesh geometry after a recompile"), (int32)Scene.Model()->nmeshvert, FirstVertCount);

	const FString Prefix = Convert->GetScenePrefix();
	const int32 BodyId = mj_name2id(Scene.Model(), mjOBJ_BODY, TCHAR_TO_UTF8(*(Prefix + TEXT("body"))));
	TestTrue(TEXT("the converted body is still there"), BodyId > 0);
	if (BodyId <= 0)
	{
		return false;
	}
	TestEqual(TEXT("still exactly one geom, not two"), (int32)Scene.Model()->body_geomnum[BodyId], 1);
	TestEqual(TEXT("the recompiled geom still has the cube's bounding radius"),
		(double)Scene.Model()->geom_rbound[Scene.Model()->body_geomadr[BodyId]], FMath::Sqrt(3.0) * 0.5,
		1e-3);

	// And with the generated file deleted, the export runs again and the compile
	// still succeeds: the disk copy is a cache, not a second source of truth.
	for (const TPair<FString, FString>& Asset : AssetFiles)
	{
		IFileManager::Get().Delete(*Asset.Value);
	}
	if (!TestTrue(TEXT("the scene compiles with the hull deleted"), Scene.Install()))
	{
		AddError(Scene.LastError);
		return false;
	}
	TestEqual(TEXT("the regenerated hull compiles to the same mesh"), (int32)Scene.Model()->nmeshvert,
		FirstVertCount);

	return true;
}

// ============================================================================
// URLab.Convert.ShippedAssetsAreNamedAsTheSpecReferencesThem
//
// A scene mounts its assets under prefixed names so two participants cannot
// collide in MuJoCo's flat VFS namespace, and every `file=` carries that prefix.
// The file on disk does not. So a caller that rebuilds the name from the path
// ships bytes under a name no spec asks for, and a remote client is handed
// a model it cannot load -- which is exactly what the bridge handshake did.
//
// The list and the specs come from the same collection pass, so this holds
// them to each other rather than to a literal.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjShippedAssetNamesMatchSpecTest,
	"URLab.Convert.ShippedAssetsAreNamedAsTheSpecReferencesThem",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjShippedAssetNamesMatchSpecTest::RunTest(const FString& Parameters)
{
	using namespace MjSceneContributorTests;

	UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
	if (!TestNotNull(TEXT("the engine cube loads"), Cube))
	{
		return false;
	}

	FContributorScene Scene;
	if (!TestTrue(TEXT("world opens"), Scene.Open()))
	{
		AddError(Scene.LastError);
		return false;
	}

	FActorSpawnParameters Params;
	AActor* Prop = Scene.World->SpawnActor<AActor>(Params);
	UStaticMeshComponent* Smc = NewObject<UStaticMeshComponent>(Prop);
	Prop->SetRootComponent(Smc);
	Smc->SetStaticMesh(Cube);
	Smc->RegisterComponent();

	UMjQuickConvertComponent* Convert = NewObject<UMjQuickConvertComponent>(Prop);
	Convert->Static = true;
	Convert->RegisterComponent();
	Prop->AddInstanceComponent(Convert);

	if (!TestTrue(TEXT("the scene compiles"), Scene.Install()))
	{
		AddError(Scene.LastError);
		return false;
	}

	FMjCompiledScene Compiled;
	FString Error;
	if (!TestTrue(TEXT("the compiled scene is available"),
			Scene.Manager->PhysicsEngine->BuildCompiledScene(Compiled, Error)))
	{
		AddError(Error);
		return false;
	}

	if (!TestTrue(TEXT("the scene ships at least one asset"), Compiled.AssetFiles.Num() >= 1))
	{
		return false;
	}

	// Every name shipped is a name some spec references. The reverse does
	// not hold: a spec may reference an asset that resolved to nothing, and
	// that case is reported elsewhere as a missing asset.
	FString AllText = Compiled.Xml;
	for (const TPair<FString, FString>& Spec : Compiled.ParticipantXml)
	{
		AllText += Spec.Value;
	}

	for (const TPair<FString, FString>& Asset : Compiled.AssetFiles)
	{
		TestTrue(FString::Printf(TEXT("a spec references the shipped name '%s'"), *Asset.Key),
			AllText.Contains(Asset.Key));

		// And the bare filename is not what it is shipped under, or the prefix
		// that makes the name unique never made it into the key.
		const FString Bare = FPaths::GetCleanFilename(Asset.Value);
		if (Bare != Asset.Key)
		{
			TestTrue(FString::Printf(TEXT("the shipped name '%s' keeps the scene's prefix, not the bare '%s'"),
						 *Asset.Key, *Bare),
				Asset.Key.EndsWith(Bare) && Asset.Key.Len() > Bare.Len());
		}
	}

	return true;
}

#endif // URLAB_MJ_GEN && WITH_EDITOR
