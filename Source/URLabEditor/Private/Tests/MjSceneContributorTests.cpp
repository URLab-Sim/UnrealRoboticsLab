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
// world pose, and then checked again after a recompile.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Components/BoxComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/Engine.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "PhysicsEngine/BodySetup.h"

#include "MuJoCo/Convert/AMjHeightfieldActor.h"
#include "MuJoCo/Convert/MjQuickConvertComponent.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Authoring/MjAuthoringCompiler.h"
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

	mjModel* PreparedModel = nullptr;
	mjData* PreparedData = nullptr;

	bool Install()
	{
		FMjAuthoredSimulationPackage Package = FMjAuthoringCompiler::Compile(*Manager);
		if (!Package.IsValid()) { LastError = Package.Error; return false; }
		mjModel* NewModel = nullptr;
		mjData* NewData = nullptr;
		const bool bCopied = Package.Candidate.VisitPrepared([&](const mjModel* SourceModel, const mjData* SourceData)
		{
			NewModel = mj_copyModel(nullptr, SourceModel);
			if (NewModel) NewData = mj_copyData(nullptr, NewModel, SourceData);
			return NewModel != nullptr && NewData != nullptr;
		});
		if (!bCopied)
		{
			if (NewData) mj_deleteData(NewData);
			if (NewModel) mj_deleteModel(NewModel);
			LastError = TEXT("Could not copy prepared authoring model for test");
			return false;
		}
		if (PreparedData) mj_deleteData(PreparedData);
		if (PreparedModel) mj_deleteModel(PreparedModel);
		PreparedModel = NewModel;
		PreparedData = NewData;
		LastError.Empty();
		return true;
	}

	mjModel* Model() const { return PreparedModel; }
	mjData* Data() const { return PreparedData; }

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
		if (PreparedData) mj_deleteData(PreparedData);
		if (PreparedModel) mj_deleteModel(PreparedModel);
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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjHeightfieldRejectsDegenerateResolutionTest,
	"URLab.Convert.HeightfieldRejectsDegenerateResolution",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjHeightfieldRejectsDegenerateResolutionTest::RunTest(const FString&)
{
	using namespace MjSceneContributorTests;

	FContributorScene Scene;
	if (!TestTrue(TEXT("world opens"), Scene.Open()))
	{
		AddError(Scene.LastError);
		return false;
	}

	FActorSpawnParameters Params;
	AMjHeightfieldActor* Terrain = Scene.World->SpawnActor<AMjHeightfieldActor>(Params);
	if (!TestNotNull(TEXT("heightfield actor spawns"), Terrain)) return false;
	Terrain->Resolution = 2;
	Terrain->HFieldName = TEXT("two_sample_terrain");
	Terrain->bForceRecache = true;
	if (!TestTrue(TEXT("two samples per axis compile"), Scene.Install()))
	{
		AddError(Scene.LastError);
		return false;
	}
	const mjModel* InstalledModel = Scene.Model();
	if (!TestNotNull(TEXT("compiled model exists"), InstalledModel)) return false;

	// UPROPERTY ClampMin protects editor entry; direct assignment and serialized
	// data can bypass it, so the authoring compiler must reject them explicitly.
	Terrain->Resolution = 1;
	TestFalse(TEXT("one sample per axis fails the authoring compile"), Scene.Install());
	TestTrue(TEXT("error names the actor and required sample count"),
		Scene.LastError.Contains(Terrain->GetName())
			&& Scene.LastError.Contains(TEXT("at least 2 samples per axis"))
			&& Scene.LastError.Contains(TEXT("Resolution is 1")));
	TestTrue(TEXT("failed compile preserves the installed model"), Scene.Model() == InstalledModel);
	return !HasAnyErrors();
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
	TestEqual(TEXT("the body carries a visual and a collision geom"), (int32)Model->body_geomnum[BodyId], 2);
	if (Model->body_geomnum[BodyId] != 2)
	{
		return false;
	}
	TestEqual(TEXT("a static conversion adds no degrees of freedom"), (int32)Model->nq, 0);

	const int32 FirstGeomId = Model->body_geomadr[BodyId];
	int32 VisualGeomId = -1;
	int32 CollisionGeomId = -1;
	for (int32 GeomOffset = 0; GeomOffset < 2; ++GeomOffset)
	{
		const int32 GeomId = FirstGeomId + GeomOffset;
		TestEqual(TEXT("both authored geoms are meshes"), (int32)Model->geom_type[GeomId], (int32)mjGEOM_MESH);
		TestTrue(TEXT("each geom names a mesh resource"), Model->geom_dataid[GeomId] >= 0);
		if (Model->geom_group[GeomId] == 2) VisualGeomId = GeomId;
		if (Model->geom_group[GeomId] == 3) CollisionGeomId = GeomId;
	}
	TestTrue(TEXT("full source mesh is the visible group-2 geom"), VisualGeomId >= 0);
	TestTrue(TEXT("simple hull is the hidden group-3 collision geom"), CollisionGeomId >= 0);
	if (VisualGeomId < 0 || CollisionGeomId < 0) return false;
	TestEqual(TEXT("visual geom does not participate in contact"), Model->geom_contype[VisualGeomId], 0);
	TestEqual(TEXT("visual geom is outside contact affinity"), Model->geom_conaffinity[VisualGeomId], 0);
	TestTrue(TEXT("collision geom participates in contact"), Model->geom_contype[CollisionGeomId] != 0);
	bool bVisualMassIsZero = false;
	for (const TObjectPtr<UMjNodeComponent>& GeomNode : Convert->GetGeomElements())
	{
		const UMjGeomBase* Geom = Cast<UMjGeomBase>(GeomNode.Get());
		if (Geom != nullptr && Geom->Group.Get(-1) == 2)
		{
			bVisualMassIsZero = Geom->Mass.IsSet() && FMath::IsNearlyZero(Geom->Mass.GetValue());
		}
	}
	TestTrue(TEXT("the source visual is explicitly massless"), bVisualMassIsZero);
	TestTrue(TEXT("collision geometry gives the body positive mass"), Model->body_mass[BodyId] > 0.0);
	TestTrue(TEXT("both visual and inline collision mesh assets compiled"), Model->nmesh >= 2);

	// The engine cube is 100 uu on a side, so half a metre either way and a
	// bounding radius of the half diagonal. This is what proves the OBJ that was
	// written carries the shape and the units, rather than merely existing.
	TestEqual(TEXT("the visual mesh has the cube's bounding radius"), (double)Model->geom_rbound[VisualGeomId],
		FMath::Sqrt(3.0) * 0.5, 1e-3);

	TestEqual(TEXT("friction reached the collision geom"), (double)Model->geom_friction[3 * CollisionGeomId + 0], 1.2,
		1e-9);
	TestEqual(TEXT("torsional friction reached the compiled geom"),
		(double)Model->geom_friction[3 * CollisionGeomId + 1], 0.006, 1e-9);

	mjData* Data = Scene.Data();
	mj_forward(Model, Data);
	// Unreal (300, 400, 500) cm is MuJoCo (3, -4, 5) m.
	TestEqual(TEXT("visual world X"), Data->geom_xpos[3 * VisualGeomId + 0], 3.0, 1e-6);
	TestEqual(TEXT("visual world Y"), Data->geom_xpos[3 * VisualGeomId + 1], -4.0, 1e-6);
	TestEqual(TEXT("visual world Z"), Data->geom_xpos[3 * VisualGeomId + 2], 5.0, 1e-6);

	TestEqual(TEXT("the debug id map names both compiled geoms"), Convert->m_geomName2ID.Num(), 2);
	for (const TPair<FString, int>& Entry : Convert->m_geomName2ID)
	{
		TestTrue(TEXT("the mapped id is one of the compiled geoms"), Entry.Value == VisualGeomId || Entry.Value == CollisionGeomId);
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
	const FString Prefix = Convert->GetScenePrefix();
	const int32 BodyId = mj_name2id(Scene.Model(), mjOBJ_BODY, TCHAR_TO_UTF8(*(Prefix + TEXT("body"))));
	TestTrue(TEXT("the converted body is still there"), BodyId > 0);
	if (BodyId <= 0)
	{
		return false;
	}
	const int32 FirstGeomCount = Scene.Model()->body_geomnum[BodyId];
	const int32 FirstGeomAddress = Scene.Model()->body_geomadr[BodyId];
	int32 FirstVisualGeom = -1;
	int32 FirstCollisionGeom = -1;
	for (int32 Offset = 0; Offset < FirstGeomCount; ++Offset)
	{
		const int32 GeomId = FirstGeomAddress + Offset;
		if (Scene.Model()->geom_contype[GeomId] == 0 && Scene.Model()->geom_conaffinity[GeomId] == 0)
		{
			FirstVisualGeom = GeomId;
		}
		else
		{
			FirstCollisionGeom = GeomId;
		}
	}
	TestEqual(TEXT("one full visual and one collision geom are authored"), FirstGeomCount, 2);
	TestTrue(TEXT("the visual geom contributes no contacts"), FirstVisualGeom >= 0);
	TestTrue(TEXT("the collision hull contributes contacts"), FirstCollisionGeom >= 0);
	if (FirstVisualGeom < 0 || FirstCollisionGeom < 0)
	{
		return false;
	}
	TestEqual(TEXT("visual mesh stays in display group 2"), Scene.Model()->geom_group[FirstVisualGeom], 2);
	TestEqual(TEXT("collision hull stays in hidden collision group 3"), Scene.Model()->geom_group[FirstCollisionGeom], 3);
	bool bVisualMassIsZero = false;
	for (const TObjectPtr<UMjNodeComponent>& GeomNode : Convert->GetGeomElements())
	{
		const UMjGeomBase* Geom = Cast<UMjGeomBase>(GeomNode.Get());
		if (Geom != nullptr && Geom->Group.Get(-1) == 2)
		{
			bVisualMassIsZero = Geom->Mass.IsSet() && FMath::IsNearlyZero(Geom->Mass.GetValue());
		}
	}
	TestTrue(TEXT("the source visual geom is explicitly massless"), bVisualMassIsZero);
	TestTrue(TEXT("collision hull preserves body mass"), Scene.Model()->body_mass[BodyId] > 0.0);
	const double FirstVisualRadius = Scene.Model()->geom_rbound[FirstVisualGeom];
	const double FirstCollisionRadius = Scene.Model()->geom_rbound[FirstCollisionGeom];
	const double FirstBodyMass = Scene.Model()->body_mass[BodyId];
	TestTrue(TEXT("a mesh was generated"), FirstMeshCount >= 1);

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
	TestEqual(TEXT("same two geoms after recompile"), Scene.Model()->body_geomnum[BodyId], FirstGeomCount);
	TestEqual(TEXT("same body mass after recompile"), (double)Scene.Model()->body_mass[BodyId], FirstBodyMass, 1e-9);
	const int32 RecompiledGeomAddress = Scene.Model()->body_geomadr[BodyId];
	int32 RecompiledVisualGeom = -1;
	int32 RecompiledCollisionGeom = -1;
	for (int32 Offset = 0; Offset < Scene.Model()->body_geomnum[BodyId]; ++Offset)
	{
		const int32 GeomId = RecompiledGeomAddress + Offset;
		if (Scene.Model()->geom_contype[GeomId] == 0 && Scene.Model()->geom_conaffinity[GeomId] == 0)
		{
			RecompiledVisualGeom = GeomId;
		}
		else
		{
			RecompiledCollisionGeom = GeomId;
		}
	}
	TestTrue(TEXT("visual geom remains contact-free after recompile"), RecompiledVisualGeom >= 0);
	TestTrue(TEXT("collision geom remains enabled after recompile"), RecompiledCollisionGeom >= 0);
	if (RecompiledVisualGeom >= 0 && RecompiledCollisionGeom >= 0)
	{
		TestEqual(TEXT("visual radius is stable"), (double)Scene.Model()->geom_rbound[RecompiledVisualGeom], FirstVisualRadius, 1e-9);
		TestEqual(TEXT("collision radius is stable"), (double)Scene.Model()->geom_rbound[RecompiledCollisionGeom], FirstCollisionRadius, 1e-9);
		TestEqual(TEXT("visual remains in group 2"), Scene.Model()->geom_group[RecompiledVisualGeom], 2);
		TestEqual(TEXT("collision remains in group 3"), Scene.Model()->geom_group[RecompiledCollisionGeom], 3);
		TestEqual(TEXT("visual remains contact-free after recompile"), Scene.Model()->geom_contype[RecompiledVisualGeom], 0);
	}


	return true;
}

#endif // URLAB_MJ_GEN && WITH_EDITOR
