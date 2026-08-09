// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CoreMinimal.h"
#include "HAL/FileManager.h"
#include "Misc/AutomationTest.h"
#include "Misc/FileHelper.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_DEV_AUTOMATION_TESTS

#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Elements/MjBody.h"
#include "MuJoCo/Elements/MjGeom.h"
#include "MuJoCo/Gen/MjElements.gen.h"
#include "MuJoCo/Spec/MjAssetSink.h"
#include "MuJoCo/Spec/MjNodeFactories.h"
#include "MuJoCo/Spec/MjSceneSpec.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "Tests/MjTestHelpers.h"

THIRD_PARTY_INCLUDES_START
#include <mujoco/mujoco.h>
THIRD_PARTY_INCLUDES_END

// Scene composition, asserted on the compiled model.
//
// Every tree here is built in the test rather than read from a fixture: what is
// under test is the crossing from components to a composed mjSpec, and a file
// would put the reader in the middle of it. The assets are the exception, and
// they have to be: two participants colliding on `base.obj` is only a real test
// if two different files with that name exist, so the pair already in the
// parity corpus is referenced rather than copied.

namespace
{

// Qualified rather than imported: the engine has a compiled-scene type of its
// own under the same name, and the two are different types.
namespace mjspec = urlab::spec;

/** Where the two same-basename meshes and the texture live. */
FString ParityDir()
{
	return FPaths::Combine(FPaths::ProjectPluginsDir(), TEXT("UnrealRoboticsLab"), TEXT("Content"), TEXT("TestData"),
		TEXT("parity"));
}

/** A world holding as many spec-carrying actors as a scene test needs. */
struct FSceneFixture
{
	UWorld* World = nullptr;
	TArray<AMjArticulation*> Actors;

	bool Init()
	{
		World = UWorld::CreateWorld(EWorldType::Game, false);
		if (World == nullptr)
		{
			return false;
		}
		FWorldContext& Context = GEngine->CreateNewWorldContext(EWorldType::Game);
		Context.SetCurrentWorld(World);
		return true;
	}

	~FSceneFixture()
	{
		if (World != nullptr)
		{
			GEngine->DestroyWorldContext(World);
			World->DestroyWorld(false);
		}
	}

	/** One spec root, with the anonymous world body MJCF requires. */
	AMjArticulation* AddActor(UMjBodyBase*& OutWorldBody)
	{
		AMjArticulation* const Actor = World->SpawnActor<AMjArticulation>(FActorSpawnParameters());
		if (Actor == nullptr)
		{
			return nullptr;
		}
		Actors.Add(Actor);
		OutWorldBody = Add<UMjBodyBase>(*Actor, Actor->Spec);
		return OutWorldBody != nullptr ? Actor : nullptr;
	}

	template <class E>
	E* Add(AMjArticulation& Owner, UMjNodeComponent* Parent, const TCHAR* Name = nullptr)
	{
		if (Parent == nullptr)
		{
			return nullptr;
		}
		mjspec::FMjInstanceScope Scope(Owner);
		UMjNodeComponent& Node = mjspec::FInstanceNodeFactory::Create<typename TMjGeneratedOf<E>::Type>(*Parent);
		if (Name != nullptr)
		{
			Node.MjName = Name;
		}
		return Cast<E>(&Node);
	}
};

/** An `<asset>` section carrying one file-backed mesh, resolved from Dir. */
UMjMeshBase* AddMesh(
	FSceneFixture& Fixture, AMjArticulation& Owner, const TCHAR* Name, const TCHAR* RelativeFile, const FString& Dir)
{
	UMjAsset* const Section = Fixture.Add<UMjAsset>(Owner, Owner.Spec);
	UMjMeshBase* const Mesh = Fixture.Add<UMjMeshBase>(Owner, Section, Name);
	if (Mesh == nullptr)
	{
		return nullptr;
	}
	Mesh->File = FString(RelativeFile);
	// Asset paths resolve against the file the element was read from, so a tree
	// nobody parsed has to say where it would have come from.
	Mesh->SourceFile = FPaths::Combine(Dir, TEXT("in_test.xml"));
	return Mesh;
}

/** A geom that renders one named mesh, under the body given. */
UMjGeom* AddMeshGeom(
	FSceneFixture& Fixture, AMjArticulation& Owner, UMjNodeComponent* Parent, const TCHAR* Name, const TCHAR* MeshName)
{
	UMjGeom* const Geom = Fixture.Add<UMjGeom>(Owner, Parent, Name);
	if (Geom == nullptr)
	{
		return nullptr;
	}
	Geom->Type = EMjGeomType::mesh;
	Geom->Mesh = FString(MeshName);
	return Geom;
}

/** Compile, reporting the diagnostics rather than swallowing them. */
mjspec::FMjCompiledScene CompileScene(FAutomationTestBase& Test, mjspec::FMjSceneSpecBuilder& Builder)
{
	mjspec::FMjCompiledScene Scene = Builder.Compile();
	if (!Scene.IsValid())
	{
		for (const FMjSpecDiagnostic& Diagnostic : Scene.Errors)
		{
			Test.AddError(Diagnostic.ToString());
		}
	}
	return Scene;
}

/** A scratch directory of this run's own, removed with everything under it. */
struct FScratchDir
{
	FString Path;

	FScratchDir()
		: Path(FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("URLabTests"),
			  FGuid::NewGuid().ToString(EGuidFormats::Digits)))
	{
		IFileManager::Get().MakeDirectory(*Path, /*Tree=*/true);
	}

	~FScratchDir() { IFileManager::Get().DeleteDirectory(*Path, /*RequireExists=*/false, /*Tree=*/true); }
};

}  // namespace

// --- Composition ------------------------------------------------------------ //

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSceneSpecComposeTest, "URLab.MuJoCo.SceneSpec.Compose",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSceneSpecComposeTest::RunTest(const FString& Parameters)
{
	FSceneFixture Fixture;
	if (!Fixture.Init())
	{
		AddError(TEXT("could not create the world"));
		return false;
	}

	UMjBodyBase* SceneWorld = nullptr;
	AMjArticulation* const Manager = Fixture.AddActor(SceneWorld);
	UMjBodyBase* ParticipantWorld = nullptr;
	AMjArticulation* const Robot = Fixture.AddActor(ParticipantWorld);
	if (Manager == nullptr || Robot == nullptr)
	{
		AddError(TEXT("could not spawn the spec actors"));
		return false;
	}

	UMjBody* const Link = Fixture.Add<UMjBody>(*Robot, ParticipantWorld, TEXT("link"));
	UMjGeom* const Shape = Fixture.Add<UMjGeom>(*Robot, Link, TEXT("shape"));
	if (Shape == nullptr)
	{
		AddError(TEXT("could not author the participant"));
		return false;
	}
	Shape->Type = EMjGeomType::sphere;
	Shape->Size = TArray<double>({0.1});

	mjspec::FMjSceneSpecBuilder Builder;
	Builder.SetSceneRoot(FSpecRef::OverActor(*Manager));
	mjspec::FMjSceneSpecParticipant Placed;
	Placed.Spec = FSpecRef::OverActor(*Robot);
	Placed.Prefix = TEXT("p0_");
	Placed.MjPos = FVector(1.0, 2.0, 3.0);
	const double QuarterTurn = 1.5707963267948966;
	Placed.MjQuat = FQuat(FVector(0.0, 0.0, 1.0), QuarterTurn);
	Builder.AddParticipant(Placed);

	mjspec::FMjCompiledScene Scene = CompileScene(*this, Builder);
	if (!Scene.IsValid())
	{
		return false;
	}

	// The participant's names carry its prefix, which is MuJoCo's doing rather
	// than ours: nothing renamed a component to get here.
	const int BodyId = mj_name2id(Scene.Model, mjOBJ_BODY, "p0_link");
	if (!TestTrue(TEXT("the participant's body is in the scene under its prefix"), BodyId >= 0))
	{
		return false;
	}
	TestTrue(TEXT("the geom came with it"), mj_name2id(Scene.Model, mjOBJ_GEOM, "p0_shape") >= 0);

	// The frame the participant was attached under placed it, so the body sits
	// where the scene put it rather than where it was authored.
	TestEqual(TEXT("the frame placed the participant in x"),
		static_cast<double>(Scene.Model->body_pos[3 * BodyId + 0]), 1.0, 1e-12);
	TestEqual(TEXT("the frame placed the participant in y"),
		static_cast<double>(Scene.Model->body_pos[3 * BodyId + 1]), 2.0, 1e-12);
	TestEqual(TEXT("the frame placed the participant in z"),
		static_cast<double>(Scene.Model->body_pos[3 * BodyId + 2]), 3.0, 1e-12);
	// MJCF orders a quaternion scalar-first, so a builder that passed Unreal's
	// component order straight through would rotate about the wrong axis.
	TestEqual(TEXT("the frame oriented the participant"),
		static_cast<double>(Scene.Model->body_quat[4 * BodyId + 0]), FMath::Cos(QuarterTurn / 2.0), 1e-12);
	TestEqual(TEXT("the frame oriented the participant"),
		static_cast<double>(Scene.Model->body_quat[4 * BodyId + 3]), FMath::Sin(QuarterTurn / 2.0), 1e-12);

	// Identity comes from the handles rather than from names: the components
	// were never given one and still bind.
	const mjspec::FMjBoundElement* const BoundBody = Scene.BoundIds.Find(Link);
	if (TestNotNull(TEXT("the participant's body bound"), BoundBody))
	{
		TestEqual(TEXT("it bound to the id it compiled to"), BoundBody->Id, BodyId);
		TestEqual(TEXT("it bound as a body"), BoundBody->ObjType, static_cast<int32>(mjOBJ_BODY));
	}
	TestTrue(TEXT("the participant's geom bound"), Scene.BoundIds.Contains(Shape));

	return !HasAnyErrors();
}

// --- The manager's own content ---------------------------------------------- //

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSceneSpecManagerContentTest, "URLab.MuJoCo.SceneSpec.ManagerContent",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSceneSpecManagerContentTest::RunTest(const FString& Parameters)
{
	// A scene root is a spec like any other: its content compiles, its assets
	// mount, and its components bind. Anything less makes manager-authored
	// geometry silently absent from the model it appears in.
	FSceneFixture Fixture;
	if (!Fixture.Init())
	{
		AddError(TEXT("could not create the world"));
		return false;
	}

	UMjBodyBase* SceneWorld = nullptr;
	AMjArticulation* const Manager = Fixture.AddActor(SceneWorld);
	UMjBodyBase* ParticipantWorld = nullptr;
	AMjArticulation* const Robot = Fixture.AddActor(ParticipantWorld);
	if (Manager == nullptr || Robot == nullptr)
	{
		AddError(TEXT("could not spawn the spec actors"));
		return false;
	}

	UMjMeshBase* const Mesh = AddMesh(Fixture, *Manager, TEXT("ground_mesh"), TEXT("meshA/base.obj"), ParityDir());
	UMjGeom* const Geom = AddMeshGeom(Fixture, *Manager, SceneWorld, TEXT("ground"), TEXT("ground_mesh"));
	if (Mesh == nullptr || Geom == nullptr)
	{
		AddError(TEXT("could not author the manager's mesh geom"));
		return false;
	}

	UMjGeom* const Ball = Fixture.Add<UMjGeom>(*Robot, ParticipantWorld, TEXT("ball"));
	if (Ball == nullptr)
	{
		AddError(TEXT("could not author the participant"));
		return false;
	}
	Ball->Type = EMjGeomType::sphere;
	Ball->Size = TArray<double>({0.1});

	mjspec::FMjSceneSpecBuilder Builder;
	Builder.SetSceneRoot(FSpecRef::OverActor(*Manager));
	mjspec::FMjSceneSpecParticipant Placed;
	Placed.Spec = FSpecRef::OverActor(*Robot);
	Placed.Prefix = TEXT("p0_");
	Builder.AddParticipant(Placed);

	mjspec::FMjCompiledScene Scene = CompileScene(*this, Builder);
	if (!Scene.IsValid())
	{
		return false;
	}

	// The mesh has to be in the model, not just in the spec: it only gets there
	// if the manager's assets were mounted for the compile.
	const int MeshId = mj_name2id(Scene.Model, mjOBJ_MESH, "ground_mesh");
	if (TestTrue(TEXT("the manager's mesh compiled"), MeshId >= 0))
	{
		TestTrue(TEXT("its geometry was read"), Scene.Model->mesh_vertnum[MeshId] > 0);
	}
	const int GeomId = mj_name2id(Scene.Model, mjOBJ_GEOM, "ground");
	if (TestTrue(TEXT("the manager's geom compiled"), GeomId >= 0))
	{
		TestEqual(TEXT("the geom uses the manager's mesh"), Scene.Model->geom_dataid[GeomId], MeshId);
	}

	// Bug 4 in one line: manager-authored content binds like a participant's.
	const mjspec::FMjBoundElement* const BoundGeom = Scene.BoundIds.Find(Geom);
	if (TestNotNull(TEXT("the manager's geom bound"), BoundGeom))
	{
		TestEqual(TEXT("it bound to the id it compiled to"), BoundGeom->Id, GeomId);
		TestEqual(TEXT("it bound as a geom"), BoundGeom->ObjType, static_cast<int32>(mjOBJ_GEOM));
	}
	TestTrue(TEXT("the manager's mesh bound"), Scene.BoundIds.Contains(Mesh));
	TestTrue(TEXT("the participant still bound"), Scene.BoundIds.Contains(Ball));

	return !HasAnyErrors();
}

// --- Asset namespacing ------------------------------------------------------ //

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSceneSpecAssetCollisionTest, "URLab.MuJoCo.SceneSpec.AssetCollision",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSceneSpecAssetCollisionTest::RunTest(const FString& Parameters)
{
	// Two participants, each referencing its own `base.obj`. MuJoCo's VFS falls
	// back to a case-insensitive basename match across every mount, so an
	// unprefixed scheme hands both participants the same mesh and says nothing.
	// The two files differ in vertex count, which is what makes the swap
	// visible at all.
	FSceneFixture Fixture;
	if (!Fixture.Init())
	{
		AddError(TEXT("could not create the world"));
		return false;
	}

	UMjBodyBase* SceneWorld = nullptr;
	AMjArticulation* const Manager = Fixture.AddActor(SceneWorld);
	if (Manager == nullptr)
	{
		AddError(TEXT("could not spawn the manager"));
		return false;
	}

	mjspec::FMjSceneSpecBuilder Builder;
	Builder.SetSceneRoot(FSpecRef::OverActor(*Manager));

	const TCHAR* const Files[] = {TEXT("meshA/base.obj"), TEXT("meshB/base.obj")};
	const TCHAR* const Prefixes[] = {TEXT("p0_"), TEXT("p1_")};
	for (int32 Index = 0; Index < 2; ++Index)
	{
		UMjBodyBase* ParticipantWorld = nullptr;
		AMjArticulation* const Robot = Fixture.AddActor(ParticipantWorld);
		if (Robot == nullptr)
		{
			AddError(TEXT("could not spawn a participant"));
			return false;
		}
		if (AddMesh(Fixture, *Robot, TEXT("part"), Files[Index], ParityDir()) == nullptr ||
			AddMeshGeom(Fixture, *Robot, ParticipantWorld, TEXT("shell"), TEXT("part")) == nullptr)
		{
			AddError(TEXT("could not author a participant"));
			return false;
		}

		mjspec::FMjSceneSpecParticipant Placed;
		Placed.Spec = FSpecRef::OverActor(*Robot);
		Placed.Prefix = Prefixes[Index];
		Builder.AddParticipant(Placed);
	}

	mjspec::FMjCompiledScene Scene = CompileScene(*this, Builder);
	if (!Scene.IsValid())
	{
		return false;
	}

	// Both files are mounted, under names that cannot collide.
	TestEqual(TEXT("both participants mounted their own mesh"), Scene.Assets.Num(), 2);
	if (Scene.Assets.Num() == 2)
	{
		TestEqual(TEXT("the first is mounted under its prefix"), Scene.Assets[0].Name, FString(TEXT("p0_base.obj")));
		TestEqual(TEXT("the second is mounted under its prefix"), Scene.Assets[1].Name, FString(TEXT("p1_base.obj")));
		TestNotEqual(TEXT("the two files really do differ"), Scene.Assets[0].Bytes.Num(), Scene.Assets[1].Bytes.Num());
	}

	const int FirstId = mj_name2id(Scene.Model, mjOBJ_MESH, "p0_part");
	const int SecondId = mj_name2id(Scene.Model, mjOBJ_MESH, "p1_part");
	if (!TestTrue(TEXT("both meshes compiled"), FirstId >= 0 && SecondId >= 0))
	{
		return false;
	}
	// The whole point: each participant got the geometry it named, and the
	// counts say so where two identical meshes would not.
	TestNotEqual(TEXT("the participants did not share one mesh"), Scene.Model->mesh_vertnum[FirstId],
		Scene.Model->mesh_vertnum[SecondId]);

	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSceneSpecUnnamedAssetTest, "URLab.MuJoCo.SceneSpec.UnnamedAssets",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSceneSpecUnnamedAssetTest::RunTest(const FString& Parameters)
{
	// An asset that authors no name is named by MuJoCo from its file, and MuJoCo
	// does that AFTER the composition has rewritten `file` to a prefixed
	// basename -- so a mesh whose file became `p0_base.obj` derives the name
	// `p0_base` and is then prefixed again to `p0_p0_base`, while the geom that
	// referred to it was prefixed once to `p0_base`. The compile fails on a
	// reference to nothing. Nothing in the parity corpus exposes this, because
	// every asset in it is named.
	FSceneFixture Fixture;
	if (!Fixture.Init())
	{
		AddError(TEXT("could not create the world"));
		return false;
	}

	UMjBodyBase* SceneWorld = nullptr;
	AMjArticulation* const Manager = Fixture.AddActor(SceneWorld);
	if (Manager == nullptr)
	{
		AddError(TEXT("could not spawn the manager"));
		return false;
	}

	mjspec::FMjSceneSpecBuilder Builder;
	Builder.SetSceneRoot(FSpecRef::OverActor(*Manager));

	const TCHAR* const Files[] = {TEXT("meshA/base.obj"), TEXT("meshB/base.obj")};
	const TCHAR* const Prefixes[] = {TEXT("p0_"), TEXT("p1_")};
	for (int32 Index = 0; Index < 2; ++Index)
	{
		UMjBodyBase* ParticipantWorld = nullptr;
		AMjArticulation* const Robot = Fixture.AddActor(ParticipantWorld);
		if (Robot == nullptr)
		{
			AddError(TEXT("could not spawn a participant"));
			return false;
		}
		// No name on the mesh, and the geom refers to it by the spelling MuJoCo
		// would derive -- which is how such a document is authored.
		if (AddMesh(Fixture, *Robot, nullptr, Files[Index], ParityDir()) == nullptr ||
			AddMeshGeom(Fixture, *Robot, ParticipantWorld, TEXT("shell"), TEXT("base")) == nullptr)
		{
			AddError(TEXT("could not author a participant"));
			return false;
		}

		mjspec::FMjSceneSpecParticipant Placed;
		Placed.Spec = FSpecRef::OverActor(*Robot);
		Placed.Prefix = Prefixes[Index];
		Builder.AddParticipant(Placed);
	}

	mjspec::FMjCompiledScene Scene = CompileScene(*this, Builder);
	if (!Scene.IsValid())
	{
		return false;
	}

	// Prefixed once each, so the geoms' references still name them.
	const int FirstId = mj_name2id(Scene.Model, mjOBJ_MESH, "p0_base");
	const int SecondId = mj_name2id(Scene.Model, mjOBJ_MESH, "p1_base");
	if (!TestTrue(TEXT("both unnamed meshes compiled under one prefix each"), FirstId >= 0 && SecondId >= 0))
	{
		return false;
	}
	TestTrue(TEXT("nothing was prefixed twice"), mj_name2id(Scene.Model, mjOBJ_MESH, "p0_p0_base") < 0);

	// And each geom got its own participant's file rather than a shared one.
	const int FirstGeom = mj_name2id(Scene.Model, mjOBJ_GEOM, "p0_shell");
	const int SecondGeom = mj_name2id(Scene.Model, mjOBJ_GEOM, "p1_shell");
	if (TestTrue(TEXT("both geoms compiled"), FirstGeom >= 0 && SecondGeom >= 0))
	{
		TestEqual(TEXT("the first geom resolved to the first mesh"), Scene.Model->geom_dataid[FirstGeom], FirstId);
		TestEqual(TEXT("the second geom resolved to the second mesh"), Scene.Model->geom_dataid[SecondGeom], SecondId);
	}
	TestNotEqual(TEXT("the two meshes really are different files"), Scene.Model->mesh_vertnum[FirstId],
		Scene.Model->mesh_vertnum[SecondId]);

	return !HasAnyErrors();
}

// --- Discarded globals ------------------------------------------------------ //

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSceneSpecDiscardedGlobalsTest, "URLab.MuJoCo.SceneSpec.DiscardedGlobals",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSceneSpecDiscardedGlobalsTest::RunTest(const FString& Parameters)
{
	FSceneFixture Fixture;
	if (!Fixture.Init())
	{
		AddError(TEXT("could not create the world"));
		return false;
	}

	UMjBodyBase* SceneWorld = nullptr;
	AMjArticulation* const Manager = Fixture.AddActor(SceneWorld);
	UMjBodyBase* ParticipantWorld = nullptr;
	AMjArticulation* const Robot = Fixture.AddActor(ParticipantWorld);
	if (Manager == nullptr || Robot == nullptr)
	{
		AddError(TEXT("could not spawn the spec actors"));
		return false;
	}

	UMjOption* const Option = Fixture.Add<UMjOption>(*Robot, Robot->Spec);
	UMjSize* const Size = Fixture.Add<UMjSize>(*Robot, Robot->Spec);
	if (Option == nullptr || Size == nullptr)
	{
		AddError(TEXT("could not author the participant's global blocks"));
		return false;
	}
	Option->Timestep = 0.001;

	UMjGeom* const Ball = Fixture.Add<UMjGeom>(*Robot, ParticipantWorld, TEXT("ball"));
	if (Ball == nullptr)
	{
		AddError(TEXT("could not author the participant"));
		return false;
	}
	Ball->Type = EMjGeomType::sphere;
	Ball->Size = TArray<double>({0.1});

	mjspec::FMjSceneSpecBuilder Builder;
	Builder.SetSceneRoot(FSpecRef::OverActor(*Manager));
	mjspec::FMjSceneSpecParticipant Placed;
	Placed.Spec = FSpecRef::OverActor(*Robot);
	Placed.Prefix = TEXT("p0_");
	Builder.AddParticipant(Placed);

	mjspec::FMjCompiledScene Scene = CompileScene(*this, Builder);
	if (!Scene.IsValid())
	{
		return false;
	}

	// Two blocks authored, two warnings, and each names the section it is
	// about: a single "globals were dropped" would not say which.
	int32 OptionWarnings = 0;
	int32 SizeWarnings = 0;
	for (const FMjSpecDiagnostic& Warning : Scene.Warnings)
	{
		OptionWarnings += Warning.Message.Contains(TEXT("<option>")) ? 1 : 0;
		SizeWarnings += Warning.Message.Contains(TEXT("<size>")) ? 1 : 0;
	}
	TestEqual(TEXT("the discarded <option> was reported"), OptionWarnings, 1);
	TestEqual(TEXT("the discarded <size> was reported"), SizeWarnings, 1);

	// And it really is discarded rather than merged: the scene keeps its own.
	TestNotEqual(TEXT("the participant's timestep did not reach the model"),
		static_cast<double>(Scene.Model->opt.timestep), 0.001);

	return !HasAnyErrors();
}

// --- The debug artefact ----------------------------------------------------- //

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSceneSpecDebugArtifactTest, "URLab.MuJoCo.SceneSpec.DebugArtifact",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSceneSpecDebugArtifactTest::RunTest(const FString& Parameters)
{
	// The artefact's whole job is being readable outside Unreal, so the test is
	// the load: mj_loadXML with no VFS, from the directory it was written to,
	// resolving its assets from the sidecar beside it.
	FSceneFixture Fixture;
	if (!Fixture.Init())
	{
		AddError(TEXT("could not create the world"));
		return false;
	}

	UMjBodyBase* SceneWorld = nullptr;
	AMjArticulation* const Manager = Fixture.AddActor(SceneWorld);
	UMjBodyBase* ParticipantWorld = nullptr;
	AMjArticulation* const Robot = Fixture.AddActor(ParticipantWorld);
	if (Manager == nullptr || Robot == nullptr)
	{
		AddError(TEXT("could not spawn the spec actors"));
		return false;
	}

	if (AddMesh(Fixture, *Robot, TEXT("part"), TEXT("meshA/base.obj"), ParityDir()) == nullptr ||
		AddMeshGeom(Fixture, *Robot, ParticipantWorld, TEXT("shell"), TEXT("part")) == nullptr)
	{
		AddError(TEXT("could not author the participant"));
		return false;
	}

	mjspec::FMjSceneSpecBuilder Builder;
	Builder.SetSceneRoot(FSpecRef::OverActor(*Manager));
	mjspec::FMjSceneSpecParticipant Placed;
	Placed.Spec = FSpecRef::OverActor(*Robot);
	Placed.Prefix = TEXT("p0_");
	Builder.AddParticipant(Placed);

	mjspec::FMjCompiledScene Scene = CompileScene(*this, Builder);
	if (!Scene.IsValid())
	{
		return false;
	}

	FScratchDir Scratch;
	TArray<FMjSpecDiagnostic> Diagnostics;
	if (!Scene.SaveDebugArtifacts(Scratch.Path, Diagnostics))
	{
		for (const FMjSpecDiagnostic& Diagnostic : Diagnostics)
		{
			AddError(Diagnostic.ToString());
		}
		return false;
	}

	const FString XmlPath = FPaths::Combine(Scratch.Path, TEXT("scene_compiled.xml"));
	TestTrue(TEXT("the mesh was written beside the xml"),
		IFileManager::Get().FileExists(*FPaths::Combine(Scratch.Path, TEXT("scene_assets"), TEXT("p0_base.obj"))));

	char Error[1024] = {0};
	mjModel* const Reloaded = mj_loadXML(TCHAR_TO_UTF8(*XmlPath), nullptr, Error, sizeof(Error));
	if (!TestNotNull(TEXT("stock MuJoCo loads the artefact"), Reloaded))
	{
		AddError(FString::Printf(TEXT("mj_loadXML: %s"), UTF8_TO_TCHAR(Error)));
		return false;
	}

	// Loaded, and loaded as the same scene: an artefact that parsed but lost
	// the participant would otherwise pass.
	TestEqual(TEXT("the reloaded model has the same bodies"), static_cast<int32>(Reloaded->nbody),
		static_cast<int32>(Scene.Model->nbody));
	TestEqual(TEXT("the reloaded model has the same meshes"), static_cast<int32>(Reloaded->nmesh),
		static_cast<int32>(Scene.Model->nmesh));
	if (Reloaded->nmesh > 0 && Scene.Model->nmesh > 0)
	{
		TestEqual(TEXT("the sidecar mesh is the one that was compiled"), Reloaded->mesh_vertnum[0],
			Scene.Model->mesh_vertnum[0]);
	}
	mj_deleteModel(Reloaded);

	return !HasAnyErrors();
}

// --- The asset pass --------------------------------------------------------- //

namespace
{
/** Records what the pass reported, without importing anything. */
class FRecordingSink final : public IMjAssetSink
{
public:
	int32 Missing = 0;
	int32 Delivered = 0;

	void OnMesh(const FMjAssetRequest&, const TArray<uint8>&) override { ++Delivered; }
	void OnTexture(const FMjAssetRequest&, const TArray<uint8>&) override { ++Delivered; }
	void OnHeightField(const FMjAssetRequest&, const TArray<uint8>&) override { ++Delivered; }
	void OnMissing(const FMjAssetRequest&) override { ++Missing; }
};
}  // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjAssetSinkResolutionTest, "URLab.MuJoCo.AssetSink.Resolution",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjAssetSinkResolutionTest::RunTest(const FString& Parameters)
{
	FSceneFixture Fixture;
	if (!Fixture.Init())
	{
		AddError(TEXT("could not create the world"));
		return false;
	}

	UMjBodyBase* WorldBody = nullptr;
	AMjArticulation* const Robot = Fixture.AddActor(WorldBody);
	if (Robot == nullptr)
	{
		AddError(TEXT("could not spawn the spec actor"));
		return false;
	}

	// One mesh that is there and one that is not, so a pass that reports
	// nothing missing and a pass that reports everything missing both fail.
	if (AddMesh(Fixture, *Robot, TEXT("present"), TEXT("meshA/base.obj"), ParityDir()) == nullptr ||
		AddMesh(Fixture, *Robot, TEXT("absent"), TEXT("meshA/nothing.obj"), ParityDir()) == nullptr)
	{
		AddError(TEXT("could not author the meshes"));
		return false;
	}

	const FSpecRef Spec = FSpecRef::OverActor(*Robot);
	for (const bool bLoadBytes : {true, false})
	{
		FRecordingSink Recorder;
		FMjAssetSink Pass(Recorder);
		Pass.bLoadBytes = bLoadBytes;
		Pass.Collect(Spec);

		const FString Label = bLoadBytes ? TEXT("reading the bytes") : TEXT("resolving only");
		TestEqual(FString::Printf(TEXT("%s: the missing file is reported"), *Label), Recorder.Missing, 1);
		TestEqual(FString::Printf(TEXT("%s: the present file is delivered"), *Label), Recorder.Delivered, 1);
	}

	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjAssetSinkCompilerDirsTest, "URLab.MuJoCo.AssetSink.CompilerDirs",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjAssetSinkCompilerDirsTest::RunTest(const FString& Parameters)
{
	// assetdir stands in for both directories, and only an authored meshdir or
	// texturedir overrides it. A pass reading them as plain values would let an
	// unauthored field, or a second <compiler> saying nothing about them, erase
	// what the document actually set.
	FSceneFixture Fixture;
	if (!Fixture.Init())
	{
		AddError(TEXT("could not create the world"));
		return false;
	}

	UMjBodyBase* WorldBody = nullptr;
	AMjArticulation* const Robot = Fixture.AddActor(WorldBody);
	if (Robot == nullptr)
	{
		AddError(TEXT("could not spawn the spec actor"));
		return false;
	}

	UMjCompiler* const Compiler = Fixture.Add<UMjCompiler>(*Robot, Robot->Spec);
	UMjCompiler* const Bare = Fixture.Add<UMjCompiler>(*Robot, Robot->Spec);
	if (Compiler == nullptr || Bare == nullptr)
	{
		AddError(TEXT("could not author the compiler blocks"));
		return false;
	}
	Compiler->Assetdir = FString(TEXT("meshA"));
	Bare->Angle = EMjAngleUnit::radian;

	UMjMeshBase* const Mesh = AddMesh(Fixture, *Robot, TEXT("part"), TEXT("base.obj"), ParityDir());
	if (Mesh == nullptr)
	{
		AddError(TEXT("could not author the mesh"));
		return false;
	}

	FRecordingSink Recorder;
	FMjAssetSink Pass(Recorder);
	Pass.bLoadBytes = false;
	Pass.Collect(FSpecRef::OverActor(*Robot));

	if (TestEqual(TEXT("one request"), Pass.GetRequests().Num(), 1))
	{
		const FMjAssetRequest& Request = Pass.GetRequests()[0];
		TestFalse(TEXT("assetdir survived the second <compiler>"), Request.bMissing);
		TestTrue(TEXT("the path resolved through assetdir"), Request.ResolvedPath.Contains(TEXT("meshA")));
	}

	return !HasAnyErrors();
}

#endif  // URLAB_MJ_GEN && WITH_DEV_AUTOMATION_TESTS
