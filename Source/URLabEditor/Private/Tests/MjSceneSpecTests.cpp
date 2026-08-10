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
#include "MuJoCo/Spec/MjSceneAssembly.h"
#include "MuJoCo/Spec/MjSceneMjcf.h"
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

// --- The composed scene's name ---------------------------------------------- //

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSceneSpecSceneNameTest, "URLab.MuJoCo.SceneSpec.SceneName",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSceneSpecSceneNameTest::RunTest(const FString& Parameters)
{
	// The compiled model's name is the first string in its name table, so the
	// assertion is on the model rather than on the spec it came from: a builder
	// that names the spec and a compile that names the model from somewhere else
	// would otherwise both pass.
	//
	// The MJCF the handshake ships is read out beside it, because the two are
	// one scene: a client is handed the model and the text together and
	// reconciles them, so a text that says `scene` over a model that says
	// `warehouse` is two scenes as far as that client can tell.
	const auto SceneNameWith = [this](const TCHAR* const Authored, FString& OutName, FString& OutTextName) {
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
		if (Manager == nullptr || Robot == nullptr || Manager->Spec == nullptr)
		{
			AddError(TEXT("could not spawn the spec actors"));
			return false;
		}
		if (Authored != nullptr)
		{
			Manager->Spec->Model = FString(Authored);
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
		OutName = UTF8_TO_TCHAR(Scene.Model->names);

		FSceneAssembly Assembly;
		Assembly.SetSceneRoot(FSpecRef::OverActor(*Manager));
		Assembly.Add(FSpecRef::OverActor(*Robot), TEXT("p0_"));

		TMap<FString, FString> ParticipantXml;
		TArray<FMjSpecDiagnostic> Diagnostics;
		const FString SceneXml = MjWriteSceneMjcf(Assembly, ParticipantXml, &Diagnostics);
		for (const FMjSpecDiagnostic& Diagnostic : Diagnostics)
		{
			AddError(Diagnostic.ToString());
		}

		// The `model` attribute of the document's root element, read off the
		// text rather than recomputed: what a client parses is the assertion.
		const FString Opening = TEXT("<mujoco model=\"");
		const int32 Start = SceneXml.Find(Opening, ESearchCase::CaseSensitive);
		if (Start == INDEX_NONE)
		{
			AddError(TEXT("the scene text has no <mujoco model=...> root element"));
			return false;
		}
		const int32 From = Start + Opening.Len();
		const int32 End = SceneXml.Find(TEXT("\""), ESearchCase::CaseSensitive, ESearchDir::FromStart, From);
		if (End == INDEX_NONE)
		{
			AddError(TEXT("the scene text's model attribute is unterminated"));
			return false;
		}
		OutTextName = SceneXml.Mid(From, End - From);
		return true;
	};

	// Unnamed: `scene`, and not the name mj_makeSpec leaves on a fresh spec.
	FString Unnamed;
	FString UnnamedText;
	if (SceneNameWith(nullptr, Unnamed, UnnamedText))
	{
		TestEqual(TEXT("a scene root that authored no model name composes as 'scene'"), Unnamed,
			FString(TEXT("scene")));
		TestEqual(TEXT("unnamed: the handshake text names the same scene the model does"), UnnamedText, Unnamed);
	}

	// Named: the manager's own, which the builder used to overwrite.
	FString Named;
	FString NamedText;
	if (SceneNameWith(TEXT("warehouse"), Named, NamedText))
	{
		TestEqual(TEXT("a scene root's authored model name is the composed scene's"), Named,
			FString(TEXT("warehouse")));
		TestEqual(TEXT("named: the handshake text names the same scene the model does"), NamedText, Named);
	}

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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSceneSpecManagerAssetsShipTest, "URLab.MuJoCo.SceneSpec.ManagerAssetsShip",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSceneSpecManagerAssetsShipTest::RunTest(const FString& Parameters)
{
	// A manager-authored mesh compiles into the scene, so a client handed the
	// scene text has to be handed the mesh too. The ship-list used to walk the
	// participants only, which shipped a document referencing bytes that were
	// never sent: the client's compile fails, or worse, MuJoCo's basename
	// fallback finds a participant's file of the same name and it does not.
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

	// The manager's mesh and the participant's are two different files sharing
	// one basename, so a ship-list that mounted the wrong bytes under the right
	// name reaches the vertex-count assertions below and fails there.
	if (AddMesh(Fixture, *Manager, TEXT("ground_mesh"), TEXT("meshA/base.obj"), ParityDir()) == nullptr ||
		AddMeshGeom(Fixture, *Manager, SceneWorld, TEXT("ground"), TEXT("ground_mesh")) == nullptr ||
		AddMesh(Fixture, *Robot, TEXT("part"), TEXT("meshB/base.obj"), ParityDir()) == nullptr ||
		AddMeshGeom(Fixture, *Robot, ParticipantWorld, TEXT("shell"), TEXT("part")) == nullptr)
	{
		AddError(TEXT("could not author the scene's meshes"));
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

	FSceneAssembly Assembly;
	Assembly.SetSceneRoot(FSpecRef::OverActor(*Manager));
	Assembly.Add(FSpecRef::OverActor(*Robot), TEXT("p0_"));

	// The scene root mounts under no prefix, so its mount name is the reference
	// its document authored, and that is the key the text asks for.
	const TMap<FString, FString> Shipped = Assembly.CollectAssetFiles();
	TestTrue(TEXT("the manager's mesh is in the ship-list"), Shipped.Contains(TEXT("meshA/base.obj")));
	TestTrue(TEXT("the participant's mesh is still in the ship-list"), Shipped.Contains(TEXT("p0_base.obj")));

	TMap<FString, FString> ParticipantXml;
	TArray<FMjSpecDiagnostic> Diagnostics;
	const FString SceneXml = MjWriteSceneMjcf(Assembly, ParticipantXml, &Diagnostics);
	for (const FMjSpecDiagnostic& Diagnostic : Diagnostics)
	{
		AddError(Diagnostic.ToString());
	}

	// End to end: the shipped payload alone, through stock MuJoCo.
	mjVFS Shipping;
	mj_defaultVFS(&Shipping);
	MjForEachSceneVfsEntry(Shipped, ParticipantXml, [&Shipping](const FString& Name, TArrayView<const uint8> Bytes) {
		mj_addBufferVFS(&Shipping, TCHAR_TO_UTF8(*Name), Bytes.GetData(), Bytes.Num());
	});

	char Error[1024] = {0};
	mjSpec* const Parsed = mj_parseXMLString(TCHAR_TO_UTF8(*SceneXml), &Shipping, Error, sizeof(Error));
	if (Parsed == nullptr)
	{
		AddError(FString::Printf(TEXT("stock MuJoCo rejected the shipped scene text: %s"), UTF8_TO_TCHAR(Error)));
		mj_deleteVFS(&Shipping);
		return false;
	}
	mjModel* const Reloaded = mj_compile(Parsed, &Shipping);
	mj_deleteVFS(&Shipping);
	if (Reloaded == nullptr)
	{
		AddError(FString::Printf(
			TEXT("the shipped payload did not compile: %s"), UTF8_TO_TCHAR(mjs_getError(Parsed))));
		mj_deleteSpec(Parsed);
		return false;
	}

	const int Mine = mj_name2id(Scene.Model, mjOBJ_MESH, "ground_mesh");
	const int Theirs = mj_name2id(Reloaded, mjOBJ_MESH, "ground_mesh");
	if (TestTrue(TEXT("the manager's mesh is in both models"), Mine >= 0 && Theirs >= 0))
	{
		const int32 Vertices = static_cast<int32>(Scene.Model->mesh_vertnum[Mine]);
		if (TestEqual(TEXT("the shipped bytes are the manager's own file"),
				static_cast<int32>(Reloaded->mesh_vertnum[Theirs]), Vertices))
		{
			TestTrue(TEXT("the manager's mesh resolved to the same bytes"),
				FMemory::Memcmp(Scene.Model->mesh_vert + 3 * Scene.Model->mesh_vertadr[Mine],
					Reloaded->mesh_vert + 3 * Reloaded->mesh_vertadr[Theirs], 3 * Vertices * sizeof(float)) == 0);
		}
		const int GroundGeom = mj_name2id(Reloaded, mjOBJ_GEOM, "ground");
		if (TestTrue(TEXT("the manager's geom reloaded"), GroundGeom >= 0))
		{
			TestEqual(TEXT("it still renders the manager's mesh"), Reloaded->geom_dataid[GroundGeom], Theirs);
		}
	}

	// And the participant's same-basename file did not stand in for it.
	const int Part = mj_name2id(Reloaded, mjOBJ_MESH, "p0_part");
	if (TestTrue(TEXT("the participant's mesh reloaded too"), Part >= 0 && Theirs >= 0))
	{
		TestNotEqual(TEXT("the two 'base.obj' stayed two files"), static_cast<int32>(Reloaded->mesh_vertnum[Theirs]),
			static_cast<int32>(Reloaded->mesh_vertnum[Part]));
	}

	mj_deleteModel(Reloaded);
	mj_deleteSpec(Parsed);
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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSceneMjcfMountNamesTest, "URLab.MuJoCo.SceneSpec.HandshakeTextNamesTheMounts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSceneMjcfMountNamesTest::RunTest(const FString& Parameters)
{
	// One participant referencing two `base.obj` from different folders. The
	// mounts are `p0_base.obj` and `p0_base_2.obj`, and the MJCF handed to a
	// remote client has to ask for those two names -- a writer deriving the name
	// from the reference's own basename asks for the first one twice, and the
	// client silently gets one mesh where the model has two.
	//
	// The compiled scene cannot show that: the compile never goes through text.
	// So the test compiles both, the scene through the spec path and the text
	// through stock MuJoCo against the very mounts the scene produced, and
	// compares the two models' mesh data.
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

	if (AddMesh(Fixture, *Robot, TEXT("left"), TEXT("meshA/base.obj"), ParityDir()) == nullptr ||
		AddMesh(Fixture, *Robot, TEXT("right"), TEXT("meshB/base.obj"), ParityDir()) == nullptr ||
		AddMeshGeom(Fixture, *Robot, ParticipantWorld, TEXT("shell_left"), TEXT("left")) == nullptr ||
		AddMeshGeom(Fixture, *Robot, ParticipantWorld, TEXT("shell_right"), TEXT("right")) == nullptr)
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

	// The mounts the client will be handed, which are what its text has to name.
	if (!TestEqual(TEXT("both meshes are mounted"), Scene.Assets.Num(), 2))
	{
		return false;
	}
	TestEqual(TEXT("the first claimant keeps the clean name"), Scene.Assets[0].Name, FString(TEXT("p0_base.obj")));
	TestEqual(TEXT("the second is disambiguated"), Scene.Assets[1].Name, FString(TEXT("p0_base_2.obj")));

	FSceneAssembly Assembly;
	Assembly.SetSceneRoot(FSpecRef::OverActor(*Manager));
	Assembly.Add(FSpecRef::OverActor(*Robot), TEXT("p0_"));

	TMap<FString, FString> ParticipantXml;
	TArray<FMjSpecDiagnostic> Diagnostics;
	const FString SceneXml = MjWriteSceneMjcf(Assembly, ParticipantXml, &Diagnostics);
	for (const FMjSpecDiagnostic& Diagnostic : Diagnostics)
	{
		AddError(Diagnostic.ToString());
	}
	const FString* const Written = ParticipantXml.Find(TEXT("p0_model.xml"));
	if (Written == nullptr)
	{
		AddError(TEXT("the scene wrote no MJCF for its participant"));
		return false;
	}
	TestTrue(TEXT("the text asks for the first mount"), Written->Contains(TEXT("file=\"p0_base.obj\"")));
	TestTrue(TEXT("the text asks for the second mount"), Written->Contains(TEXT("file=\"p0_base_2.obj\"")));

	// Exactly what a client is given: the scene text, the participant text it
	// names, and the asset bytes under the names the scene mounted them by. The
	// specs outlive the models they produce, the ordering the compiled scene
	// keeps.
	TArray<mjSpec*> Specs;
	const auto CompileHandshake = [&](const FString& Participant) -> mjModel* {
		mjVFS Vfs;
		mj_defaultVFS(&Vfs);
		for (const urlab::spec::FMjSceneAsset& Asset : Scene.Assets)
		{
			mj_addBufferVFS(&Vfs, TCHAR_TO_UTF8(*Asset.Name), Asset.Bytes.GetData(), Asset.Bytes.Num());
		}
		const FTCHARToUTF8 Payload(*Participant);
		mj_addBufferVFS(&Vfs, "p0_model.xml", Payload.Get(), Payload.Length());

		char Error[1024] = {0};
		mjSpec* const Parsed = mj_parseXMLString(TCHAR_TO_UTF8(*SceneXml), &Vfs, Error, sizeof(Error));
		if (Parsed == nullptr)
		{
			AddError(FString::Printf(TEXT("stock MuJoCo rejected the handshake MJCF: %s"), UTF8_TO_TCHAR(Error)));
			mj_deleteVFS(&Vfs);
			return nullptr;
		}
		Specs.Add(Parsed);
		mjModel* const Model = mj_compile(Parsed, &Vfs);
		if (Model == nullptr)
		{
			AddError(
				FString::Printf(TEXT("the handshake MJCF did not compile: %s"), UTF8_TO_TCHAR(mjs_getError(Parsed))));
		}
		mj_deleteVFS(&Vfs);
		return Model;
	};

	const auto Release = [&Specs](mjModel* Model) {
		mj_deleteModel(Model);
		for (mjSpec* Spec : Specs)
		{
			mj_deleteSpec(Spec);
		}
	};

	mjModel* const Reloaded = CompileHandshake(*Written);
	if (Reloaded == nullptr)
	{
		Release(nullptr);
		return false;
	}

	TestEqual(TEXT("the text compiled the same number of meshes"), static_cast<int32>(Reloaded->nmesh),
		static_cast<int32>(Scene.Model->nmesh));

	// Each mesh in the reloaded model against the same mesh in the scene the
	// text describes: same vertex and face counts, and the same vertex bytes.
	// A reference pointing at the other mount compiles perfectly well and fails
	// here.
	const auto SameMeshes = [this, &Scene](mjModel* Other, const TCHAR* Route) {
		for (const TCHAR* const MeshName : {TEXT("p0_left"), TEXT("p0_right")})
		{
			const int Mine = mj_name2id(Scene.Model, mjOBJ_MESH, TCHAR_TO_UTF8(MeshName));
			const int Theirs = mj_name2id(Other, mjOBJ_MESH, TCHAR_TO_UTF8(MeshName));
			if (!TestTrue(FString::Printf(TEXT("%s: '%s' is in both models"), Route, MeshName), Mine >= 0 && Theirs >= 0))
			{
				continue;
			}
			const int32 Vertices = static_cast<int32>(Scene.Model->mesh_vertnum[Mine]);
			if (!TestEqual(FString::Printf(TEXT("%s: '%s' has the same vertex count"), Route, MeshName),
					static_cast<int32>(Other->mesh_vertnum[Theirs]), Vertices))
			{
				continue;
			}
			TestEqual(FString::Printf(TEXT("%s: '%s' has the same face count"), Route, MeshName),
				static_cast<int32>(Other->mesh_facenum[Theirs]), static_cast<int32>(Scene.Model->mesh_facenum[Mine]));
			TestTrue(FString::Printf(TEXT("%s: '%s' resolved to the same bytes"), Route, MeshName),
				FMemory::Memcmp(Scene.Model->mesh_vert + 3 * Scene.Model->mesh_vertadr[Mine],
					Other->mesh_vert + 3 * Other->mesh_vertadr[Theirs], 3 * Vertices * sizeof(float)) == 0);
		}
	};
	SameMeshes(Reloaded, TEXT("the written text"));

	// And the two are still two: a text that names one mount twice reaches here
	// with a pair of identical meshes that each match nothing in particular.
	const int Left = mj_name2id(Reloaded, mjOBJ_MESH, "p0_left");
	const int Right = mj_name2id(Reloaded, mjOBJ_MESH, "p0_right");
	if (TestTrue(TEXT("both meshes reloaded"), Left >= 0 && Right >= 0))
	{
		TestNotEqual(TEXT("the reloaded meshes are two different meshes"),
			static_cast<int32>(Reloaded->mesh_vertnum[Left]), static_cast<int32>(Reloaded->mesh_vertnum[Right]));

		const int LeftGeom = mj_name2id(Reloaded, mjOBJ_GEOM, "p0_shell_left");
		const int RightGeom = mj_name2id(Reloaded, mjOBJ_GEOM, "p0_shell_right");
		if (TestTrue(TEXT("both geoms reloaded"), LeftGeom >= 0 && RightGeom >= 0))
		{
			TestEqual(TEXT("the left geom kept its own mesh"), Reloaded->geom_dataid[LeftGeom], Left);
			TestEqual(TEXT("the right geom kept its own mesh"), Reloaded->geom_dataid[RightGeom], Right);
		}
	}

	// And now the payload the handshake actually ships, assembled by the pass
	// the dispatcher runs rather than by this test: the scene text verbatim as
	// `mjcf_compiled`, and a VFS whose entries are the ship-list's mounts read
	// off disk plus the participant document under the name the scene's
	// `<model file=...>` row asks for. The dispatcher used to reduce every
	// `file=` in both documents to its basename before shipping them, which
	// collapsed the two mounts back onto one and handed the client one mesh
	// twice -- with the model it shipped alongside still holding two.
	const TMap<FString, FString> Shipped = Assembly.CollectAssetFiles();
	TestTrue(TEXT("the ship-list carries the first mount"), Shipped.Contains(TEXT("p0_base.obj")));
	TestTrue(TEXT("the ship-list carries the second mount"), Shipped.Contains(TEXT("p0_base_2.obj")));

	mjVFS Shipping;
	mj_defaultVFS(&Shipping);
	int32 Entries = 0;
	MjForEachSceneVfsEntry(Shipped, ParticipantXml,
		[&Shipping, &Entries](const FString& Name, TArrayView<const uint8> Bytes) {
			mj_addBufferVFS(&Shipping, TCHAR_TO_UTF8(*Name), Bytes.GetData(), Bytes.Num());
			++Entries;
		});
	// Two meshes and one participant document, and the mesh bytes came off the
	// ship-list's paths rather than out of the compile's own mounts.
	TestEqual(TEXT("the dispatcher ships both meshes and the participant document"), Entries, 3);

	char ShippedError[1024] = {0};
	mjSpec* const ShippedSpec = mj_parseXMLString(TCHAR_TO_UTF8(*SceneXml), &Shipping, ShippedError, sizeof(ShippedError));
	if (ShippedSpec == nullptr)
	{
		AddError(FString::Printf(TEXT("stock MuJoCo rejected the shipped scene text: %s"), UTF8_TO_TCHAR(ShippedError)));
	}
	else
	{
		Specs.Add(ShippedSpec);
		mjModel* const ShippedModel = mj_compile(ShippedSpec, &Shipping);
		if (ShippedModel == nullptr)
		{
			AddError(FString::Printf(
				TEXT("the shipped payload did not compile: %s"), UTF8_TO_TCHAR(mjs_getError(ShippedSpec))));
		}
		else
		{
			TestEqual(TEXT("the shipped payload compiled the same number of meshes"),
				static_cast<int32>(ShippedModel->nmesh), static_cast<int32>(Scene.Model->nmesh));
			SameMeshes(ShippedModel, TEXT("the shipped payload"));
			mj_deleteModel(ShippedModel);
		}
	}
	mj_deleteVFS(&Shipping);

	// The convention this replaced, put through the same comparison so that the
	// comparison is shown to discriminate: the two references collapsed onto one
	// name by their shared basename, which is what deriving the mount name from
	// the reference produced. It parses and compiles perfectly well, and it
	// compiles to the first mesh twice.
	FString Collapsed = *Written;
	Collapsed.ReplaceInline(TEXT("file=\"p0_base_2.obj\""), TEXT("file=\"p0_base.obj\""));
	if (TestFalse(TEXT("the collapsed text asks for one mount twice"), Collapsed.Contains(TEXT("p0_base_2.obj"))))
	{
		if (mjModel* const Wrong = CompileHandshake(Collapsed))
		{
			const int WrongLeft = mj_name2id(Wrong, mjOBJ_MESH, "p0_left");
			const int WrongRight = mj_name2id(Wrong, mjOBJ_MESH, "p0_right");
			if (TestTrue(TEXT("the collapsed text still compiles two meshes"), WrongLeft >= 0 && WrongRight >= 0))
			{
				TestEqual(TEXT("and they are one mesh twice, which the assertions above reject"),
					static_cast<int32>(Wrong->mesh_vertnum[WrongRight]),
					static_cast<int32>(Wrong->mesh_vertnum[WrongLeft]));
			}
			mj_deleteModel(Wrong);
		}
	}

	Release(Reloaded);
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

	// Two blocks authored, two reports, on the two channels their consequences
	// deserve: a single "globals were dropped" would say neither which section
	// nor how much it matters.
	int32 OptionWarnings = 0;
	int32 SizeWarnings = 0;
	for (const FMjSpecDiagnostic& Warning : Scene.Warnings)
	{
		OptionWarnings += Warning.Message.Contains(TEXT("<option>")) ? 1 : 0;
		SizeWarnings += Warning.Message.Contains(TEXT("<size>")) ? 1 : 0;
	}
	int32 SizeInfos = 0;
	int32 OptionInfos = 0;
	for (const FMjSpecDiagnostic& Info : Scene.Infos)
	{
		SizeInfos += Info.Message.Contains(TEXT("<size>")) ? 1 : 0;
		OptionInfos += Info.Message.Contains(TEXT("<option>")) ? 1 : 0;
	}
	TestEqual(TEXT("the participant's <option> was warned about"), OptionWarnings, 1);
	TestEqual(TEXT("and not merely noted"), OptionInfos, 0);
	// <size> is settled by MuJoCo's own conflict resolver, so it is information
	// rather than a warning: nothing behaves differently for it.
	TestEqual(TEXT("the participant's <size> was reported as information"), SizeInfos, 1);
	TestEqual(TEXT("and not warned about"), SizeWarnings, 0);

	// The scene keeps its own option block under the default policy, which is
	// what the warning is about.
	TestNotEqual(TEXT("the participant's timestep did not reach the model"),
		static_cast<double>(Scene.Model->opt.timestep), 0.001);

	return !HasAnyErrors();
}

// --- A refused attach -------------------------------------------------------- //

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSceneSpecAttachFailureTest, "URLab.MuJoCo.SceneSpec.AttachFailureDiagnostic",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSceneSpecAttachFailureTest::RunTest(const FString& Parameters)
{
	// A refused attach leaves the caller with the diagnostic and nothing else:
	// the scene is abandoned, and the spec the reason was written onto is the
	// one the failure may have wrecked. Forced here through the mechanism MuJoCo
	// provides for it -- the scene and the participant author different
	// timesteps, and the scene's conflict policy says `error`.
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

	UMjCompiler* const Policy = Fixture.Add<UMjCompiler>(*Manager, Manager->Spec);
	UMjOption* const SceneOption = Fixture.Add<UMjOption>(*Manager, Manager->Spec);
	UMjOption* const RobotOption = Fixture.Add<UMjOption>(*Robot, Robot->Spec);
	UMjGeom* const Ball = Fixture.Add<UMjGeom>(*Robot, ParticipantWorld, TEXT("ball"));
	if (Policy == nullptr || SceneOption == nullptr || RobotOption == nullptr || Ball == nullptr)
	{
		AddError(TEXT("could not author the conflicting scene"));
		return false;
	}
	Policy->Conflict = EMjConflict::error;
	SceneOption->Timestep = 0.01;
	RobotOption->Timestep = 0.001;
	Ball->Type = EMjGeomType::sphere;
	Ball->Size = TArray<double>({0.1});

	mjspec::FMjSceneSpecBuilder Builder;
	Builder.SetSceneRoot(FSpecRef::OverActor(*Manager));
	mjspec::FMjSceneSpecParticipant Placed;
	Placed.Spec = FSpecRef::OverActor(*Robot);
	Placed.Prefix = TEXT("p0_");
	Builder.AddParticipant(Placed);

	// Compiled directly rather than through the helper above: the errors are
	// the subject here, not a failure to report.
	const mjspec::FMjCompiledScene Scene = Builder.Compile();
	TestFalse(TEXT("a refused attach produces no model"), Scene.IsValid());
	if (!TestTrue(TEXT("and it is reported"), Scene.Errors.Num() > 0))
	{
		return false;
	}

	FString Refusal;
	for (const FMjSpecDiagnostic& Error : Scene.Errors)
	{
		if (Error.Message.Contains(TEXT("could not attach participant 'p0_'")))
		{
			Refusal = Error.Message;
		}
	}
	if (!TestFalse(TEXT("the diagnostic names the participant that could not be attached"), Refusal.IsEmpty()))
	{
		return false;
	}

	// The two halves of "correct": a reason at all, and MuJoCo's own reason.
	// Reading it off the spec after the attach used to be able to come back
	// with neither, and a caller told only that something failed has nowhere
	// to go.
	TestFalse(TEXT("the diagnostic carries a reason rather than the fallback"),
		Refusal.Contains(TEXT("no reason given")));
	TestTrue(TEXT("and the reason names the field the two documents conflicted on"),
		Refusal.Contains(TEXT("timestep")));

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
