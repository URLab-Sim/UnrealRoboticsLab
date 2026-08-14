// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// The whole import, over models nobody wrote for a test.
//
// Everything else about materials and meshes is asserted over inline MJCF with
// assets placed by hand, which proves the lookup but assumes the import. This
// runs the real thing: a real file on disk, Unreal's own importers, real PNGs
// and OBJs, and then reads what the geoms ended up wearing.
//
// Both fixtures are optional. They live outside the plugin -- a menagerie
// checkout and a card deck -- so a machine without them skips rather than fails,
// and the paths can be overridden by environment variable. What they cannot do
// is prove appearance: there are no pixels in a headless run. They prove that
// the asset exists, that the geom points at it, and that the texture reached
// the material instance the renderer will sample.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Components/StaticMeshComponent.h"
#include "Engine/Blueprint.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#include "Engine/StaticMesh.h"
#include "Engine/Texture2D.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Misc/FileHelper.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"
#include "UObject/Package.h"

#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Spec/MjAssetResolve.h"
#include "MuJoCo/Spec/MjAssetSink.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Elements/MjGeom.h"
#include "MuJoCo/Elements/MjMesh.h"

#include "MujocoGenerationAction.h"

namespace MjRealModelImportTests
{

/** The fixture path, from the environment when set and from the default when not. */
FString FixturePath(const TCHAR* EnvVar, const TCHAR* Fallback)
{
	const FString FromEnv = FPlatformMisc::GetEnvironmentVariable(EnvVar);
	return FromEnv.IsEmpty() ? FString(Fallback) : FromEnv;
}

/**
 * Import `XmlPath` the way the editor's own menu action does.
 *
 * Through UMujocoGenerationAction rather than through a parse alone, because
 * the asset half is exactly what is under test: the parse would happen either
 * way, and it is the import pass that has to produce the UAssets.
 */
UBlueprint* Import(FAutomationTestBase& Test, const FString& XmlPath)
{
	const FString Name = FString::Printf(TEXT("MjReal_%s"), *FGuid::NewGuid().ToString(EGuidFormats::Digits));
	UPackage* Package = CreatePackage(*(TEXT("/Temp/") + Name));
	UBlueprint* Blueprint = FKismetEditorUtilities::CreateBlueprint(AMjArticulation::StaticClass(), Package,
		FName(*Name), BPTYPE_Normal, UBlueprint::StaticClass(), UBlueprintGeneratedClass::StaticClass());
	if (Blueprint == nullptr)
	{
		Test.AddError(TEXT("could not create a scratch Blueprint"));
		return nullptr;
	}

	UMujocoGenerationAction* Action = NewObject<UMujocoGenerationAction>();
	if (!Action->GenerateForBlueprint(Blueprint, XmlPath))
	{
		Test.AddError(FString::Printf(TEXT("import failed for %s"), *XmlPath));
		return nullptr;
	}
	return Blueprint;
}

/**
 * The spawned geom wearing `MaterialName`, with its preview components built.
 *
 * A construction-script template never registers, so it never builds a preview.
 * Spawning the compiled class is what puts real UStaticMeshComponents in a real
 * world -- which is as close to "it renders" as a headless run reaches, since
 * everything past this point is the renderer's own business.
 */
UMjGeom* SpawnAndFindGeom(UWorld& World, UBlueprint& Blueprint, const TCHAR* MaterialName)
{
	if (Blueprint.GeneratedClass == nullptr)
	{
		return nullptr;
	}
	AActor* Actor = World.SpawnActor<AActor>(Blueprint.GeneratedClass);
	if (Actor == nullptr)
	{
		return nullptr;
	}
	TArray<UMjGeom*> Geoms;
	Actor->GetComponents(Geoms);
	for (UMjGeom* Geom : Geoms)
	{
		if (Geom != nullptr && !Geom->IsClassPartial() && Geom->EffectiveMaterialName() == MaterialName)
		{
			return Geom;
		}
	}
	return nullptr;
}

/** The dynamic instance the geom's preview is wearing, or null. */
UMaterialInstanceDynamic* PreviewMaterial(UMjGeom* Geom)
{
	UStaticMeshComponent* Mesh = Geom != nullptr ? Geom->GetVisualizerMesh() : nullptr;
	return Mesh != nullptr ? Cast<UMaterialInstanceDynamic>(Mesh->GetMaterial(0)) : nullptr;
}

/** The construction-script template of the geom named `MjName`. */
UMjGeom* GeomNamed(UBlueprint& Blueprint, const TCHAR* MjName)
{
	if (Blueprint.SimpleConstructionScript == nullptr)
	{
		return nullptr;
	}
	for (USCS_Node* Node : Blueprint.SimpleConstructionScript->GetAllNodes())
	{
		UMjGeom* Geom = Node != nullptr ? Cast<UMjGeom>(Node->ComponentTemplate) : nullptr;
		if (Geom != nullptr && Geom->MjName.IsSet() && Geom->MjName.GetValue() == MjName)
		{
			return Geom;
		}
	}
	return nullptr;
}

/** Every `<mesh>` element of the Blueprint's spec, in construction order. */
TArray<UMjMesh*> MeshElements(UBlueprint& Blueprint)
{
	TArray<UMjMesh*> Out;
	if (Blueprint.SimpleConstructionScript == nullptr)
	{
		return Out;
	}
	for (USCS_Node* Node : Blueprint.SimpleConstructionScript->GetAllNodes())
	{
		if (UMjMesh* Mesh = Node != nullptr ? Cast<UMjMesh>(Node->ComponentTemplate) : nullptr)
		{
			Out.Add(Mesh);
		}
	}
	return Out;
}

/** The first geom whose effective material resolves to `MaterialName`. */
UMjGeom* GeomWearing(UBlueprint& Blueprint, const TCHAR* MaterialName)
{
	if (Blueprint.SimpleConstructionScript == nullptr)
	{
		return nullptr;
	}
	for (USCS_Node* Node : Blueprint.SimpleConstructionScript->GetAllNodes())
	{
		UMjGeom* Geom = Node != nullptr ? Cast<UMjGeom>(Node->ComponentTemplate) : nullptr;
		if (Geom != nullptr && !Geom->IsClassPartial() && Geom->EffectiveMaterialName() == MaterialName)
		{
			return Geom;
		}
	}
	return nullptr;
}

} // namespace MjRealModelImportTests

// ============================================================================
// URLab.Import.CardsDeckTexturesBecomeAssets
//   52 real PNGs in the `<material texture=>` shorthand, on mesh geoms whose
//   OBJ is shared between every card. The acceptance case for file textures.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCardsImportTest, "URLab.Import.CardsDeckTexturesBecomeAssets",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjCardsImportTest::RunTest(const FString& Parameters)
{
	using namespace MjRealModelImportTests;

	const FString Xml = FixturePath(
		TEXT("URLAB_CARDS_XML"), TEXT("C:/Users/jonat/ParticipantXml/GitHub/mujoco-urlab/model/cards/cards.xml"));
	if (!FPaths::FileExists(Xml))
	{
		AddWarning(FString::Printf(TEXT("Skipping: no cards fixture at %s (set URLAB_CARDS_XML)"), *Xml));
		return true;
	}

	UBlueprint* Blueprint = Import(*this, Xml);
	if (Blueprint == nullptr)
	{
		return false;
	}
	const FSpecRef Spec = FSpecRef::OverBlueprint(*Blueprint);

	// The deck writes `<texture type="2d" file="2_of_clubs.png"/>` with no name
	// at all, so the name the material refers to is the one MuJoCo derives from
	// the filename. Naming the asset after the file would have worked here by
	// accident; naming it after the element is what makes both spellings work.
	UTexture2D* Face = MjResolveTexture(Spec, TEXT("2_of_clubs"));
	TestNotNull(TEXT("an unnamed file texture imported under its derived name"), Face);

	// The card mesh is shared by all 52 geoms and named after its file.
	const FMjResolvedMesh Card = MjResolveMesh(Spec, TEXT("card"));
	TestNotNull(TEXT("the card OBJ imported as a static mesh"), Card.Asset);

	UMjGeom* Geom = GeomWearing(*Blueprint, TEXT("2_of_clubs"));
	if (TestNotNull(TEXT("a geom wearing the 2_of_clubs material"), Geom))
	{
		FMjMaterialValues Values;
		TestTrue(TEXT("its material resolves"), MjResolveMaterial(Spec, TEXT("2_of_clubs"), Values));
		TestEqual(TEXT("through the shorthand's rgb layer"), Values.TextureFor(EMjMaterialRole::Rgb),
			FString(TEXT("2_of_clubs")));
		TestTrue(TEXT("and that layer resolves to the imported asset"),
			MjResolveTexture(Spec, Values.TextureFor(EMjMaterialRole::Rgb)) == Face);
	}

	// The rendering half: the card's mesh geom, spawned, holding the shared
	// card OBJ and a material instance sampling its own face image.
	UWorld* World = UWorld::CreateWorld(EWorldType::Editor, /*bInformEngineOfWorld=*/false, TEXT("MjCardsWorld"));
	if (TestNotNull(TEXT("a scratch world"), World))
	{
		UMjGeom* Spawned = SpawnAndFindGeom(*World, *Blueprint, TEXT("2_of_clubs"));
		if (TestNotNull(TEXT("the spawned card geom"), Spawned) && TestNotNull(TEXT("it built a preview mesh component"), Spawned->GetVisualizerMesh()))
		{
			TestTrue(TEXT("the component draws the imported card OBJ"),
				Spawned->GetVisualizerMesh()->GetStaticMesh() == Card.Asset);

			UTexture* Bound = nullptr;
			if (UMaterialInstanceDynamic* Instance = PreviewMaterial(Spawned))
			{
				Instance->GetTextureParameterValue(FName(TEXT("RgbTexture")), Bound);
			}
			TestTrue(TEXT("and its material instance samples that card's own face"), Bound == Face);
		}
		World->DestroyWorld(false);
	}
	return true;
}

// ============================================================================
// URLab.Import.SkydioX2RendersItsTexturedMesh
//   One OBJ, one PNG, a non-uniform <mesh scale> and a quat on the geom. The
//   smallest menagerie model that exercises the whole chain at once.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSkydioImportTest, "URLab.Import.SkydioX2RendersItsTexturedMesh",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSkydioImportTest::RunTest(const FString& Parameters)
{
	using namespace MjRealModelImportTests;

	const FString Xml = FixturePath(
		TEXT("URLAB_X2_XML"), TEXT("C:/Users/jonat/ParticipantXml/GitHub/mujoco_menagerie/skydio_x2/x2.xml"));
	if (!FPaths::FileExists(Xml))
	{
		AddWarning(FString::Printf(TEXT("Skipping: no skydio_x2 fixture at %s (set URLAB_X2_XML)"), *Xml));
		return true;
	}

	UBlueprint* Blueprint = Import(*this, Xml);
	if (Blueprint == nullptr)
	{
		return false;
	}
	const FSpecRef Spec = FSpecRef::OverBlueprint(*Blueprint);

	const FMjResolvedMesh Body = MjResolveMesh(Spec, TEXT("X2_lowpoly"));
	TestNotNull(TEXT("the drone's OBJ imported as a static mesh"), Body.Asset);

	// `<mesh class="x2">` puts scale="0.01 0.01 0.01" on the class, so the
	// element authors none of it and the chain has to supply it. With the
	// metre-to-centimetre factor that lands on 1.
	TestTrue(FString::Printf(TEXT("a class-inherited mesh scale composes to 1, got %s"), *Body.Scale.ToString()),
		Body.Scale.Equals(FVector(1.0), 1e-4));

	UTexture2D* Skin = MjResolveTexture(Spec, TEXT("X2_lowpoly_texture_SpinningProps_1024"));
	TestNotNull(TEXT("the drone's PNG imported as a texture"), Skin);

	UMjGeom* Geom = GeomWearing(*Blueprint, TEXT("phong3SG"));
	if (TestNotNull(TEXT("a geom wearing the phong3SG material"), Geom))
	{
		TestEqual(TEXT("its mesh reference resolves through the class chain"), Geom->EffectiveMeshName(),
			FString(TEXT("X2_lowpoly")));

		FMjMaterialValues Values;
		TestTrue(TEXT("its material resolves"), MjResolveMaterial(Spec, TEXT("phong3SG"), Values));
		TestTrue(TEXT("and its rgb layer resolves to the imported texture"),
			MjResolveTexture(Spec, Values.TextureFor(EMjMaterialRole::Rgb)) == Skin);
	}

	// The rendering half: a real component in a real world, holding the mesh
	// the OBJ became and a material instance sampling the PNG.
	UWorld* World = UWorld::CreateWorld(EWorldType::Editor, /*bInformEngineOfWorld=*/false, TEXT("MjX2World"));
	if (TestNotNull(TEXT("a scratch world"), World))
	{
		UMjGeom* Spawned = SpawnAndFindGeom(*World, *Blueprint, TEXT("phong3SG"));
		if (TestNotNull(TEXT("the spawned drone body geom"), Spawned) && TestNotNull(TEXT("it built a preview mesh component"), Spawned->GetVisualizerMesh()))
		{
			TestTrue(TEXT("the component draws the imported OBJ"),
				Spawned->GetVisualizerMesh()->GetStaticMesh() == Body.Asset);
			TestTrue(TEXT("the component is visible"), Spawned->GetVisualizerMesh()->IsVisible());

			UTexture* Bound = nullptr;
			if (UMaterialInstanceDynamic* Instance = PreviewMaterial(Spawned))
			{
				Instance->GetTextureParameterValue(FName(TEXT("RgbTexture")), Bound);
			}
			TestTrue(TEXT("and its material instance samples the imported PNG"), Bound == Skin);
		}
		World->DestroyWorld(false);
	}
	return true;
}

// ============================================================================
// URLab.Import.SpotMeshesResolveWhicheverWayUnrealFilesThem
//   The model the path guess died on. Spot's `spot_ue.xml` names 50-odd OBJs,
//   and the mesh-preparation step leaves a GLB beside most of them; the GLB
//   wins, Interchange rather than the FBX factory imports it, and the asset
//   lands at `Meshes/<name>/StaticMeshes/<name>` instead of `Meshes/<name>`.
//   One model, two layouts, and no rule that predicts which -- so this asserts
//   the only thing that is knowable: the element points at what was imported,
//   and what it points at is really on disk.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSpotImportTest, "URLab.Import.SpotMeshesResolveWhicheverWayUnrealFilesThem",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSpotImportTest::RunTest(const FString& Parameters)
{
	using namespace MjRealModelImportTests;

	const FString Xml = FixturePath(TEXT("URLAB_SPOT_XML"),
		TEXT("C:/Users/jonat/ParticipantXml/GitHub/mujoco_menagerie/boston_dynamics_spot/spot_ue.xml"));
	if (!FPaths::FileExists(Xml))
	{
		AddWarning(FString::Printf(TEXT("Skipping: no spot fixture at %s (set URLAB_SPOT_XML)"), *Xml));
		return true;
	}

	UBlueprint* Blueprint = Import(*this, Xml);
	if (Blueprint == nullptr)
	{
		return false;
	}

	const TArray<UMjMesh*> Meshes = MeshElements(*Blueprint);
	if (!TestTrue(TEXT("the model has mesh elements"), Meshes.Num() > 0))
	{
		return false;
	}

	int32 Nested = 0;
	int32 Flat = 0;
	for (UMjMesh* Mesh : Meshes)
	{
		const FString Name = MjAssetElementName(*Mesh);
		if (!TestNotNull(*FString::Printf(TEXT("<mesh '%s'> references an asset"), *Name), Mesh->MeshAsset.Get()))
		{
			continue;
		}

		// The reference has to name a saved asset and not something still only
		// in memory: a package path that loads back to the same object is the
		// difference between an import that landed and one that merely ran.
		const FString ObjectPath = Mesh->MeshAsset->GetPathName();
		TestTrue(*FString::Printf(TEXT("<mesh '%s'> references a saved asset at %s"), *Name, *ObjectPath),
			LoadObject<UStaticMesh>(nullptr, *ObjectPath, nullptr, LOAD_NoWarn | LOAD_Quiet) == Mesh->MeshAsset.Get());
		TestTrue(*FString::Printf(TEXT("<mesh '%s'> has drawable geometry"), *Name),
			Mesh->MeshAsset->GetNumTriangles(0) > 0);

		(ObjectPath.Contains(TEXT("/StaticMeshes/")) ? Nested : Flat) += 1;
	}

	// Both layouts in one model is the case the old guess could not survive. A
	// checkout whose meshes were never prepared has no GLBs and so no nested
	// assets, which is worth saying out loud rather than asserting away.
	AddInfo(FString::Printf(TEXT("%d nested and %d flat mesh assets"), Nested, Flat));
	if (Nested == 0)
	{
		AddWarning(TEXT("no mesh imported through Interchange: the nested layout went untested"));
	}

	// And the visual half: a spawned geom drawing the asset its own <mesh> holds.
	const FSpecRef Spec = FSpecRef::OverBlueprint(*Blueprint);
	UWorld* World = UWorld::CreateWorld(EWorldType::Editor, /*bInformEngineOfWorld=*/false, TEXT("MjSpotWorld"));
	if (TestNotNull(TEXT("a scratch world"), World))
	{
		UMjGeom* Spawned = SpawnAndFindGeom(*World, *Blueprint, TEXT("BlackAbs"));
		if (TestNotNull(TEXT("a spawned visual geom"), Spawned) && TestNotNull(TEXT("it built a preview mesh component"), Spawned->GetVisualizerMesh()))
		{
			const FMjResolvedMesh Resolved = MjResolveMesh(Spec, Spawned->EffectiveMeshName());
			TestNotNull(TEXT("its <mesh> resolves to an asset"), Resolved.Asset);
			TestTrue(TEXT("and the component draws that asset"),
				Spawned->GetVisualizerMesh()->GetStaticMesh() == Resolved.Asset);
		}
		World->DestroyWorld(false);
	}
	return true;
}

#endif // URLAB_MJ_GEN && WITH_EDITOR
