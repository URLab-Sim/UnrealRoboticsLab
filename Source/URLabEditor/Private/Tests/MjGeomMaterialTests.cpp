// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// The material a geom is meant to wear, textures included.
//
// The colour half of this was already resolved; the picture was not. A geom's
// material reference is four hops from the image -- geom to class to material
// to layer to texture -- and the preview walked none of them past the material,
// so a model whose materials carry real images drew flat colour.
//
// Appearance itself is not assertable headless: there is no frame to sample and
// no RHI to sample it with. What IS assertable is the material instance the
// preview builds, and that is what these read. A texture that reaches the right
// parameter of the right instance is a texture the renderer will sample.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Components/StaticMeshComponent.h"
#include "Engine/Engine.h"
#include "Engine/Texture2D.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Misc/Guid.h"
#include "UObject/Package.h"

#include "Engine/StaticMesh.h"

#include "MuJoCo/Spec/MjAssetResolve.h"
#include "MuJoCo/Spec/MjAssetSink.h"
#include "MuJoCo/Spec/MjCompile.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Elements/MjGeom.h"
#include "MuJoCo/Elements/MjMesh.h"
#include "MuJoCo/Elements/MjTexture.h"
#include "MuJoCo/Gen/Elements/Assets/MjMaterialLayer.gen.h"

namespace MjGeomMaterialTests
{

/** A spec over a live actor, so the preview components really register. */
struct FScratchDoc
{
	UWorld* World = nullptr;
	AActor* Actor = nullptr;

	/** The model stem, which keeps one test's scratch packages out of another's. */
	FString Stem;

	~FScratchDoc()
	{
		if (World != nullptr)
		{
			World->DestroyWorld(false);
		}
	}
};

bool Parse(FAutomationTestBase& Test, FScratchDoc& Doc, const FString& Xml)
{
	Doc.Stem = FString::Printf(TEXT("MjMat_%s"), *FGuid::NewGuid().ToString(EGuidFormats::Digits));
	Doc.World = UWorld::CreateWorld(EWorldType::Editor, /*bInformEngineOfWorld=*/false, FName(*Doc.Stem));
	if (Doc.World == nullptr)
	{
		Test.AddError(TEXT("could not create a scratch world"));
		return false;
	}
	Doc.Actor = Doc.World->SpawnActor<AActor>();
	if (Doc.Actor == nullptr)
	{
		Test.AddError(TEXT("could not spawn a scratch actor"));
		return false;
	}
	// Named after a file it does not have: the source path is what an asset's
	// own `file` resolves against, and every one of these specs states one.
	const FMjSpecParseResult Parsed = MjParseIntoActor(*Doc.Actor, Xml, Doc.Stem + TEXT(".xml"));
	if (!Parsed.IsOk())
	{
		Test.AddError(TEXT("parse failed"));
		return false;
	}
	return true;
}

/** The asset element `Name` refers to, by MuJoCo's own naming rule. */
template <class T>
T* AssetElement(const FScratchDoc& Doc, const TCHAR* Name)
{
	TArray<T*> Found;
	Doc.Actor->GetComponents(Found);
	for (T* Element : Found)
	{
		if (Element != nullptr && MjAssetElementName(*Element) == Name)
		{
			return Element;
		}
	}
	return nullptr;
}

/**
 * An asset handed to the element that references it, as the import pass leaves it.
 *
 * The import pass itself wants an image on disk and Unreal's own importers;
 * what these tests are about is what the spec then does with the result, so
 * the asset is made here and given to the element the same way. It goes to a
 * scratch package deliberately: nothing may find it by rebuilding a path out of
 * the model's name, and a package no such rule could ever name is what proves
 * nothing does.
 */
template <class Asset>
Asset* MakeScratchAsset(FAutomationTestBase& Test, const FScratchDoc& Doc, const TCHAR* MjName)
{
	const FString Name = MjSanitizeAssetName(MjName);
	UPackage* Package = CreatePackage(*(TEXT("/Temp/MjScratchAssets") / Doc.Stem / Name));
	if (Package == nullptr)
	{
		Test.AddError(TEXT("could not create a scratch package"));
		return nullptr;
	}
	Package->FullyLoad();
	return NewObject<Asset>(Package, FName(*Name), RF_Public | RF_Standalone);
}

UTexture2D* PlaceTexture(FAutomationTestBase& Test, const FScratchDoc& Doc, const TCHAR* MjName)
{
	UMjTexture* Element = AssetElement<UMjTexture>(Doc, MjName);
	if (Element == nullptr)
	{
		Test.AddError(FString::Printf(TEXT("the spec has no <texture> named '%s'"), MjName));
		return nullptr;
	}
	UTexture2D* Made = MakeScratchAsset<UTexture2D>(Test, Doc, MjName);
	Element->TextureAsset = Made;
	Element->FileAsset = FSoftObjectPath(Made);
	return Made;
}

UStaticMesh* PlaceMesh(FAutomationTestBase& Test, const FScratchDoc& Doc, const TCHAR* MjName)
{
	UMjMesh* Element = AssetElement<UMjMesh>(Doc, MjName);
	if (Element == nullptr)
	{
		Test.AddError(FString::Printf(TEXT("the spec has no <mesh> named '%s'"), MjName));
		return nullptr;
	}
	UStaticMesh* Made = MakeScratchAsset<UStaticMesh>(Test, Doc, MjName);
	Element->MeshAsset = Made;
	Element->FileAsset = FSoftObjectPath(Made);
	return Made;
}

template <class T>
T* Named(AActor& Actor, const TCHAR* MjName)
{
	TArray<T*> Found;
	Actor.GetComponents(Found);
	for (T* Element : Found)
	{
		if (Element != nullptr && Element->MjName.IsSet() && Element->MjName.GetValue() == MjName)
		{
			return Element;
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

/** Tell the element it was edited, the way the details panel does. */
void NotifyEdited(UMjNodeComponent& Node, const TCHAR* PropertyName)
{
	FProperty* Property = Node.GetClass()->FindPropertyByName(FName(PropertyName));
	FPropertyChangedEvent Event(Property);
	Node.PostEditChangeProperty(Event);
}

void TextureIs(FAutomationTestBase& Test, const TCHAR* Label, UMaterialInstanceDynamic* Instance, const TCHAR* Parameter,
	UTexture2D* Expected)
{
	if (Instance == nullptr)
	{
		Test.AddError(FString::Printf(TEXT("%s: the preview has no material instance"), Label));
		return;
	}
	UTexture* Actual = nullptr;
	Instance->GetTextureParameterValue(FName(Parameter), Actual);
	Test.TestTrue(FString::Printf(TEXT("%s: %s is %s, got %s"), Label, Parameter,
					  Expected != nullptr ? *Expected->GetName() : TEXT("null"),
					  Actual != nullptr ? *Actual->GetName() : TEXT("null")),
		Actual == Expected);
}

void ScalarIs(FAutomationTestBase& Test, const TCHAR* Label, UMaterialInstanceDynamic* Instance, const TCHAR* Parameter,
	float Expected)
{
	if (Instance == nullptr)
	{
		Test.AddError(FString::Printf(TEXT("%s: the preview has no material instance"), Label));
		return;
	}
	float Actual = 0.0f;
	Instance->GetScalarParameterValue(FName(Parameter), Actual);
	Test.TestTrue(FString::Printf(TEXT("%s: %s is %f, got %f"), Label, Parameter, Expected, Actual),
		FMath::IsNearlyEqual(Actual, Expected, 1e-4f));
}

}  // namespace MjGeomMaterialTests

// ============================================================================
// URLab.Import.GeomTextureReachesThePreviewMaterial
//   `<material texture="x">` is the shorthand every file-textured model uses
//   (the cards deck writes exactly this 52 times). The reader folds it into a
//   `<layer role="rgb">` before the spec exists, so the shorthand and the
//   PBR form are one path -- and this asserts that path end to end.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjGeomTextureTest, "URLab.Import.GeomTextureReachesThePreviewMaterial",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjGeomTextureTest::RunTest(const FString& Parameters)
{
	using namespace MjGeomMaterialTests;

	const FString Xml = TEXT(R"(<mujoco model="cards">
  <asset>
    <texture name="face" type="2d" file="face.png"/>
    <material name="card" texture="face"/>
  </asset>
  <worldbody>
    <body name="b">
      <geom name="card_geom" type="box" size="0.03 0.05 0.001" material="card"/>
      <geom name="plain_geom" type="box" size="0.03 0.05 0.001"/>
    </body>
  </worldbody>
</mujoco>
)");

	FScratchDoc Doc;
	if (!Parse(*this, Doc, Xml))
	{
		return false;
	}
	UTexture2D* Face = PlaceTexture(*this, Doc, TEXT("face"));
	if (!TestNotNull(TEXT("the placed texture asset"), Face))
	{
		return false;
	}

	UMjGeom* Card = Named<UMjGeom>(*Doc.Actor, TEXT("card_geom"));
	UMjGeom* Plain = Named<UMjGeom>(*Doc.Actor, TEXT("plain_geom"));
	if (!TestNotNull(TEXT("geom card_geom"), Card) || !TestNotNull(TEXT("geom plain_geom"), Plain))
	{
		return false;
	}

	// The asset was placed after the geoms registered, so the first resolve
	// missed it. That is the ordinary editing case too, and the refresh is what
	// the spec's own invalidation would run.
	Card->RefreshPresentation();
	Plain->RefreshPresentation();

	TextureIs(*this, TEXT("a geom whose material carries a texture"), PreviewMaterial(Card), TEXT("RgbTexture"), Face);

	// And a geom with no material at all is not accidentally wearing it: every
	// slot is written on every apply, so a stale MID cannot keep an old image.
	UMaterialInstanceDynamic* PlainInstance = PreviewMaterial(Plain);
	if (TestNotNull(TEXT("the plain geom's material instance"), PlainInstance))
	{
		UTexture* Actual = nullptr;
		PlainInstance->GetTextureParameterValue(FName(TEXT("RgbTexture")), Actual);
		TestTrue(TEXT("a geom with no material samples the neutral default"), Actual != Face);
	}

	return true;
}

// ============================================================================
// URLab.Import.MaterialLayersFillEveryTextureRole
//   The PBR form. MJCF declares nine roles and the reworked master has a slot
//   for all nine, so every one is asserted here even though no model on this
//   machine uses more than `rgb`.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjMaterialRolesTest, "URLab.Import.MaterialLayersFillEveryTextureRole",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjMaterialRolesTest::RunTest(const FString& Parameters)
{
	using namespace MjGeomMaterialTests;

	const FString Xml = TEXT(R"(<mujoco model="pbr">
  <asset>
    <texture name="t_rgb" type="2d" file="rgb.png"/>
    <texture name="t_occlusion" type="2d" file="occlusion.png"/>
    <texture name="t_roughness" type="2d" file="roughness.png"/>
    <texture name="t_metallic" type="2d" file="metallic.png"/>
    <texture name="t_normal" type="2d" file="normal.png"/>
    <texture name="t_opacity" type="2d" file="opacity.png"/>
    <texture name="t_emissive" type="2d" file="emissive.png"/>
    <texture name="t_rgba" type="2d" file="rgba.png"/>
    <texture name="t_orm" type="2d" file="orm.png"/>
    <material name="full">
      <layer texture="t_rgb" role="rgb"/>
      <layer texture="t_occlusion" role="occlusion"/>
      <layer texture="t_roughness" role="roughness"/>
      <layer texture="t_metallic" role="metallic"/>
      <layer texture="t_normal" role="normal"/>
      <layer texture="t_opacity" role="opacity"/>
      <layer texture="t_emissive" role="emissive"/>
      <layer texture="t_rgba" role="rgba"/>
      <layer texture="t_orm" role="orm"/>
    </material>
  </asset>
  <worldbody>
    <body name="b">
      <geom name="g" type="box" size="0.1 0.1 0.1" material="full"/>
    </body>
  </worldbody>
</mujoco>
)");

	FScratchDoc Doc;
	if (!Parse(*this, Doc, Xml))
	{
		return false;
	}

	UTexture2D* Placed[static_cast<int32>(EMjMaterialRole::Count)] = {};
	for (int32 Index = 0; Index < static_cast<int32>(EMjMaterialRole::Count); ++Index)
	{
		const FString Name = FString(TEXT("t_")) + MjMaterialRoleName(static_cast<EMjMaterialRole>(Index));
		Placed[Index] = PlaceTexture(*this, Doc, *Name);
	}

	UMjGeom* Geom = Named<UMjGeom>(*Doc.Actor, TEXT("g"));
	if (!TestNotNull(TEXT("geom g"), Geom))
	{
		return false;
	}
	Geom->RefreshPresentation();

	UMaterialInstanceDynamic* Instance = PreviewMaterial(Geom);
	for (int32 Index = 0; Index < static_cast<int32>(EMjMaterialRole::Count); ++Index)
	{
		const EMjMaterialRole Role = static_cast<EMjMaterialRole>(Index);
		TextureIs(*this, MjMaterialRoleName(Role), Instance, MjMaterialRoleParameter(Role), Placed[Index]);
	}
	return true;
}

// ============================================================================
// URLab.Import.MaterialThroughDefaultClassResolvesTexture
//   The humanoid's arrangement, but with an image on the material rather than
//   only a colour: the geom authors nothing and the class names the material.
//   Also the shading scalars, including MuJoCo's -1 "the material does not say"
//   sentinel, and the texture scale `texrepeat` and `texuniform` decide.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjMaterialThroughClassTest, "URLab.Import.MaterialThroughDefaultClassResolvesTexture",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjMaterialThroughClassTest::RunTest(const FString& Parameters)
{
	using namespace MjGeomMaterialTests;

	const FString Xml = TEXT(R"(<mujoco model="inherit">
  <asset>
    <texture name="skin" type="2d" file="skin.png"/>
    <material name="painted" texture="skin" texrepeat="3 4" texuniform="true"
              specular="0.25" shininess="0.75" reflectance="0.2" emission="0.1"/>
    <material name="metal" texture="skin" metallic="0.9" roughness="0.2"/>
  </asset>
  <default>
    <default class="body">
      <geom type="box" size="2 5 1" material="painted"/>
    </default>
  </default>
  <worldbody>
    <body name="b" childclass="body">
      <geom name="inherited"/>
      <geom name="metal_geom" material="metal"/>
    </body>
  </worldbody>
</mujoco>
)");

	FScratchDoc Doc;
	if (!Parse(*this, Doc, Xml))
	{
		return false;
	}
	UTexture2D* Skin = PlaceTexture(*this, Doc, TEXT("skin"));

	UMjGeom* Inherited = Named<UMjGeom>(*Doc.Actor, TEXT("inherited"));
	UMjGeom* Metal = Named<UMjGeom>(*Doc.Actor, TEXT("metal_geom"));
	if (!TestNotNull(TEXT("geom inherited"), Inherited) || !TestNotNull(TEXT("geom metal_geom"), Metal))
	{
		return false;
	}
	Inherited->RefreshPresentation();
	Metal->RefreshPresentation();

	UMaterialInstanceDynamic* Instance = PreviewMaterial(Inherited);
	TextureIs(*this, TEXT("a texture reached through the class's material"), Instance, TEXT("RgbTexture"), Skin);

	ScalarIs(*this, TEXT("specular carries across"), Instance, TEXT("Specular"), 0.25f);
	ScalarIs(*this, TEXT("reflectance carries across"), Instance, TEXT("Reflectance"), 0.2f);
	ScalarIs(*this, TEXT("emission carries across"), Instance, TEXT("Emission"), 0.1f);

	// `roughness` is -1 here, which MuJoCo means as "unset". Filament reads
	// `shininess` as glossiness in that case, and Unreal's roughness is its
	// complement, so 0.75 shiny is 0.25 rough. `metallic` unset is not metal.
	ScalarIs(*this, TEXT("an unset roughness comes from shininess"), Instance, TEXT("Roughness"), 0.25f);
	ScalarIs(*this, TEXT("an unset metallic is not metal"), Instance, TEXT("Metallic"), 0.0f);

	// texuniform multiplies texrepeat by the geom's half-extent, which is how
	// MuJoCo keeps texel density constant across differently sized geoms
	// (render_gl3.c settexture). The box is 2 x 5.
	ScalarIs(*this, TEXT("texrepeat scaled by size in u"), Instance, TEXT("TexRepeatU"), 6.0f);
	ScalarIs(*this, TEXT("texrepeat scaled by size in v"), Instance, TEXT("TexRepeatV"), 20.0f);

	// A material that authors both takes them literally, and does not scale a
	// texrepeat it never asked to have scaled.
	UMaterialInstanceDynamic* MetalInstance = PreviewMaterial(Metal);
	ScalarIs(*this, TEXT("an authored metallic"), MetalInstance, TEXT("Metallic"), 0.9f);
	ScalarIs(*this, TEXT("an authored roughness"), MetalInstance, TEXT("Roughness"), 0.2f);
	ScalarIs(*this, TEXT("no texuniform, no scaling"), MetalInstance, TEXT("TexRepeatU"), 1.0f);

	return true;
}

// ============================================================================
// URLab.Import.EditingAnAssetElementRedrawsTheGeomsThatUseIt
//   `<layer>` and `<texture>` are inputs to a picture drawn somewhere else, and
//   nothing that draws them re-reads on its own. Both sit under `<asset>`, so
//   the spec's shared-input rule should already cover them -- these assert
//   that it does, for the two edits a user actually makes.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjMaterialEditRefreshTest,
	"URLab.Import.EditingAnAssetElementRedrawsTheGeomsThatUseIt",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjMaterialEditRefreshTest::RunTest(const FString& Parameters)
{
	using namespace MjGeomMaterialTests;

	const FString Xml = TEXT(R"(<mujoco model="edits">
  <asset>
    <texture name="first" type="2d" file="first.png"/>
    <texture name="second" type="2d" file="second.png"/>
    <material name="m" texture="first"/>
  </asset>
  <worldbody>
    <body name="b">
      <geom name="g" type="box" size="0.1 0.1 0.1" material="m"/>
    </body>
  </worldbody>
</mujoco>
)");

	FScratchDoc Doc;
	if (!Parse(*this, Doc, Xml))
	{
		return false;
	}
	UTexture2D* First = PlaceTexture(*this, Doc, TEXT("first"));
	UTexture2D* Second = PlaceTexture(*this, Doc, TEXT("second"));

	UMjGeom* Geom = Named<UMjGeom>(*Doc.Actor, TEXT("g"));
	UMjTexture* FirstElement = Named<UMjTexture>(*Doc.Actor, TEXT("first"));
	if (!TestNotNull(TEXT("geom g"), Geom) || !TestNotNull(TEXT("texture element first"), FirstElement))
	{
		return false;
	}
	Geom->RefreshPresentation();
	TextureIs(*this, TEXT("before any edit"), PreviewMaterial(Geom), TEXT("RgbTexture"), First);

	// The layer the shorthand created. Repointing it is the edit a user makes
	// when swapping which image a material uses.
	UMjMaterialLayer* Layer = nullptr;
	{
		TArray<UMjMaterialLayer*> Layers;
		Doc.Actor->GetComponents(Layers);
		Layer = Layers.Num() > 0 ? Layers[0] : nullptr;
	}
	if (!TestNotNull(TEXT("the layer the shorthand created"), Layer))
	{
		return false;
	}
	TestEqual(TEXT("the shorthand's layer takes the rgb role"), Layer->Role, FString(TEXT("rgb")));

	Layer->SetTexture(TEXT("second"));
	NotifyEdited(*Layer, TEXT("Texture"));
	TextureIs(*this, TEXT("after repointing the layer"), PreviewMaterial(Geom), TEXT("RgbTexture"), Second);

	// And the texture element itself: renaming it is what breaks the reference,
	// and the geom must notice rather than keep drawing the old image.
	FirstElement->MjName = FString(TEXT("renamed"));
	NotifyEdited(*FirstElement, TEXT("MjName"));
	Layer->SetTexture(TEXT("first"));
	NotifyEdited(*Layer, TEXT("Texture"));

	UMaterialInstanceDynamic* Instance = PreviewMaterial(Geom);
	if (TestNotNull(TEXT("the material instance after the rename"), Instance))
	{
		UTexture* Actual = nullptr;
		Instance->GetTextureParameterValue(FName(TEXT("RgbTexture")), Actual);
		TestTrue(TEXT("a layer pointing at a name nothing has samples the neutral default"),
			Actual != First && Actual != Second);
	}
	return true;
}

// ============================================================================
// URLab.Import.MeshGeomsDrawTheAssetTheirElementNames
//   A mesh geom had no preview at all: the shape table has no row for it, and
//   nothing ever loaded the asset the import pass had already made. This is the
//   same name-based lookup the textures use, one element later -- plus the two
//   scales that compose onto it, and the case MJCF allows where the element's
//   name and its file's differ.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjMeshGeomTest, "URLab.Import.MeshGeomsDrawTheAssetTheirElementNames",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjMeshGeomTest::RunTest(const FString& Parameters)
{
	using namespace MjGeomMaterialTests;

	const FString Xml = TEXT(R"(<mujoco model="drone">
  <asset>
    <texture name="skin" type="2d" file="skin.png"/>
    <material name="phong3SG" texture="skin"/>
    <mesh name="tabletop" file="table.obj" scale="0.01 0.02 0.04"/>
    <mesh file="X2_lowpoly.obj"/>
  </asset>
  <default>
    <default class="visual">
      <geom type="mesh" mesh="X2_lowpoly"/>
    </default>
  </default>
  <worldbody>
    <body name="b">
      <geom name="scaled" type="mesh" mesh="tabletop"/>
      <geom name="from_class" class="visual" material="phong3SG" quat="0 0 1 1"/>
      <geom name="missing" type="mesh" mesh="nothing_named_this"/>
    </body>
  </worldbody>
</mujoco>
)");

	FScratchDoc Doc;
	if (!Parse(*this, Doc, Xml))
	{
		return false;
	}
	UStaticMesh* Tabletop = PlaceMesh(*this, Doc, TEXT("tabletop"));
	UStaticMesh* Lowpoly = PlaceMesh(*this, Doc, TEXT("X2_lowpoly"));
	UTexture2D* Skin = PlaceTexture(*this, Doc, TEXT("skin"));
	if (!TestNotNull(TEXT("the placed tabletop asset"), Tabletop) ||
		!TestNotNull(TEXT("the placed X2_lowpoly asset"), Lowpoly))
	{
		return false;
	}

	UMjGeom* Scaled = Named<UMjGeom>(*Doc.Actor, TEXT("scaled"));
	UMjGeom* FromClass = Named<UMjGeom>(*Doc.Actor, TEXT("from_class"));
	UMjGeom* Missing = Named<UMjGeom>(*Doc.Actor, TEXT("missing"));
	if (!TestNotNull(TEXT("geom scaled"), Scaled) || !TestNotNull(TEXT("geom from_class"), FromClass) ||
		!TestNotNull(TEXT("geom missing"), Missing))
	{
		return false;
	}
	Scaled->RefreshPresentation();
	FromClass->RefreshPresentation();
	Missing->RefreshPresentation();

	// The element is named `tabletop` and its file is `table.obj`. A lookup by
	// filename would find nothing, which is the bug this shape of fixture
	// exists to catch.
	if (TestNotNull(TEXT("the scaled geom has a preview mesh"), Scaled->GetVisualizerMesh()))
	{
		TestTrue(TEXT("a mesh geom draws the asset its element names"),
			Scaled->GetVisualizerMesh()->GetStaticMesh() == Tabletop);

		// The element's own `scale`, which MuJoCo would have baked into the
		// vertices, and nothing else. Units are the import's job: every mesh
		// goes in as glTF and Interchange converts metres to centimetres, so a
		// factor here too would scale the model by a hundred.
		const FVector Expected(0.01, 0.02, 0.04);
		TestTrue(FString::Printf(TEXT("<mesh scale> alone gives %s, got %s"), *Expected.ToString(),
					 *Scaled->GetVisualizerMesh()->GetRelativeScale3D().ToString()),
			Scaled->GetVisualizerMesh()->GetRelativeScale3D().Equals(Expected, 1e-4));
	}

	// `mesh` off a default class, and a material on the geom: the x2 drone's
	// arrangement, and the one that renders nothing if either hop is missed.
	if (TestNotNull(TEXT("the class-driven geom has a preview mesh"), FromClass->GetVisualizerMesh()))
	{
		TestTrue(TEXT("a mesh reached through a default class resolves"),
			FromClass->GetVisualizerMesh()->GetStaticMesh() == Lowpoly);

		// An element that authors no `scale` previews at unit scale: the asset
		// already arrived in the level's units.
		TestTrue(TEXT("an unscaled mesh element previews unscaled"),
			FromClass->GetVisualizerMesh()->GetRelativeScale3D().Equals(FVector::OneVector, 1e-4));

		TextureIs(*this, TEXT("a textured mesh geom"), PreviewMaterial(FromClass), TEXT("RgbTexture"), Skin);
	}

	// A reference to a mesh the spec does not have draws nothing rather
	// than picking up whatever asset happens to share the name.
	TestNull(TEXT("a dangling mesh reference builds no preview"), Missing->GetVisualizerMesh());
	return true;
}

// ============================================================================
// URLab.Import.PlaneAndEllipsoidGeomsHaveAPreview
//   Both were empty rows in the shape table, so neither drew anything. A plane
//   is where a floor material is, which made it the one surface a texture was
//   most likely to be looked at on.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjPlaneEllipsoidTest, "URLab.Import.PlaneAndEllipsoidGeomsHaveAPreview",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjPlaneEllipsoidTest::RunTest(const FString& Parameters)
{
	using namespace MjGeomMaterialTests;

	const FString Xml = TEXT(R"(<mujoco model="shapes">
  <worldbody>
    <body name="b">
      <geom name="finite_plane" type="plane" size="3 4 0.1"/>
      <geom name="infinite_plane" type="plane" size="0 0 0.05"/>
      <geom name="egg" type="ellipsoid" size="0.1 0.2 0.3"/>
    </body>
  </worldbody>
</mujoco>
)");

	FScratchDoc Doc;
	if (!Parse(*this, Doc, Xml))
	{
		return false;
	}

	UMjGeom* Finite = Named<UMjGeom>(*Doc.Actor, TEXT("finite_plane"));
	UMjGeom* Infinite = Named<UMjGeom>(*Doc.Actor, TEXT("infinite_plane"));
	UMjGeom* Egg = Named<UMjGeom>(*Doc.Actor, TEXT("egg"));
	if (!TestNotNull(TEXT("geom finite_plane"), Finite) || !TestNotNull(TEXT("geom infinite_plane"), Infinite) ||
		!TestNotNull(TEXT("geom egg"), Egg))
	{
		return false;
	}

	TestNotNull(TEXT("a plane has a preview mesh"), Finite->GetVisualizerMesh());
	TestNotNull(TEXT("an ellipsoid has a preview mesh"), Egg->GetVisualizerMesh());

	// A plane's size is (half-x, half-y, grid-spacing): the third component is
	// MuJoCo's reference grid and must not reach the scale, and Z stays 1
	// because a plane has no thickness.
	FVector Scale;
	if (TestTrue(TEXT("a finite plane resolves a preview scale"), Finite->TryPreviewScaleFromSpec(Scale)))
	{
		TestTrue(FString::Printf(TEXT("a 3 x 4 plane is (6, 8, 1), got %s"), *Scale.ToString()),
			Scale.Equals(FVector(6.0, 8.0, 1.0), 1e-4));
	}

	// A zero half-extent means infinite, which Unreal cannot draw, so it
	// previews at a fixed extent rather than collapsing to nothing.
	if (TestTrue(TEXT("an infinite plane resolves a preview scale"), Infinite->TryPreviewScaleFromSpec(Scale)))
	{
		TestTrue(FString::Printf(TEXT("an infinite plane previews finite and square, got %s"), *Scale.ToString()),
			Scale.X > 0.0 && FMath::IsNearlyEqual(Scale.X, Scale.Y) && FMath::IsNearlyEqual(Scale.Z, 1.0));
	}

	// And it never authors one back: there is no scale that preserves both the
	// grid spacing and the zero that means infinite.
	TestFalse(TEXT("a plane's size is read-only to the scale gizmo"), Finite->HasScaleMapping());

	// An ellipsoid is a sphere with three independent radii, unlike the sphere
	// row it shares a mesh with.
	if (TestTrue(TEXT("an ellipsoid resolves a preview scale"), Egg->TryPreviewScaleFromSpec(Scale)))
	{
		TestTrue(FString::Printf(TEXT("an ellipsoid scales per axis, got %s"), *Scale.ToString()),
			Scale.Equals(FVector(0.2, 0.4, 0.6), 1e-4));
	}
	TestTrue(TEXT("an ellipsoid's size is authorable"), Egg->HasScaleMapping());
	return true;
}

// ============================================================================
// URLab.Import.ADanglingTextureReferenceResolvesToNothing
//
// A layer naming a texture the spec does not have must resolve to null.
//
// It is the one thing the name-keyed lookup could never get right: assets from
// every model share one namespace, so a name that matched something an earlier
// import left behind produced that image instead of nothing. Resolution goes
// through the element now, and an element that is not there has no asset.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjDanglingTextureTest, "URLab.Import.ADanglingTextureReferenceResolvesToNothing",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjDanglingTextureTest::RunTest(const FString& Parameters)
{
	using namespace MjGeomMaterialTests;

	FScratchDoc Doc;
	if (!Parse(*this, Doc,
			TEXT("<mujoco><asset><texture name=\"present\" type=\"2d\" file=\"p.png\"/></asset>")
			TEXT("<worldbody><geom type=\"box\" size=\".1 .1 .1\"/></worldbody></mujoco>")))
	{
		return false;
	}
	UTexture2D* Present = PlaceTexture(*this, Doc, TEXT("present"));

	const FSpecRef Spec = FSpecRef::OverActor(*Doc.Actor);
	TestTrue(TEXT("the texture the spec does have resolves to its asset"),
		Present != nullptr && MjResolveTexture(Spec, TEXT("present")) == Present);
	TestNull(TEXT("one it does not have resolves to nothing"),
		MjResolveTexture(Spec, TEXT("absent")));
	return true;
}

// ============================================================================
// URLab.Compile.UnnamedFileAssetKeepsItsDerivedName
//
// A file-backed asset that authors no name is not anonymous to MuJoCo: it takes
// the file's basename, and that is the name every reference to it was written
// against. Scene assembly prefixes `file=` so two participants cannot collide in
// the VFS, which would move that derived name and dangle the references --
// "texture 'x' not found in material 0" out of a model that is perfectly valid.
//
// So the write states the name outright. The spec must come back unnamed:
// authoring identity nobody asked for would show up in the next diff of their
// MJCF.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjUnnamedAssetNameTest, "URLab.Compile.UnnamedFileAssetKeepsItsDerivedName",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjUnnamedAssetNameTest::RunTest(const FString& Parameters)
{
	MjGeomMaterialTests::FScratchDoc Doc;
	if (!MjGeomMaterialTests::Parse(*this, Doc,
			TEXT("<mujoco><asset>")
			TEXT("<texture type=\"2d\" file=\"2_of_clubs.png\"/>")
			TEXT("<material name=\"2_of_clubs\" texture=\"2_of_clubs\"/>")
			TEXT("</asset><worldbody>")
			TEXT("<geom type=\"box\" size=\".1 .1 .1\" material=\"2_of_clubs\"/>")
			TEXT("</worldbody></mujoco>")))
	{
		return false;
	}

	UMjTexture* Texture = Doc.Actor->FindComponentByClass<UMjTexture>();
	if (Texture == nullptr)
	{
		AddError(TEXT("the spec has no texture element"));
		return false;
	}
	TestFalse(TEXT("the authored texture has no name"), Texture->MjName.IsSet());

	// The compile can fail past this point -- there is no PNG on disk -- but the
	// text is written before anything reads the file, and the text is the claim.
	const FMjCompiled Compiled = MjCompileSpec(FSpecRef::OverActor(*Doc.Actor));
	TestTrue(TEXT("the emitted texture states its derived name"),
		Compiled.Xml.Contains(TEXT("name=\"2_of_clubs\"")));

	TestFalse(TEXT("and the spec is handed back unnamed"), Texture->MjName.IsSet());
	return true;
}

#endif  // URLAB_MJ_GEN && WITH_EDITOR
