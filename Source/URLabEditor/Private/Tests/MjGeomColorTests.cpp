// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// The colour a geom is meant to be.
//
// Nothing resolved it. `<material>` was the one asset kind the import pass never
// visited, so a geom's `material` reference pointed at something the editor had
// never looked at, and the preview drew whatever the primitive shipped with. The
// resolution is two hops and both of them are defaultable: MuJoCo's humanoid
// puts `material="body"` on a default class, and the colour on the material that
// names.
//
// Which of the two wins is a renderer's decision and the two MuJoCo renderers
// do not agree. Classic (`engine_vis_visualize.c setMaterial`) starts from the
// material and lets a non-default geom `rgba` take it back; Filament
// (`model_renderables.cc GetDefaultMaterial`) starts from the geom and lets a
// non-default material `rgba` take it back. They differ only when both are
// explicitly non-default, and this plugin follows Filament: classic does not
// support PBR at all -- it ignores `mat_metallic` and `mat_roughness` -- so its
// rule belongs to a renderer Unreal is not.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Engine/Blueprint.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#include "GameFramework/Actor.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "Misc/FileHelper.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"
#include "UObject/Package.h"

#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Elements/MjGeom.h"

namespace MjGeomColorTests
{

UBlueprint* ParseScratch(FAutomationTestBase& Test, const FString& Xml)
{
	const FString Name = FString::Printf(TEXT("MjGeomColor_%s"), *FGuid::NewGuid().ToString(EGuidFormats::Digits));
	UPackage* Package = CreatePackage(*(TEXT("/Temp/") + Name));
	UBlueprint* Blueprint = FKismetEditorUtilities::CreateBlueprint(AActor::StaticClass(), Package, FName(*Name),
		BPTYPE_Normal, UBlueprint::StaticClass(), UBlueprintGeneratedClass::StaticClass());
	if (Blueprint == nullptr)
	{
		Test.AddError(TEXT("could not create a scratch Blueprint"));
		return nullptr;
	}
	const FMjSpecParseResult Parsed = MjParseIntoBlueprint(*Blueprint, Xml, TEXT("<inline>"));
	if (!Parsed.IsOk())
	{
		TArray<FString> Lines;
		for (const FMjSpecDiagnostic& Diagnostic : Parsed.Errors)
		{
			Lines.Add(Diagnostic.ToString());
		}
		Test.AddError(FString::Printf(TEXT("parse failed: %s"), *FString::Join(Lines, TEXT("; "))));
		return nullptr;
	}
	return Blueprint;
}

UMjGeom* FindGeom(UBlueprint& Blueprint, const TCHAR* MjName)
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

void ColorIs(FAutomationTestBase& Test, const TCHAR* Label, const FLinearColor& Actual, const FLinearColor& Expected)
{
	Test.TestTrue(*FString::Printf(TEXT("%s is %s, got %s"), Label, *Expected.ToString(), *Actual.ToString()),
		Actual.Equals(Expected, 1e-5f));
}

} // namespace MjGeomColorTests

// ============================================================================
// URLab.Import.GeomColorFollowsMaterialThroughItsClass
//   The humanoid's own arrangement, plus every branch of Filament's precedence
//   rule: the geom's rgba, taken back by the material's whenever the material's
//   is anything but (0.5, 0.5, 0.5, 1) -- including the branch where that rule
//   and the classic renderer's disagree.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjGeomColorTest, "URLab.Import.GeomColorFollowsMaterialThroughItsClass",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjGeomColorTest::RunTest(const FString& Parameters)
{
	using namespace MjGeomColorTests;

	const FString Xml = TEXT(R"(<mujoco model="colour">
  <compiler angle="radian"/>
  <asset>
    <material name="body" rgba="0.8 0.6 0.4 1"/>
    <material name="plain"/>
    <material name="magic" rgba="0.5 0.5 0.5 1"/>
  </asset>
  <default>
    <default class="body">
      <geom type="capsule" size="0.05" material="body"/>
      <default class="loud">
        <geom rgba="0.1 0.2 0.9 1"/>
      </default>
    </default>
  </default>
  <worldbody>
    <body name="b" childclass="body">
      <geom name="inherited"/>
      <geom name="own_rgba" rgba="0 1 0 1"/>
      <geom name="rgba_from_class" class="loud"/>
      <geom name="default_rgba" material="plain"/>
      <geom name="magic_material" material="magic" rgba="0 1 0 1"/>
    </body>
    <body name="outside">
      <geom name="bare" type="sphere" size="0.05"/>
    </body>
  </worldbody>
</mujoco>
)");

	UBlueprint* Blueprint = ParseScratch(*this, Xml);
	if (Blueprint == nullptr)
	{
		return false;
	}

	UMjGeom* Inherited = FindGeom(*Blueprint, TEXT("inherited"));
	UMjGeom* OwnRgba = FindGeom(*Blueprint, TEXT("own_rgba"));
	UMjGeom* FromClass = FindGeom(*Blueprint, TEXT("rgba_from_class"));
	UMjGeom* DefaultRgba = FindGeom(*Blueprint, TEXT("default_rgba"));
	UMjGeom* MagicMaterial = FindGeom(*Blueprint, TEXT("magic_material"));
	UMjGeom* Bare = FindGeom(*Blueprint, TEXT("bare"));
	if (!TestNotNull(TEXT("geom inherited"), Inherited) || !TestNotNull(TEXT("geom own_rgba"), OwnRgba) || !TestNotNull(TEXT("geom rgba_from_class"), FromClass) || !TestNotNull(TEXT("geom default_rgba"), DefaultRgba) || !TestNotNull(TEXT("geom magic_material"), MagicMaterial) || !TestNotNull(TEXT("geom bare"), Bare))
	{
		return false;
	}

	// The humanoid's case exactly: the geom authors nothing, the class names a
	// material, and the material carries the colour. Two hops, neither on the
	// geom, which is why reading the geom's own storage drew the wrong thing.
	ColorIs(*this, TEXT("a colour reached through the class's material"), Inherited->GetEffectiveColor(),
		FLinearColor(0.8f, 0.6f, 0.4f, 1.0f));

	// The branch the two renderers disagree on: the geom authors a colour AND
	// the material authors one. Classic hands it to the geom (green); Filament
	// hands it to the material, and so does this.
	ColorIs(*this, TEXT("a material's rgba beats the geom's own"), OwnRgba->GetEffectiveColor(),
		FLinearColor(0.8f, 0.6f, 0.4f, 1.0f));

	// Including when the geom's came from a nearer class than the material did:
	// where in the chain a value was found does not change who wins.
	ColorIs(*this, TEXT("an rgba inherited from a class loses the same way"), FromClass->GetEffectiveColor(),
		FLinearColor(0.8f, 0.6f, 0.4f, 1.0f));

	// A material that authors no rgba is white, because `<material rgba>`
	// defaults to (1 1 1 1) and that is not the magic value the rule tests for.
	ColorIs(*this, TEXT("a material with no rgba of its own is white"), DefaultRgba->GetEffectiveColor(),
		FLinearColor(1.0f, 1.0f, 1.0f, 1.0f));

	// The one way a material declines: by authoring the magic default itself.
	// Then the geom's rgba stands, green rather than grey.
	ColorIs(*this, TEXT("a material authoring the magic default defers to the geom"),
		MagicMaterial->GetEffectiveColor(), FLinearColor(0.0f, 1.0f, 0.0f, 1.0f));

	// And with no material at all -- which means outside the childclass that
	// supplies one, since MJCF has no way to say "no material" on the geom --
	// the geom's rgba stands even when it is the default. That is the branch
	// that decides whether an unstyled model draws grey or white.
	ColorIs(*this, TEXT("with no material the geom's own rgba stands"), Bare->GetEffectiveColor(),
		FLinearColor(0.5f, 0.5f, 0.5f, 1.0f));

	return true;
}

// ============================================================================
// URLab.Import.HumanoidGeomsAreNotDefaultGrey
//   Over the real model: every geom in MuJoCo's humanoid wears `material="body"`
//   through its childclass, so not one of them should resolve to the schema
//   grey. Grey everywhere is exactly what the user saw.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjHumanoidColorTest, "URLab.Import.HumanoidGeomsAreNotDefaultGrey",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjHumanoidColorTest::RunTest(const FString& Parameters)
{
	using namespace MjGeomColorTests;

	const FString File = FPaths::Combine(FPaths::ProjectPluginsDir(), TEXT("UnrealRoboticsLab"), TEXT("Content"),
		TEXT("TestData"), TEXT("humanoid.xml"));
	FString Xml;
	if (!TestTrue(TEXT("the humanoid fixture is on disk"), FFileHelper::LoadFileToString(Xml, *File)))
	{
		return false;
	}

	UBlueprint* Blueprint = ParseScratch(*this, Xml);
	if (Blueprint == nullptr)
	{
		return false;
	}

	// The torso is a body geom: `material="body"` off the childclass, and the
	// material's rgba is the tan the model is drawn in.
	UMjGeom* Torso = FindGeom(*Blueprint, TEXT("torso"));
	if (TestNotNull(TEXT("geom torso"), Torso))
	{
		ColorIs(*this, TEXT("the torso"), Torso->GetEffectiveColor(), FLinearColor(0.8f, 0.6f, 0.4f, 1.0f));
	}

	// A geom whose fromto and class both come from the chain resolves the same
	// way -- colour and geometry are independent hops through it.
	UMjGeom* ShinRight = FindGeom(*Blueprint, TEXT("shin_right"));
	if (TestNotNull(TEXT("geom shin_right"), ShinRight))
	{
		ColorIs(*this, TEXT("a class-only geom"), ShinRight->GetEffectiveColor(),
			FLinearColor(0.8f, 0.6f, 0.4f, 1.0f));
	}

	// The floor names the `grid` material, which authors no rgba: white, and in
	// particular NOT the body tan, so the lookup is by name and not by luck.
	UMjGeom* Floor = FindGeom(*Blueprint, TEXT("floor"));
	if (TestNotNull(TEXT("geom floor"), Floor))
	{
		ColorIs(*this, TEXT("the floor"), Floor->GetEffectiveColor(), FLinearColor(1.0f, 1.0f, 1.0f, 1.0f));
	}

	return true;
}

#endif // URLAB_MJ_GEN && WITH_EDITOR
