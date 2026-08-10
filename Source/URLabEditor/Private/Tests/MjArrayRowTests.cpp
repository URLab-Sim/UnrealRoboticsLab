// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// What the numbers in an array row are called.
//
// An MJCF array is a row of unnamed doubles, and the panel names the slots so a
// user can tell a radius from a half-length. Naming them WRONG is worse than
// leaving them bare: `friction` is three coefficients on a `<geom>` and five on
// a `<pair>`, and the three-coefficient names were being put on both -- so on a
// contact pair the box labelled "torsional" was the second SLIDING coefficient,
// and a user correcting spin friction was editing tangential friction instead.
//
// The euler row is here for the same reason: it is a second way to type an
// attribute, and what makes it safe is that it appears only on elements that
// have the attribute it writes. That one is read off the built panel, because
// which elements carry the row is a fact about the panel.
//
// The labels are read off `SlotLabelsFor`, the labeller the panel calls, rather
// than off the widgets: the labels sit inside a numeric entry box on a row built
// from a real element, and the question worth asserting -- these five names, in
// this order, for a pair -- is the labeller's answer, in a form a failure can
// print.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Engine/Blueprint.h"
#include "Engine/BlueprintGeneratedClass.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#include "Framework/Application/SlateApplication.h"
#include "GameFramework/Actor.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "Misc/Guid.h"
#include "UObject/Package.h"

#include "MjArrayCustomizations.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "Tests/MjRowSupport.h"

namespace MjArrayRowTests
{

/**
 * One document carrying every array shape the labels are about.
 *
 * The two geoms differ only in their `type`, which is what decides what their
 * `size` slots mean; the pair is the element whose `friction` is five long. All
 * three author the attribute, because an unset row shows its inherited value
 * rather than its slots.
 */
const TCHAR* const Corpus = TEXT(R"(<mujoco model="arrays">
  <worldbody>
    <body name="link">
      <geom name="ball" type="sphere" size="0.1" friction="1 0.005 0.0001" quat="0.7071068 0.7071068 0 0"/>
      <geom name="brick" type="box" size="0.1 0.2 0.3"/>
    </body>
  </worldbody>
  <contact>
    <pair name="touch" geom1="ball" geom2="brick" friction="1 1 0.005 0.0001 0.0001"/>
  </contact>
</mujoco>
)");

UBlueprint* ParseScratch(FAutomationTestBase& Test)
{
	const FString Name = FString::Printf(TEXT("MjArrayRows_%s"), *FGuid::NewGuid().ToString(EGuidFormats::Digits));
	UPackage* const Package = CreatePackage(*(TEXT("/Temp/") + Name));
	UBlueprint* const Blueprint = FKismetEditorUtilities::CreateBlueprint(AActor::StaticClass(), Package, FName(*Name),
		BPTYPE_Normal, UBlueprint::StaticClass(), UBlueprintGeneratedClass::StaticClass());
	if (Blueprint == nullptr)
	{
		Test.AddError(TEXT("could not create a scratch Blueprint"));
		return nullptr;
	}
	const FMjSpecParseResult Parsed = MjParseIntoBlueprint(*Blueprint, Corpus, TEXT("<inline>"));
	if (!Parsed.IsOk())
	{
		Test.AddError(TEXT("the array corpus did not parse into a Blueprint"));
		return nullptr;
	}
	return Blueprint;
}

/** The element template this document names `MjName`, or null. */
UMjNodeComponent* TemplateNamed(UBlueprint& Blueprint, const TCHAR* MjName)
{
	if (Blueprint.SimpleConstructionScript == nullptr)
	{
		return nullptr;
	}
	for (USCS_Node* Node : Blueprint.SimpleConstructionScript->GetAllNodes())
	{
		UMjNodeComponent* const Element = Node != nullptr ? Cast<UMjNodeComponent>(Node->ComponentTemplate) : nullptr;
		if (Element != nullptr && Element->MjName.IsSet() && Element->MjName.GetValue() == MjName)
		{
			return Element;
		}
	}
	return nullptr;
}

/** The slot names as one string, so a wrong order is a readable failure. */
FString Joined(const TArray<FText>& Labels)
{
	TArray<FString> Parts;
	for (const FText& Label : Labels)
	{
		Parts.Add(Label.ToString());
	}
	return FString::Join(Parts, TEXT("|"));
}

}  // namespace MjArrayRowTests

// ============================================================================
// URLab.Editor.FrictionSlotsAreNamedForTheElementTheyAreOn
//   The defect: `<pair friction>` is five coefficients and was wearing the
//   three-coefficient geom names.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjFrictionSlotsNamedForTheirElement,
	"URLab.Editor.FrictionSlotsAreNamedForTheElementTheyAreOn",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjFrictionSlotsNamedForTheirElement::RunTest(const FString& Parameters)
{
	using namespace MjArrayRowTests;

	UBlueprint* const Blueprint = ParseScratch(*this);
	if (Blueprint == nullptr)
	{
		return false;
	}

	UMjNodeComponent* const Geom = TemplateNamed(*Blueprint, TEXT("ball"));
	UMjNodeComponent* const Pair = TemplateNamed(*Blueprint, TEXT("touch"));
	if (!TestNotNull(TEXT("the geom"), Geom) || !TestNotNull(TEXT("the contact pair"), Pair))
	{
		return false;
	}

	// A geom's three: one tangential, one about the contact normal, one rolling.
	const TArray<FText> GeomSlots = FMjArrayCustomizations::SlotLabelsFor(*Geom, FName(TEXT("Friction")));
	TestEqual(TEXT("a geom's friction is three coefficients"), GeomSlots.Num(), 3);
	TestEqual(TEXT("the geom's three, in MuJoCo's order"), Joined(GeomSlots),
		FString(TEXT("sliding|torsional|rolling")));

	// A pair's five. This is the row that was wrong: slot 1 read "torsional"
	// where MuJoCo reads the second sliding coefficient.
	const TArray<FText> PairSlots = FMjArrayCustomizations::SlotLabelsFor(*Pair, FName(TEXT("Friction")));
	TestEqual(TEXT("a contact pair's friction is five coefficients"), PairSlots.Num(), 5);
	TestEqual(TEXT("the pair's five, in MuJoCo's order"), Joined(PairSlots),
		FString(TEXT("sliding 1|sliding 2|torsional|rolling 1|rolling 2")));

	// The defect stated as the user met it: the box a pair labels "torsional"
	// was the second sliding coefficient.
	if (PairSlots.Num() == 5)
	{
		TestNotEqual(TEXT("slot 1 of a pair is not the geom's torsional coefficient"),
			PairSlots[1].ToString(), GeomSlots.IsValidIndex(1) ? GeomSlots[1].ToString() : FString());
	}

	return !HasAnyErrors();
}

// ============================================================================
// URLab.Editor.SizeSlotsAreNamedForTheShape
//   The same rule on the attribute it was already right about, so the labeller
//   taking the element into account has not cost the case that worked.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSizeSlotsNamedForTheShape, "URLab.Editor.SizeSlotsAreNamedForTheShape",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSizeSlotsNamedForTheShape::RunTest(const FString& Parameters)
{
	using namespace MjArrayRowTests;

	UBlueprint* const Blueprint = ParseScratch(*this);
	if (Blueprint == nullptr)
	{
		return false;
	}

	UMjNodeComponent* const Sphere = TemplateNamed(*Blueprint, TEXT("ball"));
	UMjNodeComponent* const Box = TemplateNamed(*Blueprint, TEXT("brick"));
	if (!TestNotNull(TEXT("the sphere"), Sphere) || !TestNotNull(TEXT("the box"), Box))
	{
		return false;
	}

	TestEqual(TEXT("a sphere's one size value is its radius"),
		Joined(FMjArrayCustomizations::SlotLabelsFor(*Sphere, FName(TEXT("Size")))), FString(TEXT("radius")));
	TestEqual(TEXT("a box's three are its half-extents"),
		Joined(FMjArrayCustomizations::SlotLabelsFor(*Box, FName(TEXT("Size")))),
		FString(TEXT("half-x|half-y|half-z")));

	return !HasAnyErrors();
}

// ============================================================================
// URLab.Editor.TheEulerRowAppearsWhereQuatDoes
//   The euler row authors `quat` and nothing else, so it belongs on the
//   elements that have one and on no others. A row offering to rotate an
//   element that cannot be rotated would write an attribute the writer never
//   emits.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjEulerRowAppearsWhereQuatDoes, "URLab.Editor.TheEulerRowAppearsWhereQuatDoes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjEulerRowAppearsWhereQuatDoes::RunTest(const FString& Parameters)
{
	using namespace MjArrayRowTests;
	using namespace MjRowSupport;

	if (!TestTrue(TEXT("Slate is up, so a row can be built at all"), FSlateApplication::IsInitialized()))
	{
		return false;
	}

	UBlueprint* const Blueprint = ParseScratch(*this);
	if (Blueprint == nullptr)
	{
		return false;
	}

	UMjNodeComponent* const Geom = TemplateNamed(*Blueprint, TEXT("ball"));
	UMjNodeComponent* const Pair = TemplateNamed(*Blueprint, TEXT("touch"));
	if (!TestNotNull(TEXT("the geom"), Geom) || !TestNotNull(TEXT("the contact pair"), Pair))
	{
		return false;
	}

	FRows GeomRows;
	if (!GeomRows.Build(*this, *Geom))
	{
		return false;
	}
	const FString GeomPanel = GeomRows.AllText();
	TestTrue(FString::Printf(TEXT("an element with a quat gets a Euler row, got '%s'"), *GeomPanel),
		GeomPanel.Contains(TEXT("Euler")));

	// The unit and the convention, spelled on the row. An euler triple means
	// nothing without them, and MuJoCo's frame is not Unreal's.
	TestTrue(TEXT("the row says which units and which frame"),
		GeomPanel.Contains(TEXT("deg, xyz (MuJoCo frame)")));

	FRows PairRows;
	if (!PairRows.Build(*this, *Pair))
	{
		return false;
	}
	TestFalse(TEXT("an element with no quat gets no Euler row"), PairRows.AllText().Contains(TEXT("Euler")));

	return !HasAnyErrors();
}

#endif  // URLAB_MJ_GEN && WITH_EDITOR
