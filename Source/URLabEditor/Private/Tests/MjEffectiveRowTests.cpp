// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// The row the user actually gets, built the way the details panel builds it.
//
// The resolver next door is checked as a function; this drives
// `FMjEffectiveDetails::CustomizeDetails` through `IPropertyRowGenerator` -- the
// same machinery a details panel runs -- over a document held as BLUEPRINT
// TEMPLATES, which is where a model is assembled and the branch nothing covered.
// What comes back is Slate, so the assertions read the widgets: the text beside
// the value, and the button, pressed.
//
// Two things are worth pinning here and nowhere else. That an attribute no
// `<default>` class mentions still shows MuJoCo's own value marked "(default)",
// because that is most rows of most elements and the panel used to show them
// blank. And that pressing Set leaves the ELEMENT holding the value the row was
// showing, rather than the zero a value-initialised optional would hold.

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

#include "MjEffectiveDetails.h"
#include "MuJoCo/Gen/Elements/Geometry/MjGeom.gen.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "Tests/MjRowSupport.h"

namespace MjEffectiveRowTests
{

/**
 * A geom that authors nothing at all.
 *
 * `size` and `rgba` come from its class, so those rows exercise the class chain.
 * `condim` is mentioned by no `<default>` in the document, so its row can only
 * be filled by the schema layer -- and `margin` is mentioned by no layer at all,
 * schema included, so its row must stay blank.
 */
const TCHAR* const Corpus = TEXT(R"(<mujoco model="rows">
  <default>
    <default class="visual">
      <geom type="sphere" size="0.3" rgba="1 0 0 1"/>
    </default>
  </default>
  <worldbody>
    <body name="base">
      <geom name="shell" class="visual"/>
    </body>
  </worldbody>
</mujoco>
)");

UBlueprint* ParseScratch(FAutomationTestBase& Test)
{
	const FString Name = FString::Printf(TEXT("MjRows_%s"), *FGuid::NewGuid().ToString(EGuidFormats::Digits));
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
		Test.AddError(TEXT("the row corpus did not parse into a Blueprint"));
		return nullptr;
	}
	return Blueprint;
}

/** The template of type `T` this document names `MjName`, or null. */
template <class T>
T* TemplateNamed(UBlueprint& Blueprint, const TCHAR* MjName)
{
	if (Blueprint.SimpleConstructionScript == nullptr)
	{
		return nullptr;
	}
	for (USCS_Node* Node : Blueprint.SimpleConstructionScript->GetAllNodes())
	{
		T* const Typed = Node != nullptr ? Cast<T>(Node->ComponentTemplate) : nullptr;
		if (Typed != nullptr && Typed->MjName.IsSet() && Typed->MjName.GetValue() == MjName)
		{
			return Typed;
		}
	}
	return nullptr;
}

}  // namespace MjEffectiveRowTests

// ============================================================================
// URLab.Editor.EffectiveRowsOverAnScsDocument
//   Selecting a geom in an imported robot's Blueprint: every unset row shows
//   what the compiler will use and says where it came from.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjEffectiveRowsOverAnScsDocument, "URLab.Editor.EffectiveRowsOverAnScsDocument",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjEffectiveRowsOverAnScsDocument::RunTest(const FString& Parameters)
{
	using namespace MjEffectiveRowTests;
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
	UMjGeomBase* const Geom = TemplateNamed<UMjGeomBase>(*Blueprint, TEXT("shell"));
	if (!TestNotNull(TEXT("the geom template"), Geom))
	{
		return false;
	}
	TestFalse(TEXT("the geom authors no size"), Geom->Size.IsSet());
	TestFalse(TEXT("and no condim"), Geom->Condim.IsSet());

	FRows Rows;
	if (!Rows.Build(*this, *Geom))
	{
		return false;
	}

	// The class chain, on a row a `<default>` does speak for.
	const FString Size = TextOf(Rows.ValueOf(TEXT("Size")));
	TestTrue(FString::Printf(TEXT("the size row shows the inherited value, got '%s'"), *Size),
		Size.Contains(TEXT("0.3")));
	TestTrue(FString::Printf(TEXT("and names the class it came from, got '%s'"), *Size),
		Size.Contains(TEXT("(from visual)")));

	// The layer this item is about. Nothing in the document mentions `condim`,
	// and before the schema became a layer this row was blank with a bare Set
	// button on it -- which is what most rows of most elements looked like.
	const FString Condim = TextOf(Rows.ValueOf(TEXT("Condim")));
	TestTrue(FString::Printf(TEXT("the condim row shows MuJoCo's own value, got '%s'"), *Condim),
		Condim.Contains(TEXT("3")));
	TestTrue(FString::Printf(TEXT("and marks it a default rather than naming a class, got '%s'"), *Condim),
		Condim.Contains(TEXT("(default)")));
	TestFalse(FString::Printf(TEXT("a schema value never claims to come from a class, got '%s'"), *Condim),
		Condim.Contains(TEXT("(from ")));
	TestTrue(FString::Printf(TEXT("and the row's own button is the one offered, got '%s'"), *Condim),
		Condim.Contains(TEXT("Set")));

	// And the honest blank: `margin` has no `=` default in the schema and no
	// class mentions it, so there is nothing to show and the row is left alone.
	if (Rows.Has(TEXT("Margin")))
	{
		const FString Margin = TextOf(Rows.ValueOf(TEXT("Margin")));
		TestFalse(FString::Printf(TEXT("an attribute no layer supplies claims no source, got '%s'"), *Margin),
			Margin.Contains(TEXT("(default)")) || Margin.Contains(TEXT("(from ")));
	}

	return !HasAnyErrors();
}

// ============================================================================
// URLab.Editor.SetSeedsTheResolvedValue
//   The button on the row. It used to be the engine's own, which
//   value-initialises: a geom inheriting 0.3 got a zero the moment the user
//   asked to author what they were looking at.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSetSeedsTheResolvedValue, "URLab.Editor.SetSeedsTheResolvedValue",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSetSeedsTheResolvedValue::RunTest(const FString& Parameters)
{
	using namespace MjEffectiveRowTests;
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
	UMjGeomBase* const Geom = TemplateNamed<UMjGeomBase>(*Blueprint, TEXT("shell"));
	if (!TestNotNull(TEXT("the geom template"), Geom))
	{
		return false;
	}

	FRows Rows;
	if (!Rows.Build(*this, *Geom))
	{
		return false;
	}

	// A schema-supplied row, because it is the one whose seed differs most
	// visibly from a value-initialised optional: 3 against 0.
	const TSharedPtr<SButton> Set = ButtonIn(Rows.ValueOf(TEXT("Condim")));
	if (!TestTrue(TEXT("the condim row carries a button"), Set.IsValid()))
	{
		return false;
	}
	TestFalse(TEXT("the element authors no condim before the press"), Geom->Condim.IsSet());
	Set->SimulateClick();

	if (!TestTrue(TEXT("pressing Set authors the attribute"), Geom->Condim.IsSet()))
	{
		return false;
	}
	TestEqual(TEXT("seeded with the value the row was showing, not with a zero"), Geom->Condim.GetValue(), 3);

	// The class-supplied row seeds from its class the same way, which is the
	// other half of "Set means author what I am looking at".
	FRows Fresh;
	if (Fresh.Build(*this, *Geom))
	{
		const TSharedPtr<SButton> SetRgba = ButtonIn(Fresh.ValueOf(TEXT("Rgba")));
		if (TestTrue(TEXT("the rgba row carries a button"), SetRgba.IsValid()))
		{
			SetRgba->SimulateClick();
			if (TestTrue(TEXT("pressing Set authors rgba"), Geom->Rgba.IsSet()))
			{
				const FLinearColor Authored = Geom->Rgba.GetValue();
				TestTrue(FString::Printf(TEXT("seeded with the class's red, got %s"), *Authored.ToString()),
					FMath::IsNearlyEqual(Authored.R, 1.0f) && FMath::IsNearlyEqual(Authored.G, 0.0f));
			}
		}
	}

	return !HasAnyErrors();
}

#endif  // URLAB_MJ_GEN && WITH_EDITOR
