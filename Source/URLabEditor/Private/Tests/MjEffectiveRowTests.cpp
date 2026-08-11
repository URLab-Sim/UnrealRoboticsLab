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
#include "Engine/World.h"
#include "Framework/Application/SlateApplication.h"
#include "GameFramework/Actor.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "Misc/Guid.h"
#include "Misc/ScopeExit.h"
#include "UObject/Package.h"

#include "MjEffectiveDetails.h"
#include "MuJoCo/Elements/MjGeom.h"
#include "MuJoCo/Gen/Elements/Geometry/MjGeom.gen.h"
#include "MuJoCo/Gen/Elements/MjModel.gen.h"
#include "MuJoCo/Gen/Elements/Joints/MjJoint.gen.h"
#include "MuJoCo/Gen/Elements/Sensors/MjTouch.gen.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "Tests/MjRowSupport.h"

namespace MjEffectiveRowTests
{

/**
 * A geom that authors nothing at all, and a sensor that must.
 *
 * `size` and `rgba` come from the geom's class, so those rows exercise the class
 * chain. `condim` and `margin` are mentioned by no `<default>` in the document,
 * so their rows can only be filled by the schema layer -- and `mass` is the one
 * MuJoCo computes rather than defaults, so its row is the "(no default)" case.
 * The `<touch>` sensor is here for the other half: `site` is REQUIRED, which is
 * a row with no layering to report and one an iterator over presence-wrapped
 * properties never saw.
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
      <site name="tip"/>
    </body>
  </worldbody>
  <sensor>
    <touch name="tap" site="tip"/>
  </sensor>
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

} // namespace MjEffectiveRowTests

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

	// `margin` is the row the owner went looking for. The schema states no `=`
	// default for it, so it used to be blank with a bare Set button; MuJoCo
	// initialises it all the same, and that is what the compiler will use.
	if (Rows.Has(TEXT("Margin")))
	{
		const FString Margin = TextOf(Rows.ValueOf(TEXT("Margin")));
		TestTrue(FString::Printf(TEXT("the margin row shows MuJoCo's own value, got '%s'"), *Margin),
			Margin.Contains(TEXT("(default)")));
		TestFalse(FString::Printf(TEXT("and never claims a class supplied it, got '%s'"), *Margin),
			Margin.Contains(TEXT("(from ")));
	}

	// And the honest one: `mass` is COMPUTED from density and volume, so no layer
	// has a value for it. The row says that in words rather than going blank,
	// which is what tells the user the panel answered rather than gave up.
	if (Rows.Has(TEXT("Mass")))
	{
		const FString Mass = TextOf(Rows.ValueOf(TEXT("Mass")));
		TestTrue(FString::Printf(TEXT("a computed attribute says it has no default, got '%s'"), *Mass),
			Mass.Contains(TEXT("(no default)")));
		TestFalse(FString::Printf(TEXT("and invents no value, got '%s'"), *Mass),
			Mass.Contains(TEXT("(default)")) && !Mass.Contains(TEXT("(no default)")));
	}

	return !HasAnyErrors();
}

// ============================================================================
// URLab.Editor.RequiredRowsSayTheyAreRequired
//   A `required` attribute has no unset state, so the inherited-value walk has
//   nothing to say about it -- and the row builder used to skip it entirely,
//   because it iterated presence-wrapped properties only. It keeps its editable
//   widget and gains the annotation, so every MuJoCo row says where it stands.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRequiredRowsSayTheyAreRequired, "URLab.Editor.RequiredRowsSayTheyAreRequired",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRequiredRowsSayTheyAreRequired::RunTest(const FString& Parameters)
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
	UMjTouch* const Touch = TemplateNamed<UMjTouch>(*Blueprint, TEXT("tap"));
	if (!TestNotNull(TEXT("the touch sensor template"), Touch))
	{
		return false;
	}
	TestEqual(TEXT("the sensor authors its required site"), Touch->Site, FString(TEXT("tip")));

	FRows Rows;
	if (!Rows.Build(*this, *Touch))
	{
		return false;
	}

	const FString Site = TextOf(Rows.ValueOf(TEXT("Site")));
	TestTrue(FString::Printf(TEXT("the required row is annotated, got '%s'"), *Site),
		Site.Contains(TEXT("(required)")));
	TestFalse(FString::Printf(TEXT("and claims neither a class nor a default, got '%s'"), *Site),
		Site.Contains(TEXT("(from ")) || Site.Contains(TEXT("(default)")));

	// The widget the user edits is still the engine's own: annotating a row must
	// not cost the ability to change it.
	TestTrue(TEXT("the required row keeps an editable widget beside the annotation"),
		Rows.ValueOf(TEXT("Site")).IsValid());

	// An engine property on the same component is none of this panel's business.
	// Widening the iterator to every FProperty is what makes that worth asserting.
	const FString Mobility = TextOf(Rows.ValueOf(TEXT("Mobility")));
	TestFalse(FString::Printf(TEXT("an engine property is left alone, got '%s'"), *Mobility),
		Mobility.Contains(TEXT("(required)")));

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

// ============================================================================
// URLab.Editor.EffectiveRowsWithoutADocument
//   The rows of an element that has no `<mujoco>` above it: a component dropped
//   onto an ordinary actor, which is how a user adds one by hand.
//
//   The layer walk went through `WithEffectiveDoc`, which answers nothing at all
//   when the spec's root cannot be reached -- so the list came back EMPTY, the
//   schema layer with it, and every attribute of such an element read "(no
//   default)". The class chain genuinely cannot be resolved without a document;
//   the schema layer never needed one, because MuJoCo's `condim` is 3 whatever
//   file it is read from. A hand-added geom therefore showed no effective values
//   at all while the same geom inside an imported robot showed all of them.
//
//   A geom and a joint together, because the two differ in exactly the way that
//   was suspected of causing it -- a geom is a hand subclass over its generated
//   base, a joint is the generated class itself -- and the answer has to be the
//   same for both.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjEffectiveRowsWithoutADocument, "URLab.Editor.EffectiveRowsWithoutADocument",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjEffectiveRowsWithoutADocument::RunTest(const FString& Parameters)
{
	using namespace MjEffectiveRowTests;
	using namespace MjRowSupport;

	if (!TestTrue(TEXT("Slate is up, so a row can be built at all"), FSlateApplication::IsInitialized()))
	{
		return false;
	}

	UWorld* const World = UWorld::CreateWorld(EWorldType::Editor, /*bInformEngineOfWorld=*/false,
		FName(*FString::Printf(TEXT("MjRowWorld_%s"), *FGuid::NewGuid().ToString(EGuidFormats::Digits))));
	if (!TestNotNull(TEXT("scratch world"), World))
	{
		return false;
	}
	ON_SCOPE_EXIT
	{
		World->DestroyWorld(false);
	};

	AActor* const Plain = World->SpawnActor<AActor>();
	if (!TestNotNull(TEXT("an actor that is not a model"), Plain))
	{
		return false;
	}

	// A root of the actor's own, so the two elements are siblings under something
	// that is not an element rather than parented to each other.
	USceneComponent* const Root = NewObject<USceneComponent>(Plain);
	Plain->SetRootComponent(Root);
	Root->RegisterComponent();

	UMjGeom* const Geom = NewObject<UMjGeom>(Plain);
	UMjJoint* const Joint = NewObject<UMjJoint>(Plain);
	if (!TestNotNull(TEXT("a hand-added geom"), Geom) || !TestNotNull(TEXT("a hand-added joint"), Joint))
	{
		return false;
	}
	Geom->AttachToComponent(Root, FAttachmentTransformRules::KeepRelativeTransform);
	Joint->AttachToComponent(Root, FAttachmentTransformRules::KeepRelativeTransform);
	Geom->RegisterComponent();
	Joint->RegisterComponent();

	// The premise: there really is no `<mujoco>` above these, so the class chain
	// really is unresolvable and what comes back can only be the schema's.
	TestNull(TEXT("the geom has no model root above it"),
		Cast<UMjModel>(FSpecRef::OverOwner(Geom).GetRoot()));
	TestFalse(TEXT("and it authors no condim"), Geom->Condim.IsSet());

	FRows GeomRows;
	if (!GeomRows.Build(*this, *Geom))
	{
		return false;
	}
	const FString Condim = TextOf(GeomRows.ValueOf(TEXT("Condim")));
	TestTrue(FString::Printf(TEXT("the geom's condim row shows MuJoCo's own value, got '%s'"), *Condim),
		Condim.Contains(TEXT("3")));
	TestTrue(FString::Printf(TEXT("marked a default rather than a class, got '%s'"), *Condim),
		Condim.Contains(TEXT("(default)")));
	TestFalse(FString::Printf(TEXT("and never claims a class supplied it, got '%s'"), *Condim),
		Condim.Contains(TEXT("(from ")));

	// The same question of the element family that has no hand subclass.
	FRows JointRows;
	if (JointRows.Build(*this, *Joint))
	{
		const FString Damping = TextOf(JointRows.ValueOf(TEXT("Damping")));
		TestTrue(FString::Printf(TEXT("a joint answers the same way, got '%s'"), *Damping),
			Damping.Contains(TEXT("(default)")));
	}

	// An attribute MuJoCo genuinely computes still says so, so the fix is the
	// schema layer arriving rather than every row being given a value.
	if (GeomRows.Has(TEXT("Mass")))
	{
		const FString Mass = TextOf(GeomRows.ValueOf(TEXT("Mass")));
		TestTrue(FString::Printf(TEXT("a computed attribute still has no default, got '%s'"), *Mass),
			Mass.Contains(TEXT("(no default)")));
	}

	// And Set still seeds what the row is showing rather than a zero.
	FRows Fresh;
	if (Fresh.Build(*this, *Geom))
	{
		const TSharedPtr<SButton> Set = ButtonIn(Fresh.ValueOf(TEXT("Condim")));
		if (TestTrue(TEXT("the condim row carries a button"), Set.IsValid()))
		{
			Set->SimulateClick();
			if (TestTrue(TEXT("pressing Set authors the attribute"), Geom->Condim.IsSet()))
			{
				TestEqual(TEXT("seeded with MuJoCo's own value"), Geom->Condim.GetValue(), 3);
			}
		}
	}

	return !HasAnyErrors();
}

#endif // URLAB_MJ_GEN && WITH_EDITOR
