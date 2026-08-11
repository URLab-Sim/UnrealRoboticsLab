// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// A document may spell any top-level section more than once, and may spell none
// of them; MuJoCo has exactly one of each either way. The importer has to
// produce one component per section -- holding the children and the authored
// attributes of every occurrence -- and to produce the untouched ones too.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Engine/Blueprint.h"
#include "Engine/SCS_Node.h"
#include "GameFramework/Actor.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "Misc/Guid.h"
#include "UObject/Package.h"

#include "MuJoCo/Gen/Elements/Assets/MjAsset.gen.h"
#include "MuJoCo/Gen/Elements/Bodies/MjBody.gen.h"
#include "MuJoCo/Gen/Elements/MjModel.gen.h"
#include "MuJoCo/Gen/Elements/Options/MjFlag.gen.h"
#include "MuJoCo/Gen/Elements/Options/MjOption.gen.h"
#include "MuJoCo/Gen/MjDispatch.gen.h"
#include "MuJoCo/Gen/MjEnums.gen.h"
#include "MuJoCo/Gen/MjStorage.gen.h"
#include "MuJoCo/Gen/MjVisit.gen.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Spec/MjTreeAdapters.h"

namespace MjSectionFoldTests
{

using urlab::spec::FMjOrderedChild;
namespace gen = urlab::spec::gen;

/**
 * Two `<worldbody>` sections, each carrying a different mix of children.
 *
 * The second holds a light, a geom and a body, so the fold is asserted over
 * three element types rather than only over bodies -- a section is whatever the
 * author put in it.
 */
const TCHAR* const TwoWorldbodies = TEXT(R"(<mujoco model="two_sections">
  <worldbody>
    <body name="robot">
      <geom name="robot_geom" type="sphere" size="0.1"/>
    </body>
  </worldbody>
  <worldbody>
    <light name="sun" pos="0 0 3"/>
    <geom name="floor" type="plane" size="5 5 0.1"/>
    <body name="prop" pos="1 0 0">
      <geom name="prop_geom" type="box" size="0.1 0.1 0.1"/>
    </body>
  </worldbody>
</mujoco>
)");

/** Two `<asset>` sections: a container section, whose whole content is children. */
const TCHAR* const TwoAssets = TEXT(R"(<mujoco model="two_assets">
  <asset>
    <material name="red" rgba="1 0 0 1"/>
  </asset>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1"/>
  </worldbody>
  <asset>
    <material name="green" rgba="0 1 0 1"/>
    <material name="blue" rgba="0 0 1 1"/>
  </asset>
</mujoco>
)");

/**
 * Two `<option>` sections: a settings section, whose whole content is attributes.
 *
 * `timestep` is authored by both, so it says which occurrence wins; `density`
 * only by the first and `integrator` only by the second, so between them they
 * say that the merge is per attribute rather than per section.
 */
const TCHAR* const TwoOptions = TEXT(R"(<mujoco model="two_options">
  <option timestep="0.001" density="1.2"/>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1"/>
  </worldbody>
  <option timestep="0.005" integrator="RK4"/>
</mujoco>
)");

/** Two `<option>` sections each carrying a `<flag>`: the merge leaves a repeat. */
const TCHAR* const TwoOptionsWithFlags = TEXT(R"(<mujoco model="two_flags">
  <option timestep="0.001">
    <flag gravity="disable"/>
  </option>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1"/>
  </worldbody>
  <option>
    <flag contact="disable"/>
  </option>
</mujoco>
)");

/** A document that spells three sections of the seventeen. */
const TCHAR* const NoOption = TEXT(R"(<mujoco model="no_option">
  <asset>
    <material name="red" rgba="1 0 0 1"/>
  </asset>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1"/>
  </worldbody>
</mujoco>
)");

UBlueprint* ParseScratch(FAutomationTestBase& Test, const FString& Xml)
{
	const FString Name = FString::Printf(TEXT("MjSectionFold_%s"), *FGuid::NewGuid().ToString(EGuidFormats::Digits));
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

/** The MJCF name of a child, or its component name when the element has none. */
FString LabelOf(const UMjNodeComponent& Node)
{
	return Node.MjName.IsSet() ? Node.MjName.GetValue() : Node.GetName();
}

/** `Parent`'s children of type `T`, in spec order. */
template <class T>
TArray<T*> ChildrenOfType(const FSpecRef& Spec, UMjNodeComponent& Parent)
{
	TArray<T*> Out;
	for (const FMjOrderedChild& Child : urlab::spec::MjOrderedChildrenOf(Spec, Parent))
	{
		if (T* Typed = Cast<T>(Child.Node))
		{
			Out.Add(Typed);
		}
	}
	return Out;
}

/** The MJCF names of `Parent`'s children, in spec order. */
TArray<FString> ChildLabels(const FSpecRef& Spec, UMjNodeComponent& Parent)
{
	TArray<FString> Out;
	for (const FMjOrderedChild& Child : urlab::spec::MjOrderedChildrenOf(Spec, Parent))
	{
		Out.Add(LabelOf(*Child.Node));
	}
	return Out;
}

/** How many times `Tag` opens an element in `Mjcf`. */
int32 CountTag(const FString& Mjcf, const TCHAR* Tag)
{
	const FString Open = FString(TEXT("<")) + Tag;
	int32 Count = 0;
	int32 At = Mjcf.Find(Open, ESearchCase::CaseSensitive, ESearchDir::FromStart, 0);
	while (At != INDEX_NONE)
	{
		// A prefix is not a tag: `<size` must not count a `<sizeof`, so the match
		// has to end where an element name ends.
		const int32 After = At + Open.Len();
		if (After < Mjcf.Len())
		{
			const TCHAR Next = Mjcf[After];
			if (Next == TEXT(' ') || Next == TEXT('>') || Next == TEXT('/') || Next == TEXT('\n'))
			{
				++Count;
			}
		}
		At = Mjcf.Find(Open, ESearchCase::CaseSensitive, ESearchDir::FromStart, At + 1);
	}
	return Count;
}

/** True once any attribute of the visited element is authored. */
struct FAnyAuthored
{
	bool bAny = false;

	template <class S>
	void field(int, const char*, S&& Slot)
	{
		bAny = bAny || gen::FMjShape::IsSet(Slot);
	}
};

/** Compare a spec-order label list against what the document declared. */
void TestLabels(FAutomationTestBase& Test, const TCHAR* What, const TArray<FString>& Actual,
	const TArray<FString>& Expected)
{
	if (!Test.TestEqual(What, Actual.Num(), Expected.Num()))
	{
		return;
	}
	for (int32 Index = 0; Index < Expected.Num(); ++Index)
	{
		Test.TestEqual(FString::Printf(TEXT("%s: entry %d"), What, Index), Actual[Index], Expected[Index]);
	}
}

} // namespace MjSectionFoldTests

// ============================================================================
// URLab.Spec.RepeatedWorldbodySectionsFold
//   Two `<worldbody>` sections reach the spec as ONE world-body component
//   holding the children of both, in the order the document declared them.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRepeatedWorldbodySectionsFoldTest, "URLab.Spec.RepeatedWorldbodySectionsFold",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRepeatedWorldbodySectionsFoldTest::RunTest(const FString& Parameters)
{
	using namespace MjSectionFoldTests;

	UBlueprint* Blueprint = ParseScratch(*this, TwoWorldbodies);
	if (Blueprint == nullptr)
	{
		return false;
	}

	const FSpecRef Spec = FSpecRef::OverBlueprint(*Blueprint);
	UMjNodeComponent* Root = Spec.GetRoot();
	if (!TestNotNull(TEXT("the parse produced a spec root"), Root))
	{
		return false;
	}

	const TArray<UMjBodyBase*> WorldBodies = ChildrenOfType<UMjBodyBase>(Spec, *Root);

	// The whole of the bug: two sections used to leave two siblings, and MuJoCo
	// has one world body.
	if (!TestEqual(TEXT("the model root holds exactly one world body"), WorldBodies.Num(), 1))
	{
		return false;
	}

	// And it is the first section's, so the panel shows it under the unsuffixed
	// tag rather than beside a `worldbody_1`.
	{
		urlab::spec::FMjScsScope Scope(*Blueprint);
		const USCS_Node* Node = Scope.FindNode(*WorldBodies[0]);
		if (TestNotNull(TEXT("the world body is in the construction script"), Node))
		{
			TestEqual(TEXT("the surviving world body is named for its tag"),
				Node->GetVariableName().ToString(), FString(TEXT("worldbody")));
		}
	}

	// Every child of both sections, in document order: a body slot holds each
	// admissible child type, so the second section's children follow the first's
	// rather than interleaving with them.
	TestLabels(*this, TEXT("the world body holds the children of both sections"),
		ChildLabels(Spec, *WorldBodies[0]), {TEXT("robot"), TEXT("sun"), TEXT("floor"), TEXT("prop")});

	// The moved subtrees came with their own children rather than being flattened
	// into the world body.
	UMjNodeComponent* Prop = nullptr;
	for (const FMjOrderedChild& Child : urlab::spec::MjOrderedChildrenOf(Spec, *WorldBodies[0]))
	{
		if (LabelOf(*Child.Node) == TEXT("prop"))
		{
			Prop = Child.Node;
		}
	}
	if (TestNotNull(TEXT("the second section's body survived the move"), Prop))
	{
		TestLabels(*this, TEXT("the moved body kept its one geom"), ChildLabels(Spec, *Prop), {TEXT("prop_geom")});
	}

	// And the document written back out spells one section, which is what makes
	// the fold a fixpoint rather than a one-way edit.
	TestEqual(TEXT("the writer emits one <worldbody>"), CountTag(Spec.WriteMjcf(), TEXT("worldbody")), 1);

	return true;
}

// ============================================================================
// URLab.Spec.RepeatedAssetSectionsFold
//   Two `<asset>` sections reach the spec as ONE asset component holding the
//   assets of both. A container section's whole content is its children.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRepeatedAssetSectionsFoldTest, "URLab.Spec.RepeatedAssetSectionsFold",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRepeatedAssetSectionsFoldTest::RunTest(const FString& Parameters)
{
	using namespace MjSectionFoldTests;

	UBlueprint* Blueprint = ParseScratch(*this, TwoAssets);
	if (Blueprint == nullptr)
	{
		return false;
	}

	const FSpecRef Spec = FSpecRef::OverBlueprint(*Blueprint);
	UMjNodeComponent* Root = Spec.GetRoot();
	if (!TestNotNull(TEXT("the parse produced a spec root"), Root))
	{
		return false;
	}

	const TArray<UMjAsset*> Assets = ChildrenOfType<UMjAsset>(Spec, *Root);
	if (!TestEqual(TEXT("the model root holds exactly one asset section"), Assets.Num(), 1))
	{
		return false;
	}

	// A section between the two -- the `<worldbody>` -- does not break the run:
	// the fold groups by type, not by adjacency.
	TestLabels(*this, TEXT("the asset section holds the assets of both"), ChildLabels(Spec, *Assets[0]),
		{TEXT("red"), TEXT("green"), TEXT("blue")});

	TestEqual(TEXT("the writer emits one <asset>"), CountTag(Spec.WriteMjcf(), TEXT("asset")), 1);

	return true;
}

// ============================================================================
// URLab.Spec.RepeatedOptionSectionsFold
//   Two `<option>` sections reach the spec as ONE option component whose
//   attributes are the merge of both, the later occurrence winning a conflict.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRepeatedOptionSectionsFoldTest, "URLab.Spec.RepeatedOptionSectionsFold",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRepeatedOptionSectionsFoldTest::RunTest(const FString& Parameters)
{
	using namespace MjSectionFoldTests;

	UBlueprint* Blueprint = ParseScratch(*this, TwoOptions);
	if (Blueprint == nullptr)
	{
		return false;
	}

	const FSpecRef Spec = FSpecRef::OverBlueprint(*Blueprint);
	UMjNodeComponent* Root = Spec.GetRoot();
	if (!TestNotNull(TEXT("the parse produced a spec root"), Root))
	{
		return false;
	}

	const TArray<UMjOption*> Options = ChildrenOfType<UMjOption>(Spec, *Root);
	if (!TestEqual(TEXT("the model root holds exactly one option section"), Options.Num(), 1))
	{
		return false;
	}
	const UMjOption& Option = *Options[0];

	// Later wins, which is what reading each occurrence into one struct in
	// document order amounts to.
	if (TestTrue(TEXT("timestep is authored"), Option.Timestep.IsSet()))
	{
		TestEqual(TEXT("timestep is the second section's"), Option.Timestep.GetValue(), 0.005);
	}

	// The first section's attribute survives an occurrence that is silent about
	// it: unauthored is not a value to copy.
	if (TestTrue(TEXT("density is authored"), Option.Density.IsSet()))
	{
		TestEqual(TEXT("density is the first section's"), Option.Density.GetValue(), 1.2);
	}

	if (TestTrue(TEXT("integrator is authored"), Option.Integrator.IsSet()))
	{
		TestTrue(TEXT("integrator is the second section's"), Option.Integrator.GetValue() == EMjIntegrator::RK4);
	}

	TestEqual(TEXT("the writer emits one <option>"), CountTag(Spec.WriteMjcf(), TEXT("option")), 1);

	return true;
}

// ============================================================================
// URLab.Spec.RepeatedOptionFlagsFold
//   Merging two `<option>` sections leaves the survivor holding two `<flag>`
//   children, and MuJoCo has one. The fold recurses into a settings section.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRepeatedOptionFlagsFoldTest, "URLab.Spec.RepeatedOptionFlagsFold",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRepeatedOptionFlagsFoldTest::RunTest(const FString& Parameters)
{
	using namespace MjSectionFoldTests;

	UBlueprint* Blueprint = ParseScratch(*this, TwoOptionsWithFlags);
	if (Blueprint == nullptr)
	{
		return false;
	}

	const FSpecRef Spec = FSpecRef::OverBlueprint(*Blueprint);
	UMjNodeComponent* Root = Spec.GetRoot();
	if (!TestNotNull(TEXT("the parse produced a spec root"), Root))
	{
		return false;
	}

	const TArray<UMjOption*> Options = ChildrenOfType<UMjOption>(Spec, *Root);
	if (!TestEqual(TEXT("the model root holds exactly one option section"), Options.Num(), 1))
	{
		return false;
	}

	const TArray<UMjFlag*> Flags = ChildrenOfType<UMjFlag>(Spec, *Options[0]);
	if (!TestEqual(TEXT("the option holds exactly one flag block"), Flags.Num(), 1))
	{
		return false;
	}
	const UMjFlag& Flag = *Flags[0];

	// Both flags' authored bits, on the one block: the recursion merges
	// attributes the same way the top level does.
	if (TestTrue(TEXT("gravity is authored"), Flag.Gravity.IsSet()))
	{
		TestTrue(TEXT("gravity is disabled"), Flag.Gravity.GetValue() == EMjEnable::disable);
	}
	if (TestTrue(TEXT("contact is authored"), Flag.Contact.IsSet()))
	{
		TestTrue(TEXT("contact is disabled"), Flag.Contact.GetValue() == EMjEnable::disable);
	}

	const FString Mjcf = Spec.WriteMjcf();
	TestEqual(TEXT("the writer emits one <option>"), CountTag(Mjcf, TEXT("option")), 1);
	TestEqual(TEXT("the writer emits one <flag>"), CountTag(Mjcf, TEXT("flag")), 1);

	return true;
}

// ============================================================================
// URLab.Spec.AbsentSectionsAreCreatedUnset
//   Every top-level section exists on the spec whether the document spelled it
//   or not, carrying nothing authored -- and an unauthored one is not written.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjAbsentSectionsAreCreatedUnsetTest, "URLab.Spec.AbsentSectionsAreCreatedUnset",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjAbsentSectionsAreCreatedUnsetTest::RunTest(const FString& Parameters)
{
	using namespace MjSectionFoldTests;

	UBlueprint* Blueprint = ParseScratch(*this, NoOption);
	if (Blueprint == nullptr)
	{
		return false;
	}

	const FSpecRef Spec = FSpecRef::OverBlueprint(*Blueprint);
	UMjNodeComponent* Root = Spec.GetRoot();
	if (!TestNotNull(TEXT("the parse produced a spec root"), Root))
	{
		return false;
	}

	// The document spells no `<option>`, and MuJoCo has an option struct: one
	// component, so there is something to edit rather than something to conjure.
	const TArray<UMjOption*> Options = ChildrenOfType<UMjOption>(Spec, *Root);
	if (!TestEqual(TEXT("a document with no <option> gets exactly one"), Options.Num(), 1))
	{
		return false;
	}

	// Unset is what makes the creation free of consequence: it is the storage's
	// own word for "the document did not author this".
	{
		FAnyAuthored Authored;
		gen::Visit(static_cast<const UMjOption&>(*Options[0]), Authored);
		TestFalse(TEXT("the created option authors nothing"), Authored.bAny);
	}

	// One of every section the schema admits under `<mujoco>`, no more.
	int32 Sections = 0;
	gen::ChildSlots(Cast<UMjModel>(Root), [&](int, auto Tag) {
		using T = typename decltype(Tag)::type;
		const int32 Count = ChildrenOfType<T>(Spec, *Root).Num();
		TestEqual(FString::Printf(TEXT("exactly one <%s>"), gen::TagForElement(gen::TMjElementType<T>::Value)),
			Count, 1);
		Sections += Count;
	});
	TestEqual(TEXT("the model root holds one child per schema slot"), Sections, 17);

	// The risk the creation carries: seventeen sections on every model would put
	// seventeen empty tags into every document we write, and a written document
	// has to say what the author said.
	const FString Mjcf = Spec.WriteMjcf();
	const TCHAR* const Unauthored[] = {TEXT("option"), TEXT("compiler"), TEXT("size"), TEXT("statistic"),
		TEXT("visual"), TEXT("default"), TEXT("extension"), TEXT("deformable"), TEXT("contact"), TEXT("tendon"),
		TEXT("equality"), TEXT("actuator"), TEXT("sensor"), TEXT("custom"), TEXT("keyframe")};
	for (const TCHAR* Tag : Unauthored)
	{
		TestEqual(FString::Printf(TEXT("the writer omits the unauthored <%s>"), Tag), CountTag(Mjcf, Tag), 0);
	}

	// And the sections the document DID spell are still there, so the omission
	// is of absences rather than of empty text.
	TestEqual(TEXT("the writer emits the authored <asset>"), CountTag(Mjcf, TEXT("asset")), 1);
	TestEqual(TEXT("the writer emits the authored <worldbody>"), CountTag(Mjcf, TEXT("worldbody")), 1);

	return true;
}

#endif // URLAB_MJ_GEN && WITH_EDITOR
