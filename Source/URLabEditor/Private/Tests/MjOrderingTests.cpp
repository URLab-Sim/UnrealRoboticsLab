// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// Where a hand-added element lands among its siblings.
//
// Declaration order is semantic in MJCF: the joints under a body, in the order
// they are declared, ARE the body's qpos layout. Adding a joint in the
// components panel used to put it at the front of that list, because the panel
// creates a component with class defaults and spec order defaulted to zero --
// the first position. Every joint after it shifted by one, and a controller
// written against the imported model started driving the wrong degree of
// freedom, silently.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Engine/Blueprint.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#include "GameFramework/Actor.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "Misc/Guid.h"
#include "UObject/Package.h"

#include "MuJoCo/Gen/Elements/Joints/MjJoint.gen.h"
#include "MuJoCo/Gen/MjDispatch.gen.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Spec/MjTreeAdapters.h"

namespace MjOrderingTests
{

UBlueprint* MakeScratchBlueprint()
{
	const FString Name = FString::Printf(TEXT("MjOrdering_%s"), *FGuid::NewGuid().ToString(EGuidFormats::Digits));
	UPackage* Package = CreatePackage(*(TEXT("/Temp/") + Name));
	return FKismetEditorUtilities::CreateBlueprint(AActor::StaticClass(), Package, FName(*Name), BPTYPE_Normal,
		UBlueprint::StaticClass(), UBlueprintGeneratedClass::StaticClass());
}

UBlueprint* ParseScratch(FAutomationTestBase& Test, const FString& Xml)
{
	UBlueprint* Blueprint = MakeScratchBlueprint();
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

/** The SCS node holding the element whose MJCF name is `MjName`. */
USCS_Node* NodeNamed(UBlueprint& Blueprint, const TCHAR* MjName)
{
	if (Blueprint.SimpleConstructionScript == nullptr)
	{
		return nullptr;
	}
	for (USCS_Node* Node : Blueprint.SimpleConstructionScript->GetAllNodes())
	{
		const UMjNodeComponent* Element = Node != nullptr ? Cast<UMjNodeComponent>(Node->ComponentTemplate) : nullptr;
		if (Element != nullptr && Element->MjName.IsSet() && Element->MjName.GetValue() == MjName)
		{
			return Node;
		}
	}
	return nullptr;
}

/**
 * Add a component under `Parent` the way the components panel does: a node made
 * from the class, linked to its parent, with a template holding class defaults
 * and nothing else.
 */
UMjJoint* AddJointByHand(UBlueprint& Blueprint, USCS_Node& Parent, const TCHAR* MjName)
{
	USCS_Node* Node = Blueprint.SimpleConstructionScript->CreateNode(UMjJoint::StaticClass(), FName(MjName));
	if (Node == nullptr)
	{
		return nullptr;
	}
	Parent.AddChildNode(Node);
	UMjJoint* Joint = Cast<UMjJoint>(Node->ComponentTemplate);
	if (Joint != nullptr)
	{
		Joint->MjName = FString(MjName);
	}
	return Joint;
}

/** The MJCF names of `Parent`'s children of type `T`, in spec order. */
template <class T>
TArray<FString> ChildNamesOfType(const FSpecRef& Spec, UMjNodeComponent& Parent)
{
	TArray<FString> Out;
	for (const urlab::spec::FMjOrderedChild& Child : urlab::spec::MjOrderedChildrenOf(Spec, Parent))
	{
		const T* Typed = Cast<T>(Child.Node);
		if (Typed != nullptr && Typed->MjName.IsSet())
		{
			Out.Add(Typed->MjName.GetValue());
		}
	}
	return Out;
}

const TCHAR* const TwoJointBody = TEXT(R"(<mujoco model="ordering">
  <worldbody>
    <body name="link" pos="0 0 1">
      <joint name="first" type="hinge" axis="0 0 1"/>
      <joint name="second" type="hinge" axis="0 1 0"/>
      <geom name="shape" type="box" size="0.1 0.1 0.1"/>
    </body>
  </worldbody>
</mujoco>
)");

}  // namespace MjOrderingTests

// ============================================================================
// URLab.Spec.AHandAddedElementAppends
//   The panel hands over a component with nothing authored on it, spec order
//   included. That element joins the END of its group, and the walk that reads
//   spec order writes the position down, so the append survives a save.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjHandAddedElementAppendsTest, "URLab.Spec.AHandAddedElementAppends",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjHandAddedElementAppendsTest::RunTest(const FString& Parameters)
{
	using namespace MjOrderingTests;

	UBlueprint* Blueprint = ParseScratch(*this, TwoJointBody);
	if (Blueprint == nullptr)
	{
		return false;
	}

	USCS_Node* BodyNode = NodeNamed(*Blueprint, TEXT("link"));
	if (!TestNotNull(TEXT("the imported body is in the construction script"), BodyNode))
	{
		return false;
	}
	UMjNodeComponent* Body = Cast<UMjNodeComponent>(BodyNode->ComponentTemplate);
	if (!TestNotNull(TEXT("the body node holds an element template"), Body))
	{
		return false;
	}

	UMjJoint* Added = AddJointByHand(*Blueprint, *BodyNode, TEXT("added"));
	if (!TestNotNull(TEXT("a joint can be added the way the panel adds one"), Added))
	{
		return false;
	}

	// The whole bug in one assertion: the panel's component expresses no
	// position, and a position of zero is not the same statement.
	TestEqual(TEXT("a hand-added element arrives with no spec order of its own"), Added->SiblingIndex,
		static_cast<int32>(INDEX_NONE));

	const FSpecRef Spec = FSpecRef::OverBlueprint(*Blueprint);
	const TArray<FString> Joints = ChildNamesOfType<UMjJoint>(Spec, *Body);

	if (TestEqual(TEXT("the body has the two imported joints and the added one"), Joints.Num(), 3))
	{
		TestEqual(TEXT("the first imported joint keeps the first position"), Joints[0], FString(TEXT("first")));
		TestEqual(TEXT("the second imported joint keeps the second position"), Joints[1], FString(TEXT("second")));
		TestEqual(TEXT("the added joint lands last"), Joints[2], FString(TEXT("added")));
	}

	// The order the compile reads is also where it is written down: an element
	// left unstamped would be re-decided by creation order on every load, and a
	// duplicated component mints a fresh serial.
	//
	// The stamp is a position within the STORAGE SLOT, and a body holds every
	// child type it admits in one slot (`ChildSlots(const UMjBodyBase*)`, all of
	// them slot 0) because MJCF body children are one interleaved declaration
	// sequence. So the number counts the geom too, and the assertion is that the
	// element was stamped after every sibling rather than any literal index --
	// which is the property, and does not go stale when a fixture gains an
	// element.
	TestNotEqual(TEXT("the walk stamped the added joint"), Added->SiblingIndex, static_cast<int32>(INDEX_NONE));
	for (const urlab::spec::FMjOrderedChild& Child : urlab::spec::MjOrderedChildrenOf(Spec, *Body))
	{
		if (Child.Node != nullptr && Child.Node != Added)
		{
			TestTrue(FString::Printf(TEXT("the added joint sorts after '%s'"), *Child.Node->GetName()),
				Added->SiblingIndex > Child.Node->SiblingIndex);
		}
	}

	// And the same order comes back out of the writer, which is the qpos layout
	// as MuJoCo will read it.
	const FString Mjcf = Spec.WriteMjcf();
	const int32 First = Mjcf.Find(TEXT("\"first\""));
	const int32 Second = Mjcf.Find(TEXT("\"second\""));
	const int32 AddedAt = Mjcf.Find(TEXT("\"added\""));
	if (TestTrue(TEXT("all three joints are written"), First >= 0 && Second >= 0 && AddedAt >= 0))
	{
		TestTrue(TEXT("the written joint order is first, second, added"), First < Second && Second < AddedAt);
	}

	return true;
}

// ============================================================================
// URLab.Spec.ImportedOrderIsUntouched
//   The guard on the change: an element the reader stamped keeps the position
//   it was given, and nothing about the new sentinel reorders a model that has
//   no hand-added elements in it.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjImportedOrderUntouchedTest, "URLab.Spec.ImportedOrderIsUntouched",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjImportedOrderUntouchedTest::RunTest(const FString& Parameters)
{
	using namespace MjOrderingTests;

	UBlueprint* Blueprint = ParseScratch(*this, TwoJointBody);
	if (Blueprint == nullptr)
	{
		return false;
	}

	USCS_Node* BodyNode = NodeNamed(*Blueprint, TEXT("link"));
	UMjNodeComponent* Body = BodyNode != nullptr ? Cast<UMjNodeComponent>(BodyNode->ComponentTemplate) : nullptr;
	if (!TestNotNull(TEXT("the imported body is in the construction script"), Body))
	{
		return false;
	}

	const FSpecRef Spec = FSpecRef::OverBlueprint(*Blueprint);
	const TArray<FString> Joints = ChildNamesOfType<UMjJoint>(Spec, *Body);
	if (TestEqual(TEXT("the body has its two imported joints"), Joints.Num(), 2))
	{
		TestEqual(TEXT("declaration order is the imported order"), Joints[0], FString(TEXT("first")));
		TestEqual(TEXT("declaration order is the imported order"), Joints[1], FString(TEXT("second")));
	}

	// The reader stamps as it builds, so an import carries no unstamped element
	// and the append path is never reached for one. The spec root is exempt: it
	// has no siblings to be ordered among, and nothing adopts it.
	USimpleConstructionScript* Scs = Blueprint->SimpleConstructionScript;
	const TArray<USCS_Node*> Roots = Scs->GetRootNodes();
	int32 Unstamped = 0;
	for (USCS_Node* Node : Scs->GetAllNodes())
	{
		const UMjNodeComponent* Element = Node != nullptr ? Cast<UMjNodeComponent>(Node->ComponentTemplate) : nullptr;
		if (Element != nullptr && !Roots.Contains(Node) && Element->SiblingIndex == INDEX_NONE)
		{
			++Unstamped;
		}
	}
	TestEqual(TEXT("an import leaves no element without a spec order"), Unstamped, 0);

	return true;
}

// ============================================================================
// URLab.Perf.ImportChildQueries
//   The reader adopts once per element, and adopting used to ask the parent for
//   its children -- resolving every one of their slots through the schema
//   tables again. A parent of k children paid that k times, so the table work
//   grew as the square of the tree's width: quadrupling the model multiplied it
//   by 5.6 rather than by 4. The slots are now resolved once per child and kept
//   on the parent, checked against the live child list before use.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjImportChildQueriesTest, "URLab.Perf.ImportChildQueries",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

namespace MjOrderingTests
{
/** The two model sizes, a factor of four apart. */
constexpr int32 SmallBodies = 25;
constexpr int32 LargeBodies = 100;

/**
 * How far past the model's own growth the table work may go.
 *
 * Linear is 4.0 for a fourfold model and is not achievable exactly: an import
 * consults the tables a model-independent number of times as well, which shows
 * up as a small constant the small size pays proportionally more of. The
 * quadratic term this replaces measured 5.64.
 *
 * 4.2 rather than 5.0. A bound of 5.0 separates 4.0 from 5.64 in the sense of
 * lying between them, but it also admits a growth of 4.9 -- most of the way back
 * to the defect -- without saying anything. Just above measured linear is the
 * bound that fails when the quadratic term starts to return, which is the only
 * event this number exists to catch.
 */
constexpr double MaxLookupGrowth = 4.2;

/** Rows a table lookup may examine on average, unchanged by the batching. */
constexpr double MaxRowsPerLookup = 4.0;

/** A flat robot of `BodyCount` links, each with a joint, a geom and a site. */
FString WideModel(int32 BodyCount)
{
	FString Xml = TEXT("<mujoco model=\"child_queries\">\n  <worldbody>\n");
	for (int32 Index = 0; Index < BodyCount; ++Index)
	{
		Xml += FString::Printf(TEXT("    <body name=\"link%d\" pos=\"%.3f 0 0.5\">\n"), Index, 0.25 * Index);
		Xml += FString::Printf(TEXT("      <joint name=\"hinge%d\" type=\"hinge\" axis=\"0 0 1\"/>\n"), Index);
		Xml += FString::Printf(
			TEXT("      <geom name=\"shape%d\" type=\"box\" size=\"0.05 0.05 0.05\"/>\n"), Index);
		Xml += FString::Printf(TEXT("      <site name=\"mount%d\" size=\"0.01\"/>\n"), Index);
		Xml += TEXT("    </body>\n");
	}
	Xml += TEXT("  </worldbody>\n</mujoco>\n");
	return Xml;
}

/** What one import of `BodyCount` links cost in schema-table work. */
struct FTableCost
{
	int64 Lookups = 0;
	int64 RowVisits = 0;

	double RowsPerLookup() const { return Lookups > 0 ? static_cast<double>(RowVisits) / Lookups : 0.0; }
};

bool MeasureImport(FAutomationTestBase& Test, int32 BodyCount, FTableCost& Out)
{
	const int64 Lookups = ps::ue::DispatchLookups();
	const int64 Rows = ps::ue::DispatchRowVisits();
	UBlueprint* Blueprint = ParseScratch(Test, WideModel(BodyCount));
	Out.Lookups = ps::ue::DispatchLookups() - Lookups;
	Out.RowVisits = ps::ue::DispatchRowVisits() - Rows;
	return Blueprint != nullptr;
}
}  // namespace MjOrderingTests

bool FMjImportChildQueriesTest::RunTest(const FString& Parameters)
{
	using namespace MjOrderingTests;

	// The first import is thrown away: the tables and the schema's static state
	// are built on first use, and that cost belongs to neither size.
	FTableCost Discard;
	if (!MeasureImport(*this, SmallBodies, Discard))
	{
		return false;
	}

	FTableCost Small;
	FTableCost Large;
	if (!MeasureImport(*this, SmallBodies, Small) || !MeasureImport(*this, LargeBodies, Large))
	{
		return false;
	}

	if (!TestTrue(TEXT("the small import consulted the schema tables"), Small.Lookups > 0))
	{
		return false;
	}

	const double Growth = static_cast<double>(Large.Lookups) / static_cast<double>(Small.Lookups);
	AddInfo(FString::Printf(
		TEXT("BENCH child_queries small_lookups=%lld large_lookups=%lld growth=%.2f rows_per_lookup=%.2f"),
		Small.Lookups, Large.Lookups, Growth, Large.RowsPerLookup()));

	TestTrue(FString::Printf(TEXT("table work grows with the model rather than with its square (%.2fx for a 4x model)"),
				 Growth),
		Growth <= MaxLookupGrowth);

	// The batching must not have been bought by making each lookup do a scan.
	TestTrue(FString::Printf(TEXT("dispatch answers in %.2f rows per lookup"), Large.RowsPerLookup()),
		Large.RowsPerLookup() <= MaxRowsPerLookup);

	return true;
}

#endif  // URLAB_MJ_GEN && WITH_EDITOR
