// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// What an import costs, in work rather than in seconds.
//
// Three surfaces the reader leans on used to answer every question by walking
// something whole, and an import asks each of them once per element, so the cost
// of reading a model was the square of its size:
//
//   the SCS node map        a whole-Blueprint walk, rebuilt after every node
//                           the reader created or linked
//   the effective context   an index of every element and every default class
//                           in the spec, built afresh for every attribute query
//   the dispatch tables     scanned end to end for a class, a tag or a slot
//
// Each of the three now carries a counter of the work it did, and this measures
// them across a fourfold change in model size. Counters and not wall clock, on
// purpose: a stopwatch on a shared machine fails for reasons that have nothing
// to do with the code, and a count of whole-spec walks says precisely which of
// the three regressed when one of them does.
//
// The two index builds must not grow with the model at all -- one per import is
// the point of the batching and of the single scope -- and the dispatch tables
// must answer in about a row per lookup rather than in a table's height.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#if WITH_EDITOR

#include "Engine/Blueprint.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#include "GameFramework/Actor.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "Misc/Guid.h"
#include "UObject/Package.h"
#include "UObject/UnrealType.h"

#include "MuJoCo/Spec/MjGenHooks.h"
#include "MuJoCo/Spec/MjSpecRef.h"

#if URLAB_MJ_GEN

#include "PrimitiveDrawInterface.h"

#include "MjElementVisualizers.h"

#include "MuJoCo/Elements/MjGeom.h"
#include "MuJoCo/Gen/Elements/Joints/MjJoint.gen.h"
#include "MuJoCo/Gen/MjDispatch.gen.h"
#include "MuJoCo/Spec/MjEffective.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjTreeAdapters.h"

// The measurement has to reach the run's own log, and URLab's categories are not
// exported to this module, so the benchmark carries its own.
DEFINE_LOG_CATEGORY_STATIC(LogMjImportBench, Display, All);

namespace MjImportPerfTests
{
/** The two model sizes, a factor of four apart. */
constexpr int32 SmallBodies = 25;
constexpr int32 LargeBodies = 100;

/**
 * How much the index builds may grow between the two sizes.
 *
 * Zero would be the ideal and is too brittle: the reader opens a small,
 * model-independent number of scopes around the parse, and one more or one fewer
 * of those is not a regression. Quadratic behaviour moves this number by the
 * body count, which is three orders of magnitude away from the slack.
 */
constexpr int64 GrowthSlack = 4;

/** Rows a table lookup may examine on average. A scan examines a hundred-odd. */
constexpr double MaxRowsPerLookup = 4.0;

/** What one import cost, in whole-structure walks and table rows. */
struct FImportCost
{
	int64 NodeMapBuilds = 0;
	int64 EffectiveContextBuilds = 0;
	int64 DispatchLookups = 0;
	int64 DispatchRowVisits = 0;

	double RowsPerLookup() const
	{
		return DispatchLookups > 0 ? static_cast<double>(DispatchRowVisits) / DispatchLookups : 0.0;
	}
};

FImportCost Snapshot()
{
	FImportCost Out;
	Out.NodeMapBuilds = urlab::spec::FMjScsScope::NodeMapBuilds();
	Out.EffectiveContextBuilds = urlab::spec::MjEffectiveContextBuilds();
	Out.DispatchLookups = ps::ue::DispatchLookups();
	Out.DispatchRowVisits = ps::ue::DispatchRowVisits();
	return Out;
}

FImportCost Since(const FImportCost& Start)
{
	const FImportCost Now = Snapshot();
	FImportCost Out;
	Out.NodeMapBuilds = Now.NodeMapBuilds - Start.NodeMapBuilds;
	Out.EffectiveContextBuilds = Now.EffectiveContextBuilds - Start.EffectiveContextBuilds;
	Out.DispatchLookups = Now.DispatchLookups - Start.DispatchLookups;
	Out.DispatchRowVisits = Now.DispatchRowVisits - Start.DispatchRowVisits;
	return Out;
}

/**
 * A flat robot of `BodyCount` links, each with a joint, a geom and a site.
 *
 * Flat rather than a chain because the cost being measured is per element and
 * per whole-spec walk, neither of which depends on the depth. The default class
 * is there so the effective-value reads have a chain to resolve rather than
 * answering from the element alone.
 */
FString BuildModel(int32 BodyCount)
{
	FString Xml = TEXT("<mujoco model=\"import_perf\">\n");
	Xml += TEXT("  <default>\n");
	Xml += TEXT("    <default class=\"link\">\n");
	Xml += TEXT("      <geom type=\"box\" size=\"0.05 0.05 0.05\"/>\n");
	Xml += TEXT("      <joint type=\"hinge\" axis=\"0 0 1\"/>\n");
	Xml += TEXT("    </default>\n");
	Xml += TEXT("  </default>\n");
	Xml += TEXT("  <worldbody>\n");
	for (int32 Index = 0; Index < BodyCount; ++Index)
	{
		Xml += FString::Printf(TEXT("    <body name=\"link%d\" pos=\"%.3f 0 0.5\">\n"),
			Index, 0.25 * Index);
		Xml += FString::Printf(TEXT("      <joint class=\"link\" name=\"hinge%d\"/>\n"), Index);
		Xml += FString::Printf(TEXT("      <geom class=\"link\" name=\"shape%d\"/>\n"), Index);
		Xml += FString::Printf(TEXT("      <site name=\"mount%d\" size=\"0.01\"/>\n"), Index);
		Xml += TEXT("    </body>\n");
	}
	Xml += TEXT("  </worldbody>\n</mujoco>\n");
	return Xml;
}

UBlueprint* MakeScratchBlueprint()
{
	const FString Name = FString::Printf(TEXT("MjImportPerf_%s"), *FGuid::NewGuid().ToString(EGuidFormats::Digits));
	UPackage* Package = CreatePackage(*(TEXT("/Temp/") + Name));
	return FKismetEditorUtilities::CreateBlueprint(AActor::StaticClass(), Package, FName(*Name), BPTYPE_Normal,
		UBlueprint::StaticClass(), UBlueprintGeneratedClass::StaticClass());
}

/** Import a model of `BodyCount` links, reporting what the import cost. */
bool MeasureImport(FAutomationTestBase& Test, int32 BodyCount, FImportCost& OutCost, int32& OutNodes)
{
	UBlueprint* Blueprint = MakeScratchBlueprint();
	if (Blueprint == nullptr)
	{
		Test.AddError(TEXT("could not create a scratch Blueprint"));
		return false;
	}

	const FString Xml = BuildModel(BodyCount);
	const FImportCost Start = Snapshot();
	const FMjSpecParseResult Parsed = MjParseIntoBlueprint(*Blueprint, Xml, TEXT("<inline>"));
	OutCost = Since(Start);

	if (!Parsed.IsOk() || Parsed.Root == nullptr)
	{
		Test.AddError(FString::Printf(TEXT("the %d-body benchmark model did not parse"), BodyCount));
		return false;
	}
	OutNodes = Blueprint->SimpleConstructionScript->GetAllNodes().Num();
	return true;
}

void Report(FAutomationTestBase& Test, int32 BodyCount, int32 Nodes, const FImportCost& Cost)
{
	const FString Bench = FString::Printf(
		TEXT("BENCH import bodies=%d scs_nodes=%d node_map_builds=%lld effective_builds=%lld ")
		TEXT("dispatch_lookups=%lld dispatch_row_visits=%lld rows_per_lookup=%.2f"),
		BodyCount, Nodes, Cost.NodeMapBuilds, Cost.EffectiveContextBuilds, Cost.DispatchLookups,
		Cost.DispatchRowVisits, Cost.RowsPerLookup());
	UE_LOG(LogMjImportBench, Display, TEXT("%s"), *Bench);
	Test.AddInfo(Bench);
}
}  // namespace MjImportPerfTests

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjImportScaling,
	"URLab.Perf.ImportScaling",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjImportScaling::RunTest(const FString& Parameters)
{
	using namespace MjImportPerfTests;

	FImportCost Small;
	FImportCost Large;
	int32 SmallNodes = 0;
	int32 LargeNodes = 0;

	// The first import is thrown away: the dispatch tables and the schema's
	// static state are built on first use, and that cost belongs to neither size.
	FImportCost Discard;
	int32 DiscardNodes = 0;
	if (!MeasureImport(*this, SmallBodies, Discard, DiscardNodes))
	{
		return false;
	}

	if (!MeasureImport(*this, SmallBodies, Small, SmallNodes)
		|| !MeasureImport(*this, LargeBodies, Large, LargeNodes))
	{
		return false;
	}

	Report(*this, SmallBodies, SmallNodes, Small);
	Report(*this, LargeBodies, LargeNodes, Large);

	// The measurement measured something. A zero here is an import that did not
	// happen, which would otherwise pass every bound below.
	if (!TestTrue(TEXT("the large import produced more spec nodes than the small one"),
			LargeNodes > SmallNodes)
		|| !TestTrue(TEXT("the large import performed dispatch lookups"), Large.DispatchLookups > 0))
	{
		return false;
	}

	// The two whole-structure indexes are built per import, not per element, so
	// quadrupling the model must not move either count.
	TestTrue(
		FString::Printf(TEXT("SCS node-map builds do not grow with the model (%lld at %d bodies, %lld at %d)"),
			Small.NodeMapBuilds, SmallBodies, Large.NodeMapBuilds, LargeBodies),
		Large.NodeMapBuilds <= Small.NodeMapBuilds + GrowthSlack);

	TestTrue(
		FString::Printf(TEXT("effective-context builds do not grow with the model (%lld at %d bodies, %lld at %d)"),
			Small.EffectiveContextBuilds, SmallBodies, Large.EffectiveContextBuilds, LargeBodies),
		Large.EffectiveContextBuilds <= Small.EffectiveContextBuilds + GrowthSlack);

	// The dispatch tables answer from an index rather than a scan, at both sizes.
	TestTrue(
		FString::Printf(TEXT("dispatch answers in %.2f rows per lookup at %d bodies"),
			Small.RowsPerLookup(), SmallBodies),
		Small.RowsPerLookup() <= MaxRowsPerLookup);
	TestTrue(
		FString::Printf(TEXT("dispatch answers in %.2f rows per lookup at %d bodies"),
			Large.RowsPerLookup(), LargeBodies),
		Large.RowsPerLookup() <= MaxRowsPerLookup);

	return true;
}

// ============================================================================
// URLab.Perf.PresentationScaling
//   The same two counters, aimed at editing rather than at reading.
//
//   An import walks the document once and is done. Editing asks the same three
//   surfaces the same questions over and over, once per keystroke, and the
//   reports are of a default edit taking tens of seconds on a model of a few
//   hundred elements. This measures where that time goes, in whole-structure
//   walks rather than in seconds, at two model sizes so a term that grows with
//   the model is separable from a constant one.
//
//   The measurement convicted all four suspects and the fixes landed, so the
//   numbers below are now bounds rather than a report. Every probe builds ONE
//   node map and at most one effective context, at either model size: an edit
//   costs the same whether the model has a hundred elements or four hundred.
//   What legitimately grows is the dispatch work inside the single index build,
//   which is one pass over the spec, so that one is bounded by a growth ratio
//   instead.
//
//   What the numbers were before, for the record. Syncing one element cost 3
//   node maps and 3 contexts, and a registration is one of those per component.
//   A whole-spec refresh built one node map PER BODY -- 27 at 25 bodies, 102 at
//   100 -- because the ancestor test opened a fresh scope per element. And a
//   `contype` edit, which cannot change any picture, cost 10 node maps and 8
//   contexts where the same edit on a joint cost 1 and 0.
// ============================================================================

namespace MjImportPerfTests
{
/** The four probes, each aimed at one of the suspects. */
struct FPresentationCost
{
	/** One `IsSharedPresentationInput()`: the ancestor walk's own scope. */
	FImportCost AncestorQuery;

	/** One `SyncPreviewFromSpec()` on a single element: the register-time sync. */
	FImportCost SingleSync;

	/** One `RefreshSpecPresentation()`: the whole-spec refresh. */
	FImportCost SpecRefresh;

	/** One details-panel edit of `size` on an ordinary geom. */
	FImportCost PlainGeomEdit;

	/** The same panel, on a geom attribute the picture is not derived from. */
	FImportCost PlainGeomPhysicsEdit;

	/** The same edit on a geom inside a `<default>` class: the reported case. */
	FImportCost DefaultClassEdit;

	/** The same edit on a joint, which has no visualizer of its own. */
	FImportCost PlainJointEdit;

	/** Creating one component under an existing body and registering it. */
	FImportCost Spawn;
};

/** Every element of `Blueprint`'s construction script, templates included. */
TArray<UMjNodeComponent*> ElementsOf(UBlueprint& Blueprint)
{
	TArray<UMjNodeComponent*> Out;
	if (Blueprint.SimpleConstructionScript == nullptr)
	{
		return Out;
	}
	for (USCS_Node* Node : Blueprint.SimpleConstructionScript->GetAllNodes())
	{
		if (UMjNodeComponent* Element = Node != nullptr ? Cast<UMjNodeComponent>(Node->ComponentTemplate) : nullptr)
		{
			Out.Add(Element);
		}
	}
	return Out;
}

/** The SCS node whose element carries `MjName`, or null. */
USCS_Node* NodeNamed(UBlueprint& Blueprint, const FString& MjName)
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

/** Announce an edit of `PropertyName` on `Element` the way the details panel does. */
void NotifyEdited(UMjNodeComponent& Element, const TCHAR* PropertyName)
{
	FProperty* Property = Element.GetClass()->FindPropertyByName(FName(PropertyName));
	FPropertyChangedEvent Event(Property);
	Element.PostEditChangeProperty(Event);
}

/** Run `Body` and report what the two whole-structure indexes did during it. */
template <class Fn>
FImportCost CostOf(Fn&& Body)
{
	const FImportCost Start = Snapshot();
	Body();
	return Since(Start);
}

void ReportProbe(FAutomationTestBase& Test, const TCHAR* Probe, int32 BodyCount, const FImportCost& Cost)
{
	const FString Bench = FString::Printf(
		TEXT("BENCH presentation probe=%s bodies=%d node_map_builds=%lld effective_builds=%lld ")
		TEXT("dispatch_lookups=%lld dispatch_row_visits=%lld"),
		Probe, BodyCount, Cost.NodeMapBuilds, Cost.EffectiveContextBuilds, Cost.DispatchLookups,
		Cost.DispatchRowVisits);
	UE_LOG(LogMjImportBench, Display, TEXT("%s"), *Bench);
	Test.AddInfo(Bench);
}

/**
 * Import a model of `BodyCount` links and run every probe over it.
 *
 * The import itself is outside every measurement: what is being measured is what
 * one edit costs on a model that is already there, which is the situation the
 * reports describe.
 */
bool MeasurePresentation(FAutomationTestBase& Test, int32 BodyCount, FPresentationCost& Out, int32& OutElements)
{
	UBlueprint* Blueprint = MakeScratchBlueprint();
	if (Blueprint == nullptr)
	{
		Test.AddError(TEXT("could not create a scratch Blueprint"));
		return false;
	}
	const FMjSpecParseResult Parsed = MjParseIntoBlueprint(*Blueprint, BuildModel(BodyCount), TEXT("<inline>"));
	if (!Parsed.IsOk())
	{
		Test.AddError(FString::Printf(TEXT("the %d-body presentation model did not parse"), BodyCount));
		return false;
	}

	const TArray<UMjNodeComponent*> Elements = ElementsOf(*Blueprint);
	OutElements = Elements.Num();

	// The two geoms the probes act on: one ordinary, one declared inside the
	// `<default class="link">` block. They differ in exactly one property --
	// whether other elements read them -- which is the branch under suspicion.
	UMjGeomBase* PlainGeom = nullptr;
	UMjGeomBase* DefaultGeom = nullptr;
	UMjNodeComponent* PlainJoint = nullptr;
	for (UMjNodeComponent* Element : Elements)
	{
		if (UMjGeomBase* Geom = Cast<UMjGeomBase>(Element))
		{
			if (Geom->IsClassPartial())
			{
				DefaultGeom = DefaultGeom != nullptr ? DefaultGeom : Geom;
			}
			else
			{
				PlainGeom = PlainGeom != nullptr ? PlainGeom : Geom;
			}
		}
		else if (UMjJoint* Joint = Cast<UMjJoint>(Element))
		{
			if (!Joint->IsClassPartial())
			{
				PlainJoint = PlainJoint != nullptr ? PlainJoint : Joint;
			}
		}
	}

	if (PlainGeom == nullptr || DefaultGeom == nullptr || PlainJoint == nullptr)
	{
		Test.AddError(FString::Printf(
			TEXT("the %d-body model did not yield the three elements the probes need (plain geom %s, class geom %s, "
				 "joint %s)"),
			BodyCount, PlainGeom != nullptr ? TEXT("yes") : TEXT("no"), DefaultGeom != nullptr ? TEXT("yes") : TEXT("no"),
			PlainJoint != nullptr ? TEXT("yes") : TEXT("no")));
		return false;
	}

	// Suspect 1: the ancestor walk opens its own scope, once per call, and the
	// callers ask it per element rather than per pass.
	Out.AncestorQuery = CostOf([PlainGeom] { PlainGeom->IsSharedPresentationInput(); });

	// Suspect 2: the register-time sync. `SyncPreviewUnderOneScope` is what
	// `OnRegister` calls, so it is what is measured; calling the inner sync
	// directly would measure a path no registration takes.
	Out.SingleSync = CostOf([PlainGeom] { PlainGeom->SyncPreviewUnderOneScope(); });

	// Suspect 3: the whole-spec refresh, which is what a shared-input edit
	// triggers. Its own scopes are the point; what matters is what it costs per
	// element of the model, which is the ratio between the two sizes.
	Out.SpecRefresh = CostOf([PlainGeom] { PlainGeom->RefreshSpecPresentation(); });

	// Suspect 4: the geom visualizer is rebuilt on every property change. The
	// joint edit is the control -- same edit, same base class hooks, no
	// visualizer override -- so the difference is what the rebuild costs.
	Out.PlainGeomEdit = CostOf([PlainGeom] { NotifyEdited(*PlainGeom, TEXT("Size")); });
	Out.PlainJointEdit = CostOf([PlainJoint] { NotifyEdited(*PlainJoint, TEXT("Damping")); });

	// The same panel, on a collision mask. Nothing about the picture depends on
	// it, so it is the one probe that says whether the rebuild is gated on what
	// changed or merely on something having changed.
	Out.PlainGeomPhysicsEdit = CostOf([PlainGeom] { NotifyEdited(*PlainGeom, TEXT("Contype")); });

	// The reported case: editing a value on a `<default>` class.
	Out.DefaultClassEdit = CostOf([DefaultGeom] { NotifyEdited(*DefaultGeom, TEXT("Size")); });

	// A spawn: a node created under an existing body the way the components
	// panel creates one, then the register that follows it.
	USCS_Node* const Parent = NodeNamed(*Blueprint, FString::Printf(TEXT("link%d"), BodyCount / 2));
	if (Parent == nullptr)
	{
		Test.AddError(FString::Printf(TEXT("the %d-body model has no body to spawn under"), BodyCount));
		return false;
	}
	Out.Spawn = CostOf([Blueprint, Parent] {
		USCS_Node* const Added =
			Blueprint->SimpleConstructionScript->CreateNode(UMjGeom::StaticClass(), FName(TEXT("Spawned")));
		if (Added == nullptr)
		{
			return;
		}
		Parent->AddChildNode(Added);
		if (UMjNodeComponent* Element = Cast<UMjNodeComponent>(Added->ComponentTemplate))
		{
			// A template never registers, so the sync `OnRegister` would perform
			// is invoked directly; it is the same call on the same object.
			Element->SyncPreviewUnderOneScope();
		}
	});

	return true;
}
}  // namespace MjImportPerfTests

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjPresentationScaling,
	"URLab.Perf.PresentationScaling",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjPresentationScaling::RunTest(const FString& Parameters)
{
	using namespace MjImportPerfTests;

	// First-use costs belong to neither size, the same reason the import
	// benchmark discards its first import.
	FPresentationCost Discard;
	int32 DiscardElements = 0;
	if (!MeasurePresentation(*this, SmallBodies, Discard, DiscardElements))
	{
		return false;
	}

	FPresentationCost Small;
	FPresentationCost Large;
	int32 SmallElements = 0;
	int32 LargeElements = 0;
	if (!MeasurePresentation(*this, SmallBodies, Small, SmallElements)
		|| !MeasurePresentation(*this, LargeBodies, Large, LargeElements))
	{
		return false;
	}

	AddInfo(FString::Printf(TEXT("BENCH presentation elements small=%d large=%d"), SmallElements, LargeElements));

	/** What one probe may cost, at either model size. */
	struct FProbeRow
	{
		const TCHAR* Name;
		const FImportCost* Small;
		const FImportCost* Large;

		/** Node maps the probe builds. One means it builds its own and no more. */
		int64 NodeMapBuilds;

		/** Effective contexts it builds. Zero means it never resolved a class chain. */
		int64 EffectiveContextBuilds;

		/**
		 * How much the dispatch work may grow across a fourfold model.
		 *
		 * One where the probe touches a fixed number of elements. Where it builds
		 * an index or walks the spec, the work is one pass over the model and so
		 * is linear in it; the bound is a little over four so that linear passes
		 * and only linear passes fit under it.
		 */
		double DispatchGrowth;
	};
	const FProbeRow Probes[] = {
		{TEXT("ancestor_query"), &Small.AncestorQuery, &Large.AncestorQuery, 1, 0, 1.0},
		{TEXT("single_sync"), &Small.SingleSync, &Large.SingleSync, 1, 1, 4.5},
		{TEXT("spec_refresh"), &Small.SpecRefresh, &Large.SpecRefresh, 1, 1, 4.5},
		{TEXT("plain_geom_edit"), &Small.PlainGeomEdit, &Large.PlainGeomEdit, 1, 1, 4.5},
		{TEXT("plain_geom_physics_edit"), &Small.PlainGeomPhysicsEdit, &Large.PlainGeomPhysicsEdit, 1, 0, 1.0},
		{TEXT("plain_joint_edit"), &Small.PlainJointEdit, &Large.PlainJointEdit, 1, 0, 1.0},
		{TEXT("default_class_edit"), &Small.DefaultClassEdit, &Large.DefaultClassEdit, 1, 1, 4.5},
		{TEXT("spawn"), &Small.Spawn, &Large.Spawn, 1, 1, 4.5},
	};

	for (const FProbeRow& Probe : Probes)
	{
		ReportProbe(*this, Probe.Name, SmallBodies, *Probe.Small);
		ReportProbe(*this, Probe.Name, LargeBodies, *Probe.Large);
	}

	// A probe set that measured nothing at all would report zeroes everywhere and
	// satisfy every bound below.
	TestTrue(TEXT("the larger model produced more elements than the smaller one"), LargeElements > SmallElements);
	if (!TestTrue(TEXT("the probes observed work happening"), Large.SpecRefresh.DispatchLookups > 0))
	{
		return false;
	}

	for (const FProbeRow& Probe : Probes)
	{
		// Exact, at both sizes. A bound of the form "no more than" would pass on
		// a probe that had stopped doing anything, and these are small enough
		// numbers that the difference between one index and two is the finding.
		TestEqual(FString::Printf(TEXT("%s builds %lld node map(s) at %d bodies"), Probe.Name, Probe.NodeMapBuilds,
					  SmallBodies),
			Probe.Small->NodeMapBuilds, Probe.NodeMapBuilds);
		TestEqual(FString::Printf(TEXT("%s builds %lld node map(s) at %d bodies"), Probe.Name, Probe.NodeMapBuilds,
					  LargeBodies),
			Probe.Large->NodeMapBuilds, Probe.NodeMapBuilds);

		TestEqual(FString::Printf(TEXT("%s builds %lld effective context(s) at %d bodies"), Probe.Name,
					  Probe.EffectiveContextBuilds, SmallBodies),
			Probe.Small->EffectiveContextBuilds, Probe.EffectiveContextBuilds);
		TestEqual(FString::Printf(TEXT("%s builds %lld effective context(s) at %d bodies"), Probe.Name,
					  Probe.EffectiveContextBuilds, LargeBodies),
			Probe.Large->EffectiveContextBuilds, Probe.EffectiveContextBuilds);

		const double Growth = Probe.Small->DispatchLookups > 0
			? static_cast<double>(Probe.Large->DispatchLookups) / Probe.Small->DispatchLookups
			: 0.0;
		TestTrue(FString::Printf(TEXT("%s dispatch work grows %.2fx across a fourfold model, bound %.2fx"), Probe.Name,
					 Growth, Probe.DispatchGrowth),
			Growth <= Probe.DispatchGrowth);
	}

	// The two relations the fixes are really about, stated as relations so they
	// survive a change in what an index build happens to cost.
	//
	// An edit to a geom attribute the picture IS derived from costs an index
	// build the joint edit does not need, and nothing else: the joint is the
	// control, same panel, same base-class hooks, no visualiser of its own.
	const int64 JointIndexes = Large.PlainJointEdit.NodeMapBuilds + Large.PlainJointEdit.EffectiveContextBuilds;
	const int64 GeomIndexes = Large.PlainGeomEdit.NodeMapBuilds + Large.PlainGeomEdit.EffectiveContextBuilds;
	TestTrue(FString::Printf(TEXT("a visual geom edit costs %lld index builds against the joint edit's %lld"),
				 GeomIndexes, JointIndexes),
		GeomIndexes <= 2 * JointIndexes);

	// An edit to one it is NOT derived from -- a collision mask -- costs exactly
	// what the joint edit costs. This is the whole of the visualiser gate.
	TestEqual(TEXT("a collision-mask edit builds the node maps a joint edit builds"),
		Large.PlainGeomPhysicsEdit.NodeMapBuilds, Large.PlainJointEdit.NodeMapBuilds);
	TestEqual(TEXT("a collision-mask edit builds the effective contexts a joint edit builds"),
		Large.PlainGeomPhysicsEdit.EffectiveContextBuilds, Large.PlainJointEdit.EffectiveContextBuilds);

	return !HasAnyErrors();
}

// ============================================================================
// URLab.Perf.VisualizerSweep
//   What drawing a selected model costs, per frame.
//
//   The element visualizer draws one component at a time, and every element it
//   draws reads EFFECTIVE values -- the default-class chain resolved the way the
//   compiler will resolve it. Resolving one indexes the whole spec. Done per
//   element that is one index per element per frame, which at a hundred elements
//   is a frame time in the hundreds of milliseconds, and a viewport that slow
//   does not read as slow: the gizmo runs ahead of the component and the drag
//   appears to fight back.
//
//   So the sweep holds ONE index: the first element of the frame builds it and
//   every element after joins it. Asserted as an exact equality at both model
//   sizes, because the difference between one and N is the whole finding.
// ============================================================================

namespace MjImportPerfTests
{
/**
 * A drawing surface that draws nothing.
 *
 * The probe is about what the visualizer ASKS the spec, not about pixels, and
 * an automation run has no scene to draw into. Every call is counted so the
 * test can prove the sweep actually drew rather than bailing out early.
 */
class FCountingPDI : public FPrimitiveDrawInterface
{
public:
	FCountingPDI() : FPrimitiveDrawInterface(nullptr) {}

	int32 Calls = 0;

	virtual bool IsHitTesting() override { return false; }
	virtual void SetHitProxy(HHitProxy* HitProxy) override {}
	virtual void RegisterDynamicResource(FDynamicPrimitiveResource* DynamicResource) override {}
	virtual void AddReserveLines(uint8, int32, bool, bool) override {}
	virtual void DrawSprite(const FVector&, float, float, const FTexture*, const FLinearColor&, uint8, float, float,
		float, float, uint8, float) override
	{
		++Calls;
	}
	virtual void DrawLine(const FVector&, const FVector&, const FLinearColor&, uint8, float, float, bool) override
	{
		++Calls;
	}
	virtual void DrawTranslucentLine(const FVector&, const FVector&, const FLinearColor&, uint8, float, float,
		bool) override
	{
		++Calls;
	}
	virtual void DrawPoint(const FVector&, const FLinearColor&, float, uint8) override { ++Calls; }
	virtual int32 DrawMesh(const FMeshBatch& Mesh) override { return 0; }
};

/** Draw every element of `Blueprint`'s spec once, as one frame's sweep would. */
FImportCost SweepCost(UBlueprint& Blueprint, int32& OutDrawn, int32& OutDrawCalls)
{
	const TArray<UMjNodeComponent*> Elements = ElementsOf(Blueprint);
	OutDrawn = Elements.Num();

	// The visualizer is a local one rather than the registered instance, so the
	// sweep's index is released when this returns instead of at end of frame.
	FMjElementVisualizer Visualizer;
	FCountingPDI PDI;
	const FImportCost Start = Snapshot();
	for (UMjNodeComponent* Element : Elements)
	{
		Visualizer.DrawVisualization(Element, /*View=*/nullptr, &PDI);
	}
	const FImportCost Cost = Since(Start);
	OutDrawCalls = PDI.Calls;
	return Cost;
}

bool MeasureSweep(FAutomationTestBase& Test, int32 BodyCount, FImportCost& OutCost, int32& OutDrawn,
	int32& OutDrawCalls)
{
	UBlueprint* Blueprint = MakeScratchBlueprint();
	if (Blueprint == nullptr)
	{
		Test.AddError(TEXT("could not create a scratch Blueprint"));
		return false;
	}
	if (!MjParseIntoBlueprint(*Blueprint, BuildModel(BodyCount), TEXT("<inline>")).IsOk())
	{
		Test.AddError(FString::Printf(TEXT("the %d-body sweep model did not parse"), BodyCount));
		return false;
	}
	OutCost = SweepCost(*Blueprint, OutDrawn, OutDrawCalls);
	return true;
}
}  // namespace MjImportPerfTests

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjVisualizerSweep,
	"URLab.Perf.VisualizerSweep",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjVisualizerSweep::RunTest(const FString& Parameters)
{
	using namespace MjImportPerfTests;

	// First-use costs belong to neither size, as in the two benchmarks above.
	FImportCost Discard;
	int32 DiscardDrawn = 0;
	int32 DiscardCalls = 0;
	if (!MeasureSweep(*this, SmallBodies, Discard, DiscardDrawn, DiscardCalls))
	{
		return false;
	}

	FImportCost Small;
	FImportCost Large;
	int32 SmallDrawn = 0;
	int32 LargeDrawn = 0;
	int32 SmallCalls = 0;
	int32 LargeCalls = 0;
	if (!MeasureSweep(*this, SmallBodies, Small, SmallDrawn, SmallCalls)
		|| !MeasureSweep(*this, LargeBodies, Large, LargeDrawn, LargeCalls))
	{
		return false;
	}

	ReportProbe(*this, TEXT("visualizer_sweep"), SmallBodies, Small);
	ReportProbe(*this, TEXT("visualizer_sweep"), LargeBodies, Large);
	AddInfo(FString::Printf(TEXT("BENCH sweep elements small=%d large=%d draw_calls small=%d large=%d"), SmallDrawn,
		LargeDrawn, SmallCalls, LargeCalls));

	// The sweep swept. A sweep that drew nothing would satisfy every bound
	// below by doing no work at all.
	if (!TestTrue(TEXT("the large sweep drew more elements than the small one"), LargeDrawn > SmallDrawn)
		|| !TestTrue(TEXT("the sweep issued drawing calls"), LargeCalls > 0))
	{
		return false;
	}

	// Exactly one, at both sizes: the first element of the sweep builds the
	// spec's index and every element after it adopts that one. Before this it
	// was one per element -- 102 at a hundred bodies -- which is the cost the
	// drag was fighting.
	TestEqual(FString::Printf(TEXT("a sweep of %d elements builds one effective context"), SmallDrawn),
		Small.EffectiveContextBuilds, static_cast<int64>(1));
	TestEqual(FString::Printf(TEXT("a sweep of %d elements builds one effective context"), LargeDrawn),
		Large.EffectiveContextBuilds, static_cast<int64>(1));

	// And one template-graph index for the same reason: the spec is held as
	// Blueprint templates, and walking it at all needs that graph open.
	TestEqual(FString::Printf(TEXT("a sweep of %d elements builds one node map"), SmallDrawn),
		Small.NodeMapBuilds, static_cast<int64>(1));
	TestEqual(FString::Printf(TEXT("a sweep of %d elements builds one node map"), LargeDrawn),
		Large.NodeMapBuilds, static_cast<int64>(1));

	return !HasAnyErrors();
}

#endif  // URLAB_MJ_GEN

#endif  // WITH_EDITOR
