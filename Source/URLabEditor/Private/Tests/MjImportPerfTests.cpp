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
#include "Engine/SimpleConstructionScript.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "Misc/Guid.h"
#include "UObject/Package.h"

#include "MuJoCo/Spec/MjGenHooks.h"
#include "MuJoCo/Spec/MjSpecRef.h"

#if URLAB_MJ_GEN

#include "MuJoCo/Gen/MjDispatch.gen.h"
#include "MuJoCo/Spec/MjEffective.h"
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

#endif  // URLAB_MJ_GEN

#endif  // WITH_EDITOR
