// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// The live stock differential: what our pipeline compiles, against what MuJoCo
// compiles from the same authored text, on every run.
//
// The goldens next door are recordings, and a recording is only ever as correct
// as the day it was taken; re-recording after a MuJoCo update would bless
// whatever the code did that day. This asks the question the recordings cannot:
// the reference is `mj_loadXML` (or `mj_parseXMLString`) of the ORIGINAL
// authored document, so nothing our reader or writer does can move both sides
// at once. Comparing our writer's output against our own reader would be that
// false pass, and is deliberately not what happens here.
//
// One divergence is deliberate and is the only one allowed: an element the
// document leaves unnamed reaches our compiled model carrying a generated
// `_ps:` name, because the name is what leaves the engine and the bridge
// reconciles by it. That shows up as name-table entries stock left empty and as
// the `nnames` those entries add. Everything else -- any size, any field, any
// forward-kinematics invariant, any name stock filled differently -- fails.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Engine/Blueprint.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

#include "MuJoCo/Spec/MjSpecBuild.h"
#include "MuJoCo/Spec/MjSpecRef.h"

#include "Tests/MjParitySupport.h"

THIRD_PARTY_INCLUDES_START
#include <mujoco/mujoco.h>

#include "model_diff_lib.h"
THIRD_PARTY_INCLUDES_END

#include <string>

namespace MjStockDiffTests
{

using namespace MjParitySupport;

/** The scratch Blueprint name prefix, so a leaked package says whose it was. */
const TCHAR* const ScratchPrefix = TEXT("MjStockDiff");

/** The one model size a generated name can move. Compared against `SizeDiff::name`. */
const char* const NameTableSize = "nnames";

/** The total allocation every other size is laid out inside: derived, never a cause. */
const char* const BufferSize = "nbuffer";

/**
 * A model MuJoCo's own front end produced, and the spec it came from.
 *
 * The spec is kept for the same reason the built-spec path keeps its own: a
 * compiled model is not obviously independent of the spec that produced it, and
 * nothing here needs it to be.
 */
struct FStockModel
{
	mjSpec* Spec = nullptr;
	mjModel* Model = nullptr;

	FStockModel() = default;
	FStockModel(const FStockModel&) = delete;
	FStockModel& operator=(const FStockModel&) = delete;

	~FStockModel()
	{
		if (Model != nullptr)
		{
			mj_deleteModel(Model);
		}
		if (Spec != nullptr)
		{
			mj_deleteSpec(Spec);
		}
	}
};

/** Compile `Path` the way a user with stock MuJoCo would. */
bool CompileStockFile(FAutomationTestBase& Test, const FString& Label, const FString& Path, FStockModel& Out)
{
	char Error[1024] = {0};
	Out.Model = mj_loadXML(TCHAR_TO_UTF8(*Path), nullptr, Error, sizeof(Error));
	if (Out.Model == nullptr)
	{
		Test.AddError(FString::Printf(
			TEXT("%s: stock mj_loadXML refused the fixture: %s"), *Label, UTF8_TO_TCHAR(Error)));
		return false;
	}
	return true;
}

/** The same, for a document that lives in this file rather than on disk. */
bool CompileStockText(FAutomationTestBase& Test, const FString& Label, const FString& Xml, FStockModel& Out)
{
	char Error[1024] = {0};
	Out.Spec = mj_parseXMLString(TCHAR_TO_UTF8(*Xml), nullptr, Error, sizeof(Error));
	if (Out.Spec == nullptr)
	{
		Test.AddError(
			FString::Printf(TEXT("%s: stock mj_parseXMLString refused the document: %s"), *Label, UTF8_TO_TCHAR(Error)));
		return false;
	}
	Out.Model = mj_compile(Out.Spec, nullptr);
	if (Out.Model == nullptr)
	{
		Test.AddError(FString::Printf(
			TEXT("%s: stock mj_compile refused the document: %s"), *Label, UTF8_TO_TCHAR(mjs_getError(Out.Spec))));
		return false;
	}
	return true;
}

/** What a stock differential came to, split into the explained and the rest. */
struct FStockVerdict
{
	/** Name entries stock left empty and we filled with a generated name. */
	int32 GeneratedNames = 0;

	/** True when the only size that moved is the name table. */
	bool bNameTableSizeOnly = true;

	/** How many model sizes differed at all. */
	int32 SizeDiffs = 0;

	/** Every divergence the generated-name convention does not account for. */
	TArray<FString> Unexplained;
};

/**
 * Split a report into the deliberate divergence and everything else.
 *
 * `Stock` is the report's `a` side throughout, which is what makes the rule
 * directional: a name stock left EMPTY and we generated is the convention, and
 * a name stock filled differently is a finding whichever way round it reads.
 */
FStockVerdict ClassifyStockDiff(const ps::harness::DiffReport& Report)
{
	FStockVerdict Verdict;

	// `nbuffer` is the size of the single allocation every other size is laid
	// out inside, so it is a consequence of the others and never an independent
	// fact about the model: a name table carrying the generated names moves it
	// too, by its own amount, because the layout pads. Counting it as a
	// divergence of its own makes "the generated names and nothing else"
	// unsatisfiable by construction. It is therefore treated as the symptom it
	// is -- accounted for whenever something else moved, and still reported in
	// the impossible case where it is the only thing that did.
	bool bOtherSizeMoved = false;
	for (const ps::harness::SizeDiff& Size : Report.sizes)
	{
		if (Size.name != BufferSize)
		{
			bOtherSizeMoved = true;
			break;
		}
	}

	for (const ps::harness::SizeDiff& Size : Report.sizes)
	{
		if (Size.name == BufferSize && bOtherSizeMoved)
		{
			continue;
		}
		++Verdict.SizeDiffs;
		if (Size.name != NameTableSize)
		{
			Verdict.bNameTableSizeOnly = false;
			Verdict.Unexplained.Add(FString::Printf(TEXT("size %s: stock %lld, ours %lld"), *Utf8ToUe(Size.name),
				static_cast<int64>(Size.a), static_cast<int64>(Size.b)));
		}
	}

	for (const ps::harness::NameDiff& Name : Report.names)
	{
		const FString Ours = Utf8ToUe(Name.b);
		if (Name.a.empty() && Ours.StartsWith(ReservedPrefix))
		{
			++Verdict.GeneratedNames;
			continue;
		}
		Verdict.Unexplained.Add(FString::Printf(TEXT("name %s[%d]: stock '%s', ours '%s'"), *Utf8ToUe(Name.objtype),
			Name.id, *Utf8ToUe(Name.a), *Ours));
	}

	// Fields and invariants are never explained by a name. The comparison skips
	// array fields outright when the sizes disagree, so a report carrying a
	// `nnames` delta has thinner field coverage than one that does not -- which
	// is a property of the fixture, not a licence granted here.
	for (const ps::harness::FieldDiff& Field : Report.fields)
	{
		Verdict.Unexplained.Add(FString::Printf(
			TEXT("field %s: %lld of %lld differ"), *Utf8ToUe(Field.field), static_cast<int64>(Field.num_diff),
			static_cast<int64>(Field.count_a)));
	}
	for (const ps::harness::FieldDiff& Invariant : Report.invariants)
	{
		Verdict.Unexplained.Add(FString::Printf(TEXT("invariant %s: %lld of %lld differ"), *Utf8ToUe(Invariant.field),
			static_cast<int64>(Invariant.num_diff), static_cast<int64>(Invariant.count_a)));
	}

	return Verdict;
}

/** Diff `Stock` against `Ours` at exact tolerance, reporting what did not complete. */
ps::harness::DiffReport DiffAgainstStock(
	FAutomationTestBase& Test, const FString& Label, const mjModel* Stock, const mjModel* Ours)
{
	std::string Err;
	const ps::harness::DiffReport Report =
		ps::harness::DiffModels(Stock, Ours, ps::harness::Tol{0.0, 0.0}, MaxExamples, Err);
	if (!Err.empty())
	{
		Test.AddError(FString::Printf(TEXT("%s: model comparison did not complete: %s"), *Label, *Utf8ToUe(Err)));
	}
	return Report;
}

/** Report every unexplained divergence, with the whole verdict for context. */
void ReportUnexplained(FAutomationTestBase& Test, const FString& Label, const FStockVerdict& Verdict,
	const ps::harness::DiffReport& Report)
{
	if (Verdict.Unexplained.IsEmpty())
	{
		return;
	}
	Test.AddError(FString::Printf(
		TEXT("%s: our compiled model differs from stock MuJoCo's beyond the generated names (%d unexplained):\n"
			 "  %s\nfull comparison:\n%s"),
		*Label, Verdict.Unexplained.Num(), *FString::Join(Verdict.Unexplained, TEXT("\n  ")), *ReportToString(Report)));
}

} // namespace MjStockDiffTests

// ============================================================================
// URLab.Parity.StockDiff
//   Every fixture under Content/TestData/parity, compiled through the spec
//   path and through stock mj_loadXML of the same file, compared at exact
//   tolerance.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStockDiffCorpusTest, "URLab.Parity.StockDiff",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStockDiffCorpusTest::RunTest(const FString& Parameters)
{
	using namespace MjStockDiffTests;

	const TArray<FString> Fixtures = FixtureFiles();
	if (Fixtures.Num() == 0)
	{
		AddError(TEXT("no fixtures under Content/TestData/parity; the spec path was checked against nothing"));
		return true;
	}

	int32 Compared = 0;
	for (const FString& Fixture : Fixtures)
	{
		const FString Label = FPaths::GetCleanFilename(Fixture);

		FString Xml;
		if (!FFileHelper::LoadFileToString(Xml, *Fixture))
		{
			AddError(FString::Printf(TEXT("%s: could not read fixture"), *Label));
			continue;
		}

		UBlueprint* const Blueprint = ParseFixture(*this, ScratchPrefix, Label, Xml, Fixture);
		if (Blueprint == nullptr)
		{
			continue;
		}

		urlab::spec::FMjBuiltSpec Built;
		mjModel* const Ours = CompileThroughSpecPath(*this, Label, FSpecRef::OverBlueprint(*Blueprint), Built);
		if (Ours == nullptr)
		{
			continue;
		}

		FStockModel Stock;
		if (CompileStockFile(*this, Label, Fixture, Stock))
		{
			const ps::harness::DiffReport Report = DiffAgainstStock(*this, Label, Stock.Model, Ours);
			const FStockVerdict Verdict = ClassifyStockDiff(Report);
			ReportUnexplained(*this, Label, Verdict, Report);
			if (Verdict.GeneratedNames > 0)
			{
				AddInfo(FString::Printf(TEXT("%s: %d generated names, otherwise identical to stock"), *Label,
					Verdict.GeneratedNames));
			}
			++Compared;
		}
		mj_deleteModel(Ours);
	}

	// Every exit from the loop above is a `continue`, so without a floor this
	// passes when the corpus has stopped being found at all.
	TestTrue(TEXT("at least one fixture was compared against stock"), Compared > 0);
	return true;
}

// ============================================================================
// URLab.Parity.StockDiffGeneratedNames
//   The one deliberate divergence, pinned: a document with unnamed elements
//   differs from stock in the generated names and the name table they grow,
//   and in nothing else.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStockDiffGeneratedNamesTest, "URLab.Parity.StockDiffGeneratedNames",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStockDiffGeneratedNamesTest::RunTest(const FString& Parameters)
{
	using namespace MjStockDiffTests;

	const FString Label = TEXT("reserved_names");
	const FString Xml = FString(UnnamedElementsXml);

	UBlueprint* const Blueprint = ParseFixture(*this, ScratchPrefix, Label, Xml, FString());
	if (Blueprint == nullptr)
	{
		return false;
	}

	urlab::spec::FMjBuiltSpec Built;
	mjModel* const Ours = CompileThroughSpecPath(*this, Label, FSpecRef::OverBlueprint(*Blueprint), Built);
	if (Ours == nullptr)
	{
		return false;
	}

	FStockModel Stock;
	if (!CompileStockText(*this, Label, Xml, Stock))
	{
		mj_deleteModel(Ours);
		return false;
	}

	const ps::harness::DiffReport Report = DiffAgainstStock(*this, Label, Stock.Model, Ours);
	const FStockVerdict Verdict = ClassifyStockDiff(Report);
	ReportUnexplained(*this, Label, Verdict, Report);

	// The names themselves, by exact string. The ordinal in a generated name
	// counts its family in document order, so it is a fact about this document
	// and not about the session that read it: the pinning can name the names.
	const TArray<FString> Expected = {
		TEXT("_ps:body:1"),
		TEXT("_ps:body:2"),
		TEXT("_ps:camera:1"),
		TEXT("_ps:geom:1"),
		TEXT("_ps:geom:2"),
		TEXT("_ps:joint:1"),
		TEXT("_ps:joint:2"),
		TEXT("_ps:light:1"),
		TEXT("_ps:site:1"),
	};
	const TArray<FString> Names = ReservedNamesOf(Ours);
	TestEqual(TEXT("the generated names are exactly the ones this document implies"),
		FString::Join(Names, TEXT(", ")), FString::Join(Expected, TEXT(", ")));

	TestEqual(TEXT("stock left exactly those names empty"), Verdict.GeneratedNames, Expected.Num());
	TestTrue(TEXT("the only model size the generated names moved is the name table"), Verdict.bNameTableSizeOnly);
	TestEqual(TEXT("the name table is the only size that moved at all"), Verdict.SizeDiffs, 1);
	TestEqual(TEXT("nothing beyond the generated names differs from stock"), Verdict.Unexplained.Num(), 0);

	mj_deleteModel(Ours);
	return !HasAnyErrors();
}

// ============================================================================
// URLab.Parity.StockDiffUntitledModel
//   A document with no `model` attribute compiles to exactly what stock
//   compiles it to, model name included.
//
//   This is the difference from stock that used to exist and no longer does:
//   an unauthored root once compiled under a stand-in name of our own, so the
//   same document reached a different name table depending on which path built
//   it. Nothing about the name is special-cased here -- the whole comparison is
//   asserted empty.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStockDiffUntitledModelTest, "URLab.Parity.StockDiffUntitledModel",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStockDiffUntitledModelTest::RunTest(const FString& Parameters)
{
	using namespace MjStockDiffTests;

	const FString Label = TEXT("untitled_model");
	const FString Xml = FString(UntitledModelXml);

	UBlueprint* const Blueprint = ParseFixture(*this, ScratchPrefix, Label, Xml, FString());
	if (Blueprint == nullptr)
	{
		return false;
	}

	urlab::spec::FMjBuiltSpec Built;
	mjModel* const Ours = CompileThroughSpecPath(*this, Label, FSpecRef::OverBlueprint(*Blueprint), Built);
	if (Ours == nullptr)
	{
		return false;
	}

	FStockModel Stock;
	if (!CompileStockText(*this, Label, Xml, Stock))
	{
		mj_deleteModel(Ours);
		return false;
	}

	const ps::harness::DiffReport Report = DiffAgainstStock(*this, Label, Stock.Model, Ours);
	if (Report.Differs())
	{
		AddError(FString::Printf(TEXT("%s: an untitled document does not compile to stock's model:\n%s"), *Label,
			*ReportToString(Report)));
	}

	// Named rather than merely implied by the empty diff: the name buffer opens
	// with the model name, so this is the byte the item is about.
	TestEqual(TEXT("the model name is MuJoCo's own default"), FString(UTF8_TO_TCHAR(Ours->names)),
		FString(TEXT("MuJoCo Model")));

	mj_deleteModel(Ours);
	return !HasAnyErrors();
}

#endif // URLAB_MJ_GEN && WITH_EDITOR
