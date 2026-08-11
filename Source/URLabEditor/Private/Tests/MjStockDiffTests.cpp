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
#include "HAL/FileManager.h"
#include "HAL/PlatformMisc.h"
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

DEFINE_LOG_CATEGORY_STATIC(LogMjStockDiff, Display, All);

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

/** Compile a document that lives in this file rather than on disk. */
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

/**
 * One document, both ways, compared.
 *
 * Ours is the file imported and compiled through the spec path; stock is
 * `mj_loadXML` of the SAME file, never of anything our writer produced, so a
 * fault in the reader or the writer cannot move both sides together.
 *
 * Errors are raised on `Test` as they are found. The return says only whether a
 * comparison happened at all, which is what a caller counts to know its corpus
 * has not quietly emptied.
 */
bool CompareAgainstStock(FAutomationTestBase& Test, const FString& Label, const FString& Path)
{
	FString Xml;
	if (!FFileHelper::LoadFileToString(Xml, *Path))
	{
		Test.AddError(FString::Printf(TEXT("%s: could not read"), *Label));
		return false;
	}

	// Stock first, and quietly. This test asks whether our import agrees with
	// MuJoCo's, so a document the MuJoCo we link cannot load ITSELF is not one
	// it has an opinion about. Engine plugins are the case that reaches here:
	// `mujoco.pid` lives in a plugin library that is not part of the runtime we
	// ship, and the plugin registry is process-global, so neither front end can
	// resolve it. Building ours first would report the absence of a plugin as
	// an import bug.
	FStockModel Stock;
	{
		char Error[1024] = {0};
		Stock.Model = mj_loadXML(TCHAR_TO_UTF8(*Path), nullptr, Error, sizeof(Error));
		if (Stock.Model == nullptr)
		{
			const FString Line = FString::Printf(
				TEXT("STOCKDIFF %-28s skipped: stock mj_loadXML refused it too (%s)"), *Label,
				UTF8_TO_TCHAR(Error));
			UE_LOG(LogMjStockDiff, Display, TEXT("%s"), *Line);
			Test.AddInfo(Line);
			return false;
		}
	}

	UBlueprint* const Blueprint = ParseFixture(Test, ScratchPrefix, Label, Xml, Path);
	if (Blueprint == nullptr)
	{
		return false;
	}

	urlab::spec::FMjBuiltSpec Built;
	mjModel* const Ours = CompileThroughSpecPath(Test, Label, FSpecRef::OverBlueprint(*Blueprint), Built);
	if (Ours == nullptr)
	{
		return false;
	}

	bool bCompared = false;
	{
		const ps::harness::DiffReport Report = DiffAgainstStock(Test, Label, Stock.Model, Ours);
		const FStockVerdict Verdict = ClassifyStockDiff(Report);
		ReportUnexplained(Test, Label, Verdict, Report);

		// One line per document, always. A single "the corpus matched" cannot
		// tell a reader which documents were in it, and over a corpus of real
		// robots that list is the result. Into the run log as well as the
		// automation report, because the run log is the one a reader has in
		// front of them.
		const FString Line = FString::Printf(
			TEXT("STOCKDIFF %-28s %s (generated names %d, sizes moved %d, unexplained %d)"), *Label,
			Verdict.Unexplained.IsEmpty() ? TEXT("matches stock") : TEXT("DIVERGES FROM STOCK"),
			Verdict.GeneratedNames, Verdict.SizeDiffs, Verdict.Unexplained.Num());
		UE_LOG(LogMjStockDiff, Display, TEXT("%s"), *Line);
		Test.AddInfo(Line);

		bCompared = true;
	}
	mj_deleteModel(Ours);
	return bCompared;
}

/**
 * One representative document per robot in a menagerie checkout.
 *
 * Menagerie gives a robot a directory holding several documents: the robot, a
 * `scene` that includes it and adds a floor and a light, MJX variants tuned for
 * a different solver, and parts on their own such as a gripper. Comparing all of
 * them says little the robot does not, so this takes one per directory.
 *
 * The robot is the document whose name appears in its directory's:
 * `panda.xml` in `franka_emika_panda`, `g1.xml` in `unitree_g1`, `cassie.xml`
 * in `agility_cassie`. That rule picks the robot and leaves behind the parts
 * and the variants, which are named for what they add rather than for the robot
 * (`hand.xml`, `panda_nohand.xml`, `g1_with_hands.xml`).
 *
 * `<include>` is deliberately not avoided. A document that pulls in another is
 * exercising the include path, which is part of what is being checked.
 */
TArray<FString> MenagerieDocuments(const FString& Root)
{
	TArray<FString> Found;
	IFileManager::Get().FindFilesRecursive(Found, *Root, TEXT("*.xml"), true, false);

	TMap<FString, FString> Chosen;
	for (const FString& File : Found)
	{
		FString Relative = File;
		FPaths::MakePathRelativeTo(Relative, *(Root / TEXT("")));
		Relative.ReplaceInline(TEXT("\\"), TEXT("/"));

		// Assets are meshes and textures. A document nested deeper than a robot's
		// own directory is a part of something rather than a robot.
		FString Directory;
		FString Leaf;
		if (!Relative.Split(TEXT("/"), &Directory, &Leaf) || Leaf.Contains(TEXT("/")))
		{
			continue;
		}

		const FString Stem = FPaths::GetBaseFilename(Leaf);

		// MJX variants are the same robot retuned for a different solver, and a
		// scene is the robot plus a floor.
		if (Stem.Contains(TEXT("mjx"), ESearchCase::IgnoreCase)
			|| Stem.StartsWith(TEXT("scene"), ESearchCase::IgnoreCase))
		{
			continue;
		}

		if (Directory.Contains(Stem, ESearchCase::IgnoreCase))
		{
			// The shortest qualifying stem is the robot itself: `panda` and
			// `panda_nohand` cannot both appear here, but if a directory ever named
			// both, the plainer one is the robot.
			FString* const Existing = Chosen.Find(Directory);
			if (Existing == nullptr || Stem.Len() < FPaths::GetBaseFilename(*Existing).Len())
			{
				Chosen.Add(Directory, File);
			}
		}
	}

	TArray<FString> Documents;
	Chosen.GenerateValueArray(Documents);
	Documents.Sort();
	return Documents;
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
	// The corpus is mostly small hand-written documents that exercise one
	// feature each. Exactly one fixture is a real robot off the menagerie, and
	// it is the one whose result a reader of the log actually wants to find --
	// so it is named, and its absence is a failure rather than a quiet corpus
	// that shrank.
	const TCHAR* const RealRobot = TEXT("vx300s.xml");
	bool bComparedTheRobot = false;

	for (const FString& Fixture : Fixtures)
	{
		const FString Label = FPaths::GetCleanFilename(Fixture);
		if (CompareAgainstStock(*this, Label, Fixture))
		{
			bComparedTheRobot = bComparedTheRobot || Label == RealRobot;
			++Compared;
		}
	}

	// Every exit from the loop above is a `continue`, so without a floor this
	// passes when the corpus has stopped being found at all.
	TestTrue(TEXT("at least one fixture was compared against stock"), Compared > 0);
	TestTrue(FString::Printf(TEXT("the real-robot fixture '%s' was among the %d compared"), RealRobot, Compared),
		bComparedTheRobot);
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

// ============================================================================
// URLab.Parity.StockDiffMenagerie
//   The same comparison over a real robot corpus, off by default.
//
//   Point URLAB_MENAGERIE at a mujoco_menagerie checkout and this compares one
//   document per robot against stock. Without it the test says what it wants
//   and passes, because a machine that has no checkout has not failed anything.
//
//   This is the question the fixture corpus cannot answer. The fixtures are
//   small and hand-written, each exercising one feature, and they were written
//   by the same people who wrote the reader. A hundred real robots were not.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStockDiffMenagerieTest, "URLab.Parity.StockDiffMenagerie",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStockDiffMenagerieTest::RunTest(const FString& Parameters)
{
	using namespace MjStockDiffTests;

	const FString Root = FPlatformMisc::GetEnvironmentVariable(TEXT("URLAB_MENAGERIE"));
	if (Root.IsEmpty())
	{
		AddInfo(TEXT("URLAB_MENAGERIE is not set, so no robot corpus was compared. Set it to a "
					 "mujoco_menagerie checkout to run this."));
		return true;
	}
	if (!IFileManager::Get().DirectoryExists(*Root))
	{
		AddError(FString::Printf(TEXT("URLAB_MENAGERIE points at '%s', which is not a directory"), *Root));
		return false;
	}

	const TArray<FString> Documents = MenagerieDocuments(Root);
	if (Documents.Num() == 0)
	{
		AddError(FString::Printf(TEXT("no robot documents found under '%s'"), *Root));
		return false;
	}

	AddInfo(FString::Printf(TEXT("comparing %d robots under %s"), Documents.Num(), *Root));

	int32 Compared = 0;
	for (const FString& Document : Documents)
	{
		// The directory, not the file: every other robot has a `robot.xml` too,
		// and a log of thirty identical labels names nothing.
		FString Relative = Document;
		FPaths::MakePathRelativeTo(Relative, *(Root / TEXT("")));
		Relative.ReplaceInline(TEXT("\\"), TEXT("/"));

		if (CompareAgainstStock(*this, Relative, Document))
		{
			++Compared;
		}
	}

	// Every failure inside the loop is reported and stepped over, so without
	// this the test passes when nothing could be read at all.
	TestTrue(TEXT("at least one robot was compared against stock"), Compared > 0);
	AddInfo(FString::Printf(TEXT("compared %d of %d robots"), Compared, Documents.Num()));
	return true;
}

#endif // URLAB_MJ_GEN && WITH_EDITOR
