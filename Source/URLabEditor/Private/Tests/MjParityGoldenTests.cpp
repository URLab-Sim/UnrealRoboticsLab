// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// Compile parity: for every fixture under Content/TestData/parity, the model
// the spec pipeline compiles must stay structurally identical to the model
// recorded for it. The recording is a serialized mjModel, and the comparison is
// mjxmacro-driven, so it covers every size, every name table, every array field
// and the forward-kinematics invariants at qpos0 rather than a hand-picked
// subset. It runs at exact tolerance: a compile that moves a single field is a
// finding, and judging it is a human's job, never this test's.
//
// Nothing is accounted for and nothing is forgiven. A generated `_ps:` name is
// stable across runs because its serial is a per-family ordinal in document
// order, which repeats, so the recorded name and ours are the same string and
// the comparison is exact. A reserved name that moves is a real change in what
// the document compiles to, and is reported like any other.
//
// A missing golden fails. Coverage that disappears -- a golden deleted,
// renamed, or never staged -- has to be as loud as a mismatch, or the suite
// passes by testing nothing. Recording is therefore an explicit operator
// action: run with URLAB_CAPTURE_GOLDENS=1 to capture the goldens that are
// absent. Even then an existing golden is left alone.
//
// A recorded mjModel is only meaningful against the engine that produced it:
// the serialized form carries whatever fields that version had, in that
// version's order. `CAPTURE.json` records which engine that was, and it is read
// before any recording is loaded, so a MuJoCo update fails here with an
// explanation instead of failing inside mj_loadModel with a byte offset.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Dom/JsonObject.h"
#include "Engine/Blueprint.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformMisc.h"
#include "Misc/DateTime.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

#include "MuJoCo/Spec/MjSceneSpec.h"
#include "MuJoCo/Spec/MjSpecBuild.h"
#include "MuJoCo/Spec/MjSpecRef.h"

#include "Tests/MjParitySupport.h"

THIRD_PARTY_INCLUDES_START
#include <mujoco/mujoco.h>

#include "model_diff_lib.h"
THIRD_PARTY_INCLUDES_END

#include <string>

namespace MjParityGoldenTests
{

using namespace MjParitySupport;

/** The suffix a single-spec golden is named with. */
const TCHAR* const GoldenSuffix = TEXT(".parity.mjb");

/** What a two-participant scene golden adds before that suffix. */
const TCHAR* const SceneInfix = TEXT(".scene2");

/** The one fixture the two-participant scene is assembled from. */
const TCHAR* const SceneFixtureStem = TEXT("assets_meshes");

/** The scratch Blueprint name prefix, so a leaked package says whose it was. */
const TCHAR* const ScratchPrefix = TEXT("MjParityGolden");

/** What the recordings under `Content/TestData/goldens` were produced by. */
const TCHAR* const CaptureManifestName = TEXT("CAPTURE.json");

FString CaptureManifestPath()
{
	return FPaths::Combine(GoldensDir(), CaptureManifestName);
}

/** Every golden of this test's kind, ignoring goldens other tests own. */
TArray<FString> GoldenFiles()
{
	TArray<FString> Found;
	IFileManager::Get().FindFilesRecursive(Found, *GoldensDir(), *(FString(TEXT("*")) + GoldenSuffix), true, false);
	Found.Sort();
	return Found;
}

/** The fixture stem a golden was recorded from, scene goldens included. */
FString StemOfGolden(const FString& GoldenPath)
{
	FString Name = FPaths::GetCleanFilename(GoldenPath);
	Name.RemoveFromEnd(GoldenSuffix);
	Name.RemoveFromEnd(SceneInfix);
	return Name;
}

/** Writing goldens is an operator action, requested through the environment. */
bool ShouldCaptureGoldens()
{
	return FPlatformMisc::GetEnvironmentVariable(TEXT("URLAB_CAPTURE_GOLDENS")) == TEXT("1");
}

/** What `CAPTURE.json` records about the engine the goldens came from. */
struct FCaptureManifest
{
	/** `mj_versionString()` as it read at capture. The one runtime-checkable field. */
	FString MujocoVersion;

	/** The `third_party/MuJoCo/src` submodule commit, for the human reading a failure. */
	FString MujocoSubmodule;

	/** The plugin commit the capture ran at, and the day it ran. */
	FString CapturedCommit;
	FString CapturedUtc;
};

/**
 * Read the manifest at `Path`.
 *
 * `OutError` is set and false returned when the file is absent or is not the
 * object this expects; an unreadable manifest is treated exactly like a missing
 * one, because both leave the recordings unattributed.
 */
bool ReadCaptureManifest(const FString& Path, FCaptureManifest& Out, FString& OutError)
{
	FString Text;
	if (!FFileHelper::LoadFileToString(Text, *Path))
	{
		OutError = FString::Printf(TEXT("could not read '%s'"), *Path);
		return false;
	}

	TSharedPtr<FJsonObject> Root;
	const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Text);
	if (!FJsonSerializer::Deserialize(Reader, Root) || !Root.IsValid())
	{
		OutError = FString::Printf(TEXT("'%s' is not a JSON object"), *Path);
		return false;
	}

	if (!Root->TryGetStringField(TEXT("mujoco_version"), Out.MujocoVersion) || Out.MujocoVersion.IsEmpty())
	{
		OutError = FString::Printf(TEXT("'%s' carries no 'mujoco_version'"), *Path);
		return false;
	}
	Root->TryGetStringField(TEXT("mujoco_submodule"), Out.MujocoSubmodule);
	Root->TryGetStringField(TEXT("captured_commit"), Out.CapturedCommit);
	Root->TryGetStringField(TEXT("captured_utc"), Out.CapturedUtc);
	return true;
}

/** Write what this run knows about itself. Only ever called for an absent manifest. */
bool WriteCaptureManifest(const FString& Path, FString& OutError)
{
	// The two commit fields are the operator's to fill: nothing inside a running
	// editor knows which commit it was built from, and a guess recorded here
	// would be worse than a blank a human has to complete.
	const FString Text = FString::Printf(TEXT("{\n")
											 TEXT("  \"mujoco_version\": \"%s\",\n")
												 TEXT("  \"mujoco_submodule\": \"\",\n")
													 TEXT("  \"captured_commit\": \"\",\n")
														 TEXT("  \"captured_utc\": \"%s\"\n")
															 TEXT("}\n"),
		UTF8_TO_TCHAR(mj_versionString()), *FDateTime::UtcNow().ToString(TEXT("%Y-%m-%d")));

	IFileManager::Get().MakeDirectory(*FPaths::GetPath(Path), /*Tree=*/true);
	if (!FFileHelper::SaveStringToFile(Text, *Path, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))
	{
		OutError = FString::Printf(TEXT("could not write '%s'"), *Path);
		return false;
	}
	return true;
}

/**
 * Why the recordings cannot be trusted against this build, or empty.
 *
 * Separated from the tests that call it so the explanation itself can be
 * asserted: a check whose message nobody reads is a check that will be
 * satisfied by re-recording, which is exactly the outcome it exists to prevent.
 */
FString CaptureMismatchExplanation(const FString& ManifestPath)
{
	FCaptureManifest Manifest;
	FString ReadError;
	if (!ReadCaptureManifest(ManifestPath, Manifest, ReadError))
	{
		return FString::Printf(
			TEXT("the goldens under Content/TestData/goldens are unattributed: %s. Every recording there is a "
				 "serialized mjModel and only means anything against the MuJoCo that wrote it, so one has to say "
				 "which. Restore the file, or re-record deliberately with URLAB_CAPTURE_GOLDENS=1"),
			*ReadError);
	}

	const FString Running = FString(UTF8_TO_TCHAR(mj_versionString()));
	if (Manifest.MujocoVersion == Running)
	{
		return FString();
	}

	return FString::Printf(
		TEXT("the goldens under Content/TestData/goldens were recorded against MuJoCo %s (submodule %s, plugin "
			 "commit %s, %s) and this build is MuJoCo %s. A serialized mjModel carries the fields that version had, "
			 "in that version's order, so the recordings do not describe this engine's output and a mismatch below "
			 "would say nothing. Re-record only after the live stock differential (URLab.Parity.StockDiff) is green "
			 "on this engine, then update %s; re-recording to make a failure go away blesses whatever the code does "
			 "today"),
		*Manifest.MujocoVersion, Manifest.MujocoSubmodule.IsEmpty() ? TEXT("unrecorded") : *Manifest.MujocoSubmodule,
		Manifest.CapturedCommit.IsEmpty() ? TEXT("unrecorded") : *Manifest.CapturedCommit,
		Manifest.CapturedUtc.IsEmpty() ? TEXT("undated") : *Manifest.CapturedUtc, *Running, CaptureManifestName);
}

/**
 * The manifest gate, run before any recording is loaded.
 *
 * Returns false when the recordings must not be read. In capture mode an absent
 * manifest is written rather than refused, because that is the one moment the
 * answer is being produced rather than checked.
 */
bool CaptureManifestAgrees(FAutomationTestBase& Test)
{
	const FString Path = CaptureManifestPath();
	if (!IFileManager::Get().FileExists(*Path) && ShouldCaptureGoldens())
	{
		FString WriteError;
		if (!WriteCaptureManifest(Path, WriteError))
		{
			Test.AddError(WriteError);
			return false;
		}
		Test.AddInfo(FString::Printf(
			TEXT("captured '%s'; fill in mujoco_submodule and captured_commit before committing it"), *Path));
		return true;
	}

	const FString Explanation = CaptureMismatchExplanation(Path);
	if (!Explanation.IsEmpty())
	{
		Test.AddError(Explanation);
		return false;
	}
	return true;
}

/**
 * Every divergence from a golden, formatted one per entry.
 *
 * Nothing is excluded, sizes and names included. The two models come from the
 * same compiler over the same authored input, so every field of one has to be
 * the field of the other; a recording that no longer describes what the code
 * compiles is a finding for a human, never something for this to absorb.
 */
TArray<FString> GoldenDiffs(const ps::harness::DiffReport& Report)
{
	TArray<FString> Differences;

	for (const ps::harness::NameDiff& Name : Report.names)
	{
		Differences.Add(FString::Printf(TEXT("name %s[%d]: golden '%s', ours '%s'"), *Utf8ToUe(Name.objtype), Name.id,
			*Utf8ToUe(Name.a), *Utf8ToUe(Name.b)));
	}

	for (const ps::harness::FieldDiff& Field : Report.fields)
	{
		Differences.Add(FString::Printf(TEXT("field %s: %lld of %lld differ"), *Utf8ToUe(Field.field),
			static_cast<int64>(Field.num_diff), static_cast<int64>(Field.count_a)));
	}

	for (const ps::harness::SizeDiff& Size : Report.sizes)
	{
		Differences.Add(FString::Printf(TEXT("size %s: golden %lld, ours %lld"), *Utf8ToUe(Size.name),
			static_cast<int64>(Size.a), static_cast<int64>(Size.b)));
	}

	for (const ps::harness::FieldDiff& Invariant : Report.invariants)
	{
		Differences.Add(FString::Printf(TEXT("invariant %s: %lld of %lld differ"), *Utf8ToUe(Invariant.field),
			static_cast<int64>(Invariant.num_diff), static_cast<int64>(Invariant.count_a)));
	}

	return Differences;
}

/** Serialize `Model` into the byte form a golden is stored as. */
bool SaveModelBytes(FAutomationTestBase& Test, const FString& Label, const mjModel* Model, const FString& Path)
{
	const mjtSize Size = mj_sizeModel(Model);
	if (Size == 0)
	{
		Test.AddError(FString::Printf(TEXT("%s: mj_sizeModel reported an empty model"), *Label));
		return false;
	}

	TArray<uint8> Buffer;
	Buffer.SetNumUninitialized(static_cast<int32>(Size));
	mj_saveModel(Model, nullptr, Buffer.GetData(), static_cast<int>(Size));

	IFileManager::Get().MakeDirectory(*FPaths::GetPath(Path), /*Tree=*/true);
	if (!FFileHelper::SaveArrayToFile(Buffer, *Path))
	{
		Test.AddError(FString::Printf(TEXT("%s: could not write golden '%s'"), *Label, *Path));
		return false;
	}
	return true;
}

/**
 * Compare `Model` against the golden at `GoldenPath`.
 *
 * An absent golden fails unless capture was asked for, so a golden that was
 * never recorded or has gone missing is reported rather than silently created.
 * An existing golden is never overwritten in either mode: it is the
 * recorded-correct answer, and a mismatch is a finding for a human.
 */
void CheckAgainstGolden(
	FAutomationTestBase& Test, const FString& Label, const FString& GoldenPath, const mjModel* Model)
{
	if (!IFileManager::Get().FileExists(*GoldenPath))
	{
		if (!ShouldCaptureGoldens())
		{
			Test.AddError(FString::Printf(
				TEXT("%s: golden '%s' is missing; re-run with URLAB_CAPTURE_GOLDENS=1 to record it"), *Label,
				*GoldenPath));
			return;
		}
		if (SaveModelBytes(Test, Label, Model, GoldenPath))
		{
			Test.AddInfo(FString::Printf(TEXT("%s: captured golden '%s'"), *Label, *GoldenPath));
		}
		return;
	}

	mjModel* Golden = mj_loadModel(TCHAR_TO_UTF8(*GoldenPath), nullptr);
	if (Golden == nullptr)
	{
		Test.AddError(FString::Printf(TEXT("%s: could not load golden '%s'"), *Label, *GoldenPath));
		return;
	}

	// Exact: the two models come from the same compiler over the same authored
	// input, so any drift at all is the finding this net exists to catch.
	std::string Err;
	const ps::harness::DiffReport Report =
		ps::harness::DiffModels(Golden, Model, ps::harness::Tol{0.0, 0.0}, MaxExamples, Err);
	mj_deleteModel(Golden);

	// Set only when the invariant check could not allocate its mjData, which
	// leaves the report partial rather than clean.
	if (!Err.empty())
	{
		Test.AddError(FString::Printf(TEXT("%s: model comparison did not complete: %s"), *Label, *Utf8ToUe(Err)));
	}

	const TArray<FString> Differences = GoldenDiffs(Report);
	if (!Differences.IsEmpty())
	{
		Test.AddError(FString::Printf(
			TEXT("%s: compiled model differs from its golden (%s), %d differences:\n  %s\nfull comparison:\n%s"), *Label,
			*GoldenPath, Differences.Num(), *FString::Join(Differences, TEXT("\n  ")), *ReportToString(Report)));
	}
}

/**
 * Every golden of this kind must still name a fixture that exists.
 *
 * A golden whose fixture was renamed or deleted records nothing and would
 * otherwise sit in the tree looking like coverage. The missing direction is
 * covered per fixture by CheckAgainstGolden.
 */
void CheckNoOrphanedGoldens(FAutomationTestBase& Test, const TArray<FString>& Fixtures)
{
	TSet<FString> FixtureStems;
	for (const FString& Fixture : Fixtures)
	{
		FixtureStems.Add(FPaths::GetBaseFilename(Fixture));
	}

	for (const FString& Golden : GoldenFiles())
	{
		const FString Stem = StemOfGolden(Golden);
		if (!FixtureStems.Contains(Stem))
		{
			Test.AddError(FString::Printf(
				TEXT("golden '%s' has no fixture '%s.xml' under Content/TestData/parity; "
					 "delete the golden or restore the fixture"),
				*FPaths::GetCleanFilename(Golden), *Stem));
		}
	}
}

/** The MJCF a scene root that contributes nothing of its own is written as. */
const TCHAR* const EmptySceneRootXml = TEXT("<mujoco model=\"scene\"><worldbody></worldbody></mujoco>");

} // namespace MjParityGoldenTests

// ============================================================================
// URLab.Parity.GoldenCaptureManifest
//   The manifest gate itself: a recorded engine that is not this one has to
//   produce the explanation, and this build's own manifest has to pass.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSpecParityCaptureManifestTest, "URLab.Parity.GoldenCaptureManifest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSpecParityCaptureManifestTest::RunTest(const FString& Parameters)
{
	using namespace MjParityGoldenTests;

	TestEqual(TEXT("the committed manifest attributes the goldens to this engine"),
		CaptureMismatchExplanation(CaptureManifestPath()), FString());

	// The interesting direction is the one no committed tree can be in, so it is
	// staged: a manifest naming a version that is not running, in a scratch
	// location, checked for the explanation rather than merely for failing.
	const FString Scratch =
		FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("URLab"), TEXT("Tests"), TEXT("CAPTURE.mismatch.json"));
	const FString Stale = TEXT("0.0.0-not-this-engine");
	const FString StaleText = FString::Printf(TEXT("{\"mujoco_version\": \"%s\", \"mujoco_submodule\": \"deadbeef\", ")
												  TEXT("\"captured_commit\": \"cafef00d\", \"captured_utc\": \"1970-01-01\"}"),
		*Stale);

	IFileManager::Get().MakeDirectory(*FPaths::GetPath(Scratch), /*Tree=*/true);
	if (!FFileHelper::SaveStringToFile(StaleText, *Scratch, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))
	{
		AddError(FString::Printf(TEXT("could not stage '%s'"), *Scratch));
		return false;
	}

	const FString Explanation = CaptureMismatchExplanation(Scratch);
	IFileManager::Get().Delete(*Scratch);

	TestTrue(TEXT("a recorded version that is not running is refused"), !Explanation.IsEmpty());
	TestTrue(TEXT("the refusal names the version the goldens were recorded against"), Explanation.Contains(Stale));
	TestTrue(TEXT("the refusal names the version that is running"),
		Explanation.Contains(FString(UTF8_TO_TCHAR(mj_versionString()))));
	TestTrue(TEXT("the refusal names the submodule the recording came from"), Explanation.Contains(TEXT("deadbeef")));
	TestTrue(TEXT("the refusal says what to do instead of re-recording blindly"),
		Explanation.Contains(TEXT("URLab.Parity.StockDiff")));

	// A missing manifest is the same refusal, not a silent pass.
	const FString Absent =
		FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("URLab"), TEXT("Tests"), TEXT("CAPTURE.absent.json"));
	IFileManager::Get().Delete(*Absent);
	TestTrue(TEXT("an absent manifest is refused too"), !CaptureMismatchExplanation(Absent).IsEmpty());

	return !HasAnyErrors();
}

// ============================================================================
// URLab.Parity.SpecGoldens
//   Every fixture under Content/TestData/parity, built as an mjSpec from its
//   components and compiled from that, compared field for field against its
//   recorded mjModel.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSpecParityGoldenTest, "URLab.Parity.SpecGoldens",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSpecParityGoldenTest::RunTest(const FString& Parameters)
{
	using namespace MjParityGoldenTests;

	// Before any recording is loaded: a recording from another engine cannot be
	// compared against, and the diff it would produce would misdescribe why.
	if (!CaptureManifestAgrees(*this))
	{
		return false;
	}

	const TArray<FString> Fixtures = FixtureFiles();
	CheckNoOrphanedGoldens(*this, Fixtures);

	if (Fixtures.Num() == 0)
	{
		AddError(TEXT("no fixtures under Content/TestData/parity; the spec path was checked against nothing"));
		return true;
	}

	for (const FString& Fixture : Fixtures)
	{
		const FString Stem = FPaths::GetBaseFilename(Fixture);
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
		mjModel* const Model = CompileThroughSpecPath(*this, Label, FSpecRef::OverBlueprint(*Blueprint), Built);
		if (Model == nullptr)
		{
			continue;
		}
		CheckAgainstGolden(*this, Label, FPaths::Combine(GoldensDir(), Stem + GoldenSuffix), Model);
		mj_deleteModel(Model);
	}

	return true;
}

// ============================================================================
// URLab.Parity.SpecSceneGolden
//   One fixture attached twice at different poses, composed through
//   FMjSceneSpecBuilder, so the prefixing and the attach frames are covered as
//   well as the single spec.
//
//   A scene is where the prefixing, the attach frames and the asset namespacing
//   all land on the name table at once, which is why it has a golden of its own.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSpecParitySceneGoldenTest, "URLab.Parity.SpecSceneGolden",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSpecParitySceneGoldenTest::RunTest(const FString& Parameters)
{
	using namespace MjParityGoldenTests;

	if (!CaptureManifestAgrees(*this))
	{
		return false;
	}

	const FString Fixture = FPaths::Combine(ParityDir(), FString(SceneFixtureStem) + TEXT(".xml"));
	const FString Label = FPaths::GetCleanFilename(Fixture);

	FString Xml;
	if (!FFileHelper::LoadFileToString(Xml, *Fixture))
	{
		AddError(FString::Printf(TEXT("%s: could not read fixture"), *Label));
		return true;
	}

	UBlueprint* const Participant = ParseFixture(*this, ScratchPrefix, Label, Xml, Fixture);
	if (Participant == nullptr)
	{
		return true;
	}

	// The builder composes INTO a root spec, where the text path assembled a
	// document around none. A root contributing nothing of its own is the same
	// scene, and it is what a manager authoring no content already is.
	UBlueprint* const Root = ParseFixture(*this, ScratchPrefix, TEXT("scene root"), EmptySceneRootXml, FString());
	if (Root == nullptr)
	{
		return true;
	}

	urlab::spec::FMjSceneSpecBuilder Builder;
	Builder.SetSceneRoot(FSpecRef::OverBlueprint(*Root));

	// Both participants are the same spec, at the poses the golden was recorded
	// from, so any difference is attributable to the composition rather than to
	// the two specs being different.
	const FSpecRef Ref = FSpecRef::OverBlueprint(*Participant);
	urlab::spec::FMjSceneSpecParticipant First;
	First.Spec = Ref;
	First.Prefix = TEXT("p0_");
	Builder.AddParticipant(First);

	urlab::spec::FMjSceneSpecParticipant Second;
	Second.Spec = Ref;
	Second.Prefix = TEXT("p1_");
	Second.MjPos = FVector(1.0, 0.5, 0.25);
	Second.MjQuat = FQuat(FVector(0.0, 0.0, 1.0), FMath::DegreesToRadians(90.0));
	Builder.AddParticipant(Second);

	urlab::spec::FMjCompiledScene Scene = Builder.Compile();
	if (!Scene.IsValid())
	{
		AddError(FString::Printf(TEXT("%s: the scene did not compile: %s"), *Label,
			*DiagnosticsToString(Scene.Errors)));
		return true;
	}

	const FString GoldenPath = FPaths::Combine(GoldensDir(), FString(SceneFixtureStem) + SceneInfix + GoldenSuffix);
	CheckAgainstGolden(*this, Label, GoldenPath, Scene.Model);

	return true;
}

// ============================================================================
// URLab.Parity.SpecReservedNames
//   An element the document leaves unnamed still arrives in the compiled model
//   carrying a name, an authored name is left exactly as it was written, and
//   the generated names are exactly these names, in every run.
//
//   The goldens cannot say the first part: every fixture in the corpus names
//   everything. They now depend on the last part, which is why it is asserted
//   here by exact string rather than by shape -- the ordinal is a count of the
//   family in document order, so it is a fact about the document and a test can
//   spell it out. A serial from a process counter could only ever have been
//   asserted as a pattern.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSpecParityReservedNamesTest, "URLab.Parity.SpecReservedNames",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSpecParityReservedNamesTest::RunTest(const FString& Parameters)
{
	using namespace MjParityGoldenTests;

	// Every generated name `UnnamedElementsXml` produces, spelled out. Two of
	// each family it leaves unnamed twice, one of each it leaves unnamed once,
	// and no ordinal spent on the one geom the document does name.
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

	const FString Label = TEXT("reserved_names");
	UBlueprint* const Blueprint = ParseFixture(*this, ScratchPrefix, Label, UnnamedElementsXml, FString());
	if (Blueprint == nullptr)
	{
		return false;
	}
	const FSpecRef Ref = FSpecRef::OverBlueprint(*Blueprint);

	urlab::spec::FMjBuiltSpec Built;
	mjModel* const ViaSpec = CompileThroughSpecPath(*this, Label, Ref, Built);
	if (ViaSpec == nullptr)
	{
		return false;
	}

	const TArray<FString> Names = ReservedNamesOf(ViaSpec);
	TestEqual(FString::Printf(TEXT("the document's generated names are exactly the expected ones (got: %s)"),
				  *FString::Join(Names, TEXT(", "))),
		FString::Join(Names, TEXT(", ")), FString::Join(Expected, TEXT(", ")));

	TestTrue(TEXT("the authored name survived untouched"), mj_name2id(ViaSpec, mjOBJ_GEOM, "authored") >= 0);

	// The same document, read and compiled again in the same process. Under a
	// process-lifetime serial this was the assertion that could not be made:
	// every name would have moved on. It is what the goldens next door rest on.
	UBlueprint* const Second = ParseFixture(*this, ScratchPrefix, Label, UnnamedElementsXml, FString());
	if (Second != nullptr)
	{
		urlab::spec::FMjBuiltSpec SecondBuilt;
		if (mjModel* const Again = CompileThroughSpecPath(*this, Label, FSpecRef::OverBlueprint(*Second), SecondBuilt))
		{
			TestEqual(TEXT("a second read of the same document reserves the same names"),
				FString::Join(ReservedNamesOf(Again), TEXT(", ")), FString::Join(Names, TEXT(", ")));
			mj_deleteModel(Again);
		}
	}

	mj_deleteModel(ViaSpec);
	return !HasAnyErrors();
}

#endif // URLAB_MJ_GEN && WITH_EDITOR
