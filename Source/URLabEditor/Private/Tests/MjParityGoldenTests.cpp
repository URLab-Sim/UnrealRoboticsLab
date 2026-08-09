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
// A missing golden fails. Coverage that disappears -- a golden deleted,
// renamed, or never staged -- has to be as loud as a mismatch, or the suite
// passes by testing nothing. Recording is therefore an explicit operator
// action: run with URLAB_CAPTURE_GOLDENS=1 to capture the goldens that are
// absent. Even then an existing golden is left alone.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Engine/Blueprint.h"
#include "GameFramework/Actor.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformMisc.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "Misc/FileHelper.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"
#include "UObject/Package.h"

#include "MuJoCo/Spec/MjCompile.h"
#include "MuJoCo/Spec/MjSceneAssembly.h"
#include "MuJoCo/Spec/MjSpecRef.h"

THIRD_PARTY_INCLUDES_START
#include <mujoco/mujoco.h>

#include "model_diff_lib.h"
THIRD_PARTY_INCLUDES_END

namespace MjParityGoldenTests
{

/** Indices sampled per differing field, and entries listed per category. */
constexpr int32 MaxExamples = 4;

/** The suffix a single-spec golden is named with. */
const TCHAR* const GoldenSuffix = TEXT(".parity.mjb");

/** What a two-participant scene golden adds before that suffix. */
const TCHAR* const SceneInfix = TEXT(".scene2");

/** The one fixture the two-participant scene is assembled from. */
const TCHAR* const SceneFixtureStem = TEXT("assets_meshes");

FString ParityDir()
{
	return FPaths::Combine(
		FPaths::ProjectPluginsDir(), TEXT("UnrealRoboticsLab"), TEXT("Content"), TEXT("TestData"), TEXT("parity"));
}

FString GoldensDir()
{
	return FPaths::Combine(
		FPaths::ProjectPluginsDir(), TEXT("UnrealRoboticsLab"), TEXT("Content"), TEXT("TestData"), TEXT("goldens"));
}

/** Every fixture under `Content/TestData/parity`, one file each. */
TArray<FString> FixtureFiles()
{
	TArray<FString> Found;
	IFileManager::Get().FindFilesRecursive(Found, *ParityDir(), TEXT("*.xml"), true, false);
	Found.Sort();
	return Found;
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

/** A throwaway Blueprint in the transient package; nothing reaches disk. */
UBlueprint* MakeScratchBlueprint()
{
	const FString Name = FString::Printf(TEXT("MjParityGolden_%s"), *FGuid::NewGuid().ToString(EGuidFormats::Digits));
	UPackage* Package = CreatePackage(*(TEXT("/Temp/") + Name));
	return FKismetEditorUtilities::CreateBlueprint(AActor::StaticClass(), Package, FName(*Name), BPTYPE_Normal,
		UBlueprint::StaticClass(), UBlueprintGeneratedClass::StaticClass());
}

FString DiagnosticsToString(const TArray<FMjSpecDiagnostic>& Diagnostics)
{
	TArray<FString> Lines;
	for (const FMjSpecDiagnostic& Diagnostic : Diagnostics)
	{
		Lines.Add(Diagnostic.ToString());
	}
	return FString::Join(Lines, TEXT("; "));
}

FString Utf8ToUe(const std::string& Text)
{
	return FString(UTF8_TO_TCHAR(Text.c_str()));
}

/** One field or invariant divergence, with a bounded sample of the indices. */
void AppendFieldDiffs(TArray<FString>& Lines, const TCHAR* Heading, const std::vector<ps::harness::FieldDiff>& Diffs)
{
	if (Diffs.empty())
	{
		return;
	}
	Lines.Add(FString::Printf(TEXT("  %s (%d):"), Heading, static_cast<int32>(Diffs.size())));

	int32 Listed = 0;
	for (const ps::harness::FieldDiff& Diff : Diffs)
	{
		if (Listed++ >= MaxExamples)
		{
			Lines.Add(FString::Printf(TEXT("    ... and %d more"), static_cast<int32>(Diffs.size()) - MaxExamples));
			break;
		}
		Lines.Add(FString::Printf(TEXT("    %s: %lld of %lld differ%s%s"), *Utf8ToUe(Diff.field),
			static_cast<int64>(Diff.num_diff), static_cast<int64>(Diff.count_a),
			Diff.note.empty() ? TEXT("") : TEXT(" -- "), *Utf8ToUe(Diff.note)));
		for (const ps::harness::FieldDiff::Example& Example : Diff.examples)
		{
			Lines.Add(FString::Printf(TEXT("      [%lld] golden %.17g, compiled %.17g"),
				static_cast<int64>(Example.index), Example.a, Example.b));
		}
	}
}

/** The whole verdict, rendered for a failure message. */
FString ReportToString(const ps::harness::DiffReport& Report)
{
	TArray<FString> Lines;
	Lines.Add(FString::Printf(TEXT("  first divergence: %s"), *Utf8ToUe(Report.FirstDivergence())));

	if (!Report.sizes.empty())
	{
		Lines.Add(FString::Printf(TEXT("  sizes (%d):"), static_cast<int32>(Report.sizes.size())));
		int32 Listed = 0;
		for (const ps::harness::SizeDiff& Size : Report.sizes)
		{
			if (Listed++ >= MaxExamples)
			{
				Lines.Add(FString::Printf(
					TEXT("    ... and %d more"), static_cast<int32>(Report.sizes.size()) - MaxExamples));
				break;
			}
			Lines.Add(FString::Printf(TEXT("    %s: golden %lld, compiled %lld"), *Utf8ToUe(Size.name),
				static_cast<int64>(Size.a), static_cast<int64>(Size.b)));
		}
	}

	if (!Report.names.empty())
	{
		Lines.Add(FString::Printf(TEXT("  names (%d):"), static_cast<int32>(Report.names.size())));
		int32 Listed = 0;
		for (const ps::harness::NameDiff& Name : Report.names)
		{
			if (Listed++ >= MaxExamples)
			{
				Lines.Add(FString::Printf(
					TEXT("    ... and %d more"), static_cast<int32>(Report.names.size()) - MaxExamples));
				break;
			}
			Lines.Add(FString::Printf(TEXT("    %s[%d]: golden '%s', compiled '%s'"), *Utf8ToUe(Name.objtype),
				Name.id, *Utf8ToUe(Name.a), *Utf8ToUe(Name.b)));
		}
	}

	AppendFieldDiffs(Lines, TEXT("fields"), Report.fields);
	AppendFieldDiffs(Lines, TEXT("invariants"), Report.invariants);
	return FString::Join(Lines, TEXT("\n"));
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

	if (Report.Differs())
	{
		Test.AddError(FString::Printf(TEXT("%s: compiled model differs from its golden (%s):\n%s"), *Label,
			*GoldenPath, *ReportToString(Report)));
	}
}

/** Parse one fixture into a scratch Blueprint, ready to compile. */
UBlueprint* ParseFixture(FAutomationTestBase& Test, const FString& Label, const FString& Xml, const FString& Path)
{
	UBlueprint* Blueprint = MakeScratchBlueprint();
	if (Blueprint == nullptr)
	{
		Test.AddError(FString::Printf(TEXT("%s: could not create a scratch Blueprint"), *Label));
		return nullptr;
	}

	// A parity fixture is authored against the supported surface, so an
	// unsupported-only parse is a fixture that stopped being covered, not a
	// reason to pass.
	const FMjSpecParseResult Parsed = MjParseIntoBlueprint(*Blueprint, Xml, Path);
	if (!Parsed.IsOk())
	{
		Test.AddError(FString::Printf(TEXT("%s: parse failed: %s"), *Label, *DiagnosticsToString(Parsed.Errors)));
		return nullptr;
	}
	return Blueprint;
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

} // namespace MjParityGoldenTests

// ============================================================================
// URLab.Parity.Goldens
//   Every fixture under Content/TestData/parity compiled through the spec
//   pipeline and compared field for field against its recorded mjModel.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjParityGoldenTest, "URLab.Parity.Goldens",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjParityGoldenTest::RunTest(const FString& Parameters)
{
	using namespace MjParityGoldenTests;

	const TArray<FString> Fixtures = FixtureFiles();
	CheckNoOrphanedGoldens(*this, Fixtures);

	if (Fixtures.Num() == 0)
	{
		if (GoldenFiles().Num() == 0)
		{
			AddInfo(TEXT("no fixtures under Content/TestData/parity; zero fixtures checked"));
		}
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

		UBlueprint* Blueprint = ParseFixture(*this, Label, Xml, Fixture);
		if (Blueprint == nullptr)
		{
			continue;
		}

		// Production options: the goldens record what the pipeline actually
		// produces, auto-naming included.
		FMjCompiled Compiled = MjCompileSpec(FSpecRef::OverBlueprint(*Blueprint));
		if (!Compiled.IsOk())
		{
			AddError(
				FString::Printf(TEXT("%s: spec compile failed: %s"), *Label, *DiagnosticsToString(Compiled.Errors)));
			continue;
		}

		CheckAgainstGolden(*this, Label, FPaths::Combine(GoldensDir(), Stem + GoldenSuffix), Compiled.Model);
	}

	return true;
}

// ============================================================================
// URLab.Parity.SceneGolden
//   One fixture attached twice at different poses, compiled as a scene, so the
//   prefixing and the attach frames are covered as well as the single spec.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjParitySceneGoldenTest, "URLab.Parity.SceneGolden",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjParitySceneGoldenTest::RunTest(const FString& Parameters)
{
	using namespace MjParityGoldenTests;

	const FString Fixture = FPaths::Combine(ParityDir(), FString(SceneFixtureStem) + TEXT(".xml"));
	if (!IFileManager::Get().FileExists(*Fixture))
	{
		AddInfo(FString::Printf(
			TEXT("fixture '%s.xml' is absent; the two-participant scene has nothing to assemble"), SceneFixtureStem));
		return true;
	}
	const FString Label = FPaths::GetCleanFilename(Fixture);

	FString Xml;
	if (!FFileHelper::LoadFileToString(Xml, *Fixture))
	{
		AddError(FString::Printf(TEXT("%s: could not read fixture"), *Label));
		return true;
	}

	UBlueprint* Blueprint = ParseFixture(*this, Label, Xml, Fixture);
	if (Blueprint == nullptr)
	{
		return true;
	}

	// Both participants are the same spec: what the scene adds over the single
	// compile is the prefixing and the two attach frames, and using one spec
	// keeps any difference attributable to those.
	const FSpecRef Ref = FSpecRef::OverBlueprint(*Blueprint);
	FSceneAssembly Scene;
	Scene.Add(Ref, TEXT("p0_"), FVector::ZeroVector, FQuat::Identity);
	Scene.Add(Ref, TEXT("p1_"), FVector(1.0, 0.5, 0.25),
		FQuat(FVector(0.0, 0.0, 1.0), FMath::DegreesToRadians(90.0)));

	FMjCompiled Compiled = MjCompileScene(Scene);
	if (!Compiled.IsOk())
	{
		AddError(FString::Printf(TEXT("%s: scene compile failed: %s"), *Label, *DiagnosticsToString(Compiled.Errors)));
		return true;
	}

	const FString GoldenPath =
		FPaths::Combine(GoldensDir(), FString(SceneFixtureStem) + SceneInfix + GoldenSuffix);
	CheckAgainstGolden(*this, Label, GoldenPath, Compiled.Model);

	return true;
}

#endif // URLAB_MJ_GEN && WITH_EDITOR
