// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// Import goldens: for every fixture under Content/TestData/parity and
// Content/TestData/boundary, the retained reader and writer (ParseIntoScs /
// WriteFromScs, reached through MjParseIntoBlueprint and FSpecRef::WriteMjcf)
// must reproduce the same authored-field snapshot every time. Boundary models
// are round-tripped but never compiled, so they belong to this net and not to
// the parity one. A recorded golden is compared byte for
// byte, so a regression at the retained boundary shows up as a diff instead of
// silently drifting. A mismatch is a finding for a human to judge; this test
// never rebaselines a golden on its own.
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

#include "MuJoCo/Spec/MjSpecRef.h"

namespace MjImportGoldenTests
{

FString TestDataDir(const TCHAR* Leaf)
{
	return FPaths::Combine(
		FPaths::ProjectPluginsDir(), TEXT("UnrealRoboticsLab"), TEXT("Content"), TEXT("TestData"), Leaf);
}

FString ParityDir()
{
	return TestDataDir(TEXT("parity"));
}

/**
 * Models that exercise the reader and writer but cannot reach a compiled
 * model in this product, so only the round trip is asserted of them.
 */
FString BoundaryDir()
{
	return TestDataDir(TEXT("boundary"));
}

FString GoldensDir()
{
	return TestDataDir(TEXT("goldens"));
}

/** The suffix every golden this test owns is named with. */
const TCHAR* const GoldenSuffix = TEXT(".roundtrip.xml");

/**
 * Every fixture this test round-trips: both corpora.
 *
 * The round trip is a reader-and-writer property, so it holds for a model
 * whether or not the engine can compile it.
 */
TArray<FString> FixtureFiles()
{
	TArray<FString> Found;
	IFileManager::Get().FindFilesRecursive(Found, *ParityDir(), TEXT("*.xml"), true, false);
	IFileManager::Get().FindFilesRecursive(Found, *BoundaryDir(), TEXT("*.xml"), true, false, /*bClearFileNames=*/false);
	Found.Sort();
	return Found;
}

/** Every golden of this test's kind, ignoring goldens other tests own. */
TArray<FString> GoldenFiles()
{
	TArray<FString> Found;
	IFileManager::Get().FindFilesRecursive(
		Found, *GoldensDir(), *(FString(TEXT("*")) + GoldenSuffix), true, false);
	Found.Sort();
	return Found;
}

/** The fixture stem a golden of this kind was recorded from. */
FString StemOfGolden(const FString& GoldenPath)
{
	FString Name = FPaths::GetCleanFilename(GoldenPath);
	Name.RemoveFromEnd(GoldenSuffix);
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
	const FString Name = FString::Printf(TEXT("MjImportGolden_%s"), *FGuid::NewGuid().ToString(EGuidFormats::Digits));
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

/**
 * LF line endings, no BOM: the form both the golden on disk and the writer's
 * in-memory output are compared in, so a checkout's autocrlf setting never
 * enters the comparison.
 */
FString NormalizeForCompare(const FString& Text)
{
	return Text.Replace(TEXT("\r\n"), TEXT("\n")).Replace(TEXT("\r"), TEXT("\n"));
}

/** A unified summary of the first line at which `Golden` and `Written` disagree. */
FString FirstDifference(const FString& Golden, const FString& Written)
{
	TArray<FString> GoldenLines;
	TArray<FString> WrittenLines;
	Golden.ParseIntoArray(GoldenLines, TEXT("\n"), false);
	Written.ParseIntoArray(WrittenLines, TEXT("\n"), false);

	const int32 Count = FMath::Max(GoldenLines.Num(), WrittenLines.Num());
	for (int32 Index = 0; Index < Count; ++Index)
	{
		const FString GoldenLine = GoldenLines.IsValidIndex(Index) ? GoldenLines[Index] : TEXT("<no line>");
		const FString WrittenLine = WrittenLines.IsValidIndex(Index) ? WrittenLines[Index] : TEXT("<no line>");
		if (GoldenLine != WrittenLine)
		{
			return FString::Printf(
				TEXT("line %d:\n- golden : %s\n+ written: %s"), Index + 1, *GoldenLine, *WrittenLine);
		}
	}
	return TEXT("the texts differ only in trailing bytes past their last line");
}

/** Parse `Xml` into a fresh Blueprint's SCS tree and write it straight back out. */
bool RoundTrip(
	FAutomationTestBase& Test, const FString& Label, const FString& Xml, const FString& Filename, FString& OutWritten)
{
	UBlueprint* Blueprint = MakeScratchBlueprint();
	if (Blueprint == nullptr)
	{
		Test.AddError(FString::Printf(TEXT("%s: could not create a scratch Blueprint"), *Label));
		return false;
	}

	const FMjSpecParseResult Parsed = MjParseIntoBlueprint(*Blueprint, Xml, Filename);
	if (!Parsed.IsOk())
	{
		Test.AddError(FString::Printf(TEXT("%s: parse failed: %s"), *Label, *DiagnosticsToString(Parsed.Errors)));
		return false;
	}

	TArray<FMjSpecDiagnostic> WriteErrors;
	OutWritten = FSpecRef::OverBlueprint(*Blueprint).WriteMjcf(&WriteErrors);
	if (WriteErrors.Num() > 0)
	{
		Test.AddError(FString::Printf(TEXT("%s: write failed: %s"), *Label, *DiagnosticsToString(WriteErrors)));
		return false;
	}
	return true;
}

/**
 * Compare `Written` against `Content/TestData/goldens/<Stem>.roundtrip.xml`.
 *
 * An absent golden fails unless capture was asked for, so a golden that was
 * never recorded or has gone missing is reported rather than silently created.
 * An existing golden is never overwritten in either mode: it is the
 * recorded-correct answer, and a mismatch is a finding for a human, not
 * something this test may resolve on its own.
 */
bool CheckAgainstGolden(FAutomationTestBase& Test, const FString& Label, const FString& Stem, const FString& Written)
{
	const FString Normalized = NormalizeForCompare(Written);
	const FString GoldenPath = FPaths::Combine(GoldensDir(), Stem + GoldenSuffix);

	if (!IFileManager::Get().FileExists(*GoldenPath))
	{
		if (!ShouldCaptureGoldens())
		{
			Test.AddError(FString::Printf(
				TEXT("%s: golden '%s' is missing; re-run with URLAB_CAPTURE_GOLDENS=1 to record it"),
				*Label, *GoldenPath));
			return false;
		}
		IFileManager::Get().MakeDirectory(*FPaths::GetPath(GoldenPath), /*Tree=*/true);
		if (!FFileHelper::SaveStringToFile(
				Normalized, *GoldenPath, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))
		{
			Test.AddError(FString::Printf(TEXT("%s: could not write golden '%s'"), *Label, *GoldenPath));
			return false;
		}
		Test.AddInfo(FString::Printf(TEXT("%s: captured golden '%s'"), *Label, *GoldenPath));
		return true;
	}

	FString GoldenText;
	if (!FFileHelper::LoadFileToString(GoldenText, *GoldenPath))
	{
		Test.AddError(FString::Printf(TEXT("%s: could not read golden '%s'"), *Label, *GoldenPath));
		return false;
	}
	const FString NormalizedGolden = NormalizeForCompare(GoldenText);

	if (NormalizedGolden != Normalized)
	{
		Test.AddError(FString::Printf(TEXT("%s: does not match its golden (%s):\n%s"), *Label, *GoldenPath,
			*FirstDifference(NormalizedGolden, Normalized)));
		return false;
	}
	return true;
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
				TEXT("golden '%s' has no fixture '%s.xml' under Content/TestData/parity "
					 "or Content/TestData/boundary; "
					 "delete the golden or restore the fixture"),
				*FPaths::GetCleanFilename(Golden), *Stem));
		}
	}
}

} // namespace MjImportGoldenTests

// ============================================================================
// URLab.Import.Goldens
//   Every fixture under Content/TestData/parity and Content/TestData/boundary
//   round-tripped through the
//   retained MJCF reader and writer and compared against its recorded golden.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FMjImportGoldenTest, "URLab.Import.Goldens", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjImportGoldenTest::RunTest(const FString& Parameters)
{
	using namespace MjImportGoldenTests;

	const TArray<FString> Fixtures = FixtureFiles();
	CheckNoOrphanedGoldens(*this, Fixtures);

	if (Fixtures.Num() == 0)
	{
		if (GoldenFiles().Num() == 0)
		{
			AddInfo(TEXT("no fixtures under Content/TestData/parity or Content/TestData/boundary; "
				"zero fixtures checked"));
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

		FString Written;
		if (!RoundTrip(*this, Label, Xml, Fixture, Written))
		{
			continue;
		}

		CheckAgainstGolden(*this, Label, Stem, Written);
	}

	return true;
}

#endif // URLAB_MJ_GEN && WITH_EDITOR
