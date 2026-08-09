// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
// Licensed under the Apache License, Version 2.0.

// The import factory's own contracts, the ones that sit outside the reader:
// which files it claims, that mesh preparation is allowed to fail the import,
// and that a failed or cancelled import leaves the project as it found it.

#include "CoreMinimal.h"
#include "HAL/FileManager.h"
#include "Misc/AutomationTest.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

#include "MjPythonHelper.h"
#include "MujocoImportFactory.h"

namespace
{
/** A throwaway directory of this test's own, emptied on the way in. */
FString ScratchDir(const TCHAR* Leaf)
{
	const FString Dir = FPaths::ConvertRelativePathToFull(
		FPaths::ProjectSavedDir() / TEXT("URLabTest/ImportFactory") / Leaf);
	IFileManager::Get().DeleteDirectory(*Dir, /*RequireExists=*/false, /*Tree=*/true);
	IFileManager::Get().MakeDirectory(*Dir, /*Tree=*/true);
	return Dir;
}

const TCHAR* kMinimalMjcf =
	TEXT("<mujoco model=\"factory_probe\">\n")
	TEXT("  <worldbody>\n")
	TEXT("    <body name=\"b1\"><geom name=\"g1\" type=\"sphere\" size=\"0.1\"/></body>\n")
	TEXT("  </worldbody>\n")
	TEXT("</mujoco>\n");

/**
 * A stand-in for the preparation script.
 *
 * The real script's contract is all this test needs: read `--out-dir`, write
 * `<stem>_ue.xml` into it, and say through the exit code whether every mesh was
 * prepared. Writing one lets the failing branch be exercised on demand rather
 * than waiting for a mesh that happens to be broken.
 */
FString WriteStubScript(const FString& Dir, const TCHAR* Name, const TCHAR* Body)
{
	const FString Path = Dir / Name;
	FFileHelper::SaveStringToFile(FString(Body), *Path);
	return Path;
}

const TCHAR* kStubSucceeds =
	TEXT("import argparse, pathlib\n")
	TEXT("p = argparse.ArgumentParser()\n")
	TEXT("p.add_argument('xml', type=pathlib.Path)\n")
	TEXT("p.add_argument('--out-dir', dest='out_dir', type=pathlib.Path)\n")
	TEXT("a = p.parse_args()\n")
	TEXT("a.out_dir.mkdir(parents=True, exist_ok=True)\n")
	TEXT("(a.out_dir / (a.xml.stem + '_ue.xml')).write_text(a.xml.read_text())\n")
	TEXT("raise SystemExit(0)\n");

const TCHAR* kStubFails =
	TEXT("import sys\n")
	TEXT("print('mesh cube could not be prepared', file=sys.stderr)\n")
	TEXT("raise SystemExit(7)\n");

const TCHAR* kStubSilentlyWritesNothing =
	TEXT("raise SystemExit(0)\n");

/** The interpreter the factory would use, or empty when there is none to use. */
FString ProbePython()
{
	const FString Python = FMjPythonHelper::ResolvePythonPath();
	return FMjPythonHelper::ValidatePythonBinary(Python) ? Python : FString();
}
}  // namespace

// ============================================================================
// URLab.Import.MeshPreparationOutcomes
//   Preparation decides whether an import may proceed. A run that succeeds
//   hands back its prepared copy, from the Saved directory rather than from
//   beside the model; a run that fails, or that claims success without
//   producing anything, fails the import and carries the reason.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjImportMeshPreparationOutcomes,
	"URLab.Import.MeshPreparationOutcomes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjImportMeshPreparationOutcomes::RunTest(const FString& Parameters)
{
	const FString Python = ProbePython();
	if (Python.IsEmpty())
	{
		AddError(TEXT("No usable Python interpreter; mesh preparation cannot be exercised."));
		return false;
	}

	const FString Dir = ScratchDir(TEXT("Prep"));
	const FString SourceXml = Dir / TEXT("prep_probe.xml");
	FFileHelper::SaveStringToFile(FString(kMinimalMjcf), *SourceXml);

	const FString PrepDir = UMujocoImportFactory::ImportPrepDir(SourceXml);
	TestTrue(TEXT("prepared copies land under the project's Saved directory"),
		PrepDir.StartsWith(FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir())));
	TestFalse(TEXT("prepared copies do not land beside the model"),
		PrepDir.StartsWith(Dir));

	// Succeeding preparation: the prepared copy is what gets parsed.
	{
		const FString Script = WriteStubScript(Dir, TEXT("stub_ok.py"), kStubSucceeds);
		FString OutXml;
		FString Error;
		const bool bOk = UMujocoImportFactory::RunMeshPreparation(
			Python, Script, SourceXml, OutXml, Error);
		TestTrue(TEXT("a clean preparation succeeds"), bOk);
		TestEqual(TEXT("no diagnostic on success"), Error, FString());
		TestEqual(TEXT("the prepared copy is what gets parsed"),
			FPaths::ConvertRelativePathToFull(OutXml),
			FPaths::ConvertRelativePathToFull(PrepDir / TEXT("prep_probe_ue.xml")));
		TestTrue(TEXT("the prepared copy exists"), FPaths::FileExists(OutXml));
		TestFalse(TEXT("nothing was written beside the model"),
			FPaths::FileExists(Dir / TEXT("prep_probe_ue.xml")));
	}

	// Failing preparation: fatal, and the script's stderr comes with it.
	{
		const FString Script = WriteStubScript(Dir, TEXT("stub_fail.py"), kStubFails);
		FString OutXml;
		FString Error;
		const bool bOk = UMujocoImportFactory::RunMeshPreparation(
			Python, Script, SourceXml, OutXml, Error);
		TestFalse(TEXT("a failed preparation fails the import"), bOk);
		TestTrue(TEXT("the diagnostic names the exit code"), Error.Contains(TEXT("7")));
		TestTrue(TEXT("the diagnostic carries the script's stderr"),
			Error.Contains(TEXT("could not be prepared")));
		TestEqual(TEXT("no prepared copy is offered"), OutXml, SourceXml);
	}

	// Preparation that claims success and produced nothing is failing too: the
	// alternative is parsing the unprepared original without saying so.
	{
		const FString Script = WriteStubScript(Dir, TEXT("stub_silent.py"), kStubSilentlyWritesNothing);
		FString OutXml;
		FString Error;
		const bool bOk = UMujocoImportFactory::RunMeshPreparation(
			Python, Script, SourceXml, OutXml, Error);
		TestFalse(TEXT("a preparation that wrote nothing fails the import"), bOk);
		TestTrue(TEXT("the diagnostic says nothing was written"), Error.Contains(TEXT("_ue.xml")));
		TestEqual(TEXT("no prepared copy is offered"), OutXml, SourceXml);
	}

	return true;
}
