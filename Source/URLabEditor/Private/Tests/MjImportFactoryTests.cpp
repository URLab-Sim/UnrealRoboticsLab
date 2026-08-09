// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
// Licensed under the Apache License, Version 2.0.

// The import factory's own contracts, the ones that sit outside the reader:
// which files it claims, that mesh preparation is allowed to fail the import,
// and that a failed or cancelled import leaves the project as it found it.

#include "CoreMinimal.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "Engine/Blueprint.h"
#include "HAL/FileManager.h"
#include "Misc/AutomationTest.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "UObject/Package.h"
#include "UObject/SoftObjectPath.h"

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

/** One import through the factory, into a package of this run's own. */
struct FImportProbe
{
	FString AssetName;
	UPackage* Package = nullptr;
	UObject* Result = nullptr;
	bool bCancelled = false;

	void Run(const FString& XmlPath)
	{
		const FString Unique = FGuid::NewGuid().ToString(EGuidFormats::Digits).Left(12);
		AssetName = FString(TEXT("FactoryProbe_")) + Unique;
		Package = CreatePackage(*(FString(TEXT("/Game/MuJoCoImportsTest/")) + AssetName));

		UMujocoImportFactory* Factory = NewObject<UMujocoImportFactory>();
		Result = Factory->FactoryCreateFile(UBlueprint::StaticClass(), Package, FName(*AssetName),
			RF_Public | RF_Standalone, XmlPath, nullptr, GWarn, bCancelled);
	}

	/** The Blueprint the import would have left behind, if it left one. */
	UBlueprint* Leftover() const { return FindObject<UBlueprint>(Package, *AssetName); }

	bool RegisteredAsAsset() const
	{
		const FString ObjectPath = FString::Printf(TEXT("/Game/MuJoCoImportsTest/%s.%s"),
			*AssetName, *AssetName);
		FAssetRegistryModule& Registry =
			FModuleManager::LoadModuleChecked<FAssetRegistryModule>(TEXT("AssetRegistry"));
		return Registry.Get().GetAssetByObjectPath(FSoftObjectPath(ObjectPath)).IsValid();
	}
};
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
		TestTrue(TEXT("the diagnostic names the exit code"), Error.Contains(TEXT("exit code 7")));
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

// ============================================================================
// URLab.Import.FactoryClaimsOnlyMuJoCoModels
//   `.xml` belongs to everybody. The importer offers itself only for documents
//   whose root tag says MuJoCo, so a settings file or a UI layout is not
//   offered a robot import.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjImportFactoryClaimsOnlyMuJoCoModels,
	"URLab.Import.FactoryClaimsOnlyMuJoCoModels",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjImportFactoryClaimsOnlyMuJoCoModels::RunTest(const FString& Parameters)
{
	const FString Dir = ScratchDir(TEXT("CanImport"));
	UMujocoImportFactory* Factory = NewObject<UMujocoImportFactory>();

	const auto Write = [&Dir](const TCHAR* Name, const TCHAR* Body) {
		const FString Path = Dir / Name;
		FFileHelper::SaveStringToFile(FString(Body), *Path);
		return Path;
	};

	const FString Model = Write(TEXT("model.xml"), kMinimalMjcf);
	const FString Fragment = Write(TEXT("fragment.xml"),
		TEXT("<mujocoinclude>\n  <asset/>\n</mujocoinclude>\n"));
	const FString Declared = Write(TEXT("declared.xml"),
		TEXT("<?xml version=\"1.0\"?>\n<!-- a model -->\n<mujoco model=\"d\"/>\n"));
	const FString Foreign = Write(TEXT("settings.xml"),
		TEXT("<?xml version=\"1.0\"?>\n<configuration><setting name=\"mujoco\"/></configuration>\n"));
	const FString Empty = Write(TEXT("empty.xml"), TEXT(""));
	const FString WrongExtension = Write(TEXT("model.txt"), kMinimalMjcf);

	TestTrue(TEXT("a MuJoCo model is claimed"), Factory->FactoryCanImport(Model));
	TestTrue(TEXT("a mujocoinclude fragment is claimed"), Factory->FactoryCanImport(Fragment));
	TestTrue(TEXT("a declaration and a comment before the root tag are fine"),
		Factory->FactoryCanImport(Declared));
	TestFalse(TEXT("an unrelated XML is not claimed"), Factory->FactoryCanImport(Foreign));
	TestFalse(TEXT("an empty file is not claimed"), Factory->FactoryCanImport(Empty));
	TestFalse(TEXT("the root tag alone is not enough without the extension"),
		Factory->FactoryCanImport(WrongExtension));
	TestFalse(TEXT("a file that is not there is not claimed"),
		Factory->FactoryCanImport(Dir / TEXT("absent.xml")));

	return true;
}

// ============================================================================
// URLab.Import.FailedImportLeavesNothing
//   An articulation left in the content browser reads as a model that
//   imported. A read that fails must leave the project as it found it, and a
//   read that succeeds must still produce the asset.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjImportFailedImportLeavesNothing,
	"URLab.Import.FailedImportLeavesNothing",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjImportFailedImportLeavesNothing::RunTest(const FString& Parameters)
{
	const FString Dir = ScratchDir(TEXT("Outcome"));

	const FString GoodXml = Dir / TEXT("import_good.xml");
	FFileHelper::SaveStringToFile(FString(kMinimalMjcf), *GoodXml);

	// Well-formed XML that the reader rejects: the preparation step is happy
	// with it, so the failure lands after the Blueprint has been made, which is
	// the case that used to leave an empty articulation behind.
	const FString BadXml = Dir / TEXT("import_bad.xml");
	FFileHelper::SaveStringToFile(FString(
		TEXT("<mujoco model=\"import_bad\">\n")
		TEXT("  <worldbody><notathing/></worldbody>\n")
		TEXT("</mujoco>\n")), *BadXml);

	{
		FImportProbe Probe;
		Probe.Run(GoodXml);
		TestNotNull(TEXT("a model that reads produces an asset"), Probe.Result);
		TestFalse(TEXT("a successful import is not a cancellation"), Probe.bCancelled);
		TestNotNull(TEXT("the asset is in its package"), Probe.Leftover());
		TestTrue(TEXT("the asset is registered"), Probe.RegisteredAsAsset());
	}

	AddExpectedErrorPlain(TEXT("notathing"), EAutomationExpectedErrorFlags::Contains, 0);
	AddExpectedErrorPlain(TEXT("Failed to read MJCF"), EAutomationExpectedErrorFlags::Contains, 0);
	AddExpectedErrorPlain(TEXT("produced no spec"), EAutomationExpectedErrorFlags::Contains, 0);

	{
		FImportProbe Probe;
		Probe.Run(BadXml);
		TestNull(TEXT("a model that does not read produces no asset"), Probe.Result);
		TestFalse(TEXT("a failure is not reported as a cancellation"), Probe.bCancelled);
		TestNull(TEXT("nothing is left in the package"), Probe.Leftover());
		TestFalse(TEXT("nothing is left in the asset registry"), Probe.RegisteredAsAsset());
	}

	return true;
}
