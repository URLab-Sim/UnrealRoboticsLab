// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
// Licensed under the Apache License, Version 2.0.

// The import factory's own contracts, the ones that sit outside the reader:
// which files it claims, that mesh preparation is allowed to fail the import,
// and that a failed or cancelled import leaves the project as it found it.

#include "CoreMinimal.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "EditorReimportHandler.h"
#include "Engine/Blueprint.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformProcess.h"
#include "IMessageLogListing.h"
#include "Interfaces/IPluginManager.h"
#include "MessageLogModule.h"
#include "Misc/AutomationTest.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"
#include "UObject/Package.h"
#include "UObject/SoftObjectPath.h"

#include "Kismet2/KismetEditorUtilities.h"

#include "MjPythonHelper.h"
#include "MujocoGenerationAction.h"
#include "MujocoImportFactory.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjSpecRef.h"

namespace
{
/** A throwaway directory of this test's own, emptied on the way in. */
FString FactoryScratchDir(const TCHAR* Leaf)
{
	const FString Dir = FPaths::ConvertRelativePathToFull(
		FPaths::ProjectSavedDir() / TEXT("URLabTest/ImportFactory") / Leaf);
	IFileManager::Get().DeleteDirectory(*Dir, /*RequireExists=*/false, /*Tree=*/true);
	IFileManager::Get().MakeDirectory(*Dir, /*Tree=*/true);
	return Dir;
}

const TCHAR* kFactoryProbeMjcf =
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
FString WriteFactoryStubScript(const FString& Dir, const TCHAR* Name, const TCHAR* Body)
{
	const FString Path = Dir / Name;
	FFileHelper::SaveStringToFile(FString(Body), *Path);
	return Path;
}

const TCHAR* kFactoryStubSucceeds =
	TEXT("import argparse, pathlib\n")
	TEXT("p = argparse.ArgumentParser()\n")
	TEXT("p.add_argument('xml', type=pathlib.Path)\n")
	TEXT("p.add_argument('--out-dir', dest='out_dir', type=pathlib.Path)\n")
	TEXT("a = p.parse_args()\n")
	TEXT("a.out_dir.mkdir(parents=True, exist_ok=True)\n")
	TEXT("(a.out_dir / (a.xml.stem + '_ue.xml')).write_text(a.xml.read_text())\n")
	TEXT("raise SystemExit(0)\n");

const TCHAR* kFactoryStubFails =
	TEXT("import sys\n")
	TEXT("print('mesh cube could not be prepared', file=sys.stderr)\n")
	TEXT("raise SystemExit(7)\n");

const TCHAR* kFactoryStubSilent =
	TEXT("raise SystemExit(0)\n");

/** Writes its own arguments where the prepared document goes, and nothing else. */
const TCHAR* kFactoryStubRecordsArgs =
	TEXT("import sys, pathlib\n")
	TEXT("out = pathlib.Path(sys.argv[sys.argv.index('--out-dir') + 1])\n")
	TEXT("out.mkdir(parents=True, exist_ok=True)\n")
	TEXT("stem = pathlib.Path(sys.argv[1]).stem\n")
	TEXT("(out / (stem + '_ue.xml')).write_text(' '.join(sys.argv[1:]))\n")
	TEXT("raise SystemExit(0)\n");

/** A model whose only include reaches out of its own folder. */
const TCHAR* kFactoryExternalIncludeMjcf =
	TEXT("<mujoco model=\"include_probe\">\n")
	TEXT("  <include file=\"../outside/extra.xml\"/>\n")
	TEXT("  <worldbody>\n")
	TEXT("    <body name=\"b1\"><geom name=\"g1\" type=\"sphere\" size=\"0.1\"/></body>\n")
	TEXT("  </worldbody>\n")
	TEXT("</mujoco>\n");

/** What that include pulls in: harmless here, and not the point. */
const TCHAR* kFactoryIncludedFragment =
	TEXT("<mujocoinclude>\n")
	TEXT("  <option timestep=\"0.004\"/>\n")
	TEXT("</mujocoinclude>\n");

/** The interpreter the factory would use, or empty when there is none to use. */
FString FactoryProbePython()
{
	const FString Python = FMjPythonHelper::ResolvePythonPath();
	return FMjPythonHelper::ValidatePythonBinary(Python) ? Python : FString();
}

/** One import through the factory, into a package of this run's own. */
struct FFactoryImportProbe
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

	UBlueprint* AsBlueprint() const { return Cast<UBlueprint>(Result); }

	bool RegisteredAsAsset() const
	{
		const FString ObjectPath = FString::Printf(TEXT("/Game/MuJoCoImportsTest/%s.%s"),
			*AssetName, *AssetName);
		FAssetRegistryModule& Registry =
			FModuleManager::LoadModuleChecked<FAssetRegistryModule>(TEXT("AssetRegistry"));
		return Registry.Get().GetAssetByObjectPath(FSoftObjectPath(ObjectPath)).IsValid();
	}
};

/** The construction script's component names, in order. */
TArray<FString> FactoryComponentNames(const UBlueprint* Blueprint)
{
	TArray<FString> Out;
	if (Blueprint == nullptr || Blueprint->SimpleConstructionScript == nullptr)
	{
		return Out;
	}
	for (const USCS_Node* Node : Blueprint->SimpleConstructionScript->GetAllNodes())
	{
		if (Node != nullptr && Node->ComponentTemplate != nullptr)
		{
			Out.Add(FString::Printf(TEXT("%s %s"),
				*Node->ComponentTemplate->GetClass()->GetName(),
				*Node->GetVariableName().ToString()));
		}
	}
	return Out;
}

/** Every authored value in the Blueprint's spec, as the MJCF it writes back. */
FString FactorySpecMjcf(UBlueprint* Blueprint)
{
	return Blueprint != nullptr ? FSpecRef::OverBlueprint(*Blueprint).WriteMjcf() : FString();
}

/** The model file a Blueprint says it came from. */
FString FactoryRecordedSource(const UBlueprint* Blueprint)
{
	if (Blueprint == nullptr || Blueprint->GeneratedClass == nullptr)
	{
		return FString();
	}
	const AMjArticulation* CDO = Cast<AMjArticulation>(Blueprint->GeneratedClass->GetDefaultObject());
	return CDO != nullptr ? CDO->MuJoCoXMLFile.FilePath : FString();
}

/** True when the Blueprint's spec holds an element the model named `MjName`. */
bool FactoryHasElementNamed(const UBlueprint* Blueprint, const FString& MjName)
{
	if (Blueprint == nullptr || Blueprint->SimpleConstructionScript == nullptr)
	{
		return false;
	}
	for (const USCS_Node* Node : Blueprint->SimpleConstructionScript->GetAllNodes())
	{
		const UMjNodeComponent* Element = Node != nullptr
			? Cast<UMjNodeComponent>(Node->ComponentTemplate) : nullptr;
		if (Element != nullptr && Element->MjName.IsSet() && Element->MjName.GetValue() == MjName)
		{
			return true;
		}
	}
	return false;
}

/** An empty articulation Blueprint in a package of this run's own. */
UBlueprint* MakeFactoryScratchBlueprint()
{
	const FString Unique = FGuid::NewGuid().ToString(EGuidFormats::Digits).Left(12);
	UPackage* Package = CreatePackage(*(FString(TEXT("/Temp/URLabImportOptions_")) + Unique));
	return FKismetEditorUtilities::CreateBlueprint(AMjArticulation::StaticClass(), Package,
		*(FString(TEXT("OptionsProbe_")) + Unique), BPTYPE_Normal, UBlueprint::StaticClass(),
		UBlueprintGeneratedClass::StaticClass());
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
	const FString Python = FactoryProbePython();
	if (Python.IsEmpty())
	{
		AddError(TEXT("No usable Python interpreter; mesh preparation cannot be exercised."));
		return false;
	}

	const FString Dir = FactoryScratchDir(TEXT("Prep"));
	const FString SourceXml = Dir / TEXT("prep_probe.xml");
	FFileHelper::SaveStringToFile(FString(kFactoryProbeMjcf), *SourceXml);

	const FString PrepDir = UMujocoImportFactory::ImportPrepDir(SourceXml);
	TestTrue(TEXT("prepared copies land under the project's Saved directory"),
		PrepDir.StartsWith(FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir())));
	TestFalse(TEXT("prepared copies do not land beside the model"),
		PrepDir.StartsWith(Dir));

	// Succeeding preparation: the prepared copy is what gets parsed.
	{
		const FString Script = WriteFactoryStubScript(Dir, TEXT("stub_ok.py"), kFactoryStubSucceeds);
		FString OutXml;
		FString Error;
		const bool bOk = UMujocoImportFactory::RunMeshPreparation(
			Python, Script, SourceXml, /*bAllowExternalIncludes=*/false, OutXml, Error);
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
		const FString Script = WriteFactoryStubScript(Dir, TEXT("stub_fail.py"), kFactoryStubFails);
		FString OutXml;
		FString Error;
		const bool bOk = UMujocoImportFactory::RunMeshPreparation(
			Python, Script, SourceXml, /*bAllowExternalIncludes=*/false, OutXml, Error);
		TestFalse(TEXT("a failed preparation fails the import"), bOk);
		TestTrue(TEXT("the diagnostic names the exit code"), Error.Contains(TEXT("exit code 7")));
		TestTrue(TEXT("the diagnostic carries the script's stderr"),
			Error.Contains(TEXT("could not be prepared")));
		TestEqual(TEXT("no prepared copy is offered"), OutXml, SourceXml);
	}

	// Preparation that claims success and produced nothing is failing too: the
	// alternative is parsing the unprepared original without saying so.
	{
		const FString Script = WriteFactoryStubScript(Dir, TEXT("stub_silent.py"), kFactoryStubSilent);
		FString OutXml;
		FString Error;
		const bool bOk = UMujocoImportFactory::RunMeshPreparation(
			Python, Script, SourceXml, /*bAllowExternalIncludes=*/false, OutXml, Error);
		TestFalse(TEXT("a preparation that wrote nothing fails the import"), bOk);
		TestTrue(TEXT("the diagnostic says nothing was written"), Error.Contains(TEXT("_ue.xml")));
		TestEqual(TEXT("no prepared copy is offered"), OutXml, SourceXml);
	}

	return true;
}

// ============================================================================
// URLab.Import.ExternalIncludeGate
//   The import option that decides whether a model may pull files from
//   elsewhere on disk has to be answered where the includes are actually
//   followed, and mesh preparation follows them first. Two halves: the option
//   reaches the script at all, and the shipped script fails closed on it.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjImportExternalIncludeGate,
	"URLab.Import.ExternalIncludeGate",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjImportExternalIncludeGate::RunTest(const FString& Parameters)
{
	const FString Python = FactoryProbePython();
	if (Python.IsEmpty())
	{
		AddError(TEXT("No usable Python interpreter; mesh preparation cannot be exercised."));
		return false;
	}

	// A model whose include reaches up and out of its own folder, which is the
	// shape the option exists for.
	const FString Dir = FactoryScratchDir(TEXT("IncludeGate"));
	const FString ModelDir = Dir / TEXT("model");
	const FString OutsideDir = Dir / TEXT("outside");
	IFileManager::Get().MakeDirectory(*ModelDir, /*Tree=*/true);
	IFileManager::Get().MakeDirectory(*OutsideDir, /*Tree=*/true);
	FFileHelper::SaveStringToFile(FString(kFactoryIncludedFragment), *(OutsideDir / TEXT("extra.xml")));
	const FString SourceXml = ModelDir / TEXT("include_probe.xml");
	FFileHelper::SaveStringToFile(FString(kFactoryExternalIncludeMjcf), *SourceXml);

	// Half one: the option reaches the script, and only when it is on. Asserted
	// against a stub that writes its own arguments out, so it holds whether or
	// not this machine can run the real preparation.
	const FString Script = WriteFactoryStubScript(Dir, TEXT("stub_args.py"), kFactoryStubRecordsArgs);
	for (const bool bAllow : {false, true})
	{
		FString OutXml;
		FString Error;
		const bool bOk = UMujocoImportFactory::RunMeshPreparation(
			Python, Script, SourceXml, bAllow, OutXml, Error);
		FString Recorded;
		if (TestTrue(TEXT("the stub preparation ran"), bOk)
			&& TestTrue(TEXT("and wrote its arguments"), FFileHelper::LoadFileToString(Recorded, *OutXml)))
		{
			const bool bPassed = Recorded.Contains(TEXT("--allow-external-includes"));
			if (bAllow)
			{
				TestTrue(TEXT("the option is passed when it is on"), bPassed);
			}
			else
			{
				TestFalse(TEXT("the option is not passed when it is off"), bPassed);
			}
		}
	}

	// Half two: the shipped script, which is where the refusal lives.
	const TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("UnrealRoboticsLab"));
	const FString RealScript = Plugin.IsValid()
		? FPaths::Combine(Plugin->GetBaseDir(), TEXT("Scripts/clean_meshes.py")) : FString();
	if (!TestTrue(TEXT("the preparation script ships with the plugin"), FPaths::FileExists(RealScript)))
	{
		return false;
	}

	// The script imports its mesh libraries at module scope, so on a machine
	// without them it cannot run at all. Said out loud rather than skipped
	// silently: this half of the gate is then untested here.
	int32 ProbeCode = -1;
	FString ProbeOut;
	FString ProbeErr;
	FPlatformProcess::ExecProcess(*Python, TEXT("-c \"import trimesh, numpy\""),
		&ProbeCode, &ProbeOut, &ProbeErr);
	if (ProbeCode != 0)
	{
		AddWarning(TEXT("Python has no trimesh/numpy, so the shipped preparation script could not "
						"be run; only the option's plumbing was checked."));
		return !HasAnyErrors();
	}

	{
		FString OutXml;
		FString Error;
		const bool bOk = UMujocoImportFactory::RunMeshPreparation(
			Python, RealScript, SourceXml, /*bAllowExternalIncludes=*/false, OutXml, Error);
		TestFalse(TEXT("preparation refuses an include outside the model's folder"), bOk);
		TestTrue(TEXT("and the diagnostic names the file it refused"),
			Error.Contains(TEXT("extra.xml")));
		TestEqual(TEXT("no prepared copy is offered"), OutXml, SourceXml);
	}
	{
		FString OutXml;
		FString Error;
		const bool bOk = UMujocoImportFactory::RunMeshPreparation(
			Python, RealScript, SourceXml, /*bAllowExternalIncludes=*/true, OutXml, Error);
		TestTrue(TEXT("the option lets the same import through"), bOk);
		TestNotEqual(TEXT("and the prepared copy is what gets parsed"), OutXml, SourceXml);
	}

	return !HasAnyErrors();
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
	const FString Dir = FactoryScratchDir(TEXT("CanImport"));
	UMujocoImportFactory* Factory = NewObject<UMujocoImportFactory>();

	const auto Write = [&Dir](const TCHAR* Name, const TCHAR* Body) {
		const FString Path = Dir / Name;
		FFileHelper::SaveStringToFile(FString(Body), *Path);
		return Path;
	};

	const FString Model = Write(TEXT("model.xml"), kFactoryProbeMjcf);
	const FString Fragment = Write(TEXT("fragment.xml"),
		TEXT("<mujocoinclude>\n  <asset/>\n</mujocoinclude>\n"));
	const FString Declared = Write(TEXT("declared.xml"),
		TEXT("<?xml version=\"1.0\"?>\n<!-- a model -->\n<mujoco model=\"d\"/>\n"));
	const FString Foreign = Write(TEXT("settings.xml"),
		TEXT("<?xml version=\"1.0\"?>\n<configuration><setting name=\"mujoco\"/></configuration>\n"));
	const FString Empty = Write(TEXT("empty.xml"), TEXT(""));
	const FString WrongExtension = Write(TEXT("model.txt"), kFactoryProbeMjcf);

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
	const FString Dir = FactoryScratchDir(TEXT("Outcome"));

	const FString GoodXml = Dir / TEXT("import_good.xml");
	FFileHelper::SaveStringToFile(FString(kFactoryProbeMjcf), *GoodXml);

	// Well-formed XML that the reader rejects: the preparation step is happy
	// with it, so the failure lands after the Blueprint has been made, which is
	// the case that used to leave an empty articulation behind.
	const FString BadXml = Dir / TEXT("import_bad.xml");
	FFileHelper::SaveStringToFile(FString(
		TEXT("<mujoco model=\"import_bad\">\n")
		TEXT("  <worldbody><notathing/></worldbody>\n")
		TEXT("</mujoco>\n")), *BadXml);

	{
		FFactoryImportProbe Probe;
		Probe.Run(GoodXml);
		TestNotNull(TEXT("a model that reads produces an asset"), Probe.Result);
		TestFalse(TEXT("a successful import is not a cancellation"), Probe.bCancelled);
		TestNotNull(TEXT("the asset is in its package"), Probe.Leftover());
		TestTrue(TEXT("the asset is registered"), Probe.RegisteredAsAsset());
	}

	AddExpectedErrorPlain(TEXT("notathing"), EAutomationExpectedErrorFlags::Contains, 0);
	AddExpectedErrorPlain(TEXT("Failed to read MJCF"), EAutomationExpectedErrorFlags::Contains, 0);
	AddExpectedErrorPlain(TEXT("produced no spec"), EAutomationExpectedErrorFlags::Contains, 0);

	// The output log is a firehose during an import, so the diagnostic is also
	// routed to the editor's Messages panel. Read it back from the listing
	// rather than trusting that the call was made: the panel is where a user
	// who missed the toast goes looking, and a message that never reaches the
	// listing is invisible in exactly the case it exists for.
	FMessageLogModule& MessageLogModule = FModuleManager::LoadModuleChecked<FMessageLogModule>(TEXT("MessageLog"));

	// The listing has to be registered, not merely written to, or the panel
	// shows it with no name -- indistinguishable from anything else that
	// never bothered to register. The module's own query answers this without
	// touching the (private) registry it is backed by.
	TestTrue(TEXT("the \"URLab\" listing is registered with the message log module"),
		MessageLogModule.IsRegisteredLogListing(TEXT("URLab")));

	const TSharedRef<IMessageLogListing> Listing = MessageLogModule.GetLogListing(TEXT("URLab"));
	Listing->ClearMessages();

	{
		FFactoryImportProbe Probe;
		Probe.Run(BadXml);
		TestNull(TEXT("a model that does not read produces no asset"), Probe.Result);
		TestFalse(TEXT("a failure is not reported as a cancellation"), Probe.bCancelled);
		TestNull(TEXT("nothing is left in the package"), Probe.Leftover());
		TestFalse(TEXT("nothing is left in the asset registry"), Probe.RegisteredAsAsset());
	}

	const FString Listed = Listing->GetAllMessagesAsString();
	TestTrue(TEXT("the failure reached the editor's message log"),
		Listed.Contains(TEXT("Failed to read MJCF")) || Listed.Contains(TEXT("produced no spec")));

	// The generation action's own dead end -- an articulation Blueprint with
	// no XML path recorded -- used to be log-only. It now reaches the same
	// listing, checked the same way: read back, not trusted from the call.
	{
		UBlueprint* Blueprint = MakeFactoryScratchBlueprint();
		if (Blueprint == nullptr)
		{
			AddError(TEXT("could not create a scratch Blueprint"));
			return false;
		}
		TestTrue(TEXT("a fresh scratch Blueprint has no recorded XML path"),
			FactoryRecordedSource(Blueprint).IsEmpty());

		AddExpectedErrorPlain(TEXT("No XML File Path set"), EAutomationExpectedErrorFlags::Contains, 0);

		Listing->ClearMessages();
		UMujocoGenerationAction* Generator = NewObject<UMujocoGenerationAction>();
		TestFalse(TEXT("regenerating a Blueprint with no XML path does nothing"),
			Generator->GenerateForSelectedBlueprint(Blueprint));

		const FString GenerationListed = Listing->GetAllMessagesAsString();
		TestTrue(TEXT("the missing-path error reached the editor's message log"),
			GenerationListed.Contains(TEXT("No XML File Path set")));
	}

	return true;
}

// ============================================================================
// URLab.Import.ReimportMatchesImport
//   Reimport used to have no handler at all, so the only way back to a changed
//   model went round the preparation step. It now runs the identical pipeline,
//   from the original path the Blueprint recorded rather than from a prepared
//   copy, and lands on the same spec.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjImportReimportMatchesImport,
	"URLab.Import.ReimportMatchesImport",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjImportReimportMatchesImport::RunTest(const FString& Parameters)
{
	const FString Dir = FactoryScratchDir(TEXT("Reimport"));
	const FString SourceXml = Dir / TEXT("reimport_probe.xml");
	FFileHelper::SaveStringToFile(FString(
		TEXT("<mujoco model=\"reimport_probe\">\n")
		TEXT("  <worldbody>\n")
		TEXT("    <body name=\"root\" pos=\"0 0 0.5\">\n")
		TEXT("      <joint name=\"hinge\" type=\"hinge\" axis=\"0 1 0\" range=\"-1 1\"/>\n")
		TEXT("      <geom name=\"link\" type=\"capsule\" size=\"0.05 0.2\" rgba=\"0.2 0.4 0.8 1\"/>\n")
		TEXT("    </body>\n")
		TEXT("  </worldbody>\n")
		TEXT("  <actuator><motor name=\"drive\" joint=\"hinge\" gear=\"25\"/></actuator>\n")
		TEXT("</mujoco>\n")), *SourceXml);

	FFactoryImportProbe Probe;
	Probe.Run(SourceXml);
	UBlueprint* Blueprint = Probe.AsBlueprint();
	if (Blueprint == nullptr)
	{
		AddError(TEXT("import setup failed: no Blueprint produced"));
		return false;
	}

	// The recorded source is the model, not the prepared copy that was parsed.
	TestEqual(TEXT("the Blueprint records the original path"), FactoryRecordedSource(Blueprint), SourceXml);

	const TArray<FString> NamesBefore = FactoryComponentNames(Blueprint);
	const FString MjcfBefore = FactorySpecMjcf(Blueprint);
	TestTrue(TEXT("the import produced components"), NamesBefore.Num() > 0);
	TestTrue(TEXT("the import produced a spec"), !MjcfBefore.IsEmpty());

	UMujocoImportFactory* Factory = NewObject<UMujocoImportFactory>();

	TArray<FString> Filenames;
	TestTrue(TEXT("an imported articulation can be reimported"),
		Factory->CanReimport(Blueprint, Filenames));
	if (Filenames.Num() == 1)
	{
		TestEqual(TEXT("reimport reads the original path"), Filenames[0], SourceXml);
	}

	TestEqual(TEXT("reimport succeeds"),
		static_cast<int32>(Factory->Reimport(Blueprint)),
		static_cast<int32>(EReimportResult::Succeeded));

	TestEqual(TEXT("reimport lands on the same components"),
		FString::Join(FactoryComponentNames(Blueprint), TEXT(", ")),
		FString::Join(NamesBefore, TEXT(", ")));
	TestEqual(TEXT("reimport lands on the same authored values"), FactorySpecMjcf(Blueprint), MjcfBefore);
	TestEqual(TEXT("reimport leaves the recorded source alone"),
		FactoryRecordedSource(Blueprint), SourceXml);

	return true;
}

// ============================================================================
// URLab.Import.CancelAndFailureLeaveNothing
//   An import that stops before it finishes must leave the project as it found
//   it, and must say which of the two happened: a cancellation is reported as
//   one, a failure is not. Nothing is created before the model is known to be
//   readable, which is what makes "leaves nothing" structural rather than a
//   clean-up step that can be forgotten.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjImportCancelAndFailureLeaveNothing,
	"URLab.Import.CancelAndFailureLeaveNothing",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjImportCancelAndFailureLeaveNothing::RunTest(const FString& Parameters)
{
	const FString Dir = FactoryScratchDir(TEXT("Cancel"));

	// A model whose mesh is not on disk: preparation runs, fails, and the
	// import stops before anything is made.
	const FString UnpreparableXml = Dir / TEXT("unpreparable.xml");
	FFileHelper::SaveStringToFile(FString(
		TEXT("<mujoco model=\"unpreparable\">\n")
		TEXT("  <asset><mesh name=\"absent\" file=\"absent.stl\"/></asset>\n")
		TEXT("  <worldbody><body name=\"b\"><geom name=\"g\" type=\"mesh\" mesh=\"absent\"/></body></worldbody>\n")
		TEXT("</mujoco>\n")), *UnpreparableXml);

	AddExpectedErrorPlain(TEXT("mesh preparation of"), EAutomationExpectedErrorFlags::Contains, 0);
	AddExpectedErrorPlain(TEXT("refusing to overwrite existing Blueprint"),
		EAutomationExpectedErrorFlags::Contains, 0);

	{
		int32 Acquisitions = 0;
		UBlueprint* Produced = nullptr;
		FString Error;
		bool bCancelled = false;
		FMjImportSettings Settings;
		Settings.bAllowPrompts = false;
		const bool bOk = UMujocoImportFactory::ImportModel(UnpreparableXml, Settings,
			[&Acquisitions]() -> UBlueprint* {
				++Acquisitions;
				return nullptr;
			},
			Produced, Error, bCancelled);

		TestFalse(TEXT("an unpreparable model does not import"), bOk);
		TestEqual(TEXT("nothing is asked for before the model is prepared"), Acquisitions, 0);
		TestNull(TEXT("no Blueprint comes back"), Produced);
		TestTrue(TEXT("the failure carries a reason"), !Error.IsEmpty());
	}

	{
		FFactoryImportProbe Probe;
		Probe.Run(UnpreparableXml);
		TestNull(TEXT("the factory imports nothing"), Probe.Result);
		TestFalse(TEXT("a failure is not reported as a cancellation"), Probe.bCancelled);
		TestNull(TEXT("nothing is left in the package"), Probe.Leftover());
		TestFalse(TEXT("nothing is left in the asset registry"), Probe.RegisteredAsAsset());
	}

	// Refusing to overwrite a Blueprint that is still in memory is the other
	// way an import stops early. It reports itself as a cancellation, and the
	// asset it declined to touch is the one that was already there.
	{
		const FString GoodXml = Dir / TEXT("cancel_probe.xml");
		FFileHelper::SaveStringToFile(FString(kFactoryProbeMjcf), *GoodXml);

		FFactoryImportProbe First;
		First.Run(GoodXml);
		if (First.AsBlueprint() == nullptr)
		{
			AddError(TEXT("import setup failed: no Blueprint produced"));
			return false;
		}
		const TArray<FString> NamesBefore = FactoryComponentNames(First.AsBlueprint());

		UMujocoImportFactory* Factory = NewObject<UMujocoImportFactory>();
		bool bCancelled = false;
		UObject* Second = Factory->FactoryCreateFile(UBlueprint::StaticClass(), First.Package,
			FName(*First.AssetName), RF_Public | RF_Standalone, GoodXml, nullptr, GWarn, bCancelled);

		TestNull(TEXT("the second import produces nothing"), Second);
		TestTrue(TEXT("stopping early is reported as a cancellation"), bCancelled);
		TestEqual(TEXT("the Blueprint that was already there is untouched"),
			FactoryComponentNames(First.AsBlueprint()), NamesBefore);
	}

	return true;
}

// ============================================================================
// URLab.Import.ExternalIncludeOptionReachesTheReader
//   The external-include boundary had no way to be reached: the reader refuses
//   an <include> that escapes the model's directory tree, and nothing in the
//   editor could say otherwise. The import dialog now carries the option, so
//   what matters here is that the option a caller sets is the one the reader
//   honours, in both positions.
//
//   Both halves go through APIs that report rather than log, so the test says
//   what happened without the run having to expect error output.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjImportExternalIncludeOption,
	"URLab.Import.ExternalIncludeOptionReachesTheReader",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjImportExternalIncludeOption::RunTest(const FString& Parameters)
{
	const FString Root = FactoryScratchDir(TEXT("Includes"));
	const FString ModelDir = Root / TEXT("model");
	const FString OutsideDir = Root / TEXT("outside");
	IFileManager::Get().MakeDirectory(*ModelDir, /*Tree=*/true);
	IFileManager::Get().MakeDirectory(*OutsideDir, /*Tree=*/true);

	FFileHelper::SaveStringToFile(FString(
		TEXT("<mujocoinclude>\n")
		TEXT("  <worldbody>\n")
		TEXT("    <body name=\"from_outside\"><geom name=\"og\" type=\"sphere\" size=\"0.1\"/></body>\n")
		TEXT("  </worldbody>\n")
		TEXT("</mujocoinclude>\n")), *(OutsideDir / TEXT("fragment.xml")));

	const FString ModelPath = ModelDir / TEXT("host.xml");
	const FString ModelXml =
		TEXT("<mujoco model=\"host\">\n")
		TEXT("  <include file=\"../outside/fragment.xml\"/>\n")
		TEXT("  <worldbody>\n")
		TEXT("    <body name=\"host_body\"><geom name=\"hg\" type=\"sphere\" size=\"0.1\"/></body>\n")
		TEXT("  </worldbody>\n")
		TEXT("</mujoco>\n");
	FFileHelper::SaveStringToFile(ModelXml, *ModelPath);

	// Off, the default: the escaping include does not reach the spec.
	{
		UBlueprint* Blueprint = MakeFactoryScratchBlueprint();
		if (Blueprint == nullptr)
		{
			AddError(TEXT("could not create a scratch Blueprint"));
			return false;
		}
		FMjDocParseOptions Options;
		TestFalse(TEXT("the option is off by default"), Options.bAllowExternalIncludes);

		MjParseIntoBlueprint(*Blueprint, ModelXml, ModelPath, Options);
		TestFalse(TEXT("an escaping include is refused by default"),
			FactoryHasElementNamed(Blueprint, TEXT("from_outside")));
	}

	// On, and carried all the way through the generation action the importer
	// uses: the same include is read.
	{
		UBlueprint* Blueprint = MakeFactoryScratchBlueprint();
		if (Blueprint == nullptr)
		{
			AddError(TEXT("could not create a scratch Blueprint"));
			return false;
		}
		FMjDocParseOptions Options;
		Options.bAllowExternalIncludes = true;

		UMujocoGenerationAction* Generator = NewObject<UMujocoGenerationAction>();
		TestTrue(TEXT("the model reads when escaping includes are allowed"),
			Generator->GenerateFromXml(Blueprint, ModelXml, ModelPath, Options));
		TestTrue(TEXT("the option reaches the reader through the generation action"),
			FactoryHasElementNamed(Blueprint, TEXT("from_outside")));
		TestTrue(TEXT("the model's own content is still there"),
			FactoryHasElementNamed(Blueprint, TEXT("host_body")));
	}

	return true;
}
