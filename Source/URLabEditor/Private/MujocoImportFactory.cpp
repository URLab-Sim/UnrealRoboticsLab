// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
// trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

#include "MujocoImportFactory.h"
#include "MjImportOptionsDialog.h"
#include "MujocoGenerationAction.h"
#include "MjPythonHelper.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "HAL/FileManager.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "Engine/Blueprint.h"
#include "Misc/FeedbackContext.h"
#include "Misc/Paths.h"
#include "Misc/ScopedSlowTask.h"
#include "Interfaces/IPluginManager.h"
#include "Logging/MessageLog.h"
#include "Logging/TokenizedMessage.h"
#include "ObjectTools.h"
#include "RenderingThread.h"
#include "ShaderCompiler.h"
#include "URLabEditorLogging.h"

namespace
{

/**
 * Say it where a failed import is actually looked for.
 *
 * The diagnostic used to go to the output log alone, which during an import is
 * a firehose: the visible outcome of a model that failed to read was an asset
 * that did not appear, with no statement of why. This puts the same text in the
 * editor's Messages panel and raises it. The log line stays -- it is what a bug
 * report is pasted from -- so this adds a place rather than moving one.
 */
void ReportImportFailure(const FText& Message)
{
	FMessageLog MessageLog(TEXT("URLab"));
	MessageLog.Error(Message);
	// Forced, because the default severity filter would let a page whose only
	// entries are the ones just written go unshown.
	MessageLog.Notify(NSLOCTEXT("URLab", "ImportFailedToast", "MuJoCo import failed"),
		EMessageSeverity::Error, /*bForce=*/true);
}

/** The mesh preparation script that ships with the plugin, if it is there. */
FString PreparationScriptPath()
{
	const TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("UnrealRoboticsLab"));
	if (!Plugin.IsValid())
	{
		return FString();
	}
	const FString ScriptPath = FPaths::Combine(Plugin->GetBaseDir(), TEXT("Scripts/clean_meshes.py"));
	return FPaths::FileExists(ScriptPath) ? ScriptPath : FString();
}

/**
 * Prepare `SourceXmlPath`'s meshes and report the document to parse.
 *
 * Preparation that cannot run at all -- no script, or a user who declined the
 * Python setup -- leaves the original in `OutXmlPath` and succeeds, because
 * that is a stated choice. Preparation that RAN and failed returns false.
 *
 * `bAllowExternalIncludes` is the reader's own option, passed on because
 * preparation flattens the model's includes before the reader ever sees it: the
 * gate has to be answered wherever the includes are actually followed.
 */
bool PrepareMeshes(const FString& SourceXmlPath, bool bAllowPrompts, bool bAllowExternalIncludes,
	FString& OutXmlPath, FString& OutError, bool& bOutCancelled)
{
	OutXmlPath = SourceXmlPath;
	OutError.Reset();
	bOutCancelled = false;

	const FString ScriptPath = PreparationScriptPath();
	if (ScriptPath.IsEmpty())
	{
		UE_LOG(LogURLabEditor, Warning,
			TEXT("Mesh preparation script not found -- importing the model as authored."));
		return true;
	}

	const FString PythonExe = FMjPythonHelper::EnsurePythonReady(bAllowPrompts, bOutCancelled);
	if (bOutCancelled)
	{
		return false;
	}
	if (PythonExe.IsEmpty())
	{
		UE_LOG(LogURLabEditor, Warning,
			TEXT("Python is not configured -- importing the model as authored, without mesh preparation."));
		return true;
	}

	return UMujocoImportFactory::RunMeshPreparation(
		PythonExe, ScriptPath, SourceXmlPath, bAllowExternalIncludes, OutXmlPath, OutError);
}

/** What the import dialog says about mesh preparation for this run. */
FText PreparationStatusText()
{
	if (PreparationScriptPath().IsEmpty())
	{
		return NSLOCTEXT("URLab", "PrepMissing",
			"Mesh preparation is unavailable: the preparation script is missing. "
			"The model will be imported exactly as authored.");
	}
	if (!FMjPythonHelper::IsPythonReady())
	{
		return NSLOCTEXT("URLab", "PrepNeedsSetup",
			"Mesh preparation needs Python with trimesh, numpy, scipy, networkx and Pillow. "
			"You will be asked to set that up before the model is read.");
	}
	return NSLOCTEXT("URLab", "PrepWillRun",
		"Meshes will be converted for Unreal before the model is read.");
}

/**
 * Take a half-built import back out of the project.
 *
 * A Blueprint cannot be built somewhere private and moved in once it works:
 * its generated class is outered to the package rather than to the Blueprint,
 * so there is no one rename that carries the asset. Removing what a failed
 * import made is the same thing seen from the other side, and the editor's own
 * deletion path is what does it.
 */
void DiscardImportedBlueprint(UBlueprint* Blueprint)
{
	if (Blueprint == nullptr)
	{
		return;
	}
	const FString Name = Blueprint->GetPathName();
	TArray<UObject*> ToDelete = {Blueprint};
	if (ObjectTools::ForceDeleteObjects(ToDelete, /*ShowConfirmation=*/false) == 0)
	{
		UE_LOG(LogURLabEditor, Warning,
			TEXT("MujocoImportFactory: could not remove '%s' after a failed import."), *Name);
	}
}

/**
 * True when the head of `Filename` carries an MJCF root tag.
 *
 * The root tag is the document's first element, so a few kilobytes is more
 * head than any real file needs; only a declaration, comments and processing
 * instructions may precede it. `<mujocoinclude>` fragments answer yes too,
 * because the reader accepts them.
 */
bool LooksLikeMjcf(const FString& Filename)
{
	const TUniquePtr<FArchive> Reader(IFileManager::Get().CreateFileReader(*Filename));
	if (!Reader.IsValid())
	{
		return false;
	}

	const int64 HeadBytes = FMath::Min<int64>(Reader->TotalSize(), 8192);
	TArray<uint8> Head;
	Head.SetNumZeroed(static_cast<int32>(HeadBytes) + 1);
	if (HeadBytes > 0)
	{
		Reader->Serialize(Head.GetData(), HeadBytes);
	}

	const FString Text(UTF8_TO_TCHAR(reinterpret_cast<const ANSICHAR*>(Head.GetData())));
	return Text.Contains(TEXT("<mujoco"), ESearchCase::IgnoreCase);
}
}  // namespace

FString UMujocoImportFactory::ImportPrepDir(const FString& SourceXmlPath)
{
	return FPaths::ConvertRelativePathToFull(
		FPaths::ProjectSavedDir() / TEXT("URLab/ImportPrep") / FPaths::GetBaseFilename(SourceXmlPath));
}

bool UMujocoImportFactory::RunMeshPreparation(const FString& PythonExe, const FString& ScriptPath,
	const FString& SourceXmlPath, bool bAllowExternalIncludes, FString& OutXmlPath, FString& OutError)
{
	OutXmlPath = SourceXmlPath;
	OutError.Reset();

	const FString PrepDir = ImportPrepDir(SourceXmlPath);
	const FString PreparedPath = FPaths::Combine(PrepDir,
		FPaths::GetBaseFilename(SourceXmlPath) + TEXT("_ue.xml"));

	// A leftover from an earlier run must never be mistaken for this one's
	// output, so the target is gone before the script is asked to write it.
	IFileManager::Get().Delete(*PreparedPath, /*RequireExists=*/false, /*EvenReadOnly=*/true, /*Quiet=*/true);

	int32 ReturnCode = -1;
	FString StdOut;
	FString StdErr;
	// The flag is passed only when the option is on, so a script that predates
	// it still runs the default import: an unknown argument would fail the
	// import for every model, where an absent one fails closed for the few that
	// reach outside their own folder.
	const FString Args = FString::Printf(TEXT("\"%s\" \"%s\" --out-dir \"%s\"%s"),
		*ScriptPath, *SourceXmlPath, *PrepDir,
		bAllowExternalIncludes ? TEXT(" --allow-external-includes") : TEXT(""));
	UE_LOG(LogURLabEditor, Log, TEXT("Running mesh preparation: %s %s"), *PythonExe, *Args);
	FPlatformProcess::ExecProcess(*PythonExe, *Args, &ReturnCode, &StdOut, &StdErr);

	if (ReturnCode != 0)
	{
		OutError = FString::Printf(
			TEXT("mesh preparation of '%s' failed (exit code %d).%s%s"),
			*SourceXmlPath, ReturnCode,
			StdErr.IsEmpty() ? TEXT("") : TEXT("\n"),
			StdErr.IsEmpty() ? TEXT("") : *StdErr);
		if (!StdOut.IsEmpty())
		{
			UE_LOG(LogURLabEditor, Log, TEXT("Mesh preparation output:\n%s"), *StdOut);
		}
		return false;
	}

	if (!FPaths::FileExists(PreparedPath))
	{
		OutError = FString::Printf(
			TEXT("mesh preparation of '%s' reported success but wrote no '%s'."),
			*SourceXmlPath, *PreparedPath);
		return false;
	}

	UE_LOG(LogURLabEditor, Log, TEXT("Using prepared XML: %s"), *PreparedPath);
	OutXmlPath = PreparedPath;
	return true;
}

UMujocoImportFactory::UMujocoImportFactory()
{
	Formats.Add(TEXT("xml;MuJoCo XML File"));
	SupportedClass = UBlueprint::StaticClass();
	bCreateNew = false;
	bEditorImport = true;
}

bool UMujocoImportFactory::ImportModel(const FString& SourceXmlPath, const FMjImportSettings& Settings,
	TFunctionRef<UBlueprint*()> AcquireBlueprint, UBlueprint*& OutBlueprint,
	FString& OutError, bool& bOutCancelled)
{
	OutBlueprint = nullptr;
	OutError.Reset();
	bOutCancelled = false;

	FString PreparedXmlPath;
	if (!PrepareMeshes(SourceXmlPath, Settings.bAllowPrompts, Settings.Parse.bAllowExternalIncludes,
			PreparedXmlPath, OutError, bOutCancelled))
	{
		return false;
	}

	OutBlueprint = AcquireBlueprint();
	if (OutBlueprint == nullptr || OutBlueprint->GeneratedClass == nullptr)
	{
		OutError = FString::Printf(TEXT("no Blueprint to read '%s' into."), *SourceXmlPath);
		return false;
	}

	// The ORIGINAL path, never the prepared copy: it is what the user chose,
	// and it is what the next reimport has to prepare again from.
	if (AMjArticulation* CDO = Cast<AMjArticulation>(OutBlueprint->GeneratedClass->GetDefaultObject()))
	{
		CDO->MuJoCoXMLFile.FilePath = SourceXmlPath;
		CDO->MarkPackageDirty();
	}

	// Reads the prepared XML into the Blueprint's construction script, imports
	// the assets it references, and compiles.
	UMujocoGenerationAction* Generator = NewObject<UMujocoGenerationAction>();
	if (!Generator->GenerateForBlueprint(OutBlueprint, PreparedXmlPath, Settings.Parse))
	{
		OutError = FString::Printf(TEXT("'%s' produced no spec."), *PreparedXmlPath);
		return false;
	}

	// Wait for all shaders to finish compiling and flush render commands.
	// Material instances created during import trigger async shader compilation.
	// If the content browser renders thumbnails before shaders are ready,
	// the render thread crashes (UE-23902).
	if (GShaderCompilingManager)
	{
		GShaderCompilingManager->FinishAllCompilation();
	}
	FlushRenderingCommands();
	return true;
}

bool UMujocoImportFactory::CanReimport(UObject* Obj, TArray<FString>& OutFilenames)
{
	const UBlueprint* Blueprint = Cast<UBlueprint>(Obj);
	if (Blueprint == nullptr || Blueprint->GeneratedClass == nullptr
		|| !Blueprint->GeneratedClass->IsChildOf(AMjArticulation::StaticClass()))
	{
		return false;
	}

	const AMjArticulation* CDO = Cast<AMjArticulation>(Blueprint->GeneratedClass->GetDefaultObject());
	if (CDO == nullptr || CDO->MuJoCoXMLFile.FilePath.IsEmpty())
	{
		return false;
	}

	OutFilenames.Add(CDO->MuJoCoXMLFile.FilePath);
	return true;
}

void UMujocoImportFactory::SetReimportPaths(UObject* Obj, const TArray<FString>& NewReimportPaths)
{
	UBlueprint* Blueprint = Cast<UBlueprint>(Obj);
	if (NewReimportPaths.Num() == 0 || Blueprint == nullptr || Blueprint->GeneratedClass == nullptr)
	{
		return;
	}
	if (AMjArticulation* CDO = Cast<AMjArticulation>(Blueprint->GeneratedClass->GetDefaultObject()))
	{
		CDO->MuJoCoXMLFile.FilePath = NewReimportPaths[0];
		CDO->MarkPackageDirty();
	}
}

EReimportResult::Type UMujocoImportFactory::Reimport(UObject* Obj)
{
	TArray<FString> Filenames;
	if (!CanReimport(Obj, Filenames))
	{
		return EReimportResult::Failed;
	}

	// No dialog on the way back in: reimport-all runs in bulk and the security
	// option returns to its default, which is the safe direction to fail in.
	FMjImportSettings Settings;
	Settings.bAllowPrompts = false;

	UBlueprint* const Existing = Cast<UBlueprint>(Obj);
	UBlueprint* Read = nullptr;
	FString Error;
	bool bCancelled = false;
	const bool bOk = ImportModel(Filenames[0], Settings, [Existing]() { return Existing; },
		Read, Error, bCancelled);

	if (bCancelled)
	{
		UE_LOG(LogURLabEditor, Log, TEXT("Reimport cancelled by user during Python setup."));
		return EReimportResult::Cancelled;
	}
	if (!bOk)
	{
		UE_LOG(LogURLabEditor, Error, TEXT("MujocoImportFactory: %s"), *Error);
		ReportImportFailure(FText::Format(
			NSLOCTEXT("URLab", "ReimportFailedDetail", "MuJoCo reimport failed: {0}"),
			FText::FromString(Error)));
		return EReimportResult::Failed;
	}
	return EReimportResult::Succeeded;
}

int32 UMujocoImportFactory::GetPriority() const
{
	return ImportPriority;
}

bool UMujocoImportFactory::FactoryCanImport(const FString& Filename)
{
	// `.xml` belongs to everybody. Claiming every one of them offers a MuJoCo
	// import for settings files, UI layouts and build manifests, so the
	// extension only gets us as far as reading the root tag.
	return FPaths::GetExtension(Filename).Equals(TEXT("xml"), ESearchCase::IgnoreCase)
		&& LooksLikeMjcf(Filename);
}

UObject* UMujocoImportFactory::FactoryCreateFile(UClass* InClass, UObject* InParent, FName InName, EObjectFlags Flags, const FString& Filename, const TCHAR* Parms, FFeedbackContext* Warn, bool& bOutOperationCanceled)
{
	bOutOperationCanceled = false;

	// Defensive: FKismetEditorUtilities::CreateBlueprint asserts when
	// an existing UBlueprint with the same name is still in memory
	// (Kismet2.cpp:424 -- FindObject<UBlueprint>(Outer, Name) == 0).
	// Callers that want to overwrite should drive ImportXmlSync with
	// force_reimport=true, which destroys the existing asset first; if
	// we get here with a stale BP anyway, bail with a logged error
	// rather than crashing the editor.
	if (FindObject<UBlueprint>(InParent, *InName.ToString()) != nullptr)
	{
		UE_LOG(LogURLabEditor, Error,
			TEXT("MujocoImportFactory: refusing to overwrite existing Blueprint '%s' in '%s'. "
				 "Call ImportXmlSync with force_reimport=true (which destroys the existing asset) "
				 "before importing again."),
			*InName.ToString(),
			InParent ? *InParent->GetPathName() : TEXT("<no outer>"));
		// The wording tracks the log line above deliberately: both say the same
		// thing, and a reader who found one and searched for the other should
		// land on it.
		ReportImportFailure(FText::Format(
			NSLOCTEXT("URLab", "ImportWouldOverwrite",
				"MuJoCo import: refusing to overwrite existing Blueprint '{0}'. Delete it, or reimport "
				"it, before importing the model again."),
			FText::FromName(InName)));
		bOutOperationCanceled = true;
		return nullptr;
	}

	// An automated import has nobody to answer a dialog, so it takes the
	// defaults; that is also what keeps the Python first-run setup silent in a
	// scripted run, where an unanswered prompt would resolve to Cancel.
	FMjImportSettings Settings;
	Settings.bAllowPrompts = !IsAutomatedImport();
	if (Settings.bAllowPrompts
		&& !ShowMjImportOptionsDialog(Filename, PreparationStatusText(), Settings.Parse))
	{
		UE_LOG(LogURLabEditor, Log, TEXT("Import of '%s' cancelled from the options dialog."), *Filename);
		bOutOperationCanceled = true;
		return nullptr;
	}

	FScopedSlowTask SlowTask(2.f, NSLOCTEXT("URLab", "ImportingMuJoCo", "Importing MuJoCo model..."));
	SlowTask.MakeDialog(/*bShowCancelButton=*/false);
	SlowTask.EnterProgressFrame(1.f, NSLOCTEXT("URLab", "ImportStep0", "Preparing meshes..."));

	// The Blueprint is made from inside the pipeline, after preparation has
	// succeeded, so a model whose meshes cannot be prepared never gets as far
	// as making an asset.
	UBlueprint* NewBP = nullptr;
	FString Error;
	const bool bImported = ImportModel(Filename, Settings,
		[&]() -> UBlueprint* {
			SlowTask.EnterProgressFrame(1.f,
				NSLOCTEXT("URLab", "ImportStep1", "Building Blueprint components..."));
			return FKismetEditorUtilities::CreateBlueprint(
				AMjArticulation::StaticClass(),
				InParent,
				InName,
				BPTYPE_Normal,
				UBlueprint::StaticClass(),
				UBlueprintGeneratedClass::StaticClass());
		},
		NewBP, Error, bOutOperationCanceled);

	if (!bImported)
	{
		if (bOutOperationCanceled)
		{
			UE_LOG(LogURLabEditor, Log, TEXT("Import cancelled by user during Python setup."));
		}
		else
		{
			UE_LOG(LogURLabEditor, Error,
				TEXT("MujocoImportFactory: %s Nothing was imported."), *Error);
			ReportImportFailure(FText::Format(
				NSLOCTEXT("URLab", "ImportFailedDetail", "MuJoCo import failed: {0} Nothing was imported."),
				FText::FromString(Error)));
		}
		// An empty articulation left in the content browser reads as a model
		// that imported, and the next thing that happens to it is a user
		// dropping it into a level and wondering where the robot went.
		DiscardImportedBlueprint(NewBP);
		return nullptr;
	}

	return NewBP;
}
