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
#include "ObjectTools.h"
#include "RenderingThread.h"
#include "ShaderCompiler.h"
#include "URLabEditorLogging.h"

namespace
{
/**
 * Prepare `SourceXmlPath`'s meshes and report the document to parse.
 *
 * Preparation that cannot run at all -- no script, or a user who declined the
 * Python setup -- leaves the original in `OutXmlPath` and succeeds, because
 * that is a stated choice. Preparation that RAN and failed returns false.
 */
bool PrepareMeshes(const FString& SourceXmlPath, FString& OutXmlPath, FString& OutError, bool& bOutCancelled)
{
	OutXmlPath = SourceXmlPath;
	OutError.Reset();
	bOutCancelled = false;

	const TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("UnrealRoboticsLab"));
	if (!Plugin.IsValid())
	{
		OutError = TEXT("the UnrealRoboticsLab plugin directory could not be located");
		return false;
	}

	const FString ScriptPath = FPaths::Combine(Plugin->GetBaseDir(), TEXT("Scripts/clean_meshes.py"));
	if (!FPaths::FileExists(ScriptPath))
	{
		UE_LOG(LogURLabEditor, Warning,
			TEXT("Mesh preparation script missing at '%s' -- importing the model as authored."), *ScriptPath);
		return true;
	}

	const FString PythonExe = FMjPythonHelper::EnsurePythonReady(bOutCancelled);
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

	return UMujocoImportFactory::RunMeshPreparation(PythonExe, ScriptPath, SourceXmlPath, OutXmlPath, OutError);
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
	const FString& SourceXmlPath, FString& OutXmlPath, FString& OutError)
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
	const FString Args = FString::Printf(TEXT("\"%s\" \"%s\" --out-dir \"%s\""),
		*ScriptPath, *SourceXmlPath, *PrepDir);
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
		bOutOperationCanceled = true;
		return nullptr;
	}

	FScopedSlowTask SlowTask(4.f, NSLOCTEXT("URLab", "ImportingMuJoCo", "Importing MuJoCo model..."));
	SlowTask.MakeDialog(/*bShowCancelButton=*/false);

	// Preparation first, because it needs nothing from the project: a model
	// whose meshes cannot be prepared never gets as far as making an asset.
	SlowTask.EnterProgressFrame(1.f, NSLOCTEXT("URLab", "ImportStep0", "Preparing meshes..."));

	FString ActualXmlPath;
	FString PrepareError;
	if (!PrepareMeshes(Filename, ActualXmlPath, PrepareError, bOutOperationCanceled))
	{
		if (bOutOperationCanceled)
		{
			UE_LOG(LogURLabEditor, Log, TEXT("Import cancelled by user during Python setup."));
		}
		else
		{
			UE_LOG(LogURLabEditor, Error, TEXT("MujocoImportFactory: %s"), *PrepareError);
		}
		return nullptr;
	}

	SlowTask.EnterProgressFrame(1.f, NSLOCTEXT("URLab", "ImportStep1", "Reading XML..."));

	UBlueprint* NewBP = FKismetEditorUtilities::CreateBlueprint(
		AMjArticulation::StaticClass(),
		InParent,
		InName,
		BPTYPE_Normal,
		UBlueprint::StaticClass(),
		UBlueprintGeneratedClass::StaticClass());
	if (NewBP == nullptr)
	{
		UE_LOG(LogURLabEditor, Error,
			TEXT("MujocoImportFactory: could not create a Blueprint named '%s'."), *InName.ToString());
		return nullptr;
	}

	// The ORIGINAL path, never the prepared copy: it is what the user chose,
	// and it is what a reimport has to prepare again from.
	if (AMjArticulation* CDO = Cast<AMjArticulation>(NewBP->GeneratedClass->GetDefaultObject()))
	{
		CDO->MuJoCoXMLFile.FilePath = Filename;
		CDO->MarkPackageDirty();
	}

	SlowTask.EnterProgressFrame(1.f, NSLOCTEXT("URLab", "ImportStep2", "Building Blueprint components..."));

	// Reads the prepared XML into the Blueprint's construction script, imports
	// the assets it references, and compiles.
	UMujocoGenerationAction* Generator = NewObject<UMujocoGenerationAction>();
	const bool bGenerated = Generator->GenerateForBlueprint(NewBP, ActualXmlPath);

	SlowTask.EnterProgressFrame(1.f, NSLOCTEXT("URLab", "ImportStep3", "Finalizing assets..."));

	if (!bGenerated)
	{
		// An empty articulation left in the content browser reads as a model
		// that imported, and the next thing that happens to it is a user
		// dropping it into a level and wondering where the robot went.
		UE_LOG(LogURLabEditor, Error,
			TEXT("MujocoImportFactory: '%s' produced no spec; nothing was imported."), *ActualXmlPath);
		DiscardImportedBlueprint(NewBP);
		return nullptr;
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

	return NewBP;
}
