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
#include "MjPythonHelper.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCoSpec/MJSpecHandle.h"
#include "MuJoCoSpec/MJXmlImporterExporter.h"
#include "HAL/FileManager.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "Engine/Blueprint.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/ScopedSlowTask.h"
#include "Logging/MessageLog.h"
#include "Logging/TokenizedMessage.h"
#include "ObjectTools.h"
#include "RenderingThread.h"
#include "ShaderCompiler.h"
#include "URLabEditorLogging.h"

namespace
{
void ReportImportFailure(const FText& Summary)
{
	FMessageLog Log(TEXT("URLab"));
	Log.Error(Summary);
	Log.Notify(Summary, EMessageSeverity::Error, /*bForce=*/true);
}

void DiscardImportedBlueprint(UBlueprint* Blueprint)
{
	if (Blueprint != nullptr)
	{
		ObjectTools::DeleteObjectsUnchecked({Blueprint});
	}
}

bool LooksLikeMjcf(const FString& Filename)
{
	FString Content;
	if (!FFileHelper::LoadFileToString(Content, *Filename))
	{
		return false;
	}
	return Content.Contains(TEXT("<mujoco"), ESearchCase::IgnoreCase);
}

void PopulateVfsFromDirectory(const FString& BaseDir, mjVFS* Vfs)
{
	if (!Vfs || BaseDir.IsEmpty() || !FPaths::DirectoryExists(BaseDir))
	{
		return;
	}
	TArray<FString> Files;
	IFileManager::Get().FindFilesRecursive(Files, *BaseDir, TEXT("*.*"), true, false);
	for (const FString& AbsFile : Files)
	{
		FString RelPath = AbsFile;
		if (!FPaths::MakePathRelativeTo(RelPath, *(BaseDir + TEXT("/"))))
		{
			RelPath = FPaths::GetCleanFilename(AbsFile);
		}
		TArray<uint8> Bytes;
		if (FFileHelper::LoadFileToArray(Bytes, *AbsFile))
		{
			mj_addBufferVFS(Vfs, TCHAR_TO_UTF8(*RelPath), Bytes.GetData(), Bytes.Num());
		}
	}
}
} // namespace

UMujocoImportFactory::UMujocoImportFactory()
{
	bCreateNew = false;
	bEditorImport = true;
	SupportedClass = UBlueprint::StaticClass();
	Formats.Add(TEXT("xml;MuJoCo MJCF Model"));
	ImportPriority = DefaultImportPriority + 1;
}

FString UMujocoImportFactory::ImportPrepDir(const FString& SourceXmlPath)
{
	const FString Stem = ObjectTools::SanitizeObjectName(FPaths::GetBaseFilename(SourceXmlPath));
	return FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("URLab/ImportPrep"), Stem);
}

bool UMujocoImportFactory::RunMeshPreparation(const FString&, const FString&, const FString& SourceXmlPath,
	bool, FString& OutXmlPath, FString& OutError)
{
	OutError.Reset();
	OutXmlPath = SourceXmlPath;
	return true;
}

bool UMujocoImportFactory::ImportModel(const FString& SourceXmlPath, const FMjImportSettings&,
	TFunctionRef<UBlueprint*()> AcquireBlueprint, UBlueprint*& OutBlueprint,
	FString& OutError, bool& bOutCancelled)
{
	bOutCancelled = false;
	OutBlueprint = nullptr;
	OutError.Reset();

	FString Xml;
	if (!FFileHelper::LoadFileToString(Xml, *SourceXmlPath))
	{
		OutError = FString::Printf(TEXT("Failed to read MJCF file '%s'"), *SourceXmlPath);
		return false;
	}

	FMJVFSHandle Vfs;
	Vfs.Init();
	PopulateVfsFromDirectory(FPaths::GetPath(SourceXmlPath), Vfs.Get());

	OutBlueprint = AcquireBlueprint();
	if (!OutBlueprint)
	{
		OutError = TEXT("Failed to create target Blueprint");
		return false;
	}

	if (!FMJXmlImporterExporter::ImportXMLToBlueprintSCS(Xml, OutBlueprint, Vfs.Get(), OutError))
	{
		return false;
	}

	FKismetEditorUtilities::CompileBlueprint(OutBlueprint);

	if (GShaderCompilingManager)
	{
		GShaderCompilingManager->FinishAllCompilation();
	}
	FlushRenderingCommands();
	return true;
}

bool UMujocoImportFactory::CanReimport(UObject*, TArray<FString>& OutFilenames)
{
	OutFilenames.Reset();
	return false;
}

void UMujocoImportFactory::SetReimportPaths(UObject*, const TArray<FString>&)
{
}

EReimportResult::Type UMujocoImportFactory::Reimport(UObject*)
{
	return EReimportResult::Failed;
}

int32 UMujocoImportFactory::GetPriority() const
{
	return ImportPriority;
}

bool UMujocoImportFactory::FactoryCanImport(const FString& Filename)
{
	return FPaths::GetExtension(Filename).Equals(TEXT("xml"), ESearchCase::IgnoreCase)
		&& LooksLikeMjcf(Filename);
}

UObject* UMujocoImportFactory::FactoryCreateFile(UClass*, UObject* InParent, FName InName, EObjectFlags, const FString& Filename, const TCHAR*, FFeedbackContext*, bool& bOutOperationCanceled)
{
	bOutOperationCanceled = false;

	if (FindObject<UBlueprint>(InParent, *InName.ToString()) != nullptr)
	{
		UE_LOG(LogURLabEditor, Error,
			TEXT("MujocoImportFactory: refusing to overwrite existing Blueprint '%s' in '%s'."),
			*InName.ToString(),
			InParent ? *InParent->GetPathName() : TEXT("<no outer>"));
		ReportImportFailure(FText::Format(
			NSLOCTEXT("URLab", "ImportWouldOverwrite",
				"MuJoCo import: refusing to overwrite existing Blueprint '{0}'. Delete it, or reimport it, before importing the model again."),
			FText::FromName(InName)));
		bOutOperationCanceled = true;
		return nullptr;
	}

	FMjImportSettings Settings;
	Settings.bAllowPrompts = !IsAutomatedImport();
	if (Settings.bAllowPrompts
		&& !ShowMjImportOptionsDialog(Filename, FText::GetEmpty(), Settings.Parse))
	{
		bOutOperationCanceled = true;
		return nullptr;
	}

	FScopedSlowTask SlowTask(2.f, NSLOCTEXT("URLab", "ImportingMuJoCo", "Importing MuJoCo model..."));
	SlowTask.MakeDialog(/*bShowCancelButton=*/false);
	SlowTask.EnterProgressFrame(1.f, NSLOCTEXT("URLab", "ImportStep0", "Reading MuJoCo model..."));

	UBlueprint* NewBP = nullptr;
	FString Error;
	const bool bImported = ImportModel(Filename, Settings, [&]() -> UBlueprint* {
		SlowTask.EnterProgressFrame(1.f,
			NSLOCTEXT("URLab", "ImportStep1", "Building Blueprint components..."));
		return FKismetEditorUtilities::CreateBlueprint(
			AMjArticulation::StaticClass(),
			InParent,
			InName,
			BPTYPE_Normal,
			UBlueprint::StaticClass(),
			UBlueprintGeneratedClass::StaticClass());
	}, NewBP, Error, bOutOperationCanceled);

	if (!bImported)
	{
		if (!bOutOperationCanceled)
		{
			UE_LOG(LogURLabEditor, Error, TEXT("MujocoImportFactory: %s Nothing was imported."), *Error);
			ReportImportFailure(FText::Format(
				NSLOCTEXT("URLab", "ImportFailedDetail", "MuJoCo import failed: {0} Nothing was imported."),
				FText::FromString(Error)));
		}
		DiscardImportedBlueprint(NewBP);
		return nullptr;
	}

	return NewBP;
}
