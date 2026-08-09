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

#pragma once

#include "CoreMinimal.h"
#include "EditorReimportHandler.h"
#include "Factories/Factory.h"

#include "MuJoCo/Spec/MjSpecRef.h"

#include "MujocoImportFactory.generated.h"

class UBlueprint;

/** What an import has been told to do, decided before it starts. */
struct FMjImportSettings
{
	/** Parse-time options, the external-include boundary among them. */
	FMjDocParseOptions Parse;

	/**
	 * Whether this import may put a dialog in front of somebody.
	 *
	 * False for automated and scripted imports, which have nobody to answer
	 * one: an unanswered dialog resolves to Cancel, so prompting unattended
	 * turns every scripted import into a cancellation.
	 */
	bool bAllowPrompts = true;
};

/**
 * @class UMujocoImportFactory
 * @brief Handles drag-and-drop import of MuJoCo .xml files into the Content Browser.
 * Creates an AMjArticulation Blueprint and populates it with components.
 */
UCLASS()
class URLABEDITOR_API UMujocoImportFactory : public UFactory, public FReimportHandler
{
	GENERATED_BODY()

public:
	UMujocoImportFactory();

	// UFactory Interface
	virtual UObject* FactoryCreateFile(UClass* InClass, UObject* InParent, FName InName, EObjectFlags Flags, const FString& Filename, const TCHAR* Parms, FFeedbackContext* Warn, bool& bOutOperationCanceled) override;
	virtual bool FactoryCanImport(const FString& Filename) override;

	// FReimportHandler Interface
	virtual bool CanReimport(UObject* Obj, TArray<FString>& OutFilenames) override;
	virtual void SetReimportPaths(UObject* Obj, const TArray<FString>& NewReimportPaths) override;
	virtual EReimportResult::Type Reimport(UObject* Obj) override;
	virtual int32 GetPriority() const override;

	/**
	 * The one import pipeline, taken by import and by reimport alike.
	 *
	 * `SourceXmlPath`'s meshes are prepared first; only once that has succeeded
	 * is `AcquireBlueprint` asked for the Blueprint to read into, which is what
	 * keeps a failed preparation from leaving an asset behind on a fresh import
	 * while letting a reimport hand back the Blueprint it already has. The
	 * Blueprint records the ORIGINAL path, never the prepared copy, so the next
	 * reimport prepares from what the user actually chose.
	 *
	 * `OutBlueprint` carries whatever `AcquireBlueprint` produced, including
	 * when the read then failed, so the caller can decide what to do with it.
	 */
	static bool ImportModel(const FString& SourceXmlPath, const FMjImportSettings& Settings,
		TFunctionRef<UBlueprint*()> AcquireBlueprint, UBlueprint*& OutBlueprint,
		FString& OutError, bool& bOutCancelled);

	/**
	 * Where a model's prepared copy is written.
	 *
	 * Under the project's Saved directory rather than beside the model, so
	 * preparation never leaves artifacts in the author's own folders, and keyed
	 * by the model's file stem so import and reimport prepare to one place.
	 */
	static FString ImportPrepDir(const FString& SourceXmlPath);

	/**
	 * Run a mesh-preparation script over `SourceXmlPath` and report the
	 * document to parse.
	 *
	 * On success `OutXmlPath` is the prepared copy in `ImportPrepDir`. A
	 * nonzero exit fails with the script's stderr in `OutError`, and so does a
	 * zero exit that wrote no prepared document: parsing the unprepared
	 * original is how a model imports at the wrong scale while looking
	 * plausible.
	 *
	 * `bAllowExternalIncludes` is the reader's security option, forwarded to
	 * the script because preparation flattens the model's `<include>` fragments
	 * itself, before the reader sees the document. Without it the gate would be
	 * decided by a step that never asked.
	 */
	static bool RunMeshPreparation(const FString& PythonExe, const FString& ScriptPath,
		const FString& SourceXmlPath, bool bAllowExternalIncludes, FString& OutXmlPath,
		FString& OutError);
};
