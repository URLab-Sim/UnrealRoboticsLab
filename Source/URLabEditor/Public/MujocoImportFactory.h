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
#include "Factories/Factory.h"
#include "MujocoImportFactory.generated.h"

/**
 * @class UMujocoImportFactory
 * @brief Handles drag-and-drop import of MuJoCo .xml files into the Content Browser.
 * Creates an AMjArticulation Blueprint and populates it with components.
 */
UCLASS()
class URLABEDITOR_API UMujocoImportFactory : public UFactory
{
	GENERATED_BODY()

public:
	UMujocoImportFactory();

	// UFactory Interface
	virtual UObject* FactoryCreateFile(UClass* InClass, UObject* InParent, FName InName, EObjectFlags Flags, const FString& Filename, const TCHAR* Parms, FFeedbackContext* Warn, bool& bOutOperationCanceled) override;
	virtual bool FactoryCanImport(const FString& Filename) override;

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
	 */
	static bool RunMeshPreparation(const FString& PythonExe, const FString& ScriptPath,
		const FString& SourceXmlPath, FString& OutXmlPath, FString& OutError);
};
