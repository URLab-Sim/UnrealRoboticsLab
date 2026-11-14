// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include "CoreMinimal.h"
#include "Engine/Blueprint.h"
#include "Engine/World.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "UObject/Package.h"

#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Authoring/MjAuthoringCompiler.h"
#include "MuJoCoSpec/MJXmlImporterExporter.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

struct FMjTestSession
{
	mjSpec* spec = nullptr;
	mjModel* m = nullptr;
	mjData* d = nullptr;

	FString LastError;

	bool CompileXml(const FString& Xml)
	{
		Cleanup();
		char szErr[1000] = "";
		spec = mj_parseXMLString(TCHAR_TO_UTF8(*Xml), nullptr, szErr, sizeof(szErr));
		if (!spec)
		{
			LastError = FString::Printf(TEXT("mj_parseXMLString failed: %hs"), szErr);
			return false;
		}
		m = mj_compile(spec, nullptr);
		if (!m)
		{
			const char* specErr = mjs_getError(spec);
			LastError = FString::Printf(TEXT("mj_compile failed: %hs"), specErr ? specErr : "unknown");
			return false;
		}
		d = mj_makeData(m);
		if (!d)
		{
			LastError = TEXT("mj_makeData returned null");
			return false;
		}
		return true;
	}

	void Step(int N = 1)
	{
		if (m && d)
			for (int i = 0; i < N; ++i)
				mj_step(m, d);
	}

	void Forward()
	{
		if (m && d)
			mj_forward(m, d);
	}

	void Reset()
	{
		if (m && d)
			mj_resetData(m, d);
	}

	int BodyId(const char* Name) const { return m ? mj_name2id(m, mjOBJ_BODY, Name) : -1; }
	int GeomId(const char* Name) const { return m ? mj_name2id(m, mjOBJ_GEOM, Name) : -1; }
	int JointId(const char* Name) const { return m ? mj_name2id(m, mjOBJ_JOINT, Name) : -1; }
	int ActuatorId(const char* Name) const { return m ? mj_name2id(m, mjOBJ_ACTUATOR, Name) : -1; }
	int SensorId(const char* Name) const { return m ? mj_name2id(m, mjOBJ_SENSOR, Name) : -1; }

	void Cleanup()
	{
		if (d) { mj_deleteData(d); d = nullptr; }
		if (m) { mj_deleteModel(m); m = nullptr; }
		if (spec) { mj_deleteSpec(spec); spec = nullptr; }
	}

	~FMjTestSession() { Cleanup(); }
};

struct FMjXmlImportSession
{
	UWorld* World = nullptr;
	AAMjManager* Manager = nullptr;
	AMjArticulation* Articulation = nullptr;
	UBlueprint* Blueprint = nullptr;
	FMjAuthoredSimulationPackage Compiled;
	FString LastError;

	bool Init(const FString& Xml)
	{
		Cleanup();

		Blueprint = FKismetEditorUtilities::CreateBlueprint(
			AMjArticulation::StaticClass(),
			GetTransientPackage(),
			MakeUniqueObjectName(GetTransientPackage(), UBlueprint::StaticClass(), TEXT("TestMjBP")),
			BPTYPE_Normal,
			UBlueprint::StaticClass(),
			UBlueprintGeneratedClass::StaticClass());
		if (!Blueprint)
		{
			LastError = TEXT("CreateBlueprint failed");
			return false;
		}

		if (!FMJXmlImporterExporter::ImportXMLToBlueprintSCS(Xml, Blueprint, nullptr, LastError))
		{
			return false;
		}
		FKismetEditorUtilities::CompileBlueprint(Blueprint);

		World = UWorld::CreateWorld(EWorldType::Editor, false);
		if (!World)
		{
			LastError = TEXT("CreateWorld failed");
			return false;
		}

		Manager = World->SpawnActor<AAMjManager>();
		Articulation = World->SpawnActor<AMjArticulation>(Blueprint->GeneratedClass, FTransform::Identity);
		if (!Manager || !Articulation)
		{
			LastError = TEXT("Failed to spawn Manager or Articulation");
			return false;
		}
		return true;
	}

	bool Compile()
	{
		if (!Manager)
		{
			LastError = TEXT("Manager is null");
			return false;
		}
		Compiled = FMjAuthoringCompiler::Compile(*Manager);
		LastError = Compiled.Error;
		return Compiled.IsValid();
	}

	bool VisitPrepared(TFunctionRef<bool(const mjModel*, const mjData*)> Visitor) const { return Compiled.Candidate.VisitPrepared(Visitor); }

	void Cleanup()
	{
		Compiled = FMjAuthoredSimulationPackage();
		Articulation = nullptr;
		Manager = nullptr;
		if (World)
		{
			World->DestroyWorld(false);
			World = nullptr;
		}
		Blueprint = nullptr;
	}

	~FMjXmlImportSession() { Cleanup(); }
};
