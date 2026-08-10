// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// The three ways a URLab test gets something to assert against.
//
// FMjTestSession is MuJoCo and nothing else: use it for the coordinate and unit
// utilities, or as a reference compile to hold FMjUESession against.
//
// FMjUESession is a world with a manager and one articulation, built through the
// same factories the reader uses and compiled through the same spec path the
// engine uses. Elements are created with Add<E>() rather than NewObject: the
// factory is what stamps identity and spec order onto a node, and an element
// missing either is not a spec element -- it will not serialise and will not
// bind, and it fails in a way that reads like a compiler bug.
//
// FMjXmlImportSession is the editor's importer: MJCF text into a Blueprint whose
// construction script IS the spec, optionally compiled afterwards.

#include "CoreMinimal.h"
#include "Engine/Blueprint.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#include "Engine/World.h"
#include "HAL/FileManager.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "Misc/FileHelper.h"
#include "PackageTools.h"
#include "UObject/Package.h"

#include "Bridge/RpcDispatcher.h"
#include "MujocoGenerationAction.h"

#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Spec/MjGenHooks.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Elements/MjBody.h"
#include "MuJoCo/Elements/MjCamera.h"
#include "MuJoCo/Elements/MjFlexcomp.h"
#include "MuJoCo/Elements/MjGeom.h"
#include "MuJoCo/Gen/Elements/Joints/MjJoint.gen.h"

#if URLAB_MJ_GEN
#include "MuJoCo/Spec/MjNodeFactories.h"
#endif

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

// ---------------------------------------------------------------------------
// FMjTestSession: MuJoCo on its own
// ---------------------------------------------------------------------------

struct FMjTestSession
{
	mjSpec* spec = nullptr;
	mjModel* m = nullptr;
	mjData* d = nullptr;

	FString LastError;

	/** Compile an inline MJCF string. Returns true on success. */
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
	int SensorId(const char* Name) const { return m ? mj_name2id(m, mjOBJ_SENSOR, Name) : -1; }
	int ActuatorId(const char* Name) const { return m ? mj_name2id(m, mjOBJ_ACTUATOR, Name) : -1; }

	bool IsValid() const { return m != nullptr && d != nullptr; }

	void Cleanup()
	{
		if (d)
		{
			mj_deleteData(d);
			d = nullptr;
		}
		if (m)
		{
			mj_deleteModel(m);
			m = nullptr;
		}
		if (spec)
		{
			mj_deleteSpec(spec);
			spec = nullptr;
		}
	}

	~FMjTestSession() { Cleanup(); }
};

// ---------------------------------------------------------------------------
// FMjUESession: a world, a manager, and one articulation
// ---------------------------------------------------------------------------

/**
 * The generated element class `E` names.
 *
 * Itself for a generated class, and its base for one of the four hand
 * subclasses. The factories are keyed on the generated type -- a hand subclass
 * is a registration, not a second element -- so a test that asks for `UMjBody`
 * has to be turned back into the `UMjBodyBase` the factory builds. Four entries,
 * because there are four subclasses and the list does not grow with the schema.
 */
template <typename E>
struct TMjGeneratedOf
{
	using Type = E;
};
template <>
struct TMjGeneratedOf<UMjBody>
{
	using Type = UMjBodyBase;
};
template <>
struct TMjGeneratedOf<UMjGeom>
{
	using Type = UMjGeomBase;
};
template <>
struct TMjGeneratedOf<UMjCamera>
{
	using Type = UMjCameraBase;
};
template <>
struct TMjGeneratedOf<UMjFlexcomp>
{
	using Type = UMjFlexcompBase;
};

/**
 * A minimal articulation -- one body carrying one geom and one joint -- spawned
 * into a throwaway world and compiled through the spec path.
 *
 * `Compile()` is driven directly because `BeginPlay` does not fire in a headless
 * test world. The optional `ConfigCallback` runs after the elements exist and
 * before the compile, which is where a test authors whatever it is asserting on.
 */
struct FMjUESession
{
	UWorld* World = nullptr;
	AAMjManager* Manager = nullptr;
	AMjArticulation* Robot = nullptr;

	/** The spec's <worldbody>. Anonymous by necessity; see Init. */
	UMjBodyBase* WorldBody = nullptr;

	/** The single user body, the geom on it, and the joint on it. */
	UMjBody* Body = nullptr;
	UMjGeom* Geom = nullptr;
	UMjJoint* Joint = nullptr;

	FString LastError;

	bool Init(TFunction<void(FMjUESession&)> ConfigCallback = nullptr)
	{
#if !URLAB_MJ_GEN
		LastError = TEXT("URLab was built without the generated MuJoCo spec profile");
		return false;
#else
		World = UWorld::CreateWorld(EWorldType::Game, false);
		if (!World)
		{
			LastError = TEXT("CreateWorld failed");
			return false;
		}

		FWorldContext& Ctx = GEngine->CreateNewWorldContext(EWorldType::Game);
		Ctx.SetCurrentWorld(World);

		FActorSpawnParameters P;
		Robot = World->SpawnActor<AMjArticulation>(P);
		if (!Robot)
		{
			LastError = TEXT("SpawnActor AMjArticulation failed");
			return false;
		}

		// The first body in the model's worldbody slot IS <worldbody>, which MJCF
		// forbids from carrying attributes -- naming it, or posing it, makes the
		// spec unparseable. So the session authors an anonymous world body
		// and hangs everything the tests care about underneath it.
		WorldBody = Add<UMjBodyBase>(Robot->Spec);
		if (WorldBody == nullptr)
		{
			LastError = TEXT("could not create the world body");
			return false;
		}
		Body = Add<UMjBody>(WorldBody, TEXT("RootBody"));
		if (Body == nullptr)
		{
			LastError = TEXT("could not create the root body");
			return false;
		}
		Geom = Add<UMjGeom>(Body, TEXT("TestGeom"));
		Joint = Add<UMjJoint>(Body, TEXT("TestJoint"));
		if (Geom == nullptr || Joint == nullptr)
		{
			LastError = TEXT("could not create the geom or the joint");
			return false;
		}
		Geom->SetSize({0.1});

		Manager = World->SpawnActor<AAMjManager>(P);
		if (!Manager)
		{
			LastError = TEXT("SpawnActor AAMjManager failed");
			return false;
		}

		if (ConfigCallback)
			ConfigCallback(*this);

		Manager->Compile();
		if (!Manager->IsInitialized())
		{
			LastError = FString::Printf(TEXT("Compile() failed: %s"),
				*Manager->PhysicsEngine->GetLastCompileError());
			return false;
		}

		// AAMjManager::BeginPlay normally stands the bridge server up; tests
		// bypass BeginPlay, and anything exercising step-server semantics needs
		// the dispatcher. The empty endpoint skips the ZMQ bind so concurrent
		// tests do not fight over the default port.
		Manager->BridgeServer = NewObject<UURLabBridgeServer>(Manager, TEXT("BridgeServer"));
		Manager->BridgeServer->SetOwnedByManager(true);
		Manager->BridgeServer->Start(TEXT(""));
		Manager->BridgeServer->RegisterManager(Manager);
		return true;
#endif
	}

	/**
	 * Create an element under `Parent`, as the reader would.
	 *
	 * Goes through the instance factory rather than `NewObject` because that is
	 * what stamps a node's identity and its position among its siblings, and an
	 * element missing either does not serialise and does not bind.
	 *
	 * `E` may be a generated class or a hand subclass of one; the factory builds
	 * whichever class is registered for the element either way, so the cast at
	 * the end is what the caller asked for.
	 */
	template <typename E>
	E* Add(UMjNodeComponent* Parent, const TCHAR* ElementName = nullptr)
	{
#if URLAB_MJ_GEN
		if (Robot == nullptr || Parent == nullptr)
		{
			return nullptr;
		}
		urlab::spec::FMjInstanceScope Scope(*Robot);
		UMjNodeComponent& Node = urlab::spec::FInstanceNodeFactory::Create<
			typename TMjGeneratedOf<E>::Type>(*Parent);
		if (ElementName != nullptr)
		{
			Node.MjName = ElementName;
		}
		return Cast<E>(&Node);
#else
		return nullptr;
#endif
	}

	/** Recompile after a spec edit, and report why when it fails. */
	bool Recompile()
	{
		if (Manager == nullptr)
		{
			LastError = TEXT("no manager");
			return false;
		}
		Manager->Compile();
		if (!Manager->IsInitialized())
		{
			LastError = Manager->PhysicsEngine->GetLastCompileError();
			return false;
		}
		return true;
	}

	mjModel* Model() const
	{
		return (Manager && Manager->PhysicsEngine) ? Manager->PhysicsEngine->m_model : nullptr;
	}
	mjData* Data() const
	{
		return (Manager && Manager->PhysicsEngine) ? Manager->PhysicsEngine->m_data : nullptr;
	}

	/**
	 * The compiled id of one of the articulation's elements, by its MJCF name
	 * under the articulation's prefix. -1 when the model holds no such object.
	 *
	 * Prefer this to an element's own bound id whenever the assertion is about
	 * the model: it then reads the model and nothing else.
	 */
	int MjId(mjtObj Type, const TCHAR* ElementName) const
	{
		mjModel* M = Model();
		if (!M || !Robot)
			return -1;
		const FString Full = Robot->GetCompiledPrefix() + ElementName;
		return mj_name2id(M, Type, TCHAR_TO_UTF8(*Full));
	}

	void Step(int N = 1)
	{
		mjModel* M = Model();
		mjData* D = Data();
		if (M && D)
			for (int i = 0; i < N; ++i)
				mj_step(M, D);
		// The session stands in for the physics worker, and the worker publishes
		// a snapshot after every step. The Blueprint-facing accessors read that
		// snapshot, so a session that stepped without publishing would answer
		// every question with the state before the step.
		if (Manager && Manager->PhysicsEngine)
			Manager->PhysicsEngine->PushRenderState();
	}

	void Cleanup()
	{
		if (Manager)
		{
			// Tear the dispatcher down before the world goes away so its custom
			// step handler is uninstalled while the engine is still valid.
			if (Manager->BridgeServer)
			{
				Manager->BridgeServer->UnregisterManager(Manager);
				Manager->BridgeServer->Stop();
				Manager->BridgeServer = nullptr;
			}
			Manager->PhysicsEngine->bShouldStopTask = true;
		}
		if (World)
		{
			World->DestroyWorld(false);
			GEngine->DestroyWorldContext(World);
			World = nullptr;
			Manager = nullptr;
			Robot = nullptr;
			WorldBody = nullptr;
			Body = nullptr;
			Geom = nullptr;
			Joint = nullptr;
		}
	}

	~FMjUESession() { Cleanup(); }
};

// ---------------------------------------------------------------------------
// FMjXmlImportSession: MJCF to a Blueprint, and optionally to a model
// ---------------------------------------------------------------------------

/**
 * Runs MJCF through the editor importer and exposes the Blueprint's construction
 * script, which after a successful import IS the spec.
 *
 * Two tiers: `Init` alone for asserting on what the importer produced, and
 * `Compile` on top for asserting on the model it compiles to.
 */
struct FMjXmlImportSession
{
	UBlueprint* Blueprint = nullptr;
	UWorld* World = nullptr;
	AAMjManager* Manager = nullptr;
	AMjArticulation* Robot = nullptr;

	FString LastError;
	FString TempXmlPath;

	// Counts from a stock MuJoCo load of the same file, for comparison.
	int32 NativeBodyCount = 0;
	int32 NativeJointCount = 0;
	int32 NativeActuatorCount = 0;
	int32 NativeGeomCount = 0;
	bool bOwnsTempFile = true;

	/** Import an MJCF file already on disk. Use for Menagerie models with meshes. */
	bool InitFromFile(const FString& XmlFilePath)
	{
		if (!FPaths::FileExists(XmlFilePath))
		{
			LastError = FString::Printf(TEXT("File not found: %s"), *XmlFilePath);
			return false;
		}
		TempXmlPath = XmlFilePath;
		bOwnsTempFile = false;

		{
			char szErr[1000] = "";
			mjModel* TmpM = mj_loadXML(TCHAR_TO_UTF8(*TempXmlPath), nullptr, szErr, sizeof(szErr));
			if (!TmpM)
			{
				LastError = FString::Printf(TEXT("MuJoCo rejected XML: %hs"), szErr);
				return false;
			}
			NativeBodyCount = TmpM->nbody;
			NativeJointCount = TmpM->njnt;
			NativeActuatorCount = TmpM->nu;
			NativeGeomCount = TmpM->ngeom;
			mj_deleteModel(TmpM);
		}

		return RunImporter();
	}

	/** Write `XmlContent` to a temp file and import it. */
	bool Init(const FString& XmlContent)
	{
		bOwnsTempFile = true;
		const FString TempDir = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("URLab/Tests"));
		IFileManager::Get().MakeDirectory(*TempDir, true);
		TempXmlPath = FPaths::Combine(TempDir,
			FString::Printf(TEXT("import_%d.xml"), FMath::RandRange(0, 999999)));

		if (!FFileHelper::SaveStringToFile(XmlContent, *TempXmlPath))
		{
			LastError = TEXT("Failed to write temp XML");
			return false;
		}

		{
			char szErr[1000] = "";
			mjModel* TmpM = mj_loadXML(TCHAR_TO_UTF8(*TempXmlPath), nullptr, szErr, sizeof(szErr));
			if (!TmpM)
			{
				LastError = FString::Printf(TEXT("MuJoCo rejected XML: %hs"), szErr);
				IFileManager::Get().Delete(*TempXmlPath);
				TempXmlPath.Empty();
				return false;
			}
			mj_deleteModel(TmpM);
		}

		return RunImporter();
	}

	/**
	 * Spawn a world, place the imported Blueprint in it, and compile.
	 *
	 * `Robot` may stay null for a model with no world body; that is not an error.
	 */
	bool Compile()
	{
		if (!Blueprint)
		{
			LastError = TEXT("Call Init() before Compile()");
			return false;
		}

		World = UWorld::CreateWorld(EWorldType::Game, false);
		if (!World)
		{
			LastError = TEXT("CreateWorld failed");
			return false;
		}

		FWorldContext& Ctx = GEngine->CreateNewWorldContext(EWorldType::Game);
		Ctx.SetCurrentWorld(World);

		FActorSpawnParameters P;
		Manager = World->SpawnActor<AAMjManager>(P);
		if (!Manager)
		{
			LastError = TEXT("SpawnActor AAMjManager failed");
			return false;
		}

		Robot = World->SpawnActor<AMjArticulation>(Blueprint->GeneratedClass,
			FVector::ZeroVector, FRotator::ZeroRotator, P);

		Manager->Compile();
		if (!Manager->IsInitialized())
		{
			LastError = FString::Printf(TEXT("Compile() failed: %s"),
				*Manager->PhysicsEngine->GetLastCompileError());
			return false;
		}
		return true;
	}

	mjModel* Model() const { return Manager ? Manager->PhysicsEngine->m_model : nullptr; }
	mjData* Data() const { return Manager ? Manager->PhysicsEngine->m_data : nullptr; }

	/** The first construction-script template of type `T` named `Name`. */
	template <typename T>
	T* FindTemplate(const FString& Name) const
	{
		if (!Blueprint || !Blueprint->SimpleConstructionScript)
			return nullptr;
		for (USCS_Node* Node : Blueprint->SimpleConstructionScript->GetAllNodes())
		{
			T* Tmpl = Cast<T>(Node->ComponentTemplate);
			if (!Tmpl)
				continue;
			if (const UMjNodeComponent* Element = Cast<UMjNodeComponent>(Tmpl))
			{
				if (Element->MjName.IsSet() && Element->MjName.GetValue() == Name)
					return Tmpl;
			}
			if (Node->GetVariableName().ToString() == Name)
				return Tmpl;
		}
		return nullptr;
	}

	/** The first template of type `T`, for when the test authored only one. */
	template <typename T>
	T* FindFirstTemplate() const
	{
		if (!Blueprint || !Blueprint->SimpleConstructionScript)
			return nullptr;
		for (USCS_Node* Node : Blueprint->SimpleConstructionScript->GetAllNodes())
		{
			if (T* Tmpl = Cast<T>(Node->ComponentTemplate))
				return Tmpl;
		}
		return nullptr;
	}

	template <typename T>
	int32 CountTemplates() const
	{
		int32 N = 0;
		if (!Blueprint || !Blueprint->SimpleConstructionScript)
			return 0;
		for (USCS_Node* Node : Blueprint->SimpleConstructionScript->GetAllNodes())
		{
			if (Cast<T>(Node->ComponentTemplate))
				++N;
		}
		return N;
	}

	void Cleanup()
	{
		if (Manager)
			Manager->PhysicsEngine->bShouldStopTask = true;
		if (World)
		{
			World->DestroyWorld(false);
			GEngine->DestroyWorldContext(World);
			World = nullptr;
			Manager = nullptr;
			Robot = nullptr;
		}
		if (!TempXmlPath.IsEmpty() && bOwnsTempFile)
		{
			IFileManager::Get().Delete(*TempXmlPath);
		}
		TempXmlPath.Empty();
		Blueprint = nullptr;
	}

	~FMjXmlImportSession() { Cleanup(); }

private:
	/**
	 * Create the Blueprint and run the importer into it.
	 *
	 * Both the package path and the generated class name carry a fresh GUID:
	 * hardcoding either produced intermittent uniqueness assertions when garbage
	 * collection had not yet reclaimed the previous test's generated class.
	 */
	bool RunImporter()
	{
		const FString UniqueSfx = FGuid::NewGuid().ToString(EGuidFormats::Digits).Left(12);
		const FString PkgName = UPackageTools::SanitizePackageName(
			FString(TEXT("/Temp/URLabImportTest_")) + UniqueSfx);
		UPackage* Pkg = CreatePackage(*PkgName);

		const FString BPName = FString(TEXT("ImportTestArt_")) + UniqueSfx;
		Blueprint = FKismetEditorUtilities::CreateBlueprint(
			AMjArticulation::StaticClass(), Pkg, *BPName,
			BPTYPE_Normal, UBlueprint::StaticClass(), UBlueprintGeneratedClass::StaticClass());

		if (!Blueprint)
		{
			LastError = TEXT("FKismetEditorUtilities::CreateBlueprint failed");
			return false;
		}

		if (AMjArticulation* CDO = Cast<AMjArticulation>(Blueprint->GeneratedClass->GetDefaultObject()))
			CDO->MuJoCoXMLFile.FilePath = TempXmlPath;

		UMujocoGenerationAction* Gen = NewObject<UMujocoGenerationAction>();
		Gen->GenerateForBlueprint(Blueprint, TempXmlPath);
		return true;
	}
};

// ---------------------------------------------------------------------------
// Comparison helpers
// ---------------------------------------------------------------------------

namespace MjTestMath
{
inline bool NearlyEqual(float A, float B, float Eps = 1e-4f)
{
	return FMath::Abs(A - B) <= Eps;
}

inline bool NearlyEqual(double A, double B, double Eps = 1e-4)
{
	return FMath::Abs(A - B) <= Eps;
}

inline bool NearlyEqual(const FVector& A, const FVector& B, float Eps = 0.1f)
{
	return A.Equals(B, Eps);
}

inline bool NearlyEqual(const FQuat& A, const FQuat& B, float Eps = 0.01f)
{
	// q and -q are the same rotation.
	return A.Equals(B, Eps) || A.Equals(B.Inverse() * FQuat(0, 0, 0, -1), Eps)
		|| FQuat::ErrorAutoNormalize(A, B) < Eps;
}
}  // namespace MjTestMath
