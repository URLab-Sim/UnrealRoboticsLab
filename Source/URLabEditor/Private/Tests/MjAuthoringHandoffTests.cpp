// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "Bridge/BridgeSubsystem.h"
#include "Bridge/BridgeServer.h"
#include "Engine/Blueprint.h"
#include "Engine/BlueprintGeneratedClass.h"
#include "Engine/Engine.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "ImageUtils.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "Misc/CommandLine.h"
#include "Misc/PackageName.h"
#include "Misc/Paths.h"
#include "UObject/Linker.h"
#include "UObject/Package.h"
#include "UObject/SavePackage.h"
#include "UObject/UObjectHash.h"
#include "MuJoCo/Authoring/MjAuthoringCompiler.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Elements/MjCamera.h"
#include "MuJoCo/Elements/MjGeom.h"
#include "MuJoCo/Elements/MjMesh.h"
#include "MuJoCo/Gen/Elements/Options/MjCompiler.gen.h"
#include "MuJoCo/Elements/MjTexture.h"
#include "MuJoCo/Gen/Elements/Assets/MjAsset.gen.h"
#include "MuJoCo/Gen/Elements/Assets/MjMaterial.gen.h"
#include "MuJoCo/Gen/Elements/Assets/MjHfield.gen.h"
#include "MuJoCo/Gen/Elements/Bodies/MjBody.gen.h"
#include "MuJoCo/Render/MjLauncherFlags.h"
#include "MuJoCo/Runtime/MjSimulationRuntime.h"
#include "MuJoCo/Runtime/MjSimulationWorldSubsystem.h"
#include "MuJoCo/Spec/MjGenHooks.h"
#include "MuJoCo/Spec/MjNodeFactories.h"
#include "MuJoCo/Spec/MjSourceBundle.h"
#include "Render/URLabRenderScene.h"

#if WITH_DEV_AUTOMATION_TESTS && URLAB_MJ_GEN

namespace
{
struct FAuthoredHandoffFixture
{
	UWorld* World = nullptr;
	AAMjManager* Manager = nullptr;
	AMjArticulation* Robot = nullptr;
	UMjGeom* Geom = nullptr;
	UMjCamera* Camera = nullptr;
	UStaticMesh* SourceMesh = nullptr;

	bool Create()
	{
		UURLabBridgeSubsystem::SetUnboundForTest(true);
		World = UWorld::CreateWorld(EWorldType::Game, false);
		if (!World || !GEngine) return false;
		FWorldContext& Context = GEngine->CreateNewWorldContext(EWorldType::Game);
		Context.SetCurrentWorld(World);
		Manager = World->SpawnActor<AAMjManager>();
		Robot = World->SpawnActor<AMjArticulation>();
		if (!Manager || !Robot) return false;

		using namespace urlab::spec;
		FMjInstanceScope Scope(*Robot);
		UMjAsset& Assets = FInstanceNodeFactory::Create<UMjAsset>(*Robot->Spec);
		UMjMesh* Mesh = Cast<UMjMesh>(
			&FInstanceNodeFactory::Create<UMjMeshBase>(Assets));
		UMjMaterial& Material = FInstanceNodeFactory::Create<UMjMaterial>(Assets);
		UMjBodyBase& WorldBody = FInstanceNodeFactory::Create<UMjBodyBase>(*Robot->Spec);
		UMjBodyBase& Body = FInstanceNodeFactory::Create<UMjBodyBase>(WorldBody);
		Geom = Cast<UMjGeom>(&FInstanceNodeFactory::Create<UMjGeomBase>(Body));
		Camera = Cast<UMjCamera>(&FInstanceNodeFactory::Create<UMjCameraBase>(Body));
		if (!Mesh || !Geom || !Camera) return false;

		Mesh->MjName = FString(TEXT("authored_mesh"));
		Mesh->Vertex = TArray<float>({0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1});
		Mesh->Face = TArray<int32>({0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3});
		SourceMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
		Mesh->MeshAsset = SourceMesh;
		Mesh->FileAsset = FSoftObjectPath(SourceMesh);

		Material.MjName = FString(TEXT("authored_material"));
		Material.Rgba = FLinearColor(0.15f, 0.35f, 0.65f, 1.0f);
		Material.Metallic = 0.7f;
		Material.Roughness = 0.2f;

		Body.MjName = FString(TEXT("body"));
		Geom->MjName = FString(TEXT("visual"));
		Geom->Type = EMjGeomType::mesh;
		Geom->Mesh = FString(TEXT("authored_mesh"));
		Geom->Material = FString(TEXT("authored_material"));
		Geom->Pos = FMjPosition3(0.25, -0.5, 0.75);

		Camera->MjName = FString(TEXT("sensor_camera"));
		Camera->Resolution = TArray<int32>({320, 200});
		Camera->CaptureMode = ERenderCameraMode::Depth;
		Camera->DepthNearCm = 3.0f;
		Camera->DepthFarCm = 900.0f;
		Camera->HistoryCapacity = 5;
		Camera->DelaySeconds = 0.125f;
		Camera->bEnableZmqBroadcast = true;
		Camera->ZmqEndpoint = TEXT("tcp://127.0.0.1:6123");
		Camera->CustomNearClipCm = 4.0f;
		Camera->CustomFarClipCm = 800.0f;
		return SourceMesh != nullptr;
	}

	~FAuthoredHandoffFixture()
	{
		if (World)
		{
			if (UMjSimulationWorldSubsystem* Runtime =
				World->GetSubsystem<UMjSimulationWorldSubsystem>())
			{
				Runtime->ShutdownService();
			}
			World->DestroyWorld(false);
			GEngine->DestroyWorldContext(World);
		}
		UURLabBridgeSubsystem::SetUnboundForTest(false);
	}
};
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjAuthoredOwnedHandoffTest,
	"URLab.Authoring.OwnedCompilerHandoffSurvivesAuthorRetirement",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjAuthoredOwnedHandoffTest::RunTest(const FString&)
{
	FAuthoredHandoffFixture Fixture;
	if (!TestTrue(TEXT("authored fixture created"), Fixture.Create())) return false;

	FMjAuthoredSimulationPackage Package = FMjAuthoringCompiler::Compile(*Fixture.Manager);
	if (!TestTrue(TEXT("authored scene produced an owned candidate"), Package.IsValid()))
	{
		AddError(Package.Error);
		return false;
	}
	const int32 CameraId = Fixture.Camera->GetBoundId().Get(-1);
	// Geometry and material appearance remain those compiled by MuJoCo; the
	// authored handoff contributes camera capture settings only.
	const FMjResolvedRenderCameraOverride* CameraOverride =
		Package.Presentation.CameraOverrides.Find(CameraId);
	if (TestNotNull(TEXT("bound camera has a resolved override"), CameraOverride))
	{
		TestEqual(TEXT("camera capture mode retained"), CameraOverride->Capture.Mode,
			ERenderCameraMode::Depth);
		TestEqual(TEXT("camera history retained"), CameraOverride->Capture.HistoryCapacity, 5);
		TestEqual(TEXT("camera delay retained"), CameraOverride->Capture.DelaySeconds, 0.125f);
		TestTrue(TEXT("camera ZMQ choice retained"), CameraOverride->bEnableZmq);
		TestEqual(TEXT("camera endpoint retained"), CameraOverride->ZmqEndpoint,
			FString(TEXT("tcp://127.0.0.1:6123")));
	}

	UMjSimulationWorldSubsystem* Runtime =
		Fixture.World->GetSubsystem<UMjSimulationWorldSubsystem>();
	FString Error;
	if (!TestNotNull(TEXT("world simulation subsystem"), Runtime)
		|| !TestTrue(TEXT("simulation runtime activates"), Runtime->ActivateRuntime(Error)))
	{
		AddError(Error);
		return false;
	}
	const FMjSimulationWorldInstallResult Installed = Runtime->InstallCandidate(
		MoveTemp(Package.Candidate), Package.Presentation);
	if (!TestTrue(TEXT("authored candidate installs through common coordinator"),
		Installed.bCommitted))
	{
		AddError(Installed.Error);
		return false;
	}
	AURLabRenderScene* Scene = Runtime->GetActiveScene();
	if (!TestNotNull(TEXT("owned neutral scene is active"), Scene)) return false;
	const int32 GeomId = Fixture.Geom->GetBoundId().Get(-1);
	if (!TestTrue(TEXT("neutral scene contains compiled authored geom"),
		Scene->GetScene().Geoms.IsValidIndex(GeomId))) return false;
	const FRenderGeomDesc* CompiledGeom = &Scene->GetScene().Geoms[GeomId];
	TestFalse(TEXT("native compiled geom pose reaches the neutral scene"),
		CompiledGeom->BodyLocalTransform.Equals(FTransform::Identity));
	TestTrue(TEXT("neutral scene uses compiled mesh resource"),
		CompiledGeom->MeshResourceIndex != INDEX_NONE);

	Fixture.Robot->Destroy();
	Fixture.Manager->Destroy();
	Fixture.Robot = nullptr;
	Fixture.Manager = nullptr;
	CollectGarbage(RF_NoFlags);

	FMjModelDescriptor Descriptor;
	FMjModelObservation Observation;
	TestTrue(TEXT("typed descriptor survives author retirement"),
		Runtime->GetRuntime()->GetDescriptor(Descriptor));
	TestTrue(TEXT("typed observation survives author retirement"),
		Runtime->GetRuntime()->GetObservation(Observation));
	TestEqual(TEXT("compiled camera survives author retirement"), Descriptor.Cameras.Num(), 1);
	TestTrue(TEXT("neutral scene survives author retirement"),
		Runtime->GetActiveScene() == Scene && Scene->NumBuiltGeoms() > 0);
	TestTrue(TEXT("installed source is the copied compiled model"),
		Installed.Source.Kind == EMjModelSourceKind::CompiledModel);
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjManagerAuthoredBeginPlayCutoverTest,
	"URLab.Authoring.ManagerBeginPlayHandsOffAndRetires",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjManagerAuthoredBeginPlayCutoverTest::RunTest(const FString&)
{
	URLabLauncherFlags::FScopedCommandLine CommandLine(TEXT("-someothergame"));
	FAuthoredHandoffFixture Fixture;
	if (!TestTrue(TEXT("authored fixture created"), Fixture.Create())) return false;
	AAMjManager* Manager = Fixture.Manager;
	AMjArticulation* Robot = Fixture.Robot;

	Manager->DispatchBeginPlay();
	UMjSimulationWorldSubsystem* Runtime =
		Fixture.World->GetSubsystem<UMjSimulationWorldSubsystem>();
	if (!TestNotNull(TEXT("world simulation subsystem"), Runtime)) return false;
	TestTrue(TEXT("manager BeginPlay activates the independent runtime"), Runtime->IsRuntimeActive());
	TestFalse(TEXT("default authored boot remains local without RPC"), Runtime->IsBridgeAttached());
	TestNull(TEXT("default authored boot did not create a bridge"),
		UURLabBridgeSubsystem::Find(Fixture.World));
	TestTrue(TEXT("manager BeginPlay installs the authored model"),
		Runtime->GetRuntime().IsValid() && Runtime->GetRuntime()->HasModel());
	TestNotNull(TEXT("manager BeginPlay installs the neutral scene"), Runtime->GetActiveScene());
	TestTrue(TEXT("manager retires after its synchronous handoff"),
		Manager->IsActorBeingDestroyed());
	TestTrue(TEXT("articulation retires after its synchronous handoff"),
		Robot->IsActorBeingDestroyed());

	FMjModelObservation Observation;
	TestTrue(TEXT("runtime remains readable after author EndPlay"),
		Runtime->GetRuntime()->GetObservation(Observation));
	Fixture.Manager = nullptr;
	Fixture.Robot = nullptr;
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjGeneratedAssetPayloadTest,
	"URLab.Authoring.GeneratedAssetsUseCurrentUEState",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjGeneratedAssetPayloadTest::RunTest(const FString&)
{
	FAuthoredHandoffFixture Fixture;
	if (!TestTrue(TEXT("authored fixture created"), Fixture.Create())) return false;

	UMjMesh* Mesh = nullptr;
	TArray<UMjNodeComponent*> Nodes;
	Fixture.Robot->GetComponents(Nodes);
	for (UMjNodeComponent* Node : Nodes)
		if ((Mesh = Cast<UMjMesh>(Node)) != nullptr) break;
	if (!TestNotNull(TEXT("mesh asset node"), Mesh)) return false;
	Mesh->Vertex.Reset();
	Mesh->Face.Reset();
	Mesh->SourceFile = TEXT("/urlab/original/path/model.xml");
	UMjCompiler* Compiler = nullptr;
	{
		using namespace urlab::spec;
		FMjInstanceScope Scope(*Fixture.Robot);
		UMjCompiler& CompilerRef = FInstanceNodeFactory::Create<UMjCompiler>(*Fixture.Robot->Spec);
		Compiler = &CompilerRef;
		CompilerRef.SetAssetdir(TEXT("old-assets"));
		CompilerRef.SetMeshdir(TEXT("old-meshes"));
		CompilerRef.SetTexturedir(TEXT("old-textures"));
		CompilerRef.SetStrippath(true);
	}

	FString Error;
	if (!TestTrue(TEXT("generate compiler resource from current UStaticMesh"),
		MjPrepareAssetResource(*Mesh, Error)))
	{
		AddError(Error);
		return false;
	}
	TestTrue(TEXT("generated filename is independent of original source filename"),
		Mesh->GetFile().StartsWith(TEXT("__urlab_mesh_")));
	const FMjPreparedAssetResource* Row = Mesh->PreparedAssetResources.FindByPredicate(
		[](const FMjPreparedAssetResource& Candidate)
		{
			return Candidate.Attribute == TEXT("file");
		});
	if (!TestNotNull(TEXT("generated mesh row exists"), Row)) return false;
	TestEqual(TEXT("VFS mount uses the generated current filename"), Row->VfsName, Row->File);
	TestTrue(TEXT("generated OBJ bytes are present"), !Row->Bytes.IsEmpty());
	TestEqual(TEXT("row references selected UE asset"), Row->AssetPath,
		FSoftObjectPath(Mesh->MeshAsset.Get()));
	TestTrue(TEXT("generated payload records its current asset fingerprint"),
		!Row->Fingerprint.IsEmpty());

	MjFinalizeImportedAuthoredResources(FSpecRef::OverOwner(Fixture.Robot->Spec));
	TestTrue(TEXT("source path is cleared after materialization"), Mesh->SourceFile.IsEmpty());
	if (TestNotNull(TEXT("compiler node survives finalization"), Compiler))
	{
		TestFalse(TEXT("imported assetdir host path is cleared"), Compiler->HasAssetdir());
		TestFalse(TEXT("imported meshdir host path is cleared"), Compiler->HasMeshdir());
		TestFalse(TEXT("imported texturedir host path is cleared"), Compiler->HasTexturedir());
		TestFalse(TEXT("source-only path stripping is cleared"), Compiler->HasStrippath());
	}
	FMjAuthoredSimulationPackage Package = FMjAuthoringCompiler::Compile(*Fixture.Manager);
	if (!TestTrue(TEXT("generated-only resource compiles through the native authored handoff"),
		Package.IsValid()))
	{
		AddError(Package.Error);
		return false;
	}

	FMjSourceBundle Bundle;
	TArray<FMjSpecDiagnostic> Diagnostics;
	if (!TestTrue(TEXT("portable export is generated from current UE state"),
		MjBuildSourceBundle(FSpecRef::OverOwner(Fixture.Robot->Spec), Bundle, Diagnostics)))
	{
		for (const FMjSpecDiagnostic& Diagnostic : Diagnostics) AddError(Diagnostic.ToString());
		return false;
	}
	TestTrue(TEXT("bundle contains no source XML resource row"),
		!Bundle.Resources.ContainsByPredicate([](const FMjGeneratedBundleResource& Resource)
		{
			return FPaths::GetExtension(Resource.Key).Equals(TEXT("xml"), ESearchCase::IgnoreCase);
		}));
	return !HasAnyErrors();
}

#endif
