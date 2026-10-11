// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#include "Misc/AutomationTest.h"
#include "Bridge/BridgeServer.h"
#include "Bridge/BridgeSubsystem.h"
#include "Bridge/RpcDispatcher.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"
#include "MuJoCo/Runtime/MjSimulationRuntime.h"
#include "MuJoCo/Runtime/MjSimulationWorldSubsystem.h"
#include "Render/RenderWorldSubsystem.h"
#include "Render/URLabRenderScene.h"

#include <limits>

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

#if WITH_DEV_AUTOMATION_TESTS
namespace
{
const TCHAR* WorldXml = TEXT(R"(
<mujoco model="world_subsystem">
  <worldbody>
    <body name="body" pos="0 0 1">
      <freejoint/><geom name="box" type="box" size="0.1 0.2 0.3" mass="1"/>
      <camera name="eye"/>
    </body>
  </worldbody>
</mujoco>)");

TArray<uint8> Utf8Bytes(const FString& Text)
{
	FTCHARToUTF8 Converted(*Text);
	TArray<uint8> Bytes;
	Bytes.Append(reinterpret_cast<const uint8*>(Converted.Get()), Converted.Length());
	return Bytes;
}

bool PrepareInvalidTimestepCandidate(FMjOwnedModelCandidate& OutCandidate,
	FString& OutError)
{
	FTCHARToUTF8 Xml(WorldXml);
	char Error[1024]{};
	mjSpec* Spec = mj_parseXMLString(Xml.Get(), nullptr, Error, sizeof(Error));
	if (!Spec)
	{
		OutError = UTF8_TO_TCHAR(Error);
		return false;
	}
	mjModel* Model = mj_compile(Spec, nullptr);
	if (!Model) OutError = UTF8_TO_TCHAR(mjs_getError(Spec));
	mj_deleteSpec(Spec);
	if (!Model) return false;
	Model->opt.timestep = 0.0;
	const bool bCopied = FMjOwnedModelCandidate::CopyCompiled(Model, OutCandidate,
		OutError, TEXT("invalid native timestep fixture"));
	mj_deleteModel(Model);
	return bCopied;
}

struct FGameWorldFixture
{
	UWorld* World = nullptr;

	bool Create()
	{
		UURLabBridgeSubsystem::SetUnboundForTest(true);
		World = UWorld::CreateWorld(EWorldType::Game, false);
		if (!World || !GEngine) return false;
		FWorldContext& Context = GEngine->CreateNewWorldContext(EWorldType::Game);
		Context.SetCurrentWorld(World);
		return true;
	}

	~FGameWorldFixture()
	{
		if (World)
		{
			if (UMjSimulationWorldSubsystem* Simulation =
				World->GetSubsystem<UMjSimulationWorldSubsystem>())
			{
				Simulation->ShutdownService();
			}
			World->DestroyWorld(false);
			GEngine->DestroyWorldContext(World);
		}
		UURLabBridgeSubsystem::SetUnboundForTest(false);
	}
};
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSimulationSourceManifestTest,
	"URLab.Runtime.WorldCoordinator.StrictPortableSourceManifest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSimulationSourceManifestTest::RunTest(const FString&)
{
	const FString Directory = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("URLabTests"),
		FGuid::NewGuid().ToString(EGuidFormats::Digits));
	IFileManager::Get().MakeDirectory(*Directory, true);
	const FString XmlPath = FPaths::Combine(Directory, TEXT("model.xml"));
	const FString AssetPath = FPaths::Combine(Directory, TEXT("shape.obj"));
	const FString ManifestPath = FPaths::Combine(Directory, TEXT("source.json"));
	FFileHelper::SaveStringToFile(WorldXml, *XmlPath);
	FFileHelper::SaveStringToFile(TEXT("v 0 0 0\n"), *AssetPath);
	FFileHelper::SaveStringToFile(
		TEXT("{\"format\":\"xml\",\"path\":\"model.xml\",\"assets\":{\"mesh/shape.obj\":\"shape.obj\"}}"),
		*ManifestPath);

	FMjModelSourcePayload Payload;
	FString Error;
	TestTrue(TEXT("exact manifest loads"),
		FMjModelSourceLoader::LoadManifest(ManifestPath, Payload, Error));
	TestEqual(TEXT("source kind remains XML"), Payload.Kind,
		EMjModelSourceKind::XmlWithAssets);
	TestEqual(TEXT("primary bytes loaded"), Payload.PrimaryBytes, Utf8Bytes(WorldXml));
	TestEqual(TEXT("VFS asset name retained"), Payload.Assets.Num(), 1);
	TestTrue(TEXT("VFS asset bytes loaded"), Payload.Assets.Contains(TEXT("mesh/shape.obj")));
	TestTrue(TEXT("diagnostic label is resolved source path"), FPaths::IsSamePath(Payload.Label, XmlPath));

	const FString BadManifestPath = FPaths::Combine(Directory, TEXT("bad.json"));
	FFileHelper::SaveStringToFile(
		TEXT("{\"format\":\"xml\",\"path\":\"model.xml\",\"extra\":true}"),
		*BadManifestPath);
	const FString PreviousLabel = Payload.Label;
	TestFalse(TEXT("unknown manifest field rejected"),
		FMjModelSourceLoader::LoadManifest(BadManifestPath, Payload, Error));
	TestEqual(TEXT("failed load preserves prior payload"), Payload.Label, PreviousLabel);
	IFileManager::Get().DeleteDirectory(*Directory, false, true);
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSimulationWorldCoordinatorTransactionTest,
	"URLab.Runtime.WorldCoordinator.ExplicitServiceAndSceneTransaction",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSimulationWorldCoordinatorTransactionTest::RunTest(const FString&)
{
	FGameWorldFixture Fixture;
	if (!TestTrue(TEXT("game world creates"), Fixture.Create())) return false;
	UMjSimulationWorldSubsystem* Simulation =
		Fixture.World->GetSubsystem<UMjSimulationWorldSubsystem>();
	if (!TestNotNull(TEXT("game world owns simulation subsystem"), Simulation)) return false;
	TestFalse(TEXT("subsystem does not activate implicitly"), Simulation->IsRuntimeActive());
	TestFalse(TEXT("subsystem does not attach a bridge implicitly"), Simulation->IsBridgeAttached());
	TestNull(TEXT("resolving subsystem did not start a bridge"),
		UURLabBridgeSubsystem::Find(Fixture.World));

	FString Error;
	if (!TestTrue(TEXT("explicit runtime activation succeeds"), Simulation->ActivateRuntime(Error)))
	{
		AddError(Error);
		return false;
	}
	TestTrue(TEXT("same subsystem activation is idempotent"), Simulation->ActivateRuntime(Error));
	TestTrue(TEXT("runtime is active without RPC"), Simulation->IsRuntimeActive());
	UURLabRenderWorldSubsystem* RenderCoordinator =
		Fixture.World->GetSubsystem<UURLabRenderWorldSubsystem>();
	if (!TestNotNull(TEXT("simulation uses the world render coordinator"),
		RenderCoordinator)) return false;
	FURLabRenderProducerRegistration CompetingProducer;
	TestFalse(TEXT("simulation activation owns the sole render producer registration"),
		RenderCoordinator->RegisterProducer(CompetingProducer, Error));
	TestFalse(TEXT("activation alone has no bridge"), Simulation->IsBridgeAttached());
	TestNull(TEXT("activation alone did not create a bridge"),
		UURLabBridgeSubsystem::Find(Fixture.World));
	TestTrue(TEXT("model-free activation stores freerun intent"),
		Simulation->ConfigureLocalClockMode(EMjSimulationClockMode::FreeRun, Error));

	FMjModelSourcePayload Source;
	Source.Kind = EMjModelSourceKind::XmlWithAssets;
	Source.PrimaryBytes = Utf8Bytes(WorldXml);
	Source.Label = TEXT("world coordinator fixture");
	FMjSimulationPresentation Presentation;
	const FMjSimulationWorldInstallResult First =
		Simulation->InstallSource(MoveTemp(Source), Presentation);
	if (!TestTrue(TEXT("initial source commits"), First.bCommitted))
	{
		AddError(First.Error);
		return false;
	}
	AURLabRenderScene* FirstScene = Simulation->GetActiveScene();
	if (!TestNotNull(TEXT("neutral scene is active"), FirstScene)) return false;
	TestEqual(TEXT("scene revision returned exactly"), Simulation->GetActiveSceneRevision(),
		First.SceneRevision);
	TestEqual(TEXT("scene carries returned revision"), FirstScene->GetScene().SceneRevision,
		First.SceneRevision);
	TestEqual(TEXT("initial frame carries committed epoch"),
		FirstScene->GetAppliedFrame().Identity.ClockEpoch, First.ClockEpoch);
	TestEqual(TEXT("adapter carries returned revision"),
		Simulation->GetActiveAdapter()->GetSceneRevision(), First.SceneRevision);
	TestEqual(TEXT("initial install applies stored clock intent"),
		static_cast<int32>(Simulation->GetRuntime()->GetClockStatus().Mode),
		static_cast<int32>(EMjSimulationClockMode::FreeRun));
	TestTrue(TEXT("local mode can return to stepped without a remote claim"),
		Simulation->ConfigureLocalClockMode(EMjSimulationClockMode::Stepped, Error));

	FString StepError;
	TestTrue(TEXT("stepped runtime mutates independently of RPC"),
		Simulation->GetRuntime()->Step(1, StepError));
	FMjModelObservation SteppedObservation;
	TestTrue(TEXT("stepped observation is available"),
		Simulation->GetRuntime()->GetObservation(SteppedObservation));
	Simulation->Tick(0.0f);
	const FRenderFrame PublishedFrame = FirstScene->GetAppliedFrame();
	TestEqual(TEXT("changed state receives the next scene-local frame id"),
		PublishedFrame.Identity.FrameId, static_cast<uint64>(1));
	TestEqual(TEXT("published frame carries the matching epoch"),
		PublishedFrame.Identity.ClockEpoch, SteppedObservation.ClockEpoch);
	TestEqual(TEXT("published frame carries the matching source time"),
		PublishedFrame.Identity.SourceTime, SteppedObservation.Time);
	Simulation->Tick(0.0f);
	TestEqual(TEXT("unchanged state revision is coalesced"),
		FirstScene->GetAppliedFrame().Identity.FrameId, PublishedFrame.Identity.FrameId);

	FMjSimulationCartesianSnapshot Snapshot;
	TestEqual(TEXT("owned Cartesian snapshot captures"),
		static_cast<int32>(Simulation->GetRuntime()->CaptureCartesianSnapshot(Snapshot)),
		static_cast<int32>(EMjSimulationSnapshotError::None));
	Snapshot.Identity.Model.Generation++;
	FRenderFrame Untouched = PublishedFrame;
	TestFalse(TEXT("adapter rejects a stale model generation"),
		Simulation->GetActiveAdapter()->FrameFromSnapshot(Snapshot, First.Generation,
			99, Untouched, Error));
	TestEqual(TEXT("stale snapshot leaves output frame untouched"),
		Untouched.Identity.FrameId, PublishedFrame.Identity.FrameId);

	FMjOwnedModelCandidate Competing;
	FMjOwnedModelCandidate::FromBytes(EMjModelSourceKind::XmlWithAssets,
		Utf8Bytes(WorldXml), {}, Competing, Error);
	const FMjSimulationWorldInstallResult CompetingResult =
		Simulation->InstallCandidate(MoveTemp(Competing), Presentation);
	TestFalse(TEXT("ordinary initial install cannot overwrite model"), CompetingResult.bCommitted);
	TestTrue(TEXT("initial install directs caller to explicit replacement"),
		CompetingResult.Error.Contains(TEXT("ReplaceCandidate")));

	FMjOwnedModelCandidate InvalidPresentationCandidate;
	FMjOwnedModelCandidate::FromBytes(EMjModelSourceKind::XmlWithAssets,
		Utf8Bytes(WorldXml), {}, InvalidPresentationCandidate, Error);
	FMjSimulationPresentation InvalidPresentation;
	InvalidPresentation.SceneOptions.SceneTransform.SetLocation(
		FVector(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0));
	const FMjSimulationWorldInstallResult FailedReplacement = Simulation->ReplaceCandidate(
		MoveTemp(InvalidPresentationCandidate), InvalidPresentation);
	TestFalse(TEXT("invalid staged presentation rejects replacement"),
		FailedReplacement.bCommitted);
	TestTrue(TEXT("failed replacement preserves active scene"),
		Simulation->GetActiveScene() == FirstScene);
	TestEqual(TEXT("failed replacement preserves runtime generation"),
		Simulation->GetRuntime()->GetModelGeneration(), First.Generation);

	FMjOwnedModelCandidate NativeRejectedCandidate;
	if (!TestTrue(TEXT("native rejection candidate prepares through owned copy"),
		PrepareInvalidTimestepCandidate(NativeRejectedCandidate, Error)))
	{
		AddError(Error);
		return false;
	}
	const FMjSimulationWorldInstallResult NativeRejected = Simulation->ReplaceCandidate(
		MoveTemp(NativeRejectedCandidate), Presentation);
	TestFalse(TEXT("native runtime rejection aborts coordinated replacement"),
		NativeRejected.bCommitted);
	TestTrue(TEXT("native rejection reports the invalid timestep"),
		NativeRejected.Error.Contains(TEXT("timestep")));
	TestTrue(TEXT("native rejection preserves coordinator scene"),
		Simulation->GetActiveScene() == FirstScene);
	TestEqual(TEXT("native rejection preserves scene revision"),
		Simulation->GetActiveSceneRevision(), First.SceneRevision);
	TestEqual(TEXT("native rejection preserves runtime generation"),
		Simulation->GetRuntime()->GetModelGeneration(), First.Generation);

	TestTrue(TEXT("RPC attachment is explicit after local installation"),
		Simulation->AttachBridge(Error));
	UURLabBridgeServer* Bridge = UURLabBridgeSubsystem::Find(Fixture.World);
	if (!TestNotNull(TEXT("attachment creates the world bridge"), Bridge)) return false;
	TestTrue(TEXT("opaque bridge binding is active"), Simulation->IsBridgeAttached());
	Bridge->Stop();
	TestFalse(TEXT("stopped dispatcher invalidates only its registration"),
		Simulation->IsBridgeAttached());
	TestTrue(TEXT("bridge stop preserves active simulation"), Simulation->IsRuntimeActive());
	TestTrue(TEXT("bridge stop preserves neutral scene"), Simulation->GetActiveScene() == FirstScene);
	Bridge->Start(TEXT(""));
	TestFalse(TEXT("replacement dispatcher does not inherit stale registration"),
		Simulation->IsBridgeAttached());
	TestTrue(TEXT("explicit restart attaches the retained runtime to replacement dispatcher"),
		Simulation->AttachBridge(Error));
	TestEqual(TEXT("dispatcher replacement preserves installed runtime"),
		Simulation->GetRuntime()->GetModelGeneration(), First.Generation);
	Simulation->DetachBridge();
	TestFalse(TEXT("exact detach clears only bridge binding"), Simulation->IsBridgeAttached());
	TestEqual(TEXT("exact detach preserves installed runtime"),
		Simulation->GetRuntime()->GetModelGeneration(), First.Generation);
	TestTrue(TEXT("explicit local freerun configuration succeeds"),
		Simulation->ConfigureLocalClockMode(EMjSimulationClockMode::FreeRun, Error));
	FMjModelSourcePayload ReplacementSource;
	ReplacementSource.Kind = EMjModelSourceKind::XmlWithAssets;
	ReplacementSource.PrimaryBytes = Utf8Bytes(WorldXml);
	ReplacementSource.Label = TEXT("replacement world coordinator fixture");
	const FMjSimulationWorldInstallResult Replacement =
		Simulation->ReplaceSource(MoveTemp(ReplacementSource), Presentation);
	TestTrue(TEXT("explicit replacement commits without RPC"), Replacement.bCommitted);
	TestTrue(TEXT("replacement advances model generation"),
		Replacement.Generation > First.Generation);
	TestEqual(TEXT("replacement preserves explicitly selected clock mode"),
		static_cast<int32>(Simulation->GetRuntime()->GetClockStatus().Mode),
		static_cast<int32>(EMjSimulationClockMode::FreeRun));

	Simulation->ShutdownService();
	TestFalse(TEXT("shutdown clears runtime"), Simulation->IsRuntimeActive());
	TestNull(TEXT("shutdown retires neutral scene"), Simulation->GetActiveScene());
	FURLabRenderProducerRegistration ReplacementProducer;
	TestTrue(TEXT("shutdown releases the exact render producer registration"),
		RenderCoordinator->RegisterProducer(ReplacementProducer, Error));
	TestTrue(TEXT("replacement render producer releases"),
		RenderCoordinator->ReleaseProducer(ReplacementProducer, Error));
	const TSharedRef<FMjSimulationRuntime, ESPMode::ThreadSafe> ReplacementRuntime =
		MakeShared<FMjSimulationRuntime, ESPMode::ThreadSafe>();
	FURLabRpcDispatcher::FSimulationRuntimeRegistration ReplacementRegistration;
	TestTrue(TEXT("exact detach permits the next runtime owner"),
		Bridge->GetDispatcher()->AttachSimulationRuntime(ReplacementRuntime,
			ReplacementRegistration, Error));
	TestTrue(TEXT("replacement owner detaches"),
		Bridge->GetDispatcher()->DetachSimulationRuntime(ReplacementRegistration));
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSimulationRenderRegistrationLifetimeTest,
	"URLab.Runtime.WorldCoordinator.RenderRegistrationLifetime",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSimulationRenderRegistrationLifetimeTest::RunTest(const FString&)
{
	FGameWorldFixture Fixture;
	if (!TestTrue(TEXT("game world creates"), Fixture.Create())) return false;
	UMjSimulationWorldSubsystem* Simulation =
		Fixture.World->GetSubsystem<UMjSimulationWorldSubsystem>();
	UURLabRenderWorldSubsystem* Coordinator =
		Fixture.World->GetSubsystem<UURLabRenderWorldSubsystem>();
	if (!TestNotNull(TEXT("simulation subsystem exists"), Simulation)
		|| !TestNotNull(TEXT("render coordinator exists"), Coordinator)) return false;

	FString Error;
	FURLabRenderProducerRegistration BlockingProducer;
	TestTrue(TEXT("independent producer can reserve an inactive world"),
		Coordinator->RegisterProducer(BlockingProducer, Error));
	TestFalse(TEXT("simulation activation refuses competing scene ownership"),
		Simulation->ActivateRuntime(Error));
	TestFalse(TEXT("failed activation creates no simulation runtime"),
		Simulation->IsRuntimeActive());
	TestTrue(TEXT("blocking producer releases exactly"),
		Coordinator->ReleaseProducer(BlockingProducer, Error));
	if (!TestTrue(TEXT("simulation acquires producer when it becomes available"),
		Simulation->ActivateRuntime(Error)))
	{
		AddError(Error);
		return false;
	}

	FMjModelSourcePayload Source;
	Source.Kind = EMjModelSourceKind::XmlWithAssets;
	Source.PrimaryBytes = Utf8Bytes(WorldXml);
	Source.Label = TEXT("render registration lifetime fixture");
	const FMjSimulationWorldInstallResult Installed = Simulation->InstallSource(
		MoveTemp(Source), FMjSimulationPresentation());
	if (!TestTrue(TEXT("source installs through registered coordinator"),
		Installed.bCommitted))
	{
		AddError(Installed.Error);
		return false;
	}
	AURLabRenderScene* InstalledScene = Simulation->GetActiveScene();
	if (!TestNotNull(TEXT("coordinator publishes installed scene"), InstalledScene)) return false;
	const uint64 InstalledGeneration = Simulation->GetRuntime()->GetModelGeneration();

	Coordinator->Deinitialize();
	TestTrue(TEXT("coordinator shutdown retires its owned scene"),
		InstalledScene->IsActorBeingDestroyed());
	TestNull(TEXT("stale registration resolves no simulation scene"),
		Simulation->GetActiveScene());
	TestTrue(TEXT("runtime remains an independent service after scene-owner loss"),
		Simulation->IsRuntimeActive());
	FString StepError;
	TestTrue(TEXT("runtime can still mutate independently"),
		Simulation->GetRuntime()->Step(1, StepError));
	Simulation->Tick(0.0f);
	TestNull(TEXT("tick does not republish through stale registration"),
		Simulation->GetActiveScene());

	FMjOwnedModelCandidate ReplacementCandidate;
	TestTrue(TEXT("replacement candidate prepares"), FMjOwnedModelCandidate::FromBytes(
		EMjModelSourceKind::XmlWithAssets, Utf8Bytes(WorldXml), {}, ReplacementCandidate,
		Error));
	const FMjSimulationWorldInstallResult Replacement = Simulation->ReplaceCandidate(
		MoveTemp(ReplacementCandidate), FMjSimulationPresentation());
	TestFalse(TEXT("stale registration blocks further scene/model publication"),
		Replacement.bCommitted);
	TestEqual(TEXT("stale registration rejection preserves runtime generation"),
		Simulation->GetRuntime()->GetModelGeneration(), InstalledGeneration);
	Simulation->ShutdownService();
	TestFalse(TEXT("shutdown clears runtime after coordinator loss"),
		Simulation->IsRuntimeActive());
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSimulationDeferredShutdownDuringCommitTest,
	"URLab.Runtime.WorldCoordinator.DeferredShutdownDuringSceneRetirement",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSimulationDeferredShutdownDuringCommitTest::RunTest(const FString&)
{
	FGameWorldFixture Fixture;
	if (!TestTrue(TEXT("game world creates"), Fixture.Create())) return false;
	UMjSimulationWorldSubsystem* Simulation =
		Fixture.World->GetSubsystem<UMjSimulationWorldSubsystem>();
	UURLabRenderWorldSubsystem* Coordinator =
		Fixture.World->GetSubsystem<UURLabRenderWorldSubsystem>();
	if (!TestNotNull(TEXT("simulation subsystem exists"), Simulation)
		|| !TestNotNull(TEXT("render coordinator exists"), Coordinator)) return false;
	FString Error;
	if (!TestTrue(TEXT("simulation activates"), Simulation->ActivateRuntime(Error)))
	{
		AddError(Error);
		return false;
	}

	FMjModelSourcePayload InitialSource;
	InitialSource.Kind = EMjModelSourceKind::XmlWithAssets;
	InitialSource.PrimaryBytes = Utf8Bytes(WorldXml);
	InitialSource.Label = TEXT("deferred shutdown initial fixture");
	const FMjSimulationWorldInstallResult Initial = Simulation->InstallSource(
		MoveTemp(InitialSource), FMjSimulationPresentation());
	if (!TestTrue(TEXT("initial source installs"), Initial.bCommitted))
	{
		AddError(Initial.Error);
		return false;
	}
	AURLabRenderScene* InitialScene = Simulation->GetActiveScene();
	if (!TestNotNull(TEXT("initial scene is active"), InitialScene)) return false;

	bool bRetirementCallbackRan = false;
	bool bActivateAcceptedDuringInstall = true;
	FString ReentrantError;
	const FDelegateHandle DestroyHandle = Fixture.World->AddOnActorDestroyedHandler(
		FOnActorDestroyed::FDelegate::CreateLambda([&](AActor* DestroyedActor)
		{
			if (DestroyedActor != InitialScene || bRetirementCallbackRan) return;
			bRetirementCallbackRan = true;
			Simulation->ShutdownService();
			bActivateAcceptedDuringInstall = Simulation->ActivateRuntime(ReentrantError);
		}));
	FMjModelSourcePayload ReplacementSource;
	ReplacementSource.Kind = EMjModelSourceKind::XmlWithAssets;
	ReplacementSource.PrimaryBytes = Utf8Bytes(WorldXml);
	ReplacementSource.Label = TEXT("deferred shutdown replacement fixture");
	const FMjSimulationWorldInstallResult Replacement = Simulation->ReplaceSource(
		MoveTemp(ReplacementSource), FMjSimulationPresentation());
	Fixture.World->RemoveOnActorDestroyedHandler(DestroyHandle);

	TestTrue(TEXT("replacement transaction reaches a consistent commit"),
		Replacement.bCommitted);
	TestTrue(TEXT("old scene retirement callback ran"), bRetirementCallbackRan);
	TestFalse(TEXT("activation is guarded during the install transaction"),
		bActivateAcceptedDuringInstall);
	TestFalse(TEXT("deferred shutdown clears runtime before install returns"),
		Simulation->IsRuntimeActive());
	TestNull(TEXT("deferred shutdown retires the committed scene"),
		Simulation->GetActiveScene());
	FURLabRenderProducerRegistration Successor;
	TestTrue(TEXT("deferred shutdown leaves no orphan render registration"),
		Coordinator->RegisterProducer(Successor, Error));
	TestTrue(TEXT("successor render producer releases"),
		Coordinator->ReleaseProducer(Successor, Error));
	return !HasAnyErrors();
}
#endif
