// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "Misc/AutomationTest.h"

#include "Bridge/BridgeSubsystem.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "MuJoCo/Core/MjModelSourcePayload.h"
#include "MuJoCo/Render/MjRenderKinematics.h"
#include "MuJoCo/Runtime/MjSimulationWorldSubsystem.h"
#include "Render/RenderCameraSubsystem.h"
#include "Render/RenderWorldSubsystem.h"
#include "Render/URLabRenderScene.h"
#include "Workflows/RenderServerWorldSubsystem.h"

#include <atomic>

#if WITH_DEV_AUTOMATION_TESTS
namespace URLabRenderServerWorldSubsystemTest
{
const char* KinematicsXml = R"(
<mujoco model="render_server_workflow">
  <worldbody>
    <body name="slider">
      <joint name="slide" type="slide" axis="1 0 0"/>
      <geom name="box" type="box" size="0.1 0.1 0.1" mass="1"/>
	  <camera name="exact" pos="0 0 0.25"/>
    </body>
  </worldbody>
</mujoco>)";

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
			if (UURLabRenderServerWorldSubsystem* Workflow =
				World->GetSubsystem<UURLabRenderServerWorldSubsystem>())
			{
				Workflow->ShutdownWorkflow();
			}
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

struct FCountingLease final : IRenderCameraCaptureLifetimeLease
{
	explicit FCountingLease(
		TSharedRef<std::atomic<int32>, ESPMode::ThreadSafe> InLiveCount)
		: LiveCount(MoveTemp(InLiveCount))
	{
		LiveCount->fetch_add(1, std::memory_order_relaxed);
	}

	virtual ~FCountingLease() override
	{
		LiveCount->fetch_sub(1, std::memory_order_relaxed);
	}

	TSharedRef<std::atomic<int32>, ESPMode::ThreadSafe> LiveCount;
};

FRenderCameraCaptureLifetimeLeasePtr MakeLease(
	const TSharedRef<std::atomic<int32>, ESPMode::ThreadSafe>& LiveCount)
{
	return MakeShared<FCountingLease, ESPMode::ThreadSafe>(LiveCount);
}

FURLabRenderOwnerSessionScopePtr MakeScope(const TCHAR* Label)
{
	FString Error;
	return FURLabRenderOwnerSessionScope::Create(Label, Error);
}

void MakeNeutralScene(const FString& Revision, double X, FRenderScene& OutScene,
	FRenderSceneResources& OutResources, FRenderFrame& OutFrame)
{
	OutScene.SceneRevision = Revision;
	OutScene.Bodies.AddDefaulted_GetRef().Name = TEXT("neutral_body");
	OutScene.Cameras.AddDefaulted_GetRef().CanonicalName = TEXT("model/exact");
	OutResources.SceneRevision = Revision;
	OutFrame.Identity.SceneRevision = Revision;
	OutFrame.Identity.FrameId = 1;
	OutFrame.BodySceneTransforms.Add(FTransform(FVector(X, 0.0, 0.0)));
	OutFrame.CameraSceneTransforms.Add(FTransform(FVector(0.0, X, 0.0)));
}

AURLabRenderScene* FindScene(UWorld& World, const FString& Revision)
{
	for (TActorIterator<AURLabRenderScene> It(&World); It; ++It)
	{
		if (It->GetScene().SceneRevision == Revision) return *It;
	}
	return nullptr;
}

bool PrepareKinematics(const FString& Revision,
	FMjRenderKinematicsCandidate& OutCandidate, FString& OutError)
{
	FMjModelSourcePayload Source;
	Source.Kind = EMjModelSourceKind::XmlWithAssets;
	Source.PrimaryBytes.Append(reinterpret_cast<const uint8*>(KinematicsXml),
		FCStringAnsi::Strlen(KinematicsXml));
	Source.Label = TEXT("render-server workflow test XML");
	return FMjRenderKinematicsCandidate::Prepare(MoveTemp(Source), Revision,
		FMjRenderKinematicsOptions(), OutCandidate, OutError);
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabRenderServerActivationTest,
	"URLab.Workflows.RenderServerWorld.ActivationAndProducerConflict",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabRenderServerActivationTest::RunTest(const FString&)
{
	using namespace URLabRenderServerWorldSubsystemTest;
	FGameWorldFixture Fixture;
	if (!TestTrue(TEXT("isolated game world creates"), Fixture.Create())) return false;
	UURLabRenderServerWorldSubsystem* Workflow =
		Fixture.World->GetSubsystem<UURLabRenderServerWorldSubsystem>();
	UMjSimulationWorldSubsystem* Simulation =
		Fixture.World->GetSubsystem<UMjSimulationWorldSubsystem>();
	if (!TestNotNull(TEXT("world automatically registers render-server subsystem"), Workflow)
		|| !TestNotNull(TEXT("world owns simulation subsystem"), Simulation)) return false;

	TestFalse(TEXT("subsystem construction is inert"), Workflow->IsActive());
	const FURLabRenderOwnerStatusResult InertStatus = Workflow->OwnerStatus();
	TestEqual(TEXT("inert workflow reports unavailable authority"), InertStatus.Error,
		EURLabRenderOwnerAuthorityError::Unavailable);
	TestFalse(TEXT("inert workflow does not advertise owner availability"),
		InertStatus.Status.bAvailable);
	TestFalse(TEXT("owner cannot attach before activation"),
		Workflow->AttachOwner(MakeScope(TEXT("early-owner"))).IsOk());
	FString Error;
	if (!TestTrue(TEXT("simulation claims the neutral producer"),
		Simulation->ActivateRuntime(Error)))
	{
		AddError(Error);
		return false;
	}
	TestFalse(TEXT("render-server activation refuses the existing producer"),
		Workflow->Activate(Error));
	TestTrue(TEXT("failed activation leaves workflow inert"), !Workflow->IsActive());
	TestTrue(TEXT("producer conflict does not disturb simulation"),
		Simulation->IsRuntimeActive());
	Simulation->ShutdownService();
	UURLabRenderWorldSubsystem* Coordinator =
		Fixture.World->GetSubsystem<UURLabRenderWorldSubsystem>();
	FURLabRenderProducerRegistration ForeignProducer;
	if (!TestTrue(TEXT("independent coordinator producer registers"), Coordinator
		&& Coordinator->RegisterProducer(ForeignProducer, Error))) return false;
	TestFalse(TEXT("render-server also refuses a non-simulation producer"),
		Workflow->Activate(Error));
	TestTrue(TEXT("foreign producer remains current after conflict"),
		Coordinator->IsRegistrationCurrent(ForeignProducer));
	TestTrue(TEXT("foreign producer releases exactly"),
		Coordinator->ReleaseProducer(ForeignProducer, Error));
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabRenderServerNeutralLifecycleTest,
	"URLab.Workflows.RenderServerWorld.NeutralOwnerLifecycle",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabRenderServerNeutralLifecycleTest::RunTest(const FString&)
{
	using namespace URLabRenderServerWorldSubsystemTest;
	FGameWorldFixture Fixture;
	if (!TestTrue(TEXT("isolated game world creates"), Fixture.Create())) return false;
	UURLabRenderServerWorldSubsystem* Workflow =
		Fixture.World->GetSubsystem<UURLabRenderServerWorldSubsystem>();
	if (!TestNotNull(TEXT("render-server subsystem resolves"), Workflow)) return false;
	FString Error;
	if (!TestTrue(TEXT("explicit activation claims producer"), Workflow->Activate(Error)))
	{
		AddError(Error);
		return false;
	}
	auto Owner = MakeScope(TEXT("neutral-owner"));
	const FURLabRenderOwnerAttachResult Attached = Workflow->AttachOwner(Owner);
	if (!TestTrue(TEXT("owner attaches"), Attached.IsOk())) return false;

	FRenderScene Scene;
	FRenderSceneResources Resources;
	FRenderFrame Initial;
	MakeNeutralScene(TEXT("neutral-a"), 1.0, Scene, Resources, Initial);
	const FURLabRenderServerInstallResult Installed = Workflow->InstallNeutralScene(
		Attached.Proof, FString(), FRenderSceneInstallPayload(MoveTemp(Scene), MoveTemp(Resources), FRenderFrame(Initial)),
		FRenderSceneInstallOptions());
	if (!TestTrue(TEXT("neutral source installs"), Installed.bCommitted))
	{
		AddError(Installed.Error);
		return false;
	}
	AURLabRenderScene* Visible = FindScene(*Fixture.World, TEXT("neutral-a"));
	if (!TestNotNull(TEXT("installed scene is visible"), Visible)) return false;
	FURLabRenderSourceDescription Description;
	if (!TestTrue(TEXT("neutral source describes for exact owner"),
		Workflow->DescribeSource(Attached.Proof, Description, Error))) return false;
	TestEqual(TEXT("description carries exact revision"), Description.SceneRevision,
		Installed.SceneRevision);
	TestEqual(TEXT("description carries exact generation"), Description.SceneGeneration,
		Installed.SceneGeneration);
	TestEqual(TEXT("description preserves body order"), Description.BodyNames.Num(), 1);
	TestEqual(TEXT("description preserves model camera order"),
		Description.ModelCameras.Num(), 1);
	TestFalse(TEXT("neutral source does not advertise qpos"), Description.bSupportsQpos);
	FRenderFrame Published = Initial;
	Published.Identity.FrameId = 2;
	Published.BodySceneTransforms[0].SetTranslation(FVector(3.0, 0.0, 0.0));
	TestTrue(TEXT("Cartesian source publishes"), Workflow->PublishCartesian(
		Attached.Proof, Installed.SceneRevision, Published).bAccepted);
	TestEqual(TEXT("published frame reaches visible scene"),
		Visible->GetAppliedFrame().Identity.FrameId, uint64(2));

	FRenderFrame BadScene;
	FRenderScene BadDescription;
	FRenderSceneResources BadResources;
	MakeNeutralScene(TEXT("neutral-b"), 8.0, BadDescription, BadResources, BadScene);
	BadScene.Identity.SceneRevision = TEXT("wrong-revision");
	const FURLabRenderServerInstallResult Failed = Workflow->InstallNeutralScene(
		Attached.Proof, Installed.SceneRevision, FRenderSceneInstallPayload(MoveTemp(BadDescription), MoveTemp(BadResources), FRenderFrame(BadScene)), FRenderSceneInstallOptions());
	TestFalse(TEXT("invalid replacement fails"), Failed.bCommitted);
	TestTrue(TEXT("failed replacement preserves visible source"),
		FindScene(*Fixture.World, TEXT("neutral-a")) == Visible && !Visible->IsHidden());
	Published.Identity.FrameId = 3;
	TestTrue(TEXT("failed replacement preserves source authority"),
		Workflow->PublishCartesian(Attached.Proof, Installed.SceneRevision,
			Published).bAccepted);
	TestFalse(TEXT("stale revision cannot publish"), Workflow->PublishCartesian(
		Attached.Proof, TEXT("neutral-stale"), Published).bAccepted);

	Owner->Deactivate();
	Workflow->Tick(0.0f);
	TestTrue(TEXT("retirement leaves the last scene visible and stale"),
		FindScene(*Fixture.World, Installed.SceneRevision) == Visible && !Visible->IsHidden());
	TestFalse(TEXT("retired proof cannot publish"), Workflow->PublishCartesian(
		Attached.Proof, Installed.SceneRevision, Published).bAccepted);
	auto Successor = MakeScope(TEXT("neutral-successor"));
	const FURLabRenderOwnerAttachResult SuccessorAttach = Workflow->AttachOwner(Successor);
	if (!TestTrue(TEXT("successor attaches after retirement barrier"),
		SuccessorAttach.IsOk())) return false;
	TestFalse(TEXT("successor cannot publish before installing its source"),
		Workflow->PublishCartesian(SuccessorAttach.Proof, Installed.SceneRevision,
			Published).bAccepted);

	FRenderScene SuccessorScene;
	FRenderSceneResources SuccessorResources;
	FRenderFrame SuccessorInitial;
	MakeNeutralScene(TEXT("neutral-successor"), 12.0, SuccessorScene,
		SuccessorResources, SuccessorInitial);
	const FURLabRenderServerInstallResult SuccessorInstalled =
		Workflow->InstallNeutralScene(SuccessorAttach.Proof, Installed.SceneRevision,
			FRenderSceneInstallPayload(MoveTemp(SuccessorScene), MoveTemp(SuccessorResources), FRenderFrame(SuccessorInitial)),
			FRenderSceneInstallOptions());
	TestTrue(TEXT("successor explicitly replaces the stale scene"),
		SuccessorInstalled.bCommitted);
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabRenderServerKinematicsTest,
	"URLab.Workflows.RenderServerWorld.PreparedKinematicsAndQpos",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabRenderServerKinematicsTest::RunTest(const FString&)
{
	using namespace URLabRenderServerWorldSubsystemTest;
	FGameWorldFixture Fixture;
	if (!TestTrue(TEXT("isolated game world creates"), Fixture.Create())) return false;
	UURLabRenderServerWorldSubsystem* Workflow =
		Fixture.World->GetSubsystem<UURLabRenderServerWorldSubsystem>();
	FString Error;
	if (!TestTrue(TEXT("workflow activates"), Workflow && Workflow->Activate(Error)))
	{
		AddError(Error);
		return false;
	}
	auto Owner = MakeScope(TEXT("qpos-owner"));
	const FURLabRenderOwnerAttachResult Attached = Workflow->AttachOwner(Owner);
	if (!TestTrue(TEXT("qpos owner attaches"), Attached.IsOk())) return false;
	FMjRenderKinematicsCandidate Candidate;
	if (!TestTrue(TEXT("prepared MuJoCo candidate builds"),
		PrepareKinematics(TEXT("qpos-scene"), Candidate, Error)))
	{
		AddError(Error);
		return false;
	}
	const FURLabRenderServerInstallResult Installed = Workflow->InstallKinematicsCandidate(
		Attached.Proof, FString(), MoveTemp(Candidate), FRenderSceneInstallOptions());
	if (!TestTrue(TEXT("prepared MuJoCo candidate installs"), Installed.bCommitted))
	{
		AddError(Installed.Error);
		return false;
	}
	AURLabRenderScene* Visible = FindScene(*Fixture.World, Installed.SceneRevision);
	if (!TestNotNull(TEXT("kinematics scene is visible"), Visible)) return false;
	FURLabRenderSourceDescription Description;
	if (!TestTrue(TEXT("kinematics source describes for exact owner"),
		Workflow->DescribeSource(Attached.Proof, Description, Error))) return false;
	TestTrue(TEXT("kinematics source advertises qpos"), Description.bSupportsQpos);
	TestEqual(TEXT("kinematics description exposes exact qpos width"),
		Description.QposWidth, 1);
	const TArray<double> Qpos = {0.75};
	const FURLabRenderFramePublicationResult Published = Workflow->PublishQpos(
		Attached.Proof, Installed.SceneRevision, Qpos, 9, 4.5, true, 3);
	TestTrue(TEXT("qpos publishes through render-only FK"), Published.bAccepted);
	TestEqual(TEXT("qpos frame identity reaches scene"),
		Visible->GetAppliedFrame().Identity.FrameId, uint64(9));
	TestTrue(TEXT("qpos moved the model body"),
		Visible->GetAppliedFrame().BodySceneTransforms.IsValidIndex(1)
		&& !FMath::IsNearlyZero(Visible->GetAppliedFrame()
			.BodySceneTransforms[1].GetTranslation().X));
	FRenderScene InvalidScene;
	FRenderSceneResources InvalidResources;
	FRenderFrame InvalidFrame;
	MakeNeutralScene(TEXT("invalid-after-fk"), 2.0, InvalidScene,
		InvalidResources, InvalidFrame);
	InvalidFrame.Identity.SceneRevision = TEXT("mismatched-frame");
	TestFalse(TEXT("failed replacement does not commit over FK source"),
		Workflow->InstallNeutralScene(Attached.Proof, Installed.SceneRevision,
			FRenderSceneInstallPayload(MoveTemp(InvalidScene), MoveTemp(InvalidResources), FRenderFrame(InvalidFrame)),
			FRenderSceneInstallOptions()).bCommitted);
	TestTrue(TEXT("failed replacement preserves installed FK"), Workflow->PublishQpos(
		Attached.Proof, Installed.SceneRevision, Qpos, 10, 5.0, true, 3).bAccepted);
	TestEqual(TEXT("preserved FK publication reaches original scene"),
		Visible->GetAppliedFrame().Identity.FrameId, uint64(10));
	TestFalse(TEXT("stale owner revision rejects qpos"), Workflow->PublishQpos(
		Attached.Proof, TEXT("stale-qpos"), Qpos, 11, 5.5, true, 3).bAccepted);
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabRenderServerDeferredShutdownTest,
	"URLab.Workflows.RenderServerWorld.DeferredShutdownDuringActorRetirement",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabRenderServerDeferredShutdownTest::RunTest(const FString&)
{
	using namespace URLabRenderServerWorldSubsystemTest;
	FGameWorldFixture Fixture;
	if (!TestTrue(TEXT("isolated game world creates"), Fixture.Create())) return false;
	UURLabRenderServerWorldSubsystem* Workflow =
		Fixture.World->GetSubsystem<UURLabRenderServerWorldSubsystem>();
	UURLabRenderWorldSubsystem* Coordinator =
		Fixture.World->GetSubsystem<UURLabRenderWorldSubsystem>();
	FString Error;
	if (!TestTrue(TEXT("workflow activates"), Workflow && Coordinator
		&& Workflow->Activate(Error)))
	{
		AddError(Error);
		return false;
	}
	auto Owner = MakeScope(TEXT("shutdown-owner"));
	const FURLabRenderOwnerAttachResult Attached = Workflow->AttachOwner(Owner);
	if (!TestTrue(TEXT("shutdown owner attaches"), Attached.IsOk())) return false;
	FRenderScene FirstScene;
	FRenderSceneResources FirstResources;
	FRenderFrame FirstFrame;
	MakeNeutralScene(TEXT("shutdown-a"), 1.0, FirstScene, FirstResources, FirstFrame);
	const FURLabRenderServerInstallResult First = Workflow->InstallNeutralScene(
		Attached.Proof, FString(), FRenderSceneInstallPayload(MoveTemp(FirstScene), MoveTemp(FirstResources), FRenderFrame(FirstFrame)), FRenderSceneInstallOptions());
	if (!TestTrue(TEXT("initial shutdown fixture installs"), First.bCommitted)) return false;
	AURLabRenderScene* FirstActor = FindScene(*Fixture.World, First.SceneRevision);
	if (!TestNotNull(TEXT("initial shutdown actor exists"), FirstActor)) return false;

	bool bRetirementCallbackRan = false;
	bool bStillActiveInCallback = false;
	const FDelegateHandle DestroyHandle = Fixture.World->AddOnActorDestroyedHandler(
		FOnActorDestroyed::FDelegate::CreateLambda([&](AActor* DestroyedActor)
		{
			if (DestroyedActor != FirstActor || bRetirementCallbackRan) return;
			bRetirementCallbackRan = true;
			Workflow->ShutdownWorkflow();
			bStillActiveInCallback = Workflow->IsActive();
		}));
	FRenderScene ReplacementScene;
	FRenderSceneResources ReplacementResources;
	FRenderFrame ReplacementFrame;
	MakeNeutralScene(TEXT("shutdown-b"), 2.0, ReplacementScene,
		ReplacementResources, ReplacementFrame);
	const FURLabRenderServerInstallResult Replacement = Workflow->InstallNeutralScene(
		Attached.Proof, First.SceneRevision, FRenderSceneInstallPayload(MoveTemp(ReplacementScene), MoveTemp(ReplacementResources), FRenderFrame(ReplacementFrame)), FRenderSceneInstallOptions());
	Fixture.World->RemoveOnActorDestroyedHandler(DestroyHandle);

	TestTrue(TEXT("replacement commit finishes before deferred shutdown"),
		Replacement.bCommitted);
	TestTrue(TEXT("actor retirement callback requested shutdown"),
		bRetirementCallbackRan);
	TestTrue(TEXT("producer is retained inside retirement callback"),
		bStillActiveInCallback);
	TestFalse(TEXT("deferred shutdown completes after coordinator mutation"),
		Workflow->IsActive());
	FURLabRenderProducerRegistration AfterShutdown;
	TestTrue(TEXT("exact workflow producer was released"),
		Coordinator->RegisterProducer(AfterShutdown, Error));
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabRenderServerCaptureAdmissionTest,
	"URLab.Workflows.RenderServerWorld.CaptureAdmissionIdentityCapacityAndOwnership",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabRenderServerCaptureAdmissionTest::RunTest(const FString&)
{
	using namespace URLabRenderServerWorldSubsystemTest;
	FGameWorldFixture Fixture;
	if (!TestTrue(TEXT("isolated game world creates"), Fixture.Create())) return false;
	UURLabRenderServerWorldSubsystem* Workflow =
		Fixture.World->GetSubsystem<UURLabRenderServerWorldSubsystem>();
	URenderCameraSubsystem* Scheduler =
		Fixture.World->GetSubsystem<URenderCameraSubsystem>();
	if (!TestNotNull(TEXT("render-server subsystem resolves"), Workflow)
		|| !TestNotNull(TEXT("camera scheduler resolves"), Scheduler)) return false;
	const FRenderFrame InertFrame;
	TestFalse(TEXT("capture submission is inert before activation"),
		Workflow->SubmitCartesianCapture(FURLabRenderOwnerProof(), FString(),
			FRenderFrame(InertFrame), {}, {}, {}, 1.0).bAccepted);

	FString Error;
	if (!TestTrue(TEXT("workflow activates"), Workflow->Activate(Error))) return false;
	auto Owner = MakeScope(TEXT("capture-owner"));
	const FURLabRenderOwnerAttachResult Attached = Workflow->AttachOwner(Owner);
	if (!TestTrue(TEXT("capture owner attaches"), Attached.IsOk())) return false;
	FMjRenderKinematicsCandidate Candidate;
	if (!TestTrue(TEXT("capture kinematics prepares"),
		PrepareKinematics(TEXT("capture-scene"), Candidate, Error)))
	{
		AddError(Error);
		return false;
	}
	const FURLabRenderServerInstallResult Installed = Workflow->InstallKinematicsCandidate(
		Attached.Proof, FString(), MoveTemp(Candidate), FRenderSceneInstallOptions());
	if (!TestTrue(TEXT("capture kinematics installs"), Installed.bCommitted)) return false;
	AURLabRenderScene* Visible = FindScene(*Fixture.World, Installed.SceneRevision);
	if (!TestNotNull(TEXT("capture scene is visible"), Visible)) return false;
	if (!TestTrue(TEXT("prepared scene exposes its model camera"),
		Visible->GetScene().Cameras.Num() == 1)) return false;
	const FString CameraName = Visible->GetScene().Cameras[0].CanonicalName;
	FURLabRenderExactCapturePlan CapturePlan;
	if (!TestTrue(TEXT("capture plan resolves"), Workflow->PlanCapture(Attached.Proof,
		Installed.SceneRevision, {CameraName}, CapturePlan, Error))) return false;
	FRenderFrame Cartesian = Visible->GetAppliedFrame();
	Cartesian.Identity.FrameId = 50;
	Cartesian.Identity.SourceTime = 5.0;
	Cartesian.Identity.ClockEpoch = 4;
	Cartesian.Identity.bHasSourceTime = true;

	auto LiveLeases = MakeShared<std::atomic<int32>, ESPMode::ThreadSafe>(0);
	FRenderCameraCaptureLifetimeLeasePtr MissingResultPixel = MakeLease(LiveLeases);
	TestFalse(TEXT("both capacity leases are required"),
		Workflow->SubmitCartesianCapture(Attached.Proof, Installed.SceneRevision,
			FRenderFrame(Cartesian), CapturePlan, MoveTemp(MissingResultPixel), {}, 1.0)
			.bAccepted);
	TestEqual(TEXT("rejected missing-lease admission rolls back pixel capacity"),
		LiveLeases->load(std::memory_order_relaxed), 0);
	TestFalse(TEXT("invalid timeout is rejected before scheduler admission"),
		Workflow->SubmitCartesianCapture(Attached.Proof, Installed.SceneRevision,
			FRenderFrame(Cartesian), CapturePlan, MakeLease(LiveLeases),
			MakeLease(LiveLeases), -1.0).bAccepted);
	TestEqual(TEXT("invalid timeout rolls back both leases"),
		LiveLeases->load(std::memory_order_relaxed), 0);
	TestFalse(TEXT("empty camera selection is rejected"),
		Workflow->SubmitCartesianCapture(Attached.Proof, Installed.SceneRevision,
			FRenderFrame(Cartesian), {}, MakeLease(LiveLeases), MakeLease(LiveLeases), 1.0)
			.bAccepted);
	TestFalse(TEXT("stale capture revision is rejected"),
		Workflow->SubmitCartesianCapture(Attached.Proof, TEXT("stale-capture"),
			FRenderFrame(Cartesian), CapturePlan, MakeLease(LiveLeases),
			MakeLease(LiveLeases), 1.0).bAccepted);
	TestEqual(TEXT("invalid selection and revision roll back capacity"),
		LiveLeases->load(std::memory_order_relaxed), 0);

	const FURLabRenderServerCaptureAdmission CartesianAdmission =
		Workflow->SubmitCartesianCapture(Attached.Proof, Installed.SceneRevision,
			FRenderFrame(Cartesian), CapturePlan, MakeLease(LiveLeases),
			MakeLease(LiveLeases), 1.0);
	const TArray<double> Qpos = {0.25};
	const FURLabRenderServerCaptureAdmission QposAdmission = Workflow->SubmitQposCapture(
		Attached.Proof, Installed.SceneRevision, Qpos, 51, 5.1, true, 4,
		CapturePlan, MakeLease(LiveLeases), MakeLease(LiveLeases), 1.0);
	if (!TestTrue(TEXT("Cartesian capture queues"), CartesianAdmission.bAccepted)
		|| !TestTrue(TEXT("qpos capture queues"), QposAdmission.bAccepted)) return false;
	TestTrue(TEXT("workflow handles are distinct"),
		CartesianAdmission.Handle.IsValid() && QposAdmission.Handle.IsValid()
		&& CartesianAdmission.Handle.GetSubmittedFrameIdentity().FrameId
			!= QposAdmission.Handle.GetSubmittedFrameIdentity().FrameId);
	TestEqual(TEXT("Cartesian submitted identity is retained"),
		CartesianAdmission.Handle.GetSubmittedFrameIdentity().FrameId, uint64(50));
	TestEqual(TEXT("qpos submitted identity is retained"),
		QposAdmission.Handle.GetSubmittedFrameIdentity().FrameId, uint64(51));
	TestEqual(TEXT("accepted jobs retain pixel and result capacity"),
		LiveLeases->load(std::memory_order_relaxed), 4);

	FURLabRenderCaptureJobSnapshot Snapshot;
	TestFalse(TEXT("foreign proof cannot poll a retained job"), Workflow->PollCapture(
		FURLabRenderOwnerProof(), CartesianAdmission.Handle, Snapshot, Error));
	TestFalse(TEXT("foreign proof cannot cancel a retained job"), Workflow->CancelCapture(
		FURLabRenderOwnerProof(), CartesianAdmission.Handle, Error));
	TestFalse(TEXT("foreign proof cannot release a retained job"), Workflow->ReleaseCapture(
		FURLabRenderOwnerProof(), CartesianAdmission.Handle, Error));
	TestFalse(TEXT("nonterminal job cannot be released"), Workflow->ReleaseCapture(
		Attached.Proof, CartesianAdmission.Handle, Error));
	TestTrue(TEXT("exact owner cancels Cartesian job"), Workflow->CancelCapture(
		Attached.Proof, CartesianAdmission.Handle, Error));
	TestTrue(TEXT("exact owner cancels qpos job"), Workflow->CancelCapture(
		Attached.Proof, QposAdmission.Handle, Error));
	Scheduler->Tick(0.0f);
	TestTrue(TEXT("cancelled job remains truthfully pollable"), Workflow->PollCapture(
		Attached.Proof, CartesianAdmission.Handle, Snapshot, Error));
	TestEqual(TEXT("poll reports scheduler cancellation"), Snapshot.State,
		EURLabRenderCaptureJobState::Cancelled);
	TestEqual(TEXT("cancelled jobs retain result capacity after pixel drain"),
		LiveLeases->load(std::memory_order_relaxed), 2);
	TestTrue(TEXT("terminal Cartesian record releases"), Workflow->ReleaseCapture(
		Attached.Proof, CartesianAdmission.Handle, Error));
	TestTrue(TEXT("terminal qpos record releases"), Workflow->ReleaseCapture(
		Attached.Proof, QposAdmission.Handle, Error));
	TestEqual(TEXT("explicit release frees retained result capacity"),
		LiveLeases->load(std::memory_order_relaxed), 0);
	TestFalse(TEXT("released handle is stale"), Workflow->PollCapture(
		Attached.Proof, CartesianAdmission.Handle, Snapshot, Error));

	TArray<FURLabRenderServerCaptureHandle> Retained;
	for (int32 Index = 0;
		Index < UURLabRenderServerWorldSubsystem::MaxRetainedCaptureJobs; ++Index)
	{
		Cartesian.Identity.FrameId = static_cast<uint64>(100 + Index);
		const FURLabRenderServerCaptureAdmission Admission =
			Workflow->SubmitCartesianCapture(Attached.Proof, Installed.SceneRevision,
				FRenderFrame(Cartesian), CapturePlan, MakeLease(LiveLeases),
				MakeLease(LiveLeases), 1.0);
		if (!TestTrue(TEXT("workflow retained-record slot accepts"),
			Admission.bAccepted)) return false;
		Retained.Add(Admission.Handle);
		if (!TestTrue(TEXT("queued capacity fixture cancels"), Workflow->CancelCapture(
			Attached.Proof, Admission.Handle, Error))) return false;
		Scheduler->Tick(0.0f);
	}
	TestEqual(TEXT("terminal records retain one result lease each"),
		LiveLeases->load(std::memory_order_relaxed),
		UURLabRenderServerWorldSubsystem::MaxRetainedCaptureJobs);
	Cartesian.Identity.FrameId = 999;
	TestFalse(TEXT("thirty-third retained record is rejected"),
		Workflow->SubmitCartesianCapture(Attached.Proof, Installed.SceneRevision,
			FRenderFrame(Cartesian), CapturePlan, MakeLease(LiveLeases),
			MakeLease(LiveLeases), 1.0).bAccepted);
	TestEqual(TEXT("record-capacity rejection rolls back both leases"),
		LiveLeases->load(std::memory_order_relaxed),
		UURLabRenderServerWorldSubsystem::MaxRetainedCaptureJobs);
	FURLabRenderHistoryQuery FullHistoryQuery;
	FullHistoryQuery.ExpectedSceneRevision = Installed.SceneRevision;
	const FURLabRenderServerCaptureAdmission FullHistoryAdmission =
		Workflow->QueryCaptureHistory(Attached.Proof, FullHistoryQuery,
			MakeLease(LiveLeases));
	TestFalse(TEXT("history cannot exceed retained record capacity"),
		FullHistoryAdmission.bAccepted);
	TestTrue(TEXT("history reports retained record capacity"),
		FullHistoryAdmission.Error.Contains(TEXT("capacity")));
	TestEqual(TEXT("rejected history releases its result capacity"),
		LiveLeases->load(std::memory_order_relaxed),
		UURLabRenderServerWorldSubsystem::MaxRetainedCaptureJobs);
	for (const FURLabRenderServerCaptureHandle& Handle : Retained)
		TestTrue(TEXT("terminal retained record releases"),
			Workflow->ReleaseCapture(Attached.Proof, Handle, Error));
	TestEqual(TEXT("releasing every retained result frees capacity"),
		LiveLeases->load(std::memory_order_relaxed), 0);
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabRenderServerCaptureRetirementTest,
	"URLab.Workflows.RenderServerWorld.CaptureReplacementAndOwnerRetirement",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabRenderServerCaptureRetirementTest::RunTest(const FString&)
{
	using namespace URLabRenderServerWorldSubsystemTest;
	FGameWorldFixture Fixture;
	if (!TestTrue(TEXT("isolated game world creates"), Fixture.Create())) return false;
	UURLabRenderServerWorldSubsystem* Workflow =
		Fixture.World->GetSubsystem<UURLabRenderServerWorldSubsystem>();
	URenderCameraSubsystem* Scheduler =
		Fixture.World->GetSubsystem<URenderCameraSubsystem>();
	FString Error;
	if (!TestTrue(TEXT("workflow activates"), Workflow && Scheduler
		&& Workflow->Activate(Error))) return false;
	auto Owner = MakeScope(TEXT("retiring-capture-owner"));
	const FURLabRenderOwnerAttachResult Attached = Workflow->AttachOwner(Owner);
	if (!TestTrue(TEXT("capture owner attaches"), Attached.IsOk())) return false;
	FRenderScene Scene;
	FRenderSceneResources Resources;
	FRenderFrame Initial;
	MakeNeutralScene(TEXT("capture-retire-a"), 1.0, Scene, Resources, Initial);
	const FURLabRenderServerInstallResult Installed = Workflow->InstallNeutralScene(
		Attached.Proof, FString(), FRenderSceneInstallPayload(MoveTemp(Scene), MoveTemp(Resources), FRenderFrame(Initial)),
		FRenderSceneInstallOptions());
	if (!TestTrue(TEXT("initial capture scene installs"), Installed.bCommitted)) return false;
	FURLabRenderExactCapturePlan InitialPlan;
	if (!TestTrue(TEXT("initial capture plan resolves"), Workflow->PlanCapture(
		Attached.Proof, Installed.SceneRevision, {TEXT("model/exact")}, InitialPlan,
		Error))) return false;
	auto LiveLeases = MakeShared<std::atomic<int32>, ESPMode::ThreadSafe>(0);
	FRenderFrame JobFrame = Initial;
	JobFrame.Identity.FrameId = 20;
	JobFrame.Identity.SourceTime = 2.0;
	JobFrame.Identity.ClockEpoch = 1;
	JobFrame.Identity.bHasSourceTime = true;
	const FURLabRenderServerCaptureAdmission BeforeReplacement =
		Workflow->SubmitCartesianCapture(Attached.Proof, Installed.SceneRevision,
			MoveTemp(JobFrame), InitialPlan, MakeLease(LiveLeases),
			MakeLease(LiveLeases), 1.0);
	if (!TestTrue(TEXT("old-scene capture queues"), BeforeReplacement.bAccepted))
		return false;

	FRenderScene ReplacementScene;
	FRenderSceneResources ReplacementResources;
	FRenderFrame ReplacementFrame;
	MakeNeutralScene(TEXT("capture-retire-b"), 2.0, ReplacementScene,
		ReplacementResources, ReplacementFrame);
	const FURLabRenderServerInstallResult Replacement = Workflow->InstallNeutralScene(
		Attached.Proof, Installed.SceneRevision, FRenderSceneInstallPayload(MoveTemp(ReplacementScene), MoveTemp(ReplacementResources), FRenderFrame(ReplacementFrame)), FRenderSceneInstallOptions());
	if (!TestTrue(TEXT("scene replacement commits"), Replacement.bCommitted)) return false;
	FURLabRenderExactCapturePlan ReplacementPlan;
	if (!TestTrue(TEXT("replacement capture plan resolves"), Workflow->PlanCapture(
		Attached.Proof, Replacement.SceneRevision, {TEXT("model/exact")},
		ReplacementPlan, Error))) return false;
	FURLabRenderCaptureJobSnapshot Snapshot;
	TestTrue(TEXT("same owner can collect old-scene cancellation"), Workflow->PollCapture(
		Attached.Proof, BeforeReplacement.Handle, Snapshot, Error));
	TestEqual(TEXT("replacement cancelled queued old-scene job"), Snapshot.State,
		EURLabRenderCaptureJobState::Cancelled);
	Scheduler->Tick(0.0f);
	TestEqual(TEXT("replacement cancellation retains its result lease after drain"),
		LiveLeases->load(std::memory_order_relaxed), 1);
	TestTrue(TEXT("same owner releases cancelled old-scene record"),
		Workflow->ReleaseCapture(Attached.Proof, BeforeReplacement.Handle, Error));
	TestEqual(TEXT("explicit release frees old-scene result capacity"),
		LiveLeases->load(std::memory_order_relaxed), 0);

	ReplacementFrame.Identity.FrameId = 30;
	ReplacementFrame.Identity.SourceTime = 3.0;
	ReplacementFrame.Identity.ClockEpoch = 1;
	ReplacementFrame.Identity.bHasSourceTime = true;
	const FURLabRenderServerCaptureAdmission BeforeRetirement =
		Workflow->SubmitCartesianCapture(Attached.Proof, Replacement.SceneRevision,
			FRenderFrame(ReplacementFrame), ReplacementPlan, MakeLease(LiveLeases),
			MakeLease(LiveLeases), 1.0);
	if (!TestTrue(TEXT("replacement-scene capture queues"), BeforeRetirement.bAccepted))
		return false;
	Owner->Deactivate();
	Workflow->Tick(0.0f);
	Scheduler->Tick(0.0f);
	TestEqual(TEXT("owner retirement releases result and drained pixel leases"),
		LiveLeases->load(std::memory_order_relaxed), 0);
	TestFalse(TEXT("retired owner record is removed"), Workflow->PollCapture(
		Attached.Proof, BeforeRetirement.Handle, Snapshot, Error));

	auto Successor = MakeScope(TEXT("capture-successor"));
	const FURLabRenderOwnerAttachResult SuccessorAttach = Workflow->AttachOwner(Successor);
	if (!TestTrue(TEXT("successor attaches after capture retirement"),
		SuccessorAttach.IsOk())) return false;
	TestFalse(TEXT("successor cannot access predecessor handle"), Workflow->PollCapture(
		SuccessorAttach.Proof, BeforeRetirement.Handle, Snapshot, Error));
	TestFalse(TEXT("successor cannot submit before installing a source"),
		Workflow->SubmitCartesianCapture(SuccessorAttach.Proof,
			Replacement.SceneRevision, FRenderFrame(ReplacementFrame), ReplacementPlan,
			MakeLease(LiveLeases), MakeLease(LiveLeases), 1.0).bAccepted);
	TestEqual(TEXT("successor pre-install rejection rolls back capacity"),
		LiveLeases->load(std::memory_order_relaxed), 0);

	FRenderScene SuccessorScene;
	FRenderSceneResources SuccessorResources;
	FRenderFrame SuccessorFrame;
	MakeNeutralScene(TEXT("capture-successor-scene"), 3.0, SuccessorScene,
		SuccessorResources, SuccessorFrame);
	const FURLabRenderServerInstallResult SuccessorInstalled =
		Workflow->InstallNeutralScene(SuccessorAttach.Proof, Replacement.SceneRevision,
			FRenderSceneInstallPayload(MoveTemp(SuccessorScene), MoveTemp(SuccessorResources), FRenderFrame(SuccessorFrame)),
			FRenderSceneInstallOptions());
	if (!TestTrue(TEXT("successor installs its own source"),
		SuccessorInstalled.bCommitted)) return false;
	FURLabRenderExactCapturePlan SuccessorPlan;
	if (!TestTrue(TEXT("successor capture plan resolves"), Workflow->PlanCapture(
		SuccessorAttach.Proof, SuccessorInstalled.SceneRevision,
		{TEXT("model/exact")}, SuccessorPlan, Error))) return false;
	SuccessorFrame.Identity.FrameId = 40;
	SuccessorFrame.Identity.SourceTime = 4.0;
	SuccessorFrame.Identity.ClockEpoch = 2;
	SuccessorFrame.Identity.bHasSourceTime = true;
	const FURLabRenderServerCaptureAdmission SuccessorJob =
		Workflow->SubmitCartesianCapture(SuccessorAttach.Proof,
			SuccessorInstalled.SceneRevision, MoveTemp(SuccessorFrame), SuccessorPlan,
			MakeLease(LiveLeases), MakeLease(LiveLeases), 1.0);
	TestTrue(TEXT("successor submits only after source installation"),
		SuccessorJob.bAccepted);
	if (SuccessorJob.bAccepted)
	{
		TestTrue(TEXT("successor job cancels"), Workflow->CancelCapture(
			SuccessorAttach.Proof, SuccessorJob.Handle, Error));
		Scheduler->Tick(0.0f);
		TestTrue(TEXT("successor terminal record releases"), Workflow->ReleaseCapture(
			SuccessorAttach.Proof, SuccessorJob.Handle, Error));
	}
	TestEqual(TEXT("successor cleanup releases both leases"),
		LiveLeases->load(std::memory_order_relaxed), 0);
	return !HasAnyErrors();
}

#endif
