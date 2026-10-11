// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "Misc/AutomationTest.h"

#include "Engine/Engine.h"
#include "Engine/World.h"
#include "Render/RenderCameraComponent.h"
#include "Render/RenderCameraRig.h"
#include "Render/RenderCameraSubsystem.h"
#include "Render/URLabRenderScene.h"

#if WITH_DEV_AUTOMATION_TESTS
namespace URLabRenderCaptureCoordinatorTest
{
struct FCaptureCoordinatorWorld
{
	UWorld* World = nullptr;
	bool Create()
	{
		World = UWorld::CreateWorld(EWorldType::Game, false);
		if (!World || !GEngine) return false;
		FWorldContext& Context = GEngine->CreateNewWorldContext(EWorldType::Game);
		Context.SetCurrentWorld(World);
		return true;
	}
	~FCaptureCoordinatorWorld()
	{
		if (!World) return;
		World->DestroyWorld(false);
		if (GEngine) GEngine->DestroyWorldContext(World);
	}
};

class FCaptureCoordinatorLease final : public IRenderCameraCaptureLifetimeLease
{
};

void MakeCameraScene(const FString& Revision, uint64 FrameId, double CameraX,
	FRenderScene& OutScene, FRenderSceneResources& OutResources, FRenderFrame& OutFrame)
{
	OutScene.SceneRevision = Revision;
	OutScene.Cameras.AddDefaulted_GetRef().CanonicalName = TEXT("model/exact");
	OutResources.SceneRevision = Revision;
	OutFrame.Identity.SceneRevision = Revision;
	OutFrame.Identity.FrameId = FrameId;
	OutFrame.Identity.SourceTime = static_cast<double>(FrameId);
	OutFrame.Identity.ClockEpoch = 1;
	OutFrame.Identity.bHasSourceTime = true;
	OutFrame.CameraSceneTransforms.Add(FTransform(FVector(CameraX, 0.0, 0.0)));
}

bool InstallCameraScene(UURLabRenderWorldSubsystem& Coordinator,
	const FURLabRenderProducerRegistration& Registration, const FString& Baseline,
	const FString& Revision, FRenderFrame& OutInitial, FString& OutError)
{
	FRenderScene Scene;
	FRenderSceneResources Resources;
	MakeCameraScene(Revision, 1, 10.0, Scene, Resources, OutInitial);
	FURLabRenderSceneCandidate Candidate;
	FRenderSceneInstallOptions Options;
	if (!Coordinator.StageScene(Registration, Baseline, FRenderSceneInstallPayload(MoveTemp(Scene), MoveTemp(Resources), FRenderFrame(OutInitial)), Options, Candidate, OutError))
		return false;
	const FURLabRenderWorldCommitResult Commit =
		Coordinator.CommitCandidate(Registration, Candidate);
	if (!Commit.bCommitted) OutError = Commit.Error;
	return Commit.bCommitted;
}

FURLabRenderCaptureJobRequest MakeRequest(
	URenderCameraSubsystem& Scheduler,
	const FURLabRenderProducerRegistration& Registration,
	const FString& Revision, uint64 FrameId, double CameraX,
	TUniqueFunction<void()> OnTerminal = {})
{
	FURLabRenderCaptureJobRequest Request;
	Request.Producer = Registration;
	Request.ExpectedSceneRevision = Revision;
	Request.PresentationFrame.Identity.SceneRevision = Revision;
	Request.PresentationFrame.Identity.FrameId = FrameId;
	Request.PresentationFrame.Identity.SourceTime = static_cast<double>(FrameId);
	Request.PresentationFrame.Identity.ClockEpoch = 1;
	Request.PresentationFrame.Identity.bHasSourceTime = true;
	Request.PresentationFrame.CameraSceneTransforms.Add(
		FTransform(FVector(CameraX, 0.0, 0.0)));
	FString Error;
	if (!Scheduler.PlanExactCapture(Registration, Revision,
		{TEXT("model/exact")}, Request.Plan, Error))
	{
		return {};
	}
	UURLabRenderWorldSubsystem* Coordinator = Scheduler.GetWorld()
		? Scheduler.GetWorld()->GetSubsystem<UURLabRenderWorldSubsystem>() : nullptr;
	AURLabRenderScene* Scene = Coordinator
		? Coordinator->ResolveActiveScene(Registration) : nullptr;
	if (!Scene || !Scene->PlanGeomPoseOverrides({}, Request.Plan.GeomPoseOverrides,
		Request.Plan.GeometryConfigurationId, Error))
	{
		return {};
	}
	Request.LifetimeLease =
		MakeShared<FCaptureCoordinatorLease, ESPMode::ThreadSafe>();
	Request.OnTerminal = MoveTemp(OnTerminal);
	return Request;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabCaptureCoordinatorHoldCoalescesTest,
	"URLab.Render.CaptureCoordinator.HoldCoalescesAndRestoresAuthoritativeFrame",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabCaptureCoordinatorHoldCoalescesTest::RunTest(const FString&)
{
	using namespace URLabRenderCaptureCoordinatorTest;
	FCaptureCoordinatorWorld Fixture;
	if (!TestTrue(TEXT("isolated world creates"), Fixture.Create())) return false;
	UURLabRenderWorldSubsystem* Coordinator =
		Fixture.World->GetSubsystem<UURLabRenderWorldSubsystem>();
	if (!TestNotNull(TEXT("world coordinator"), Coordinator)) return false;
	FString Error;
	FURLabRenderProducerRegistration Registration;
	if (!TestTrue(TEXT("producer registers"),
		Coordinator->RegisterProducer(Registration, Error))) return false;
	FRenderFrame Initial;
	if (!TestTrue(TEXT("camera scene installs"), InstallCameraScene(*Coordinator,
		Registration, FString(), TEXT("hold-scene"), Initial, Error)))
	{
		AddError(Error);
		return false;
	}

	FRenderFrame JobFrame = Initial;
	JobFrame.Identity.FrameId = 2;
	JobFrame.CameraSceneTransforms[0].SetTranslation(FVector(20.0, 0.0, 0.0));
	FURLabRenderPresentationHold Hold;
	if (!TestTrue(TEXT("exact presentation hold acquires"),
		Coordinator->AcquirePresentationHold(Registration, TEXT("hold-scene"),
			JobFrame, Hold, Error))) return false;
	AURLabRenderScene* Actor = Coordinator->ResolveActiveScene(Registration);
	if (!TestNotNull(TEXT("active scene"), Actor)) return false;
	TestEqual(TEXT("job presentation is applied"), Actor->GetAppliedFrame().Identity.FrameId,
		static_cast<uint64>(2));

	FRenderFrame Authoritative = Initial;
	Authoritative.Identity.FrameId = 3;
	Authoritative.CameraSceneTransforms[0].SetTranslation(FVector(30.0, 0.0, 0.0));
	const FURLabRenderFramePublicationResult Publication =
		Coordinator->PublishAuthoritativeFrame(Registration, TEXT("hold-scene"), Authoritative);
	TestTrue(TEXT("physics publication is accepted"), Publication.bAccepted);
	TestEqual(TEXT("physics publication coalesces"),
		Publication.Disposition, EURLabRenderFramePublication::Coalesced);
	TestEqual(TEXT("hold remains visually stable"), Actor->GetAppliedFrame().Identity.FrameId,
		static_cast<uint64>(2));
	TestTrue(TEXT("exact hold restores newest authoritative frame"),
		Coordinator->ReleasePresentationHold(Registration, Hold, Error));
	TestEqual(TEXT("coalesced authoritative frame is restored"),
		Actor->GetAppliedFrame().Identity.FrameId, static_cast<uint64>(3));
	TestFalse(TEXT("stale hold cannot affect restored scene"),
		Coordinator->ReleasePresentationHold(Registration, Hold, Error));
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabCaptureCoordinatorBoundedFifoTest,
	"URLab.Render.CaptureCoordinator.BoundedFifoAndDeferredCancellationNotification",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabCaptureCoordinatorBoundedFifoTest::RunTest(const FString&)
{
	using namespace URLabRenderCaptureCoordinatorTest;
	FCaptureCoordinatorWorld Fixture;
	if (!TestTrue(TEXT("isolated world creates"), Fixture.Create())) return false;
	UURLabRenderWorldSubsystem* Coordinator =
		Fixture.World->GetSubsystem<UURLabRenderWorldSubsystem>();
	URenderCameraSubsystem* Scheduler =
		Fixture.World->GetSubsystem<URenderCameraSubsystem>();
	if (!TestNotNull(TEXT("world coordinator"), Coordinator)
		|| !TestNotNull(TEXT("camera scheduler"), Scheduler)) return false;
	FString Error;
	FURLabRenderProducerRegistration Registration;
	if (!Coordinator->RegisterProducer(Registration, Error)) return false;
	FRenderFrame Initial;
	if (!InstallCameraScene(*Coordinator, Registration, FString(),
		TEXT("fifo-scene"), Initial, Error)) return false;

	int32 Notifications = 0;
	TArray<FURLabRenderCaptureJob> Jobs;
	for (int32 Index = 0;
		Index < URenderCameraSubsystem::MaxQueuedExactCaptureJobs; ++Index)
	{
		FURLabRenderCaptureJobAdmission Admission = Scheduler->SubmitExactCapture(
			MakeRequest(*Scheduler, Registration, TEXT("fifo-scene"),
				static_cast<uint64>(Index + 2), 20.0 + Index,
				[&Notifications]() { ++Notifications; }));
		if (TestTrue(TEXT("bounded FIFO accepts reserved slot"), Admission.bAccepted))
			Jobs.Add(Admission.Job);
	}
	const FURLabRenderCaptureJobAdmission Overflow = Scheduler->SubmitExactCapture(
		MakeRequest(*Scheduler, Registration, TEXT("fifo-scene"), 100, 100.0));
	TestFalse(TEXT("seventeenth queued job is rejected"), Overflow.bAccepted);
	AURLabRenderScene* Actor = Coordinator->ResolveActiveScene(Registration);
	if (!TestNotNull(TEXT("active scene"), Actor)) return false;
	TestEqual(TEXT("admission and overflow do not mutate presentation"),
		Actor->GetAppliedFrame().Identity.FrameId, static_cast<uint64>(1));

	FRenderScene ReplacementScene;
	FRenderSceneResources ReplacementResources;
	FRenderFrame ReplacementFrame;
	MakeCameraScene(TEXT("fifo-replacement"), 1, 50.0,
		ReplacementScene, ReplacementResources, ReplacementFrame);
	FURLabRenderSceneCandidate Candidate;
	FRenderSceneInstallOptions Options;
	if (!Coordinator->StageScene(Registration, TEXT("fifo-scene"),
		FRenderSceneInstallPayload(MoveTemp(ReplacementScene), MoveTemp(ReplacementResources), FRenderFrame(ReplacementFrame)),
		Options, Candidate, Error)) return false;
	bool bInsideRelatedCommit = false;
	const FURLabRenderWorldCommitResult Rejected = Coordinator->CommitCandidate(
		Registration, Candidate, [&](FString& OutError)
		{
			bInsideRelatedCommit = true;
			TestEqual(TEXT("terminal callbacks do not run in related commit"),
				Notifications, 0);
			OutError = TEXT("fixture rejects related state");
			return false;
		});
	TestTrue(TEXT("related callback ran"), bInsideRelatedCommit);
	TestFalse(TEXT("fixture replacement rejected"), Rejected.bCommitted);
	for (const FURLabRenderCaptureJob& Job : Jobs)
		TestEqual(TEXT("old-scene queued job is explicitly cancelled"),
			Job.PollStatus().State, EURLabRenderCaptureJobState::Cancelled);
	TestEqual(TEXT("callbacks remain deferred after coordinator mutation"),
		Notifications, 0);
	FURLabRenderCaptureJobAdmission LastRetained = Scheduler->SubmitExactCapture(
		MakeRequest(*Scheduler, Registration, TEXT("fifo-scene"), 200, 200.0,
			[&Notifications]() { ++Notifications; }));
	if (TestTrue(TEXT("one final retained record fits the total bound"),
		LastRetained.bAccepted))
	{
		TestTrue(TEXT("final retained job cancels"),
			Scheduler->CancelExactCapture(LastRetained.Job, Error));
	}
	const FURLabRenderCaptureJobAdmission RetainedOverflow = Scheduler->SubmitExactCapture(
		MakeRequest(*Scheduler, Registration, TEXT("fifo-scene"), 201, 201.0));
	TestFalse(TEXT("cancel-submit churn cannot exceed retained record bound"),
		RetainedOverflow.bAccepted);
	Scheduler->Tick(0.0f);
	TestEqual(TEXT("scheduler later drains every terminal notification"),
		Notifications, URenderCameraSubsystem::MaxRetainedExactCaptureJobs);
	TestEqual(TEXT("rejected replacement preserves authoritative presentation"),
		Actor->GetAppliedFrame().Identity.FrameId, static_cast<uint64>(1));
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabCaptureCoordinatorQueuedDeadlineTest,
	"URLab.Render.CaptureCoordinator.QueueDeadlineExpiresWithoutSceneMutation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabCaptureCoordinatorQueuedDeadlineTest::RunTest(const FString&)
{
	using namespace URLabRenderCaptureCoordinatorTest;
	FCaptureCoordinatorWorld Fixture;
	if (!TestTrue(TEXT("isolated world creates"), Fixture.Create())) return false;
	UURLabRenderWorldSubsystem* Coordinator =
		Fixture.World->GetSubsystem<UURLabRenderWorldSubsystem>();
	URenderCameraSubsystem* Scheduler =
		Fixture.World->GetSubsystem<URenderCameraSubsystem>();
	if (!Coordinator || !Scheduler) return false;
	FString Error;
	FURLabRenderProducerRegistration Registration;
	if (!Coordinator->RegisterProducer(Registration, Error)) return false;
	FRenderFrame Initial;
	if (!InstallCameraScene(*Coordinator, Registration, FString(),
		TEXT("deadline-scene"), Initial, Error)) return false;
	Scheduler->SetMonotonicSecondsForTest(100.0);
	FURLabRenderCaptureJobRequest Request =
		MakeRequest(*Scheduler, Registration, TEXT("deadline-scene"), 2, 20.0);
	Request.TimeoutSeconds = 1.0;
	const FURLabRenderCaptureJobAdmission Admission =
		Scheduler->SubmitExactCapture(MoveTemp(Request));
	if (!TestTrue(TEXT("deadline job is admitted"), Admission.bAccepted)) return false;
	Scheduler->SetMonotonicSecondsForTest(101.0);
	Scheduler->Tick(0.0f);
	const FURLabRenderCaptureJobStatus Status = Admission.Job.PollStatus();
	TestEqual(TEXT("deadline produces explicit failure"),
		Status.State, EURLabRenderCaptureJobState::Failed);
	TestTrue(TEXT("deadline failure is already drained"), Status.bGpuWritersDrained);
	AURLabRenderScene* Actor = Coordinator->ResolveActiveScene(Registration);
	if (TestNotNull(TEXT("active scene"), Actor))
		TestEqual(TEXT("expired queue entry never mutates presentation"),
			Actor->GetAppliedFrame().Identity.FrameId, static_cast<uint64>(1));
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabCaptureCoordinatorExactWorldAdmissionTest,
	"URLab.Render.CaptureCoordinator.ExactRevisionAndWorldAdmission",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabCaptureCoordinatorExactWorldAdmissionTest::RunTest(const FString&)
{
	using namespace URLabRenderCaptureCoordinatorTest;
	FCaptureCoordinatorWorld SourceFixture;
	FCaptureCoordinatorWorld OtherFixture;
	if (!TestTrue(TEXT("source world creates"), SourceFixture.Create())
		|| !TestTrue(TEXT("other world creates"), OtherFixture.Create())) return false;
	UURLabRenderWorldSubsystem* Coordinator =
		SourceFixture.World->GetSubsystem<UURLabRenderWorldSubsystem>();
	URenderCameraSubsystem* SourceScheduler =
		SourceFixture.World->GetSubsystem<URenderCameraSubsystem>();
	URenderCameraSubsystem* OtherScheduler =
		OtherFixture.World->GetSubsystem<URenderCameraSubsystem>();
	if (!Coordinator || !SourceScheduler || !OtherScheduler) return false;
	FString Error;
	FURLabRenderProducerRegistration Registration;
	if (!Coordinator->RegisterProducer(Registration, Error)) return false;
	FRenderFrame Initial;
	if (!InstallCameraScene(*Coordinator, Registration, FString(),
		TEXT("world-bound-scene"), Initial, Error)) return false;

	FURLabRenderCaptureJobRequest EmptyRevision =
		MakeRequest(*SourceScheduler, Registration, TEXT("world-bound-scene"), 2, 20.0);
	EmptyRevision.ExpectedSceneRevision.Reset();
	EmptyRevision.PresentationFrame.Identity.SceneRevision.Reset();
	TestFalse(TEXT("exact job refuses a wildcard revision"),
		SourceScheduler->SubmitExactCapture(MoveTemp(EmptyRevision)).bAccepted);
	TestFalse(TEXT("foreign world scheduler refuses a valid producer registration"),
		OtherScheduler->SubmitExactCapture(MakeRequest(*SourceScheduler, Registration,
			TEXT("world-bound-scene"), 3, 30.0)).bAccepted);
	AURLabRenderScene* Actor = Coordinator->ResolveActiveScene(Registration);
	if (TestNotNull(TEXT("source scene remains active"), Actor))
		TestEqual(TEXT("rejected admission leaves source presentation unchanged"),
			Actor->GetAppliedFrame().Identity.FrameId, static_cast<uint64>(1));
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabCapturePlanAdmissionValidationTest,
	"URLab.Render.CaptureCoordinator.PlanRejectsStaleConfigurationAndTampering",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabCapturePlanAdmissionValidationTest::RunTest(const FString&)
{
	using namespace URLabRenderCaptureCoordinatorTest;
	FCaptureCoordinatorWorld Fixture;
	if (!TestTrue(TEXT("isolated world creates"), Fixture.Create())) return false;
	UURLabRenderWorldSubsystem* Coordinator =
		Fixture.World->GetSubsystem<UURLabRenderWorldSubsystem>();
	URenderCameraSubsystem* Scheduler =
		Fixture.World->GetSubsystem<URenderCameraSubsystem>();
	if (!Coordinator || !Scheduler) return false;
	FString Error;
	FURLabRenderProducerRegistration Registration;
	if (!Coordinator->RegisterProducer(Registration, Error)) return false;
	FRenderFrame Initial;
	if (!InstallCameraScene(*Coordinator, Registration, FString(),
		TEXT("plan-admission-scene"), Initial, Error)) return false;
	AURLabRenderScene* Scene = Coordinator->ResolveActiveScene(Registration);
	URenderCameraComponent* Camera = Scene && Scene->GetCameraRig()
		? Scene->GetCameraRig()->GetCamera(0) : nullptr;
	if (!TestNotNull(TEXT("planned camera resolves"), Camera)) return false;

	FURLabRenderCaptureJobRequest Stale = MakeRequest(*Scheduler, Registration,
		TEXT("plan-admission-scene"), 2, 20.0);
	Camera->SetCaptureRate(false, 17.0f);
	const FURLabRenderCaptureJobAdmission StaleAdmission =
		Scheduler->SubmitExactCapture(MoveTemp(Stale));
	TestFalse(TEXT("configuration change rejects a stale plan at admission"),
		StaleAdmission.bAccepted);
	TestFalse(TEXT("stale-plan rejection explains the failure"),
		StaleAdmission.Error.IsEmpty());
	TestEqual(TEXT("stale-plan rejection does not apply presentation"),
		Scene->GetAppliedFrame().Identity.FrameId, static_cast<uint64>(1));

	FURLabRenderCaptureJobRequest Tampered = MakeRequest(*Scheduler, Registration,
		TEXT("plan-admission-scene"), 3, 30.0);
	Tampered.Plan.AggregateLogicalPixelBytes += 1;
	const FURLabRenderCaptureJobAdmission TamperedAdmission =
		Scheduler->SubmitExactCapture(MoveTemp(Tampered));
	TestFalse(TEXT("tampered aggregate byte bounds are rejected"),
		TamperedAdmission.bAccepted);
	TestFalse(TEXT("tampered-plan rejection explains the failure"),
		TamperedAdmission.Error.IsEmpty());
	TestEqual(TEXT("tampered-plan rejection does not apply presentation"),
		Scene->GetAppliedFrame().Identity.FrameId, static_cast<uint64>(1));
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabCapturePlanQueuedConfigurationTest,
	"URLab.Render.CaptureCoordinator.RetryPlanChangeFailsBeforeIssue",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabCapturePlanQueuedConfigurationTest::RunTest(const FString&)
{
	using namespace URLabRenderCaptureCoordinatorTest;
	FCaptureCoordinatorWorld Fixture;
	if (!TestTrue(TEXT("isolated world creates"), Fixture.Create())) return false;
	UURLabRenderWorldSubsystem* Coordinator =
		Fixture.World->GetSubsystem<UURLabRenderWorldSubsystem>();
	URenderCameraSubsystem* Scheduler =
		Fixture.World->GetSubsystem<URenderCameraSubsystem>();
	if (!Coordinator || !Scheduler) return false;
	FString Error;
	FURLabRenderProducerRegistration Registration;
	if (!Coordinator->RegisterProducer(Registration, Error)) return false;
	FRenderFrame Initial;
	if (!InstallCameraScene(*Coordinator, Registration, FString(),
		TEXT("plan-queued-scene"), Initial, Error)) return false;
	AURLabRenderScene* Scene = Coordinator->ResolveActiveScene(Registration);
	URenderCameraComponent* Camera = Scene && Scene->GetCameraRig()
		? Scene->GetCameraRig()->GetCamera(0) : nullptr;
	if (!TestNotNull(TEXT("queued camera resolves"), Camera)) return false;
	Camera->SetStreamingEnabled(false);
	const FURLabRenderCaptureJobAdmission Admission = Scheduler->SubmitExactCapture(
		MakeRequest(*Scheduler, Registration, TEXT("plan-queued-scene"), 2, 20.0));
	if (!TestTrue(TEXT("fresh plan queues"), Admission.bAccepted)) return false;
	Scheduler->Tick(0.0f);
	TestEqual(TEXT("cold render target leaves the job retrying"),
		Admission.Job.PollStatus().State, EURLabRenderCaptureJobState::Running);
	Camera->SetCaptureRate(false, 23.0f);
	Scheduler->Tick(0.0f);
	const FURLabRenderCaptureJobStatus Status = Admission.Job.PollStatus();
	TestEqual(TEXT("configuration change during retry fails before issue"), Status.State,
		EURLabRenderCaptureJobState::Failed);
	TestFalse(TEXT("queued configuration failure is explicit"), Status.Message.IsEmpty());
	TestTrue(TEXT("no GPU allocation remains to drain"), Status.bGpuWritersDrained);
	TestEqual(TEXT("stale queued plan never applies presentation"),
		Scene->GetAppliedFrame().Identity.FrameId, static_cast<uint64>(1));
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabRenderResourceReadinessGateTest,
	"URLab.Render.CaptureCoordinator.ResourceReadinessGatesLiveAndExactIssue",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabRenderResourceReadinessGateTest::RunTest(const FString&)
{
	using namespace URLabRenderCaptureCoordinatorTest;
	FCaptureCoordinatorWorld Fixture;
	if (!TestTrue(TEXT("isolated world creates"), Fixture.Create())) return false;
	UURLabRenderWorldSubsystem* Coordinator =
		Fixture.World->GetSubsystem<UURLabRenderWorldSubsystem>();
	URenderCameraSubsystem* Scheduler =
		Fixture.World->GetSubsystem<URenderCameraSubsystem>();
	if (!Coordinator || !Scheduler) return false;
	FString Error;
	FURLabRenderProducerRegistration Registration;
	if (!Coordinator->RegisterProducer(Registration, Error)) return false;
	FRenderFrame Initial;
	if (!InstallCameraScene(*Coordinator, Registration, FString(),
		TEXT("resource-readiness-scene"), Initial, Error)) return false;
	AURLabRenderScene* Scene = Coordinator->ResolveActiveScene(Registration);
	URenderCameraComponent* Camera = Scene && Scene->GetCameraRig()
		? Scene->GetCameraRig()->GetCamera(0) : nullptr;
	if (!TestNotNull(TEXT("active scene"), Scene)
		|| !TestNotNull(TEXT("active camera"), Camera)) return false;

	Scene->SetRenderResourcesReadyForTest(false);
	Camera->SetStreamingEnabled(false);
	Camera->bContinuousCapture = true;
	Camera->bManualCaptureOnly = false;
	Scheduler->Tick(0.0f);
	TestFalse(TEXT("live camera stays inactive while scene resources are pending"),
		Camera->IsStreamingActive());

	const FURLabRenderCaptureJobAdmission Admission = Scheduler->SubmitExactCapture(
		MakeRequest(*Scheduler, Registration, TEXT("resource-readiness-scene"), 2, 20.0));
	if (!TestTrue(TEXT("exact job is admitted while resources are pending"),
		Admission.bAccepted)) return false;
	Scheduler->Tick(0.0f);
	const FURLabRenderCaptureJobStatus Pending = Admission.Job.PollStatus();
	TestEqual(TEXT("exact job remains pending under its existing running state"),
		Pending.State, EURLabRenderCaptureJobState::Running);
	TestTrue(TEXT("pending readiness does not create a GPU writer"),
		Pending.bGpuWritersDrained);
	TestFalse(TEXT("exact setup does not bypass the live readiness gate"),
		Camera->IsStreamingActive());

	TestTrue(TEXT("pending exact job remains cancellable"),
		Scheduler->CancelExactCapture(Admission.Job, Error));
	Scheduler->Tick(0.0f);
	Scene->SetRenderResourcesReadyForTest(true);
	Scheduler->Tick(0.0f);
	TestTrue(TEXT("live camera activates after scene resources become ready"),
		Camera->IsStreamingActive());
	Camera->bContinuousCapture = false;
	Camera->SetStreamingEnabled(false);
	Scene->SetRenderResourcesReadyForTest(TOptional<bool>());
	return !HasAnyErrors();
}

#endif
