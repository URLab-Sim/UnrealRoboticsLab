// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "Misc/AutomationTest.h"

#include "Editor.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/World.h"
#include "HAL/PlatformTime.h"
#include "Misc/App.h"
#include "RHIGlobals.h"
#include "Render/RenderCameraSubsystem.h"
#include "Render/URLabRenderScene.h"

#if WITH_EDITOR && WITH_DEV_AUTOMATION_TESTS
namespace URLabRenderCaptureSchedulerGpuTest
{
constexpr int32 Width = 192;
constexpr int32 Height = 128;

class FSchedulerGpuLease final : public IRenderCameraCaptureLifetimeLease
{
};

FRenderFrame MakeFrame(const FString& Revision, uint64 FrameId, double BodyY)
{
	FRenderFrame Frame;
	Frame.Identity.SceneRevision = Revision;
	Frame.Identity.FrameId = FrameId;
	Frame.Identity.SourceTime = static_cast<double>(FrameId) * 0.1;
	Frame.Identity.ClockEpoch = 4;
	Frame.Identity.bHasSourceTime = true;
	Frame.BodySceneTransforms.Add(FTransform(FVector(300.0, BodyY, 0.0)));
	Frame.CameraSceneTransforms.Add(FTransform::Identity);
	return Frame;
}

void MakeScene(const FString& Revision, FRenderScene& OutScene,
	FRenderSceneResources& OutResources)
{
	OutScene.SceneRevision = Revision;
	OutResources.SceneRevision = Revision;
	OutScene.Bodies.AddDefaulted_GetRef().Name = TEXT("scheduler_box_body");
	FRenderMaterialResource& Material =
		OutResources.Materials.AddDefaulted_GetRef();
	Material.Emission = 3.0f;
	Material.Roughness = 1.0f;
	FRenderGeomDesc& Box = OutScene.Geoms.AddDefaulted_GetRef();
	Box.BodyIndex = 0;
	Box.Kind = ERenderGeomKind::Box;
	Box.SizeCm = FVector(40.0, 40.0, 40.0);
	Box.Name = TEXT("scheduler_red_box");
	FRenderSurfaceBinding& Surface = OutResources.Geoms.AddDefaulted_GetRef();
	Surface.MaterialIndex = 0;
	Surface.Rgba = FLinearColor(1.0f, 0.01f, 0.01f, 1.0f);
	FRenderCameraDesc& Camera = OutScene.Cameras.AddDefaulted_GetRef();
	Camera.CanonicalName = TEXT("scheduler/camera");
	Camera.Optics.Resolution = FIntPoint(Width, Height);
	Camera.Optics.Projection = ERenderCameraProjection::Orthographic;
	Camera.Optics.OrthographicWidthCm = 400.0f;
	Camera.Optics.NearClipCm = 2.0f;
	Camera.Optics.FarClipCm = 1000.0f;
	Camera.Capture.Mode = ERenderCameraMode::Real;
	Camera.Capture.bManualCaptureOnly = true;
	Camera.Capture.HistoryCapacity = 4;
}

bool StageAndCommit(UURLabRenderWorldSubsystem& Coordinator,
	const FURLabRenderProducerRegistration& Registration, const FString& Baseline,
	const FString& Revision, const FRenderFrame& InitialFrame, FString& OutError)
{
	FRenderScene Scene;
	FRenderSceneResources Resources;
	MakeScene(Revision, Scene, Resources);
	FRenderSceneInstallOptions Options;
	Options.bForceRebuildAssets = true;
	FURLabRenderSceneCandidate Candidate;
	if (!Coordinator.StageScene(Registration, Baseline, FRenderSceneInstallPayload(MoveTemp(Scene), MoveTemp(Resources), FRenderFrame(InitialFrame)), Options, Candidate, OutError))
		return false;
	const FURLabRenderWorldCommitResult Commit =
		Coordinator.CommitCandidate(Registration, Candidate);
	if (!Commit.bCommitted) OutError = Commit.Error;
	return Commit.bCommitted;
}

FURLabRenderCaptureJobRequest MakeRequest(
	URenderCameraSubsystem& Scheduler,
	const FURLabRenderProducerRegistration& Registration,
	const FString& Revision, uint64 FrameId, double BodyY)
{
	FURLabRenderCaptureJobRequest Request;
	Request.Producer = Registration;
	Request.ExpectedSceneRevision = Revision;
	Request.PresentationFrame = MakeFrame(Revision, FrameId, BodyY);
	FString Error;
	if (!Scheduler.PlanExactCapture(Registration, Revision,
		{TEXT("scheduler/camera")}, Request.Plan, Error))
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
	Request.TimeoutSeconds = 20.0;
	Request.LifetimeLease = MakeShared<FSchedulerGpuLease, ESPMode::ThreadSafe>();
	return Request;
}

struct FSchedulerGpuState
{
	FAutomationTestBase* Test = nullptr;
	UWorld* World = nullptr;
	UURLabRenderWorldSubsystem* Coordinator = nullptr;
	URenderCameraSubsystem* Scheduler = nullptr;
	FURLabRenderProducerRegistration Registration;
	FURLabRenderCaptureJob First;
	FURLabRenderCaptureJob Second;
	FURLabRenderCaptureJob Cancelled;
	FURLabRenderSceneCandidate ReplacementCandidate;
	double StartedSeconds = 0.0;
	int32 Phase = 0;
	bool bReadinessReleased = false;
	bool bCleaned = false;

	void Cleanup()
	{
		if (bCleaned) return;
		bCleaned = true;
		First = {};
		Second = {};
		Cancelled = {};
		if (Coordinator && Coordinator->IsRegistrationCurrent(Registration))
		{
			FString Error;
			Coordinator->ReleaseProducer(Registration, Error);
		}
	}
	~FSchedulerGpuState() { Cleanup(); }
};

struct FRedMetrics
{
	int32 Count = 0;
	double XSum = 0.0;
	double Centroid() const { return Count > 0 ? XSum / Count : -1.0; }
};

FRedMetrics MeasureRed(const FRenderCameraFrame& Frame)
{
	FRedMetrics Result;
	for (int32 Index = 0; Index < Frame.Color.Num(); ++Index)
	{
		const FColor& Pixel = Frame.Color[Index];
		if (Pixel.R > 80 && Pixel.R > Pixel.G + 30 && Pixel.R > Pixel.B + 30)
		{
			++Result.Count;
			Result.XSum += Index % Frame.Width;
		}
	}
	return Result;
}
}

DEFINE_LATENT_AUTOMATION_COMMAND_ONE_PARAMETER(
	FURLabPumpRenderCaptureSchedulerGpu,
	TSharedPtr<URLabRenderCaptureSchedulerGpuTest::FSchedulerGpuState>, State);

bool FURLabPumpRenderCaptureSchedulerGpu::Update()
{
	using namespace URLabRenderCaptureSchedulerGpuTest;
	if (!State.IsValid() || !State->Test || !State->Scheduler)
		return true;
	FAutomationTestBase& Test = *State->Test;
	if (FPlatformTime::Seconds() - State->StartedSeconds > 25.0)
	{
		Test.AddError(TEXT("scheduler GPU acceptance exceeded its bounded deadline"));
		State->Cleanup();
		return true;
	}
	if (!State->bReadinessReleased)
	{
		State->Scheduler->Tick(0.0f);
		const FURLabRenderCaptureJobStatus Pending = State->First.PollStatus();
		Test.TestEqual(TEXT("cold-resource exact job remains running"), Pending.State,
			EURLabRenderCaptureJobState::Running);
		Test.TestTrue(TEXT("cold-resource gate issues no GPU writer"),
			Pending.bGpuWritersDrained);
		AURLabRenderScene* Scene =
			State->Coordinator->ResolveActiveScene(State->Registration);
		if (!Test.TestNotNull(TEXT("cold-resource scene remains active"), Scene))
		{
			State->Cleanup();
			return true;
		}
		Scene->SetRenderResourcesReadyForTest(TOptional<bool>());
		State->bReadinessReleased = true;
		return false;
	}

	if (State->Phase == 0)
	{
		State->Scheduler->Tick(0.0f);
		const FURLabRenderCaptureJobStatus FirstStatus = State->First.PollStatus();
		const FURLabRenderCaptureJobStatus SecondStatus = State->Second.PollStatus();
		if (!FirstStatus.IsTerminal() || !SecondStatus.IsTerminal()) return false;
		Test.AddInfo(FString::Printf(
			TEXT("FIFO terminals: first=%d drained=%d camera='%s' message='%s'; "
				"second=%d drained=%d camera='%s' message='%s'"),
			static_cast<int32>(FirstStatus.State), FirstStatus.bGpuWritersDrained ? 1 : 0,
			*FirstStatus.AffectedCamera, *FirstStatus.Message,
			static_cast<int32>(SecondStatus.State), SecondStatus.bGpuWritersDrained ? 1 : 0,
			*SecondStatus.AffectedCamera, *SecondStatus.Message));
		if (!Test.TestEqual(TEXT("first FIFO job succeeds"), FirstStatus.State,
			EURLabRenderCaptureJobState::Succeeded)
			|| !Test.TestEqual(TEXT("second FIFO job succeeds"), SecondStatus.State,
				EURLabRenderCaptureJobState::Succeeded))
		{
			State->Cleanup();
			return true;
		}
		const FURLabRenderCaptureJobSnapshot FirstResult = State->First.Poll();
		const FURLabRenderCaptureJobSnapshot SecondResult = State->Second.Poll();
		if (!Test.TestEqual(TEXT("first job returns exactly one camera"), FirstResult.Frames.Num(), 1)
			|| !Test.TestEqual(TEXT("second job returns exactly one camera"), SecondResult.Frames.Num(), 1)
			|| !FirstResult.Frames[0].IsValid() || !SecondResult.Frames[0].IsValid())
		{
			Test.AddError(TEXT("successful scheduler jobs did not retain exact owned frames"));
			State->Cleanup();
			return true;
		}
		const FRenderCameraFrame& FirstFrame = *FirstResult.Frames[0];
		const FRenderCameraFrame& SecondFrame = *SecondResult.Frames[0];
		Test.TestEqual(TEXT("first result keeps exact source frame"), FirstFrame.Identity.FrameId,
			static_cast<uint64>(11));
		Test.TestEqual(TEXT("second result keeps exact source frame"), SecondFrame.Identity.FrameId,
			static_cast<uint64>(12));
		Test.TestEqual(TEXT("first result keeps exact scene revision"),
			FirstFrame.Identity.SceneRevision, FString(TEXT("scheduler-gpu-a")));
		Test.TestEqual(TEXT("second result keeps exact scene revision"),
			SecondFrame.Identity.SceneRevision, FString(TEXT("scheduler-gpu-a")));
		Test.TestTrue(TEXT("first result keeps exact source clock"),
			FirstFrame.Identity.bHasSourceTime && FirstFrame.Identity.ClockEpoch == 4
				&& FMath::IsNearlyEqual(FirstFrame.Identity.SourceTime, 1.1, 1e-9));
		Test.TestTrue(TEXT("second result keeps exact source clock"),
			SecondFrame.Identity.bHasSourceTime && SecondFrame.Identity.ClockEpoch == 4
				&& FMath::IsNearlyEqual(SecondFrame.Identity.SourceTime, 1.2, 1e-9));
		Test.TestTrue(TEXT("FIFO captures have distinct increasing camera serials"),
			FirstFrame.CaptureSerial > 0
				&& SecondFrame.CaptureSerial > FirstFrame.CaptureSerial);
		Test.TestTrue(TEXT("both frames retain the exact stable camera configuration"),
			FirstFrame.CameraConfigurationRevision > 0
				&& SecondFrame.CameraConfigurationRevision
					== FirstFrame.CameraConfigurationRevision);
		Test.TestEqual(TEXT("first exact result has configured pixel count"),
			FirstFrame.Color.Num(), Width * Height);
		Test.TestEqual(TEXT("second exact result has configured pixel count"),
			SecondFrame.Color.Num(), Width * Height);
		Test.TestTrue(TEXT("first pixels retain workflow capacity"),
			FirstFrame.PixelLifetimeLease.IsValid());
		Test.TestTrue(TEXT("second pixels retain workflow capacity"),
			SecondFrame.PixelLifetimeLease.IsValid());
		const FRedMetrics FirstRed = MeasureRed(FirstFrame);
		const FRedMetrics SecondRed = MeasureRed(SecondFrame);
		Test.AddInfo(FString::Printf(TEXT("scheduler GPU red pixels: first=%d x=%.2f second=%d x=%.2f"),
			FirstRed.Count, FirstRed.Centroid(), SecondRed.Count, SecondRed.Centroid()));
		Test.TestTrue(TEXT("first exact capture contains production red pixels"),
			FirstRed.Count > 100);
		Test.TestTrue(TEXT("second exact capture contains production red pixels"),
			SecondRed.Count > 100);
		Test.TestTrue(TEXT("distinct Cartesian frames move the red image region"),
			FMath::Abs(FirstRed.Centroid() - SecondRed.Centroid()) > 20.0);
		AURLabRenderScene* Active =
			State->Coordinator->ResolveActiveScene(State->Registration);
		if (!Test.TestNotNull(TEXT("scene remains active after FIFO"), Active))
		{
			State->Cleanup();
			return true;
		}
		Test.TestEqual(TEXT("FIFO completion restores authoritative presentation"),
			Active->GetAppliedFrame().Identity.FrameId, static_cast<uint64>(10));

		FRenderScene ReplacementScene;
		FRenderSceneResources ReplacementResources;
		MakeScene(TEXT("scheduler-gpu-b"), ReplacementScene, ReplacementResources);
		FRenderSceneInstallOptions ReplacementOptions;
		ReplacementOptions.bForceRebuildAssets = true;
		const FRenderFrame ReplacementInitial =
			MakeFrame(TEXT("scheduler-gpu-b"), 20, 0.0);
		FString StageError;
		if (!Test.TestTrue(TEXT("replacement stages before the cancellation writer"),
			State->Coordinator->StageScene(State->Registration, TEXT("scheduler-gpu-a"),
				FRenderSceneInstallPayload(MoveTemp(ReplacementScene), MoveTemp(ReplacementResources), FRenderFrame(ReplacementInitial)), ReplacementOptions, State->ReplacementCandidate,
				StageError)))
		{
			Test.AddError(StageError);
			State->Cleanup();
			return true;
		}
		const FURLabRenderCaptureJobAdmission CancelAdmission =
			State->Scheduler->SubmitExactCapture(MakeRequest(*State->Scheduler, State->Registration,
				TEXT("scheduler-gpu-a"), 13, 0.0));
		if (!Test.TestTrue(TEXT("cancellation fixture job is admitted"),
			CancelAdmission.bAccepted))
		{
			State->Cleanup();
			return true;
		}
		State->Cancelled = CancelAdmission.Job;
		State->Phase = 1;
		// The first two captures warmed the target. This tick starts the third
		// job and can issue its writer, but cannot harvest that just-issued writer.
		State->Scheduler->Tick(0.0f);
	}

	if (State->Phase == 1)
	{
		FURLabRenderCaptureJobStatus Status = State->Cancelled.PollStatus();
		if (Status.State == EURLabRenderCaptureJobState::Failed)
		{
			Test.AddError(FString::Printf(TEXT("cancellation fixture failed before receipt: %s"),
				*Status.Message));
			State->Cleanup();
			return true;
		}
		if (Status.State == EURLabRenderCaptureJobState::Succeeded)
		{
			Test.AddError(TEXT("cancellation fixture completed before scene replacement"));
			State->Cleanup();
			return true;
		}
		if (Status.State != EURLabRenderCaptureJobState::Running
			|| Status.bGpuWritersDrained)
		{
			// Check before ticking so a writer issued by the world's normal tick is
			// cancelled before any scheduler harvest pass. A retry tick similarly
			// issues after its camera-pipeline pass.
			State->Scheduler->Tick(0.0f);
			Status = State->Cancelled.PollStatus();
			if (Status.State != EURLabRenderCaptureJobState::Running
				|| Status.bGpuWritersDrained) return false;
		}

		const FURLabRenderWorldCommitResult ReplacementCommit =
			State->Coordinator->CommitCandidate(
				State->Registration, State->ReplacementCandidate);
		if (!Test.TestTrue(TEXT("replacement commits while accepted writer is pending"),
			ReplacementCommit.bCommitted))
		{
			Test.AddError(ReplacementCommit.Error);
			State->Cleanup();
			return true;
		}
		const FURLabRenderCaptureJobStatus CancelledNow =
			State->Cancelled.PollStatus();
		Test.TestEqual(TEXT("scene replacement makes active receipt explicitly cancelled"),
			CancelledNow.State, EURLabRenderCaptureJobState::Cancelled);
		Test.TestTrue(TEXT("replacement cancelled a writer observed pending before commit"),
			!Status.bGpuWritersDrained);
		State->Phase = 2;
		return false;
	}

	State->Scheduler->Tick(0.0f);
	const FURLabRenderCaptureJobStatus CancelledStatus =
		State->Cancelled.PollStatus();
	Test.TestEqual(TEXT("late GPU completion cannot become success"),
		CancelledStatus.State, EURLabRenderCaptureJobState::Cancelled);
	if (!CancelledStatus.bGpuWritersDrained) return false;
	AURLabRenderScene* Replacement =
		State->Coordinator->ResolveActiveScene(State->Registration);
	if (Test.TestNotNull(TEXT("replacement scene remains authoritative"), Replacement))
	{
		Test.TestEqual(TEXT("late old-scene writer cannot publish into successor"),
			Replacement->GetAppliedFrame().Identity.FrameId, static_cast<uint64>(20));
	}
	State->Cleanup();
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabRenderCaptureSchedulerGpuTest,
	"URLab.Render.CaptureSchedulerGPU.FifoExactPixelsReplacementAndDrain",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FURLabRenderCaptureSchedulerGpuTest::RunTest(const FString&)
{
	using namespace URLabRenderCaptureSchedulerGpuTest;
	if (!FApp::CanEverRender() || GUsingNullRHI)
	{
		AddInfo(TEXT("Skipped scheduler GPU acceptance: run with -RenderOffScreen and without -NullRHI"));
		return true;
	}
	TSharedPtr<FSchedulerGpuState> State = MakeShared<FSchedulerGpuState>();
	State->Test = this;
	State->World = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr;
	if (!TestNotNull(TEXT("rendering editor world"), State->World)) return false;
	State->Coordinator = State->World->GetSubsystem<UURLabRenderWorldSubsystem>();
	State->Scheduler = State->World->GetSubsystem<URenderCameraSubsystem>();
	if (!TestNotNull(TEXT("production render coordinator"), State->Coordinator)
		|| !TestNotNull(TEXT("production capture scheduler"), State->Scheduler)) return false;
	FString Error;
	if (!TestTrue(TEXT("GPU scheduler producer registers"),
		State->Coordinator->RegisterProducer(State->Registration, Error)))
	{
		AddError(Error);
		return false;
	}
	const FRenderFrame Initial = MakeFrame(TEXT("scheduler-gpu-a"), 10, 0.0);
	if (!TestTrue(TEXT("GPU scheduler scene installs"), StageAndCommit(
		*State->Coordinator, State->Registration, FString(),
		TEXT("scheduler-gpu-a"), Initial, Error)))
	{
		AddError(Error);
		State->Cleanup();
		return false;
	}
	AURLabRenderScene* Active =
		State->Coordinator->ResolveActiveScene(State->Registration);
	if (!TestNotNull(TEXT("GPU scheduler active scene"), Active))
	{
		State->Cleanup();
		return false;
	}
	if (UPrimitiveComponent* Geom = Active->GetGeomComponent(0))
	{
		Geom->MarkRenderStateDirty();
		TestFalse(TEXT("production readiness observes deferred render-state recreation"),
			Active->AreRenderResourcesReadyForCapture());
	}
	else
	{
		AddError(TEXT("GPU scheduler scene did not build its expected geom"));
		State->Cleanup();
		return false;
	}
	Active->SetRenderResourcesReadyForTest(false);
	const FURLabRenderCaptureJobAdmission First = State->Scheduler->SubmitExactCapture(
		MakeRequest(*State->Scheduler, State->Registration, TEXT("scheduler-gpu-a"), 11, -100.0));
	const FURLabRenderCaptureJobAdmission Second = State->Scheduler->SubmitExactCapture(
		MakeRequest(*State->Scheduler, State->Registration, TEXT("scheduler-gpu-a"), 12, 100.0));
	if (!TestTrue(TEXT("first Cartesian job admitted"), First.bAccepted)
		|| !TestTrue(TEXT("second Cartesian job admitted"), Second.bAccepted))
	{
		State->Cleanup();
		return false;
	}
	State->First = First.Job;
	State->Second = Second.Job;
	State->StartedSeconds = FPlatformTime::Seconds();
	ADD_LATENT_AUTOMATION_COMMAND(FURLabPumpRenderCaptureSchedulerGpu(State));
	return true;
}
#endif
