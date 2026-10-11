// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
#include "Render/RenderCameraSubsystem.h"

#include "HAL/PlatformTime.h"
#include "Async/TaskGraphInterfaces.h"
#include "Misc/ScopeLock.h"
#include "Misc/SecureHash.h"
#include "Render/RenderCameraComponent.h"
#include "Render/RenderCameraRig.h"
#include "Render/URLabRenderScene.h"

struct FURLabRenderCaptureJobSharedState
{
	mutable FCriticalSection Lock;
	EURLabRenderCaptureJobState State = EURLabRenderCaptureJobState::Queued;
	FString SceneRevision;
	FString GeometryConfigurationId;
	TArray<FString> CameraNames;
	TArray<FRenderCameraFramePtr> Frames;
	/** Retained so drain status remains truthful after scheduler teardown. */
	TArray<FRenderCameraCaptureReceipt> WriterReceipts;
	FString AffectedCamera;
	FString Message;
	bool bGpuWritersDrained = true;
};

struct URenderCameraSubsystem::FQueuedJob
{
	uint64 Serial = 0;
	TWeakObjectPtr<UURLabRenderWorldSubsystem> Coordinator;
	FURLabRenderProducerRegistration Producer;
	uint64 SceneGeneration = 0;
	FString SceneRevision;
	FURLabRenderExactCapturePlan Plan;
	FRenderFrame PresentationFrame;
	TArray<FString> CameraNames;
	double DeadlineSeconds = 0.0;
	FRenderCameraCaptureLifetimeLeasePtr LifetimeLease;
	TUniqueFunction<void()> OnTerminal;
	TSharedPtr<FURLabRenderCaptureJobSharedState, ESPMode::ThreadSafe> Shared;
	FURLabRenderPresentationHold Hold;
	TArray<TWeakObjectPtr<URenderCameraComponent>> Cameras;
	TArray<FRenderCameraOverrideToken> OverrideTokens;
	FRenderGeomPoseOverrideToken GeomOverrideToken;
	TArray<FRenderCameraCaptureReceipt> Receipts;
	EURLabRenderCaptureJobState PendingTerminalState = EURLabRenderCaptureJobState::Failed;
	FString PendingAffectedCamera;
	FString PendingMessage;
	bool bPendingRestorePresentation = false;
	bool bTerminalPublished = false;
};

void URenderCameraSubsystem::FQueuedJobDeleter::operator()(FQueuedJob* Job) const
{
	delete Job;
}

URenderCameraSubsystem::URenderCameraSubsystem() = default;
URenderCameraSubsystem::~URenderCameraSubsystem() = default;

namespace
{
void SetJobState(const TSharedPtr<FURLabRenderCaptureJobSharedState, ESPMode::ThreadSafe>& Shared,
	EURLabRenderCaptureJobState State, const FString& AffectedCamera,
	const FString& Message, bool bGpuWritersDrained)
{
	if (!Shared.IsValid()) return;
	FScopeLock Lock(&Shared->Lock);
	Shared->State = State;
	Shared->AffectedCamera = AffectedCamera;
	Shared->Message = Message;
	Shared->bGpuWritersDrained = bGpuWritersDrained;
	if (State != EURLabRenderCaptureJobState::Succeeded) Shared->Frames.Reset();
}

bool ReceiptsDrained(const TArray<FRenderCameraCaptureReceipt>& Receipts)
{
	for (const FRenderCameraCaptureReceipt& Receipt : Receipts)
	{
		if (!Receipt.IsGpuWriterDrained()) return false;
	}
	return true;
}

bool SameFrameIdentity(const FRenderFrameIdentity& A, const FRenderFrameIdentity& B)
{
	return A.SceneRevision == B.SceneRevision && A.FrameId == B.FrameId
		&& A.SourceTime == B.SourceTime && A.ClockEpoch == B.ClockEpoch
		&& A.bHasSourceTime == B.bHasSourceTime;
}

bool CompletionMatchesReceipt(const FRenderCameraCaptureReceipt& Receipt,
	const FRenderCameraFrame& Frame)
{
	return SameFrameIdentity(Receipt.Identity, Frame.Identity)
		&& Receipt.Mode == Frame.Mode
		&& Receipt.ConfigurationRevision == Frame.CameraConfigurationRevision
		&& Receipt.CaptureSerial == Frame.CaptureSerial
		&& Receipt.Resolution.X == Frame.Width
		&& Receipt.Resolution.Y == Frame.Height;
}

bool SameOptics(const FRenderCameraOptics& A, const FRenderCameraOptics& B)
{
	return A.Resolution == B.Resolution && A.Projection == B.Projection
		&& A.VerticalFovDegrees == B.VerticalFovDegrees
		&& A.OrthographicWidthCm == B.OrthographicWidthCm
		&& A.bHasIntrinsics == B.bHasIntrinsics
		&& A.FocalPixelsX == B.FocalPixelsX && A.FocalPixelsY == B.FocalPixelsY
		&& A.PrincipalPixelsX == B.PrincipalPixelsX && A.PrincipalPixelsY == B.PrincipalPixelsY
		&& A.NearClipCm == B.NearClipCm && A.FarClipCm == B.FarClipCm;
}

bool SameCameraConfiguration(const FRenderCameraEffectiveConfiguration& A,
	const FRenderCameraEffectiveConfiguration& B)
{
	return A.CanonicalName == B.CanonicalName && A.Mode == B.Mode
		&& A.BaselineConfigurationRevision == B.BaselineConfigurationRevision
		&& SameOptics(A.Optics, B.Optics)
		&& A.DepthNearCm == B.DepthNearCm && A.DepthFarCm == B.DepthFarCm
		&& A.LogicalPixelBytes == B.LogicalPixelBytes
		&& A.PhysicalArrayBytes == B.PhysicalArrayBytes;
}

bool ResolvePlannedConfiguration(URenderCameraComponent& Camera,
	const FRenderCameraEffectiveConfiguration& Planned,
	FRenderCameraEffectiveConfiguration& Out, FString& OutError)
{
	FRenderCameraCaptureOverride Override;
	Override.Mode = Planned.Mode;
	Override.CompleteOptics = Planned.Optics;
	if (Planned.Mode == ERenderCameraMode::Depth)
	{
		Override.DepthNearCm = Planned.DepthNearCm;
		Override.DepthFarCm = Planned.DepthFarCm;
	}
	return Camera.ResolveCaptureConfiguration(Override, Out, OutError)
		&& SameCameraConfiguration(Out, Planned);
}

void AppendConfigurationDigest(FSHA1& Hash, const void* Data, int32 Bytes)
{
	Hash.Update(static_cast<const uint8*>(Data), Bytes);
}
}

FString RenderCameraConfigurationId(const FRenderCameraEffectiveConfiguration& C)
{
	FSHA1 Hash;
	FTCHARToUTF8 Name(*C.CanonicalName);
	Hash.Update(reinterpret_cast<const uint8*>(Name.Get()), Name.Length());
	AppendConfigurationDigest(Hash, &C.Mode, sizeof(C.Mode));
	AppendConfigurationDigest(Hash, &C.BaselineConfigurationRevision,
		sizeof(C.BaselineConfigurationRevision));
	AppendConfigurationDigest(Hash, &C.Optics.Resolution.X, sizeof(C.Optics.Resolution.X));
	AppendConfigurationDigest(Hash, &C.Optics.Resolution.Y, sizeof(C.Optics.Resolution.Y));
	AppendConfigurationDigest(Hash, &C.Optics.Projection, sizeof(C.Optics.Projection));
	AppendConfigurationDigest(Hash, &C.Optics.VerticalFovDegrees, sizeof(float));
	AppendConfigurationDigest(Hash, &C.Optics.OrthographicWidthCm, sizeof(float));
	AppendConfigurationDigest(Hash, &C.Optics.bHasIntrinsics, sizeof(bool));
	AppendConfigurationDigest(Hash, &C.Optics.FocalPixelsX, sizeof(float));
	AppendConfigurationDigest(Hash, &C.Optics.FocalPixelsY, sizeof(float));
	AppendConfigurationDigest(Hash, &C.Optics.PrincipalPixelsX, sizeof(float));
	AppendConfigurationDigest(Hash, &C.Optics.PrincipalPixelsY, sizeof(float));
	AppendConfigurationDigest(Hash, &C.Optics.NearClipCm, sizeof(float));
	AppendConfigurationDigest(Hash, &C.Optics.FarClipCm, sizeof(float));
	AppendConfigurationDigest(Hash, &C.DepthNearCm, sizeof(C.DepthNearCm));
	AppendConfigurationDigest(Hash, &C.DepthFarCm, sizeof(C.DepthFarCm));
	Hash.Final();
	uint8 Digest[FSHA1::DigestSize]; Hash.GetHash(Digest);
	return BytesToHex(Digest, UE_ARRAY_COUNT(Digest)).ToLower();
}

bool FURLabRenderCaptureJob::IsValid() const
{
	return SchedulerLifetime.IsValid() && JobSerial > 0 && State.IsValid();
}

FURLabRenderCaptureJobStatus FURLabRenderCaptureJob::PollStatus() const
{
	FURLabRenderCaptureJobStatus Result;
	if (!State.IsValid())
	{
		Result.State = EURLabRenderCaptureJobState::Cancelled;
		Result.Message = TEXT("exact capture job handle is invalid");
		return Result;
	}
	FScopeLock Lock(&State->Lock);
	Result.State = State->State;
	Result.SceneRevision = State->SceneRevision;
	Result.GeometryConfigurationId = State->GeometryConfigurationId;
	Result.CameraNames = State->CameraNames;
	Result.AffectedCamera = State->AffectedCamera;
	Result.Message = State->Message;
	Result.bGpuWritersDrained = State->bGpuWritersDrained;
	if (!Result.bGpuWritersDrained)
	{
		Result.bGpuWritersDrained = true;
		for (const FRenderCameraCaptureReceipt& Receipt : State->WriterReceipts)
			Result.bGpuWritersDrained &= Receipt.IsGpuWriterDrained();
		if (Result.bGpuWritersDrained) State->bGpuWritersDrained = true;
	}
	return Result;
}

FURLabRenderCaptureJobSnapshot FURLabRenderCaptureJob::Poll() const
{
	check(IsInGameThread());
	FURLabRenderCaptureJobSnapshot Result;
	if (!State.IsValid())
	{
		Result.State = EURLabRenderCaptureJobState::Cancelled;
		Result.Message = TEXT("exact capture job handle is invalid");
		return Result;
	}
	FScopeLock Lock(&State->Lock);
	Result.State = State->State;
	Result.SceneRevision = State->SceneRevision;
	Result.GeometryConfigurationId = State->GeometryConfigurationId;
	Result.CameraNames = State->CameraNames;
	Result.Frames = State->Frames;
	Result.AffectedCamera = State->AffectedCamera;
	Result.Message = State->Message;
	Result.bGpuWritersDrained = State->bGpuWritersDrained;
	if (!Result.bGpuWritersDrained)
	{
		Result.bGpuWritersDrained = true;
		for (const FRenderCameraCaptureReceipt& Receipt : State->WriterReceipts)
			Result.bGpuWritersDrained &= Receipt.IsGpuWriterDrained();
		if (Result.bGpuWritersDrained) State->bGpuWritersDrained = true;
	}
	return Result;
}

void URenderCameraSubsystem::RegisterCamera(URenderCameraComponent* Camera)
{
	if (Camera) Cameras.AddUnique(Camera);
}

void URenderCameraSubsystem::UnregisterCamera(URenderCameraComponent* Camera)
{
	Cameras.RemoveAll([Camera](const TWeakObjectPtr<URenderCameraComponent>& Item)
	{
		return !Item.IsValid() || Item.Get() == Camera;
	});
}

double URenderCameraSubsystem::ReadMonotonicSeconds() const
{
#if WITH_DEV_AUTOMATION_TESTS
	if (TestMonotonicSeconds.IsSet()) return TestMonotonicSeconds.GetValue();
#endif
	return FPlatformTime::Seconds();
}

bool URenderCameraSubsystem::PlanExactCapture(
	const FURLabRenderProducerRegistration& Producer,
	const FString& ExpectedSceneRevision, const TArray<FString>& CameraNames,
	FURLabRenderExactCapturePlan& OutPlan, FString& OutError) const
{
	TArray<FRenderCameraCaptureSelection> Selections;
	Selections.Reserve(CameraNames.Num());
	for (const FString& Name : CameraNames)
	{
		FRenderCameraCaptureSelection& Selection = Selections.AddDefaulted_GetRef();
		Selection.CameraName = Name;
	}
	return PlanExactCapture(Producer, ExpectedSceneRevision, Selections, OutPlan, OutError);
}

bool URenderCameraSubsystem::PlanExactCapture(
	const FURLabRenderProducerRegistration& Producer,
	const FString& ExpectedSceneRevision,
	const TArray<FRenderCameraCaptureSelection>& CameraSelections,
	FURLabRenderExactCapturePlan& OutPlan, FString& OutError) const
{
	check(IsInGameThread());
	OutPlan = {};
	OutError.Reset();
	if (bShuttingDown)
	{
		OutError = TEXT("render camera scheduler is shutting down");
		return false;
	}
	if (ExpectedSceneRevision.IsEmpty() || CameraSelections.Num() <= 0
		|| CameraSelections.Num() > MaxCamerasPerExactCaptureJob)
	{
		OutError = TEXT("exact capture plan requires a scene and supported camera count");
		return false;
	}
	UURLabRenderWorldSubsystem* Coordinator = Producer.Coordinator.Get();
	if (!Coordinator || !Coordinator->ValidateRegistration(Producer, OutError)
		|| Coordinator->GetWorld() != GetWorld())
	{
		if (OutError.IsEmpty()) OutError = TEXT("exact capture producer belongs to another world");
		return false;
	}
	if (Coordinator->ResolveActiveSceneRevision(Producer) != ExpectedSceneRevision)
	{
		OutError = TEXT("exact capture plan scene revision is not active");
		return false;
	}
	AURLabRenderScene* Scene = Coordinator->ResolveActiveScene(Producer);
	URenderCameraRig* Rig = Scene ? Scene->GetCameraRig() : nullptr;
	if (!Rig)
	{
		OutError = TEXT("exact capture target scene has no camera rig");
		return false;
	}
	TSet<FString> UniqueNames;
	for (const FRenderCameraCaptureSelection& Selection : CameraSelections)
	{
		const FString& Name = Selection.CameraName;
		if (Name.IsEmpty() || UniqueNames.Contains(Name))
		{
			OutError = TEXT("exact capture camera names must be nonempty and unique");
			return false;
		}
		URenderCameraComponent* Camera = Rig->FindByCanonicalName(Name);
		if (!Camera)
		{
			OutError = FString::Printf(TEXT("exact capture camera '%s' is not installed"), *Name);
			return false;
		}
		FRenderCameraEffectiveConfiguration Snapshot;
		if (!Camera->ResolveCaptureConfiguration(Selection.Override, Snapshot, OutError)) return false;
		if (Snapshot.CanonicalName != Name
			|| Snapshot.LogicalPixelBytes > MAX_int64 - OutPlan.AggregateLogicalPixelBytes
			|| Snapshot.PhysicalArrayBytes > MAX_int64 - OutPlan.AggregatePhysicalArrayBytes)
		{
			OutError = TEXT("exact capture camera plan identity or aggregate size is invalid");
			return false;
		}
		UniqueNames.Add(Name);
		OutPlan.AggregateLogicalPixelBytes += Snapshot.LogicalPixelBytes;
		OutPlan.AggregatePhysicalArrayBytes += Snapshot.PhysicalArrayBytes;
		OutPlan.Cameras.Add(MoveTemp(Snapshot));
	}
	OutPlan.SceneRevision = ExpectedSceneRevision;
	OutPlan.Producer = Producer;
	OutPlan.SceneGeneration = Coordinator->ResolveActiveSceneGeneration(Producer);
	if (OutPlan.SceneGeneration == 0)
	{
		OutError = TEXT("exact capture target scene is not active");
		OutPlan = {};
		return false;
	}
	return true;
}

bool URenderCameraSubsystem::ValidateAdmission(const FURLabRenderCaptureJobRequest& Request,
	UURLabRenderWorldSubsystem*& OutCoordinator, uint64& OutSceneGeneration,
	FString& OutError) const
{
	OutCoordinator = nullptr;
	OutSceneGeneration = 0;
	OutError.Reset();
	if (!IsInGameThread())
	{
		OutError = TEXT("exact capture admission must run on the game thread");
		return false;
	}
	if (bShuttingDown)
	{
		OutError = TEXT("render camera scheduler is shutting down");
		return false;
	}
	if (!Request.LifetimeLease.IsValid())
	{
		OutError = TEXT("exact capture requires a retained workflow lifetime lease");
		return false;
	}
	if (Request.ExpectedSceneRevision.IsEmpty()
		|| Request.Plan.SceneRevision != Request.ExpectedSceneRevision
		|| Request.Plan.GeometryConfigurationId.IsEmpty())
	{
		OutError = TEXT("exact capture requires a nonempty scene revision");
		return false;
	}
	if (!Request.ExpectedSceneRevision.IsEmpty()
		&& Request.PresentationFrame.Identity.SceneRevision
			!= Request.ExpectedSceneRevision)
	{
		OutError = TEXT("exact capture presentation targets another scene revision");
		return false;
	}
	const double Timeout = Request.TimeoutSeconds == 0.0
		? DefaultExactCaptureDeadlineSeconds : Request.TimeoutSeconds;
	if (!FMath::IsFinite(Timeout) || Timeout <= 0.0
		|| Timeout > MaxExactCaptureDeadlineSeconds)
	{
		OutError = TEXT("exact capture timeout must be finite and within the supported bound");
		return false;
	}
	OutCoordinator = Request.Producer.Coordinator.Get();
	if (!OutCoordinator || !OutCoordinator->ValidateRegistration(Request.Producer, OutError))
		return false;
	if (OutCoordinator->GetWorld() != GetWorld())
	{
		OutError = TEXT("exact capture producer belongs to another world");
		return false;
	}
	if (!OutCoordinator->ValidateActiveFrame(Request.ExpectedSceneRevision,
		Request.PresentationFrame, OutError))
		return false;
	OutSceneGeneration = OutCoordinator->ResolveActiveSceneGeneration(Request.Producer);
	if (OutSceneGeneration == 0)
	{
		OutError = TEXT("exact capture target scene is not active");
		return false;
	}
	FURLabRenderExactCapturePlan CurrentPlan;
	TArray<FString> Names;
	for (const FRenderCameraEffectiveConfiguration& Camera : Request.Plan.Cameras)
		Names.Add(Camera.CanonicalName);
	TArray<FRenderCameraCaptureSelection> Selections;
	for (const FRenderCameraEffectiveConfiguration& Camera : Request.Plan.Cameras)
	{
		FRenderCameraCaptureSelection& Selection = Selections.AddDefaulted_GetRef();
		Selection.CameraName = Camera.CanonicalName;
		FRenderCameraCaptureOverride Override;
		Override.Mode = Camera.Mode;
		Override.CompleteOptics = Camera.Optics;
		if (Camera.Mode == ERenderCameraMode::Depth)
		{
			Override.DepthNearCm = Camera.DepthNearCm;
			Override.DepthFarCm = Camera.DepthFarCm;
		}
		Selection.Override = MoveTemp(Override);
	}
	if (!PlanExactCapture(Request.Producer, Request.ExpectedSceneRevision,
		Selections, CurrentPlan, OutError)) return false;
	if (Request.Plan.Producer.Coordinator.Get() != Request.Producer.Coordinator.Get()
		|| Request.Plan.Producer.WorldLifetime != Request.Producer.WorldLifetime
		|| Request.Plan.Producer.ProducerSerial != Request.Producer.ProducerSerial
		|| CurrentPlan.SceneGeneration != Request.Plan.SceneGeneration
		|| CurrentPlan.AggregateLogicalPixelBytes != Request.Plan.AggregateLogicalPixelBytes
		|| CurrentPlan.AggregatePhysicalArrayBytes != Request.Plan.AggregatePhysicalArrayBytes
		|| CurrentPlan.Cameras.Num() != Request.Plan.Cameras.Num())
	{
		OutError = TEXT("exact capture plan no longer matches installed camera configuration");
		return false;
	}
	for (int32 Index = 0; Index < CurrentPlan.Cameras.Num(); ++Index)
	{
		if (!SameCameraConfiguration(CurrentPlan.Cameras[Index], Request.Plan.Cameras[Index]))
		{
			OutError = TEXT("exact capture plan no longer matches installed camera configuration");
			return false;
		}
	}
	return true;
}

FURLabRenderCaptureJobAdmission URenderCameraSubsystem::SubmitExactCapture(
	FURLabRenderCaptureJobRequest&& Request)
{
	FURLabRenderCaptureJobAdmission Result;
	if (!IsInGameThread())
	{
		Result.Error = TEXT("exact capture admission must run on the game thread");
		return Result;
	}
	const int32 RetainedJobs = QueuedJobs.Num() + (ActiveJob.IsValid() ? 1 : 0)
		+ DrainingJobs.Num();
	if (RetainedJobs >= MaxRetainedExactCaptureJobs)
	{
		Result.Error = TEXT("exact capture retained-job capacity is full");
		return Result;
	}
	if (QueuedJobs.Num() >= MaxQueuedExactCaptureJobs)
	{
		Result.Error = TEXT("exact capture queue is full");
		return Result;
	}
	if (NextJobSerial == 0 || NextJobSerial == MAX_uint64)
	{
		Result.Error = TEXT("exact capture job identity space is exhausted");
		return Result;
	}
	UURLabRenderWorldSubsystem* Coordinator = nullptr;
	uint64 SceneGeneration = 0;
	if (!ValidateAdmission(Request, Coordinator, SceneGeneration, Result.Error)) return Result;
	const double Now = ReadMonotonicSeconds();
	const double Timeout = Request.TimeoutSeconds == 0.0
		? DefaultExactCaptureDeadlineSeconds : Request.TimeoutSeconds;
	if (!FMath::IsFinite(Now) || !FMath::IsFinite(Now + Timeout))
	{
		Result.Error = TEXT("exact capture monotonic clock is invalid");
		return Result;
	}

	if (!SchedulerLifetime.IsValid()) SchedulerLifetime = FGuid::NewGuid();
	FQueuedJobPtr Job(new FQueuedJob());
	Job->Serial = NextJobSerial++;
	Job->Coordinator = Coordinator;
	Job->Producer = Request.Producer;
	Job->SceneGeneration = SceneGeneration;
	Job->SceneRevision = MoveTemp(Request.ExpectedSceneRevision);
	Job->Plan = MoveTemp(Request.Plan);
	Job->PresentationFrame = MoveTemp(Request.PresentationFrame);
	for (const FRenderCameraEffectiveConfiguration& Camera : Job->Plan.Cameras)
		Job->CameraNames.Add(Camera.CanonicalName);
	Job->DeadlineSeconds = Now + Timeout;
	Job->LifetimeLease = MoveTemp(Request.LifetimeLease);
	Job->OnTerminal = MoveTemp(Request.OnTerminal);
	Job->Shared = MakeShared<FURLabRenderCaptureJobSharedState, ESPMode::ThreadSafe>();
	Job->Shared->SceneRevision = Job->SceneRevision;
	Job->Shared->GeometryConfigurationId = Job->Plan.GeometryConfigurationId;
	Job->Shared->CameraNames = Job->CameraNames;

	Result.Job.Scheduler = this;
	Result.Job.SchedulerLifetime = SchedulerLifetime;
	Result.Job.JobSerial = Job->Serial;
	Result.Job.State = Job->Shared;
	Result.bAccepted = true;
	QueuedJobs.Add(MoveTemp(Job));
	return Result;
}

bool URenderCameraSubsystem::JobMatchesScene(const FQueuedJob& Job,
	UURLabRenderWorldSubsystem* Coordinator, uint64 ProducerSerial,
	uint64 SceneGeneration, const FString& SceneRevision) const
{
	return Job.Coordinator.Get() == Coordinator
		&& Job.Producer.ProducerSerial == ProducerSerial
		&& Job.SceneGeneration == SceneGeneration
		&& Job.SceneRevision == SceneRevision;
}

void URenderCameraSubsystem::QueueNotification(FQueuedJob& Job)
{
	if (Job.OnTerminal) PendingNotifications.Add(MoveTemp(Job.OnTerminal));
}

void URenderCameraSubsystem::MarkQueuedTerminal(FQueuedJob& Job,
	EURLabRenderCaptureJobState State, const FString& Message)
{
	SetJobState(Job.Shared, State, FString(), Message, true);
	QueueNotification(Job);
	Job.bTerminalPublished = true;
}

void URenderCameraSubsystem::StartNextJob(double NowSeconds)
{
	while (!ActiveJob.IsValid() && QueuedJobs.Num() > 0)
	{
		FQueuedJobPtr Job = MoveTemp(QueuedJobs[0]);
		QueuedJobs.RemoveAt(0, 1, EAllowShrinking::No);
		if (NowSeconds >= Job->DeadlineSeconds)
		{
			MarkQueuedTerminal(*Job, EURLabRenderCaptureJobState::Failed,
				TEXT("exact capture deadline expired in the queue"));
			continue;
		}
		UURLabRenderWorldSubsystem* Coordinator = Job->Coordinator.Get();
		FString Error;
		if (!Coordinator
			|| Coordinator->ResolveActiveSceneRevision(Job->Producer) != Job->Plan.SceneRevision
			|| Coordinator->ResolveActiveSceneGeneration(Job->Producer)
				!= Job->Plan.SceneGeneration)
		{
			MarkQueuedTerminal(*Job, EURLabRenderCaptureJobState::Failed,
				TEXT("exact capture scene identity changed before execution"));
			continue;
		}
		AURLabRenderScene* Scene = Coordinator
			? Coordinator->ResolveActiveScene(Job->Producer) : nullptr;
		URenderCameraRig* Rig = Scene ? Scene->GetCameraRig() : nullptr;
		if (!Coordinator || !Rig)
		{
			MarkQueuedTerminal(*Job, EURLabRenderCaptureJobState::Failed,
				TEXT("exact capture scene or camera rig is no longer available"));
			continue;
		}
		bool bPlanCurrent = Job->Plan.Cameras.Num() == Job->CameraNames.Num();
		for (int32 Index = 0; bPlanCurrent && Index < Job->Plan.Cameras.Num(); ++Index)
		{
			URenderCameraComponent* Camera =
				Rig->FindByCanonicalName(Job->CameraNames[Index]);
			FRenderCameraEffectiveConfiguration Current;
			bPlanCurrent = Camera
				&& ResolvePlannedConfiguration(*Camera,
					Job->Plan.Cameras[Index], Current, Error);
			if (bPlanCurrent) Job->Cameras.Add(Camera);
		}
		if (!bPlanCurrent)
		{
			MarkQueuedTerminal(*Job, EURLabRenderCaptureJobState::Failed,
				Error.IsEmpty()
					? TEXT("exact capture camera configuration changed before execution") : Error);
			continue;
		}
		if (!Coordinator->AcquirePresentationHold(Job->Producer,
				Job->SceneRevision, Job->PresentationFrame, Job->Hold, Error))
		{
			MarkQueuedTerminal(*Job, EURLabRenderCaptureJobState::Failed,
				Error.IsEmpty() ? TEXT("exact capture scene is no longer available") : Error);
			continue;
		}
		if (!Scene->BeginGeomPoseOverrides(Job->Plan.GeomPoseOverrides,
				Job->GeomOverrideToken, Error))
		{
			FString Ignored;
			Coordinator->ReleasePresentationHold(Job->Producer, Job->Hold, Ignored);
			MarkQueuedTerminal(*Job, EURLabRenderCaptureJobState::Failed,
				Error.IsEmpty() ? TEXT("geometry override application failed") : Error);
			continue;
		}
		bool bOverridesApplied = true;
		for (int32 Index = 0; Index < Job->Cameras.Num(); ++Index)
		{
			FRenderCameraOverrideToken Token;
			URenderCameraComponent* Camera = Job->Cameras[Index].Get();
			if (!Camera || !Camera->BeginCaptureOverride(Job->Plan.Cameras[Index], Token, Error))
			{
				bOverridesApplied = false;
				break;
			}
			Job->OverrideTokens.Add(MoveTemp(Token));
		}
		if (!bOverridesApplied)
		{
			for (int32 Index = Job->OverrideTokens.Num() - 1; Index >= 0; --Index)
			{
				FString Ignored;
				if (URenderCameraComponent* Camera = Job->Cameras[Index].Get())
					Camera->RestoreCaptureOverride(Job->OverrideTokens[Index], Ignored);
			}
			FString GeomRestoreError;
			if (!Scene->RestoreGeomPoseOverrides(Job->GeomOverrideToken, GeomRestoreError)
				&& Error.IsEmpty()) Error = MoveTemp(GeomRestoreError);
			Job->GeomOverrideToken = {};
			FString Ignored;
			Coordinator->ReleasePresentationHold(Job->Producer, Job->Hold, Ignored);
			MarkQueuedTerminal(*Job, EURLabRenderCaptureJobState::Failed,
				Error.IsEmpty() ? TEXT("camera override application failed") : Error);
			continue;
		}
		Job->Receipts.SetNum(Job->CameraNames.Num());
		SetJobState(Job->Shared, EURLabRenderCaptureJobState::Running,
			FString(), FString(), true);
		ActiveJob = MoveTemp(Job);
	}
}

void URenderCameraSubsystem::FinishActive(EURLabRenderCaptureJobState State,
	const FString& AffectedCamera, const FString& Message, bool bRestorePresentation)
{
	if (!ActiveJob.IsValid()) return;
	for (const FRenderCameraCaptureReceipt& Receipt : ActiveJob->Receipts)
	{
		Receipt.Cancel(Message);
	}
	const bool bDrained = ReceiptsDrained(ActiveJob->Receipts);
	ActiveJob->PendingTerminalState = State;
	ActiveJob->PendingAffectedCamera = AffectedCamera;
	ActiveJob->PendingMessage = Message;
	ActiveJob->bPendingRestorePresentation = bRestorePresentation;
	if (!bDrained && bRestorePresentation)
	{
		// Publish the requested terminal outcome immediately while the cancelled GPU
		// writers retain their lifetime leases. Presentation restoration remains
		// deferred until those writers drain.
		SetJobState(ActiveJob->Shared, State, AffectedCamera,
			ActiveJob->PendingMessage, false);
		QueueNotification(*ActiveJob);
		ActiveJob->bTerminalPublished = true;
		DrainingJobs.Add(MoveTemp(ActiveJob));
		return;
	}
	for (int32 Index = bRestorePresentation ? ActiveJob->OverrideTokens.Num() - 1 : -1;
		Index >= 0; --Index)
	{
		FString RestoreError;
		URenderCameraComponent* Camera = ActiveJob->Cameras.IsValidIndex(Index)
			? ActiveJob->Cameras[Index].Get() : nullptr;
		if (!Camera || !Camera->RestoreCaptureOverride(
			ActiveJob->OverrideTokens[Index], RestoreError))
		{
			State = EURLabRenderCaptureJobState::Failed;
			if (ActiveJob->PendingMessage.IsEmpty()) ActiveJob->PendingMessage = RestoreError;
		}
	}
	ActiveJob->OverrideTokens.Reset();
	if (bRestorePresentation && ActiveJob->GeomOverrideToken.IsValid())
	{
		FString RestoreError;
		AURLabRenderScene* Scene = ActiveJob->Coordinator.IsValid()
			? ActiveJob->Coordinator->ResolveActiveScene(ActiveJob->Producer) : nullptr;
		if (!Scene || !Scene->RestoreGeomPoseOverrides(
				ActiveJob->GeomOverrideToken, RestoreError))
		{
			State = EURLabRenderCaptureJobState::Failed;
			if (ActiveJob->PendingMessage.IsEmpty())
				ActiveJob->PendingMessage = RestoreError.IsEmpty()
					? TEXT("geometry override restoration failed") : MoveTemp(RestoreError);
		}
		ActiveJob->GeomOverrideToken = {};
	}
	SetJobState(ActiveJob->Shared, State, AffectedCamera,
		ActiveJob->PendingMessage, true);
	if (bRestorePresentation && ActiveJob->Hold.IsValid())
	{
		if (UURLabRenderWorldSubsystem* Coordinator = ActiveJob->Coordinator.Get())
		{
			FString RestoreError;
			if (!Coordinator->ReleasePresentationHold(
				ActiveJob->Producer, ActiveJob->Hold, RestoreError))
			{
				FScopeLock Lock(&ActiveJob->Shared->Lock);
				if (ActiveJob->Shared->Message.IsEmpty())
					ActiveJob->Shared->Message = MoveTemp(RestoreError);
			}
		}
	}
	QueueNotification(*ActiveJob);
	ActiveJob->bTerminalPublished = true;
	// Retire even an already-drained record on a later scheduler pass. This keeps
	// workflow-owned lease destructors out of coordinator scene mutation.
	DrainingJobs.Add(MoveTemp(ActiveJob));
}

void URenderCameraSubsystem::PumpActiveJob(double NowSeconds)
{
	if (!ActiveJob.IsValid()) return;
	if (NowSeconds >= ActiveJob->DeadlineSeconds)
	{
		FinishActive(EURLabRenderCaptureJobState::Failed, FString(),
			TEXT("exact capture deadline expired"), true);
		return;
	}
	UURLabRenderWorldSubsystem* Coordinator = ActiveJob->Coordinator.Get();
	AURLabRenderScene* Scene = Coordinator
		? Coordinator->ResolveActiveScene(ActiveJob->Producer) : nullptr;
	if (!Coordinator || !Scene
		|| Coordinator->ResolveActiveSceneRevision(ActiveJob->Producer)
			!= ActiveJob->SceneRevision
		|| Coordinator->ResolveActiveSceneGeneration(ActiveJob->Producer)
			!= ActiveJob->SceneGeneration)
	{
		FinishActive(EURLabRenderCaptureJobState::Failed, FString(),
			TEXT("exact capture scene changed before issuance"), true);
		return;
	}
	for (int32 Index = 0; Index < ActiveJob->Receipts.Num(); ++Index)
	{
		if (ActiveJob->Receipts[Index].IsValid()) continue;
		URenderCameraComponent* Camera = ActiveJob->Cameras.IsValidIndex(Index)
			? ActiveJob->Cameras[Index].Get() : nullptr;
		if (!Camera)
		{
			FinishActive(EURLabRenderCaptureJobState::Failed,
				ActiveJob->CameraNames[Index],
				TEXT("exact capture camera was retired before issuance"), true);
			return;
		}
		FRenderCameraEffectiveConfiguration Current;
		FString ConfigurationError;
		if (!ActiveJob->Plan.Cameras.IsValidIndex(Index)
			|| !ResolvePlannedConfiguration(*Camera,
				ActiveJob->Plan.Cameras[Index], Current, ConfigurationError))
		{
			FinishActive(EURLabRenderCaptureJobState::Failed,
				ActiveJob->CameraNames[Index], ConfigurationError.IsEmpty()
					? TEXT("exact capture camera configuration changed before issuance")
					: ConfigurationError, true);
			return;
		}
		// Material assignment can start component PSO work after scene installation.
		// Validate scene and camera identity first, then leave the accepted job pending
		// under its existing deadline until the registered primitives are ready.
		if (!Scene->AreRenderResourcesReadyForCapture()) return;
		const FRenderCameraCaptureIssueResult Issue =
			Camera->TryIssueExactCapture(ActiveJob->LifetimeLease);
		if (Issue.IsAccepted())
		{
			// Retain the writer before inspecting public metadata. Even a malformed
			// accepted receipt may own a GPU copy and its workflow capacity lease.
			ActiveJob->Receipts[Index] = Issue.Receipt;
			{
				FScopeLock Lock(&ActiveJob->Shared->Lock);
				ActiveJob->Shared->WriterReceipts.Add(Issue.Receipt);
				ActiveJob->Shared->bGpuWritersDrained = false;
			}
			if (!Issue.Receipt.IsValid()
				|| Issue.Receipt.CameraName != ActiveJob->CameraNames[Index]
				|| Issue.Receipt.Mode != ActiveJob->Plan.Cameras[Index].Mode
				|| Issue.Receipt.Resolution != ActiveJob->Plan.Cameras[Index].Optics.Resolution
				|| Issue.Receipt.ConfigurationRevision
					!= ActiveJob->Plan.Cameras[Index].BaselineConfigurationRevision
				|| !SameFrameIdentity(Issue.Receipt.Identity,
					ActiveJob->PresentationFrame.Identity))
			{
				FinishActive(EURLabRenderCaptureJobState::Failed,
					ActiveJob->CameraNames[Index],
					TEXT("camera accepted an inexact capture receipt"), true);
				return;
			}
		}
		else if (!Issue.IsRetryable())
		{
			FinishActive(EURLabRenderCaptureJobState::Failed,
				ActiveJob->CameraNames[Index], Issue.Message, true);
			return;
		}
	}

	for (const FRenderCameraCaptureReceipt& Receipt : ActiveJob->Receipts)
		if (!Receipt.IsValid()) return;

	TArray<FRenderCameraFramePtr> Frames;
	Frames.Reserve(ActiveJob->Receipts.Num());
	for (int32 Index = 0; Index < ActiveJob->Receipts.Num(); ++Index)
	{
		const FRenderCameraCaptureCompletion Completion =
			ActiveJob->Receipts[Index].Poll();
		if (!Completion.IsTerminal()) return;
		if (!Completion.IsSuccessful())
		{
			FinishActive(EURLabRenderCaptureJobState::Failed,
				ActiveJob->CameraNames[Index], Completion.Message, true);
			return;
		}
		if (!Completion.Frame.IsValid()
			|| !CompletionMatchesReceipt(ActiveJob->Receipts[Index],
				*Completion.Frame))
		{
			FinishActive(EURLabRenderCaptureJobState::Failed,
				ActiveJob->CameraNames[Index],
				TEXT("exact capture completed without its exact owned frame"), true);
			return;
		}
		Frames.Add(Completion.Frame);
	}
	{
		FScopeLock Lock(&ActiveJob->Shared->Lock);
		ActiveJob->Shared->Frames = MoveTemp(Frames);
	}
	TArray<FRenderCameraFramePtr> HistoryFrames;
	{ FScopeLock Lock(&ActiveJob->Shared->Lock); HistoryFrames = ActiveJob->Shared->Frames; }
	RetainExactCaptureHistory(ActiveJob->Plan, ActiveJob->PresentationFrame.Identity, HistoryFrames);
	FinishActive(EURLabRenderCaptureJobState::Succeeded, FString(), FString(), true);
}

FURLabRenderCaptureJobAdmission URenderCameraSubsystem::QueryExactCaptureHistory(
	const FURLabRenderHistoryQuery& Query)
{
	check(IsInGameThread());
	FURLabRenderCaptureJobAdmission Result;
	if (!Query.RetentionContext.IsValid() || Query.ExpectedSceneRevision.IsEmpty()
		|| Query.GeometryConfigurationId.IsEmpty()
		|| !FMath::IsFinite(Query.TargetSourceTime) || Query.Cameras.IsEmpty()
		|| Query.Cameras.Num() > MaxCamerasPerExactCaptureJob)
	{
		Result.Error = TEXT("history query is invalid"); return Result;
	}
	const FHistoryGroup* Selected = nullptr;
	TArray<int32> Indices;
	for (int32 GroupIndex = HistoryGroups.Num() - 1; GroupIndex >= 0; --GroupIndex)
	{
		const FHistoryGroup& Group = HistoryGroups[GroupIndex];
		if (Group.RetentionContext != Query.RetentionContext
			|| Group.SceneRevision != Query.ExpectedSceneRevision
			|| Group.GeometryConfigurationId != Query.GeometryConfigurationId
			|| !Group.Identity.bHasSourceTime || Group.Identity.ClockEpoch != Query.ClockEpoch
			|| Group.Identity.SourceTime > Query.TargetSourceTime) continue;
		TArray<int32> Candidate;
		for (const FURLabRenderHistoryCameraQuery& Camera : Query.Cameras)
		{
			int32 Match = INDEX_NONE;
			for (int32 Index = 0; Index < Group.Names.Num(); ++Index)
				if (Group.Names[Index] == Camera.Name
					&& Group.ConfigurationIds.IsValidIndex(Index)
					&& Group.ConfigurationIds[Index] == Camera.ConfigurationId) { Match = Index; break; }
			if (Match == INDEX_NONE) { Candidate.Reset(); break; }
			Candidate.Add(Match);
		}
		if (Candidate.Num() == Query.Cameras.Num())
		{
			Selected = &Group; Indices = MoveTemp(Candidate); break;
		}
	}
	if (!Selected) { Result.Error = TEXT("history_miss"); return Result; }
	auto Shared = MakeShared<FURLabRenderCaptureJobSharedState, ESPMode::ThreadSafe>();
	Shared->State = EURLabRenderCaptureJobState::Succeeded;
	Shared->SceneRevision = Selected->SceneRevision;
	Shared->GeometryConfigurationId = Selected->GeometryConfigurationId;
	for (int32 QueryIndex = 0; QueryIndex < Query.Cameras.Num(); ++QueryIndex)
	{
		Shared->CameraNames.Add(Query.Cameras[QueryIndex].Name);
		Shared->Frames.Add(Selected->Frames[Indices[QueryIndex]]);
	}
	FURLabRenderCaptureJob Job;
	Job.Scheduler = this; Job.SchedulerLifetime = SchedulerLifetime;
	Job.JobSerial = NextJobSerial++; Job.State = Shared;
	Result.bAccepted = true; Result.Job = MoveTemp(Job); return Result;
}

bool URenderCameraSubsystem::ResolveExactCaptureHistory(const FURLabRenderHistoryQuery& Query,
	FURLabRenderExactCapturePlan& OutPlan, FRenderFrameIdentity& OutIdentity,
	FString& OutError) const
{
	check(IsInGameThread()); OutPlan = {}; OutIdentity = {};
	for (int32 GroupIndex = HistoryGroups.Num() - 1; GroupIndex >= 0; --GroupIndex)
	{
		const FHistoryGroup& Group = HistoryGroups[GroupIndex];
		if (Group.RetentionContext != Query.RetentionContext
			|| Group.SceneRevision != Query.ExpectedSceneRevision
			|| Group.GeometryConfigurationId != Query.GeometryConfigurationId
			|| !Group.Identity.bHasSourceTime
			|| Group.Identity.ClockEpoch != Query.ClockEpoch
			|| Group.Identity.SourceTime > Query.TargetSourceTime) continue;
		FURLabRenderExactCapturePlan Candidate; Candidate.RetentionContext = Query.RetentionContext;
		Candidate.Producer = Group.Producer; Candidate.SceneGeneration = Group.SceneGeneration;
		Candidate.SceneRevision = Group.SceneRevision;
		Candidate.GeometryConfigurationId = Group.GeometryConfigurationId;
		for (const FURLabRenderHistoryCameraQuery& Requested : Query.Cameras)
		{
			int32 Match = INDEX_NONE;
			for (int32 Index = 0; Index < Group.Names.Num(); ++Index)
				if (Group.Names[Index] == Requested.Name && Group.ConfigurationIds[Index] == Requested.ConfigurationId)
				{ Match = Index; break; }
			if (Match == INDEX_NONE) { Candidate.Cameras.Reset(); break; }
			Candidate.Cameras.Add(Group.Configurations[Match]);
			Candidate.AggregateLogicalPixelBytes += Group.Configurations[Match].LogicalPixelBytes;
			Candidate.AggregatePhysicalArrayBytes += Group.Configurations[Match].PhysicalArrayBytes;
		}
		if (Candidate.Cameras.Num() == Query.Cameras.Num())
		{
			OutPlan = MoveTemp(Candidate); OutIdentity = Group.Identity; OutError.Reset(); return true;
		}
	}
	OutError = TEXT("history_miss"); return false;
}

void URenderCameraSubsystem::RetireExactCaptureHistory(const FGuid& RetentionContext)
{
	check(IsInGameThread());
	for (int32 Index = HistoryGroups.Num() - 1; Index >= 0; --Index)
	{
		if (HistoryGroups[Index].RetentionContext != RetentionContext) continue;
		HistoryPhysicalBytes -= HistoryGroups[Index].PhysicalBytes;
		HistoryGroups.RemoveAt(Index, 1, EAllowShrinking::No);
	}
}

bool URenderCameraSubsystem::RetainExactCaptureHistory(
	const FURLabRenderExactCapturePlan& Plan, const FRenderFrameIdentity& Identity,
	const TArray<FRenderCameraFramePtr>& Frames)
{
	check(IsInGameThread());
	if (!Plan.RetentionContext.IsValid() || Plan.Cameras.Num() != Frames.Num()
		|| Plan.AggregatePhysicalArrayBytes < 0
		|| Plan.AggregatePhysicalArrayBytes > MaxHistoryPhysicalBytes) return false;
	FHistoryGroup Group; Group.RetentionContext = Plan.RetentionContext;
	Group.SceneRevision = Plan.SceneRevision; Group.Producer = Plan.Producer;
	Group.SceneGeneration = Plan.SceneGeneration; Group.Identity = Identity;
	Group.GeometryConfigurationId = Plan.GeometryConfigurationId;
	Group.PhysicalBytes = Plan.AggregatePhysicalArrayBytes; Group.Configurations = Plan.Cameras;
	Group.Frames = Frames;
	for (const FRenderCameraEffectiveConfiguration& Camera : Plan.Cameras)
	{
		Group.Names.Add(Camera.CanonicalName);
		Group.ConfigurationIds.Add(RenderCameraConfigurationId(Camera));
	}
	while (!HistoryGroups.IsEmpty() && (HistoryGroups.Num() >= MaxHistoryGroups
		|| HistoryPhysicalBytes + Group.PhysicalBytes > MaxHistoryPhysicalBytes))
	{
		HistoryPhysicalBytes -= HistoryGroups[0].PhysicalBytes;
		HistoryGroups.RemoveAt(0, 1, EAllowShrinking::No);
	}
	HistoryPhysicalBytes += Group.PhysicalBytes; HistoryGroups.Add(MoveTemp(Group)); return true;
}

#if WITH_DEV_AUTOMATION_TESTS
bool URenderCameraSubsystem::SeedExactCaptureHistoryForTest(
	const FURLabRenderExactCapturePlan& Plan, const FRenderFrameIdentity& Identity,
	const TArray<FRenderCameraFramePtr>& Frames)
{
	return RetainExactCaptureHistory(Plan, Identity, Frames);
}
#endif

void URenderCameraSubsystem::PumpDrainingJobs()
{
	for (int32 Index = DrainingJobs.Num() - 1; Index >= 0; --Index)
	{
		FQueuedJob& Job = *DrainingJobs[Index];
		if (!ReceiptsDrained(Job.Receipts)) continue;
		EURLabRenderCaptureJobState TerminalState = Job.PendingTerminalState;
		UURLabRenderWorldSubsystem* JobCoordinator = Job.Coordinator.Get();
		const bool bOriginalSceneStillActive = JobCoordinator
			&& JobCoordinator->ResolveActiveSceneRevision(Job.Producer) == Job.SceneRevision
			&& JobCoordinator->ResolveActiveSceneGeneration(Job.Producer) == Job.SceneGeneration;
		if (Job.bPendingRestorePresentation && !bOriginalSceneStillActive)
		{
			// A successful scene replacement already retired the presentation owned by
			// this job. Keep the cancelled writers alive through drain, but never apply
			// old-scene restore tokens to the successor scene.
			Job.OverrideTokens.Reset();
			Job.GeomOverrideToken = {};
			Job.Hold = {};
			Job.bPendingRestorePresentation = false;
		}
		for (int32 CameraIndex = Job.OverrideTokens.Num() - 1; CameraIndex >= 0; --CameraIndex)
		{
			FString RestoreError;
			URenderCameraComponent* Camera = Job.Cameras.IsValidIndex(CameraIndex)
				? Job.Cameras[CameraIndex].Get() : nullptr;
			if (!Camera || !Camera->RestoreCaptureOverride(
				Job.OverrideTokens[CameraIndex], RestoreError))
			{
				TerminalState = EURLabRenderCaptureJobState::Failed;
				if (Job.PendingMessage.IsEmpty()) Job.PendingMessage = RestoreError;
			}
		}
		Job.OverrideTokens.Reset();
		if (Job.bPendingRestorePresentation && Job.GeomOverrideToken.IsValid())
		{
			FString RestoreError;
			AURLabRenderScene* Scene = Job.Coordinator.IsValid()
				? Job.Coordinator->ResolveActiveScene(Job.Producer) : nullptr;
			if (!Scene || !Scene->RestoreGeomPoseOverrides(Job.GeomOverrideToken, RestoreError))
			{
				TerminalState = EURLabRenderCaptureJobState::Failed;
				if (Job.PendingMessage.IsEmpty())
					Job.PendingMessage = RestoreError.IsEmpty()
						? TEXT("geometry override restoration failed") : MoveTemp(RestoreError);
			}
			Job.GeomOverrideToken = {};
		}
		if (Job.bPendingRestorePresentation && Job.Hold.IsValid())
		{
			if (UURLabRenderWorldSubsystem* Coordinator = Job.Coordinator.Get())
			{
				FString RestoreError;
				if (!Coordinator->ReleasePresentationHold(Job.Producer, Job.Hold, RestoreError)
					&& Job.PendingMessage.IsEmpty()) Job.PendingMessage = MoveTemp(RestoreError);
			}
		}
		if (!Job.bTerminalPublished)
		{
			SetJobState(Job.Shared, TerminalState, Job.PendingAffectedCamera,
				Job.PendingMessage, true);
			QueueNotification(Job);
			Job.bTerminalPublished = true;
		}
		{
			FScopeLock Lock(&Job.Shared->Lock);
			Job.Shared->bGpuWritersDrained = true;
		}
		Job.LifetimeLease.Reset();
		DrainingJobs.RemoveAt(Index, 1, EAllowShrinking::No);
	}
}

bool URenderCameraSubsystem::CancelExactCapture(
	const FURLabRenderCaptureJob& Job, FString& OutError)
{
	OutError.Reset();
	if (!IsInGameThread())
	{
		OutError = TEXT("exact capture cancellation must run on the game thread");
		return false;
	}
	if (!Job.IsValid() || Job.Scheduler.Get() != this
		|| Job.SchedulerLifetime != SchedulerLifetime)
	{
		OutError = TEXT("exact capture job handle is stale or belongs to another scheduler");
		return false;
	}
	for (int32 Index = 0; Index < QueuedJobs.Num(); ++Index)
	{
		if (QueuedJobs[Index]->Serial != Job.JobSerial) continue;
		MarkQueuedTerminal(*QueuedJobs[Index], EURLabRenderCaptureJobState::Cancelled,
			TEXT("exact capture job was cancelled before execution"));
		DrainingJobs.Add(MoveTemp(QueuedJobs[Index]));
		QueuedJobs.RemoveAt(Index, 1, EAllowShrinking::No);
		return true;
	}
	if (ActiveJob.IsValid() && ActiveJob->Serial == Job.JobSerial)
	{
		FinishActive(EURLabRenderCaptureJobState::Cancelled, FString(),
			TEXT("exact capture job was cancelled"), true);
		return true;
	}
	OutError = TEXT("exact capture job is already terminal");
	return false;
}

void URenderCameraSubsystem::CancelJobsForSceneMutation(
	UURLabRenderWorldSubsystem* Coordinator, uint64 ProducerSerial,
	uint64 SceneGeneration, const FString& SceneRevision, bool bRestorePresentation)
{
	check(IsInGameThread());
	for (int32 Index = QueuedJobs.Num() - 1; Index >= 0; --Index)
	{
		if (!JobMatchesScene(*QueuedJobs[Index], Coordinator, ProducerSerial,
			SceneGeneration, SceneRevision)) continue;
		MarkQueuedTerminal(*QueuedJobs[Index], EURLabRenderCaptureJobState::Cancelled,
			TEXT("exact capture job was cancelled because its scene is retiring"));
		DrainingJobs.Add(MoveTemp(QueuedJobs[Index]));
		QueuedJobs.RemoveAt(Index, 1, EAllowShrinking::No);
	}
	if (ActiveJob.IsValid() && JobMatchesScene(*ActiveJob, Coordinator,
		ProducerSerial, SceneGeneration, SceneRevision))
	{
		FinishActive(EURLabRenderCaptureJobState::Cancelled, FString(),
			TEXT("exact capture job was cancelled because its scene is retiring"),
			bRestorePresentation);
	}
}

void URenderCameraSubsystem::DrainNotifications()
{
	check(IsInGameThread());
	TArray<TUniqueFunction<void()>> Notifications = MoveTemp(PendingNotifications);
	for (TUniqueFunction<void()>& Notification : Notifications)
		if (Notification) Notification();
}

void URenderCameraSubsystem::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	if (bTickingScheduler || bShuttingDown) return;
	bTickingScheduler = true;
	for (int32 Index = Cameras.Num() - 1; Index >= 0; --Index)
	{
		if (URenderCameraComponent* Camera = Cameras[Index].Get())
		{
			const AURLabRenderScene* Scene = Cast<AURLabRenderScene>(Camera->GetOwner());
			Camera->UpdateCapturePipeline(!Scene
				|| Scene->AreRenderResourcesReadyForCapture());
		}
		else
			Cameras.RemoveAtSwap(Index, EAllowShrinking::No);
	}
	PumpDrainingJobs();
	const double Now = ReadMonotonicSeconds();
	if (FMath::IsFinite(Now))
	{
		StartNextJob(Now);
		PumpActiveJob(Now);
	}
	bTickingScheduler = false;
	DrainNotifications();
}

void URenderCameraSubsystem::Deinitialize()
{
	if (!bShuttingDown)
	{
		bShuttingDown = true;
		for (FQueuedJobPtr& Job : QueuedJobs)
		{
			MarkQueuedTerminal(*Job, EURLabRenderCaptureJobState::Cancelled,
				TEXT("render camera scheduler shut down before job execution"));
			DrainingJobs.Add(MoveTemp(Job));
		}
		QueuedJobs.Reset();
		if (ActiveJob.IsValid())
			FinishActive(EURLabRenderCaptureJobState::Cancelled, FString(),
				TEXT("render camera scheduler shut down during capture"), true);
		// Receipt completion states, frames and their neutral leases outlive this
		// UObject. No callback captures the subsystem or a camera component.
		Cameras.Reset();
		SchedulerLifetime = FGuid::NewGuid();
		// Deinitialize may itself be reached from a coordinator mutation callback.
		// The task owns records as well as workflow closures: callbacks run first
		// while their capacity leases are still alive, then records retire. Accepted
		// camera receipt states independently retain leases until their writers drain.
		if (!PendingNotifications.IsEmpty() || !DrainingJobs.IsEmpty())
		{
			auto DeferredNotifications =
				MakeShared<TArray<TUniqueFunction<void()>>, ESPMode::ThreadSafe>(
				MoveTemp(PendingNotifications));
			auto DeferredRecords = MakeShared<TArray<FQueuedJobPtr>, ESPMode::ThreadSafe>(
				MoveTemp(DrainingJobs));
			FFunctionGraphTask::CreateAndDispatchWhenReady(
				[DeferredNotifications, DeferredRecords]() mutable
			{
				for (TUniqueFunction<void()>& Notification : *DeferredNotifications)
					if (Notification) Notification();
				DeferredNotifications->Reset();
				DeferredRecords->Reset();
			}, TStatId(), nullptr, ENamedThreads::GameThread);
		}
	}
	Super::Deinitialize();
}

TStatId URenderCameraSubsystem::GetStatId() const
{
	RETURN_QUICK_DECLARE_CYCLE_STAT(URenderCameraSubsystem, STATGROUP_Tickables);
}

bool URenderCameraSubsystem::DoesSupportWorldType(EWorldType::Type WorldType) const
{
	return WorldType == EWorldType::Game || WorldType == EWorldType::PIE
		|| WorldType == EWorldType::Editor;
}
