// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "Bridge/RenderCapturePayloadAdapter.h"

#include "Render/RenderCameraEncoding.h"
#include "Transport/ShmExactPayload.h"

namespace
{
bool SameIdentity(const FRenderFrameIdentity& A, const FRenderFrameIdentity& B)
{
	return A.SceneRevision == B.SceneRevision && A.FrameId == B.FrameId
		&& A.SourceTime == B.SourceTime && A.ClockEpoch == B.ClockEpoch
		&& A.bHasSourceTime == B.bHasSourceTime;
}

class FPixelLease final : public IRenderCameraCaptureLifetimeLease
{
public:
	explicit FPixelLease(FURLabPayloadPhysicalReservationPtr InReservation)
		: Reservation(MoveTemp(InReservation)) {}
private:
	FURLabPayloadPhysicalReservationPtr Reservation;
};
}

struct FURLabCapturePayloadAdmission::FState
{
	~FState() { AbortInternal(); }

	void AbortInternal()
	{
		if (bAborted) return;
		bAborted = true;
		EncodedBytes.Empty();
		Cached.Reset();
		ReleaseProducer();
	}

	void ReleaseProducer()
	{
		if (Producer && !Producer->GetBlobId().IsEmpty())
		{
			// Active-session cleanup removes the exact staging/sealed catalog entry.
			// After session deactivation the dispatcher owns RevokeSession ordering;
			// dropping our handles then returns the remaining caller-held reservations.
			Store->Release(Owner, Producer->GetBlobId());
		}
		Producer.Reset();
	}

	TSharedRef<FURLabPayloadStore, ESPMode::ThreadSafe> Store;
	FURLabPayloadOwner Owner;
	FURLabRenderExactCapturePlan Plan;
	FURLabPayloadProducerPtr Producer;
	TArray<uint8> EncodedBytes;
	TOptional<FURLabCapturePayloadMaterialization> Cached;
	bool bAborted = false;

	FState(TSharedRef<FURLabPayloadStore, ESPMode::ThreadSafe> InStore,
		FURLabPayloadOwner InOwner, FURLabRenderExactCapturePlan InPlan,
		FURLabPayloadProducerPtr InProducer)
		: Store(MoveTemp(InStore)), Owner(MoveTemp(InOwner)), Plan(MoveTemp(InPlan)),
		  Producer(MoveTemp(InProducer)) {}
};

class FURLabCapturePayloadResultLease final : public IRenderCameraCaptureLifetimeLease
{
public:
	explicit FURLabCapturePayloadResultLease(
		TSharedRef<FURLabCapturePayloadAdmission::FState, ESPMode::ThreadSafe> InState)
		: State(MoveTemp(InState)) {}
private:
	TSharedRef<FURLabCapturePayloadAdmission::FState, ESPMode::ThreadSafe> State;
};

namespace
{
bool ValidatePlan(const FURLabRenderExactCapturePlan& Plan, FString& OutError)
{
	if (!Plan.Producer.IsValid() || Plan.SceneRevision.IsEmpty()
		|| Plan.SceneGeneration == 0 || Plan.Cameras.IsEmpty()
		|| Plan.Cameras.Num() > URenderCameraSubsystem::MaxCamerasPerExactCaptureJob)
	{
		OutError = TEXT("capture payload admission requires a complete exact plan");
		return false;
	}
	int64 Logical = 0;
	int64 Physical = 0;
	TSet<FString> Names;
	for (const FRenderCameraEffectiveConfiguration& Camera : Plan.Cameras)
	{
		int32 Elements = 0;
		int64 CameraLogical = 0;
		int64 CameraPhysical = 0;
		if (Camera.CanonicalName.IsEmpty() || Names.Contains(Camera.CanonicalName)
			|| Camera.ConfigurationRevision == 0)
		{
			OutError = TEXT("capture payload plan has an invalid camera identity");
			return false;
		}
		if (!RenderCheckedCameraBufferSizing(Camera.Mode, Camera.Resolution,
			Elements, CameraLogical, CameraPhysical, OutError)) return false;
		if (Camera.Mode == ERenderCameraMode::Depth
			&& (!FMath::IsFinite(Camera.DepthNearCm) || !FMath::IsFinite(Camera.DepthFarCm)
				|| Camera.DepthNearCm <= 0.0f || Camera.DepthFarCm <= Camera.DepthNearCm))
		{
			OutError = TEXT("capture payload depth plan has an invalid effective range");
			return false;
		}
		if (CameraLogical != Camera.LogicalPixelBytes
			|| CameraPhysical != Camera.PhysicalArrayBytes
			|| CameraLogical > MAX_int64 - Logical || CameraPhysical > MAX_int64 - Physical)
		{
			OutError = TEXT("capture payload plan contains forged or overflowing bounds");
			return false;
		}
		Names.Add(Camera.CanonicalName);
		Logical += CameraLogical;
		Physical += CameraPhysical;
	}
	if (Logical != Plan.AggregateLogicalPixelBytes
		|| Physical != Plan.AggregatePhysicalArrayBytes || Logical <= 0 || Physical <= 0
		|| Logical > MAX_int32)
	{
		OutError = TEXT("capture payload aggregate bounds do not match camera snapshots");
		return false;
	}
	return true;
}

FURLabCapturePayloadMaterialization Failed(FString Error)
{
	FURLabCapturePayloadMaterialization Result;
	Result.Error = MoveTemp(Error);
	return Result;
}
}

FURLabCapturePayloadAdmission::FURLabCapturePayloadAdmission(
	TSharedRef<FState, ESPMode::ThreadSafe> InState)
	: State(MoveTemp(InState))
{
}

FURLabCapturePayloadAdmission::~FURLabCapturePayloadAdmission() = default;

FURLabCapturePayloadAdmissionResult FURLabCapturePayloadAdmission::Admit(
	TSharedRef<FURLabPayloadStore, ESPMode::ThreadSafe> Store,
	const FURLabPayloadOwner& Owner, EURLabPayloadBackingKind Backing,
	TSharedPtr<FURLabShmExactRegion, ESPMode::ThreadSafe> ShmRegion,
	const FURLabRenderExactCapturePlan& Plan)
{
	check(IsInGameThread());
	FURLabCapturePayloadAdmissionResult Result;
	FString Error;
	if (!ValidatePlan(Plan, Error)) { Result.Error = MoveTemp(Error); return Result; }
	if (Backing != EURLabPayloadBackingKind::NetworkBytes
		&& Backing != EURLabPayloadBackingKind::ShmSealedFile)
	{
		Result.Error = TEXT("capture payload backing is unsupported");
		return Result;
	}
	if ((Backing == EURLabPayloadBackingKind::ShmSealedFile) != ShmRegion.IsValid())
	{
		Result.Error = TEXT("SHM capture payload admission requires exactly one explicit region");
		return Result;
	}
	FURLabPayloadPhysicalReservationPtr PixelReservation;
	FURLabPayloadStoreStatus Status = Store->ReservePhysicalCapacity(
		Owner, Plan.AggregatePhysicalArrayBytes, PixelReservation);
	if (!Status) { Result.Error = Status.Message; return Result; }
	FURLabPayloadProducerPtr Producer;
	if (Backing == EURLabPayloadBackingKind::NetworkBytes)
		Status = Store->BeginProduce(Owner, Plan.AggregateLogicalPixelBytes, Producer);
	else
		Status = Store->BeginShmProduce(Owner, ShmRegion,
			Plan.AggregateLogicalPixelBytes, Producer);
	if (!Status)
	{
		Result.Error = Status.Message;
		return Result;
	}
	TSharedRef<FState, ESPMode::ThreadSafe> State = MakeShared<FState, ESPMode::ThreadSafe>(
		Store, Owner, Plan, Producer);
	Result.PixelLease = MakeShared<FPixelLease, ESPMode::ThreadSafe>(MoveTemp(PixelReservation));
	Result.ResultLease = MakeShared<FURLabCapturePayloadResultLease, ESPMode::ThreadSafe>(State);
	Result.Admission = MakeShareable(new FURLabCapturePayloadAdmission(State));
	Result.bAccepted = true;
	return Result;
}

FURLabCapturePayloadAdmissionResult FURLabCapturePayloadAdmission::AdmitRetained(
	TSharedRef<FURLabPayloadStore, ESPMode::ThreadSafe> Store,
	const FURLabPayloadOwner& Owner, EURLabPayloadBackingKind Backing,
	TSharedPtr<FURLabShmExactRegion, ESPMode::ThreadSafe> ShmRegion,
	const FURLabRenderExactCapturePlan& Plan)
{
	check(IsInGameThread());
	FURLabCapturePayloadAdmissionResult Result; FString Error;
	if (!ValidatePlan(Plan, Error)) { Result.Error = MoveTemp(Error); return Result; }
	if ((Backing == EURLabPayloadBackingKind::ShmSealedFile) != ShmRegion.IsValid()
		|| (Backing != EURLabPayloadBackingKind::NetworkBytes
			&& Backing != EURLabPayloadBackingKind::ShmSealedFile))
	{
		Result.Error = TEXT("retained capture payload backing is invalid"); return Result;
	}
	FURLabPayloadProducerPtr Producer;
	FURLabPayloadStoreStatus Status = Backing == EURLabPayloadBackingKind::NetworkBytes
		? Store->BeginProduce(Owner, Plan.AggregateLogicalPixelBytes, Producer)
		: Store->BeginShmProduce(Owner, ShmRegion, Plan.AggregateLogicalPixelBytes, Producer);
	if (!Status) { Result.Error = Status.Message; return Result; }
	TSharedRef<FState, ESPMode::ThreadSafe> State = MakeShared<FState, ESPMode::ThreadSafe>(
		Store, Owner, Plan, Producer);
	Result.ResultLease = MakeShared<FURLabCapturePayloadResultLease, ESPMode::ThreadSafe>(State);
	Result.Admission = MakeShareable(new FURLabCapturePayloadAdmission(State));
	Result.bAccepted = true; return Result;
}

FURLabCapturePayloadMaterialization FURLabCapturePayloadAdmission::Materialize(
	const FRenderFrameIdentity& SubmittedIdentity,
	const FURLabRenderCaptureJobSnapshot& Snapshot)
{
	check(IsInGameThread());
	if (!State->Owner.Session || !State->Owner.Session->IsActive())
		return Failed(TEXT("capture payload owner session is inactive"));
	if (State->bAborted) return Failed(TEXT("capture payload admission is retired"));
	if (State->Cached.IsSet())
	{
		if (!SameIdentity(State->Cached->Identity, SubmittedIdentity))
			return Failed(TEXT("materialization identity differs from the retained outcome"));
		if (State->Cached->bSucceeded)
		{
			FURLabStoredPayloadDescriptor Current;
			const FURLabPayloadStoreStatus Status = State->Store->GetDescriptor(
				State->Owner, State->Cached->Descriptor.BlobId, Current);
			if (!Status) return Failed(Status.Message);
			State->Cached->Descriptor = MoveTemp(Current);
		}
		return State->Cached.GetValue();
	}
	if (SubmittedIdentity.SceneRevision != State->Plan.SceneRevision
		|| Snapshot.State != EURLabRenderCaptureJobState::Succeeded
		|| Snapshot.SceneRevision != State->Plan.SceneRevision
		|| Snapshot.CameraNames.Num() != State->Plan.Cameras.Num()
		|| Snapshot.Frames.Num() != State->Plan.Cameras.Num())
	{
		FURLabCapturePayloadMaterialization Failure = Failed(TEXT("capture snapshot is not an exact all-camera success"));
		Failure.Identity = SubmittedIdentity;
		State->Cached = MoveTemp(Failure);
		State->EncodedBytes.Empty();
		State->ReleaseProducer();
		return State->Cached.GetValue();
	}

	TArray<FURLabCapturePayloadCameraMetadata> Metadata;
	Metadata.Reserve(State->Plan.Cameras.Num());
	for (int32 Index = 0; Index < State->Plan.Cameras.Num(); ++Index)
	{
		const FRenderCameraEffectiveConfiguration& Planned = State->Plan.Cameras[Index];
		const FRenderCameraFramePtr& Frame = Snapshot.Frames[Index];
		if (Snapshot.CameraNames[Index] != Planned.CanonicalName || !Frame
			|| !SameIdentity(Frame->Identity, SubmittedIdentity)
			|| Frame->Mode != Planned.Mode || Frame->Width != Planned.Resolution.X
			|| Frame->Height != Planned.Resolution.Y
			|| Frame->CameraConfigurationRevision != Planned.ConfigurationRevision
			|| Frame->CaptureSerial == 0)
		{
			FURLabCapturePayloadMaterialization Failure = Failed(TEXT("capture frame identity or configuration mismatches admission"));
			Failure.Identity = SubmittedIdentity;
			State->Cached = MoveTemp(Failure);
			State->EncodedBytes.Empty();
			State->ReleaseProducer();
			return State->Cached.GetValue();
		}
		const int64 Pixels = static_cast<int64>(Frame->Width) * Frame->Height;
		const bool bCountsMatch = Planned.Mode == ERenderCameraMode::Real
			? Frame->Color.Num() == Pixels && Frame->Depth.IsEmpty() && Frame->Seg.IsEmpty()
			: (Planned.Mode == ERenderCameraMode::Depth
				? Frame->Depth.Num() == Pixels && Frame->Color.IsEmpty() && Frame->Seg.IsEmpty()
				: Frame->Seg.Num() == Pixels * 2 && Frame->Color.IsEmpty() && Frame->Depth.IsEmpty());
		if (!bCountsMatch)
		{
			FURLabCapturePayloadMaterialization Failure = Failed(TEXT("capture frame plane count mismatches admitted format"));
			Failure.Identity = SubmittedIdentity;
			State->Cached = MoveTemp(Failure);
			State->EncodedBytes.Empty();
			State->ReleaseProducer();
			return State->Cached.GetValue();
		}
		FURLabCapturePayloadCameraMetadata& Camera = Metadata.AddDefaulted_GetRef();
		Camera.Name = Planned.CanonicalName;
		Camera.Mode = Planned.Mode;
		Camera.Resolution = Planned.Resolution;
		Camera.Optics = Planned.Optics;
		Camera.DepthNearCm = Planned.DepthNearCm;
		Camera.DepthFarCm = Planned.DepthFarCm;
		Camera.ByteOffset = Index == 0 ? 0
			: Metadata[Index - 1].ByteOffset + Metadata[Index - 1].ByteLength;
		Camera.ByteLength = Planned.LogicalPixelBytes;
		Camera.ConfigurationRevision = Planned.ConfigurationRevision;
		Camera.BaselineConfigurationRevision = Planned.BaselineConfigurationRevision;
		Camera.ConfigurationId = RenderCameraConfigurationId(Planned);
		Camera.CaptureSerial = Frame->CaptureSerial;
	}

	if (State->EncodedBytes.IsEmpty())
	{
		State->EncodedBytes.Reserve(static_cast<int32>(State->Plan.AggregateLogicalPixelBytes));
		State->EncodedBytes.SetNumUninitialized(
			static_cast<int32>(State->Plan.AggregateLogicalPixelBytes));
		for (int32 Index = 0; Index < Snapshot.Frames.Num(); ++Index)
		{
			const FRenderCameraFrame& Frame = *Snapshot.Frames[Index];
			FString Encoding;
			int32 StrideBytes = 0;
			TArray<uint8> CameraBytes;
			FString EncodeError;
			if (!RenderEncodeCameraFrame(Frame, State->Plan.Cameras[Index], Encoding,
				StrideBytes, CameraBytes, EncodeError))
			{
				FURLabCapturePayloadMaterialization Failure = Failed(MoveTemp(EncodeError));
				Failure.Identity = SubmittedIdentity;
				State->Cached = MoveTemp(Failure);
				State->EncodedBytes.Empty();
				State->ReleaseProducer();
				return State->Cached.GetValue();
			}
			FMemory::Memcpy(State->EncodedBytes.GetData() + Metadata[Index].ByteOffset,
				CameraBytes.GetData(), CameraBytes.Num());
		}
		State->EncodedBytes.Shrink();
	}

	FURLabStoredPayloadDescriptor Descriptor;
	FURLabPayloadPin Pin;
	const FURLabPayloadStoreStatus Status = State->Store->SealProduced(
		State->Owner, State->Producer, MoveTemp(State->EncodedBytes), Descriptor, Pin);
	if (!Status)
	{
		FURLabCapturePayloadMaterialization Failure = Failed(Status.Message);
		Failure.Identity = SubmittedIdentity;
		State->Cached = Failure;
		// This bounded adapter treats every seal failure as terminal. Release removes
		// the exact producer while its session is live; RevokeSession owns cleanup
		// after deactivation and no single-job path revokes the whole session.
		State->EncodedBytes.Empty();
		State->ReleaseProducer();
		return State->Cached.GetValue();
	}
	FURLabCapturePayloadMaterialization Success;
	Success.bSucceeded = true;
	Success.Identity = SubmittedIdentity;
	Success.Cameras = MoveTemp(Metadata);
	Success.Descriptor = MoveTemp(Descriptor);
	Success.Pin = Pin;
	State->Cached = Success;
	return Success;
}

void FURLabCapturePayloadAdmission::Abort()
{
	check(IsInGameThread());
	State->AbortInternal();
}
