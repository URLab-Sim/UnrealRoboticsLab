// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
#pragma once

#include "CoreMinimal.h"
#include "Bridge/URLabPayloadStore.h"
#include "Render/RenderCameraSubsystem.h"

class FURLabShmExactRegion;
class FURLabCapturePayloadAdmission;
class FURLabCapturePayloadResultLease;

struct URLAB_API FURLabCapturePayloadCameraMetadata
{
	FString Name;
	ERenderCameraMode Mode = ERenderCameraMode::Real;
	FIntPoint Resolution = FIntPoint::ZeroValue;
	FRenderCameraOptics Optics;
	float DepthNearCm = 0.0f;
	float DepthFarCm = 0.0f;
	int64 ByteOffset = 0;
	int64 ByteLength = 0;
	uint64 ConfigurationRevision = 0;
	uint64 BaselineConfigurationRevision = 0;
	FString ConfigurationId;
	uint64 CaptureSerial = 0;
};

struct URLAB_API FURLabCapturePayloadMaterialization
{
	bool bSucceeded = false;
	FRenderFrameIdentity Identity;
	TArray<FURLabCapturePayloadCameraMetadata> Cameras;
	FURLabStoredPayloadDescriptor Descriptor;
	FURLabPayloadPin Pin;
	FString Error;
};

struct URLAB_API FURLabCapturePayloadAdmissionResult
{
	bool bAccepted = false;
	FRenderCameraCaptureLifetimeLeasePtr PixelLease;
	FRenderCameraCaptureLifetimeLeasePtr ResultLease;
	TSharedPtr<FURLabCapturePayloadAdmission, ESPMode::ThreadSafe> Admission;
	FString Error;
};

/** Bridge-owned exact capacity admission and one-shot all-camera payload materializer. */
class URLAB_API FURLabCapturePayloadAdmission
{
public:
	static FURLabCapturePayloadAdmissionResult Admit(
		TSharedRef<FURLabPayloadStore, ESPMode::ThreadSafe> Store,
		const FURLabPayloadOwner& Owner, EURLabPayloadBackingKind Backing,
		TSharedPtr<FURLabShmExactRegion, ESPMode::ThreadSafe> ShmRegion,
		const FURLabRenderExactCapturePlan& Plan);
	static FURLabCapturePayloadAdmissionResult AdmitRetained(
		TSharedRef<FURLabPayloadStore, ESPMode::ThreadSafe> Store,
		const FURLabPayloadOwner& Owner, EURLabPayloadBackingKind Backing,
		TSharedPtr<FURLabShmExactRegion, ESPMode::ThreadSafe> ShmRegion,
		const FURLabRenderExactCapturePlan& Plan);

	~FURLabCapturePayloadAdmission();
	FURLabCapturePayloadMaterialization Materialize(
		const FRenderFrameIdentity& SubmittedIdentity,
		const FURLabRenderCaptureJobSnapshot& Snapshot);
	void Abort();

	FURLabCapturePayloadAdmission(const FURLabCapturePayloadAdmission&) = delete;
	FURLabCapturePayloadAdmission& operator=(const FURLabCapturePayloadAdmission&) = delete;

private:
	friend class FURLabCapturePayloadResultLease;
	struct FState;
	explicit FURLabCapturePayloadAdmission(
		TSharedRef<FState, ESPMode::ThreadSafe> InState);
	TSharedRef<FState, ESPMode::ThreadSafe> State;
};
