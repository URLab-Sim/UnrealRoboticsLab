// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
#pragma once

#include "CoreMinimal.h"
#include "Bridge/RenderSourceIngress.h"
#include "Render/RenderCameraTypes.h"
#include "Workflows/RenderServerWorldSubsystem.h"
#include "Wire/RequestEnvelope.h"

enum class EURLabRenderPoseKind : uint8
{
	Qpos,
	Cartesian,
};

struct URLAB_API FURLabRenderPoseRequest
{
	EURLabRenderPoseKind Kind = EURLabRenderPoseKind::Qpos;
	TOptional<FURLabRenderSourceBlobRef> Payload;
	TOptional<FURLabWireValue> Visualization;
	FRenderFrameIdentity Identity;
};

struct URLAB_API FURLabDecodedRenderPose
{
	FURLabDecodedRenderPose() = default;
	~FURLabDecodedRenderPose() = default;
	FURLabDecodedRenderPose(FURLabDecodedRenderPose&& Other) noexcept;
	FURLabDecodedRenderPose& operator=(FURLabDecodedRenderPose&& Other) noexcept;
	FURLabDecodedRenderPose(const FURLabDecodedRenderPose&) = delete;
	FURLabDecodedRenderPose& operator=(const FURLabDecodedRenderPose&) = delete;
	void Reset();

	// Declared first so reverse destruction releases decoded arrays before their charge.
	FRenderCameraCaptureLifetimeLeasePtr PresentationLease;
	EURLabRenderPoseKind Kind = EURLabRenderPoseKind::Qpos;
	TArray<double> Qpos;
	FRenderFrame Cartesian;
};

/** Exact immutable pose decoding and charged presentation lifetime composition. */
class URLAB_API FURLabRenderPoseIngress
{
public:
	static constexpr double QuaternionNormTolerance = 1.0e-6;

	static bool Decode(
		const TSharedPtr<FURLabPayloadStore, ESPMode::ThreadSafe>& Store,
		const FURLabPayloadOwner& Owner,
		const FURLabRenderSourceDescription& Description,
		const FURLabRenderPoseRequest& Request,
		FURLabDecodedRenderPose& OutPose,
		FString& OutError);

	static FRenderCameraCaptureLifetimeLeasePtr ComposeLifetimeLeases(
		FRenderCameraCaptureLifetimeLeasePtr First,
		FRenderCameraCaptureLifetimeLeasePtr Second);

	/** Reserve the conservative retained allocation bound for one Cartesian presentation. */
	static bool ReserveCartesianPresentation(
		const TSharedPtr<FURLabPayloadStore, ESPMode::ThreadSafe>& Store,
		const FURLabPayloadOwner& Owner, int32 BodyCount, int32 CameraCount,
		bool bHasUserCamera, int32 LightCount,
		FRenderCameraCaptureLifetimeLeasePtr& OutLease,
		FString& OutError);

	/** Reserve the exact decoded sparse geometry-override array before allocation. */
	static bool ReserveGeomPoseOverrides(
		const TSharedPtr<FURLabPayloadStore, ESPMode::ThreadSafe>& Store,
		const FURLabPayloadOwner& Owner, int32 Count,
		FRenderCameraCaptureLifetimeLeasePtr& OutLease, FString& OutError);
};
