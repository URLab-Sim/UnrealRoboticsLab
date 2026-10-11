// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
#pragma once

#include "CoreMinimal.h"
#include "Bridge/URLabPayloadStore.h"
#include "Render/RenderScene.h"
#include "Render/RenderCameraTypes.h"
#include "Wire/RequestEnvelope.h"

/** Bridge-only decoder for the portable neutral render-source wire variant. */
class URLAB_API FURLabNeutralRenderSourceIngress
{
public:
	static constexpr int32 MaxDistinctPayloadRefs = 63;
	static constexpr int32 MaxNameUtf8Bytes = 1024;
	static constexpr double QuaternionNormTolerance = 1.e-6;

	static bool Prepare(
		const TSharedPtr<FURLabPayloadStore, ESPMode::ThreadSafe>& Store,
		const FURLabPayloadOwner& Owner,
		const FURLabWireValue& Source,
		const FString& ServerSceneRevision,
		FRenderSceneInstallPayload& OutPayload,
		FString& OutError);

	/** Decode owner-bound dynamic views against the installed deformable topology. */
	static bool DecodeFrameVisualization(
		const TSharedPtr<FURLabPayloadStore, ESPMode::ThreadSafe>& Store,
		const FURLabPayloadOwner& Owner,
		const FURLabWireValue& Visualization,
		const TArray<FRenderDeformableDesc>& Deformables,
		FRenderFrame& InOutFrame,
		FRenderCameraCaptureLifetimeLeasePtr& OutLease,
		FString& OutError);
};
