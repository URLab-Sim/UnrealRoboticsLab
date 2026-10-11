// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
#pragma once

#include "CoreMinimal.h"
#include "Bridge/URLabPayloadStore.h"
#include "MuJoCo/Render/MjRenderKinematics.h"

struct URLAB_API FURLabRenderSourceBlobRef
{
	FString BlobId;
	FString Sha256;
};

struct URLAB_API FURLabRenderSourceAssetRef
{
	FString Name;
	FURLabRenderSourceBlobRef Payload;
};

struct URLAB_API FURLabRenderSourceRequest
{
	EMjModelSourceKind Kind = EMjModelSourceKind::Mjb;
	FURLabRenderSourceBlobRef Primary;
	TArray<FURLabRenderSourceAssetRef> Assets;
	FString Label;
};

/** Bridge-owned decoding of exact immutable payload references into a prepared candidate. */
class URLAB_API FURLabRenderSourceIngress
{
public:
	static constexpr int32 MaxSourceRefs = 63;
	static constexpr int32 MaxLabelChars = 1024;
	static constexpr int32 MaxAssetNameChars = 1024;

	static bool Prepare(
		const TSharedPtr<FURLabPayloadStore, ESPMode::ThreadSafe>& Store,
		const FURLabPayloadOwner& Owner,
		const FURLabRenderSourceRequest& Request,
		const FString& SceneRevision,
		const FMjRenderKinematicsOptions& Options,
		FMjRenderKinematicsCandidate& OutCandidate,
		FString& OutError);
};
