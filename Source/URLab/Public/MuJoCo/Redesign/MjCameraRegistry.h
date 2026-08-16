// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"

struct mjModel_;
typedef mjModel_ mjModel;
struct FMjEntity;

/** One camera's stable identity, decoupled from Cast<AMjArticulation>. */
struct URLAB_API FMjCameraInfo
{
	int32 CameraId = -1;
	FName CanonicalName;  // entity prefix + camera name; stable ZMQ topic / SHM stem
	FName EntityName;     // owning entity
};

/**
 * Renderer-agnostic camera registry. Canonical camera identity is computed once from the model +
 * the entity partition, so ANY renderer (compiled actor OR lightweight body actor) produces the
 * same topics/stems -- the RPC camera surface enumerates THIS, not GetAllArticulations.
 */
struct URLAB_API FMjCameraRegistry
{
	TArray<FMjCameraInfo> Cameras;

	void Build(const mjModel* Model, const TArray<FMjEntity>& Entities);
	const FMjCameraInfo* Find(FName Canonical) const;
};
