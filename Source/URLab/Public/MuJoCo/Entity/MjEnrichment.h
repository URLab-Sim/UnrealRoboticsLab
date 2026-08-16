// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"

enum class EMjEnrichmentScope : uint8
{
	Entity,
	Scene
};

/**
 * The ONE side-channel for data that is NOT an mjData id slice: twist echo, user channels, scene
 * producers, and compiled camera/controller handshake metadata. Keyed by entity name, off the
 * per-step core path, consumed by the ROS layer and opt-in clients. Sparse (0..few), never one per
 * joint.
 */
struct URLAB_API FMjEnrichment
{
	FName EntityName;
	TWeakObjectPtr<UObject> Producer;
	EMjEnrichmentScope Scope = EMjEnrichmentScope::Entity;
};
