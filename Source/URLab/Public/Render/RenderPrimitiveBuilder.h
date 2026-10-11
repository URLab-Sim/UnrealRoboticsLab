// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"

class AActor;
class UPrimitiveComponent;
struct FRenderGeomDesc;

/**
 * Build one engine primitive (and capsule cap children) from neutral scene data.
 * The descriptor is in Unreal centimetres. Invalid, unsupported, and mesh descriptors
 * return null. The material callback runs for every component that is created.
 */
URLAB_API UPrimitiveComponent* BuildNeutralPrimitive(const FRenderGeomDesc& Geom, AActor* Body,
	TFunctionRef<void(UPrimitiveComponent*)> ApplyMaterial);
