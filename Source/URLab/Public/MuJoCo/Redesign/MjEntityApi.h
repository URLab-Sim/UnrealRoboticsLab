// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"

class AActor;

/**
 * Typed, resolve-once handles for addressing an entity's parts. Each carries the resolved id + a
 * weak entity ref, so per-call use is id-based (no string lookup, no silent miss). Author-time these
 * are DROPDOWN-picked, not free-typed.
 *
 * STEP 0 freezes the SHAPE as plain C++. Phase 9 promotes these to USTRUCT(BlueprintType) + adds the
 * UINTERFACE reflection and the Blueprint picker customization; the string identity underneath is
 * what the model + Python wire use.
 */
struct URLAB_API FMjJoint
{
	int32 Id = -1;
	TWeakObjectPtr<AActor> Entity;
	float Pos() const;
	float Vel() const;
};

struct URLAB_API FMjActuator
{
	int32 Id = -1;
	TWeakObjectPtr<AActor> Entity;
	void SetCtrl(double Value) const;
};

struct URLAB_API FMjGeom
{
	int32 Id = -1;
	TWeakObjectPtr<AActor> Entity;
};

/**
 * The ONE entity API surface (C++ + Blueprint + mirrored in Python). Resolve a part by name ONCE to
 * a handle, then use the handle. Implemented by the runtime entity face (thin actor) -- authored
 * logic and scene-wide Level-BP logic both script against this.
 */
class URLAB_API IMjEntityApi
{
public:
	virtual ~IMjEntityApi() = default;

	virtual FMjJoint    Joint(FName Name) const = 0;
	virtual FMjActuator Actuator(FName Name) const = 0;
	virtual FMjGeom     Geom(FName Name) const = 0;
};
