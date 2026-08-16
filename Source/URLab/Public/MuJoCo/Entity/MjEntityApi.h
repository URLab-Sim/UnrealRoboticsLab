// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"

class AActor;

/**
 * Typed, resolve-once handles for addressing an entity's parts. Each carries the resolved id and a
 * weak entity ref, so per-call use is id-based (no string lookup, no silent miss). In the editor
 * these are picked from a dropdown rather than typed; the name is the identity the model and the
 * Python wire use.
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
 * The entity API surface (C++ and Blueprint, mirrored in Python). Resolve a part by name once to a
 * handle, then use the handle. Implemented by the runtime entity face; per-asset logic and
 * scene-wide logic both script against it.
 */
class URLAB_API IMjEntityApi
{
public:
	virtual ~IMjEntityApi() = default;

	virtual FMjJoint    Joint(FName Name) const = 0;
	virtual FMjActuator Actuator(FName Name) const = 0;
	virtual FMjGeom     Geom(FName Name) const = 0;
};
