// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"

class UMjPhysicsEngine;

/** Which of an entity's id slices a name is resolved against. */
enum class EMjEntityMember : uint8
{
	Joint,
	Actuator,
	Geom
};

/**
 * Turning a member name into the compiled id that answers to it, within one entity.
 *
 * The compiled model names an element by its participant prefix ("panda_shoulder"); a caller means it
 * by the short name ("shoulder"). These resolve either form against the entity's own id slice, so a
 * name only ever matches inside the entity that owns it. Geoms have no slice of their own, so an
 * entity's geoms are the geoms of the bodies it owns.
 */
namespace MjEntityMembers
{
	/**
	 * The pickable member names of one family within an entity, in slice order: the prefix-stripped
	 * short name when the compiled name carries the entity's prefix, else the compiled name. Empty
	 * when there is no compiled model or the entity is unknown.
	 */
	URLAB_API TArray<FName> Names(const UMjPhysicsEngine* Engine, FName EntityName, EMjEntityMember Family);

	/**
	 * The global compiled id a member name resolves to within an entity's slice, or -1. Accepts the
	 * short name or the full compiled name.
	 */
	URLAB_API int32 ResolveId(const UMjPhysicsEngine* Engine, FName EntityName, EMjEntityMember Family,
		FName Member);
}
