// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"

#include "MjLightHandle.generated.h"

/** How a light was authored. Drives what an override can meaningfully set: a
 *  directional light ignores position, a sky light ignores both position and
 *  direction. */
UENUM()
enum class EMjLightKind : uint8
{
	Directional,
	Point,
	Spot,
	Sky,
	/** A light that came from the level rather than the model. */
	Unknown,
};

URLAB_API const TCHAR* MjLightKindName(EMjLightKind Kind);

/**
 * The name tag that makes an imported light addressable.
 *
 * MuJoCo lights are spawned as plain UE light actors whose UE names are
 * uniquified spawn names -- nothing a client could address. This component
 * records the MJCF light name (and the compiled id) on the spawned actor, so a
 * client can say `set_light("key_light", ...)` in exactly the way it says
 * `set_geom_appearance("can_body", ...)`. It is a census marker only; it holds
 * no behaviour and ticks never.
 *
 * SceneOrigin is the offset the importer placed this light at. A position
 * override arrives in the MuJoCo frame, so replaying it has to add the same
 * origin back or a light in an offset scene instance would jump to the world
 * origin.
 */
UCLASS(ClassGroup = (MuJoCo))
class URLAB_API UMjLightHandle : public UActorComponent
{
	GENERATED_BODY()

public:
	UMjLightHandle();

	/** The MJCF <light> name, else a synthesised `light<i>` for an unnamed one.
	 *
	 *  Matching only. Do NOT report this to a client: `FName` is case-insensitive
	 *  and, in a build without WITH_CASE_PRESERVING_NAME (i.e. anything cooked),
	 *  `ToString()` returns whichever spelling entered the global name table
	 *  FIRST, not the one this was constructed with. A model light named `key`
	 *  in a level that already has an actor called `Key` comes back as `Key`,
	 *  and the client cannot address it by the name its own model gave it.
	 *  Editor builds define WITH_CASE_PRESERVING_NAME, so this is invisible
	 *  until the build is cooked -- which is how it shipped. */
	UPROPERTY(Transient)
	FName MjName;

	/** The MJCF <light> name exactly as `mj_id2name` spelled it.
	 *
	 *  This is what goes on the wire. Kept as an FString beside the FName above
	 *  because only the FString preserves case in a cooked build. */
	UPROPERTY(Transient)
	FString MjNameExact;

	/** The compiled light id, or INDEX_NONE for a light with no model row. */
	UPROPERTY(Transient)
	int32 MjId = INDEX_NONE;

	UPROPERTY(Transient)
	EMjLightKind Kind = EMjLightKind::Unknown;

	/** Scene origin the importer placed this light at (UE centimetres). */
	UPROPERTY(Transient)
	FVector SceneOrigin = FVector::ZeroVector;
};
