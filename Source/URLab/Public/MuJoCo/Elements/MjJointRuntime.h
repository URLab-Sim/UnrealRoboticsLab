// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// Reading and driving a joint, and reading a tendon.
//
// Same shape as the sensor library and for the same reason: <joint> and
// <freejoint> are separate element classes, <tendon> is a section whose real
// elements are <spatial> and <fixed>, and none of them share a base that could
// carry this. What they do share is that every one of these operations is a
// function of the compiled id and the model -- qpos through jnt_qposadr, qvel
// through jnt_dofadr, tendon length through ten_length -- so none of it needs
// to live on a component.
//
// The single-slot accessors read the joint's FIRST qpos or dof slot. That is
// the whole story for a hinge or a slide and only part of it for a ball or a
// free joint, which is why the state IR carries the full width instead. These
// stay scalar because that is what a Blueprint slider and a details row want.

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "MjJointRuntime.generated.h"

class UMjNodeComponent;

/**
 * Type classification for authored joint elements. Runtime joint state is
 * accessed through the model-wide simulation service.
 */
UCLASS()
class URLAB_API UMjJointRuntime : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/** True when `Node` is a `<joint>` or `<freejoint>` element. */
	static bool IsJoint(const UMjNodeComponent* Node);
};

/**
 * Type classification for authored tendon elements.
 */
UCLASS()
class URLAB_API UMjTendonRuntime : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/** True when `Node` is a `<spatial>` or `<fixed>` tendon element. */
	static bool IsTendon(const UMjNodeComponent* Node);
};
