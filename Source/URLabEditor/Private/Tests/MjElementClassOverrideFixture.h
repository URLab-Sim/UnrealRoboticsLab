// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// Stand-ins for the hand element subclasses, so the override registry can be
// tested without waiting on the classes that will use it.
//
// They carry a UPROPERTY apiece for the same reason the real subclasses will:
// the whole point of a subclass over a function library is per-instance state
// the reflection system has to see. What they must not carry is any schema
// attribute -- the generated base owns all of those, and adding one here would
// be authoring a field the reader and writer know nothing about.
//
// Not guarded on URLAB_MJ_GEN: a UCLASS behind a directive UHT does not
// evaluate is reflected anyway and then absent from the translation unit, which
// is the one failure mode worse than not compiling. The generated element
// headers are checked in and depend on nothing but the engine, so including
// them unconditionally is safe where including the profile would not be.

#include "CoreMinimal.h"

#include "MuJoCo/Gen/Elements/Bodies/MjBody.gen.h"
#include "MuJoCo/Gen/Elements/Geometry/MjGeom.gen.h"

#include "MjElementClassOverrideFixture.generated.h"

UCLASS()
class UMjTestPresentationGeom : public UMjGeomBase
{
	GENERATED_BODY()

public:
	UPROPERTY(Transient)
	TObjectPtr<USceneComponent> Visualizer;
};

UCLASS()
class UMjTestPresentationBody : public UMjBodyBase
{
	GENERATED_BODY()

public:
	UPROPERTY(Transient)
	FVector MeshPivotOffset = FVector::ZeroVector;
};
