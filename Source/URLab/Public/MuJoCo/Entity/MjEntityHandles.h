// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"

#include "MjEntityHandles.generated.h"

class AActor;

// Blueprint mirrors of the frozen C++ handles (MjEntityApi.h). The frozen handles are plain structs
// with methods, which reflection cannot see; these carry the same two fields as reflected USTRUCTs so
// a Blueprint can hold a resolved handle and pass it to the entity function library, which rebuilds
// the frozen handle to run the actual read or write.

/** A resolved joint handle: the compiled joint id and the entity face that resolved it. */
USTRUCT(BlueprintType)
struct URLAB_API FMjJointHandle
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category = "MuJoCo|Entity")
	int32 Id = -1;

	UPROPERTY()
	TWeakObjectPtr<AActor> Entity;
};

/** A resolved actuator handle: the compiled actuator id and the entity face that resolved it. */
USTRUCT(BlueprintType)
struct URLAB_API FMjActuatorHandle
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category = "MuJoCo|Entity")
	int32 Id = -1;

	UPROPERTY()
	TWeakObjectPtr<AActor> Entity;
};

/** A resolved geom handle: the compiled geom id and the entity face that resolved it. */
USTRUCT(BlueprintType)
struct URLAB_API FMjGeomHandle
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category = "MuJoCo|Entity")
	int32 Id = -1;

	UPROPERTY()
	TWeakObjectPtr<AActor> Entity;
};
