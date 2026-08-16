// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"

#include "MjEntityPickers.generated.h"

class AActor;

// Authoring-time part references. A picker is an entity plus the name of one of its parts; the editor
// draws the name as a dropdown of that entity's members (see the URLabEditor customization) rather
// than a free-text field. At runtime the entity function library turns a picker into a resolved
// handle. The entity ref is soft so a picker survives an unloaded level.

/** A named joint of a referenced entity. */
USTRUCT(BlueprintType)
struct URLAB_API FMjJointPicker
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Entity")
	TSoftObjectPtr<AActor> Entity;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Entity")
	FName Name;
};

/** A named actuator of a referenced entity. */
USTRUCT(BlueprintType)
struct URLAB_API FMjActuatorPicker
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Entity")
	TSoftObjectPtr<AActor> Entity;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Entity")
	FName Name;
};

/** A named geom of a referenced entity. */
USTRUCT(BlueprintType)
struct URLAB_API FMjGeomPicker
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Entity")
	TSoftObjectPtr<AActor> Entity;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Entity")
	FName Name;
};
