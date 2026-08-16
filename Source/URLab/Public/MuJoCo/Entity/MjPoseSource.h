// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"
#include "MjPoseSource.generated.h"

/**
 * The mode axis: where each frame's render pose comes from. Owner-ness (can advertise and serve) is
 * derived -- FreeRun/Stepped/StatePushed have live state, Mirror does not.
 */
UENUM(BlueprintType)
enum class EMjPoseSource : uint8
{
	FreeRun     UMETA(DisplayName = "Free-run (UE steps, own clock)"),
	Stepped     UMETA(DisplayName = "Stepped (UE steps on client request)"),
	StatePushed UMETA(DisplayName = "State-pushed (client integrates, UE mj_forward)"),
	Mirror      UMETA(DisplayName = "Mirror (draw streamed transforms, no physics)")
};
