// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"
#include "MjPoseSource.generated.h"

/**
 * The ONE mode axis: where each frame's render pose comes from. Replaces EStepMode + EMjbRunMode.
 * Owner-ness (can advertise + serve) is DERIVED (FreeRun/Stepped/StatePushed have live state; Mirror
 * does not). Auto is a launch policy, not a value.
 */
UENUM(BlueprintType)
enum class EMjPoseSource : uint8
{
	FreeRun     UMETA(DisplayName = "Free-run (UE steps, own clock)"),
	Stepped     UMETA(DisplayName = "Stepped (UE steps on client request)"),
	StatePushed UMETA(DisplayName = "State-pushed (client integrates, UE mj_forward)"),
	Mirror      UMETA(DisplayName = "Mirror (draw streamed transforms, no physics)")
};

/**
 * Composable, OPEN capability set on an instance -- NOT modes. New features are added as
 * capabilities, never as a new mode. "Render server" = StreamCameras; "viewer" = AcceptInput.
 */
enum class EMjCapability : uint8
{
	None          = 0,
	StreamCameras = 1 << 0,  // publish frames from model cameras and/or own view
	AcceptInput   = 1 << 1,  // xfrc / wrench / drag / requests (applied locally or forwarded to owner)
};
ENUM_CLASS_FLAGS(EMjCapability);
