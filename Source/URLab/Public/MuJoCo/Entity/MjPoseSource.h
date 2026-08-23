// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"
#include "MjPoseSource.generated.h"

/**
 * Legacy producer pose axis. Phase 1.4 split this: the engine's producer step-mode moved onto
 * EMjStepMode (below) and the renderer's consumer notion moved onto EMjDrive (on AMjRenderer), so
 * `Mirror` -- the consumer value that was never a legitimate engine value -- is gone. With that split
 * complete this enum has NO remaining references; it is retained (minus `Mirror`) only so its full
 * deletion can be sequenced separately.
 */
UENUM(BlueprintType)
enum class EMjPoseSource : uint8
{
	FreeRun     UMETA(DisplayName = "Free-run (UE steps, own clock)"),
	Stepped     UMETA(DisplayName = "Stepped (UE steps on client request)"),
	StatePushed UMETA(DisplayName = "State-pushed (client integrates, UE mj_forward)")
};

/**
 * Producer-only step-mode: how a `sim` (Producer) instance advances its own physics state. A
 * consumer (drive=stream|push) has no step-mode -- it has a pose sink, not an engine to advance.
 *
 * The engine's resolved step-mode axis (UMjPhysicsEngine::ResolvedPoseSource, the RPC dispatcher's
 * ActiveStepMode, AMjManager::StepMode) is typed on this since the Phase 1.4 enum split. It drops
 * `Mirror` -- never a legitimate producer/engine value; the consumer notion it named now lives on
 * EMjDrive (stream/push) on AMjRenderer.
 *
 * - FreeRun: steps on its own clock.
 * - Stepped: steps only on a client step request (any transport).
 * - StatePushed: the client integrates elsewhere and pushes qpos/qvel/ctrl/time; the engine runs
 *   mj_forward only (no stepping).
 */
UENUM(BlueprintType)
enum class EMjStepMode : uint8
{
	FreeRun     UMETA(DisplayName = "Free-run (steps on own clock)"),
	Stepped     UMETA(DisplayName = "Stepped (steps on client request)"),
	StatePushed UMETA(DisplayName = "State-pushed (client integrates, engine mj_forward)")
};

/**
 * Composable, OPEN capability set on an instance -- NOT modes. New features are added as
 * capabilities, never as a new mode. "Render server" = StreamCameras; "viewer" = AcceptInput.
 * Capabilities compose freely with any Drive/step-mode: a stream/push consumer can also stream its
 * own view and accept input at the same time, with no new mode.
 */
UENUM(BlueprintType, meta = (Bitflags))
enum class EMjCapability : uint8
{
	None          = 0,
	StreamCameras = 1 << 0,  // publish frames from model cameras and/or own view
	AcceptInput   = 1 << 1,  // xfrc / wrench / drag / requests (applied locally or forwarded to owner)
};
ENUM_CLASS_FLAGS(EMjCapability);
