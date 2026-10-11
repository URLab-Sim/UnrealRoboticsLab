// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "Transport/TransportBase.h"
#include "CameraPublishTransport.generated.h"

/**
 * One camera frame handed to a transport for egress. Pixels are raw -- BGRA8
 * for colour, float32 for depth -- and the buffer is BORROWED: a backend that
 * ships asynchronously copies anything it needs beyond the call. The metadata
 * mirrors the per-camera wire header the camera workers emit.
 */
struct FMjCameraWireFrame
{
	int32 CameraIndex = 0;             // stable per-renderer channel id
	int32 Width = 0;
	int32 Height = 0;
	uint64 FrameId = 0;                // the post-step frame this image shows
	double SimTime = 0.0;              // mujoco.time of that frame (seconds)
	double CaptureUnixSeconds = 0.0;   // wall clock at capture, for the wire meta
	FName Dtype;                       // "bgra8" | "float32" | "int32"
	// Components per pixel: 4 (BGRA), 1 (depth), 2 (segmentation objid+objtype).
	// Dtype alone stops being a stride once one dtype serves two shapes.
	int32 Channels = 4;
	const uint8* Bytes = nullptr;      // borrowed pixel buffer
	int32 NumBytes = 0;
};

/**
 * @class UURLabCameraPublishTransport
 * @brief Per-camera IMAGE egress -- the role UURLabPublishTransport excludes.
 *
 * Camera streaming does not fit the small/frequent `Publish(topic, bytes)`
 * shape: frames are ~MB, each camera has its own lifetime, and delivery runs
 * off a per-camera GPU-readback worker rather than the physics step.
 *
 * Channel model: one transport instance multiplexes N cameras, keyed by
 * `CameraIndex`. A per-socket backend (ZMQ) may stand up a socket/thread per
 * channel internally; a single-connection backend may fold every channel onto
 * one stream. The renderer sees neither.
 *
 * Threading: `PublishCameraFrame` is called from a camera's publish worker, OFF
 * the game thread; concrete backends serialise internally.
 */
UCLASS(Abstract)
class URLAB_API UURLabCameraPublishTransport : public UURLabTransportBase
{
	GENERATED_BODY()

public:
	/** Register a camera channel before any frame flows (stream start, game
	 *  thread). `CanonicalName` is the wire identity. */
	virtual void OpenCameraChannel(int32 CameraIndex, const FString& CanonicalName)
		PURE_VIRTUAL(UURLabCameraPublishTransport::OpenCameraChannel, );

	/** Ship one frame for `Frame.CameraIndex`. Called off the game thread; the
	 *  backend copies anything it needs beyond this call. */
	virtual void PublishCameraFrame(const FMjCameraWireFrame& Frame)
		PURE_VIRTUAL(UURLabCameraPublishTransport::PublishCameraFrame, );

	/** Close one camera channel (stream stop or camera destroyed). Idempotent. */
	virtual void CloseCameraChannel(int32 CameraIndex)
		PURE_VIRTUAL(UURLabCameraPublishTransport::CloseCameraChannel, );

};
