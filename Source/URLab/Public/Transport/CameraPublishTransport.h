// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "CameraPublishTransport.generated.h"

/**
 * One camera frame handed to a transport for egress. Pixels are raw -- BGRA8
 * for color, float32 for depth -- and the buffer is BORROWED: a backend that
 * ships asynchronously copies anything it needs beyond the call. The metadata
 * mirrors the per-camera wire header the camera workers already emit.
 */
struct FMjCameraWireFrame
{
	int32 CameraIndex = 0;             // stable per-renderer channel id
	int32 Width = 0;
	int32 Height = 0;
	uint64 FrameId = 0;                // the post-step frame this image shows
	double SimTime = 0.0;              // mujoco.time of that frame (seconds)
	double CaptureUnixSeconds = 0.0;   // wall clock at capture, for the wire meta
	FName Dtype;                       // "bgra8" | "float32"
	const uint8* Bytes = nullptr;      // borrowed pixel buffer
	int32 NumBytes = 0;
};

/**
 * @class UURLabCameraPublishTransport
 * @brief Abstract base for per-camera IMAGE egress -- the role UURLabPublishTransport
 *        deliberately excludes.
 *
 * Camera streaming does not fit the small/frequent `Publish(topic, bytes)` shape:
 * frames are ~MB, each camera has its own lifetime, and delivery runs off a
 * per-camera GPU-readback worker rather than the physics step. This base gives
 * that egress its own role so UMjCamera stops owning a raw ZMQ PUB socket
 * (`FCameraZmqPublisher`) and a raw mmap writer (`FCameraShmWriter`) directly, and
 * a backend (ZMQ / SHM / gRPC-dm_env) is chosen by configuration instead.
 *
 * Channel model: one transport instance multiplexes N cameras, keyed by
 * `CameraIndex`. A per-socket backend (ZMQ) may stand up a socket/thread per
 * channel internally; a single-connection backend (gRPC) may fold every channel
 * onto one stream and ignore `StreamPortIndex`. The renderer sees neither.
 *
 * Threading: `PublishCameraFrame` is called from a camera's publish worker, OFF
 * the game thread; concrete backends serialise internally, exactly as the ZMQ /
 * SHM publish transports already do.
 */
UCLASS(Abstract)
class URLAB_API UURLabCameraPublishTransport : public UObject
{
	GENERATED_BODY()

public:
	/** Register a camera channel before any frame flows (stream start, game
	 *  thread). `CanonicalName` is the wire identity; `StreamPortIndex` is the
	 *  backend's per-camera slot (ZMQ port offset / SHM region index) and may be
	 *  ignored by a multiplexing backend. */
	virtual void OpenCameraChannel(int32 CameraIndex, const FString& CanonicalName,
		int32 StreamPortIndex)
		PURE_VIRTUAL(UURLabCameraPublishTransport::OpenCameraChannel, );

	/** Ship one frame for `Frame.CameraIndex`. Called off the game thread; the
	 *  backend copies anything it needs beyond this call. */
	virtual void PublishCameraFrame(const FMjCameraWireFrame& Frame)
		PURE_VIRTUAL(UURLabCameraPublishTransport::PublishCameraFrame, );

	/** Close one camera channel (stream stop or camera destroyed). Idempotent. */
	virtual void CloseCameraChannel(int32 CameraIndex)
		PURE_VIRTUAL(UURLabCameraPublishTransport::CloseCameraChannel, );

	/** Open backend handles shared across channels. Returns false on failure. */
	virtual bool TransportInit()
		PURE_VIRTUAL(UURLabCameraPublishTransport::TransportInit, return false;);

	/** Close every channel and release backend handles. Idempotent. */
	virtual void TransportShutdown()
		PURE_VIRTUAL(UURLabCameraPublishTransport::TransportShutdown, );

	/** Stable identifier, e.g. "zmq-cam". */
	virtual FString GetTransportName() const
		PURE_VIRTUAL(UURLabCameraPublishTransport::GetTransportName, return FString(););
};
