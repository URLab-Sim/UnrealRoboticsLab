// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
// trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

#pragma once

#include "CoreMinimal.h"
#include "Transport/CameraPublishTransport.h"
#include "ZmqCameraPublishTransport.generated.h"

class FCameraZmqPublisher;
class FRunnableThread;

/**
 * @class UURLabZmqCameraPublishTransport
 * @brief The built-in ZeroMQ backend for per-camera image egress.
 *
 * Owns one `FCameraZmqPublisher` per open channel -- each a PUB socket on its own
 * thread, HWM=1, draining to the freshest frame -- so the per-camera publish
 * thread lives here rather than inside UMjCamera. `Configure` supplies the
 * resolved bind endpoint and the frame resolution before any channel opens;
 * `OpenCameraChannel` stands a publisher up, `PublishCameraFrame` routes a frame
 * to the channel's publisher, and `CloseCameraChannel` stops and frees it.
 */
UCLASS()
class URLAB_API UURLabZmqCameraPublishTransport : public UURLabCameraPublishTransport
{
	GENERATED_BODY()

public:
	UURLabZmqCameraPublishTransport() = default;
	virtual ~UURLabZmqCameraPublishTransport() override;

	/** Supply the bind endpoint and frame resolution before opening a channel.
	 *  The endpoint is the fully resolved per-camera endpoint (the core already
	 *  folded in the instance's camera port block and the per-camera slot); the
	 *  publisher still auto-increments the port on a bind conflict. */
	void Configure(const FString& InEndpoint, FIntPoint InResolution);

	/** The endpoint a channel's publisher actually bound, which differs from the
	 *  configured one after an auto-increment. Empty when the channel is not open. */
	FString GetBoundEndpoint(int32 CameraIndex) const;

	// UURLabCameraPublishTransport contract.
	virtual void OpenCameraChannel(int32 CameraIndex, const FString& CanonicalName) override;
	virtual void PublishCameraFrame(const FMjCameraWireFrame& Frame) override;
	virtual void CloseCameraChannel(int32 CameraIndex) override;
	virtual bool TransportInit() override { return true; }
	virtual void TransportShutdown() override;
	virtual FString GetTransportName() const override { return TEXT("zmq-cam"); }

private:
	/** Resolved bind endpoint supplied by Configure. */
	FString Endpoint;

	/** Frame resolution, used to validate a pushed frame's pixel count. */
	FIntPoint Resolution = FIntPoint::ZeroValue;

	/**
	 * One PUB-socket publisher per channel plus the send thread driving it. The
	 * publisher is a forward-declared FRunnable defined in the .cpp, so it and the
	 * thread are held by raw pointer: a TUniquePtr value in this header-defined
	 * struct would need the complete type at the struct's implicit destructor, which
	 * the .cpp-local publisher type does not expose here. Ownership is released in
	 * CloseCameraChannel / TransportShutdown, and the destructor sweeps any that
	 * remain.
	 */
	struct FChannel
	{
		FCameraZmqPublisher* Publisher = nullptr;
		FRunnableThread* Thread = nullptr;
	};
	TMap<int32, FChannel> Channels;
};
