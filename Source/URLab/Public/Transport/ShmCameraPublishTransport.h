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

#pragma once

#include "CoreMinimal.h"
#include "MuJoCo/Capture/CameraShmWriter.h"
#include "Transport/CameraPublishTransport.h"
#include "ShmCameraPublishTransport.generated.h"

/**
 * @class UURLabShmCameraPublishTransport
 * @brief The built-in shared-memory backend for per-camera image egress.
 *
 * Owns one `FCameraShmWriter` per open channel, each an mmap'd double-buffered
 * region named `cam_<art>_<part>.shm` under the live URLab session dir. No
 * background thread: `PublishCameraFrame` writes into the region inline, the same
 * `[u32 size][meta][pixels]` layout the ZMQ transport sends. `Configure` supplies
 * the frame resolution before any channel opens.
 */
UCLASS()
class URLAB_API UURLabShmCameraPublishTransport : public UURLabCameraPublishTransport
{
	GENERATED_BODY()

public:
	UURLabShmCameraPublishTransport() = default;

	/** Supply the frame resolution before opening a channel; it sizes each SHM
	 *  region and validates a pushed frame's byte count. */
	void Configure(FIntPoint InResolution);

	// UURLabCameraPublishTransport contract.
	virtual void OpenCameraChannel(int32 CameraIndex, const FString& CanonicalName,
		int32 StreamPortIndex) override;
	virtual void PublishCameraFrame(const FMjCameraWireFrame& Frame) override;
	virtual void CloseCameraChannel(int32 CameraIndex) override;
	virtual bool TransportInit() override { return true; }
	virtual void TransportShutdown() override;
	virtual FString GetTransportName() const override { return TEXT("shm-cam"); }

private:
	/** Frame resolution, used to size each region and validate pushed frames. */
	FIntPoint Resolution = FIntPoint::ZeroValue;

	/** One mmap writer per channel. */
	TMap<int32, TUniquePtr<FCameraShmWriter>> Writers;
};
