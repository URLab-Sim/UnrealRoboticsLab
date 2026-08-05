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

/**
 * @struct FMjCameraFramePayload
 * @brief One camera frame handed to out-of-core image sinks, transport-neutral.
 *
 * The camera owns no image-transport handle of its own beyond the ZMQ / SHM
 * workers; any additional sink (e.g. an optional message-bus image publisher in
 * a separate module) subscribes to FMjCameraFrameBus and receives this payload
 * per published frame. Data points at the camera's frame buffer and is valid
 * only for the duration of the broadcast, so a sink must copy or publish before
 * returning; it must not retain the pointer.
 */
struct FMjCameraFramePayload
{
	/** Canonical "<art>/<part>" identity, the same string used as the ZMQ topic
	 *  and SHM filename stem; a sink derives its topic from it. */
	FString CanonicalName;
	int32 Width = 0;
	int32 Height = 0;
	/** true: single-channel float32 depth. false: BGRA8 colour (FColor order). */
	bool bDepth = false;
	/** Tightly-packed pixel bytes (row stride == Width * bytes-per-pixel). */
	const uint8* Data = nullptr;
	int32 DataNumBytes = 0;
	/** Bytes per row, i.e. Width * sizeof(pixel). */
	int32 RowStrideBytes = 0;
	double SimTime = 0.0;
	uint64 FrameId = 0;
};

/**
 * @class FMjCameraFrameBus
 * @brief Process-wide broadcast point decoupling UMjCamera from any additional
 *        image sink.
 *
 * The camera broadcasts every published frame on OnFrameReady and announces when
 * a camera stops streaming on OnStreamStopped (so a sink can release per-camera
 * resources). Baseline ZMQ / SHM streaming does not use the bus; it exists so an
 * optional out-of-core module can consume frames without the camera referencing
 * that module's types. Both delegates fire on the game thread.
 */
class URLAB_API FMjCameraFrameBus
{
public:
	static FMjCameraFrameBus& Get();

	DECLARE_MULTICAST_DELEGATE_OneParam(FOnFrameReady, const FMjCameraFramePayload&);
	DECLARE_MULTICAST_DELEGATE_OneParam(FOnStreamStopped, const FString& /*CanonicalName*/);

	FOnFrameReady OnFrameReady;
	FOnStreamStopped OnStreamStopped;

private:
	FMjCameraFrameBus() = default;
};
