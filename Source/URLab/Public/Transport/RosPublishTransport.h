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
#include "Transport/PublishTransport.h"
#include "RosPublishTransport.generated.h"

// Opaque publisher handle from the rcl seam; defined in UrlabRclCore.cpp. Held
// by pointer so this Public header never includes the Private core header.
struct UrlabRclJointStatePub;

struct FMjStateSnapshot;
struct FMjArticulationState;

/**
 * @class UURLabRosPublishTransport
 * @brief Publishes the per-step state IR as typed ROS 2 messages.
 *
 * Unlike the ZMQ / SHM transports, this one does not move opaque bytes: it IS
 * the encoder. It owns one rcl publisher per articulation and fills the rosidl C
 * structs (through the UrlabRclCore seam) directly from `FMjStateSnapshot`. The
 * byte `Publish(topic, payload)` path is therefore a no-op; state flows in via
 * `PublishState`, called from the manager's post-step fan-out in every mode.
 *
 * The publisher set is rebuilt only when the IR's `StructureVersion` changes
 * (an articulation registry change), so the steady-state per-step cost is one
 * fill + `rcl_publish` per articulation.
 *
 * Publishes one `sensor_msgs/JointState` per articulation on
 * `/<art>/joint_states`.
 */
UCLASS()
class URLAB_API UURLabRosPublishTransport : public UURLabPublishTransport
{
	GENERATED_BODY()

public:
	// UURLabPublishTransport contract.
	virtual bool TransportInit() override;
	virtual void TransportShutdown() override;
	virtual FString GetTransportName() const override { return TEXT("ros2-pub"); }

	/** The byte path is unused: this transport encodes the IR itself. */
	virtual void Publish(const FString& /*Topic*/, const TArray<uint8>& /*Payload*/) override {}

	/** Encode the snapshot to typed ROS messages and publish. Called from the
	 *  manager's post-step fan-out; safe to call when ROS is unavailable (no-op).
	 *  Rebuilds the publisher set first if the structure version changed. */
	void PublishState(const FMjStateSnapshot& Snapshot);

	/** Flatten one articulation's joints into the parallel arrays a
	 *  `sensor_msgs/JointState` carries: one name per joint (the canonical part
	 *  segment), and the joints' concatenated qpos / qvel slices. Pure function,
	 *  exposed for the fill-correctness test. */
	static void FillJointState(const FMjArticulationState& Art,
		TArray<FString>& OutNames, TArray<double>& OutPositions,
		TArray<double>& OutVelocities);

private:
	struct FArtPublishers
	{
		FName ArtSegment;
		UrlabRclJointStatePub* JointStatePub = nullptr;
	};

	TArray<FArtPublishers> Publishers;
	uint32 CachedStructureVersion = 0;
	bool bPublishersBuilt = false;

	/** Destroy and recreate the per-articulation publishers from the snapshot's
	 *  current structure. */
	void RebuildPublishers(const FMjStateSnapshot& Snapshot);

	/** Destroy every publisher handle, in reverse order. */
	void ReleasePublishers();
};
