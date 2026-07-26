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

// Opaque publisher handles from the rcl seam; defined in UrlabRclCore.cpp. Held
// by pointer so this Public header never includes the Private core header.
struct UrlabRclJointStatePub;
struct UrlabRclImuPub;
struct UrlabRclTfPub;
struct UrlabRclTwistStampedPub;
struct UrlabRclClockPub;

struct FMjStateSnapshot;
struct FMjArticulationState;
struct FMjClock;

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
 * fill + `rcl_publish` per publisher.
 *
 * Per articulation, on `/<art>/...`:
 *  - `sensor_msgs/JointState` on `joint_states` (always),
 *  - `sensor_msgs/Imu` on `imu` (when the art carries a gyro and/or accel),
 *  - `geometry_msgs/TwistStamped` on `cmd_twist` (when the art has a twist).
 *
 * Process-wide (one each):
 *  - `tf2_msgs/TFMessage` on `/tf`, one transform per body (parent `world`,
 *    child `<art>/<body>`),
 *  - `rosgraph_msgs/Clock` on `/clock`, from the IR sim time.
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

	/** Collapse an articulation's gyro + accel sensors into the components a
	 *  `sensor_msgs/Imu` carries: the first gyro's angular velocity and the first
	 *  accel's linear acceleration. Either may be absent (the flags say which are
	 *  present); an unpaired gyro still yields angular velocity only. Returns true
	 *  when at least one component is present, i.e. an Imu is worth publishing.
	 *  Pure function, exposed for the pairing test. */
	static bool FillImu(const FMjArticulationState& Art,
		double OutAngularVel[3], bool& bOutHasAngularVel,
		double OutLinearAccel[3], bool& bOutHasLinearAccel);

	/** Copy an articulation's twist command into the linear + angular vectors a
	 *  `geometry_msgs/TwistStamped` carries. Returns false when the art has no
	 *  twist. Pure function. */
	static bool FillTwistStamped(const FMjArticulationState& Art,
		double OutLinear[3], double OutAngular[3]);

	/** Flatten every body across the snapshot into the parallel arrays a
	 *  `tf2_msgs/TFMessage` carries: parent `world`, child `<art>/<body>`,
	 *  translation from the body's world position, rotation reordered from MuJoCo
	 *  wxyz to the xyzw the core expects. Pure function. */
	static void FillTf(const FMjStateSnapshot& Snapshot,
		TArray<FString>& OutParents, TArray<FString>& OutChildren,
		TArray<double>& OutTranslations, TArray<double>& OutRotationsXyzw);

	/** Project the IR clock's sim time to nanoseconds, matching the sec/nsec
	 *  split `FURLabRpcDispatcher::AppendClockFields` uses for the msgpack wire.
	 *  Pure function, exposed for the clock-parity test. */
	static int64 FillClock(const FMjClock& Clock);

	/** Test seams: inspect the rebuilt publisher set and the per-step publish
	 *  count without a ROS runtime dependency. */
	int32 GetArtPublisherCountForTest() const { return Publishers.Num(); }
	uint32 GetCachedStructureVersionForTest() const { return CachedStructureVersion; }
	int64 GetPublishStateCountForTest() const { return PublishStateCount; }

private:
	struct FArtPublishers
	{
		FName ArtSegment;
		UrlabRclJointStatePub* JointStatePub = nullptr;
		UrlabRclImuPub* ImuPub = nullptr;           // null when the art has no gyro/accel
		UrlabRclTwistStampedPub* TwistPub = nullptr; // null when the art has no twist
	};

	TArray<FArtPublishers> Publishers;
	UrlabRclTfPub* TfPub = nullptr;   // process-wide /tf tree
	UrlabRclClockPub* ClockPub = nullptr; // process-wide /clock
	uint32 CachedStructureVersion = 0;
	bool bPublishersBuilt = false;

	/** Counts PublishState invocations so a test can assert the fan-out drove ROS
	 *  in a given mode; incremented before the availability short-circuit is not
	 *  useful, so it counts only calls that reached the publish body. */
	int64 PublishStateCount = 0;

	/** Destroy and recreate the publisher set from the snapshot's current
	 *  structure. */
	void RebuildPublishers(const FMjStateSnapshot& Snapshot);

	/** Destroy every publisher handle, in reverse order. */
	void ReleasePublishers();
};
