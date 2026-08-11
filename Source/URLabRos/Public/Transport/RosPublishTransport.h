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
#include "Transport/RosOutputProvider.h"
#include "State/MjStateConsumer.h"
#include "RosPublishTransport.generated.h"

struct FMjStateSnapshot;
struct FMjArticulationState;
struct FMjClock;

/**
 * @class UURLabRosPublishTransport
 * @brief Publishes the per-step state IR as typed ROS 2 messages.
 *
 * Unlike the ZMQ / SHM transports, this one does not move opaque bytes: it IS
 * the encoder. It does not, however, hard-code the message set. It drives a
 * self-registering provider library (`FMjRosOutputRegistry`): on a
 * `StructureVersion` change it instantiates one provider per registered output,
 * calls `Build` to create that output's publishers for the current model, then
 * calls `Publish` on every provider each step. Adding an output is dropping a
 * self-registering provider file; there is no central switch here to edit. The
 * byte `Publish(topic, payload)` path is a no-op; state flows in via
 * `PublishState`, from the manager's post-step fan-out in every mode.
 *
 * The built-in providers cover, per articulation on `/<art>/...`:
 *  - `sensor_msgs/JointState` on `joint_states`,
 *  - `sensor_msgs/Imu` on `imu` (gyro and/or accel),
 *  - `geometry_msgs/TwistStamped` on `cmd_twist` (twist command),
 *  - total sensor routing (Force+Torque -> `WrenchStamped`, Rangefinder ->
 *    `Range`, Magnetometer -> `MagneticField`, Velocimeter -> `TwistStamped`,
 *    everything else -> `std_msgs/Float64MultiArray` on `sensors/<name>`),
 *  - latched `robot_description` (exported URDF),
 *  - `nav_msgs/Odometry` on `odom` and ground-truth
 *    `geometry_msgs/PoseWithCovarianceStamped` on `pose` (free-base arts),
 *  - `sensor_msgs/CameraInfo` on `<camera>/camera_info` (one per camera),
 * and process-wide: `tf2_msgs/TFMessage` on `/tf`, `rosgraph_msgs/Clock` on
 * `/clock`, and the REP-105 `map -> odom -> world` ground-truth static chain on
 * `/tf_static`.
 *
 * The pure IR -> array fill helpers (`Fill*`) stay static here so they are
 * tested with no rcl dependency; the providers call them.
 */
UCLASS()
class URLABROS_API UURLabRosPublishTransport : public UURLabPublishTransport
	, public IMjStateConsumer
{
	GENERATED_BODY()

public:
	// UURLabPublishTransport contract.
	virtual bool TransportInit() override;
	virtual void TransportShutdown() override;
	virtual FString GetTransportName() const override { return TEXT("ros2-pub"); }

	/** The byte path is unused: this transport encodes the IR itself. */
	virtual void Publish(const FString& /*Topic*/, const TArray<uint8>& /*Payload*/) override {}

	/** IMjStateConsumer: the manager's post-step fan-out entry point. Forwards to
	 *  PublishState so ROS receives the typed IR every step in all modes. */
	virtual void ConsumeState(const FMjStateSnapshot& Snapshot) override;

	/** Encode the snapshot to typed ROS messages and publish. Safe to call when
	 *  ROS is unavailable (no-op). Rebuilds the publisher set first if the
	 *  structure version changed. */
	void PublishState(const FMjStateSnapshot& Snapshot);

	/** Flatten one articulation's 1-DOF joints into the parallel arrays a
	 *  `sensor_msgs/JointState` carries: one entry per hinge / slide joint (the
	 *  canonical part segment), each with a scalar position (`qpos - qpos0`),
	 *  velocity, and effort. Free and ball joints are multi-slot and not scalar
	 *  URDF joints, so they are skipped here (their pose reaches ROS via `/tf`);
	 *  including them would misalign names against values and truncate the tail.
	 *  Effort is the driving actuator's force, matched to the joint by shared
	 *  canonical name (the 1:1 transmission case); `OutEfforts` is left empty when
	 *  no actuator drives any of the joints. Pure function, exposed for the
	 *  fill-correctness test. */
	static void FillJointState(const FMjArticulationState& Art,
		TArray<FString>& OutNames, TArray<double>& OutPositions,
		TArray<double>& OutVelocities, TArray<double>& OutEfforts);

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

	/** Test seams: inspect the rebuilt provider set and the per-step publish count
	 *  without a ROS runtime dependency. */
	int32 GetArtPublisherCountForTest() const;
	int32 GetProviderCountForTest() const { return Providers.Num(); }
	uint32 GetCachedStructureVersionForTest() const { return CachedStructureVersion; }
	int64 GetPublishStateCountForTest() const { return PublishStateCount; }

private:
	/** The registered output providers, instantiated on each structure change.
	 *  Destroying an entry releases its publishers. */
	TArray<TUniquePtr<IMjRosOutputProvider>> Providers;
	uint32 CachedStructureVersion = 0;
	bool bProvidersBuilt = false;

	/** Counts PublishState invocations so a test can assert the fan-out drove ROS
	 *  in a given mode; incremented before the availability short-circuit is not
	 *  useful, so it counts only calls that reached the publish body. */
	int64 PublishStateCount = 0;

	/** Instantiate the registered providers and Build them against the snapshot's
	 *  current structure, releasing the previous set first. */
	void RebuildProviders(const FMjStateSnapshot& Snapshot);
};
