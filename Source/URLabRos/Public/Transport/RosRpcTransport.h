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
#include "Transport/RpcTransport.h"
#include <atomic>
#include "RosRpcTransport.generated.h"

class FRunnableThread;
class FRunnable;
class AAMjManager;

// Opaque core handle, defined only in UrlabRclCore.cpp; held by pointer so this
// Public header never includes the Private core header.
struct UrlabRclContext;

// Per-articulation command binding (the cmd_ctrl / cmd_vel / joint_command
// subscriptions, the claim / release services, and the identity used to resolve +
// gate writes). Defined in the .cpp; held by pointer.
struct FRosArtCommand;

// Per declared user-input-channel subscription binding. Defined in the .cpp.
struct FRosUserInputSub;

// IR user-channel kind; forward-declared so this public header stays free of the
// core state-types include.
enum class EMjUserChannelKind : uint8;

/**
 * @class UURLabRosRpcTransport
 * @brief Request/reply + control-in surface for the in-process ROS 2 node.
 *
 * Sibling of the ZMQ / SHM RPC transports: owned by `UURLabBridgeServer`, bound
 * through `EnsureExternalTransportsBound`. It owns a ROS executor thread (an `FRunnable`) that
 * drives the core wait set through `UrlabRcl_SpinSome`, pumping the per-art
 * command subscriptions and marshalling them into the sim's control write paths.
 *
 * Control-in surface (FreeRun step mode only; Stepped / StatePushed bundle
 * control into their step / push calls, so writes are dropped outside FreeRun):
 *  - `/<art>/cmd_ctrl` (`std_msgs/Float64MultiArray`), values in the entity's
 *    actuator-list order, routed through `IMjControlIngress::WriteCtrl` exactly as
 *    `ApplyStepCtrl` does;
 *  - `/<art>/cmd_vel` (`geometry_msgs/Twist`), resolved onto the entity's base
 *    joints through `IMjControlIngress::WriteTwist`.
 * Every write is tagged source id `RosControlSourceId()` and must pass
 * `FMjControlOwnership::CheckWrite` first; a non-owning write is dropped.
 *
 * The subscription set is rebuilt when the entity partition changes (a
 * `StructureVersion` bump) or the live manager swaps, mirroring the publish
 * transport's per-entity rebuild rule. All rcl handles are created, spun, and
 * destroyed on the executor thread, per the core's single-thread-per-handle
 * contract.
 */
UCLASS()
class URLABROS_API UURLabRosRpcTransport : public UURLabRpcTransport
{
	GENERATED_BODY()

public:
	// --- UURLabRpcTransport contract ---
	virtual bool TransportInit() override;
	virtual void TransportShutdown() override;
	virtual FString GetTransportName() const override { return TEXT("ros2-rpc"); }
	/** ROS is a full peer surface, so it accepts editor ops like ZMQ does. */
	virtual bool AcceptsEditorOps() const override { return true; }

	/** Source id every ROS control write carries, mirroring the node name
	 *  `FURLabRosContext` creates. */
	static FString RosControlSourceId();

	/** Marshal a received `cmd_ctrl` message for `ArtName` (the entity wire key) into
	 *  the control ingress `ApplyStepCtrl` writes to: gate on ownership (`CheckWrite`)
	 *  then that step mode is FreeRun, and on success route each value onto the entity's actuator ids
	 *  in list order. Public so the C subscription trampoline can reach it. */
	void HandleRosCtrl(const FString& ArtName, const double* Values, int32 Count);

	/** Marshal a received `cmd_vel` message for `ArtName` onto the entity's base
	 *  joints via the control ingress, gated identically to HandleRosCtrl. */
	void HandleRosTwist(const FString& ArtName, const double Linear[3],
		const double Angular[3]);

	/** Marshal a received `sensor_msgs/JointState` on `/<art>/joint_command`: resolve
	 *  each named joint to the actuator driving it (or a named actuator) off the model
	 *  and route its position target through the control ingress, gated identically to
	 *  HandleRosCtrl (ownership + FreeRun step mode). This is what lets the standard
	 *  joint_state_publisher_gui jog the entity. Public so the C subscription
	 *  trampoline can reach it. */
	void HandleRosJointCommand(const FString& ArtName, const char** Names,
		const double* Positions, int32 Count);

	/** Marshal a user-channel input value for `ArtOrNone` (canonical art segment, or
	 *  None for scene scope) into the declaring component via the core
	 *  `ApplyUserChannelInput`. Unlike control writes, user-channel input is NOT
	 *  ownership- or FreeRun-mode-gated: it is app-level data owned by user logic, not
	 *  a control write that fights the physics authority. Public for the C
	 *  subscription trampoline. */
	void HandleRosUserChannel(FName ArtOrNone, FName Channel, EMjUserChannelKind Kind,
		const double* Values, int32 Count);

	/** Serve a `std_srvs/Trigger` claim_control / release_control request for
	 *  `ArtName`: build the request the ZMQ/SHM path builds (source preset to the
	 *  ROS node id, session preset to the active session), route it through
	 *  `Dispatch`, and report ok + the resulting owner in the response. The art is
	 *  encoded in the service NAME, so the request carries no art field of its own.
	 *  TTL is not settable over the ROS service (the default TTL applies). Public
	 *  for the C service trampolines. */
	void HandleRosClaimRelease(const FString& ArtName, bool bClaim, int32* OutSuccess,
		char* OutMessage, int32 OutMessageCap);

	// --- Test seams ---
	/** Drive HandleRosCtrl directly (no wire), for the mode / ownership gating
	 *  tests. */
	void ApplyRosCtrlForTest(const FString& ArtName, const TArray<double>& Values);
	/** Drive HandleRosJointCommand directly (no wire), for the jog input test. */
	void ApplyRosJointCommandForTest(const FString& ArtName, const TArray<FString>& Names,
		const TArray<double>& Positions);
	/** Drive HandleRosClaimRelease directly (no wire), for the service ownership
	 *  test. Returns the service success flag. */
	bool ApplyRosClaimReleaseForTest(const FString& ArtName, bool bClaim);
	/** Build the active manager's command subscriptions on the CALLING thread,
	 *  publish `Values` on `Topic` via rcl, and pump the wait set until the ctrl
	 *  callback fires or the spin budget is exhausted, then tear the subs down.
	 *  Single-threaded, so the executor thread must not be running. Returns true
	 *  if the subscription callback fired. */
	bool PublishAndPumpCtrlForTest(const FString& Topic, const TArray<double>& Values);
	int64 GetRosCtrlCallbackCountForTest() const { return RosCtrlCallbackCount.load(); }
	int64 GetRosTwistCallbackCountForTest() const { return RosTwistCallbackCount.load(); }
	int64 GetRosJointCommandCallbackCountForTest() const { return RosJointCommandCallbackCount.load(); }
	int64 GetRosUserChannelCallbackCountForTest() const { return RosUserChannelCallbackCount.load(); }
	int32 GetCommandSubCountForTest() const { return ArtCommands.Num(); }
	int32 GetUserInputSubCountForTest() const { return UserInputSubs.Num(); }

private:
	FRunnableThread* WorkerThread = nullptr;
	/** Runnable driving WorkerThread. FRunnableThread does not own it, so the
	 *  transport keeps the pointer and deletes it at shutdown. */
	FRunnable* WorkerRunnable = nullptr;
	std::atomic<bool> bStop{false};
	bool bIsInitialized = false;

	/** Executor loop: keeps the per-art command subscriptions in sync with the
	 *  live registry, then spins the core wait set to fire their callbacks. */
	void RunExecutorLoop();

	/** Rebuild the command subscriptions if the live manager or its structure
	 *  version changed since the set was last built; a cheap per-tick check. */
	void SyncCommandSubscriptions(UrlabRclContext* Ctx);
	/** Tear down and recreate cmd_ctrl / cmd_vel subscriptions for every
	 *  articulation of the active manager. */
	void RebuildCommandSubscriptions(UrlabRclContext* Ctx);
	/** Destroy every command subscription. Runs on the same thread that built
	 *  them (executor thread, or the calling thread of a test seam). */
	void TeardownCommandSubscriptions();

	/** Per-art command subscriptions; owned here, created / destroyed on one
	 *  thread. Raw pointers with manual lifetime (as the runnable is), so the
	 *  binding type stays confined to the .cpp. */
	TArray<FRosArtCommand*> ArtCommands;
	/** Per declared user-input-channel subscriptions; owned here, created / destroyed
	 *  on the one executor / test thread alongside ArtCommands. */
	TArray<FRosUserInputSub*> UserInputSubs;
	TWeakObjectPtr<AAMjManager> SubscribedManager;
	uint32 SubscribedStructureVersion = 0;
	bool bHaveSubscriptions = false;

	std::atomic<int64> RosCtrlCallbackCount{0};
	std::atomic<int64> RosTwistCallbackCount{0};
	std::atomic<int64> RosJointCommandCallbackCount{0};
	std::atomic<int64> RosUserChannelCallbackCount{0};

	friend class FRosExecutorRunnable;
};
