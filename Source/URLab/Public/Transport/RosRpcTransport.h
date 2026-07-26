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

/**
 * @class UURLabRosRpcTransport
 * @brief Request/reply + control-in surface for the in-process ROS 2 node.
 *
 * Sibling of the ZMQ / SHM RPC transports: owned by `UURLabBridgeServer`, bound
 * through `EnsureRosBound`. It owns a ROS executor thread (an `FRunnable`) that
 * drives the core wait set through `UrlabRcl_SpinSome`, pumping any registered
 * ROS services and subscriptions and routing them into `Dispatch()` exactly as
 * the ZMQ / SHM worker loops do.
 *
 * This phase lands the class and its executor thread with no ROS services or
 * subscriptions yet (they arrive with ROS control-in), so the thread spins the
 * wait set as a no-op. That keeps `EnsureRosBound` complete: the RPC leg exists
 * and shuts down cleanly alongside the publish leg.
 */
UCLASS()
class URLAB_API UURLabRosRpcTransport : public UURLabRpcTransport
{
	GENERATED_BODY()

public:
	// --- UURLabRpcTransport contract ---
	virtual bool TransportInit() override;
	virtual void TransportShutdown() override;
	virtual FString GetTransportName() const override { return TEXT("ros2-rpc"); }
	/** ROS is a full peer surface, so it accepts editor ops like ZMQ does. */
	virtual bool AcceptsEditorOps() const override { return true; }

private:
	FRunnableThread* WorkerThread = nullptr;
	/** Runnable driving WorkerThread. FRunnableThread does not own it, so the
	 *  transport keeps the pointer and deletes it at shutdown. */
	FRunnable* WorkerRunnable = nullptr;
	std::atomic<bool> bStop{false};
	bool bIsInitialized = false;

	/** Executor loop: spins the core wait set and (from ROS control-in on) routes
	 *  service / subscription traffic through the dispatcher. */
	void RunExecutorLoop();

	friend class FRosExecutorRunnable;
};
