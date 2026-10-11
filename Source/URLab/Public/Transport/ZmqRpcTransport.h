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
#include "Bridge/StepCommands.h"
#include "Dom/JsonObject.h"
#include "HAL/Event.h"
#include "HAL/PlatformProcess.h"
#include "Transport/WorkerThread.h"
#include <atomic>
#include "ZmqRpcTransport.generated.h"

/**
 * @class UURLabZmqRpcTransport
 * @brief ZMQ REQ/REP adapter. Owned by `UURLabBridgeServer`; binds a
 *        REP socket on `StepEndpoint` and runs a polling worker thread.
 *        Wire framing only — dispatcher lookup + encoding live on the base.
 */
UCLASS()
class URLAB_API UURLabZmqRpcTransport : public UURLabRpcTransport
{
	GENERATED_BODY()

public:
	UURLabZmqRpcTransport();

	/** Explicit bind override for the REP socket. Empty (the default) resolves
	 *  tcp://{BindAddress}:{StepPort} from the owning bridge's net config, so
	 *  the only place a default step port is written down is the config
	 *  struct. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ZMQ")
	FString StepEndpoint;

	/** Polling interval for the REP socket in milliseconds. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ZMQ")
	int32 PollTimeoutMs = 50;

	// --- UURLabRpcTransport contract ---
	virtual bool TransportInit() override;
	virtual void TransportShutdown() override;
	virtual FString GetTransportName() const override { return TEXT("zmq"); }
	/** ZMQ accepts every op; SHM is the only transport that refuses. */

	/** The endpoint the REP socket actually bound, resolved once in
	 *  TransportInit. Empty until then. */
	FString GetBoundEndpoint() const { return BoundEndpoint; }

private:
	void* ZmqContext = nullptr;
	void* ZmqRep = nullptr;
	FURLabWorkerThread Worker;
	std::atomic<bool> bStop{false};
	bool bIsInitialized = false;

	/** StepEndpoint when set, else tcp://{BindAddress}:{StepPort} from the
	 *  owning bridge's config (struct defaults when there is no bridge). */
	FString ResolveEndpoint() const;
	FString BoundEndpoint;

	/** Create the REP socket, apply timeouts + LINGER, and bind StepEndpoint.
	 *  Shared by TransportInit and the send-error recovery path so a wedged
	 *  REP state machine can be reset without duplicating socket setup. */
	bool CreateAndBindRep();

	/** Worker thread loop. Runs zmq_poll on the REP socket and forwards each
	 *  parsed request to the dispatcher; sends the reply back to the wire. */
	void RunPollLoop();
};
