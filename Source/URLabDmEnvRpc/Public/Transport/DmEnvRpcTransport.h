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

#pragma once

#include "CoreMinimal.h"
#include "Transport/RpcTransport.h"
#include "Transport/WorkerThread.h"
#include <atomic>
#include <future>
#include <memory>
#include <mutex>
#include "DmEnvRpcTransport.generated.h"


namespace grpc
{
class Server;
}

/**
 * @class UURLabDmEnvRpcTransport
 * @brief dm_env_rpc bi-directional streaming RPC transport for URLabBridgeServer.
 *
 * Implements UURLabRpcTransport to provide gRPC dm_env_rpc connectivity:
 *   - Receives dm_env_rpc.v1.EnvironmentRequest messages
 *   - Unpacks UrlabPacket from EnvironmentRequest.extension
 *   - Forwards to Bridge->ProcessRequestBytes
 *   - Encodes reply into UrlabPacket -> EnvironmentResponse.extension
 */
UCLASS()
class URLABDMENVRPC_API UURLabDmEnvRpcTransport : public UURLabRpcTransport
{
	GENERATED_BODY()

public:
	UURLabDmEnvRpcTransport();

	// --- UURLabRpcTransport interface ---
	virtual bool TransportInit() override;
	virtual void TransportShutdown() override;
	virtual FString GetTransportName() const override { return TEXT("dm_env_rpc"); }
	virtual void AppendHandshakeBlock(TSharedPtr<class FJsonObject>& Reply) const override;

	/** Sets the port the gRPC server binds. The bridge's net config supplies it
	 *  (via the factory), so the port advertised in the registry is the port
	 *  bound. */
	void SetListenPort(int32 InPort) { ListenPort = InPort; }
	int32 GetListenPort() const { return ListenPort; }

	/** Worker execution hook called from the background runnable. */
	void RunServerLoop();

	bool ShouldStop() const { return bShouldStop.load(std::memory_order_acquire); }

private:
	/** Cancel every active stream exactly once. Safe before or after server publication. */
	void RequestServerStop();

	/** Bind port and address. Both come from the bridge's resolved net config; a
	 *  default here would be a second place the gRPC port is written down. */
	UPROPERTY()
	int32 ListenPort = 0;

	UPROPERTY()
	FString BindAddress;

	std::atomic<bool> bShouldStop{false};
	std::atomic<bool> bShutdownIssued{false};

	// RunServerLoop and a shutdown caller take shared ownership. The mutex protects
	// publication/clear only; Shutdown and Wait are always called without holding it.
	std::mutex ServerMutex;
	std::shared_ptr<grpc::Server> Server;

	// The worker signals the ACTUAL gRPC bind result here; TransportInit blocks on
	// the paired future until the configured port is bound (true) or the bind fails
	// (false). Init must not report success before the port is really bound.
	// Re-assigned fresh each TransportInit (std::promise is one-shot).
	std::promise<bool> BindResultPromise;

	// Declared last so its destructor joins before the cancellation state above is
	// destroyed if UObject teardown reaches the native destructor after BeginDestroy.
	FURLabWorkerThread Worker;
};
