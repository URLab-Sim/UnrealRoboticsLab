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
#include <atomic>
#include <memory>
#include "DmEnvRpcTransport.generated.h"

class FRunnableThread;
class FURLabDmEnvRpcRunnable;

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
	virtual bool AcceptsEditorOps() const override { return true; }
	virtual void AppendHandshakeBlock(TSharedPtr<class FJsonObject>& Reply) const override;

	/** Sets port for gRPC server binding (default 50051). */
	void SetListenPort(int32 InPort) { ListenPort = InPort; }
	int32 GetListenPort() const { return ListenPort; }

	/** Worker execution hook called from the background runnable. */
	void RunServerLoop();

	bool ShouldStop() const { return bShouldStop.load(std::memory_order_relaxed); }

	// --- viewer-frame cache: the latest {t,qpos,qvel} the owner produced, fed by
	// FMjExternalTransportProvider::OnViewerFrame and streamed by a subscribe_viewer
	// gRPC call. Thread-safe. ---
	void SetViewerFrame(const TArray<uint8>& Bytes);
	// Copy the latest frame into Out iff its sequence advanced past InOutSeq (which
	// is then updated). Returns true when a fresh frame was written.
	bool GetViewerFrame(TArray<uint8>& Out, uint64& InOutSeq) const;

private:
	UPROPERTY()
	int32 ListenPort = 50051;

	FURLabDmEnvRpcRunnable* WorkerRunnable = nullptr;
	FRunnableThread* WorkerThread = nullptr;
	std::atomic<bool> bShouldStop{false};

	grpc::Server* Server = nullptr;

	mutable FCriticalSection ViewerCacheLock;
	TArray<uint8> LatestViewerFrame;
	uint64 ViewerFrameSeq = 0;
	FDelegateHandle ViewerSinkHandle;
};
