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
#include "Transport/ShmRegion.h"
#include <atomic>
#include "ShmRpcTransport.generated.h"

class FRunnableThread;

/**
 * @class UURLabShmRpcTransport
 * @brief Shared-memory implementation of the bridge's RPC transport.
 *
 * Owns two SHM regions per session:
 *   `req.shm` -- bridge writes msgpack request, UE reads.
 *   `rep.shm` -- UE writes msgpack reply, bridge reads.
 *
 * Bridge-owned UObject (NOT a UActorComponent). Co-exists with
 * `UURLabZmqRpcTransport`; clients pick which transport to talk to.
 *
 * **Scope: manager-required ops only.** SHM is the latency-optimised
 * inner-loop transport (1 kHz controller channels). Editor-only ops
 * (`import_xml`, `spawn_actor`, `list_actors`, etc.) get a
 * `wrong_transport: use_zmq` reply via the base class's
 * `AcceptsEditorOps()=false` short-circuit. The bridge-side client
 * auto-routes editor ops to ZMQ; nothing needs to be re-tried.
 */
UCLASS()
class URLAB_API UURLabShmRpcTransport : public UURLabRpcTransport
{
	GENERATED_BODY()

public:
	UURLabShmRpcTransport();

	/** Request-slot size (bridge -> UE). Step requests are small (qpos /
	 *  qvel / ctrl arrays), so 1 MiB is ample. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|SHM")
	int32 BufferStride = 1024 * 1024;

	/** Reply-slot size (UE -> bridge). The ring is a fixed mmap sized once at
	 *  open. Replies that exceed it are NOT dropped — they get an immediate
	 *  `wrong_transport` reply so the bridge re-routes that request to ZMQ
	 *  (the designed fallback for oversize replies). 16 MiB comfortably holds
	 *  the hello MJB and a single HD frame over SHM; multi-camera or 4K
	 *  `include_cameras` replies exceed it and fall back to ZMQ. Note the
	 *  fast image path is the per-camera cam_*.shm / ZMQ streams, NOT this
	 *  reply slot. Raise this only if you specifically want large inline
	 *  camera replies carried over the SHM RPC channel. The bridge reads the
	 *  actual stride from the SHM header (offset 8), so it adapts without a
	 *  hardcoded size. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|SHM")
	int32 ReplyBufferStride = 16 * 1024 * 1024;

	/** Optional explicit session id (defaults to "live"; mirrors the
	 *  publisher's path scheme). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|SHM")
	FString SessionId;

	/** How long the worker thread waits between sequence checks
	 *  (microseconds). On Windows the worker waits on a named event, so
	 *  this only matters when the event isn't usable (other platforms,
	 *  bridge running w/o event support); it then becomes a poll
	 *  interval. 200 us strikes a balance between latency and idle CPU. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|SHM")
	int32 PollIntervalUs = 200;

	// UURLabRpcTransport contract
	virtual bool TransportInit() override;
	virtual void TransportShutdown() override;
	virtual FString GetTransportName() const override { return TEXT("shm"); }
	/** SHM scope narrowing: editor ops never reach the dispatcher on
	 *  this transport. */
	virtual bool AcceptsEditorOps() const override { return false; }

	/** Resolved on-disk paths (set after TransportInit). */
	FString GetReqPath() const { return ReqPath; }
	FString GetRepPath() const { return RepPath; }

private:
	FMjShmRegion ReqRegion; // bridge writes, UE reads
	FMjShmRegion RepRegion; // UE writes, bridge reads

	FString ReqPath;
	FString RepPath;

	/** Named-event handles for kernel-wakeup signalling. Bridge calls
	 *  SetEvent on `ReqReadyEvent` after writing req.shm; UE's worker
	 *  waits on it. Symmetric for `RepReadyEvent`. Stored as void* to
	 *  keep <windows.h> out of the public header. nullptr on platforms
	 *  without named-event support; the worker falls back to polling. */
	void* ReqReadyEvent = nullptr;
	void* RepReadyEvent = nullptr;

	FRunnableThread* WorkerThread = nullptr;
	std::atomic<bool> bStop{false};
	bool bInitialized = false;

	/** Worker-thread loop: wait for a request (event or poll), dispatch,
	 *  write rep.shm, signal the bridge. */
	void RunPollLoop();
	friend class FSmStepTransportRunnable;
};
