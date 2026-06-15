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

#include "Transport/ShmRpcTransport.h"
#include "Transport/ShmPublishTransport.h" // ResolveSessionDir
#include "Bridge/BridgeServer.h"
#include "Bridge/RpcDispatcher.h" // MakeError
#include "Misc/Paths.h"
#include "HAL/FileManager.h"
#include "HAL/RunnableThread.h"
#include "HAL/Runnable.h"
#include "GenericPlatform/GenericPlatformProcess.h"
#include "Dom/JsonObject.h"
#include "Utils/URLabLogging.h"

#if PLATFORM_WINDOWS
#include "Windows/AllowWindowsPlatformTypes.h"
#include <windows.h>
#include "Windows/HideWindowsPlatformTypes.h"
#endif

namespace
{
/** Build the canonical Windows event name for a given session id +
 *  direction. `Local\` namespace = same-session-only; matches the
 *  same-host scope of SHM regions. */
FString MakeEventName(const FString& SessionId, const TCHAR* Direction)
{
	return FString::Printf(TEXT("Local\\URLab_%s_%s_ready"), *SessionId, Direction);
}
} // namespace

class FSmStepTransportRunnable : public FRunnable
{
public:
	UURLabShmRpcTransport* Owner;
	explicit FSmStepTransportRunnable(UURLabShmRpcTransport* InOwner)
		: Owner(InOwner) {}
	virtual uint32 Run() override
	{
		Owner->RunPollLoop();
		return 0;
	}
	virtual void Stop() override { Owner->bStop = true; }
};

UURLabShmRpcTransport::UURLabShmRpcTransport() = default;

bool UURLabShmRpcTransport::TransportInit()
{
	if (bInitialized)
		return true;

	FString Sid = SessionId;
	if (Sid.IsEmpty())
		Sid = TEXT("live");
	const FString Dir = UURLabShmPublishTransport::ResolveSessionDir(Sid);
	IFileManager::Get().MakeDirectory(*Dir, /*Tree=*/true);
	ReqPath = FPaths::Combine(Dir, TEXT("req.shm"));
	RepPath = FPaths::Combine(Dir, TEXT("rep.shm"));

	if (!ReqRegion.Open(ReqPath, static_cast<uint32>(BufferStride), /*NBuffers=*/2))
	{
		UE_LOG(LogURLabNet, Error,
			TEXT("UURLabShmRpcTransport: failed to open req.shm at %s"), *ReqPath);
		return false;
	}
	if (!RepRegion.Open(RepPath, static_cast<uint32>(ReplyBufferStride), /*NBuffers=*/2))
	{
		UE_LOG(LogURLabNet, Error,
			TEXT("UURLabShmRpcTransport: failed to open rep.shm at %s"), *RepPath);
		ReqRegion.Close(/*bDeleteFile=*/true);
		return false;
	}

#if PLATFORM_WINDOWS
	// Create named auto-reset events for kernel-wakeup signalling. Both
	// sides reference these by name; the bridge uses OpenEventW. Auto-reset
	// means a single SetEvent unblocks exactly one waiter and self-clears.
	// Initial state = unsignaled; the first signal comes from the producer.
	{
		const FString ReqName = MakeEventName(Sid, TEXT("req"));
		const FString RepName = MakeEventName(Sid, TEXT("rep"));
		// UE's Windows wrappers hide the TRUE/FALSE macros; pass integers
		// directly (BOOL is int).
		ReqReadyEvent = ::CreateEventW(nullptr, /*bManualReset=*/0,
			/*bInitialState=*/0, *ReqName);
		RepReadyEvent = ::CreateEventW(nullptr, /*bManualReset=*/0,
			/*bInitialState=*/0, *RepName);
		if (!ReqReadyEvent || !RepReadyEvent)
		{
			UE_LOG(LogURLabNet, Warning,
				TEXT("UURLabShmRpcTransport: CreateEventW failed (err=%lu); falling back to polling"),
				::GetLastError());
			if (ReqReadyEvent)
			{
				::CloseHandle(static_cast<HANDLE>(ReqReadyEvent));
				ReqReadyEvent = nullptr;
			}
			if (RepReadyEvent)
			{
				::CloseHandle(static_cast<HANDLE>(RepReadyEvent));
				RepReadyEvent = nullptr;
			}
		}
	}
#endif

	bStop = false;
	bInitialized = true;

	FSmStepTransportRunnable* Runner = new FSmStepTransportRunnable(this);
	WorkerThread = FRunnableThread::Create(Runner, TEXT("URLabSmStepTransport"));

	UE_LOG(LogURLabNet, Log,
		TEXT("UURLabShmRpcTransport: req=%s, rep=%s, sync=%s"),
		*ReqPath, *RepPath,
		ReqReadyEvent ? TEXT("kernel events") : TEXT("polling"));
	return true;
}

void UURLabShmRpcTransport::TransportShutdown()
{
	if (!bInitialized)
		return;

	bStop = true;
#if PLATFORM_WINDOWS
	// Wake the worker out of WaitForSingleObject(ReqReadyEvent) so it can
	// observe bStop and exit. The wake is wasted work (no real request),
	// which the worker filters by sequence comparison.
	if (ReqReadyEvent)
		::SetEvent(static_cast<HANDLE>(ReqReadyEvent));
#endif

	if (WorkerThread)
	{
		WorkerThread->WaitForCompletion();
		delete WorkerThread;
		WorkerThread = nullptr;
	}

#if PLATFORM_WINDOWS
	if (ReqReadyEvent)
	{
		::CloseHandle(static_cast<HANDLE>(ReqReadyEvent));
		ReqReadyEvent = nullptr;
	}
	if (RepReadyEvent)
	{
		::CloseHandle(static_cast<HANDLE>(RepReadyEvent));
		RepReadyEvent = nullptr;
	}
#endif

	ReqRegion.Close(/*bDeleteFile=*/true);
	RepRegion.Close(/*bDeleteFile=*/true);
	bInitialized = false;
}

void UURLabShmRpcTransport::RunPollLoop()
{
	FMjShmHeader* ReqHdr = static_cast<FMjShmHeader*>(ReqRegion.GetData());
	FMjShmHeader* RepHdr = static_cast<FMjShmHeader*>(RepRegion.GetData());
	if (!ReqHdr || !RepHdr)
		return;

	uint64 LastSeenReqSeq = ReqHdr->Sequence.load(std::memory_order_acquire);
	const uint32 ReqStride = ReqRegion.GetBufferStride();
	const uint32 RepStride = RepRegion.GetBufferStride();

	while (!bStop.load(std::memory_order_acquire))
	{
#if PLATFORM_WINDOWS
		// Block on the bridge's signal that req.shm has fresh content.
		// 100 ms timeout so we re-check bStop periodically -- shutdown
		// also calls SetEvent on this handle to wake immediately. Falls
		// back to polling if event creation failed in TransportInit.
		if (ReqReadyEvent)
		{
			const DWORD WaitResult = ::WaitForSingleObject(
				static_cast<HANDLE>(ReqReadyEvent), /*ms=*/100);
			if (bStop.load(std::memory_order_acquire))
				break;
			// Whether the wait timed out or signalled, fall through to
			// the sequence check -- we still need to verify there's a
			// real request waiting.
			(void)WaitResult;
		}
		else
		{
			FPlatformProcess::SleepNoStats(static_cast<float>(PollIntervalUs) * 1e-6f);
		}
#else
		FPlatformProcess::SleepNoStats(static_cast<float>(PollIntervalUs) * 1e-6f);
#endif

		const uint64 CurSeq = ReqHdr->Sequence.load(std::memory_order_acquire);
		if (CurSeq == LastSeenReqSeq)
		{
			// Spurious wake or just a periodic timeout poll -- keep going.
			continue;
		}

		// Read the latest request slot. Producer wrote the slot, then
		// bumped latest_idx, then bumped sequence -- so at this point
		// both writes are visible. Re-check sequence after the copy to
		// detect a torn read where the producer raced past us.
		const uint32 Idx = ReqHdr->LatestIdx.load(std::memory_order_acquire);
		const uint8* Slot = ReqRegion.GetSlot(Idx);
		if (!Slot)
		{
			UE_LOG(LogURLab, Warning,
				TEXT("ShmRpcTransport: dropping request seq=%llu with out-of-range latest_idx=%u (nbuffers=%u)"),
				static_cast<unsigned long long>(CurSeq), Idx, ReqHdr->NBuffers);
			LastSeenReqSeq = CurSeq;
			continue;
		}
		uint32 Size = 0;
		FMemory::Memcpy(&Size, Slot, sizeof(uint32));
		if (Size == 0 || Size + sizeof(uint32) > ReqStride)
		{
			UE_LOG(LogURLab, Warning,
				TEXT("ShmRpcTransport: dropping request seq=%llu with invalid size=%u (stride=%u)"),
				static_cast<unsigned long long>(CurSeq), Size, ReqStride);
			LastSeenReqSeq = CurSeq;
			continue;
		}

		TArray<uint8> ReqBytes;
		ReqBytes.SetNumUninitialized(static_cast<int32>(Size));
		FMemory::Memcpy(ReqBytes.GetData(), Slot + sizeof(uint32), Size);

		const uint64 SeqAfter = ReqHdr->Sequence.load(std::memory_order_acquire);
		if (SeqAfter - CurSeq > ReqHdr->NBuffers)
		{
			// Producer wrapped past our read. Skip.
			LastSeenReqSeq = SeqAfter;
			continue;
		}
		LastSeenReqSeq = CurSeq;

		// Wire detect / parse / dispatch / encode all live on the base.
		// SHM scope narrowing: the base short-circuits editor-only ops to
		// a `wrong_transport: use_zmq` reply because AcceptsEditorOps()
		// returns false on this transport.
		TArray<uint8> RepBytes;
		ProcessRequestBytes(ReqBytes, RepBytes);

		if (static_cast<uint32>(RepBytes.Num()) + sizeof(uint32) > RepStride)
		{
			// Reply doesn't fit the fixed SHM reply slot — e.g. a multi-camera
			// include_cameras frame or a large hello MJB. By design SHM hands
			// oversize replies to ZMQ. Return an explicit `wrong_transport`
			// reply NOW so the bridge re-routes this one request to ZMQ
			// immediately, instead of dropping it and forcing the client to
			// wait out its full recv timeout (the 5s stall). The fast image
			// path for SHM consumers is the per-camera cam_*.shm streams, not
			// this RPC reply slot; raise ReplyBufferStride only if you want
			// large inline replies carried over SHM.
			UE_LOG(LogURLabNet, Warning,
				TEXT("UURLabShmRpcTransport: reply %d bytes exceeds reply slot %u; routing to zmq "
					 "(raise ReplyBufferStride to carry it over shm)"),
				RepBytes.Num(), RepStride);

			TSharedPtr<FJsonObject> Err = FURLabRpcDispatcher::MakeError(
				TEXT("wrong_transport"),
				FString::Printf(
					TEXT("reply %d bytes exceeds shm reply slot %u; use zmq for this request"),
					RepBytes.Num(), RepStride));
			TArray<uint8> ErrBytes;
			EncodeReply(Err, ErrBytes);
			if (static_cast<uint32>(ErrBytes.Num()) + sizeof(uint32) > RepStride)
			{
				// Error envelope itself won't fit (pathologically tiny stride).
				// Nothing safe to write; skip and let the bridge time out.
				continue;
			}
			RepBytes = MoveTemp(ErrBytes);
		}

		const uint32 CurLatest = RepHdr->LatestIdx.load(std::memory_order_acquire);
		const uint32 NBuffers = RepHdr->NBuffers > 0 ? RepHdr->NBuffers : 1;
		const uint32 Target = (CurLatest + 1) % NBuffers;
		uint8* RepSlot = RepRegion.GetSlot(Target);
		if (!RepSlot)
			continue;

		const uint32 RepSize = static_cast<uint32>(RepBytes.Num());
		FMemory::Memcpy(RepSlot, &RepSize, sizeof(uint32));
		if (RepSize > 0)
		{
			FMemory::Memcpy(RepSlot + sizeof(uint32), RepBytes.GetData(), RepSize);
		}
		RepHdr->LatestIdx.store(Target, std::memory_order_release);
		RepHdr->Sequence.fetch_add(1, std::memory_order_release);
#if PLATFORM_WINDOWS
		// Wake the bridge's WaitForSingleObject(RepReadyEvent) so it can
		// pick up the reply without polling. Auto-reset event self-clears.
		if (RepReadyEvent)
			::SetEvent(static_cast<HANDLE>(RepReadyEvent));
#endif
	}
}
