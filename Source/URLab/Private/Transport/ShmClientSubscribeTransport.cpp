// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "Transport/ShmClientSubscribeTransport.h"

#include "HAL/PlatformProcess.h"
#include "Misc/Paths.h"
#include "Transport/ShmSession.h"
#include "Utils/URLabLogging.h"

namespace
{
/** How long the worker waits between attach attempts while the owner's segment
 *  does not exist yet. Long enough not to spin a core, short enough that a
 *  subscriber started first joins promptly once the owner comes up. */
constexpr float kAttachRetrySeconds = 0.25f;
/** Poll period once attached. The ring holds only the latest frame, so this
 *  bounds staleness rather than throughput. */
constexpr float kPollSeconds = 0.001f;
} // namespace

void UURLabShmClientSubscribeTransport::Configure(const FString& InEndpoint,
	const FString& InTopic, FOnClientMessage Callback)
{
	// "shm://<dir>": everything after the scheme is the directory, absolute
	// ("shm:///var/...") or relative alike, matching how the Python client reads
	// the same endpoint.
	SessionDir = InEndpoint;
	const int32 SchemeEnd = SessionDir.Find(TEXT("://"), ESearchCase::CaseSensitive);
	if (SchemeEnd != INDEX_NONE)
	{
		SessionDir = SessionDir.Mid(SchemeEnd + 3);
	}
	Topic = InTopic;
	OnMessage = Callback;
}

bool UURLabShmClientSubscribeTransport::SetRenderDebugCaps(const FMjRenderDebugCaps& /*DebugCaps*/)
{
	// Declined, not silently dropped -- see the class comment. Create logs the one
	// line naming this transport when the subscriber actually wanted the tier.
	return false;
}

bool UURLabShmClientSubscribeTransport::TransportInit()
{
	if (SessionDir.IsEmpty())
	{
		UE_LOG(LogURLabNet, Warning, TEXT("[ShmClientSub] no session directory in endpoint"));
		return false;
	}

	// Deliberately NOT a hard failure when the segment is missing: an owner and a
	// subscriber start in either order, and a ZMQ subscriber connecting before its
	// publisher binds is not an error either. The worker attaches when it can.
	bStop.store(false, std::memory_order_release);
	LastSequence = 0;
	Worker.Start(TEXT("URLab.Shm.ClientSub"),
		[this]() { RunReceiveLoop(); },
		[this]() { bStop.store(true, std::memory_order_release); });
	UE_LOG(LogURLabNet, Log, TEXT("[ShmClientSub] watching %s for '%s'"),
		*FPaths::Combine(SessionDir, FURLabShmSession::RenderSegmentFileName), *Topic);
	return true;
}

void UURLabShmClientSubscribeTransport::TransportShutdown()
{
	bStop.store(true, std::memory_order_release);
	Worker.Stop();
	// Only the producer deletes the file; a consumer that removed it would strand
	// every other subscriber on the same host.
	Region.Close(/*bDeleteFile=*/false);
}

bool UURLabShmClientSubscribeTransport::TryAttach()
{
	return Region.OpenExisting(
		FPaths::Combine(SessionDir, FURLabShmSession::RenderSegmentFileName));
}

bool UURLabShmClientSubscribeTransport::ReadLatest(TArray<uint8>& Out)
{
	const FMjShmHeader* Hdr = static_cast<const FMjShmHeader*>(Region.GetData());
	if (!Hdr)
	{
		return false;
	}
	const uint32 Idx = Hdr->LatestIdx.load(std::memory_order_acquire);
	const uint8* Slot = Region.GetSlot(Idx);
	if (!Slot)
	{
		return false;
	}
	uint32 Size = 0;
	FMemory::Memcpy(&Size, Slot, sizeof(uint32));
	if (Size == 0 || Size + sizeof(uint32) > Region.GetBufferStride())
	{
		return false;
	}
	Out.SetNumUninitialized(static_cast<int32>(Size), EAllowShrinking::No);
	FMemory::Memcpy(Out.GetData(), Slot + sizeof(uint32), Size);
	return true;
}

void UURLabShmClientSubscribeTransport::RunReceiveLoop()
{
	TArray<uint8> Frame;

	while (!bStop.load(std::memory_order_acquire))
	{
		if (!Region.IsOpen())
		{
			if (!TryAttach())
			{
				FPlatformProcess::Sleep(kAttachRetrySeconds);
				continue;
			}
			UE_LOG(LogURLabNet, Log, TEXT("[ShmClientSub] attached to %s"), *Region.GetPath());
		}

		const FMjShmHeader* Hdr = static_cast<const FMjShmHeader*>(Region.GetData());
		const uint64 Seq = Hdr ? Hdr->Sequence.load(std::memory_order_acquire) : LastSequence;
		if (Seq == LastSequence)
		{
			FPlatformProcess::Sleep(kPollSeconds);
			continue;
		}
		// Skipping straight to the newest sequence is the point: intermediate
		// frames were overwritten in the ring and a render sink wants the latest
		// state, not a backlog it can never catch up on.
		LastSequence = Seq;
		if (ReadLatest(Frame) && Frame.Num() > 0 && OnMessage.IsBound())
		{
			OnMessage.Execute(Topic, Frame);
		}
	}
}
