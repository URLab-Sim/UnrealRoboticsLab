// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "Transport/ClientSubscribeTransport.h"
#include "Transport/ShmRegion.h"
#include "Transport/WorkerThread.h"
#include <atomic>
#include "ShmClientSubscribeTransport.generated.h"

/**
 * Shared-memory backend for a client-side render receiver: the consumer half of
 * the latest-frame render ring.
 *
 * `shm://<session dir>` names the directory the owner's handshake advertises;
 * the render bus is the `render.shm` segment inside it, a sibling of the state
 * and camera rings. The worker attaches to the segment (retrying, because a
 * subscriber may well start before its owner) and then polls the header's sequence
 * counter, handing the newest fully-published slot to the consumer. There is no
 * backlog to drain: the ring only ever holds the latest frame, which is exactly
 * what a render sink wants.
 *
 * It carries the lean transform tier only, and says so rather than accepting a
 * debug request it cannot honour. A ring is one writer and any number of
 * anonymous readers: nothing tells the writer who is attached or what they want,
 * so there is no per-subscription negotiation to be had, and inventing a
 * request word in the header would give back the process-wide union -- set by
 * whoever attached first, never cleared -- that the per-subscription work
 * removed. Overlays ride a transport that can be asked: `tcp://` (the topic is
 * the request) or `grpc://`.
 */
UCLASS()
class URLAB_API UURLabShmClientSubscribeTransport : public UURLabClientSubscribeTransport
{
	GENERATED_BODY()

public:
	virtual void Configure(const FString& Endpoint, const FString& Topic,
		FOnClientMessage Callback) override;
	virtual bool SetRenderDebugCaps(const FMjRenderDebugCaps& DebugCaps) override;
	virtual bool TransportInit() override;
	virtual void TransportShutdown() override;
	virtual FString GetTransportName() const override { return TEXT("shm-sub"); }

private:
	/** Session directory parsed out of the `shm://` endpoint. */
	FString SessionDir;
	FString Topic;
	FOnClientMessage OnMessage;

	FMjShmRegion Region;
	FURLabWorkerThread Worker;
	std::atomic<bool> bStop{false};
	/** Sequence of the last slot delivered, so an unchanged ring costs one
	 *  atomic load per poll and no copy. */
	uint64 LastSequence = 0;

	/** Worker body: attach (retrying), then poll the sequence and deliver. */
	void RunReceiveLoop();
	/** One attach attempt. False while the owner has not created the segment. */
	bool TryAttach();
	/** Copy the newest published slot into Out. False when the slot's declared
	 *  size does not fit the stride, which is a torn or foreign write. */
	bool ReadLatest(TArray<uint8>& Out);
};
