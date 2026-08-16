// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "HAL/CriticalSection.h"
#include <atomic>
#include "MjbTransportBus.generated.h"

class UURLabClientSubscribeTransport;

/**
 * @class UMjbTransportBus
 * @brief Receive plumbing for the owner -> renderer transform bus.
 *
 * Owns the client-subscribe transport that mirrors an owner's "geoms" broadcast
 * (ZMQ now; SHM/ROS/gRPC via the transport hook). The transport's worker thread
 * only stashes the newest raw payload here (no UObject / msgpack work off the game
 * thread); the game thread pulls it with TakeLatestFrame and does the decode +
 * apply itself. A UObject so BusTransport is a UPROPERTY the GC roots.
 */
UCLASS()
class URLAB_API UMjbTransportBus : public UObject
{
	GENERATED_BODY()

public:
	/** Connect to the owner's "geoms" broadcast at Endpoint and start the worker.
	 *  No-op if Endpoint is empty or a transport is already connected. */
	void Start(const FString& Endpoint);
	/** Stop + release the transport and drop any buffered frame. Idempotent. */
	void Stop();
	/** True while a transport is connected (a bus subscription is live). */
	bool IsConnected() const;
	/** Move the newest raw payload (if one arrived since the last take) into Out and
	 *  clear the pending flag. Returns true (and fills Out) when a frame was pending. */
	bool TakeLatestFrame(TArray<uint8>& Out);
	/** True once at least one transform frame has ever been received off the bus. */
	bool HasEverReceived() const { return bEverReceived.load(std::memory_order_acquire); }

private:
	// The client-subscribe transport that receives the owner's "geoms" broadcast
	// (ZMQ now; SHM/ROS/gRPC via the transport hook). Owned here.
	UPROPERTY(Transient)
	TObjectPtr<UURLabClientSubscribeTransport> BusTransport;
	// OnBusMessage (worker thread) only copies the newest raw payload here (no UE
	// allocation / no msgpack decode off the game thread); the game thread decodes
	// + applies it in Tick.
	FCriticalSection FrameMutex;
	TArray<uint8> RxFrame; // guarded by FrameMutex
	bool bRxPending = false; // guarded by FrameMutex
	std::atomic<bool> bEverReceived{false};

	// Worker-thread delivery from BusTransport: stash the newest raw payload for the
	// game thread to decode + apply.
	void OnBusMessage(const FString& Topic, const TArray<uint8>& Payload);
};
