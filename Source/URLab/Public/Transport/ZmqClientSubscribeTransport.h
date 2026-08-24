// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "Transport/ClientSubscribeTransport.h"
#include <atomic>
#include "ZmqClientSubscribeTransport.generated.h"

class FRunnable;
class FRunnableThread;

/**
 * ZMQ backend for a client-side render receiver: a ZMQ_SUB connected to a
 * publisher's endpoint, subscribed to one topic, pumped by a worker thread that
 * blocks for a frame then drains non-blocking to the NEWEST one (a render sink
 * wants the latest state, not a backlog) and delivers it to the consumer
 * callback. This is the single home for the raw-libzmq SUB loop shared by the
 * fast-path renderer and the viewer.
 */
UCLASS()
class URLAB_API UURLabZmqClientSubscribeTransport : public UURLabClientSubscribeTransport
{
	GENERATED_BODY()

public:
	virtual void Configure(const FString& Endpoint, const FString& Topic,
		FOnClientMessage Callback) override;
	virtual bool TransportInit() override;
	virtual void TransportShutdown() override;
	virtual FString GetTransportName() const override { return TEXT("zmq-sub"); }

	virtual void BeginDestroy() override;

	/** Worker-thread entry (public so the FRunnable can drive it). */
	void RunReceiveLoop();
	void SignalStop() { bStop.store(true, std::memory_order_release); }

private:
	FString Endpoint;
	FString Topic;
	FOnClientMessage OnMessage;

	void* ZmqCtx = nullptr;
	void* ZmqSub = nullptr;
	FRunnable* Runnable = nullptr;
	FRunnableThread* Thread = nullptr;
	std::atomic<bool> bStop{false};
};
