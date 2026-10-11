// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "Transport/ClientSubscribeTransport.h"
#include "Transport/WorkerThread.h"
#include <atomic>
#include "ZmqClientSubscribeTransport.generated.h"

/**
 * ZMQ backend for a client-side render receiver: a ZMQ_SUB connected to a
 * publisher's endpoint, pumped by a worker thread that blocks for a frame then
 * drains non-blocking to the NEWEST one (a render sink wants the latest state,
 * not a backlog) and delivers it to the consumer callback. This is the single
 * home for the raw-libzmq SUB loop every subscribing renderer runs.
 *
 * It negotiates the debug tier through the subscription itself. Asked for
 * contacts or overlay decor, it subscribes to BOTH the lean topic and the tiered
 * topic that spells out those capabilities (see URLabRenderBusTopic). An owner
 * that reads its subscriptions -- any XPUB publisher -- starts serving the tiered
 * topic; the first frame that arrives on it is the acknowledgement, and the lean
 * subscription is dropped there and then so the transforms are not carried
 * twice. An owner that does not serve the tier keeps sending only the lean topic,
 * so the subscriber renders transforms forever instead of starving on a topic nobody
 * writes.
 */
UCLASS()
class URLAB_API UURLabZmqClientSubscribeTransport : public UURLabClientSubscribeTransport
{
	GENERATED_BODY()

public:
	virtual void Configure(const FString& Endpoint, const FString& Topic,
		FOnClientMessage Callback) override;
	virtual bool SetRenderDebugCaps(const FMjRenderDebugCaps& DebugCaps) override;
	virtual bool TransportInit() override;
	virtual void TransportShutdown() override;
	virtual FString GetTransportName() const override { return TEXT("zmq-sub"); }

private:
	FString Endpoint;
	FString Topic;
	FOnClientMessage OnMessage;
	/** The tiered topic this subscriber negotiated, empty when it asked for nothing.
	 *  Subscribed alongside `Topic`; see the class comment for why both. */
	FString TieredTopic;
	/** Set once a frame has arrived on TieredTopic: the owner is serving the
	 *  tier, so the lean topic is unsubscribed and any lean frame still queued
	 *  behind it is dropped rather than blanking the decor for a tick. */
	bool bTierAcknowledged = false;

	void* ZmqCtx = nullptr;
	void* ZmqSub = nullptr;
	FURLabWorkerThread Worker;
	std::atomic<bool> bStop{false};

	/** Worker body: block for a frame, drain to the newest, deliver. */
	void RunReceiveLoop();

	/** Decide whether one received `[topic][payload]` becomes the frame to
	 *  deliver, moving it into `Newest` when it does. This is where the tier
	 *  acknowledgement happens, because the first tiered frame is the only signal
	 *  a subscriber ever gets that the owner honoured its subscription. */
	bool AcceptFrame(const FString& InTopic, TArray<uint8>& Incoming, TArray<uint8>& Newest);
};
