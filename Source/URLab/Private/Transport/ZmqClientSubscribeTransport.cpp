// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "Transport/ZmqClientSubscribeTransport.h"

#include "Transport/RenderBusTopics.h"
#include "Transport/ZmqContext.h"
#include "Transport/ZmqUtil.h"
#include "Utils/URLabLogging.h"

#include "zmq.h"

void UURLabZmqClientSubscribeTransport::Configure(const FString& InEndpoint, const FString& InTopic,
	FOnClientMessage Callback)
{
	Endpoint = InEndpoint;
	Topic = InTopic;
	OnMessage = Callback;
}

bool UURLabZmqClientSubscribeTransport::SetRenderDebugCaps(const FMjRenderDebugCaps& DebugCaps)
{
	// Called between Configure and TransportInit, so the tiered subscription goes
	// on with the lean one and the owner learns what this subscriber wants from the
	// very first XPUB event rather than after a reconnect.
	if (!DebugCaps.WantsAny())
	{
		TieredTopic.Reset();
		return true;
	}
	TieredTopic = URLabRenderBusTopic::ForCaps(DebugCaps.bContacts, DebugCaps.bOverlay,
		DebugCaps.MaxContacts);
	return true;
}

bool UURLabZmqClientSubscribeTransport::TransportInit()
{
	if (ZmqSub != nullptr)
	{
		return true; // already connected
	}
	if (Endpoint.IsEmpty())
	{
		UE_LOG(LogURLabNet, Warning, TEXT("[ZmqClientSub] no endpoint configured"));
		return false;
	}

	ZmqCtx = FURLabZmqContext::Acquire();
	ZmqSub = ZmqCtx ? zmq_socket(ZmqCtx, ZMQ_SUB) : nullptr;
	if (ZmqSub == nullptr)
	{
		UE_LOG(LogURLabNet, Error, TEXT("[ZmqClientSub] zmq_socket failed"));
		FURLabZmqContext::Release(ZmqCtx);
		return false;
	}
	// A render sink wants the latest frame, not a backlog: bounded recv timeout so
	// the worker checks the stop flag, tiny high-water mark, drop lingering sends.
	const int RcvTimeout = 200;
	const int Linger = 0;
	const int RcvHwm = 8;
	zmq_setsockopt(ZmqSub, ZMQ_RCVTIMEO, &RcvTimeout, sizeof(RcvTimeout));
	zmq_setsockopt(ZmqSub, ZMQ_LINGER, &Linger, sizeof(Linger));
	zmq_setsockopt(ZmqSub, ZMQ_RCVHWM, &RcvHwm, sizeof(RcvHwm));
	if (zmq_connect(ZmqSub, TCHAR_TO_UTF8(*Endpoint)) != 0)
	{
		UE_LOG(LogURLabNet, Error, TEXT("[ZmqClientSub] zmq_connect('%s') failed"), *Endpoint);
		zmq_close(ZmqSub);
		ZmqSub = nullptr;
		FURLabZmqContext::Release(ZmqCtx);
		return false;
	}
	const FTCHARToUTF8 TopicUtf8(*Topic);
	zmq_setsockopt(ZmqSub, ZMQ_SUBSCRIBE, TopicUtf8.Get(), TopicUtf8.Length());
	if (!TieredTopic.IsEmpty())
	{
		const FTCHARToUTF8 TieredUtf8(*TieredTopic);
		zmq_setsockopt(ZmqSub, ZMQ_SUBSCRIBE, TieredUtf8.Get(), TieredUtf8.Length());
	}

	bStop.store(false, std::memory_order_release);
	bTierAcknowledged = false;
	Worker.Start(TEXT("URLab.Zmq.ClientSub"),
		[this]() { RunReceiveLoop(); },
		[this]() { bStop.store(true, std::memory_order_release); });
	if (TieredTopic.IsEmpty())
	{
		UE_LOG(LogURLabNet, Log, TEXT("[ZmqClientSub] subscribed '%s' on %s"), *Topic, *Endpoint);
	}
	else
	{
		UE_LOG(LogURLabNet, Log,
			TEXT("[ZmqClientSub] subscribed '%s' + debug tier '%s' on %s"),
			*Topic, *TieredTopic, *Endpoint);
	}
	return true;
}

void UURLabZmqClientSubscribeTransport::TransportShutdown()
{
	bStop.store(true, std::memory_order_release);
	Worker.Stop();

	// Socket first, then the context reference: zmq_ctx_term blocks until every
	// socket on the shared context is closed.
	if (ZmqSub != nullptr)
	{
		zmq_close(ZmqSub);
		ZmqSub = nullptr;
	}
	FURLabZmqContext::Release(ZmqCtx);
}

void UURLabZmqClientSubscribeTransport::RunReceiveLoop()
{
	TArray<uint8> Incoming;
	TArray<uint8> Newest;
	FString InTopic;

	while (!bStop.load(std::memory_order_acquire))
	{
		Newest.Reset();
		// Blocks up to ZMQ_RCVTIMEO so the stop flag is re-checked promptly. The
		// topic IS decoded when a tier was negotiated: two subscriptions share this
		// socket and only the topic distinguishes them.
		FString* TopicOut = TieredTopic.IsEmpty() ? nullptr : &InTopic;
		bool bAny = false;
		if (URLabZmqUtil::RecvTopicPayload(ZmqSub, /*Flags=*/0, TopicOut, Incoming))
		{
			bAny |= AcceptFrame(InTopic, Incoming, Newest);
			// A render sink wants the latest state, not a backlog: drain to the
			// newest queued frame and drop the rest.
			while (!bStop.load(std::memory_order_acquire)
				&& URLabZmqUtil::RecvTopicPayload(ZmqSub, ZMQ_DONTWAIT, TopicOut, Incoming))
			{
				bAny |= AcceptFrame(InTopic, Incoming, Newest);
			}
		}
		if (bAny && Newest.Num() > 0 && OnMessage.IsBound())
		{
			OnMessage.Execute(Topic, Newest);
		}
	}
}

bool UURLabZmqClientSubscribeTransport::AcceptFrame(const FString& InTopic,
	TArray<uint8>& Incoming, TArray<uint8>& Newest)
{
	if (TieredTopic.IsEmpty())
	{
		Newest = MoveTemp(Incoming);
		return true;
	}
	if (InTopic == TieredTopic)
	{
		if (!bTierAcknowledged)
		{
			// The owner is serving the tier. Drop the lean subscription rather than
			// keep receiving the same transforms twice; a tiered frame is a superset
			// of a lean one, so nothing is lost.
			bTierAcknowledged = true;
			const FTCHARToUTF8 LeanUtf8(*Topic);
			zmq_setsockopt(ZmqSub, ZMQ_UNSUBSCRIBE, LeanUtf8.Get(), LeanUtf8.Length());
			UE_LOG(LogURLabNet, Log,
				TEXT("[ZmqClientSub] owner is serving debug tier '%s'; dropped the lean '%s' subscription"),
				*TieredTopic, *Topic);
		}
		Newest = MoveTemp(Incoming);
		return true;
	}
	// A lean frame. Before the tier is acknowledged it is the only thing arriving
	// and it is what keeps the subscriber rendering against an owner that does not
	// serve the tier. Afterwards it can only be a frame queued before the
	// unsubscribe took effect, and applying it would blank the decor for a tick.
	if (bTierAcknowledged)
	{
		return false;
	}
	Newest = MoveTemp(Incoming);
	return true;
}
