// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "Transport/ZmqClientSubscribeTransport.h"

#include "Utils/URLabLogging.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"

#include "zmq.h"

namespace
{
// Drives the receive loop on a worker thread.
class FClientSubRunnable : public FRunnable
{
public:
	explicit FClientSubRunnable(UURLabZmqClientSubscribeTransport* InOwner) : Owner(InOwner) {}
	virtual uint32 Run() override
	{
		if (Owner)
		{
			Owner->RunReceiveLoop();
		}
		return 0;
	}
	virtual void Stop() override
	{
		if (Owner)
		{
			Owner->SignalStop();
		}
	}

private:
	UURLabZmqClientSubscribeTransport* Owner = nullptr;
};
} // namespace

void UURLabZmqClientSubscribeTransport::Configure(const FString& InEndpoint, const FString& InTopic,
	FOnClientMessage Callback)
{
	Endpoint = InEndpoint;
	Topic = InTopic;
	OnMessage = Callback;
}

bool UURLabZmqClientSubscribeTransport::TransportInit()
{
	if (ZmqSub != nullptr)
	{
		return true; // already connected
	}
	if (Endpoint.IsEmpty())
	{
		UE_LOG(LogURLab, Warning, TEXT("[ZmqClientSub] no endpoint configured"));
		return false;
	}

	ZmqCtx = zmq_ctx_new();
	ZmqSub = zmq_socket(ZmqCtx, ZMQ_SUB);
	if (ZmqSub == nullptr)
	{
		UE_LOG(LogURLab, Error, TEXT("[ZmqClientSub] zmq_socket failed"));
		zmq_ctx_term(ZmqCtx);
		ZmqCtx = nullptr;
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
		UE_LOG(LogURLab, Error, TEXT("[ZmqClientSub] zmq_connect('%s') failed"), *Endpoint);
		zmq_close(ZmqSub);
		zmq_ctx_term(ZmqCtx);
		ZmqSub = nullptr;
		ZmqCtx = nullptr;
		return false;
	}
	const FTCHARToUTF8 TopicUtf8(*Topic);
	zmq_setsockopt(ZmqSub, ZMQ_SUBSCRIBE, TopicUtf8.Get(), TopicUtf8.Length());

	bStop.store(false, std::memory_order_release);
	Runnable = new FClientSubRunnable(this);
	Thread = FRunnableThread::Create(Runnable, TEXT("URLabZmqClientSub"));
	UE_LOG(LogURLab, Log, TEXT("[ZmqClientSub] subscribed '%s' on %s"), *Topic, *Endpoint);
	return true;
}

void UURLabZmqClientSubscribeTransport::TransportShutdown()
{
	bStop.store(true, std::memory_order_release);
	if (Thread != nullptr)
	{
		Thread->Kill(/*bShouldWait=*/true); // joins; FRunnable::Stop sets the flag too
		delete Thread;
		Thread = nullptr;
	}
	if (Runnable != nullptr)
	{
		delete Runnable;
		Runnable = nullptr;
	}
	if (ZmqSub != nullptr)
	{
		zmq_close(ZmqSub);
		ZmqSub = nullptr;
	}
	if (ZmqCtx != nullptr)
	{
		zmq_ctx_term(ZmqCtx);
		ZmqCtx = nullptr;
	}
}

void UURLabZmqClientSubscribeTransport::BeginDestroy()
{
	TransportShutdown();
	Super::BeginDestroy();
}

void UURLabZmqClientSubscribeTransport::RunReceiveLoop()
{
	TArray<uint8> Newest;

	// Receive the payload frame of a [topic][payload] pair into Out. Returns false
	// on timeout / no-message (flags = 0 blocks up to RCVTIMEO; ZMQ_DONTWAIT polls).
	auto RecvPair = [this](TArray<uint8>& Out, int Flags) -> bool
	{
		zmq_msg_t TopicMsg;
		zmq_msg_init(&TopicMsg);
		const int R = zmq_msg_recv(&TopicMsg, ZmqSub, Flags);
		if (R < 0)
		{
			zmq_msg_close(&TopicMsg);
			return false;
		}
		zmq_msg_close(&TopicMsg); // topic is the known subscription; discard the frame
		zmq_msg_t PayMsg;
		zmq_msg_init(&PayMsg);
		if (zmq_msg_recv(&PayMsg, ZmqSub, 0) < 0)
		{
			zmq_msg_close(&PayMsg);
			return false;
		}
		const int32 Size = static_cast<int32>(zmq_msg_size(&PayMsg));
		Out.SetNumUninitialized(Size, EAllowShrinking::No);
		if (Size > 0)
		{
			FMemory::Memcpy(Out.GetData(), zmq_msg_data(&PayMsg), Size);
		}
		zmq_msg_close(&PayMsg);
		return true;
	};

	while (!bStop.load(std::memory_order_acquire))
	{
		if (!RecvPair(Newest, 0))
		{
			continue; // timed out; re-check the stop flag
		}
		// Drain to the newest queued frame (drop the backlog).
		while (!bStop.load(std::memory_order_acquire) && RecvPair(Newest, ZMQ_DONTWAIT))
		{
		}
		if (Newest.Num() > 0 && OnMessage.IsBound())
		{
			OnMessage.Execute(Topic, Newest);
		}
	}
}
