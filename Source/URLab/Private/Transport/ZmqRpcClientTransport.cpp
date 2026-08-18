// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "Transport/ZmqRpcClientTransport.h"

#include "Utils/URLabLogging.h"

#include "zmq.h"

void UURLabZmqRpcClientTransport::Configure(const FString& InEndpoint)
{
	Endpoint = InEndpoint;
}

bool UURLabZmqRpcClientTransport::TransportInit()
{
	if (ZmqReq != nullptr)
	{
		return true; // already connected
	}
	if (Endpoint.IsEmpty())
	{
		UE_LOG(LogURLab, Warning, TEXT("[ZmqRpcClient] no endpoint configured"));
		return false;
	}

	ZmqCtx = zmq_ctx_new();
	ZmqReq = zmq_socket(ZmqCtx, ZMQ_REQ);
	if (ZmqReq == nullptr)
	{
		UE_LOG(LogURLab, Error, TEXT("[ZmqRpcClient] zmq_socket failed"));
		zmq_ctx_term(ZmqCtx);
		ZmqCtx = nullptr;
		return false;
	}
	// Drop lingering sends on close; per-Request send/receive timeouts are set on
	// each call from that call's budget.
	const int Linger = 0;
	zmq_setsockopt(ZmqReq, ZMQ_LINGER, &Linger, sizeof(Linger));
	if (zmq_connect(ZmqReq, TCHAR_TO_UTF8(*Endpoint)) != 0)
	{
		UE_LOG(LogURLab, Error, TEXT("[ZmqRpcClient] zmq_connect('%s') failed"), *Endpoint);
		zmq_close(ZmqReq);
		zmq_ctx_term(ZmqCtx);
		ZmqReq = nullptr;
		ZmqCtx = nullptr;
		return false;
	}
	return true;
}

bool UURLabZmqRpcClientTransport::Request(const TArray<uint8>& Payload, TArray<uint8>& OutReply, int32 TimeoutMs)
{
	OutReply.Reset();
	if (ZmqReq == nullptr)
	{
		return false;
	}
	const int Timeout = static_cast<int>(TimeoutMs);
	zmq_setsockopt(ZmqReq, ZMQ_RCVTIMEO, &Timeout, sizeof(Timeout));
	zmq_setsockopt(ZmqReq, ZMQ_SNDTIMEO, &Timeout, sizeof(Timeout));

	if (zmq_send(ZmqReq, Payload.GetData(), Payload.Num(), 0) < 0)
	{
		return false;
	}

	zmq_msg_t Msg;
	zmq_msg_init(&Msg);
	if (zmq_msg_recv(&Msg, ZmqReq, 0) < 0)
	{
		zmq_msg_close(&Msg);
		return false;
	}
	const int32 Size = static_cast<int32>(zmq_msg_size(&Msg));
	OutReply.SetNumUninitialized(Size, EAllowShrinking::No);
	if (Size > 0)
	{
		FMemory::Memcpy(OutReply.GetData(), zmq_msg_data(&Msg), Size);
	}
	zmq_msg_close(&Msg);
	return true;
}

void UURLabZmqRpcClientTransport::TransportShutdown()
{
	if (ZmqReq != nullptr)
	{
		zmq_close(ZmqReq);
		ZmqReq = nullptr;
	}
	if (ZmqCtx != nullptr)
	{
		zmq_ctx_term(ZmqCtx);
		ZmqCtx = nullptr;
	}
}

void UURLabZmqRpcClientTransport::BeginDestroy()
{
	TransportShutdown();
	Super::BeginDestroy();
}
