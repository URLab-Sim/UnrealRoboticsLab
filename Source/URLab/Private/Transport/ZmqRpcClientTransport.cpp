// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "Transport/ZmqRpcClientTransport.h"

#include "Transport/ZmqContext.h"
#include "Utils/URLabLogging.h"

#include "zmq.h"

void UURLabZmqRpcClientTransport::Configure(const FString& InEndpoint)
{
	Endpoint = InEndpoint;
}

bool UURLabZmqRpcClientTransport::OpenReqSocket()
{
	ZmqReq = zmq_socket(ZmqCtx, ZMQ_REQ);
	if (ZmqReq == nullptr)
	{
		UE_LOG(LogURLabNet, Error, TEXT("[ZmqRpcClient] zmq_socket failed"));
		return false;
	}
	// Drop lingering sends on close; per-Request send/receive timeouts are set on
	// each call from that call's budget.
	const int Linger = 0;
	zmq_setsockopt(ZmqReq, ZMQ_LINGER, &Linger, sizeof(Linger));
	// Relax the strict REQ send/recv alternation so a timed-out request does not
	// leave the socket stuck in "must recv" (EFSM on every later send), and
	// correlate replies so a late answer to the abandoned request is discarded
	// rather than handed back as the answer to the next one. Correlate is only
	// meaningful together with relaxed, which is why both are set here.
	const int One = 1;
	zmq_setsockopt(ZmqReq, ZMQ_REQ_RELAXED, &One, sizeof(One));
	zmq_setsockopt(ZmqReq, ZMQ_REQ_CORRELATE, &One, sizeof(One));
	if (zmq_connect(ZmqReq, TCHAR_TO_UTF8(*Endpoint)) != 0)
	{
		UE_LOG(LogURLabNet, Error, TEXT("[ZmqRpcClient] zmq_connect('%s') failed"), *Endpoint);
		zmq_close(ZmqReq);
		ZmqReq = nullptr;
		return false;
	}
	return true;
}

void UURLabZmqRpcClientTransport::CloseReqSocket()
{
	if (ZmqReq != nullptr)
	{
		zmq_close(ZmqReq);
		ZmqReq = nullptr;
	}
}

bool UURLabZmqRpcClientTransport::TransportInit()
{
	if (ZmqReq != nullptr)
	{
		return true; // already connected
	}
	if (Endpoint.IsEmpty())
	{
		UE_LOG(LogURLabNet, Warning, TEXT("[ZmqRpcClient] no endpoint configured"));
		return false;
	}

	if (ZmqCtx == nullptr)
	{
		ZmqCtx = FURLabZmqContext::Acquire();
	}
	if (!OpenReqSocket())
	{
		FURLabZmqContext::Release(ZmqCtx);
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
		// Rebuild before returning: a send that failed part-way (or on an older
		// libzmq that ignored ZMQ_REQ_RELAXED) leaves the REQ state machine wedged,
		// and the caller keeps this client for its next request.
		CloseReqSocket();
		OpenReqSocket();
		return false;
	}

	zmq_msg_t Msg;
	zmq_msg_init(&Msg);
	if (zmq_msg_recv(&Msg, ZmqReq, 0) < 0)
	{
		zmq_msg_close(&Msg);
		// Timed out waiting for the reply. Discard the socket so the abandoned
		// request cannot be answered later against a subsequent send, and reopen so
		// the next Request on this cached client starts clean.
		CloseReqSocket();
		OpenReqSocket();
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
	// Socket first, then the context reference: zmq_ctx_term blocks until every
	// socket on the shared context is closed.
	CloseReqSocket();
	FURLabZmqContext::Release(ZmqCtx);
}
