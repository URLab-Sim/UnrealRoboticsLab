// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "Transport/RpcClientTransport.h"
#include "ZmqRpcClientTransport.generated.h"

/**
 * ZMQ backend for the client side of request/reply: a ZMQ_REQ connected to a
 * remote owner/driver endpoint. Request sets the send/receive timeout from the
 * per-call budget, sends the payload, and blocks for the single reply frame.
 * This is the single home for the raw-libzmq REQ round trip used by the
 * render server's model fetch and perturbation calls.
 *
 * A REQ socket is a strict send/recv state machine: a missed reply leaves it in
 * "must recv" and every later send fails with EFSM. Callers cache this client for
 * a whole interaction (the renderer holds one for the duration of a drag), so one
 * slow owner ack would otherwise kill the channel until the client is destroyed.
 * Two defences: ZMQ_REQ_RELAXED/ZMQ_REQ_CORRELATE let a new request be sent while
 * the previous reply is outstanding (and discard that late reply by request id),
 * and any failed round trip drops and reopens the socket so the next call starts
 * from a clean state machine even where the options are unavailable.
 */
UCLASS()
class URLAB_API UURLabZmqRpcClientTransport : public UURLabRpcClientTransport
{
	GENERATED_BODY()

public:
	virtual void Configure(const FString& Endpoint) override;
	virtual bool Request(const TArray<uint8>& Payload, TArray<uint8>& OutReply, int32 TimeoutMs) override;
	virtual bool TransportInit() override;
	virtual void TransportShutdown() override;
	virtual FString GetTransportName() const override { return TEXT("zmq-req"); }

private:
	FString Endpoint;

	void* ZmqCtx = nullptr;
	void* ZmqReq = nullptr;

	/** Create + configure + connect the REQ socket on the existing context.
	 *  Returns false (and leaves ZmqReq null) when the socket cannot be opened. */
	bool OpenReqSocket();

	/** Close the REQ socket, leaving the context alive so it can be reopened. */
	void CloseReqSocket();
};
