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
 * fast-path renderer's model fetch and perturbation calls.
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

	virtual void BeginDestroy() override;

private:
	FString Endpoint;

	void* ZmqCtx = nullptr;
	void* ZmqReq = nullptr;
};
