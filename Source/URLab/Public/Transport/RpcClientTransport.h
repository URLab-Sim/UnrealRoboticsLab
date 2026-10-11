// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "Transport/TransportBase.h"
#include "RpcClientTransport.generated.h"

/**
 * @class UURLabRpcClientTransport
 * @brief The CLIENT side of a request/reply exchange.
 *
 * A renderer sends one request to a remote owner endpoint and blocks for the
 * reply: the model fetch and the perturbation ack. Concrete backends own the
 * socket + connection; Request is synchronous up to TimeoutMs.
 */
UCLASS(Abstract)
class URLAB_API UURLabRpcClientTransport : public UURLabTransportBase
{
	GENERATED_BODY()

public:
	/** Construct + connect the RPC client backend for `Endpoint`'s scheme, owned
	 *  by Outer. The backend is chosen here so every request/reply call site
	 *  shares one construction seam instead of naming a concrete transport.
	 *  Returns nullptr when no backend serves that scheme, the chosen backend
	 *  does not carry request/reply, or it could not connect. */
	static UURLabRpcClientTransport* Create(UObject* Outer, const FString& Endpoint);

	/** Remote REQ endpoint (e.g. "tcp://host:5571"). Call before TransportInit. */
	virtual void Configure(const FString& Endpoint)
		PURE_VIRTUAL(UURLabRpcClientTransport::Configure, );

	/** Send one request, block for the reply up to TimeoutMs, fill OutReply.
	 *  True on a completed round trip, false on send/receive failure. */
	virtual bool Request(const TArray<uint8>& Payload, TArray<uint8>& OutReply, int32 TimeoutMs)
		PURE_VIRTUAL(UURLabRpcClientTransport::Request, return false;);

};
