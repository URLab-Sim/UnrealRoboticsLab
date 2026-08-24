// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "RpcClientTransport.generated.h"

/**
 * @class UURLabRpcClientTransport
 * @brief Abstract base for the CLIENT side of a request/reply exchange.
 *
 * The counterpart to the server-side control RPC: a renderer sends one request
 * to a remote owner/driver endpoint and blocks for the reply. It is the shared
 * shape behind the fast-path renderer's short-lived REQ calls -- the model
 * fetch (fastpath_hello) and the perturbation ack.
 *
 * Concrete backends (ZMQ now; SHM / ROS / gRPC via the external-transport
 * provider hook) own the socket + connection. Request is synchronous: the
 * caller blocks up to TimeoutMs for the reply.
 */
UCLASS(Abstract)
class URLAB_API UURLabRpcClientTransport : public UObject
{
	GENERATED_BODY()

public:
	/** Construct + connect the configured RPC client backend, owned by Outer,
	 *  targeting the remote REQ endpoint. The backend (ZMQ today; SHM / ROS /
	 *  gRPC as they are added) is chosen here so every request/reply call site
	 *  shares one construction seam instead of naming a concrete transport.
	 *  Returns the connected transport, or nullptr if the backend could not
	 *  connect. */
	static UURLabRpcClientTransport* Create(UObject* Outer, const FString& Endpoint);

	/** Remote REQ endpoint (e.g. "tcp://host:5571"). Call before TransportInit. */
	virtual void Configure(const FString& Endpoint)
		PURE_VIRTUAL(UURLabRpcClientTransport::Configure, );

	/** Send one request, block for the reply up to TimeoutMs, fill OutReply.
	 *  Returns true on a completed round trip, false on send/receive failure. */
	virtual bool Request(const TArray<uint8>& Payload, TArray<uint8>& OutReply, int32 TimeoutMs)
		PURE_VIRTUAL(UURLabRpcClientTransport::Request, return false;);

	/** Create the socket + connect. False if the runtime is unavailable. */
	virtual bool TransportInit()
		PURE_VIRTUAL(UURLabRpcClientTransport::TransportInit, return false;);

	/** Close the socket + release the connection. Idempotent. */
	virtual void TransportShutdown()
		PURE_VIRTUAL(UURLabRpcClientTransport::TransportShutdown, );

	/** Stable identifier, e.g. "zmq-req". */
	virtual FString GetTransportName() const
		PURE_VIRTUAL(UURLabRpcClientTransport::GetTransportName, return FString(););
};
