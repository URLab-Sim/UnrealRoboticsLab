// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "Transport/RpcClientTransport.h"
#include "DmEnvRpcClientTransports.generated.h"

/**
 * gRPC (dm_env_rpc) client request/reply transport for URLab render operations.
 * gRPC is not a viewer bus and supplies no client subscribe transport.
 */

/** Request/reply over gRPC: one UrlabPacket per call (op read from the msgpack
 *  request, payload passed through). Used by MjRendererDriverClient for
 *  render_hello (model fetch) and render_perturb. */
UCLASS()
class UURLabDmEnvRpcRpcClientTransport : public UURLabRpcClientTransport
{
	GENERATED_BODY()
public:
	virtual void Configure(const FString& InEndpoint) override;
	virtual bool Request(const TArray<uint8>& Payload, TArray<uint8>& OutReply, int32 TimeoutMs) override;
	virtual bool TransportInit() override;
	virtual void TransportShutdown() override;
	virtual FString GetTransportName() const override { return TEXT("dmenv-grpc-req"); }

private:
	FString Endpoint;   // host:port (grpc:// stripped)
	struct FChan;
	FChan* Chan = nullptr;   // PIMPL: hides grpc types from the header
};

/** Factory binder installed for the grpc:// request endpoint scheme. */
UURLabRpcClientTransport* MakeDmEnvRpcRpcClientTransport(UObject* Outer);
