// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "HAL/ThreadSafeBool.h"
#include "Transport/RpcClientTransport.h"
#include "Transport/ClientSubscribeTransport.h"
#include "DmEnvRpcClientTransports.generated.h"

/**
 * gRPC (dm_env_rpc) CLIENT-side transports so a UE Mirror renderer can reach a
 * fast-path OWNER over gRPC -- the peek/mirror direction (owner is the source,
 * the mirror subscribes). Two backends plug into the core's existing factory
 * hooks and are selected when the endpoint is "grpc://host:port":
 *   - Rpc client  (FetchModel via fastpath_hello, perturb) -> FMjMakeExternalRpcClientTransport
 *   - Subscribe   (the owner's transform view stream)       -> FMjMakeExternalClientSubscribeTransport
 * The renderer's model-build + bxpos/bxquat apply path is reused unchanged; only
 * the wire under it changes from ZMQ to gRPC.
 */

/** Request/reply over gRPC: one UrlabPacket per call (op read from the msgpack
 *  request, payload passed through). Used by MjRendererDriverClient for
 *  fastpath_hello (model fetch) and fastpath_perturb. */
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
	virtual void BeginDestroy() override;

private:
	FString Endpoint;   // host:port (grpc:// stripped)
	struct FChan;
	FChan* Chan = nullptr;   // PIMPL: hides grpc types from the header
};

/** Server-stream over gRPC: subscribe(format=transforms) -> a stream of the
 *  owner's per-body transform frames, delivered to the mirror's OnBusMessage
 *  sink exactly like a ZMQ "geoms" subscriber. */
UCLASS()
class UURLabDmEnvRpcClientSubscribeTransport : public UURLabClientSubscribeTransport
{
	GENERATED_BODY()
public:
	virtual void Configure(const FString& InEndpoint, const FString& InTopic, FOnClientMessage InCallback) override;
	virtual bool TransportInit() override;
	virtual void TransportShutdown() override;
	virtual FString GetTransportName() const override { return TEXT("dmenv-grpc-sub"); }
	virtual void BeginDestroy() override;

	/** Worker-thread body (public so the FRunnable can drive it). */
	void RunLoop();

private:
	FString Endpoint;   // host:port (grpc:// stripped)
	FString Topic;
	FOnClientMessage OnMessage;

	struct FSubState;
	FSubState* Sub = nullptr;   // PIMPL: grpc channel/stub/context/thread
	FThreadSafeBool bStop;
};

/** Factory binders installed on the core's provider hooks in the module startup
 *  (selected by the "grpc://" endpoint scheme). */
UURLabRpcClientTransport* MakeDmEnvRpcRpcClientTransport(UObject* Outer);
UURLabClientSubscribeTransport* MakeDmEnvRpcClientSubscribeTransport(UObject* Outer);
