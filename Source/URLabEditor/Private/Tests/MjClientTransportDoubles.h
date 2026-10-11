// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
//
// Test doubles for the client transport scheme-routing tests
// (URLab.Transport.ClientSchemeSelectsBackend). Each stands in for an external
// backend (gRPC / SHM) a scheme->factory registration would supply, WITHOUT
// opening any socket: Configure records the endpoint, TransportInit is a no-op
// that succeeds, and GetTransportName returns a sentinel the test asserts on.

#pragma once

#include "CoreMinimal.h"
#include "Transport/ClientSubscribeTransport.h"
#include "Transport/RpcClientTransport.h"
#include "MjClientTransportDoubles.generated.h"

/** Socket-free stand-in for an external client-subscribe backend. */
UCLASS()
class UMjFakeClientSubscribeTransport : public UURLabClientSubscribeTransport
{
	GENERATED_BODY()

public:
	FString ConfiguredEndpoint;
	FString SentinelName = TEXT("fake-sub");

	virtual void Configure(const FString& Endpoint, const FString& /*Topic*/,
		FOnClientMessage /*Callback*/) override
	{
		ConfiguredEndpoint = Endpoint;
	}
	virtual bool TransportInit() override { return true; }
	virtual void TransportShutdown() override {}
	virtual FString GetTransportName() const override { return SentinelName; }
};

/** Socket-free stand-in for an external client request/reply backend. */
UCLASS()
class UMjFakeRpcClientTransport : public UURLabRpcClientTransport
{
	GENERATED_BODY()

public:
	FString ConfiguredEndpoint;
	FString SentinelName = TEXT("fake-req");

	virtual void Configure(const FString& Endpoint) override
	{
		ConfiguredEndpoint = Endpoint;
	}
	virtual bool Request(const TArray<uint8>& /*Payload*/, TArray<uint8>& /*OutReply*/,
		int32 /*TimeoutMs*/) override
	{
		return false;
	}
	virtual bool TransportInit() override { return true; }
	virtual void TransportShutdown() override {}
	virtual FString GetTransportName() const override { return SentinelName; }
};
