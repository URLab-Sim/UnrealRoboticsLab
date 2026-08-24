// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "Transport/ClientSubscribeTransport.h"

#include "Transport/ZmqClientSubscribeTransport.h"
#include "Transport/MjExternalTransportProvider.h"

UURLabClientSubscribeTransport* UURLabClientSubscribeTransport::Create(UObject* Outer,
	const FString& Endpoint, const FString& Topic, FOnClientMessage Callback,
	const FMjRenderDebugCaps& DebugCaps)
{
	// An optional external module (SHM / ROS / gRPC-dm_env) may supply the render
	// sink; when none is installed the core uses ZMQ. Either way the caller names
	// only the role, never the backend.
	UURLabClientSubscribeTransport* Sub = nullptr;
	// Backend by endpoint scheme (source-of-truth 9.1): "tcp://host:port" is the
	// built-in ZMQ transform bus; "grpc://host:port" and "shm://<session-dir>"
	// are non-ZMQ backends supplied by optional external modules. Each module
	// registers its factory in the scheme->factory map under its own scheme, so
	// gRPC and SHM answer their own scheme side by side and no registration ever
	// hijacks a ZMQ bus. No entry for a scheme (including "tcp") => ZMQ fallback.
	const FString Scheme = FMjExternalTransportProvider::SchemeOf(Endpoint);
	if (const FMjMakeExternalClientSubscribeTransport* Factory =
			FMjExternalTransportProvider::ClientSubscribeTransportFactories.Find(Scheme))
	{
		if (Factory->IsBound())
		{
			Sub = Factory->Execute(Outer);
		}
	}
	if (!Sub)
	{
		Sub = NewObject<UURLabZmqClientSubscribeTransport>(Outer);
	}
	Sub->Configure(Endpoint, Topic, Callback);
	// Negotiate the debug tier (if any) BEFORE the worker starts, so the backend's
	// first subscribe request already carries the caps. Default caps are all-off,
	// and backends that can't carry them (ZMQ) inherit the base no-op.
	Sub->SetRenderDebugCaps(DebugCaps);
	if (!Sub->TransportInit())
	{
		return nullptr;
	}
	return Sub;
}
