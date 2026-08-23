// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "Transport/ClientSubscribeTransport.h"

#include "Transport/ZmqClientSubscribeTransport.h"
#include "Transport/MjExternalTransportProvider.h"

UURLabClientSubscribeTransport* UURLabClientSubscribeTransport::Create(UObject* Outer,
	const FString& Endpoint, const FString& Topic, FOnClientMessage Callback)
{
	// An optional external module (SHM / ROS / gRPC-dm_env) may supply the render
	// sink; when none is installed the core uses ZMQ. Either way the caller names
	// only the role, never the backend.
	UURLabClientSubscribeTransport* Sub = nullptr;
	// Backend by endpoint scheme (source-of-truth 9.1): "tcp://host:port" is the
	// built-in ZMQ transform bus; "grpc://host:port" and "shm://<session-dir>"
	// are non-ZMQ backends supplied by an optional external module through the
	// pluggable factory hook -- so binding that hook never hijacks a ZMQ bus.
	// (The per-scheme factory map that lets gRPC and SHM each answer their own
	// scheme side by side lands in step 5.2; today the one installed external
	// module answers a non-ZMQ scheme, else the core falls back to ZMQ.)
	const bool bExternalScheme =
		Endpoint.StartsWith(TEXT("grpc://")) || Endpoint.StartsWith(TEXT("shm://"));
	if (bExternalScheme && FMjExternalTransportProvider::MakeClientSubscribeTransport.IsBound())
	{
		Sub = FMjExternalTransportProvider::MakeClientSubscribeTransport.Execute(Outer);
	}
	if (!Sub)
	{
		Sub = NewObject<UURLabZmqClientSubscribeTransport>(Outer);
	}
	Sub->Configure(Endpoint, Topic, Callback);
	if (!Sub->TransportInit())
	{
		return nullptr;
	}
	return Sub;
}
