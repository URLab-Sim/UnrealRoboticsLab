// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "Transport/RpcClientTransport.h"

#include "Transport/ZmqRpcClientTransport.h"
#include "Transport/MjExternalTransportProvider.h"

UURLabRpcClientTransport* UURLabRpcClientTransport::Create(UObject* Outer, const FString& Endpoint)
{
	// An optional external module (SHM / ROS / gRPC) may supply the request/reply
	// client; when none is installed the core uses ZMQ. Either way the caller names
	// only the role, never the backend.
	UURLabRpcClientTransport* Client = nullptr;
	// The backend is chosen by the endpoint scheme (source-of-truth 9.1):
	// "tcp://host:port" is the built-in ZMQ request/reply client; "grpc://host:port"
	// and "shm://<session-dir>" are non-ZMQ backends supplied by an optional
	// external module through the pluggable factory hook -- so binding that hook
	// never hijacks ZMQ owners. (The per-scheme factory map that lets gRPC and SHM
	// each answer their own scheme side by side lands in step 5.2; today the one
	// installed external module answers a non-ZMQ scheme, else the core falls back
	// to ZMQ.)
	const bool bExternalScheme =
		Endpoint.StartsWith(TEXT("grpc://")) || Endpoint.StartsWith(TEXT("shm://"));
	if (bExternalScheme && FMjExternalTransportProvider::MakeRpcClientTransport.IsBound())
	{
		Client = FMjExternalTransportProvider::MakeRpcClientTransport.Execute(Outer);
	}
	if (!Client)
	{
		Client = NewObject<UURLabZmqRpcClientTransport>(Outer);
	}
	Client->Configure(Endpoint);
	if (!Client->TransportInit())
	{
		return nullptr;
	}
	return Client;
}
