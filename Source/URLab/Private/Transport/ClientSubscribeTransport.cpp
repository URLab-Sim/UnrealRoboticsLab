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
	// Backend by endpoint scheme: "grpc://host:port" uses the external gRPC
	// subscribe backend when installed; everything else (tcp://) is ZMQ -- so
	// binding the gRPC hook never hijacks a ZMQ transform bus.
	if (Endpoint.StartsWith(TEXT("grpc://")) && FMjExternalTransportProvider::MakeClientSubscribeTransport.IsBound())
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
