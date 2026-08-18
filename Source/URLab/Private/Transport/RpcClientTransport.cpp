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
	if (FMjExternalTransportProvider::MakeRpcClientTransport.IsBound())
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
