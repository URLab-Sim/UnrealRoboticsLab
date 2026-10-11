// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "Transport/RpcClientTransport.h"

#include "Transport/ZmqRpcClientTransport.h"
#include "Transport/MjExternalTransportProvider.h"
#include "Transport/TransportBase.h"
#include "Utils/URLabLogging.h"

UURLabRpcClientTransport* UURLabRpcClientTransport::Create(UObject* Outer, const FString& Endpoint)
{
	// An optional external module (SHM / ROS / gRPC) may supply the request/reply
	// client; when none is installed the core uses ZMQ. Either way the caller names
	// only the role, never the backend.
	UURLabRpcClientTransport* Client = nullptr;
	// The backend is chosen by the endpoint scheme: "tcp://host:port" (and a
	// scheme-less "host:port") is the built-in ZMQ request/reply client;
	// "grpc://host:port" and "shm://<session-dir>" are non-ZMQ backends supplied by
	// optional external modules. Each module registers its factory in the
	// scheme->factory map under its own scheme, so gRPC and SHM answer their own
	// scheme side by side and no registration ever hijacks a ZMQ owner.
	const FString Scheme = FMjExternalTransportProvider::SchemeOf(Endpoint);
	if (const FMjMakeExternalRpcClientTransport* Factory =
			FMjExternalTransportProvider::RpcClientTransportFactories.Find(Scheme))
	{
		if (Factory->IsBound())
		{
			Client = Factory->Execute(Outer);
		}
	}
	if (!Client)
	{
		// Only ZMQ's own schemes may fall back to ZMQ. A "grpc://" endpoint on a
		// platform where URLabDmEnvRpc is excluded, or a "shm://" endpoint that no
		// module claims, used to be handed to a ZMQ socket and fail with an error
		// naming neither the backend nor the missing module; fail loudly instead.
		if (!FMjExternalTransportProvider::IsBuiltInZmqScheme(Scheme))
		{
			UE_LOG(LogURLabNet, Error, TEXT("[RpcClient] %s"),
				*FMjExternalTransportProvider::DescribeUnavailableScheme(Scheme, Endpoint));
			return nullptr;
		}
		Client = NewObject<UURLabZmqRpcClientTransport>(Outer);
	}
	Client->Configure(Endpoint);
	if (!Client->TransportInit())
	{
		return nullptr;
	}
	return Client;
}
