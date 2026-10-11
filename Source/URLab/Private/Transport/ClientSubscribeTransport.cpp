// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "Transport/ClientSubscribeTransport.h"

#include "Transport/ZmqClientSubscribeTransport.h"
#include "Transport/MjExternalTransportProvider.h"
#include "Transport/ShmClientSubscribeTransport.h"
#include "Transport/TransportBase.h"
#include "Utils/URLabLogging.h"

UURLabClientSubscribeTransport* UURLabClientSubscribeTransport::Create(UObject* Outer,
	const FString& Endpoint, const FString& Topic, FOnClientMessage Callback,
	const FMjRenderDebugCaps& DebugCaps)
{
	// An optional external module may supply the render
	// sink; when none is installed the core uses ZMQ. Either way the caller names
	// only the role, never the backend.
	UURLabClientSubscribeTransport* Sub = nullptr;
	// Backend by endpoint scheme: "tcp://host:port" (and a scheme-less "host:port")
	// is the built-in ZMQ transform bus; "shm://<session-dir>"
	// are non-ZMQ backends supplied by optional external modules. Each module
	// registers its factory in the scheme->factory map under its own scheme, so
	// external schemes answer their own factory and no registration ever
	// hijacks a ZMQ bus.
	const FString Scheme = FMjExternalTransportProvider::SchemeOf(Endpoint);
	if (const FMjMakeExternalClientSubscribeTransport* Factory =
			FMjExternalTransportProvider::ClientSubscribeTransportFactories.Find(Scheme))
	{
		if (Factory->IsBound())
		{
			Sub = Factory->Execute(Outer);
		}
	}
	if (!Sub && Scheme.Equals(TEXT("shm"), ESearchCase::IgnoreCase))
	{
		// Shared memory is in-core, not an optional module: the ring the owner
		// writes is the same FMjShmRegion the state and camera segments use, so the
		// consumer lives here beside the ZMQ one rather than behind a factory.
		Sub = NewObject<UURLabShmClientSubscribeTransport>(Outer);
	}
	if (!Sub)
	{
		// Only ZMQ's own schemes may fall back to ZMQ -- see the matching guard in
		// UURLabRpcClientTransport::Create. Anything else names a backend some other
		// module has to supply, and handing an unsupported scheme to ZMQ fails
		// with an error naming neither gRPC nor the module that was meant to serve it.
		if (!FMjExternalTransportProvider::IsBuiltInZmqScheme(Scheme))
		{
			UE_LOG(LogURLabNet, Error, TEXT("[ClientSubscribe] %s"),
				*FMjExternalTransportProvider::DescribeUnavailableScheme(Scheme, Endpoint));
			return nullptr;
		}
		Sub = NewObject<UURLabZmqClientSubscribeTransport>(Outer);
	}
	Sub->Configure(Endpoint, Topic, Callback);
	// Negotiate the debug tier (if any) BEFORE the worker starts, so the backend's
	// first subscribe -- request payload on gRPC, topic on ZMQ -- already carries
	// the caps. A backend that cannot ask says so, and the subscriber is told once
	// here rather than left waiting for decor nobody negotiated.
	if (!Sub->SetRenderDebugCaps(DebugCaps) && DebugCaps.WantsAny())
	{
		UE_LOG(LogURLabNet, Warning,
			TEXT("[ClientSubscribe] '%s' cannot negotiate the debug tier on '%s'; this subscriber ")
			TEXT("renders transforms only. Contacts and overlay decor need a transport whose ")
			TEXT("subscription reaches the owner: 'tcp://host:port'."),
			*Sub->GetTransportName(), *Endpoint);
	}
	if (!Sub->TransportInit())
	{
		return nullptr;
	}
	return Sub;
}
