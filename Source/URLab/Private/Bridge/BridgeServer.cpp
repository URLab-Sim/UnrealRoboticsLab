// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "Bridge/BridgeServer.h"
#include "Bridge/RpcDispatcher.h"
#include "Transport/ZmqRpcTransport.h"
#include "Transport/ShmRpcTransport.h"
#include "MuJoCo/Core/AMjManager.h"
#include "Utils/URLabLogging.h"

UURLabBridgeServer::UURLabBridgeServer() = default;

void UURLabBridgeServer::BeginDestroy()
{
	Stop();
	Super::BeginDestroy();
}

void UURLabBridgeServer::Start(const FString& StepEndpoint)
{
	if (!Dispatcher.IsValid())
	{
		Dispatcher = MakeUnique<FURLabRpcDispatcher>();
	}

	// Empty endpoint: dispatcher only, no transports (test path).
	if (StepEndpoint.IsEmpty())
		return;

	EnsureZmqBound(StepEndpoint);
}

bool UURLabBridgeServer::EnsureZmqBound(const FString& Endpoint)
{
	if (Endpoint.IsEmpty())
		return false;

	if (!Dispatcher.IsValid())
	{
		Dispatcher = MakeUnique<FURLabRpcDispatcher>();
	}

	for (const TObjectPtr<UURLabRpcTransport>& T : RpcTransports)
	{
		UURLabZmqRpcTransport* Existing = Cast<UURLabZmqRpcTransport>(T);
		if (Existing && Existing->StepEndpoint == Endpoint)
			return true;
	}

	UURLabZmqRpcTransport* Zmq = NewObject<UURLabZmqRpcTransport>(this, TEXT("BridgeZmqTransport"));
	Zmq->StepEndpoint = Endpoint;
	Zmq->SetOwningBridge(this);
	if (!Zmq->TransportInit())
	{
		UE_LOG(LogURLabNet, Error,
			TEXT("UURLabBridgeServer: ZMQ bind failed on %s"), *Endpoint);
		return false;
	}
	RpcTransports.Add(Zmq);
	UE_LOG(LogURLabNet, Log,
		TEXT("UURLabBridgeServer: ZMQ REP bound at %s"), *Endpoint);
	return true;
}

bool UURLabBridgeServer::EnsureShmBound(const FString& SessionId)
{
	if (!Dispatcher.IsValid())
	{
		Dispatcher = MakeUnique<FURLabRpcDispatcher>();
	}

	const FString Sid = SessionId.IsEmpty() ? FString(TEXT("live")) : SessionId;

	for (const TObjectPtr<UURLabRpcTransport>& T : RpcTransports)
	{
		UURLabShmRpcTransport* Existing = Cast<UURLabShmRpcTransport>(T);
		if (Existing)
		{
			const FString ExistingSid = Existing->SessionId.IsEmpty()
										  ? FString(TEXT("live"))
										  : Existing->SessionId;
			if (ExistingSid == Sid)
				return true;
		}
	}

	UURLabShmRpcTransport* Shm = NewObject<UURLabShmRpcTransport>(this, TEXT("BridgeShmTransport"));
	Shm->SessionId = SessionId; // empty -> defaults to "live" inside Init
	Shm->SetOwningBridge(this);
	if (!Shm->TransportInit())
	{
		UE_LOG(LogURLabNet, Error,
			TEXT("UURLabBridgeServer: SHM open failed (session=%s)"), *Sid);
		return false;
	}
	RpcTransports.Add(Shm);
	UE_LOG(LogURLabNet, Log,
		TEXT("UURLabBridgeServer: SHM RPC bound (session=%s)"), *Sid);
	return true;
}

void UURLabBridgeServer::Stop()
{
	// Drain before tearing transports down so blocking handlers see the
	// flag on their next 50ms tick and return `shutting_down` instead of
	// pinning the worker thread.
	if (Dispatcher.IsValid())
	{
		Dispatcher->SetDraining(true);
	}

	for (const TObjectPtr<UURLabRpcTransport>& T : RpcTransports)
	{
		if (T)
			T->TransportShutdown();
	}
	RpcTransports.Reset();

	if (!Dispatcher.IsValid())
		return;
	Dispatcher->Shutdown();
	Dispatcher.Reset();
	ActiveManager.Reset();
}

void UURLabBridgeServer::RegisterManager(AAMjManager* InManager)
{
	ActiveManager = InManager;
	if (Dispatcher.IsValid() && InManager)
	{
		Dispatcher->Init(InManager);
	}
}

void UURLabBridgeServer::UnregisterManager(AAMjManager* InManager)
{
	// Idempotent against mismatched cleanup orderings during PIE teardown.
	if (ActiveManager.Get() != InManager)
		return;
	if (Dispatcher.IsValid())
	{
		// Per-cycle teardown only — session, encoding, and observation
		// level are bridge-level and survive PIE end / level changes.
		// BridgeServer::Stop is what fully drops them.
		Dispatcher->OnManagerGone();
	}
	ActiveManager.Reset();
}
