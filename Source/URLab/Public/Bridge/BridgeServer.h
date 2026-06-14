// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "Bridge/RpcDispatcher.h"
#include "BridgeServer.generated.h"

class AAMjManager;
class UURLabRpcTransport;

/**
 * @class UURLabBridgeServer
 * @brief Owns the FURLabRpcDispatcher and every RPC transport.
 *
 * Bridge-owned transports are the single source of truth: the editor
 * subsystem owns one `UURLabBridgeServer` across PIE cycles, the cooked
 * path owns one per AAMjManager. Both call `EnsureZmqBound` /
 * `EnsureShmBound` to bring up the wire — the bridge `NewObject`s the
 * concrete transport, sets the owning-bridge weak ref, calls
 * `TransportInit`, and stores it in `RpcTransports`.
 */
UCLASS()
class URLAB_API UURLabBridgeServer : public UObject
{
	GENERATED_BODY()

public:
	UURLabBridgeServer();
	virtual void BeginDestroy() override;

	/** Construct the dispatcher and optionally bind a ZMQ REP listener.
	 *  Empty endpoint skips binding (test path). Idempotent. */
	void Start(const FString& StepEndpoint = TEXT("tcp://0.0.0.0:5559"));

	/** Tear down the dispatcher and every transport. Idempotent. */
	void Stop();

	bool IsRunning() const { return Dispatcher.IsValid(); }

	bool HasBoundRpcTransport() const { return RpcTransports.Num() > 0; }

	/** Bind a ZMQ REP listener if not already bound on `Endpoint`. */
	bool EnsureZmqBound(const FString& Endpoint);

	/** Open req.shm / rep.shm under `SessionId` if not already open.
	 *  Empty string means "live". */
	bool EnsureShmBound(const FString& SessionId = TEXT(""));

	/** Dispatcher when running, nullptr otherwise. */
	FURLabRpcDispatcher* GetDispatcher() const { return Dispatcher.Get(); }

	/** True when AAMjManager owns this server (cooked path, or editor
	 *  without subsystem auto-start). EndPlay tears it down only when so. */
	bool IsOwnedByManager() const { return bOwnedByManager; }
	void SetOwnedByManager(bool b) { bOwnedByManager = b; }

	/** Called from AAMjManager BeginPlay / EndPlay so the dispatcher can
	 *  resolve the live PIE manager regardless of who owns the server. */
	void RegisterManager(AAMjManager* InManager);
	void UnregisterManager(AAMjManager* InManager);

	/** Live PIE manager when one is registered, nullptr otherwise. */
	AAMjManager* GetActiveManager() const { return ActiveManager.Get(); }

	/** Bridge-owned RPC transports. Exposed for tests; production code
	 *  doesn't need to inspect these directly. */
	const TArray<TObjectPtr<UURLabRpcTransport>>& GetRpcTransports() const { return RpcTransports; }

private:
	TUniquePtr<FURLabRpcDispatcher> Dispatcher;
	TWeakObjectPtr<AAMjManager> ActiveManager;
	bool bOwnedByManager = false;

	/** Every bound RPC transport. Survives PIE transitions. Transient so
	 *  UE GC won't try to serialise these alongside the bridge UObject. */
	UPROPERTY(Transient)
	TArray<TObjectPtr<UURLabRpcTransport>> RpcTransports;
};
