// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "Containers/Ticker.h"
#include "Bridge/RpcDispatcher.h"
#include "Bridge/BridgeServerConfig.h"
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

	/** Bring up the optional out-of-core transport surface if not already up: the
	 *  state publish transport (registered with the active manager's fan-out) and
	 *  the control RPC transport (stored in `RpcTransports`), both created through
	 *  the FMjExternalTransportProvider factory hooks. No-op returning false when
	 *  no external transport module is loaded or its runtime is unavailable. */
	bool EnsureExternalTransportsBound();

	/** Dispatcher when running, nullptr otherwise. */
	FURLabRpcDispatcher* GetDispatcher() const { return Dispatcher.Get(); }

	/** Resolved per-instance config (ports, bind address, instance identity).
	 *  Set by the owner before Start so the handshake `instance` block and the
	 *  manager-owned state/camera endpoints read one source of truth. */
	void SetInstanceConfig(const FURLabBridgeServerConfig& InConfig) { InstanceConfig = InConfig; }
	const FURLabBridgeServerConfig& GetInstanceConfig() const { return InstanceConfig; }

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

	// --- Cooperative render-farm lease (not a security boundary) ---
	// One lease per process. A pool client claims this instance so the pool
	// won't hand the same editor process to a second client. Guarded by
	// LeaseMutex since RPC threads on multiple transports may touch it.

	/** Claim the lease if free. On success returns true and fills OutLeaseId
	 *  with a fresh id; when already held returns false and fills
	 *  OutExistingLeaseId with the current holder's id. Lazily expires an idle
	 *  lease past its TTL before deciding. */
	bool TryAcquireLease(const FString& Owner, double TtlSeconds,
		FString& OutLeaseId, FString& OutExistingLeaseId);

	/** Release the lease when InLeaseId matches the current holder. Returns
	 *  false when no lease is held or the id doesn't match. */
	bool ReleaseLease(const FString& InLeaseId);

	/** Refresh the activity timestamp so an active client keeps its lease.
	 *  No-op when no lease is held. */
	void TouchLease();

	/** True when a lease is currently held. Performs lazy TTL expiry: an idle
	 *  lease past its TTL is auto-released before the state is reported. */
	bool IsLeaseHeld();

	/** Current lease id, or empty when none is held. */
	FString GetLeaseId() const;

	/** Test seam: pin the lease clock to a fixed value so TTL expiry is
	 *  deterministic without sleeping. Every lease op consults it — including
	 *  TouchLease on the dispatch path — so a driven Dispatch() sees injected
	 *  time. A negative value restores the wall clock. */
	void SetLeaseClockForTest(double NowSeconds);

private:
	/** Construct the dispatcher if needed and wire its back-pointer to this
	 *  server so no-manager ops (e.g. leasing) can reach per-instance state. */
	void EnsureDispatcher();

	// --- Broker-less discovery registry (source-of-truth §12) ---
	// The bridge server owns the registry entry lifecycle so that BOTH the
	// editor subsystem-owned server AND the cooked/packaged manager-owned
	// server register: WriteEntry on Start, a 10 s heartbeat refresh, and
	// RemoveEntry on Stop. This is the single writer — the editor subsystem no
	// longer writes directly — so an instance never writes two entries.

	/** Write (or overwrite) this instance's registry entry from InstanceConfig.
	 *  manager_present and busy are derived live. */
	void WriteRegistryEntry();

	/** Ticker callback: refresh the entry mtime + live busy/manager fields.
	 *  Returns false (stop ticking) once the dispatcher is gone. */
	bool RefreshRegistryHeartbeat(float DeltaTime);

	/** Ticker handle for the registry heartbeat; invalid when not running. */
	FTSTicker::FDelegateHandle RegistryHeartbeatHandle;

	/** URLab version string, cached from the dispatcher at first write. */
	FString CachedUrlabVersion;

	/** Current lease clock: the test override when set (>= 0), else the wall
	 *  clock. Callers hold LeaseMutex. */
	double LeaseNow() const;

	/** Lazy-expire then report lease state. Assumes LeaseMutex is held. */
	bool IsLeaseHeldInternal(double NowSeconds);

	mutable FCriticalSection LeaseMutex;
	bool bLeaseHeld = false;
	FString LeaseId;
	double LeaseTtlSeconds = 0.0;
	double LeaseLastActivitySeconds = 0.0;
	double LeaseClockOverrideForTest = -1.0;

	TUniquePtr<FURLabRpcDispatcher> Dispatcher;
	TWeakObjectPtr<AAMjManager> ActiveManager;
	bool bOwnedByManager = false;

	/** Resolved config, set via SetInstanceConfig. Defaults reproduce the
	 *  single-editor behaviour when no owner supplies one. */
	FURLabBridgeServerConfig InstanceConfig;

	// Disable editor frame pacing (VSync + FPS cap) while a real transport is
	// bound, so camera frame delivery isn't capped at the monitor refresh.
	// Prior cvar values are saved and restored on Stop.
	void ApplyPerformanceOverrides();
	void RestorePerformanceOverrides();

	bool bPacingOverridden = false;
	bool bHadVSync = false;
	bool bHadVSyncEditor = false;
	bool bHadMaxFPS = false;
	bool bHadSlateThrottle = false;
	bool bHadIdleWhenNotForeground = false;
	float SavedVSync = 1.0f;
	float SavedVSyncEditor = 1.0f;
	float SavedMaxFPS = 0.0f;
	float SavedSlateThrottle = 1.0f;
	float SavedIdleWhenNotForeground = 0.0f;

	/** Every bound RPC transport. Survives PIE transitions. Transient so
	 *  UE GC won't try to serialise these alongside the bridge UObject. */
	UPROPERTY(Transient)
	TArray<TObjectPtr<UURLabRpcTransport>> RpcTransports;
};
