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
#include "MuJoCo/Core/MjCapabilities.h"
#include "Transport/ShmSession.h"
#include "BridgeServer.generated.h"

class UURLabRpcTransport;

/**
 * @class UURLabBridgeServer
 * @brief Owns the FURLabRpcDispatcher and every RPC transport.
 *
 * Bridge-owned transports are the single source of truth: the editor
 * subsystem owns one `UURLabBridgeServer` across PIE cycles, the cooked
 * path one per world (UURLabBridgeSubsystem). They call the appropriate
 * bind method to bring up the wire — the bridge `NewObject`s the
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
	void Start(const FString& StepEndpoint);

	/** Tear down the dispatcher and every transport. Idempotent. */
	void Stop();

	bool IsRunning() const { return Dispatcher.IsValid(); }

	bool HasBoundRpcTransport() const { return RpcTransports.Num() > 0; }

	/** Bind a ZMQ REP listener if not already bound on `Endpoint`. */
	bool EnsureZmqBound(const FString& Endpoint);

	/** Bring up the optional out-of-core transport surface if not already up: the
	 *  control RPC transports (stored in `RpcTransports`) created through
	 *  the FMjExternalTransportProvider factory hooks. No-op returning false when
	 *  no external transport module is loaded or its runtime is unavailable. */
	bool EnsureExternalTransportsBound();

	/** Dispatcher when running, nullptr otherwise. */
	FURLabRpcDispatcher* GetDispatcher() const { return Dispatcher.Get(); }

	/** Resolved per-instance config (ports, bind address, instance identity).
	 *  Set by the owner before Start so the handshake `instance` block and the
	 *  manager-owned state/camera endpoints read one source of truth. Supplying
	 *  one marks the config resolved, so ResolveInstanceConfig will not go back
	 *  to the INI and command line and produce a second, possibly different,
	 *  answer. */
	void SetInstanceConfig(const FURLabBridgeServerConfig& InConfig);
	const FURLabBridgeServerConfig& GetInstanceConfig() const { return InstanceConfig; }

	/** The instance config, loading it once (INI, then environment, then
	 *  -URLabNet=) if no owner supplied one. Every other reader -- the manager's
	 *  state PUB and render bus, the control ingress, the cameras -- goes
	 *  through this rather than repeating the load, because a second load at a
	 *  different moment can see a different environment and hand out a port the
	 *  bridge never bound. */
	const FURLabBridgeServerConfig& ResolveInstanceConfig();

	/** This instance's latest-stream shared-memory session: the directory the
	 *  state, render and camera rings use and the one hello advertises. */
	const FURLabShmSession& GetShmSession();

	// --- Granted capabilities ---
	// The bridge is where a peer's authority is decided, because the bridge is what a
	// peer reaches: a render server has no manager to ask, and a manager-owned answer is
	// how the camera grant came to live on an object half the instances did not have.

	/** Atomically replace the complete permission policy. Omitted permissions become false. */
	void SetGrants(const FMjCapabilitySet& InGrants);

	/** True when peers may do `Cap` here. Meaningful only for permission capabilities;
	 *  the other kinds are enforced where they are stood up or produced, not per request. */
	bool GrantsPermission(EMjCapability Cap) const;

	/** Every granted permission's advertised wire token, for the handshake and the
	 *  registry entry -- one source, so the two can never advertise different grants. */
	void CollectGrantTokens(TArray<FString>& Out) const;

	/** Bridge-owned RPC transports. Exposed for tests; production code
	 *  doesn't need to inspect these directly. */
	const TArray<TObjectPtr<UURLabRpcTransport>>& GetRpcTransports() const { return RpcTransports; }

	/** Dialable command endpoints backed by transports that are currently bound. */
	void CollectCommandEndpoints(TArray<FString>& Out) const;
	/** Pure endpoint-normalisation seam used by transport contract tests. */
	static FString NormalizeBoundCommandEndpoint(const FString& Bound, const FString& DialHost);

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
	void RefreshCommandEndpointSnapshot();
	/** Construct the dispatcher if needed and wire its back-pointer to this
	 *  server so no-manager ops (e.g. leasing) can reach per-instance state. */
	void EnsureDispatcher();

	// --- Broker-less discovery registry ---
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
	/** Unactivated bridges grant no peer authority. A workflow installs the complete policy. */
	FMjCapabilitySet Grants;
	mutable FCriticalSection AdvertisementMutex;
	TArray<FString> CommandEndpointSnapshot;

	/** Resolved config, set via SetInstanceConfig or loaded by
	 *  ResolveInstanceConfig. Defaults reproduce the single-editor behaviour. */
	FURLabBridgeServerConfig InstanceConfig;
	bool bInstanceConfigResolved = false;

	/** Resolved latest-stream SHM session; empty Dir until first use. */
	FURLabShmSession ShmSession;

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
