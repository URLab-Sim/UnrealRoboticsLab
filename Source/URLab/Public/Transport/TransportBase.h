// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "TransportBase.generated.h"

class UURLabBridgeServer;

/**
 * @class UURLabTransportBase
 * @brief The one base under every transport role.
 *
 * Six role abstractions (publish, control ingress, client subscribe, RPC
 * server, RPC client, camera publish) each used to re-declare the same
 * lifecycle, the same owning-bridge weak pointer and the same handshake hook,
 * each carrying a comment claiming UE single inheritance made a shared base
 * impossible. Every one of them is a UObject, so it never was: this is that
 * base, and a role now adds only what is genuinely its own.
 *
 * Lifecycle contract, uniform across roles: `TransportInit` brings up handles
 * and returns false on failure; `TransportShutdown` releases them and is
 * idempotent; `BeginDestroy` calls it, so a transport that is simply dropped
 * still closes its sockets and joins its threads on the game thread rather
 * than in a destructor that may run on the GC thread.
 */
UCLASS(Abstract)
class URLAB_API UURLabTransportBase : public UObject
{
	GENERATED_BODY()

public:
	/** Bridge that owns (or hosts) this transport. Set once by the creator,
	 *  before TransportInit, so the transport can resolve the dispatcher and
	 *  the one resolved net config instead of loading its own. */
	void SetOwningBridge(UURLabBridgeServer* Bridge);
	UURLabBridgeServer* GetOwningBridge() const { return OwningBridge.Get(); }

	/** Bind sockets / mmap regions / start workers. False on failure; the
	 *  creator drops a transport that returns false. Idempotent: a repeat call
	 *  on an already-live transport returns true. */
	virtual bool TransportInit() PURE_VIRTUAL(UURLabTransportBase::TransportInit, return false;);

	/** Stop workers, release handles. Idempotent. */
	virtual void TransportShutdown() PURE_VIRTUAL(UURLabTransportBase::TransportShutdown, );

	/** Stable wire-facing identifier ("zmq", "shm", "zmq-cam", ...). Used for
	 *  dedup in the bridge's transport lists and in every log line, so it must
	 *  be unique across transports that can be bound at the same time. */
	virtual FString GetTransportName() const PURE_VIRTUAL(UURLabTransportBase::GetTransportName, return FString(););

	/** Append transport-specific fields to the hello handshake reply. Default
	 *  no-op; the SHM transports advertise their session paths through it. */
	virtual void AppendHandshakeBlock(TSharedPtr<class FJsonObject>& Reply) const {}

	// --- Per-step physics hooks (physics thread) --------------------------
	// Default-empty so a transport that publishes on demand rather than on the
	// step does not have to opt in. The manager calls these on the engine's
	// pre/post-step callbacks for the transports it owns.

	virtual void PreStep(struct mjModel_* /*m*/, struct mjData_* /*d*/) {}
	virtual void PostStep(struct mjModel_* /*m*/, struct mjData_* /*d*/) {}

	/** Shut down before the object is collected. A transport owning a worker
	 *  thread must not join it from ~UObject: that destructor can run on the
	 *  GC or async-loading thread. */
	virtual void BeginDestroy() override;

protected:
	TWeakObjectPtr<UURLabBridgeServer> OwningBridge;
};
