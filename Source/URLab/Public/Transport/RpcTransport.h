// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "RpcTransport.generated.h"

class UURLabBridgeServer;
class FURLabRpcDispatcher;

/**
 * @class UURLabRpcTransport
 * @brief Abstract base for request/reply transports owned by the bridge.
 *
 * UObject-only (NOT UActorComponent) so transports live and die with
 * the bridge regardless of PIE state.
 *
 * Concrete transports (`UURLabZmqRpcTransport`, `UURLabShmRpcTransport`)
 * own their own worker thread + transport handles. They implement:
 *   - `TransportInit` / `TransportShutdown`: bind/release handles.
 *   - The poll loop hands raw bytes to `ProcessRequestBytes` and ships
 *     the encoded reply bytes back to the wire.
 *
 * Bytes-in / bytes-out is shared on this base: msgpack-or-JSON detect,
 * dispatcher resolution, dispatch, reply encoding. Lives here so every
 * concrete transport gets the same wire framing without duplication.
 */
UCLASS(Abstract)
class URLAB_API UURLabRpcTransport : public UObject
{
    GENERATED_BODY()

public:
    /** Bridge that owns this transport. Set once at construction by the
     *  bridge `NewObject<...>(this, ...)` + `SetOwningBridge(this)` pair.
     *  Used by `ResolveDispatcher` to find the live dispatcher. */
    void SetOwningBridge(UURLabBridgeServer* Bridge);
    UURLabBridgeServer* GetOwningBridge() const { return OwningBridge.Get(); }

    /** Bind sockets / mmap regions / start the worker thread. Returns
     *  false on failure (port collision, SHM open failure). The bridge
     *  treats false as fatal for that transport — logs and removes it
     *  from `RpcTransports`. Idempotent: repeat calls return true. */
    virtual bool TransportInit() PURE_VIRTUAL(UURLabRpcTransport::TransportInit, return false;);

    /** Stop the worker thread, release handles. Idempotent. */
    virtual void TransportShutdown() PURE_VIRTUAL(UURLabRpcTransport::TransportShutdown, );

    /** Human-readable transport name for logs ("zmq" / "shm" / future
     *  "ros"). Used in error messages and the SHM-rejects-editor-ops
     *  path. */
    virtual FString GetTransportName() const PURE_VIRTUAL(UURLabRpcTransport::GetTransportName, return FString(); );

    /** Categorisation: does this transport accept editor-only ops?
     *  ZMQ accepts everything; SHM rejects editor-only ops with a
     *  `wrong_transport: use_zmq` reply (SHM scope narrowing).
     *  Default true so new transports are universal unless they opt out. */
    virtual bool AcceptsEditorOps() const { return true; }

    /** Shared request handler. Concrete transports call this from their
     *  worker loop with raw inbound bytes; receives encoded reply bytes
     *  ready to ship back. Handles:
     *   1. Wire detection (msgpack vs JSON by leading byte).
     *   2. Parse to FJsonObject.
     *   3. Route to dispatcher (via `ResolveDispatcher`).
     *   4. SHM-only editor-op rejection (when `AcceptsEditorOps()` is false).
     *   5. Encode reply (msgpack default, JSON when handshake flag set).
     *
     *  Returns true if a reply was produced. False indicates a parse
     *  failure where the caller should ship an error reply itself
     *  (rare; current flow always emits a reply). */
    bool ProcessRequestBytes(const TArray<uint8>& InBytes, TArray<uint8>& OutReplyBytes);

    /** Resolve the live dispatcher. Always goes through the owning
     *  bridge. Returns nullptr if the bridge has been torn down —
     *  caller should ship a `not_ready` error reply. */
    FURLabRpcDispatcher* ResolveDispatcher() const;

protected:
    TWeakObjectPtr<UURLabBridgeServer> OwningBridge;
};
