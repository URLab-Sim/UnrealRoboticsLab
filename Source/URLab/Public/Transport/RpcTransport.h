// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "Transport/TransportBase.h"
#include "RpcTransport.generated.h"

class FURLabRpcDispatcher;

/**
 * @class UURLabRpcTransport
 * @brief Abstract base for request/reply transports owned by the bridge.
 *
 * UObject-only (NOT UActorComponent) so transports live and die with
 * the bridge regardless of PIE state.
 *
 * Concrete transports such as `UURLabZmqRpcTransport`
 * own their own worker thread + transport handles. They implement:
 *   - `TransportInit` / `TransportShutdown`: bind/release handles.
 *   - The poll loop hands raw bytes to `ProcessRequestBytes` and ships
 *     the encoded reply bytes back to the wire.
 *
 * Bytes-in / bytes-out is shared on this base: MsgPack parsing,
 * dispatcher resolution, dispatch, reply encoding. Lives here so every
 * concrete transport gets the same wire framing without duplication.
 */
UCLASS(Abstract)
class URLAB_API UURLabRpcTransport : public UURLabTransportBase
{
	GENERATED_BODY()

public:
	/** Shared request handler. Concrete transports call this from their
	 *  worker loop with raw inbound bytes; receives encoded reply bytes
	 *  ready to ship back. Handles:
	 *   1. Reject non-MsgPack input.
	 *   2. Parse to FJsonObject.
	 *   3. Route to dispatcher (via `ResolveDispatcher`).
	 *   4. Encode the reply as MsgPack.
	 *
	 *  Returns true if a reply was produced. False indicates a parse
	 *  failure where the caller should ship an error reply itself
	 *  (rare; current flow always emits a reply). */
	bool ProcessRequestBytes(const TArray<uint8>& InBytes, TArray<uint8>& OutReplyBytes,
		const FString* OuterOp = nullptr);

	/** Resolve the live dispatcher. Always goes through the owning
	 *  bridge. Returns nullptr if the bridge has been torn down —
	 *  caller should ship a `not_ready` error reply. */
	FURLabRpcDispatcher* ResolveDispatcher() const;

	/** Encode a reply object to MsgPack wire bytes. For
	 *  transports that synthesize a reply outside the normal
	 *  ProcessRequestBytes path — e.g. a fixed-size transport rejecting an
	 *  oversize reply with a fast `wrong_transport`/`reply_too_large` error
	 *  instead of dropping it. */
	void EncodeReply(const TSharedPtr<class FJsonObject>& Reply, TArray<uint8>& OutBytes) const;
};
