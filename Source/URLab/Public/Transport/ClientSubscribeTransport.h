// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "Transport/TransportBase.h"
#include "ClientSubscribeTransport.generated.h"

/**
 * @struct FMjRenderDebugCaps
 * @brief The debug tier a render subscriber asks the owner to serialise onto each
 * streamed frame. Plain POD (no reflection): threaded through the render bus
 * into the concrete subscribe transport, which folds it into its
 * subscribe request. Default is all-off, so a lean subscriber negotiates nothing
 * and pays zero extra bytes.
 *
 * The field roles map 1:1 onto the Python owner's `parse_render_debug_caps`:
 * bContacts -> `contacts`, bOverlay -> `overlay`, MaxContacts -> the
 * `maxcontacts` cap (0 = uncapped).
 */
struct FMjRenderDebugCaps
{
	bool bContacts = false;
	bool bOverlay = false;
	int32 MaxContacts = 0;   // 0 = owner streams all contacts

	/** True when the subscriber wants any part of the heavier debug tier. */
	bool WantsAny() const { return bContacts || bOverlay; }
};

/**
 * @class UURLabClientSubscribeTransport
 * @brief Client-side receiver of a server's broadcast.
 *
 * The counterpart to UURLabPublishTransport: a publish transport fans a topic
 * out from the authoritative sim; this connects to that broadcast and delivers
 * each `[topic][payload]` message to a consumer callback. It is the shared
 * shape behind the render sinks -- the renderer applying streamed
 * transforms directly.
 *
 * Concrete backends own a worker thread and fire the callback FROM that thread,
 * so a consumer must treat the callback as off-game-thread.
 */
UCLASS(Abstract)
class URLAB_API UURLabClientSubscribeTransport : public UURLabTransportBase
{
	GENERATED_BODY()

public:
	/** Per-message delegate: topic + raw payload bytes. Fires on the transport's
	 *  worker thread -- the consumer does its own decode + hand-off. */
	DECLARE_DELEGATE_TwoParams(FOnClientMessage, const FString& /*Topic*/,
		const TArray<uint8>& /*Payload*/);

	/** Construct + connect the client-subscribe backend for `Endpoint`'s scheme,
	 *  owned by Outer, delivering `Topic` payloads to `Callback`. The backend is
	 *  chosen here so every render sink shares one construction seam instead of
	 *  naming a concrete transport. Returns the started transport, or nullptr
	 *  when no backend serves that scheme, the chosen backend does not carry a
	 *  state stream, or it could not connect. */
	static UURLabClientSubscribeTransport* Create(UObject* Outer,
		const FString& Endpoint, const FString& Topic, FOnClientMessage Callback,
		const FMjRenderDebugCaps& DebugCaps = FMjRenderDebugCaps());

	/** Source endpoint (e.g. "tcp://host:port"), topic to subscribe, and the
	 *  delivery callback. Call before TransportInit. */
	virtual void Configure(const FString& Endpoint, const FString& Topic,
		FOnClientMessage Callback)
		PURE_VIRTUAL(UURLabClientSubscribeTransport::Configure, );

	/** Ask the owner to serialise the heavier debug tier (contacts/overlay) onto
	 *  each streamed frame. Backends that carry a per-subscription request fold
	 *  these caps into their subscribe payload (gRPC) or into the topic they
	 *  subscribe to (ZMQ). Call before TransportInit (Create does this between
	 *  Configure and TransportInit).
	 *
	 *  Returns whether the backend will carry the request. The base returns false:
	 *  a backend with no way to ask says so, and Create turns that into one line
	 *  naming the transport, instead of the subscriber waiting forever for decor that
	 *  was never negotiated. */
	virtual bool SetRenderDebugCaps(const FMjRenderDebugCaps& /*DebugCaps*/) { return false; }

};
