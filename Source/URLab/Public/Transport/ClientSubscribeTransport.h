// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "ClientSubscribeTransport.generated.h"

/**
 * @struct FMjRenderDebugCaps
 * @brief The debug-tier a render mirror asks the owner to serialize onto each
 * streamed frame (source-of-truth §8.2). Plain POD (no reflection): threaded from
 * AMjRenderer down through UMjRendererBus into the concrete subscribe transport,
 * which folds it into its subscribe request. Default is all-off, so a lean mirror
 * negotiates nothing and pays zero extra bytes.
 *
 * The field roles map 1:1 onto the Python owner's `parse_render_debug_caps`
 * (owner_server.py / fastpath_owner.py): bContacts -> the `contacts` bool
 * (StreamContacts), bOverlay -> the `overlay` bool (StreamOverlay), MaxContacts ->
 * the `maxcontacts` int cap (0 = uncapped).
 */
struct FMjRenderDebugCaps
{
	bool bContacts = false;
	bool bOverlay = false;
	int32 MaxContacts = 0;   // 0 = owner streams all contacts

	/** True when the mirror wants any part of the heavier debug tier. */
	bool WantsAny() const { return bContacts || bOverlay; }
};

/**
 * @class UURLabClientSubscribeTransport
 * @brief Abstract base for a CLIENT-side receiver of a server's broadcast.
 *
 * The counterpart to UURLabPublishTransport: a publish transport fans a topic
 * out from the authoritative sim; this connects to that broadcast and delivers
 * each `[topic][payload]` message to a consumer callback. It is the shared
 * shape behind the lightweight render receivers -- the fast-path renderer
 * (AMjRenderer, which applies streamed transforms directly) -- which previously
 * hand-rolled the same raw-libzmq SUB + worker loop.
 *
 * Deliberately distinct from UURLabSubscribeTransport: that one is server-side
 * control-IN (draining messages into the authoritative mjData before a step);
 * this one is a client-side render sink with no bridge, no step hooks, and no
 * authority over the sim. UE single inheritance keeps the two bases separate.
 *
 * Concrete backends (ZMQ now; SHM / ROS / gRPC via the external-transport
 * provider hook) own a worker thread and fire the callback FROM that thread, so
 * a consumer must treat the callback as off-game-thread.
 */
UCLASS(Abstract)
class URLAB_API UURLabClientSubscribeTransport : public UObject
{
	GENERATED_BODY()

public:
	/** Per-message delegate: topic + raw payload bytes. Fires on the transport's
	 *  worker thread -- the consumer does its own decode + hand-off. */
	DECLARE_DELEGATE_TwoParams(FOnClientMessage, const FString& /*Topic*/,
		const TArray<uint8>& /*Payload*/);

	/** Construct + connect the configured client-subscribe backend, owned by
	 *  Outer, delivering `Topic` payloads from `Endpoint` to `Callback`. The
	 *  backend (ZMQ today; SHM / ROS / gRPC as they are added) is chosen here so
	 *  every render sink -- the fast-path transform bus and the viewer -- shares
	 *  one construction seam instead of naming a concrete transport. Returns the
	 *  started transport, or nullptr if the backend could not connect. */
	static UURLabClientSubscribeTransport* Create(UObject* Outer,
		const FString& Endpoint, const FString& Topic, FOnClientMessage Callback,
		const FMjRenderDebugCaps& DebugCaps = FMjRenderDebugCaps());

	/** Source endpoint (e.g. "tcp://host:port"), topic to subscribe, and the
	 *  delivery callback. Call before TransportInit. */
	virtual void Configure(const FString& Endpoint, const FString& Topic,
		FOnClientMessage Callback)
		PURE_VIRTUAL(UURLabClientSubscribeTransport::Configure, );

	/** Ask the owner to serialize the heavier debug tier (contacts/overlay) onto
	 *  each streamed frame (source-of-truth §8.2). Backends that carry a per-
	 *  subscription request (gRPC/dm_env) fold these caps into their subscribe
	 *  payload; the base default is a no-op so a backend that cannot negotiate the
	 *  tier just streams the lean transform frames. Call before TransportInit
	 *  (Create does this between Configure and TransportInit).
	 *
	 *  ZMQ status: the ZMQ bus is topic-per-tier and advertises the "render"
	 *  (lean transform) topic only, so requesting the debug tier over ZMQ needs a
	 *  new topic/control channel -- deferred as a follow-up. The ZMQ backend
	 *  inherits this no-op, so a ZMQ mirror silently draws only the [XFORM]/[MODEL]
	 *  overlays that need no stream; gRPC is the debug-overlay demo path. */
	virtual void SetRenderDebugCaps(const FMjRenderDebugCaps& /*DebugCaps*/) {}

	/** Connect + start the worker. False if the runtime is unavailable. */
	virtual bool TransportInit()
		PURE_VIRTUAL(UURLabClientSubscribeTransport::TransportInit, return false;);

	/** Stop + join the worker and release the connection. Idempotent. */
	virtual void TransportShutdown()
		PURE_VIRTUAL(UURLabClientSubscribeTransport::TransportShutdown, );

	/** Stable identifier, e.g. "zmq-sub". */
	virtual FString GetTransportName() const
		PURE_VIRTUAL(UURLabClientSubscribeTransport::GetTransportName, return FString(););
};
