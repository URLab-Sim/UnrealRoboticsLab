// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "PublishTransport.generated.h"

class UURLabBridgeServer;

/**
 * @class UURLabPublishTransport
 * @brief Abstract base for server -> client streaming transports.
 *
 * Folds in the small / frequent / single-socket publish shape today
 * spread across `UURLabShmPublishTransport`, `UURLabZmqPublishTransport`,
 * and other state-stream producers. Future ROS analogue:
 * `rclcpp::Publisher`.
 *
 * Per-camera image streaming is INTENTIONALLY out of scope. Camera
 * workers (`FCameraZmqWorker`) keep their dedicated per-camera thread
 * + socket because GPU readback + large compressed payloads + per-camera
 * lifetime don't fit a shared `Publish(topic, bytes)` shape.
 *
 * Threading: each concrete transport owns its own concurrency (per-
 * socket lock for ZMQ; lock-free SPSC ring for SHM single-producer
 * snapshots). The base class declares `Publish` abstract; subclasses
 * pick a synchronisation primitive that matches their backend.
 */
UCLASS(Abstract)
class URLAB_API UURLabPublishTransport : public UObject
{
	GENERATED_BODY()

public:
	void SetOwningBridge(UURLabBridgeServer* Bridge);
	UURLabBridgeServer* GetOwningBridge() const { return OwningBridge.Get(); }

	/** Publish `Payload` on `Topic`. Backends interpret the topic in
	 *  their own world (ZMQ topic frame, SHM region key, ROS topic
	 *  name). Called from N producer threads — each concrete transport
	 *  serialises internally. */
	virtual void Publish(const FString& Topic,
		const TArray<uint8>& Payload) PURE_VIRTUAL(UURLabPublishTransport::Publish, );

	/** Optional advance-advertise hook. ROS publishers need to be
	 *  created up front via `rclcpp::create_publisher`; ZMQ / SHM
	 *  default to no-op. */
	virtual void RegisterTopic(const FString& /*Topic*/,
		const FString& /*Schema*/) {}

	/** Bind / open backend handles. Returns false on failure. */
	virtual bool TransportInit() PURE_VIRTUAL(UURLabPublishTransport::TransportInit, return false;);

	/** Stop producers, release handles. Idempotent. */
	virtual void TransportShutdown() PURE_VIRTUAL(UURLabPublishTransport::TransportShutdown, );

	virtual FString GetTransportName() const PURE_VIRTUAL(UURLabPublishTransport::GetTransportName, return FString(););

	// --- Per-step physics hooks (Async thread) ----------------------------
	// Default-empty so transports that don't tie to the physics step (e.g.
	// sensor-shaped publishers that publish on demand) don't need to opt in.
	// Manager iterates `ManagerOwnedPublishTransports` and calls these on
	// the engine's pre/post-step callbacks.

	/** Runs before mj_step() on the physics thread. */
	virtual void PreStep(struct mjModel_* /*m*/, struct mjData_* /*d*/) {}

	/** Runs after mj_step() on the physics thread. */
	virtual void PostStep(struct mjModel_* /*m*/, struct mjData_* /*d*/) {}

protected:
	TWeakObjectPtr<UURLabBridgeServer> OwningBridge;
};
