// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "Transport/TransportBase.h"
#include "PublishTransport.generated.h"

/**
 * @class UURLabPublishTransport
 * @brief Server -> client streaming: one `Publish(topic, bytes)` fan-out.
 *
 * The small / frequent / single-socket publish shape shared by the state
 * snapshot stream, the render bus and the ROS publisher.
 *
 * Per-camera image streaming is deliberately a separate role
 * (`UURLabCameraPublishTransport`): frames are ~MB, each camera has its own
 * lifetime, and delivery runs off a GPU-readback worker rather than the step.
 *
 * Threading: each concrete transport owns its own concurrency (per-socket lock
 * for ZMQ, lock-free double buffer for SHM). `Publish` is called from producer
 * threads, so a backend serialises internally.
 */
UCLASS(Abstract)
class URLAB_API UURLabPublishTransport : public UURLabTransportBase
{
	GENERATED_BODY()

public:
	/** Publish `Payload` on `Topic`. Backends interpret the topic in their own
	 *  world (ZMQ topic frame, SHM region key, ROS topic name). */
	virtual void Publish(const FString& Topic,
		const TArray<uint8>& Payload) PURE_VIRTUAL(UURLabPublishTransport::Publish, );

	/** Optional advance-advertise hook. ROS publishers must be created up front
	 *  via `rclcpp::create_publisher`; ZMQ / SHM default to no-op. */
	virtual void RegisterTopic(const FString& /*Topic*/,
		const FString& /*Schema*/) {}

	/** The tiered topics live subscribers have asked this transport for, beyond
	 *  the base topic every consumer receives (see URLabRenderBusTopic). This is
	 *  how a publish/subscribe backend answers "who negotiated what" without a
	 *  request channel: the subscription is the request, so a backend that sees
	 *  its subscriptions can report them and the producer builds one frame per
	 *  distinct request. Default empty -- a backend with no subscription
	 *  back-channel negotiates nothing and its consumers get the base topic. */
	virtual void CollectTieredTopics(TSet<FString>& /*OutTopics*/) const {}

};
