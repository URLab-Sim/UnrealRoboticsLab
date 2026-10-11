// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"

/**
 * The topic vocabulary of the owner -> subscriber render bus, and the one place the
 * debug tier is spelled into a topic name.
 *
 * A publish/subscribe bus has no request channel, so the SUBSCRIPTION ITSELF is
 * the request. libzmq hands every subscribe and unsubscribe to an XPUB
 * publisher, so a subscriber asks for the heavier debug tier by subscribing to a
 * topic that spells out the capabilities it wants, and releases it by
 * unsubscribing -- which libzmq also synthesises when the peer disappears. That
 * gives ZMQ the same per-subscription, explicitly-released negotiation gRPC has,
 * without a second socket, a new op, or a process-wide union that outlives the
 * subscriber that asked.
 *
 * Tiered topics deliberately do NOT begin with the lean topic. ZMQ subscriptions
 * are PREFIX matches, so a topic under "render..." would be delivered to every
 * lean subscriber subscribed to "render" -- exactly the leak that scoping the caps
 * per subscription removed. They live under their own root instead.
 *
 * The trailing separator is load-bearing for the same reason: without it "...m1"
 * is a prefix of "...m10", and a subscriber capped at one contact would also receive
 * the frames built for one capped at ten.
 */
namespace URLabRenderBusTopic
{
/** The always-published transform tier. It carries no debug fields, so a lean
 *  subscriber pays nothing for another subscriber's negotiation. */
URLAB_API extern const TCHAR* Lean;

/** The topic a subscriber subscribes to in order to request these capabilities.
 *  Returns `Lean` when nothing is asked for, so a lean subscriber needs no special
 *  case at the call site. */
URLAB_API FString ForCaps(bool bContacts, bool bOverlay, int32 MaxContacts);

/** True when `Topic` names the debug tier rather than the lean transform tier. */
URLAB_API bool IsTiered(const FString& Topic);

/** Read back the capabilities a tiered topic spells out. False (leaving the
 *  outputs untouched) when `Topic` is the lean tier or is malformed, so a
 *  publisher never invents a tier from a topic it does not understand. */
URLAB_API bool ParseCaps(const FString& Topic, bool& bOutContacts, bool& bOutOverlay,
	int32& OutMaxContacts);
} // namespace URLabRenderBusTopic
