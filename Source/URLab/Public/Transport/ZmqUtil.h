// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"

/**
 * The `[topic][payload]` framing every URLab ZMQ stream uses, in one place.
 *
 * The server-side control ingress and the client-side render sink each carried
 * their own copy of this two-frame receive, and they had drifted: one checked
 * ZMQ_RCVMORE before reading the payload and the other assumed it, so a
 * single-frame message left one of them mid-message on the socket.
 */
namespace URLabZmqUtil
{
/** Send `[topic][payload]`. The topic frame carries UTF-8 BYTES: FString::Len
 *  is a TCHAR count, and using it truncates any non-ASCII topic. */
URLAB_API bool SendTopicPayload(void* Socket, const FString& Topic,
	const uint8* Payload, int32 PayloadSize);

/** Receive one `[topic][payload]` pair. `Flags` is passed to the topic-frame
 *  recv (0 blocks up to ZMQ_RCVTIMEO, ZMQ_DONTWAIT polls); the payload frame is
 *  always read blocking, because by then the message is already half consumed.
 *
 *  Returns false when no message was available, when the peer sent a topic with
 *  no payload frame, or on error -- in every case having closed whatever frames
 *  it opened. `OutTopic` is optional: a subscriber with a single subscription
 *  already knows the topic and passing nullptr skips the UTF-8 decode. */
URLAB_API bool RecvTopicPayload(void* Socket, int32 Flags, FString* OutTopic,
	TArray<uint8>& OutPayload);
} // namespace URLabZmqUtil
