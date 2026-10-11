// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "Transport/ZmqUtil.h"

#if PLATFORM_WINDOWS
#include "Windows/AllowWindowsPlatformTypes.h"
#endif
#include "zmq.h"
#if PLATFORM_WINDOWS
#include "Windows/HideWindowsPlatformTypes.h"
#endif

namespace URLabZmqUtil
{
bool SendTopicPayload(void* Socket, const FString& Topic, const uint8* Payload,
	int32 PayloadSize)
{
	if (Socket == nullptr)
	{
		return false;
	}
	const FTCHARToUTF8 TopicUtf8(*Topic);
	if (zmq_send(Socket, TopicUtf8.Get(), TopicUtf8.Length(), ZMQ_SNDMORE) < 0)
	{
		return false;
	}
	return zmq_send(Socket, Payload, PayloadSize, 0) >= 0;
}

bool RecvTopicPayload(void* Socket, int32 Flags, FString* OutTopic,
	TArray<uint8>& OutPayload)
{
	if (Socket == nullptr)
	{
		return false;
	}

	zmq_msg_t TopicMsg;
	zmq_msg_init(&TopicMsg);
	if (zmq_msg_recv(&TopicMsg, Socket, Flags) < 0)
	{
		zmq_msg_close(&TopicMsg);
		return false;
	}

	if (OutTopic != nullptr)
	{
		const int32 TopicSize = static_cast<int32>(zmq_msg_size(&TopicMsg));
		TArray<char> TopicBuf;
		TopicBuf.SetNumUninitialized(TopicSize + 1);
		if (TopicSize > 0)
		{
			FMemory::Memcpy(TopicBuf.GetData(), zmq_msg_data(&TopicMsg), TopicSize);
		}
		TopicBuf[TopicSize] = '\0';
		*OutTopic = UTF8_TO_TCHAR(TopicBuf.GetData());
		OutTopic->TrimEndInline();
	}

	// A topic with no payload frame is a malformed message, not an empty one:
	// reading blindly would consume the NEXT message's topic as this one's body.
	int More = 0;
	size_t MoreSize = sizeof(More);
	zmq_getsockopt(Socket, ZMQ_RCVMORE, &More, &MoreSize);
	zmq_msg_close(&TopicMsg);
	if (!More)
	{
		return false;
	}

	zmq_msg_t PayloadMsg;
	zmq_msg_init(&PayloadMsg);
	if (zmq_msg_recv(&PayloadMsg, Socket, 0) < 0)
	{
		zmq_msg_close(&PayloadMsg);
		return false;
	}
	const int32 Size = static_cast<int32>(zmq_msg_size(&PayloadMsg));
	OutPayload.SetNumUninitialized(Size, EAllowShrinking::No);
	if (Size > 0)
	{
		FMemory::Memcpy(OutPayload.GetData(), zmq_msg_data(&PayloadMsg), Size);
	}
	zmq_msg_close(&PayloadMsg);
	return true;
}
} // namespace URLabZmqUtil
