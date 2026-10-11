// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "Transport/ZmqContext.h"

#include "Utils/URLabLogging.h"

#if PLATFORM_WINDOWS
#include "Windows/AllowWindowsPlatformTypes.h"
#endif
#include "zmq.h"
#if PLATFORM_WINDOWS
#include "Windows/HideWindowsPlatformTypes.h"
#endif

namespace
{
FCriticalSection GZmqContextMutex;
void* GZmqContext = nullptr;
int32 GZmqContextRefCount = 0;
} // namespace

void* FURLabZmqContext::Acquire()
{
	FScopeLock Lock(&GZmqContextMutex);
	if (GZmqContext == nullptr)
	{
		GZmqContext = zmq_ctx_new();
		if (GZmqContext == nullptr)
		{
			UE_LOG(LogURLabNet, Error, TEXT("FURLabZmqContext: zmq_ctx_new failed"));
			return nullptr;
		}
		UE_LOG(LogURLabNet, Verbose, TEXT("FURLabZmqContext: created the process ZMQ context"));
	}
	++GZmqContextRefCount;
	return GZmqContext;
}

void FURLabZmqContext::Release(void*& Ctx)
{
	if (Ctx == nullptr)
	{
		return;
	}
	Ctx = nullptr;

	FScopeLock Lock(&GZmqContextMutex);
	if (GZmqContextRefCount <= 0)
	{
		// A double release would otherwise terminate a context other holders
		// still have sockets on, which shows up much later as an unexplained
		// dead socket. Say so here instead.
		UE_LOG(LogURLabNet, Warning,
			TEXT("FURLabZmqContext: released more references than were acquired"));
		return;
	}

	if (--GZmqContextRefCount == 0 && GZmqContext != nullptr)
	{
		// Blocks until every socket on the context is closed. Holders close
		// theirs before releasing, and every socket sets LINGER 0, so this
		// cannot stall editor exit on a peer that stopped reading.
		zmq_ctx_term(GZmqContext);
		GZmqContext = nullptr;
		UE_LOG(LogURLabNet, Verbose, TEXT("FURLabZmqContext: terminated the process ZMQ context"));
	}
}

int32 FURLabZmqContext::GetRefCount()
{
	FScopeLock Lock(&GZmqContextMutex);
	return GZmqContextRefCount;
}
