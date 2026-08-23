// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "Transport/ZmqPublishTransport.h"
#if PLATFORM_WINDOWS
#include "Windows/AllowWindowsPlatformTypes.h"
#endif
#include "zmq.h"
#if PLATFORM_WINDOWS
#include "Windows/HideWindowsPlatformTypes.h"
#endif
#include "MuJoCo/Core/AMjManager.h"
#include "Utils/URLabLogging.h"

namespace
{
// FString.Len() returns TCHAR count; the zmq frame needs UTF-8 byte
// count. Non-ASCII topics (any multibyte character) diverge.
inline int SendTopic(void* Socket, const FString& Topic, int Flags)
{
	const FTCHARToUTF8 Utf8(*Topic);
	return zmq_send(Socket, Utf8.Get(), Utf8.Length(), Flags);
}
} // namespace

void UURLabZmqPublishTransport::SetOwningManager(AAMjManager* InMgr)
{
	OwningManager = InMgr;
}

bool UURLabZmqPublishTransport::TransportInit()
{
	InitZmqSocket();
	if (!bIsInitialized)
		return false;

	if (AAMjManager* Mgr = OwningManager.Get())
	{
		Mgr->RegisterSnapshotPublisher(this, this);
	}
	return true;
}

void UURLabZmqPublishTransport::TransportShutdown()
{
	if (AAMjManager* Mgr = OwningManager.Get())
	{
		Mgr->UnregisterSnapshotPublisher(this);
	}
	ShutdownZmqSocket();
}

void UURLabZmqPublishTransport::InitZmqSocket()
{
	if (bIsInitialized)
		return;

	ZmqContext = zmq_ctx_new();
	ZmqPublisher = zmq_socket(ZmqContext, ZMQ_PUB);

	int rc = zmq_bind(ZmqPublisher, TCHAR_TO_UTF8(*ZmqEndpoint));
	if (rc == 0)
	{
		UE_LOG(LogURLabNet, Log, TEXT("UURLabZmqPublishTransport: bound ZMQ PUB at %s"), *ZmqEndpoint);
		bIsInitialized = true;
	}
	else
	{
		UE_LOG(LogURLabNet, Error, TEXT("UURLabZmqPublishTransport: failed to bind ZMQ at %s"), *ZmqEndpoint);
		if (GEngine)
		{
			GEngine->AddOnScreenDebugMessage(-1, 10.f, FColor::Red,
				FString::Printf(TEXT("URLab: ZMQ bind failed on %s — check for port conflicts"), *ZmqEndpoint));
		}
		// Bind failed: free all resources here. bIsInitialized stays false,
		// so ShutdownZmqSocket early-outs and would otherwise leak the
		// socket and context. Close both and null the handles.
		zmq_close(ZmqPublisher);
		zmq_ctx_term(ZmqContext);
		ZmqPublisher = nullptr;
		ZmqContext = nullptr;
	}
}

void UURLabZmqPublishTransport::ShutdownZmqSocket()
{
	if (!bIsInitialized)
		return;
	zmq_close(ZmqPublisher);
	zmq_ctx_term(ZmqContext);
	ZmqPublisher = nullptr;
	ZmqContext = nullptr;
	bIsInitialized = false;
}

void UURLabZmqPublishTransport::Publish(const FString& Topic, const TArray<uint8>& Payload)
{
	if (!ZmqPublisher || Payload.Num() == 0)
		return;
	SendTopic(ZmqPublisher, Topic, ZMQ_SNDMORE);
	zmq_send(ZmqPublisher, Payload.GetData(), Payload.Num(), 0);
}
