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

#include "Transport/ZmqRpcTransport.h"
#include "Bridge/BridgeServer.h"
#include "HAL/RunnableThread.h"
#include "HAL/Runnable.h"
#include "Engine/Engine.h"
#include "Utils/URLabLogging.h"
#include "zmq.h"

UURLabZmqRpcTransport::UURLabZmqRpcTransport() = default;

bool UURLabZmqRpcTransport::TransportInit()
{
	if (bIsInitialized)
		return true;

	ZmqContext = zmq_ctx_new();
	ZmqRep = zmq_socket(ZmqContext, ZMQ_REP);
	int Timeout = PollTimeoutMs;
	zmq_setsockopt(ZmqRep, ZMQ_RCVTIMEO, &Timeout, sizeof(Timeout));
	zmq_setsockopt(ZmqRep, ZMQ_SNDTIMEO, &Timeout, sizeof(Timeout));

	int rc = zmq_bind(ZmqRep, TCHAR_TO_UTF8(*StepEndpoint));
	if (rc != 0)
	{
		UE_LOG(LogURLabNet, Error, TEXT("UURLabZmqRpcTransport: failed to bind REP at %s"), *StepEndpoint);
		if (GEngine)
		{
			GEngine->AddOnScreenDebugMessage(-1, 10.f, FColor::Red,
				FString::Printf(TEXT("URLab: ZMQ bind failed on %s — port conflict?"), *StepEndpoint));
		}
		zmq_close(ZmqRep);
		zmq_ctx_term(ZmqContext);
		ZmqRep = nullptr;
		ZmqContext = nullptr;
		return false;
	}

	bIsInitialized = true;
	bStop = false;

	class FStepServerRunnable : public FRunnable
	{
	public:
		UURLabZmqRpcTransport* Server;
		explicit FStepServerRunnable(UURLabZmqRpcTransport* S)
			: Server(S) {}
		virtual uint32 Run() override
		{
			Server->RunPollLoop();
			return 0;
		}
		virtual void Stop() override { Server->bStop = true; }
	};
	FStepServerRunnable* Runner = new FStepServerRunnable(this);
	WorkerThread = FRunnableThread::Create(Runner, TEXT("URLabStepServer"));

	UE_LOG(LogURLabNet, Log, TEXT("UURLabZmqRpcTransport initialised at %s"), *StepEndpoint);
	return true;
}

void UURLabZmqRpcTransport::TransportShutdown()
{
	if (!bIsInitialized)
		return;

	bStop = true;
	if (WorkerThread)
	{
		WorkerThread->WaitForCompletion();
		delete WorkerThread;
		WorkerThread = nullptr;
	}

	if (ZmqRep)
	{
		zmq_close(ZmqRep);
		ZmqRep = nullptr;
	}
	if (ZmqContext)
	{
		zmq_ctx_term(ZmqContext);
		ZmqContext = nullptr;
	}
	bIsInitialized = false;
}

void UURLabZmqRpcTransport::RunPollLoop()
{
	while (!bStop.load(std::memory_order_acquire))
	{
		zmq_msg_t Msg;
		zmq_msg_init(&Msg);
		int rc = zmq_msg_recv(&Msg, ZmqRep, 0);
		if (rc < 0)
		{
			zmq_msg_close(&Msg);
			// Timeout or shutdown — loop and check bStop.
			continue;
		}

		const int Size = zmq_msg_size(&Msg);
		TArray<uint8> InBytes;
		InBytes.SetNumUninitialized(Size);
		if (Size > 0)
			FMemory::Memcpy(InBytes.GetData(), zmq_msg_data(&Msg), Size);
		zmq_msg_close(&Msg);

		// Wire detect / parse / dispatch / encode all live on the base.
		TArray<uint8> OutBytes;
		ProcessRequestBytes(InBytes, OutBytes);
		zmq_send(ZmqRep, OutBytes.GetData(), OutBytes.Num(), 0);
	}
}
