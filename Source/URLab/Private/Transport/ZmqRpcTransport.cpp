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
#include "Bridge/BridgeServerConfig.h"
#include "Transport/ZmqContext.h"
#include "Wire/RequestEnvelope.h"
#include "Wire/UrlabWire.gen.h"
#include "Engine/Engine.h"
#include "Utils/URLabLogging.h"
#include "zmq.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#endif

namespace
{
constexpr int64 MaxCommandEnvelopeBytes = 4 * 1024 * 1024;

bool IsCommandEnvelopeSizeAllowed(size_t Size)
{
	return Size <= static_cast<size_t>(MaxCommandEnvelopeBytes)
		&& Size <= static_cast<size_t>(MAX_int32);
}
}

UURLabZmqRpcTransport::UURLabZmqRpcTransport() = default;

FString UURLabZmqRpcTransport::ResolveEndpoint() const
{
	if (!StepEndpoint.IsEmpty())
	{
		return StepEndpoint;
	}
	if (UURLabBridgeServer* Bridge = GetOwningBridge())
	{
		const FURLabBridgeServerConfig& Cfg = Bridge->GetInstanceConfig();
		return FString::Printf(TEXT("tcp://%s:%d"), *Cfg.BindAddress, Cfg.StepPort);
	}
	// No bridge to read from. Use the config struct's own defaults rather than
	// repeating a port literal here.
	const FURLabBridgeServerConfig Defaults;
	return FString::Printf(TEXT("tcp://%s:%d"), *Defaults.BindAddress, Defaults.StepPort);
}

bool UURLabZmqRpcTransport::CreateAndBindRep()
{
	ZmqRep = zmq_socket(ZmqContext, ZMQ_REP);
	if (!ZmqRep)
		return false;

	int Timeout = PollTimeoutMs;
	zmq_setsockopt(ZmqRep, ZMQ_RCVTIMEO, &Timeout, sizeof(Timeout));
	zmq_setsockopt(ZmqRep, ZMQ_SNDTIMEO, &Timeout, sizeof(Timeout));
	// LINGER 0: on close, drop any unsent reply immediately instead of the
	// libzmq default (infinite), so zmq_ctx_term can never block editor exit
	// on a client that stopped reading.
	int Linger = 0;
	zmq_setsockopt(ZmqRep, ZMQ_LINGER, &Linger, sizeof(Linger));
	// Apply the protocol envelope limit at libzmq's ingress boundary. This keeps
	// an oversized peer frame from being allocated before the typed decoder can
	// enforce the same 4 MiB bound.
	const int64 MaxMessageSize = MaxCommandEnvelopeBytes;
	if (zmq_setsockopt(ZmqRep, ZMQ_MAXMSGSIZE, &MaxMessageSize, sizeof(MaxMessageSize)) != 0)
	{
		zmq_close(ZmqRep);
		ZmqRep = nullptr;
		return false;
	}

	if (zmq_bind(ZmqRep, TCHAR_TO_UTF8(*BoundEndpoint)) != 0)
	{
		zmq_close(ZmqRep);
		ZmqRep = nullptr;
		return false;
	}
	return true;
}

bool UURLabZmqRpcTransport::TransportInit()
{
	if (bIsInitialized)
		return true;

	BoundEndpoint = ResolveEndpoint();
	ZmqContext = FURLabZmqContext::Acquire();
	if (!ZmqContext || !CreateAndBindRep())
	{
		UE_LOG(LogURLabNet, Error, TEXT("UURLabZmqRpcTransport: failed to bind REP at %s"), *BoundEndpoint);
		if (GEngine)
		{
			GEngine->AddOnScreenDebugMessage(-1, 10.f, FColor::Red,
				FString::Printf(TEXT("URLab: ZMQ bind failed on %s, port conflict?"), *BoundEndpoint));
		}
		FURLabZmqContext::Release(ZmqContext);
		return false;
	}

	bIsInitialized = true;
	bStop = false;

	Worker.Start(TEXT("URLab.Zmq.Rep"),
		[this]() { RunPollLoop(); },
		[this]() { bStop = true; });

	UE_LOG(LogURLabNet, Log, TEXT("UURLabZmqRpcTransport initialised at %s"), *BoundEndpoint);
	return true;
}

void UURLabZmqRpcTransport::TransportShutdown()
{
	if (!bIsInitialized)
		return;

	bStop = true;
	Worker.Stop();

	// Socket first, then the context reference: zmq_ctx_term blocks until every
	// socket on the shared context is closed.
	if (ZmqRep)
	{
		zmq_close(ZmqRep);
		ZmqRep = nullptr;
	}
	FURLabZmqContext::Release(ZmqContext);
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

		const size_t WireSize = zmq_msg_size(&Msg);
		TArray<uint8> OutBytes;
		if (!IsCommandEnvelopeSizeAllowed(WireSize))
		{
			// ZMQ_MAXMSGSIZE should reject this before delivery. Keep the
			// post-receive check so no platform/backend can reintroduce an unsafe
			// size_t-to-int32 narrowing.
			FURLabRequestEnvelopeCodec::EncodeProtocolError(
				{TEXT("bad_request"), TEXT("Request exceeds the 4 MiB command-envelope limit")},
				OutBytes);
		}
		else
		{
			const int32 Size = static_cast<int32>(WireSize);
			TArray<uint8> InBytes;
			InBytes.SetNumUninitialized(Size);
			if (Size > 0)
			{
				FMemory::Memcpy(InBytes.GetData(), zmq_msg_data(&Msg), Size);
			}
			// Wire detect / parse / dispatch / encode all live on the base.
			ProcessRequestBytes(InBytes, OutBytes);
		}
		zmq_msg_close(&Msg);

		if (OutBytes.Num() > URLabWire::Limits::MaxInlineReplyBytes)
		{
			FURLabRequestEnvelopeCodec::EncodeProtocolError(
				{TEXT("reply_too_large"), TEXT("Command reply exceeds the inline terminal-reply limit")},
				OutBytes);
		}

		// REP is a strict recv-then-send state machine: having received, we
		// MUST send before the next recv. If the send fails (EAGAIN under the
		// send timeout, or a peer that vanished) the socket stays stuck in the
		// must-send state and every later recv returns EFSM, spinning a core.
		// Retry a few times while progress is still possible, then rebuild the
		// socket to reset the state machine instead of busy-spinning.
		constexpr int MaxSendAttempts = 3;
		bool bSent = false;
		for (int Attempt = 0; Attempt < MaxSendAttempts && !bStop.load(std::memory_order_acquire); ++Attempt)
		{
			if (zmq_send(ZmqRep, OutBytes.GetData(), OutBytes.Num(), 0) >= 0)
			{
				bSent = true;
				break;
			}
			if (zmq_errno() != EAGAIN)
				break; // hard error; rebuild rather than keep retrying
		}
		if (!bSent && !bStop.load(std::memory_order_acquire))
		{
			UE_LOG(LogURLabNet, Warning,
				TEXT("UURLabZmqRpcTransport: reply send failed (errno=%d); resetting REP socket at %s"),
				zmq_errno(), *BoundEndpoint);
			zmq_close(ZmqRep);
			ZmqRep = nullptr;
			if (!CreateAndBindRep())
			{
				UE_LOG(LogURLabNet, Error,
					TEXT("UURLabZmqRpcTransport: failed to rebind REP at %s after send error; stopping poll loop"),
					*BoundEndpoint);
				break;
			}
		}
	}
}

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabZmqCommandEnvelopeBoundTest,
	"URLab.Protocol.Transport.Zmq.CommandEnvelopeBound",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabZmqCommandEnvelopeBoundTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	TestTrue(TEXT("4 MiB envelope is accepted"),
		IsCommandEnvelopeSizeAllowed(static_cast<size_t>(MaxCommandEnvelopeBytes)));
	TestFalse(TEXT("4 MiB plus one byte is rejected"),
		IsCommandEnvelopeSizeAllowed(static_cast<size_t>(MaxCommandEnvelopeBytes) + 1));
	return true;
}
#endif
