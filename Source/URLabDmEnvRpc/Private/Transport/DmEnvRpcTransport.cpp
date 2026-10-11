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

#ifndef PROTOBUF_ENABLE_DEBUG_LOGGING_MAY_LEAK_PII
#define PROTOBUF_ENABLE_DEBUG_LOGGING_MAY_LEAK_PII 0
#endif

#include "CoreMinimal.h"

// UE's intrusive assertion/formatting macros collide with member names inside
// grpc/abseil (e.g. absl btree's `verify()`), so undef the whole family across the
// third-party block and restore it after.
#pragma push_macro("check")
#undef check
#pragma push_macro("checkf")
#undef checkf
#pragma push_macro("verify")
#undef verify
#pragma push_macro("verifyf")
#undef verifyf
#pragma push_macro("ensure")
#undef ensure
#pragma push_macro("TEXT")
#undef TEXT
#pragma push_macro("PI")
#undef PI

// grpc/absl on Win64 need the real <windows.h> (MemoryBarrier, Interlocked*, ...),
// which UE does not include globally. UE's WindowsHWrapper.h is `#pragma once` and
// CoreMinimal already consumed it, so re-including it is a no-op -- include <windows.h>
// directly (its own _WINDOWS_ guard is independent of UE's wrapper). WIN32_LEAN_AND_MEAN
// + NOMINMAX keep min/max/winsock1 out of the gRPC headers, and the Allow/Hide pairs
// push then pop the platform-type + atomics macros so the UE headers below stay clean.
#if PLATFORM_WINDOWS
#include "Windows/AllowWindowsPlatformTypes.h"
#include "Windows/AllowWindowsPlatformAtomics.h"
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

THIRD_PARTY_INCLUDES_START
#include <grpcpp/grpcpp.h>
#include "dm_env_rpc.grpc.pb.h"
#include "urlab_dm_env_rpc.pb.h"
THIRD_PARTY_INCLUDES_END

#if PLATFORM_WINDOWS
#include "Windows/HideWindowsPlatformAtomics.h"
#include "Windows/HideWindowsPlatformTypes.h"
#endif

#pragma pop_macro("PI")
#pragma pop_macro("TEXT")
#pragma pop_macro("ensure")
#pragma pop_macro("verifyf")
#pragma pop_macro("verify")
#pragma pop_macro("checkf")
#pragma pop_macro("check")

#include "Transport/DmEnvRpcTransport.h"
#include "URLabDmEnvRpc.h"
#include "Bridge/BridgeServer.h"
#include "Bridge/BridgeServerConfig.h"
#include "Dom/JsonObject.h"
#include "HAL/PlatformTime.h"
#include "Wire/RequestEnvelope.h"
#include "Wire/UrlabWire.gen.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#endif

#include <atomic>
#include <chrono>

namespace
{
// The MessagePack decoder accepts at most 4 MiB. gRPC applies its limit to the
// complete EnvironmentRequest/Response protobuf, so allow a fixed amount for the
// Any type URL, UrlabPacket fields, op and sequence around the bounded payload.
constexpr int32 MaxCommandEnvelopeBytes = 4 * 1024 * 1024;
constexpr int32 GrpcOuterAllowanceBytes = 1024;
constexpr int32 MaxGrpcRequestBytes = MaxCommandEnvelopeBytes + GrpcOuterAllowanceBytes;
constexpr int32 MaxGrpcReplyBytes =
	static_cast<int32>(URLabWire::Limits::MaxInlineReplyBytes) + GrpcOuterAllowanceBytes;
constexpr std::chrono::seconds GrpcShutdownGrace(2);

struct FPacketValidation
{
	grpc::StatusCode Code = grpc::StatusCode::OK;
	const char* Message = nullptr;

	explicit operator bool() const { return Message == nullptr; }
};

FPacketValidation ValidateCommandPacket(int64 SequenceId, uint64 PayloadBytes)
{
	if (SequenceId == 0)
	{
		return {grpc::StatusCode::INVALID_ARGUMENT,
			"UrlabPacket.sequence_id must be nonzero"};
	}
	if (PayloadBytes > static_cast<uint64>(MaxCommandEnvelopeBytes))
	{
		return {grpc::StatusCode::RESOURCE_EXHAUSTED,
			"UrlabPacket.payload exceeds the 4 MiB command-envelope limit"};
	}
	return {};
}
} // namespace

class FURLabDmEnvRpcServiceImpl final : public dm_env_rpc::v1::Environment::Service
{
public:
	explicit FURLabDmEnvRpcServiceImpl(UURLabDmEnvRpcTransport* InTransport)
		: Transport(InTransport)
	{
	}

	virtual grpc::Status Process(
		grpc::ServerContext*,
		grpc::ServerReaderWriter<dm_env_rpc::v1::EnvironmentResponse, dm_env_rpc::v1::EnvironmentRequest>* Stream) override
	{
		dm_env_rpc::v1::EnvironmentRequest Request;
		while (!Transport->ShouldStop() && Stream->Read(&Request))
		{
			dm_env_rpc::v1::EnvironmentResponse Response;

			// 1. URLab custom extension packet (UrlabPacket)
			if (Request.has_extension())
			{
				urlab::dm_env_rpc::v1::UrlabPacket InPacket;
				if (Request.extension().UnpackTo(&InPacket))
				{
					const FPacketValidation PacketValidation = ValidateCommandPacket(
						InPacket.sequence_id(), static_cast<uint64>(InPacket.payload().size()));
					if (!PacketValidation)
					{
						Response.mutable_extension()->PackFrom(MakeStatusPacket(
							InPacket, PacketValidation.Code, PacketValidation.Message));
						if (!Stream->Write(Response))
						{
							break;
						}
						continue;
					}

					// UrlabPacket.op is required as the outer dispatch key. The shared
					// typed dispatcher compares it exactly with the envelope op; the
					// payload is never rewritten or passed through JSON here.
					FString PacketOp;
					const FURLabWireStatus OpStatus = FURLabRequestEnvelopeCodec::DecodeIdentifierUtf8(
						TConstArrayView<uint8>(reinterpret_cast<const uint8*>(InPacket.op().data()),
							static_cast<int32>(InPacket.op().size())), PacketOp);
					if (!OpStatus)
					{
						Response.mutable_extension()->PackFrom(
							MakeStatusPacket(InPacket, grpc::StatusCode::INVALID_ARGUMENT,
								"UrlabPacket.op is required; it is the dispatch key"));
						if (!Stream->Write(Response))
						{
							break;
						}
						continue;
					}

					const int32 PayloadSize = static_cast<int32>(InPacket.payload().size());
					TArray<uint8> InBytes(
						reinterpret_cast<const uint8*>(InPacket.payload().data()),
						PayloadSize);
					TArray<uint8> OutReplyBytes;

					if (Transport->GetOwningBridge())
					{
						// DispatchBytes synchronizes typed domain access and schedules any
						// deferred work itself. Dispatching inline keeps request, op, reply,
						// service and transport lifetimes inside this Process call.
						Transport->ProcessRequestBytes(InBytes, OutReplyBytes, &PacketOp);
					}

					if (OutReplyBytes.Num() > URLabWire::Limits::MaxInlineReplyBytes)
					{
						Response.mutable_extension()->PackFrom(MakeStatusPacket(InPacket,
							grpc::StatusCode::RESOURCE_EXHAUSTED,
							"Command reply exceeds the inline terminal-reply limit"));
					}
					else
					{
						urlab::dm_env_rpc::v1::UrlabPacket OutPacket;
						OutPacket.set_op(InPacket.op());
						OutPacket.set_sequence_id(InPacket.sequence_id());
						OutPacket.set_payload(OutReplyBytes.GetData(), OutReplyBytes.Num());
						Response.mutable_extension()->PackFrom(OutPacket);
					}
				}
				else
				{
					// An extension that is not a UrlabPacket carries no op, and the
					// op is the dispatch key. Handing the raw Any bytes to the
					// dispatcher and hoping the body named an op is exactly the
					// second dispatch key this policy removes, so refuse instead.
					SetResponseError(Response, grpc::StatusCode::INVALID_ARGUMENT,
						"EnvironmentRequest.extension must be a urlab.dm_env_rpc.v1.UrlabPacket");
				}
			}
			// 2. Standard dm_env_rpc operations. None of them is implemented:
			// URLab uses this service as an opaque byte pipe and never populates a
			// Tensor, TensorSpec or action/observation spec. They used to answer
			// with an empty message of the right type -- an empty step reply is
			// indistinguishable from a successful one -- and create_world had no
			// branch at all, so it fell through to the unconditional write below
			// and the caller got a response with no field set. Answer with the
			// protocol's own typed error instead, so a dm_env_rpc-conformant
			// client is told the operation does not exist rather than being handed
			// a plausible empty success.
			else
			{
				SetResponseError(Response, grpc::StatusCode::UNIMPLEMENTED,
					"native dm_env_rpc operations are not implemented by URLab; "
					"send a urlab.dm_env_rpc.v1.UrlabPacket in EnvironmentRequest.extension");
			}

			if (!Stream->Write(Response))
			{
				break;
			}
		}
		return grpc::Status::OK;
	}

private:
	/** Fill EnvironmentResponse.error, the protocol's own typed error channel.
	 *  Used for failures that happen before any URLab op runs -- an unroutable
	 *  request -- as distinct from an error raised BY an op, which travels in
	 *  the msgpack reply body like it does on every other transport. */
	static void SetResponseError(dm_env_rpc::v1::EnvironmentResponse& Response,
		grpc::StatusCode Code, const char* Message)
	{
		google::rpc::Status* Error = Response.mutable_error();
		Error->set_code(static_cast<int32_t>(Code));
		Error->set_message(Message);
	}

	/** A reply packet that carries no payload, only UrlabPacket.status. This is
	 *  what that field is for: the envelope could not be routed, so there is no
	 *  dispatcher reply to put an error body into. It echoes op and sequence_id
	 *  so a client correlating on either still matches this reply. */
	static urlab::dm_env_rpc::v1::UrlabPacket MakeStatusPacket(
		const urlab::dm_env_rpc::v1::UrlabPacket& InPacket,
		grpc::StatusCode Code, const char* Message)
	{
		urlab::dm_env_rpc::v1::UrlabPacket Out;
		Out.set_op(InPacket.op());
		Out.set_sequence_id(InPacket.sequence_id());
		Out.mutable_status()->set_code(static_cast<int32_t>(Code));
		Out.mutable_status()->set_message(Message);
		return Out;
	}

	UURLabDmEnvRpcTransport* Transport = nullptr;
};

UURLabDmEnvRpcTransport::UURLabDmEnvRpcTransport() = default;

bool UURLabDmEnvRpcTransport::TransportInit()
{
	// Port and bind address come from the bridge's one resolved net config. This
	// used to re-parse the command line for itself, which meant the registry
	// could advertise one gRPC port while the server bound another.
	if (const UURLabBridgeServer* Bridge = GetOwningBridge())
	{
		const FURLabBridgeServerConfig& Cfg = Bridge->GetInstanceConfig();
		if (ListenPort <= 0)
		{
			ListenPort = Cfg.DmEnvPort;
		}
		if (BindAddress.IsEmpty())
		{
			BindAddress = Cfg.BindAddress;
		}
	}
	if (ListenPort <= 0 || BindAddress.IsEmpty())
	{
		// No bridge to read from. Use the config struct's own defaults rather
		// than repeating a port literal here.
		const FURLabBridgeServerConfig Defaults;
		if (ListenPort <= 0)
		{
			ListenPort = Defaults.DmEnvPort;
		}
		if (BindAddress.IsEmpty())
		{
			BindAddress = Defaults.BindAddress;
		}
	}

	// Start the worker and block until it reports the REAL bind result before doing
	// anything that treats this transport as live. std::promise is one-shot, so give
	// it a fresh instance for this init and grab the paired future up front.
	bShouldStop.store(false, std::memory_order_release);
	bShutdownIssued.store(false, std::memory_order_release);
	BindResultPromise = std::promise<bool>();
	std::future<bool> BindResultFuture = BindResultPromise.get_future();

	// The stop flag is set by TransportShutdown, which also shuts the gRPC
	// server down; FRunnable::Stop has nothing extra to do here.
	if (!Worker.Start(TEXT("URLab.Grpc.Server"), [this]() { RunServerLoop(); },
		[this]() { RequestServerStop(); }))
	{
		UE_LOG(LogURLabDmEnvRpc, Error, TEXT("Failed to create URLabDmEnvRpcServer worker thread."));
		return false;
	}

	// Wait for the worker to actually bind the port. RunServerLoop sets this to true
	// once grpc::Server is listening, or false if BuildAndStart() fails to bind (e.g.
	// the port is already in use). Blocking here keeps TransportInit honest -- a failed
	// bind must return false so the caller never registers this as a live transport.
	const bool bBound = BindResultFuture.get();
	if (!bBound)
	{
		// Bind failed: the worker (RunServerLoop) has already returned. Join and tear
		// it down without registering this transport as a live endpoint.
		UE_LOG(LogURLabDmEnvRpc, Error, TEXT("[URLabDmEnvRpc] dm_env_rpc server failed to bind port %d; transport not registered."), ListenPort);
		bShouldStop.store(true, std::memory_order_release);
		Worker.Stop();
		return false;
	}

	UE_LOG(LogURLabDmEnvRpc, Display, TEXT("[URLabDmEnvRpc] URLabDmEnvRpcTransport initialized on port %d."), ListenPort);
	return true;
}

void UURLabDmEnvRpcTransport::TransportShutdown()
{
	RequestServerStop();
	Worker.Stop();
	UE_LOG(LogURLabDmEnvRpc, Display, TEXT("[URLabDmEnvRpc] URLabDmEnvRpcTransport shut down."));
}

void UURLabDmEnvRpcTransport::RequestServerStop()
{
	bShouldStop.store(true, std::memory_order_release);
	std::shared_ptr<grpc::Server> LiveServer;
	{
		std::lock_guard<std::mutex> Lock(ServerMutex);
		LiveServer = Server;
	}
	if (!LiveServer)
	{
		return;
	}
	bool bExpected = false;
	if (bShutdownIssued.compare_exchange_strong(bExpected, true,
			std::memory_order_acq_rel, std::memory_order_acquire))
	{
		// This cancels blocking Stream->Read calls. RunServerLoop owns the server
		// and waits until every Process call has returned before destroying it.
		// The deadline force-cancels idle streams. It cannot impose a hard wall on
		// an in-process native dispatch that has already entered user code.
		LiveServer->Shutdown(std::chrono::system_clock::now() + GrpcShutdownGrace);
	}
}

void UURLabDmEnvRpcTransport::AppendHandshakeBlock(TSharedPtr<FJsonObject>& Reply) const
{
	if (Reply.IsValid())
	{
		Reply->SetNumberField(TEXT("dmenvrpc_port"), ListenPort);
		Reply->SetStringField(TEXT("dmenvrpc_transport"), TEXT("active"));
	}
}

void UURLabDmEnvRpcTransport::RunServerLoop()
{
	// Honour the configured interface rather than always binding every one: an
	// instance configured for loopback should not be reachable off the host.
	std::string ServerAddress =
		std::string(TCHAR_TO_UTF8(*BindAddress)) + ":" + std::to_string(ListenPort);
	FURLabDmEnvRpcServiceImpl Service(this);

	grpc::ServerBuilder Builder;
	Builder.SetMaxReceiveMessageSize(MaxGrpcRequestBytes);
	Builder.SetMaxSendMessageSize(MaxGrpcReplyBytes);

	// Keepalive permits. The Python client pings every 20 s with no active calls
	// (its stream is long-lived and mostly idle between RPCs); gRPC-core's server
	// defaults permit far fewer, so that client accrued ping strikes until the
	// server sent GOAWAY too_many_pings, which surfaced as a periodic broken
	// stream and a silent reconnect. Permit pings without calls, accept them down
	// to a 10 s interval (half the client's, so a little clock skew cannot make a
	// conforming ping look too eager), and stop counting strikes.
	Builder.AddChannelArgument(GRPC_ARG_KEEPALIVE_PERMIT_WITHOUT_CALLS, 1);
	Builder.AddChannelArgument(GRPC_ARG_HTTP2_MIN_RECV_PING_INTERVAL_WITHOUT_DATA_MS, 10000);
	Builder.AddChannelArgument(GRPC_ARG_HTTP2_MAX_PING_STRIKES, 0);

	Builder.AddListeningPort(ServerAddress, grpc::InsecureServerCredentials());
	Builder.RegisterService(&Service);

	std::shared_ptr<grpc::Server> StartedServer(Builder.BuildAndStart());
	if (!StartedServer)
	{
		UE_LOG(LogURLabDmEnvRpc, Error, TEXT("[URLabDmEnvRpc] Failed to start dm_env_rpc server on %s"), UTF8_TO_TCHAR(ServerAddress.c_str()));
		// Report the real bind failure back to TransportInit so it returns false and
			// never registers a dead command endpoint. Returning here
		// (with the promise satisfied) also unblocks init to tear the worker down.
		BindResultPromise.set_value(false);
		return;
	}

	{
		std::lock_guard<std::mutex> Lock(ServerMutex);
		Server = StartedServer;
	}
	UE_LOG(LogURLabDmEnvRpc, Display, TEXT("[URLabDmEnvRpc] dm_env_rpc gRPC server listening on %s"), UTF8_TO_TCHAR(ServerAddress.c_str()));
	// Port is bound and the server is listening -- unblock TransportInit to proceed.
	BindResultPromise.set_value(true);

	while (!ShouldStop())
	{
		FPlatformProcess::Sleep(0.01f);
	}

	RequestServerStop();
	StartedServer->Wait();
	{
		std::lock_guard<std::mutex> Lock(ServerMutex);
		if (Server == StartedServer)
		{
			Server.reset();
		}
	}
}

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabDmEnvRpcProtocolBoundsTest,
	"URLab.Protocol.Transport.DmEnvRpc.ProtocolBounds",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabDmEnvRpcProtocolBoundsTest::RunTest(const FString& Parameters)
{
	(void)Parameters;
	const FPacketValidation Valid = ValidateCommandPacket(1, MaxCommandEnvelopeBytes);
	TestTrue(TEXT("nonzero sequence and 4 MiB payload are accepted"),
		static_cast<bool>(Valid));

	const FPacketValidation ZeroSequence = ValidateCommandPacket(0, 0);
	TestFalse(TEXT("zero sequence is rejected before dispatch"),
		static_cast<bool>(ZeroSequence));
	TestTrue(TEXT("zero sequence is invalid argument"),
		ZeroSequence.Code == grpc::StatusCode::INVALID_ARGUMENT);

	const FPacketValidation Oversized =
		ValidateCommandPacket(1, static_cast<uint64>(MaxCommandEnvelopeBytes) + 1);
	TestFalse(TEXT("payload above 4 MiB is rejected before copy"),
		static_cast<bool>(Oversized));
	TestTrue(TEXT("oversized payload exhausts bounded resources"),
		Oversized.Code == grpc::StatusCode::RESOURCE_EXHAUSTED);

	TestEqual(TEXT("request outer allowance is fixed"), MaxGrpcRequestBytes,
		MaxCommandEnvelopeBytes + GrpcOuterAllowanceBytes);
	TestEqual(TEXT("reply outer allowance uses the terminal reply cap"), MaxGrpcReplyBytes,
		static_cast<int32>(URLabWire::Limits::MaxInlineReplyBytes) + GrpcOuterAllowanceBytes);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabDmEnvRpcIdleStreamShutdownTest,
	"URLab.Protocol.Transport.DmEnvRpc.IdleStreamShutdown",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabDmEnvRpcIdleStreamShutdownTest::RunTest(const FString& Parameters)
{
	(void)Parameters;

	// Reserve a kernel-selected loopback port, then release it for the transport.
	// This keeps the test independent of the configured production port.
	grpc::ServerBuilder ProbeBuilder;
	dm_env_rpc::v1::Environment::Service ProbeService;
	ProbeBuilder.RegisterService(&ProbeService);
	int32 SelectedPort = 0;
	ProbeBuilder.AddListeningPort("127.0.0.1:0", grpc::InsecureServerCredentials(),
		&SelectedPort);
	std::unique_ptr<grpc::Server> Probe = ProbeBuilder.BuildAndStart();
	if (!TestNotNull(TEXT("ephemeral-port probe server starts"), Probe.get())
		|| !TestTrue(TEXT("kernel selected a port"), SelectedPort > 0))
	{
		return false;
	}
	Probe->Shutdown();
	Probe->Wait();
	Probe.reset();

	UURLabDmEnvRpcTransport* Transport = NewObject<UURLabDmEnvRpcTransport>();
	Transport->AddToRoot();
	Transport->SetListenPort(SelectedPort);
	if (!TestTrue(TEXT("transport binds the selected port"), Transport->TransportInit()))
	{
		Transport->TransportShutdown();
		Transport->RemoveFromRoot();
		return false;
	}

	const std::string Address = "127.0.0.1:" + std::to_string(SelectedPort);
	std::unique_ptr<dm_env_rpc::v1::Environment::Stub> Stub =
		dm_env_rpc::v1::Environment::NewStub(
			grpc::CreateChannel(Address, grpc::InsecureChannelCredentials()));
	grpc::ClientContext Context;
	Context.set_deadline(std::chrono::system_clock::now() + std::chrono::seconds(10));
	std::unique_ptr<grpc::ClientReaderWriter<dm_env_rpc::v1::EnvironmentRequest,
		dm_env_rpc::v1::EnvironmentResponse>> Stream = Stub->Process(&Context);
	dm_env_rpc::v1::EnvironmentRequest Request;
	dm_env_rpc::v1::EnvironmentResponse Response;
	TestTrue(TEXT("idle extension stream enters the service"), Stream->Write(Request));
	TestTrue(TEXT("service answers before becoming idle"), Stream->Read(&Response));
	TestTrue(TEXT("standard request receives the typed unimplemented error"),
		Response.has_error()
			&& Response.error().code() == static_cast<int32>(grpc::StatusCode::UNIMPLEMENTED));

	const double StartedAt = FPlatformTime::Seconds();
	Transport->TransportShutdown();
	const double ShutdownSeconds = FPlatformTime::Seconds() - StartedAt;
	TestTrue(TEXT("shutdown deadline cancels an idle stream and joins the worker"),
		ShutdownSeconds <= static_cast<double>(GrpcShutdownGrace.count()) + 2.0);
	(void)Stream->Finish();
	Transport->RemoveFromRoot();
	return true;
}
#endif
