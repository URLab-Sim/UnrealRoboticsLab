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
#include "Transport/MjExternalTransportProvider.h"
#include "Utils/MsgpackHelpers.h"
#include "URLabDmEnvRpc.h"
#include "Bridge/BridgeServer.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Async/Async.h"
#include "Dom/JsonObject.h"

class FURLabDmEnvRpcServiceImpl final : public dm_env_rpc::v1::Environment::Service
{
public:
	explicit FURLabDmEnvRpcServiceImpl(UURLabDmEnvRpcTransport* InTransport)
		: Transport(InTransport)
	{
	}

	virtual grpc::Status Process(
		grpc::ServerContext* Context,
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
					// subscribe(format=render): this call becomes a dedicated
					// server-stream of the owner's render frames until the client
					// cancels -- NOT the one-reply-per-request path below. The client
					// opens a separate Process call for this, so it never blocks its rpc
					// stream. Requires the owner to broadcast (-URLabBroadcastViewers=1).
					//
					// The one surviving subscribe is the render tier (op "view_frame",
					// per-body transforms + optional debug fields). The qpos render tier
					// (subscribe_viewer / subscribe{format:qpos}, {t,qpos,qvel}) was
					// removed in Phase 3.2. This is the H3 contract shared with the Python
					// owner, so a UE lean gRPC mirror renders over pure gRPC.
					if (InPacket.op() == "subscribe")
					{
						// Route the subscribe's debug-tier negotiation back to the UE
						// owner. The mirror packs {contacts, overlay, maxcontacts} into
						// the subscribe payload (DmEnvRpcClientTransports RunLoop); the
						// owning AAMjManager binds OnRenderDebugCapsRequested and folds
						// (unions) it into its thread-safe ActiveRenderDebugCaps, so a UE
						// gRPC owner serializes the requested contacts/overlay tier -- the
						// reverse leg of the OnViewerFrame render sink. A lean mirror sends
						// the bare {format:"render"} payload, which parses to no caps, so
						// the owner still streams zero debug bytes.
						//
						// Hop to a UE-managed background thread before broadcasting: the
						// owner's handler parses the payload via msgpack + JSON, and (per
						// the ProcessRequestBytes note above) UE JSON/msgpack allocation on
						// a raw gRPC thread SEGVs. Fire-and-forget -- the caps only need to
						// land once, and the frame stream below proceeds independently, so
						// at most the first frame or two after subscribe omit the debug
						// tier before the caps apply. (AsyncTask enqueues from any thread,
						// as the request paths below already rely on.) Copy the payload by
						// value so it outlives this Read iteration.
						if (FMjExternalTransportProvider::OnRenderDebugCapsRequested.IsBound()
							&& InPacket.payload().size() > 0)
						{
							TArray<uint8> SubBytes(
								reinterpret_cast<const uint8*>(InPacket.payload().data()),
								InPacket.payload().size());
							AsyncTask(ENamedThreads::AnyBackgroundThreadNormalTask,
								[SubBytes = MoveTemp(SubBytes)]()
								{
									if (FMjExternalTransportProvider::OnRenderDebugCapsRequested.IsBound())
									{
										FMjExternalTransportProvider::OnRenderDebugCapsRequested.Broadcast(SubBytes);
									}
								});
						}

						const int64 Seq = InPacket.sequence_id();
						uint64 LastSeq = 0;
						TArray<uint8> Frame;
						while (!Transport->ShouldStop() && !Context->IsCancelled())
						{
							const bool bGotFrame = Transport->GetRenderFrame(Frame, LastSeq);
							if (bGotFrame)
							{
								urlab::dm_env_rpc::v1::UrlabPacket OutFrame;
								OutFrame.set_op("view_frame");
								OutFrame.set_sequence_id(Seq);
								OutFrame.set_payload(Frame.GetData(), Frame.Num());
								dm_env_rpc::v1::EnvironmentResponse FrameResp;
								FrameResp.mutable_extension()->PackFrom(OutFrame);
								if (!Stream->Write(FrameResp))
								{
									break;
								}
							}
							else
							{
								FPlatformProcess::Sleep(0.005f);  // ~200Hz; only new frames
							}
						}
						continue;  // next Read ends the stream once the client is gone
					}

					TArray<uint8> InBytes(
						reinterpret_cast<const uint8*>(InPacket.payload().data()),
						InPacket.payload().size());
					TArray<uint8> OutReplyBytes;

					if (Transport->GetOwningBridge())
					{
						DispatchBytesOnBackgroundThread(InBytes, OutReplyBytes);
					}

					urlab::dm_env_rpc::v1::UrlabPacket OutPacket;
					OutPacket.set_op(InPacket.op());
					OutPacket.set_sequence_id(InPacket.sequence_id());
					OutPacket.set_payload(OutReplyBytes.GetData(), OutReplyBytes.Num());

					Response.mutable_extension()->PackFrom(OutPacket);
				}
				else
				{
					// Direct bytes payload in Any
					TArray<uint8> InBytes(
						reinterpret_cast<const uint8*>(Request.extension().value().data()),
						Request.extension().value().size());
					TArray<uint8> OutReplyBytes;

					if (Transport->GetOwningBridge())
					{
						DispatchBytesOnBackgroundThread(InBytes, OutReplyBytes);
					}

					urlab::dm_env_rpc::v1::UrlabPacket OutPacket;
					OutPacket.set_payload(OutReplyBytes.GetData(), OutReplyBytes.Num());
					Response.mutable_extension()->PackFrom(OutPacket);
				}
			}
			// 2. Standard dm_env_rpc operations
			else if (Request.has_join_world())
			{
				auto* JoinResp = Response.mutable_join_world();
				JoinResp->mutable_specs();
			}
			else if (Request.has_reset())
			{
				Response.mutable_reset();
			}
			else if (Request.has_step())
			{
				Response.mutable_step();
			}
			else if (Request.has_leave_world())
			{
				Response.mutable_leave_world();
			}

			if (!Stream->Write(Response))
			{
				break;
			}
		}
		return grpc::Status::OK;
	}

private:
	// Dispatch InBytes through the bridge on a UE-managed BACKGROUND task-graph
	// thread (never the game thread) and block until the reply is written into
	// OutReplyBytes. gRPC invokes us on its own non-UE threads, so we must hop
	// onto a UE thread before doing UE allocations; and it must not be the game
	// thread, because op handlers marshal to the game thread and block on it
	// (e.g. fastpath_render), which would deadlock. Matches the ZMQ / SHM model.
	void DispatchBytesOnBackgroundThread(const TArray<uint8>& InBytes, TArray<uint8>& OutReplyBytes)
	{
		FEvent* SyncEvent = FGenericPlatformProcess::GetSynchEventFromPool(false);
		AsyncTask(ENamedThreads::AnyBackgroundThreadNormalTask,
			[this, &InBytes, &OutReplyBytes, SyncEvent]()
			{
				Transport->ProcessRequestBytes(InBytes, OutReplyBytes);
				SyncEvent->Trigger();
			});
		SyncEvent->Wait();
		FGenericPlatformProcess::ReturnSynchEventToPool(SyncEvent);
	}

	UURLabDmEnvRpcTransport* Transport = nullptr;
};

class FURLabDmEnvRpcRunnable : public FRunnable
{
public:
	explicit FURLabDmEnvRpcRunnable(UURLabDmEnvRpcTransport* InTransport)
		: Transport(InTransport)
	{
	}

	virtual uint32 Run() override
	{
		if (Transport)
		{
			Transport->RunServerLoop();
		}
		return 0;
	}

private:
	UURLabDmEnvRpcTransport* Transport = nullptr;
};

UURLabDmEnvRpcTransport::UURLabDmEnvRpcTransport()
	: ListenPort(50051)
{
}

bool UURLabDmEnvRpcTransport::TransportInit()
{
	// -URLabDmEnvPort=N lets several render-server instances share one host on
	// distinct gRPC ports (default 50051 when absent, so a single instance is
	// unchanged). The RenderPool's orchestrator passes it per co-located instance.
	int32 PortOverride = 0;
	if (FParse::Value(FCommandLine::Get(), TEXT("URLabDmEnvPort="), PortOverride) && PortOverride > 0)
	{
		SetListenPort(PortOverride);
	}

	// Start the worker and block until it reports the REAL bind result before doing
	// anything that treats this transport as live. std::promise is one-shot, so give
	// it a fresh instance for this init and grab the paired future up front. (H2b)
	bShouldStop.store(false);
	BindResultPromise = std::promise<bool>();
	std::future<bool> BindResultFuture = BindResultPromise.get_future();

	WorkerRunnable = new FURLabDmEnvRpcRunnable(this);
	WorkerThread = FRunnableThread::Create(WorkerRunnable, TEXT("URLabDmEnvRpcServer"), 0, TPri_AboveNormal);
	if (!WorkerThread)
	{
		UE_LOG(LogURLabDmEnvRpc, Error, TEXT("Failed to create URLabDmEnvRpcServer worker thread."));
		delete WorkerRunnable;
		WorkerRunnable = nullptr;
		return false;
	}

	// Wait for the worker to actually bind the port. RunServerLoop sets this to true
	// once grpc::Server is listening, or false if BuildAndStart() fails to bind (e.g.
	// :50051 already in use). Blocking here keeps TransportInit honest -- a failed
	// bind must return false so the caller never registers this as a live transport.
	const bool bBound = BindResultFuture.get();
	if (!bBound)
	{
		// Bind failed: the worker (RunServerLoop) has already returned. Join and tear
		// it down. Crucially, no OnViewerFrame sink was registered yet, so a failed
		// :50051 bind leaves NO live sink copying frames into a dead transport (H2b).
		UE_LOG(LogURLabDmEnvRpc, Error, TEXT("[URLabDmEnvRpc] dm_env_rpc server failed to bind port %d; transport not registered."), ListenPort);
		bShouldStop.store(true);
		WorkerThread->WaitForCompletion();
		delete WorkerThread;
		WorkerThread = nullptr;
		delete WorkerRunnable;
		WorkerRunnable = nullptr;
		return false;
	}

	// Port is bound. Only now cache the owner's per-step render frames so a subscribe
	// stream can serve them. One sink for the "render" tier (transforms + optional
	// debug), streamed as "view_frame" by subscribe(format=render). Binding it here is
	// the H3 fix -- the gRPC egress no longer depends on a bound ZMQ viewer bus. Bound
	// only while this transport is up (and, per H2b, only after a successful bind, so a
	// dead transport never holds a live sink). (The qpos "viewer" tier was removed in
	// Phase 3.2.)
	ViewerSinkHandle = FMjExternalTransportProvider::OnViewerFrame.AddLambda(
		[this](const FString& Topic, const TArray<uint8>& Bytes)
		{
			if (Topic == TEXT("render"))
			{
				SetRenderFrame(Bytes);
			}
		});

	UE_LOG(LogURLabDmEnvRpc, Display, TEXT("[URLabDmEnvRpc] URLabDmEnvRpcTransport initialized on port %d."), ListenPort);
	return true;
}

void UURLabDmEnvRpcTransport::SetRenderFrame(const TArray<uint8>& Bytes)
{
	// The gRPC render seam (source-of-truth §8.1/§8.2). Fed by the Phase 2.4 owner
	// render sink; read by a subscribe(format=render) server-stream.
	FScopeLock Lock(&RenderCacheLock);
	LatestRenderFrame = Bytes;
	++RenderFrameSeq;
}

bool UURLabDmEnvRpcTransport::GetRenderFrame(TArray<uint8>& Out, uint64& InOutSeq) const
{
	FScopeLock Lock(&RenderCacheLock);
	if (RenderFrameSeq == InOutSeq || LatestRenderFrame.Num() == 0)
	{
		return false;
	}
	Out = LatestRenderFrame;
	InOutSeq = RenderFrameSeq;
	return true;
}

void UURLabDmEnvRpcTransport::TransportShutdown()
{
	if (ViewerSinkHandle.IsValid())
	{
		FMjExternalTransportProvider::OnViewerFrame.Remove(ViewerSinkHandle);
		ViewerSinkHandle.Reset();
	}
	bShouldStop.store(true);
	if (grpc::Server* S = Server.load())
	{
		S->Shutdown();
	}
	if (WorkerThread)
	{
		WorkerThread->WaitForCompletion();
		delete WorkerThread;
		WorkerThread = nullptr;
	}
	if (WorkerRunnable)
	{
		delete WorkerRunnable;
		WorkerRunnable = nullptr;
	}
	// Safe to delete unsynchronized past WaitForCompletion (worker has exited), but
	// go through the atomic to keep all access to Server consistent.
	if (grpc::Server* S = Server.load())
	{
		delete S;
		Server.store(nullptr);
	}
	UE_LOG(LogURLabDmEnvRpc, Display, TEXT("[URLabDmEnvRpc] URLabDmEnvRpcTransport shut down."));
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
	std::string ServerAddress = "0.0.0.0:" + std::to_string(ListenPort);
	FURLabDmEnvRpcServiceImpl Service(this);

	grpc::ServerBuilder Builder;
	Builder.SetMaxReceiveMessageSize(-1);
	Builder.SetMaxSendMessageSize(-1);
	Builder.AddListeningPort(ServerAddress, grpc::InsecureServerCredentials());
	Builder.RegisterService(&Service);

	std::unique_ptr<grpc::Server> StartedServer = Builder.BuildAndStart();
	if (!StartedServer)
	{
		UE_LOG(LogURLabDmEnvRpc, Error, TEXT("[URLabDmEnvRpc] Failed to start dm_env_rpc server on %s"), UTF8_TO_TCHAR(ServerAddress.c_str()));
		// Report the real bind failure back to TransportInit so it returns false and
		// never registers a live sink for this dead transport (H2b). Returning here
		// (with the promise satisfied) also unblocks init to tear the worker down.
		BindResultPromise.set_value(false);
		return;
	}

	Server.store(StartedServer.release());
	UE_LOG(LogURLabDmEnvRpc, Display, TEXT("[URLabDmEnvRpc] dm_env_rpc gRPC server listening on %s"), UTF8_TO_TCHAR(ServerAddress.c_str()));
	// Port is bound and the server is listening -- unblock TransportInit to proceed.
	BindResultPromise.set_value(true);

	while (!bShouldStop.load(std::memory_order_relaxed))
	{
		FPlatformProcess::Sleep(0.01f);
	}

	if (grpc::Server* S = Server.load())
	{
		S->Shutdown();
	}
}
