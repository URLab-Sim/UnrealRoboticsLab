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
					TArray<uint8> InBytes(
						reinterpret_cast<const uint8*>(InPacket.payload().data()),
						InPacket.payload().size());
					TArray<uint8> OutReplyBytes;

					if (Transport->GetOwningBridge())
					{
						// Dispatch on a UE-managed BACKGROUND task-graph thread -- not the
						// game thread. Two constraints force this:
						//  * ProcessRequestBytes does UE allocations (TArray / JSON / msgpack).
						//    gRPC invokes us on its own threads, which aren't registered UE
						//    threads; allocating there SEGVs -- hence the original code hopped
						//    onto a UE thread first.
						//  * It must NOT be the game thread: op handlers marshal to the game
						//    thread and block on it (e.g. fastpath_render enqueues its capture
						//    as AsyncTask(GameThread) and waits). Running here on the game
						//    thread queues that capture behind the blocked game thread and
						//    deadlocks it ("fastpath_render game-thread capture timed out").
						// A background task-graph thread satisfies both (matches the ZMQ / SHM
						// model, which run ProcessRequestBytes off the game thread).
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
						// Same as above: a UE-managed background thread, never the game
						// thread, so game-thread-marshaling handlers can't deadlock.
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

	bShouldStop.store(false);
	WorkerRunnable = new FURLabDmEnvRpcRunnable(this);
	WorkerThread = FRunnableThread::Create(WorkerRunnable, TEXT("URLabDmEnvRpcServer"), 0, TPri_AboveNormal);
	if (!WorkerThread)
	{
		UE_LOG(LogURLabDmEnvRpc, Error, TEXT("Failed to create URLabDmEnvRpcServer worker thread."));
		delete WorkerRunnable;
		WorkerRunnable = nullptr;
		return false;
	}

	UE_LOG(LogURLabDmEnvRpc, Display, TEXT("[URLabDmEnvRpc] URLabDmEnvRpcTransport initialized on port %d."), ListenPort);
	return true;
}

void UURLabDmEnvRpcTransport::TransportShutdown()
{
	bShouldStop.store(true);
	if (Server)
	{
		Server->Shutdown();
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
	if (Server)
	{
		delete Server;
		Server = nullptr;
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
	Builder.SetMaxReceiveMessageSize(-1);
        Builder.SetMaxSendMessageSize(-1);
        Builder.AddListeningPort(ServerAddress, grpc::InsecureServerCredentials());
	Builder.RegisterService(&Service);

	std::unique_ptr<grpc::Server> StartedServer = Builder.BuildAndStart();
	if (!StartedServer)
	{
		UE_LOG(LogURLabDmEnvRpc, Error, TEXT("[URLabDmEnvRpc] Failed to start dm_env_rpc server on %s"), UTF8_TO_TCHAR(ServerAddress.c_str()));
		return;
	}

	Server = StartedServer.release();
	UE_LOG(LogURLabDmEnvRpc, Display, TEXT("[URLabDmEnvRpc] dm_env_rpc gRPC server listening on %s"), UTF8_TO_TCHAR(ServerAddress.c_str()));

	while (!bShouldStop.load(std::memory_order_relaxed))
	{
		FPlatformProcess::Sleep(0.01f);
	}

	if (Server)
	{
		Server->Shutdown();
	}
}
