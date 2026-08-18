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

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wundef"
#pragma clang diagnostic ignored "-Wshadow"
#pragma clang diagnostic ignored "-Wunused-parameter"
#pragma clang diagnostic ignored "-Wdeprecated-declarations"

#ifndef PROTOBUF_ENABLE_DEBUG_LOGGING_MAY_LEAK_PII
#define PROTOBUF_ENABLE_DEBUG_LOGGING_MAY_LEAK_PII 0
#endif

// Include third-party gRPC & Protobuf headers first before UE defines intrusive macros
#pragma push_macro("check")
#undef check
#pragma push_macro("TEXT")
#undef TEXT
#pragma push_macro("PI")
#undef PI

#include <grpcpp/grpcpp.h>
#include "dm_env_rpc.grpc.pb.h"
#include "urlab_dm_env_rpc.pb.h"

#pragma pop_macro("PI")
#pragma pop_macro("TEXT")
#pragma pop_macro("check")
#pragma clang diagnostic pop

#include "Transport/DmEnvRpcTransport.h"
#include "URLabDmEnvRpc.h"
#include "Bridge/BridgeServer.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"
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
                                                FEvent* SyncEvent = FGenericPlatformProcess::GetSynchEventFromPool(false);
                                                // Safely force memory mapping to standard allocations inside lambda to prevent SEGV
                                                AsyncTask(ENamedThreads::GameThread, [this, &InBytes, &OutReplyBytes, SyncEvent]()
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
                                                FEvent* SyncEvent = FGenericPlatformProcess::GetSynchEventFromPool(false);
                                                // Safely force memory mapping to standard allocations inside lambda to prevent SEGV
                                                AsyncTask(ENamedThreads::GameThread, [this, &InBytes, &OutReplyBytes, SyncEvent]()
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
