// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef PROTOBUF_ENABLE_DEBUG_LOGGING_MAY_LEAK_PII
#define PROTOBUF_ENABLE_DEBUG_LOGGING_MAY_LEAK_PII 0
#endif

#include "Transport/DmEnvRpcClientTransports.h"

#include "URLabDmEnvRpc.h"
#include "Utils/MsgpackHelpers.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"
#include "HAL/PlatformProcess.h"

// UE's intrusive macros (check/verify/TEXT/PI, ...) collide with member names in
// grpc/abseil (e.g. absl btree's verify()), so undef the family across the
// third-party block and restore it after -- same as DmEnvRpcTransport.cpp.
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

#include <chrono>
#include <memory>
#include <string>

namespace
{
// A channel with unlimited message sizes (an owner's MJB can be tens of MB, well
// past gRPC's 4 MB default) -- matches the server's SetMax*MessageSize(-1).
std::shared_ptr<grpc::Channel> MakeChannel(const FString& Endpoint)
{
	grpc::ChannelArguments Args;
	Args.SetMaxReceiveMessageSize(-1);
	Args.SetMaxSendMessageSize(-1);
	return grpc::CreateCustomChannel(
		TCHAR_TO_UTF8(*Endpoint), grpc::InsecureChannelCredentials(), Args);
}

FString StripGrpcScheme(const FString& In)
{
	FString Out = In;
	Out.RemoveFromStart(TEXT("grpc://"));
	return Out;
}
} // namespace

// ============================ RPC client (req/reply) ========================= //

struct UURLabDmEnvRpcRpcClientTransport::FChan
{
	std::shared_ptr<grpc::Channel> Channel;
	std::unique_ptr<dm_env_rpc::v1::Environment::Stub> Stub;
	uint64 Seq = 0;
};

void UURLabDmEnvRpcRpcClientTransport::Configure(const FString& InEndpoint)
{
	Endpoint = StripGrpcScheme(InEndpoint);
}

bool UURLabDmEnvRpcRpcClientTransport::TransportInit()
{
	if (Endpoint.IsEmpty())
	{
		return false;
	}
	Chan = new FChan();
	Chan->Channel = MakeChannel(Endpoint);
	Chan->Stub = dm_env_rpc::v1::Environment::NewStub(Chan->Channel);
	UE_LOG(LogURLabDmEnvRpc, Log, TEXT("[dmenv-grpc-req] connected %s"), *Endpoint);
	return true;
}

bool UURLabDmEnvRpcRpcClientTransport::Request(
	const TArray<uint8>& Payload, TArray<uint8>& OutReply, int32 TimeoutMs)
{
	if (!Chan || !Chan->Stub)
	{
		return false;
	}
	// The gRPC envelope carries the op as a field; the ZMQ path put it inside the
	// msgpack body, so lift it out (payload passes through unchanged).
	FString Op;
	TSharedPtr<FJsonObject> ReqObj;
	if (FURLabMsgpackUtil::UnpackToJsonObject(Payload.GetData(), Payload.Num(), ReqObj)
		&& ReqObj.IsValid())
	{
		ReqObj->TryGetStringField(TEXT("op"), Op);
	}

	urlab::dm_env_rpc::v1::UrlabPacket InPkt;
	InPkt.set_op(TCHAR_TO_UTF8(*Op));
	InPkt.set_payload(std::string(reinterpret_cast<const char*>(Payload.GetData()), Payload.Num()));
	InPkt.set_sequence_id(++Chan->Seq);
	dm_env_rpc::v1::EnvironmentRequest EnvReq;
	EnvReq.mutable_extension()->PackFrom(InPkt);

	grpc::ClientContext Ctx;
	Ctx.set_deadline(std::chrono::system_clock::now() + std::chrono::milliseconds(TimeoutMs));
	std::shared_ptr<grpc::ClientReaderWriter<dm_env_rpc::v1::EnvironmentRequest,
		dm_env_rpc::v1::EnvironmentResponse>> Stream(Chan->Stub->Process(&Ctx).release());
	if (!Stream)
	{
		return false;
	}
	if (!Stream->Write(EnvReq))
	{
		return false;
	}
	Stream->WritesDone();
	dm_env_rpc::v1::EnvironmentResponse EnvResp;
	if (!Stream->Read(&EnvResp))
	{
		Stream->Finish();
		return false;
	}
	Stream->Finish();
	if (!EnvResp.has_extension())
	{
		return false;
	}
	urlab::dm_env_rpc::v1::UrlabPacket OutPkt;
	if (!EnvResp.extension().UnpackTo(&OutPkt))
	{
		return false;
	}
	const std::string& Pl = OutPkt.payload();
	OutReply.SetNumUninitialized(static_cast<int32>(Pl.size()));
	if (Pl.size() > 0)
	{
		FMemory::Memcpy(OutReply.GetData(), Pl.data(), Pl.size());
	}
	return true;
}

void UURLabDmEnvRpcRpcClientTransport::TransportShutdown()
{
	if (Chan)
	{
		delete Chan;
		Chan = nullptr;
	}
}

void UURLabDmEnvRpcRpcClientTransport::BeginDestroy()
{
	TransportShutdown();
	Super::BeginDestroy();
}

// ============================ Subscribe (stream) ============================= //

class FDmEnvSubRunnable : public FRunnable
{
public:
	explicit FDmEnvSubRunnable(UURLabDmEnvRpcClientSubscribeTransport* InOwner) : Owner(InOwner) {}
	virtual uint32 Run() override { if (Owner) { Owner->RunLoop(); } return 0; }
private:
	UURLabDmEnvRpcClientSubscribeTransport* Owner = nullptr;
};

struct UURLabDmEnvRpcClientSubscribeTransport::FSubState
{
	std::shared_ptr<grpc::Channel> Channel;
	std::unique_ptr<dm_env_rpc::v1::Environment::Stub> Stub;
	grpc::ClientContext* ActiveCtx = nullptr;   // for TryCancel on shutdown
	// Guards ActiveCtx: the worker sets/clears it around the stack-local Ctx while
	// TransportShutdown TryCancels it from another thread. Serializing the clear
	// (before Ctx is destroyed) against the TryCancel prevents a use-after-free.
	FCriticalSection CtxLock;
	FRunnable* Runnable = nullptr;
	FRunnableThread* Thread = nullptr;
};

void UURLabDmEnvRpcClientSubscribeTransport::Configure(
	const FString& InEndpoint, const FString& InTopic, FOnClientMessage InCallback)
{
	Endpoint = StripGrpcScheme(InEndpoint);
	Topic = InTopic;
	OnMessage = InCallback;
}

bool UURLabDmEnvRpcClientSubscribeTransport::TransportInit()
{
	if (Endpoint.IsEmpty())
	{
		return false;
	}
	Sub = new FSubState();
	Sub->Channel = MakeChannel(Endpoint);
	Sub->Stub = dm_env_rpc::v1::Environment::NewStub(Sub->Channel);
	bStop = false;
	Sub->Runnable = new FDmEnvSubRunnable(this);
	Sub->Thread = FRunnableThread::Create(Sub->Runnable, TEXT("URLabDmEnvGrpcSub"));
	if (!Sub->Thread)
	{
		return false;
	}
	UE_LOG(LogURLabDmEnvRpc, Log, TEXT("[dmenv-grpc-sub] subscribing '%s' on %s"), *Topic, *Endpoint);
	return true;
}

void UURLabDmEnvRpcClientSubscribeTransport::RunLoop()
{
	if (!Sub || !Sub->Stub)
	{
		return;
	}
	// subscribe payload: {format: "render"} -> the owner streams the render tier as
	// op "view_frame" (per-body bxpos/bxquat + optional camera/debug fields). The
	// format token matches the Python owner and the UE server (owner_server.py
	// _req_format / DmEnvRpcTransport subscribe handling): anything but "qpos"
	// selects the render tier.
	TSharedPtr<FJsonObject> SubObj = MakeShared<FJsonObject>();
	SubObj->SetStringField(TEXT("format"), TEXT("render"));
	TArray<uint8> SubPayload;
	FURLabMsgpackUtil::PackJsonObject(SubObj, SubPayload);

	while (!bStop)
	{
		grpc::ClientContext Ctx;
		{
			FScopeLock CtxScope(&Sub->CtxLock);
			Sub->ActiveCtx = &Ctx;
		}
		auto Stream = Sub->Stub->Process(&Ctx);

		urlab::dm_env_rpc::v1::UrlabPacket InPkt;
		InPkt.set_op("subscribe");
		InPkt.set_payload(std::string(reinterpret_cast<const char*>(SubPayload.GetData()), SubPayload.Num()));
		InPkt.set_sequence_id(1);
		dm_env_rpc::v1::EnvironmentRequest EnvReq;
		EnvReq.mutable_extension()->PackFrom(InPkt);

		if (Stream && Stream->Write(EnvReq))
		{
			dm_env_rpc::v1::EnvironmentResponse EnvResp;
			while (!bStop && Stream->Read(&EnvResp))
			{
				if (!EnvResp.has_extension())
				{
					continue;
				}
				urlab::dm_env_rpc::v1::UrlabPacket OutPkt;
				if (!EnvResp.extension().UnpackTo(&OutPkt) || OutPkt.op() != "view_frame")
				{
					continue;
				}
				const std::string& Pl = OutPkt.payload();
				TArray<uint8> Frame;
				Frame.SetNumUninitialized(static_cast<int32>(Pl.size()));
				if (Pl.size() > 0)
				{
					FMemory::Memcpy(Frame.GetData(), Pl.data(), Pl.size());
				}
				OnMessage.ExecuteIfBound(Topic, Frame);
			}
		}
		if (Stream)
		{
			Stream->Finish();
		}
		{
			// Clear under the lock before Ctx leaves scope, so a concurrent
			// TransportShutdown never TryCancels a destroyed ClientContext.
			FScopeLock CtxScope(&Sub->CtxLock);
			Sub->ActiveCtx = nullptr;
		}
		if (bStop)
		{
			break;
		}
		FPlatformProcess::Sleep(0.3f);   // reconnect backoff
	}
}

void UURLabDmEnvRpcClientSubscribeTransport::TransportShutdown()
{
	bStop = true;
	if (Sub)
	{
		{
			// Lock only around the cancel -- never across Thread->Kill below, or the
			// worker (which needs CtxLock to clear ActiveCtx) would deadlock the wait.
			FScopeLock CtxScope(&Sub->CtxLock);
			if (Sub->ActiveCtx)
			{
				Sub->ActiveCtx->TryCancel();
			}
		}
		if (Sub->Thread)
		{
			Sub->Thread->Kill(/*bShouldWait=*/true);
			delete Sub->Thread;
			Sub->Thread = nullptr;
		}
		if (Sub->Runnable)
		{
			delete Sub->Runnable;
			Sub->Runnable = nullptr;
		}
		delete Sub;
		Sub = nullptr;
	}
}

void UURLabDmEnvRpcClientSubscribeTransport::BeginDestroy()
{
	TransportShutdown();
	Super::BeginDestroy();
}

// ============================ factory binders ================================ //

UURLabRpcClientTransport* MakeDmEnvRpcRpcClientTransport(UObject* Outer)
{
	return NewObject<UURLabDmEnvRpcRpcClientTransport>(Outer);
}

UURLabClientSubscribeTransport* MakeDmEnvRpcClientSubscribeTransport(UObject* Outer)
{
	return NewObject<UURLabDmEnvRpcClientSubscribeTransport>(Outer);
}
