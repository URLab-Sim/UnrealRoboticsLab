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
	// UrlabPacket.op is the dispatch key on this face, and the caller hands us a
	// finished ZMQ-shaped body, so read the op out of it to fill the envelope.
	// The body's own copy is no longer a dispatch key anywhere: both servers now
	// route on the envelope, and the UE server overwrites the body's copy with it
	// before dispatch. The payload passes through unchanged -- re-encoding it
	// here to strip one key would round-trip every model upload through the JSON
	// representation for nothing.
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

// ============================ factory binders ================================ //

UURLabRpcClientTransport* MakeDmEnvRpcRpcClientTransport(UObject* Outer)
{
	return NewObject<UURLabDmEnvRpcRpcClientTransport>(Outer);
}
