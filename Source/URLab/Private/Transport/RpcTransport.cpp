// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "Transport/RpcTransport.h"
#include "Bridge/BridgeServer.h"
#include "Bridge/RpcDispatcher.h"
#include "Bridge/RpcErrorCodes.h"
#include "Utils/MsgpackHelpers.h"
#include "Bridge/OpRegistry.h"
#include "Wire/UrlabWire.gen.h"
#include "Wire/RequestEnvelope.h"

FURLabRpcDispatcher* UURLabRpcTransport::ResolveDispatcher() const
{
	UURLabBridgeServer* Bridge = OwningBridge.Get();
	return Bridge ? Bridge->GetDispatcher() : nullptr;
}

void UURLabRpcTransport::EncodeReply(const TSharedPtr<FJsonObject>& Reply,
	TArray<uint8>& OutBytes) const
{
	OutBytes.Reset();
	FURLabMsgpackUtil::PackJsonObject(Reply, OutBytes);
}

bool UURLabRpcTransport::ProcessRequestBytes(const TArray<uint8>& InBytes,
	TArray<uint8>& OutReplyBytes, const FString* OuterOp)
{
	FURLabRpcDispatcher* Disp = ResolveDispatcher();
	if (!Disp)
	{
		FURLabProtocolError Error{TEXT("not_ready"), TEXT("Bridge / dispatcher missing")};
		return FURLabRequestEnvelopeCodec::EncodeProtocolError(Error, OutReplyBytes).IsOk();
	}
	FString Adapter = GetTransportName();
	if (Adapter == TEXT("dm_env_rpc")) Adapter = TEXT("grpc");
	return Disp->DispatchBytes(InBytes, Adapter, OutReplyBytes, OuterOp);
}
