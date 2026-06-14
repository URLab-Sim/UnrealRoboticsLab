// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "Transport/RpcTransport.h"
#include "Bridge/BridgeServer.h"
#include "Bridge/RpcDispatcher.h"
#include "Bridge/MsgpackHelpers.h"
#include "Bridge/OpRegistry.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonWriter.h"
#include "Policies/CondensedJsonPrintPolicy.h"

namespace
{
FString SerializeRpcReplyJson(const TSharedPtr<FJsonObject>& Obj)
{
	FString Out;
	TSharedRef<TJsonWriter<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>> Writer =
		TJsonWriterFactory<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>::Create(&Out);
	if (Obj.IsValid())
	{
		FJsonSerializer::Serialize(Obj.ToSharedRef(), Writer);
	}
	return Out;
}
} // namespace

void UURLabRpcTransport::SetOwningBridge(UURLabBridgeServer* Bridge)
{
	OwningBridge = Bridge;
}

FURLabRpcDispatcher* UURLabRpcTransport::ResolveDispatcher() const
{
	UURLabBridgeServer* Bridge = OwningBridge.Get();
	return Bridge ? Bridge->GetDispatcher() : nullptr;
}

bool UURLabRpcTransport::ProcessRequestBytes(const TArray<uint8>& InBytes,
	TArray<uint8>& OutReplyBytes)
{
	OutReplyBytes.Reset();

	// Wire detect: JSON requests start with '{' / whitespace; msgpack
	// maps start with 0x80..0x8f (fixmap), 0xde (map16) or 0xdf (map32).
	TSharedPtr<FJsonObject> Req;
	bool bIsMsgpack = false;
	if (InBytes.Num() > 0)
	{
		const uint8 Lead = InBytes[0];
		const bool bLooksJson = (Lead == '{' || Lead == ' ' || Lead == '\t'
								 || Lead == '\n' || Lead == '\r');
		if (!bLooksJson)
		{
			bIsMsgpack = FURLabMsgpackUtil::UnpackToJsonObject(
				InBytes.GetData(), InBytes.Num(), Req);
		}
		if (!Req.IsValid())
		{
			FString JsonStr = FString(InBytes.Num(),
				UTF8_TO_TCHAR((const char*)InBytes.GetData()));
			TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(JsonStr);
			if (!FJsonSerializer::Deserialize(Reader, Req))
				Req.Reset();
			bIsMsgpack = false;
		}
	}

	FURLabRpcDispatcher* Disp = ResolveDispatcher();

	TSharedPtr<FJsonObject> Reply;
	if (!Disp)
	{
		Reply = FURLabRpcDispatcher::MakeError(TEXT("not_ready"),
			TEXT("Bridge / dispatcher missing"));
	}
	else if (!AcceptsEditorOps() && Req.IsValid())
	{
		// SHM scope narrowing: editor-only ops never reach the
		// dispatcher on a runtime-only transport. The bridge-side client
		// re-routes such ops to ZMQ on receipt of `wrong_transport`.
		FString Op;
		Req->TryGetStringField(TEXT("op"), Op);
		if (URLabOpRegistry::IsEditorOnlyOp(Op))
		{
			Reply = FURLabRpcDispatcher::MakeError(TEXT("wrong_transport"),
				FString::Printf(TEXT("op '%s' not accepted on %s; use zmq"),
					*Op, *GetTransportName()));
		}
		else
		{
			Reply = Disp->Dispatch(Req);
		}
	}
	else
	{
		Reply = Disp->Dispatch(Req);
	}

	// Reply encoding: follow the dispatcher's stored flag once a session
	// is up; pre-session, mirror the request's encoding so a msgpack
	// hello gets a msgpack hello_ok back.
	const bool bUseJson = Disp ? Disp->GetUseJsonEncoding() : !bIsMsgpack;
	if (!bUseJson)
	{
		FURLabMsgpackUtil::PackJsonObject(Reply, OutReplyBytes);
	}
	else
	{
		FString Out = SerializeRpcReplyJson(Reply);
		FTCHARToUTF8 OutUtf8(*Out);
		OutReplyBytes.SetNumUninitialized(OutUtf8.Length());
		FMemory::Memcpy(OutReplyBytes.GetData(), OutUtf8.Get(), OutUtf8.Length());
	}
	return true;
}
