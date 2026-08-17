// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. This plugin incorporates
// third-party software: MuJoCo (Apache 2.0). See ThirdPartyNotices.txt.

#include "MuJoCo/Fast/MjRendererDriverClient.h"

#include "MuJoCo/Entity/MjModelSource.h"
#include "Utils/MsgpackHelpers.h"
#include "Dom/JsonObject.h"
#include "Misc/Base64.h"

#include "zmq.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

bool FMjRendererDriverClient::FetchModel(const FString& ControlEndpoint,
	TArray<uint8>& OutMjb, FString& OutBusEndpoint, FString& OutError)
{
	OutMjb.Reset();
	OutBusEndpoint.Empty();
	OutError.Empty();

	void* Ctx = zmq_ctx_new();
	void* Req = zmq_socket(Ctx, ZMQ_REQ);
	int Timeout = 5000;
	zmq_setsockopt(Req, ZMQ_RCVTIMEO, &Timeout, sizeof(Timeout));
	zmq_setsockopt(Req, ZMQ_SNDTIMEO, &Timeout, sizeof(Timeout));
	int Linger = 0;
	zmq_setsockopt(Req, ZMQ_LINGER, &Linger, sizeof(Linger));

	bool bOk = false;
	do
	{
		if (zmq_connect(Req, TCHAR_TO_UTF8(*ControlEndpoint)) != 0)
		{
			OutError = FString::Printf(TEXT("connect failed: %s"), *ControlEndpoint);
			break;
		}

		// Request: {op:"fastpath_hello"}. Both a Python owner and a UE live/direct
		// owner answer this with their MJB bytes and geoms-bus endpoint.
		TSharedPtr<FJsonObject> ReqObj = MakeShared<FJsonObject>();
		ReqObj->SetStringField(TEXT("op"), TEXT("fastpath_hello"));
		TArray<uint8> ReqBuf;
		FURLabMsgpackUtil::PackJsonObject(ReqObj, ReqBuf);
		if (zmq_send(Req, ReqBuf.GetData(), ReqBuf.Num(), 0) < 0)
		{
			OutError = TEXT("send failed");
			break;
		}

		zmq_msg_t Msg;
		zmq_msg_init(&Msg);
		if (zmq_msg_recv(&Msg, Req, 0) < 0)
		{
			zmq_msg_close(&Msg);
			OutError = TEXT("no reply (owner not answering fastpath_hello within timeout)");
			break;
		}
		TSharedPtr<FJsonObject> Reply;
		const bool bUnpacked = FURLabMsgpackUtil::UnpackToJsonObject(
			static_cast<const uint8*>(zmq_msg_data(&Msg)), static_cast<int32>(zmq_msg_size(&Msg)), Reply);
		zmq_msg_close(&Msg);
		if (!bUnpacked || !Reply.IsValid())
		{
			OutError = TEXT("reply was not msgpack");
			break;
		}

		// The owner advertises the source format. XML recompiles here with this
		// renderer's own libmujoco, so it is immune to MJB version skew; an owner
		// that only ships MJB (or omits the field) still loads through the raw path.
		FString Format = TEXT("mjb");
		Reply->TryGetStringField(TEXT("model_format"), Format);

		if (!Format.Equals(TEXT("mjb"), ESearchCase::IgnoreCase))
		{
			// Non-MJB is served as text (MJCF) plus the VFS bundle it references, so
			// it is normalized to an MJB with this libmujoco -- exactly what the push
			// path does in HandleFastpathLoad -- and the rest of the load is unchanged.
			FString Xml;
			if (!Reply->TryGetStringField(TEXT("xml"), Xml) || Xml.IsEmpty())
			{
				OutError = FString::Printf(TEXT("reply advertised format '%s' but carried no xml"), *Format);
				break;
			}

			// vfs_assets: each msgpack-bin value arrives base64-encoded under a
			// `__b64__`-suffixed key; strip the suffix to recover the mount name the
			// MJCF references, so a VFS built from this map resolves every `file=`.
			TMap<FString, TArray<uint8>> Assets;
			const TSharedPtr<FJsonObject>* VfsObj = nullptr;
			if (Reply->TryGetObjectField(TEXT("vfs_assets"), VfsObj) && VfsObj && VfsObj->IsValid())
			{
				for (const TPair<FString, TSharedPtr<FJsonValue>>& KV : (*VfsObj)->Values)
				{
					FString Name = KV.Key;
					Name.RemoveFromEnd(TEXT("__b64__"));
					FString AssetB64;
					TArray<uint8> AssetBytes;
					if (KV.Value.IsValid() && KV.Value->TryGetString(AssetB64)
						&& FBase64::Decode(AssetB64, AssetBytes) && AssetBytes.Num() > 0)
					{
						Assets.Add(Name, MoveTemp(AssetBytes));
					}
				}
			}

			const FTCHARToUTF8 XmlUtf8(*Xml);
			TArray<uint8> XmlBytes;
			XmlBytes.Append(reinterpret_cast<const uint8*>(XmlUtf8.Get()), XmlUtf8.Length());

			FString CompileErr;
			mjModel* Compiled = MjModelSource::FromBytes(XmlBytes, Format, Assets, CompileErr);
			if (Compiled == nullptr)
			{
				OutError = FString::Printf(TEXT("owner-served %s did not compile: %s"), *Format, *CompileErr);
				break;
			}
			const int32 Sz = mj_sizeModel(Compiled);
			OutMjb.SetNumUninitialized(Sz);
			mj_saveModel(Compiled, nullptr, OutMjb.GetData(), Sz);
			mj_deleteModel(Compiled);
			if (OutMjb.Num() == 0)
			{
				OutError = TEXT("recompiled model serialized to zero bytes");
				break;
			}
			Reply->TryGetStringField(TEXT("bus"), OutBusEndpoint);
			bOk = true;
			break;
		}

		// MJB bytes are msgpack bin, which unpacks to a base64 string under the
		// `__b64__`-suffixed key. Tolerate a plain base64 `mjb` string too.
		FString B64;
		if (!Reply->TryGetStringField(TEXT("mjb__b64__"), B64) || B64.IsEmpty())
		{
			Reply->TryGetStringField(TEXT("mjb"), B64);
		}
		if (B64.IsEmpty() || !FBase64::Decode(B64, OutMjb) || OutMjb.Num() == 0)
		{
			FString RemoteErr;
			Reply->TryGetStringField(TEXT("error"), RemoteErr);
			OutError = RemoteErr.IsEmpty() ? TEXT("reply carried no mjb") : RemoteErr;
			break;
		}
		Reply->TryGetStringField(TEXT("bus"), OutBusEndpoint);
		bOk = true;
	} while (false);

	if (Req)
	{
		zmq_close(Req);
	}
	if (Ctx)
	{
		zmq_ctx_term(Ctx);
	}
	return bOk;
}
