// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "Bridge/RpcDispatcher.h"
#include "Bridge/RpcErrorCodes.h"
#include "Bridge/BridgeServer.h"
#include "Bridge/BridgeServerConfig.h"
#include "Bridge/MsgpackHelpers.h"

#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"

#include "Dom/JsonObject.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

// Fast-path owner handshake. A fast-path renderer (AMjbScene) sends
// `fastpath_hello` to pull this owner's compiled MJB and the transform-bus
// endpoint, so it can build the scene with no shared file and subscribe to the
// geoms stream. This mirrors what a Python FastPathOwner serves, letting a UE
// live/direct instance act as an owner for renderers.
TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleFastpathHello(const TSharedPtr<FJsonObject>& /*Req*/)
{
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr || !Mgr->PhysicsEngine)
	{
		return MakeError(URLabError::NotReady,
			TEXT("no live model to serve (start a live/direct session first)"));
	}

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("fastpath_hello_ok"));

	// Serialize the MJB under the engine's fence: a concurrent compile/uninstall
	// can retire the model pointer between the check and the save. MJB bytes ride
	// as msgpack bin, the same idiom as the hello handshake.
	{
		FScopeLock Lock(&Mgr->PhysicsEngine->CallbackMutex);
		mjModel* m = Mgr->PhysicsEngine->GetModel();
		if (!m)
		{
			return MakeError(URLabError::NotReady, TEXT("model not loaded"));
		}
		const int Sz = mj_sizeModel(m);
		if (Sz <= 0)
		{
			return MakeError(URLabError::NotReady, TEXT("model has an invalid serialized size"));
		}
		TArray<uint8> Buf;
		Buf.SetNum(Sz);
		mj_saveModel(m, nullptr, Buf.GetData(), Sz);
		FURLabMsgpackUtil::SetBinaryField(Reply, TEXT("mjb"), Buf.GetData(), Sz);
		Reply->SetNumberField(TEXT("ngeom"), m->ngeom);
	}

	// The geoms transform bus endpoint the renderer subscribes to. This is the
	// owner's viewer port; the geoms broadcast rides that bus (see
	// AAMjManager::PublishGeomFrame). `broadcasting` is set in both cases so the
	// client never has to treat a missing field as true: false means no transform
	// stream, so the renderer would only show the rest pose.
	if (UURLabBridgeServer* Bridge = OwningBridge.Get())
	{
		const FURLabBridgeServerConfig& Cfg = Bridge->GetInstanceConfig();
		const FString Host = FPlatformProcess::ComputerName();
		Reply->SetStringField(TEXT("bus"),
			FString::Printf(TEXT("tcp://%s:%d"), *Host, Cfg.ViewerPort));
		Reply->SetBoolField(TEXT("broadcasting"), Cfg.bBroadcastViewers);
	}
	return Reply;
}
