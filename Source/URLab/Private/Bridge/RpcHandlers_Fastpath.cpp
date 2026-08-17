// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "Bridge/RpcDispatcher.h"
#include "Bridge/RpcErrorCodes.h"
#include "Bridge/BridgeServer.h"
#include "Bridge/BridgeServerConfig.h"
#include "Utils/MsgpackHelpers.h"

#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Entity/MjModelSource.h"
#include "MuJoCo/Fast/MjRenderer.h"
#include "MuJoCo/Spec/MjSceneMjcf.h"
#include "Utils/URLabLogging.h"

#include "Dom/JsonObject.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "Async/Async.h"
#include "Misc/Base64.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

// Fast-path owner handshake. A fast-path renderer (AMjRenderer) sends
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

	// Advertise the source format so a renderer can recompile locally instead of
	// loading a version-locked MJB. When the compiled MJCF is available the scene
	// is served as XML plus its VFS bundle (participant specs + asset files), which
	// the renderer compiles with its own libmujoco -- immune to MJB version skew.
	// The MJB above stays as the back-compat fallback for a renderer that does not
	// read `model_format`. XML is only offered with the assets it references, so a
	// renderer never receives text it cannot compile.
	Reply->SetStringField(TEXT("model_format"), TEXT("mjb"));
	{
		FMjCompiledScene CompiledScene;
		FString SceneError;
		if (Mgr->PhysicsEngine->BuildCompiledScene(CompiledScene, SceneError))
		{
			Reply->SetStringField(TEXT("model_format"), TEXT("xml"));
			Reply->SetStringField(TEXT("xml"), CompiledScene.Xml);

			TSharedPtr<FJsonObject> VfsAssets = MakeShared<FJsonObject>();
			MjForEachSceneVfsEntry(CompiledScene.AssetFiles, CompiledScene.ParticipantXml,
				[&VfsAssets](const FString& Name, TArrayView<const uint8> Bytes) {
					FURLabMsgpackUtil::SetBinaryField(VfsAssets, Name, Bytes.GetData(), Bytes.Num());
				});
			Reply->SetObjectField(TEXT("vfs_assets"), VfsAssets);
		}
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

// Fast-path live scene swap. An owner/controller ships new MJB bytes; the running
// render-server renderer retires its current model and rebuilds from the new one
// without a relaunch. Bytes travel over the wire (msgpack bin -> base64), so a
// renderer on a different machine than the owner works -- this is a render server.
TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleFastpathLoad(const TSharedPtr<FJsonObject>& Req)
{
	if (!OwnerMgr.IsValid())
	{
		return MakeError(URLabError::NotReady, TEXT("no manager to load a fast-path scene into"));
	}

	// New model bytes: msgpack bin arrives as base64 under a `__b64__`-suffixed key
	// (accept the bare key too, e.g. an already-base64 string). Accept the `mjb` key
	// for back-compat and the format-neutral `model` key alongside it.
	FString B64;
	if (!Req->TryGetStringField(TEXT("mjb__b64__"), B64) && !Req->TryGetStringField(TEXT("mjb"), B64)
		&& !Req->TryGetStringField(TEXT("model__b64__"), B64) && !Req->TryGetStringField(TEXT("model"), B64))
	{
		return MakeError(URLabError::BadRequest, TEXT("missing model bytes"));
	}
	TArray<uint8> Src;
	if (!FBase64::Decode(B64, Src) || Src.Num() == 0)
	{
		return MakeError(URLabError::BadRequest, TEXT("model bytes are not valid base64 or are empty"));
	}

	// Normalize whatever form arrived to an mjb buffer with THIS libmujoco, so xml/mjz recompile
	// locally (immune to MJB version skew) and the existing mjb reload path is reused unchanged.
	FString Format = TEXT("mjb");
	Req->TryGetStringField(TEXT("format"), Format);
	TArray<uint8> Mjb;
	if (Format.Equals(TEXT("mjb"), ESearchCase::IgnoreCase))
	{
		Mjb = MoveTemp(Src);
	}
	else
	{
		TMap<FString, TArray<uint8>> Assets;
		const TSharedPtr<FJsonObject>* AssetsObj = nullptr;
		if (Req->TryGetObjectField(TEXT("assets"), AssetsObj) && AssetsObj && AssetsObj->IsValid())
		{
			for (const TPair<FString, TSharedPtr<FJsonValue>>& KV : (*AssetsObj)->Values)
			{
				TArray<uint8> Bytes;
				FString AssetB64;
				if (KV.Value.IsValid() && KV.Value->TryGetString(AssetB64) && FBase64::Decode(AssetB64, Bytes))
				{
					Assets.Add(KV.Key, MoveTemp(Bytes));
				}
			}
		}
		FString Err;
		mjModel* Compiled = MjModelSource::FromBytes(Src, Format, Assets, Err);
		if (Compiled == nullptr)
		{
			return MakeError(URLabError::BadRequest,
				FString::Printf(TEXT("model format '%s' did not compile: %s"), *Format, *Err));
		}
		const int32 Sz = mj_sizeModel(Compiled);
		Mjb.SetNumUninitialized(Sz);
		mj_saveModel(Compiled, nullptr, Mjb.GetData(), Sz);
		mj_deleteModel(Compiled);
		if (Mjb.Num() == 0)
		{
			return MakeError(URLabError::BadRequest, TEXT("normalized model serialized to zero bytes"));
		}
	}

	// Finding the renderer (TActorIterator) AND the reload both assert game-thread,
	// so marshal the whole thing there. Fire-and-forget: acknowledge the accepted
	// bytes now; the swap happens on the next game tick.
	const int32 NumBytes = Mjb.Num();
	TWeakObjectPtr<AAMjManager> WeakMgr = OwnerMgr;
	AsyncTask(ENamedThreads::GameThread, [WeakMgr, Mjb = MoveTemp(Mjb)]() {
		AAMjManager* Mgr = WeakMgr.Get();
		UWorld* World = Mgr ? Mgr->GetWorld() : nullptr;
		if (World == nullptr)
		{
			return;
		}
		AMjRenderer* Scene = nullptr;
		for (TActorIterator<AMjRenderer> It(World); It; ++It)
		{
			Scene = *It;
			break;
		}
		if (Scene != nullptr)
		{
			Scene->ReloadFromBytes(Mjb);
		}
		else
		{
			UE_LOG(LogURLab, Warning, TEXT("[fastpath_load] no fast-path renderer in this world"));
		}
	});

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("fastpath_load_ok"));
	Reply->SetNumberField(TEXT("bytes"), NumBytes);
	return Reply;
}

// Fast-path interactive perturbation. A renderer (Mirror/viewer) forwards a drag
// as an external force+torque on a body back to this owner; we stamp it into the
// live model's xfrc_applied so the owner's own step integrates it. The renderer
// already converts UE world space to MuJoCo (AMjRenderer::SendPerturbation), and
// MuJoCo's xfrc_applied is [force xyz, torque xyz], so the wire vectors are copied
// straight through with no further frame or unit change. This mirrors the Python
// FastPathOwner, whose step loop writes the same 6-vector to data.xfrc_applied; the
// value persists until the next drag frame overwrites it or a zero wrench clears it.
TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleFastpathPerturb(const TSharedPtr<FJsonObject>& Req)
{
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr || !Mgr->PhysicsEngine)
	{
		return MakeError(URLabError::NotReady,
			TEXT("no live model to perturb (start a live/direct session first)"));
	}
	// Interactive input is a capability: an instance with AcceptInput off refuses forwarded perturbs.
	if (!Mgr->HasCapability(EMjCapability::AcceptInput))
	{
		return MakeError(URLabError::CapabilityDisabled,
			TEXT("this instance does not accept input (AcceptInput capability is off)"));
	}

	double BodyNum = -1.0;
	if (!Req->TryGetNumberField(TEXT("body"), BodyNum) || BodyNum < 0.0)
	{
		return MakeError(URLabError::BadRequest, TEXT("missing or negative body id"));
	}
	const int32 BodyId = static_cast<int32>(BodyNum);

	// Reject an out-of-range id under the fence so a stale renderer can't index past
	// the model; the pointer can retire between the check and the wrench write.
	{
		FScopeLock Lock(&Mgr->PhysicsEngine->CallbackMutex);
		mjModel* m = Mgr->PhysicsEngine->GetModel();
		if (!m)
		{
			return MakeError(URLabError::NotReady, TEXT("model not loaded"));
		}
		if (BodyId >= m->nbody)
		{
			return MakeError(URLabError::BadRequest,
				FString::Printf(TEXT("body id %d out of range (nbody=%d)"), BodyId, m->nbody));
		}
	}

	// force / torque are MuJoCo-frame 3-vectors; pad a short/absent array to 3 so a
	// truncated wire vector can't index past the end, matching the Python owner.
	auto ReadVec3 = [&Req](const TCHAR* Field, double Out[3]) {
		Out[0] = Out[1] = Out[2] = 0.0;
		const TArray<TSharedPtr<FJsonValue>>* Arr = nullptr;
		if (Req->TryGetArrayField(Field, Arr) && Arr)
		{
			for (int32 i = 0; i < 3 && i < Arr->Num(); ++i)
			{
				Out[i] = (*Arr)[i]->AsNumber();
			}
		}
	};
	double Force[3];
	double Torque[3];
	ReadVec3(TEXT("force"), Force);
	ReadVec3(TEXT("torque"), Torque);

	// The command channel enqueues under CommandMutex and drains into
	// d->xfrc_applied under CallbackMutex right before mj_step -- the same
	// thread-safe apply path UMjBody::ApplyForce uses.
	const double Xfrc[6] = {Force[0], Force[1], Force[2], Torque[0], Torque[1], Torque[2]};
	Mgr->PhysicsEngine->SubmitWrench(BodyId, Xfrc);

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("fastpath_perturb_ok"));
	Reply->SetNumberField(TEXT("body"), BodyId);
	return Reply;
}
