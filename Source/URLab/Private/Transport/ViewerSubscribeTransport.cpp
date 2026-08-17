// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
// trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

#include "Transport/ViewerSubscribeTransport.h"

#include "Transport/ClientSubscribeTransport.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "Bridge/MsgpackHelpers.h"
#include "Utils/URLabLogging.h"
#include "Dom/JsonObject.h"

#include "mujoco/mujoco.h"

namespace
{
// Pull a JSON array field into a TArray<double>. Missing/!array -> empty.
void ReadDoubles(const TSharedPtr<FJsonObject>& Obj, const TCHAR* Field, TArray<double>& Out)
{
	Out.Reset();
	const TArray<TSharedPtr<FJsonValue>>* Arr = nullptr;
	if (Obj->TryGetArrayField(Field, Arr) && Arr)
	{
		Out.Reserve(Arr->Num());
		for (const TSharedPtr<FJsonValue>& V : *Arr)
			Out.Add(V.IsValid() ? V->AsNumber() : 0.0);
	}
}
} // namespace

void UURLabViewerSubscribeTransport::SetOwningManager(AAMjManager* InMgr)
{
	OwningManager = InMgr;
}

bool UURLabViewerSubscribeTransport::TransportInit()
{
	if (bIsInitialized)
		return true;
	if (SourceEndpoint.IsEmpty())
	{
		UE_LOG(LogURLabNet, Warning, TEXT("ViewerSubscribeTransport: empty SourceEndpoint; not started"));
		return false;
	}

	AAMjManager* Mgr = OwningManager.Get();
	Engine = Mgr ? Mgr->PhysicsEngine : nullptr;
	if (!Engine)
	{
		UE_LOG(LogURLabNet, Warning, TEXT("ViewerSubscribeTransport: no PhysicsEngine; not started"));
		return false;
	}

	// Receive the "viewer" topic through the agnostic client-subscribe transport
	// (backend chosen by the base); OnMessage decodes + applies each newest frame.
	Sub = UURLabClientSubscribeTransport::Create(this, SourceEndpoint, Topic,
		UURLabClientSubscribeTransport::FOnClientMessage::CreateUObject(
			this, &UURLabViewerSubscribeTransport::OnMessage));
	if (!Sub)
	{
		UE_LOG(LogURLabNet, Error, TEXT("ViewerSubscribeTransport: connect failed to %s"), *SourceEndpoint);
		return false;
	}
	bIsInitialized = true;
	UE_LOG(LogURLabNet, Log, TEXT("ViewerSubscribeTransport: subscribing to %s (topic '%s')"),
		*SourceEndpoint, *Topic);
	return true;
}

void UURLabViewerSubscribeTransport::TransportShutdown()
{
	if (!bIsInitialized)
		return;
	if (Sub)
	{
		Sub->TransportShutdown();
		Sub = nullptr;
	}
	Engine = nullptr;
	bIsInitialized = false;
}

void UURLabViewerSubscribeTransport::OnMessage(const FString& /*InTopic*/, const TArray<uint8>& Payload)
{
	// Worker thread (of the client-subscribe transport): decode the newest
	// {t,qpos,qvel} frame and apply it into this viewer's own mjData.
	TSharedPtr<FJsonObject> Obj;
	if (!FURLabMsgpackUtil::UnpackToJsonObject(Payload.GetData(), Payload.Num(), Obj) || !Obj.IsValid())
	{
		return;
	}
	double Time = 0.0;
	Obj->TryGetNumberField(TEXT("t"), Time);
	TArray<double> QPos;
	TArray<double> QVel;
	ReadDoubles(Obj, TEXT("qpos"), QPos);
	ReadDoubles(Obj, TEXT("qvel"), QVel);
	ApplyFrame(Time, QPos, QVel);
}

void UURLabViewerSubscribeTransport::ApplyFrame(double Time, const TArray<double>& QPos, const TArray<double>& QVel)
{
	if (!Engine)
		return;
	// Same discipline as the puppet apply: fetch model/data AFTER the lock, since
	// a concurrent CompileModel frees them under CallbackMutex.
	FScopeLock Lock(&Engine->CallbackMutex);
	mjModel* m = Engine->GetModel();
	mjData* d = Engine->GetData();
	if (!m || !d)
		return;

	// Model-identity guard: the owner's qpos/qvel must fit this viewer's model.
	// A mismatch means the viewer loaded a different scene than the owner; apply
	// nothing (a wrong scatter would render garbage) and warn once so it is
	// diagnosable instead of a silent freeze.
	if (QPos.Num() != m->nq || QVel.Num() != m->nv)
	{
		if (!bWarnedMismatch)
		{
			UE_LOG(LogURLabNet, Warning,
				TEXT("ViewerSubscribeTransport: model mismatch -- owner sent qpos=%d qvel=%d but this ")
				TEXT("viewer has nq=%d nv=%d. Load the SAME scene/model as the owner; ignoring frames ")
				TEXT("until it matches."),
				QPos.Num(), QVel.Num(), m->nq, m->nv);
			bWarnedMismatch = true;
		}
		return;
	}

	FMemory::Memcpy(d->qpos, QPos.GetData(), m->nq * sizeof(mjtNum));
	FMemory::Memcpy(d->qvel, QVel.GetData(), m->nv * sizeof(mjtNum));
	d->time = Time;
	mj_forward(m, d);
	Engine->PushRenderState();

	if (bWarnedMismatch)
	{
		UE_LOG(LogURLabNet, Log, TEXT("ViewerSubscribeTransport: model matches again; resuming render."));
		bWarnedMismatch = false;
	}
}
