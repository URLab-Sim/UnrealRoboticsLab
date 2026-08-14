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

#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "Bridge/MsgpackHelpers.h"
#include "Utils/URLabLogging.h"
#include "Dom/JsonObject.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"

#include "zmq.h"
#include "mujoco/mujoco.h"

namespace
{
// Drive RunReceiveLoop on a dedicated thread so the render cadence follows the
// owner's broadcast rate, independent of the (paused) physics step loop.
class FViewerReceiveRunnable : public FRunnable
{
public:
	explicit FViewerReceiveRunnable(UURLabViewerSubscribeTransport* InT) : T(InT) {}
	virtual uint32 Run() override
	{
		if (T)
			T->RunReceiveLoop();
		return 0;
	}
	virtual void Stop() override
	{
		if (T)
			T->bStop = true;
	}

private:
	UURLabViewerSubscribeTransport* T = nullptr;
};

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

	ZmqContext = zmq_ctx_new();
	Subscriber = zmq_socket(ZmqContext, ZMQ_SUB);

	int Timeout = 200; // ms: recv returns so the loop can observe bStop
	zmq_setsockopt(Subscriber, ZMQ_RCVTIMEO, &Timeout, sizeof(Timeout));
	int Linger = 0;
	zmq_setsockopt(Subscriber, ZMQ_LINGER, &Linger, sizeof(Linger));
	// A viewer only ever wants the newest state, so keep a shallow inbound queue
	// (drop old frames rather than let a backlog build if this thread stalls).
	int RcvHwm = 8;
	zmq_setsockopt(Subscriber, ZMQ_RCVHWM, &RcvHwm, sizeof(RcvHwm));

	if (zmq_connect(Subscriber, TCHAR_TO_UTF8(*SourceEndpoint)) != 0)
	{
		UE_LOG(LogURLabNet, Error, TEXT("ViewerSubscribeTransport: connect failed to %s"), *SourceEndpoint);
		zmq_close(Subscriber);
		Subscriber = nullptr;
		zmq_ctx_term(ZmqContext);
		ZmqContext = nullptr;
		return false;
	}
	{
		const FTCHARToUTF8 TopicUtf8(*Topic);
		zmq_setsockopt(Subscriber, ZMQ_SUBSCRIBE, TopicUtf8.Get(), TopicUtf8.Length());
	}

	bIsInitialized = true;
	bStop = false;
	WorkerRunnable = new FViewerReceiveRunnable(this);
	WorkerThread = FRunnableThread::Create(WorkerRunnable, TEXT("URLabViewerSub"));
	UE_LOG(LogURLabNet, Log, TEXT("ViewerSubscribeTransport: subscribing to %s (topic '%s')"),
		*SourceEndpoint, *Topic);
	return true;
}

void UURLabViewerSubscribeTransport::TransportShutdown()
{
	if (!bIsInitialized)
		return;
	bStop = true;
	if (WorkerThread)
	{
		WorkerThread->WaitForCompletion();
		delete WorkerThread;
		WorkerThread = nullptr;
	}
	delete WorkerRunnable;
	WorkerRunnable = nullptr;
	if (Subscriber)
	{
		zmq_close(Subscriber);
		Subscriber = nullptr;
	}
	if (ZmqContext)
	{
		zmq_ctx_term(ZmqContext);
		ZmqContext = nullptr;
	}
	Engine = nullptr;
	bIsInitialized = false;
}

void UURLabViewerSubscribeTransport::RunReceiveLoop()
{
	// Receive one [topic, payload] pair. bBlock: honour RCVTIMEO (first read of
	// a batch); else non-blocking (draining backlog). Returns true and fills
	// OutPayload when a payload frame arrived.
	auto RecvPair = [this](bool bBlock, TArray<uint8>& OutPayload) -> bool
	{
		const int Flags = bBlock ? 0 : ZMQ_DONTWAIT;
		zmq_msg_t TopicMsg;
		zmq_msg_init(&TopicMsg);
		const int rc = zmq_msg_recv(&TopicMsg, Subscriber, Flags);
		if (rc < 0)
		{
			zmq_msg_close(&TopicMsg);
			return false; // timeout / EAGAIN / interrupted
		}
		int More = 0;
		size_t MoreSz = sizeof(More);
		zmq_getsockopt(Subscriber, ZMQ_RCVMORE, &More, &MoreSz);
		zmq_msg_close(&TopicMsg);
		if (!More)
			return false; // malformed: topic with no payload

		zmq_msg_t PayMsg;
		zmq_msg_init(&PayMsg);
		if (zmq_msg_recv(&PayMsg, Subscriber, 0) < 0)
		{
			zmq_msg_close(&PayMsg);
			return false;
		}
		const int Size = zmq_msg_size(&PayMsg);
		OutPayload.SetNumUninitialized(Size);
		if (Size > 0)
			FMemory::Memcpy(OutPayload.GetData(), zmq_msg_data(&PayMsg), Size);
		zmq_msg_close(&PayMsg);
		return true;
	};

	TArray<uint8> Payload;
	while (!bStop.load(std::memory_order_acquire))
	{
		// Block for one frame, then drain any backlog and keep only the newest,
		// so a slow viewer never lags the owner's authoritative state.
		if (!RecvPair(/*bBlock=*/true, Payload))
			continue;
		// Drain the backlog and keep only the newest frame. Bounded so a
		// flooding publisher can never trap the loop here.
		int32 DrainGuard = 0;
		while (RecvPair(/*bBlock=*/false, Payload) && ++DrainGuard < 4096)
		{
		}

		TSharedPtr<FJsonObject> Obj;
		if (!FURLabMsgpackUtil::UnpackToJsonObject(Payload.GetData(), Payload.Num(), Obj) || !Obj.IsValid())
			continue;

		double Time = 0.0;
		Obj->TryGetNumberField(TEXT("t"), Time);
		TArray<double> QPos;
		TArray<double> QVel;
		ReadDoubles(Obj, TEXT("qpos"), QPos);
		ReadDoubles(Obj, TEXT("qvel"), QVel);
		ApplyFrame(Time, QPos, QVel);
	}
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
