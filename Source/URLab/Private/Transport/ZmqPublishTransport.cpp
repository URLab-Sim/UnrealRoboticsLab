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

#include "Transport/ZmqPublishTransport.h"
#if PLATFORM_WINDOWS
#include "Windows/AllowWindowsPlatformTypes.h"
#endif
#include "zmq.h"
#if PLATFORM_WINDOWS
#include "Windows/HideWindowsPlatformTypes.h"
#endif
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Components/MjComponent.h"
#include "MuJoCo/Input/MjTwistController.h"
#include "Bridge/MsgpackHelpers.h"
#include "Serialization/BufferArchive.h"
#include "Async/Async.h"
#include "Misc/ScopeExit.h"
#include "Utils/URLabLogging.h"

namespace
{
    // FString.Len() returns TCHAR count; the zmq frame needs UTF-8 byte
    // count. Non-ASCII topics (any multibyte character) diverge.
    inline int SendTopic(void* Socket, const FString& Topic, int Flags)
    {
        const FTCHARToUTF8 Utf8(*Topic);
        return zmq_send(Socket, Utf8.Get(), Utf8.Length(), Flags);
    }
}

void UURLabZmqPublishTransport::SetOwningManager(AAMjManager* InMgr)
{
	OwningManager = InMgr;
}

bool UURLabZmqPublishTransport::TransportInit()
{
	InitZmqSocket();
	if (!bIsInitialized) return false;

	if (AAMjManager* Mgr = OwningManager.Get())
	{
		Mgr->RegisterSnapshotPublisher(this, this);
	}
	return true;
}

void UURLabZmqPublishTransport::TransportShutdown()
{
	if (AAMjManager* Mgr = OwningManager.Get())
	{
		Mgr->UnregisterSnapshotPublisher(this);
	}
	// Make sure the physics thread stops reading our cache before tear-down.
	bCacheBuilt.store(false, std::memory_order_release);
	CachedRecords.Reset();
	ShutdownZmqSocket();
}

void UURLabZmqPublishTransport::InitZmqSocket()
{
	if (bIsInitialized) return;

	ZmqContext = zmq_ctx_new();
	ZmqPublisher = zmq_socket(ZmqContext, ZMQ_PUB);

	int rc = zmq_bind(ZmqPublisher, TCHAR_TO_UTF8(*ZmqEndpoint));
	if (rc == 0)
	{
		UE_LOG(LogURLabNet, Log, TEXT("UURLabZmqPublishTransport: bound ZMQ PUB at %s"), *ZmqEndpoint);
		bIsInitialized = true;
	}
	else
	{
		UE_LOG(LogURLabNet, Error, TEXT("UURLabZmqPublishTransport: failed to bind ZMQ at %s"), *ZmqEndpoint);
		if (GEngine)
		{
			GEngine->AddOnScreenDebugMessage(-1, 10.f, FColor::Red,
			    FString::Printf(TEXT("URLab: ZMQ bind failed on %s — check for port conflicts"), *ZmqEndpoint));
		}
	}
}

void UURLabZmqPublishTransport::ShutdownZmqSocket()
{
	if (!bIsInitialized) return;
	zmq_close(ZmqPublisher);
	zmq_ctx_term(ZmqContext);
	ZmqPublisher = nullptr;
	ZmqContext = nullptr;
	bIsInitialized = false;
}

void UURLabZmqPublishTransport::RequestGameThreadCacheBuild()
{
	bool Expected = false;
	if (!bCacheBuildScheduled.compare_exchange_strong(Expected, true,
	                                                  std::memory_order_acq_rel))
	{
		return;  // already scheduled
	}
	TWeakObjectPtr<UURLabZmqPublishTransport> WeakSelf(this);
	AsyncTask(ENamedThreads::GameThread, [WeakSelf]()
	{
		if (UURLabZmqPublishTransport* Self = WeakSelf.Get())
		{
			Self->BuildBroadcastCacheGameThread();
		}
	});
}

void UURLabZmqPublishTransport::BuildBroadcastCacheGameThread()
{
	// Always reset the scheduled flag on exit so a future call can
	// re-schedule (e.g. articulations were added after we ran).
	ON_SCOPE_EXIT{
		bCacheBuildScheduled.store(false, std::memory_order_release);
	};

	AAMjManager* Manager = OwningManager.Get();
	if (!Manager) return;

	TArray<AMjArticulation*> Articulations = Manager->GetAllArticulations();
	if (Articulations.Num() == 0) return;

	CachedRecords.Reset(Articulations.Num());
	for (AMjArticulation* Art : Articulations)
	{
		if (!Art) continue;

		FArticulationBroadcastRecord Rec;
		Rec.Articulation = Art;
		Rec.ArticPrefix  = Art->GetName();
		Art->GetComponents<UMjComponent>(Rec.TelemetryComponents);
		Rec.TwistCtrl    = Art->FindComponentByClass<UMjTwistController>();
		CachedRecords.Add(MoveTemp(Rec));
	}

	bCacheBuilt.store(true, std::memory_order_release);

	UE_LOG(LogURLabNet, Log,
		TEXT("UURLabZmqPublishTransport: built broadcast cache (%d articulations)."),
		CachedRecords.Num());
}

void UURLabZmqPublishTransport::PostStep(mjModel* m, mjData* d)
{
	static constexpr int32 kLogInterval = 500;
	bool bShouldLog = (FrameCounter++ % kLogInterval == 0);

	if (!bIsInitialized) return;

	// Single source of truth for "publishers paused" — flipped by
	// UURLabZmqRpcTransport on Direct / Puppet mode entry so we don't
	// double-write to the wire while the step server drives cadence.
	if (AAMjManager* Mgr = OwningManager.Get())
	{
		if (Mgr->bPublishersPaused.load(std::memory_order_acquire))
		{
			return;
		}
	}

	// Acquire-load: if the game thread hasn't published the cache yet,
	// schedule a build (idempotent) and skip this step. We DO NOT touch
	// AActor::OwnedComponents from this thread.
	if (!bCacheBuilt.load(std::memory_order_acquire))
	{
		RequestGameThreadCacheBuild();
		return;
	}

	int BroadcastCount = 0;
	if (bShouldLog)
	{
		UE_LOG(LogURLabNet, Verbose,
			TEXT("UURLabZmqPublishTransport PostStep: broadcasting %d cached articulations"),
			CachedRecords.Num());
	}

	for (const FArticulationBroadcastRecord& Rec : CachedRecords)
	{
		if (!Rec.Articulation) continue;

		for (UMjComponent* Comp : Rec.TelemetryComponents)
		{
			if (!Comp || Comp->bIsDefault) continue;

			FString TopicSuffix = Comp->GetTelemetryTopicName();
			if (TopicSuffix.IsEmpty()) continue;

			FString FullTopic = FString::Printf(TEXT("%s/%s"), *Rec.ArticPrefix, *TopicSuffix);

			FBufferArchive Payload;
			Comp->BuildBinaryPayload(Payload);

			if (Payload.Num() > 0)
			{
				SendTopic(ZmqPublisher, FullTopic, ZMQ_SNDMORE);
				zmq_send(ZmqPublisher, Payload.GetData(), Payload.Num(), 0);
				BroadcastCount++;
			}
		}

		if (Rec.TwistCtrl)
		{
			FVector Twist = Rec.TwistCtrl->GetTwist();
			FString TwistTopic = FString::Printf(TEXT("%s/twist"), *Rec.ArticPrefix);
			float TwistData[3] = { (float)Twist.X, (float)Twist.Y, (float)Twist.Z };
			SendTopic(ZmqPublisher, TwistTopic, ZMQ_SNDMORE);
			zmq_send(ZmqPublisher, TwistData, sizeof(TwistData), 0);
			BroadcastCount++;

			int32 Actions = Rec.TwistCtrl->GetActiveActions();
			if (Actions != 0)
			{
				FString ActionTopic = FString::Printf(TEXT("%s/actions"), *Rec.ArticPrefix);
				SendTopic(ZmqPublisher, ActionTopic, ZMQ_SNDMORE);
				zmq_send(ZmqPublisher, &Actions, sizeof(Actions), 0);
				BroadcastCount++;
			}
		}
	}

	// Non-articulation dynamic bodies (free-jointed props, heightfields, etc).
	if (AAMjManager* Mgr = OwningManager.Get())
	{
		const TArray<FMjEntityRecord>& Entities = Mgr->GetEntities();
		for (const FMjEntityRecord& Rec : Entities)
		{
			if (Rec.MjId < 0 || Rec.MjId >= m->nbody) continue;

			FString XposTopic = FString::Printf(TEXT("scene/%s/xpos"), *Rec.Name);
			SendTopic(ZmqPublisher, XposTopic, ZMQ_SNDMORE);
			zmq_send(ZmqPublisher, &d->xpos[Rec.MjId * 3], 3 * sizeof(mjtNum), 0);
			BroadcastCount++;

			FString XquatTopic = FString::Printf(TEXT("scene/%s/xquat"), *Rec.Name);
			SendTopic(ZmqPublisher, XquatTopic, ZMQ_SNDMORE);
			zmq_send(ZmqPublisher, &d->xquat[Rec.MjId * 4], 4 * sizeof(mjtNum), 0);
			BroadcastCount++;

			if (Rec.bHasFreeBase && m->body_jntnum && m->body_jntadr &&
			    m->jnt_type && m->jnt_qposadr && m->jnt_dofadr)
			{
				int FirstJnt = m->body_jntadr[Rec.MjId];
				int NumJnt   = m->body_jntnum[Rec.MjId];
				if (FirstJnt >= 0 && NumJnt > 0 && FirstJnt < m->njnt &&
				    m->jnt_type[FirstJnt] == mjJNT_FREE)
				{
					int QAddr = m->jnt_qposadr[FirstJnt];
					int VAddr = m->jnt_dofadr[FirstJnt];
					FString QposTopic = FString::Printf(TEXT("scene/%s/qpos"), *Rec.Name);
					SendTopic(ZmqPublisher, QposTopic, ZMQ_SNDMORE);
					zmq_send(ZmqPublisher, &d->qpos[QAddr], 7 * sizeof(mjtNum), 0);
					FString QvelTopic = FString::Printf(TEXT("scene/%s/qvel"), *Rec.Name);
					SendTopic(ZmqPublisher, QvelTopic, ZMQ_SNDMORE);
					zmq_send(ZmqPublisher, &d->qvel[VAddr], 6 * sizeof(mjtNum), 0);
					BroadcastCount += 2;
				}
			}
		}
	}

	// state/full snapshots are built once per step by AAMjManager and
	// fanned out via PublishSnapshot to every IMjSnapshotPublisher.
	if (bShouldLog && BroadcastCount == 0)
	{
		UE_LOG(LogURLabNet, Warning,
			TEXT("UURLabZmqPublishTransport: Found components but NONE produced a valid binary payload!"));
	}
	else if (bShouldLog)
	{
		UE_LOG(LogURLabNet, Verbose,
			TEXT("UURLabZmqPublishTransport: broadcast %d messages to ZMQ."), BroadcastCount);
	}
}

void UURLabZmqPublishTransport::Publish(const FString& Topic, const TArray<uint8>& Payload)
{
	if (!ZmqPublisher || Payload.Num() == 0) return;
	SendTopic(ZmqPublisher, Topic, ZMQ_SNDMORE);
	zmq_send(ZmqPublisher, Payload.GetData(), Payload.Num(), 0);
}
