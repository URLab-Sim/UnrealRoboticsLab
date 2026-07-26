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

// Pre-encode a topic to its UTF-8 wire bytes (no NUL terminator) once, at
// cache-build time, so the per-step publish can send them without a TCHAR->UTF8
// conversion on the physics thread.
inline void EncodeTopic(const FString& Topic, TArray<uint8>& Out)
{
	const FTCHARToUTF8 Utf8(*Topic);
	Out.SetNumUninitialized(Utf8.Length());
	if (Utf8.Length() > 0)
		FMemory::Memcpy(Out.GetData(), Utf8.Get(), Utf8.Length());
}

inline int SendTopicBytes(void* Socket, const TArray<uint8>& TopicUtf8, int Flags)
{
	return zmq_send(Socket, TopicUtf8.GetData(), TopicUtf8.Num(), Flags);
}
} // namespace

void UURLabZmqPublishTransport::SetOwningManager(AAMjManager* InMgr)
{
	OwningManager = InMgr;
}

bool UURLabZmqPublishTransport::TransportInit()
{
	InitZmqSocket();
	if (!bIsInitialized)
		return false;

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
	// bCacheBuilt=false makes a fresh PostStep bail early; CacheMutex waits out
	// any iteration already in flight (a detached worker may still be stepping).
	bCacheBuilt.store(false, std::memory_order_release);
	{
		FScopeLock CacheLock(&CacheMutex);
		CachedRecords.Reset();
		CachedEntities.Reset();
	}
	ShutdownZmqSocket();
}

void UURLabZmqPublishTransport::InitZmqSocket()
{
	if (bIsInitialized)
		return;

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
	if (!bIsInitialized)
		return;
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
		return; // already scheduled
	}
	TWeakObjectPtr<UURLabZmqPublishTransport> WeakSelf(this);
	AsyncTask(ENamedThreads::GameThread, [WeakSelf]() {
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
	ON_SCOPE_EXIT
	{
		bCacheBuildScheduled.store(false, std::memory_order_release);
	};

	AAMjManager* Manager = OwningManager.Get();
	if (!Manager)
		return;

	TArray<AMjArticulation*> Articulations = Manager->GetAllArticulations();
	if (Articulations.Num() == 0)
		return;

	// Assemble into locals with no lock held; the physics thread keeps reading
	// the previous cache meanwhile. Only the swap below is guarded.
	TArray<FArticulationBroadcastRecord> NewRecords;
	NewRecords.Reserve(Articulations.Num());
	for (AMjArticulation* Art : Articulations)
	{
		if (!Art)
			continue;

		const FString ArticPrefix = Art->GetName();

		FArticulationBroadcastRecord Rec;
		Rec.Articulation = Art;

		TArray<UMjComponent*> Components;
		Art->GetComponents<UMjComponent>(Components);
		for (UMjComponent* Comp : Components)
		{
			if (!Comp || Comp->bIsDefault)
				continue;
			const FString TopicSuffix = Comp->GetTelemetryTopicName();
			if (TopicSuffix.IsEmpty())
				continue;
			FBroadcastComponentEntry Entry;
			Entry.Component = Comp;
			EncodeTopic(FString::Printf(TEXT("%s/%s"), *ArticPrefix, *TopicSuffix), Entry.TopicUtf8);
			Rec.Components.Add(MoveTemp(Entry));
		}

		if (UMjTwistController* TwistCtrl = Art->FindComponentByClass<UMjTwistController>())
		{
			Rec.TwistCtrl = TwistCtrl;
			EncodeTopic(FString::Printf(TEXT("%s/twist"), *ArticPrefix), Rec.TwistTopicUtf8);
			EncodeTopic(FString::Printf(TEXT("%s/actions"), *ArticPrefix), Rec.ActionsTopicUtf8);
		}

		NewRecords.Add(MoveTemp(Rec));
	}

	// Non-articulation dynamic bodies (free-jointed props, heightfields). Pull
	// MjId + free-base flag from the manager's entity cache (built in
	// PostCompile) and pre-encode the scene topics; only the raw mjData reads
	// stay on the per-step path.
	const TArray<FMjEntityRecord>& Entities = Manager->GetEntities();
	TArray<FEntityBroadcastEntry> NewEntities;
	NewEntities.Reserve(Entities.Num());
	for (const FMjEntityRecord& Ent : Entities)
	{
		if (Ent.MjId < 0)
			continue;
		FEntityBroadcastEntry Out;
		Out.MjId = Ent.MjId;
		Out.bHasFreeBase = Ent.bHasFreeBase;
		EncodeTopic(FString::Printf(TEXT("scene/%s/xpos"), *Ent.Name), Out.XposTopicUtf8);
		EncodeTopic(FString::Printf(TEXT("scene/%s/xquat"), *Ent.Name), Out.XquatTopicUtf8);
		EncodeTopic(FString::Printf(TEXT("scene/%s/qpos"), *Ent.Name), Out.QposTopicUtf8);
		EncodeTopic(FString::Printf(TEXT("scene/%s/qvel"), *Ent.Name), Out.QvelTopicUtf8);
		NewEntities.Add(MoveTemp(Out));
	}

	const int32 NumRecords = NewRecords.Num();
	const int32 NumEntities = NewEntities.Num();
	{
		FScopeLock Lock(&CacheMutex);
		CachedRecords = MoveTemp(NewRecords);
		CachedEntities = MoveTemp(NewEntities);
	}

	bCacheBuilt.store(true, std::memory_order_release);

	UE_LOG(LogURLabNet, Log,
		TEXT("UURLabZmqPublishTransport: built broadcast cache (%d articulations, %d entities)."),
		NumRecords, NumEntities);
}

void UURLabZmqPublishTransport::PostStep(mjModel* m, mjData* d)
{
	static constexpr int32 kLogInterval = 500;
	bool bShouldLog = (FrameCounter++ % kLogInterval == 0);

	if (!bIsInitialized)
		return;

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

	// Hold CacheMutex across the iteration so a concurrent rebuild (game thread)
	// swaps the arrays only between steps, never mid-walk. Uncontended in the
	// steady state (rebuilds fire only on a registry change), so this stays off
	// the per-step cost that the pre-encoded topics + scratch buffer target.
	FScopeLock CacheLock(&CacheMutex);

	int BroadcastCount = 0;
	bool bStaleRef = false;
	if (bShouldLog)
	{
		UE_LOG(LogURLabNet, Verbose,
			TEXT("UURLabZmqPublishTransport PostStep: broadcasting %d cached articulations"),
			CachedRecords.Num());
	}

	for (const FArticulationBroadcastRecord& Rec : CachedRecords)
	{
		if (!Rec.Articulation.IsValid())
		{
			// Articulation despawned since the cache was built. Skip it and ask
			// the game thread to rebuild so the cache drops the dead record.
			bStaleRef = true;
			continue;
		}

		for (const FBroadcastComponentEntry& Entry : Rec.Components)
		{
			UMjComponent* Comp = Entry.Component.Get();
			if (!Comp)
			{
				bStaleRef = true;
				continue;
			}
			if (Comp->bIsDefault)
				continue;

			// Reuse the scratch buffer: Seek(0) rewinds the write cursor and the
			// serializer overwrites in place (growing only past the high-water
			// mark), so the steady state does not reallocate. Tell() is the byte
			// count written this pass; the array's Num() is the high-water mark.
			ScratchPayload.Seek(0);
			Comp->BuildBinaryPayload(ScratchPayload);
			const int64 PayloadLen = ScratchPayload.Tell();

			if (PayloadLen > 0)
			{
				SendTopicBytes(ZmqPublisher, Entry.TopicUtf8, ZMQ_SNDMORE);
				zmq_send(ZmqPublisher, ScratchPayload.GetData(), PayloadLen, 0);
				BroadcastCount++;
			}
		}

		if (UMjTwistController* TwistCtrl = Rec.TwistCtrl.Get())
		{
			FVector Twist = TwistCtrl->GetTwist();
			float TwistData[3] = {(float)Twist.X, (float)Twist.Y, (float)Twist.Z};
			SendTopicBytes(ZmqPublisher, Rec.TwistTopicUtf8, ZMQ_SNDMORE);
			zmq_send(ZmqPublisher, TwistData, sizeof(TwistData), 0);
			BroadcastCount++;

			int32 Actions = TwistCtrl->GetActiveActions();
			if (Actions != 0)
			{
				SendTopicBytes(ZmqPublisher, Rec.ActionsTopicUtf8, ZMQ_SNDMORE);
				zmq_send(ZmqPublisher, &Actions, sizeof(Actions), 0);
				BroadcastCount++;
			}
		}
	}

	// Non-articulation dynamic bodies (free-jointed props, heightfields, etc).
	for (const FEntityBroadcastEntry& Ent : CachedEntities)
	{
		if (Ent.MjId < 0 || Ent.MjId >= m->nbody)
			continue;

		SendTopicBytes(ZmqPublisher, Ent.XposTopicUtf8, ZMQ_SNDMORE);
		zmq_send(ZmqPublisher, &d->xpos[Ent.MjId * 3], 3 * sizeof(mjtNum), 0);
		BroadcastCount++;

		SendTopicBytes(ZmqPublisher, Ent.XquatTopicUtf8, ZMQ_SNDMORE);
		zmq_send(ZmqPublisher, &d->xquat[Ent.MjId * 4], 4 * sizeof(mjtNum), 0);
		BroadcastCount++;

		if (Ent.bHasFreeBase && m->body_jntnum && m->body_jntadr && m->jnt_type && m->jnt_qposadr && m->jnt_dofadr)
		{
			int FirstJnt = m->body_jntadr[Ent.MjId];
			int NumJnt = m->body_jntnum[Ent.MjId];
			if (FirstJnt >= 0 && NumJnt > 0 && FirstJnt < m->njnt && m->jnt_type[FirstJnt] == mjJNT_FREE)
			{
				int QAddr = m->jnt_qposadr[FirstJnt];
				int VAddr = m->jnt_dofadr[FirstJnt];
				SendTopicBytes(ZmqPublisher, Ent.QposTopicUtf8, ZMQ_SNDMORE);
				zmq_send(ZmqPublisher, &d->qpos[QAddr], 7 * sizeof(mjtNum), 0);
				SendTopicBytes(ZmqPublisher, Ent.QvelTopicUtf8, ZMQ_SNDMORE);
				zmq_send(ZmqPublisher, &d->qvel[VAddr], 6 * sizeof(mjtNum), 0);
				BroadcastCount += 2;
			}
		}
	}

	// A stale weak ref means the registry changed under us; rebuild once
	// (idempotent) so the cache re-syncs to the live set.
	if (bStaleRef)
		RequestGameThreadCacheBuild();

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
	if (!ZmqPublisher || Payload.Num() == 0)
		return;
	SendTopic(ZmqPublisher, Topic, ZMQ_SNDMORE);
	zmq_send(ZmqPublisher, Payload.GetData(), Payload.Num(), 0);
}
