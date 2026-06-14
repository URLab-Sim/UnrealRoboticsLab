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
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
// trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

#pragma once

#include <atomic>

#include "CoreMinimal.h"
#include "Transport/PublishTransport.h"
#include "Transport/SnapshotPublisher.h"
#include "ZmqPublishTransport.generated.h"

class AAMjManager;
class AMjArticulation;
class UMjComponent;
class UMjTwistController;

/**
 * @class UURLabZmqPublishTransport
 * @brief ZMQ PUB transport broadcasting per-articulation telemetry +
 *        the `state/full` snapshot.
 *
 * Plain UObject deriving from UURLabPublishTransport, created via
 * `NewObject` + `SetOwningManager` + `TransportInit`. The game-thread
 * cache build runs lazily on the first PostStep via `AsyncTask` to the
 * game thread.
 */
UCLASS()
class URLAB_API UURLabZmqPublishTransport : public UURLabPublishTransport
	, public IMjSnapshotPublisher
{
	GENERATED_BODY()

public:
	UURLabZmqPublishTransport() = default;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ZMQ")
	FString ZmqEndpoint = "tcp://0.0.0.0:5555";

	/** Bridge-style ownership setter. Must be called before TransportInit
	 *  so the broadcaster can register with the manager's snapshot fan-out
	 *  and resolve articulations on the game thread. */
	void SetOwningManager(AAMjManager* InMgr);

	// UURLabPublishTransport contract.
	virtual bool TransportInit() override;
	virtual void TransportShutdown() override;
	virtual FString GetTransportName() const override { return TEXT("zmq-pub"); }
	virtual void Publish(const FString& Topic,
		const TArray<uint8>& Payload) override;

	// Per-step hook (Async / physics thread).
	virtual void PostStep(struct mjModel_* m, struct mjData_* d) override;

	// IMjSnapshotPublisher: route through to Publish("state/full", bytes)
	// so the manager's existing snapshot fan-out keeps working.
	virtual void PublishSnapshot(const TArray<uint8>& Bytes) override
	{
		Publish(TEXT("state/full"), Bytes);
	}

private:
	TWeakObjectPtr<AAMjManager> OwningManager;
	void* ZmqContext = nullptr;
	void* ZmqPublisher = nullptr;
	int32 FrameCounter = 0;
	bool bIsInitialized = false;

	/** Per-articulation snapshot built once on the game thread and read
	 *  repeatedly from the physics thread in PostStep. Iterating
	 *  OwnedComponents on the physics thread is unsafe (the game thread
	 *  can mutate it during actor BeginPlay — e.g. auto-created twist
	 *  controllers), and tripping the sparse-array range-for ensure
	 *  corrupts nearby heap state, producing seemingly-unrelated RHI
	 *  crashes further along. */
	struct FArticulationBroadcastRecord
	{
		AMjArticulation* Articulation = nullptr;
		FString ArticPrefix;
		TArray<UMjComponent*> TelemetryComponents;
		UMjTwistController* TwistCtrl = nullptr;
	};

	/** Populated once on the game thread. bCacheBuilt (with acquire/release
	 *  ordering) publishes visibility to the physics thread. No mid-play
	 *  refresh — broadcaster assumes articulations and their components are
	 *  stable across a single play session. */
	TArray<FArticulationBroadcastRecord> CachedRecords;
	std::atomic<bool> bCacheBuilt{false};
	std::atomic<bool> bCacheBuildScheduled{false};

	/** Schedule a one-shot AsyncTask(GameThread) to build the cache.
	 *  Idempotent: only the first call goes through. */
	void RequestGameThreadCacheBuild();

	/** Game-thread-only: enumerate articulations + components into CachedRecords. */
	void BuildBroadcastCacheGameThread();

	void InitZmqSocket();
	void ShutdownZmqSocket();
};
