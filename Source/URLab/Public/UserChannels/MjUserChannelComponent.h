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

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "State/MjStateProducer.h"
#include "State/MjStateTypes.h"
#include "MjUserChannelComponent.generated.h"

class AAMjManager;

/**
 * The single authoring surface for user-declared payload channels, for Blueprint
 * and convenience C++ alike. Drop it on an actor and call a Publish* node; the
 * value flows out over every enabled byte transport (ZMQ PUB, SHM ring, RPC step
 * replies) with no per-transport work, because the channel lives in the state IR.
 *
 * Threading. Publish* runs on the game thread (or any thread) and writes a
 * double-mailbox TMap under a light lock. DescribeState / DescribeSceneState run
 * on the physics thread and copy the latest mailbox values into the IR under the
 * same lock. Values are sample-and-hold: physics steps between publishes re-emit
 * the last value.
 *
 * Scope by attachment. On an AMjArticulation actor the channels land in that
 * art's block (arts/<art>/user); on any other actor they land in the scene block
 * (top-level user). The collector resolves this at cache-rebuild time, so this
 * component implements both producer entry points and fills whichever the
 * collector hands it.
 *
 * Spatial kinds convert UE space (cm, left-handed) to raw MuJoCo SI (metres,
 * wxyz) in the publish path when bConvertFromUESpace is set, keeping the IR
 * consistent with every other pose in the snapshot.
 */
UCLASS(ClassGroup = (URLab), meta = (BlueprintSpawnableComponent))
class URLAB_API UMjUserChannelComponent : public UActorComponent, public IMjStateProducer
{
	GENERATED_BODY()

public:
	UMjUserChannelComponent();

	// --- Output: publish a value into a named channel (game thread typical) ---

	UFUNCTION(BlueprintCallable, Category = "URLab|User Channels")
	void PublishBool(FName Channel, bool bValue);

	UFUNCTION(BlueprintCallable, Category = "URLab|User Channels")
	void PublishInt(FName Channel, int64 Value);

	UFUNCTION(BlueprintCallable, Category = "URLab|User Channels")
	void PublishFloat(FName Channel, double Value);

	UFUNCTION(BlueprintCallable, Category = "URLab|User Channels")
	void PublishVector(FName Channel, FVector Value, bool bConvertFromUESpace = true);

	UFUNCTION(BlueprintCallable, Category = "URLab|User Channels")
	void PublishQuat(FName Channel, FQuat Value, bool bConvertFromUESpace = true);

	UFUNCTION(BlueprintCallable, Category = "URLab|User Channels")
	void PublishTransform(FName Channel, FTransform Value, bool bConvertFromUESpace = true);

	UFUNCTION(BlueprintCallable, Category = "URLab|User Channels")
	void PublishFloatArray(FName Channel, const TArray<double>& Values);

	UFUNCTION(BlueprintCallable, Category = "URLab|User Channels")
	void PublishString(FName Channel, const FString& Value);

	/** Publish a Struct-kind channel from pre-packed msgpack-map bytes. The
	 *  encoder splices the map verbatim into the snapshot. This is the C++ entry
	 *  for the Struct kind; the wildcard-pin Blueprint PublishStruct (reflection
	 *  packing via FJsonObjectConverter) is the P8 follow-up. */
	void PublishStructBytes(FName Channel, const TArray<uint8>& PackedMsgpackMap);

	// --- IMjStateProducer: physics thread, under the engine CallbackMutex ---
	virtual void DescribeState(FMjArticulationState& Out) const override;
	virtual void DescribeSceneState(FMjStateSnapshot& Out) const override;

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
	/** Sanitize a raw channel name once into a canonical FName (msgpack key and,
	 *  later, ROS topic segment are the same string on every transport). */
	static FName MakeChannelName(FName Raw);

	/** Store a channel into the mailbox; marks the producer cache dirty when the
	 *  channel set changes (new name or kind change) so consumers rebuild. */
	void StoreChannel(FMjUserChannel&& Channel);

	/** Resolve (and cache) the owning MuJoCo manager. */
	AAMjManager* ResolveManager();

	mutable FCriticalSection MailboxMutex;
	TMap<FName, FMjUserChannel> Mailbox;
	TWeakObjectPtr<AAMjManager> CachedManager;
};
