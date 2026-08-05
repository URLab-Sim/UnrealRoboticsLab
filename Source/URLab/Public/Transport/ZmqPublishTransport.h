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
#include "Transport/PublishTransport.h"
#include "Transport/SnapshotPublisher.h"
#include "ZmqPublishTransport.generated.h"

class AAMjManager;

/**
 * @class UURLabZmqPublishTransport
 * @brief ZMQ PUB transport broadcasting the `state/full` snapshot.
 *
 * Plain UObject deriving from UURLabPublishTransport, created via
 * `NewObject` + `SetOwningManager` + `TransportInit`. It registers as an
 * IMjSnapshotPublisher; the manager's post-step callback builds + encodes
 * the snapshot once and fans the bytes out via PublishSnapshot.
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

	// IMjSnapshotPublisher: route through to Publish("state/full", bytes)
	// so the manager's snapshot fan-out delivers to the wire.
	virtual void PublishSnapshot(const TArray<uint8>& Bytes) override
	{
		Publish(TEXT("state/full"), Bytes);
	}

private:
	TWeakObjectPtr<AAMjManager> OwningManager;
	void* ZmqContext = nullptr;
	void* ZmqPublisher = nullptr;
	bool bIsInitialized = false;

	void InitZmqSocket();
	void ShutdownZmqSocket();
};
