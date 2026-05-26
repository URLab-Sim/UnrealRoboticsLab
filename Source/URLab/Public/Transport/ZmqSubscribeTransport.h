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

#pragma once

#include "CoreMinimal.h"
#include "Transport/SubscribeTransport.h"
#include "ZmqSubscribeTransport.generated.h"

class AAMjManager;
class UMjActuator;

/**
 * @class UURLabZmqSubscribeTransport
 * @brief ZMQ SUB transport draining inbound control / xfrc messages on
 *        each PreStep, plus a small PUB channel that ships per-actuator
 *        metadata for client-side display.
 *
 * UObject deriving from `UURLabSubscribeTransport`. Manager creates
 * via `NewObject` + `SetOwningManager` + `TransportInit`; per-step
 * PreStep is driven by the manager's PreStepCallback iterating
 * `ManagerOwnedSubscribeTransports`.
 */
UCLASS()
class URLAB_API UURLabZmqSubscribeTransport : public UURLabSubscribeTransport
{
	GENERATED_BODY()

public:
	UURLabZmqSubscribeTransport() = default;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ZMQ")
	FString ControlEndpoint = "tcp://0.0.0.0:5556";

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ZMQ")
	FString InfoEndpoint = "tcp://0.0.0.0:5557";

	void SetOwningManager(AAMjManager* InMgr);

	// UURLabSubscribeTransport contract.
	virtual bool TransportInit() override;
	virtual void TransportShutdown() override;
	virtual FString GetTransportName() const override { return TEXT("zmq-sub"); }
	/** No-op: this subscriber dispatches messages internally on PreStep
	 *  (legacy single-channel control + xfrc shape). The Subscribe API
	 *  is reserved for future ROS-style topic registrations. */
	virtual void Subscribe(const FString& /*Topic*/, FOnMessage /*Callback*/) override {}

	// Per-step (Async / physics thread).
	virtual void PreStep(struct mjModel_* m, struct mjData_* d) override;

private:
	TWeakObjectPtr<AAMjManager> OwningManager;
	void* ZmqContext = nullptr;
	void* ControlSubscriber = nullptr;
	void* InfoPublisher = nullptr;
	bool bIsInitialized = false;

	int InfoBroadcastCounter = 0;
	int TotalStepCount = 0;
	TMap<FString, int> ActuatorCache;
	TMap<int32, UMjActuator*> ActuatorComponentCache;
	bool bCacheBuilt = false;

	int32 ControlLogCounter = 0;

	void InitZmqSocket();
	void ShutdownZmqSocket();
	void BuildCache(struct mjModel_* m);
	void BroadcastInfo(struct mjModel_* m);
};
