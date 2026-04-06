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
#include "MuJoCo/Net/MjZmqComponent.h"
#include "ZmqSensorBroadcaster.generated.h"

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class URLAB_API UZmqSensorBroadcaster : public UMjZmqComponent
{
	GENERATED_BODY()

public:	
	UZmqSensorBroadcaster();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	// ZMQ Settings
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ZMQ")
	FString ZmqEndpoint = "tcp://*:5555";

	// Called from MuJoCo Async Thread
	virtual void InitZmq() override;
	virtual void ShutdownZmq() override;
	virtual void PostStep(mjModel* m, mjData* d) override;
    
private:
	void* ZmqContext = nullptr;
	void* ZmqPublisher = nullptr;
	int32 FrameCounter = 0;
};
