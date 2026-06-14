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
#include "Transport/RpcTransport.h"
#include "Dom/JsonObject.h"
#include "HAL/Event.h"
#include "HAL/PlatformProcess.h"
#include <atomic>
#include "ZmqRpcTransport.generated.h"

class FRunnableThread;

/**
 * @struct FMjStepRequest
 * @brief One Direct-mode step request, parsed from a client RPC and pushed to
 *        the physics-thread queue. Owns the per-articulation ctrl writes and
 *        the n_steps count.
 */
struct FMjStepRequest
{
    int32 NSteps = 1;
    /** prefix -> array of (actuator_name, value). Names are local (no prefix). */
    TMap<FString, TArray<TPair<FString, float>>> PerArticulationCtrl;
    /** Per-articulation control mode: "ue_controller" (default) or "raw". */
    TMap<FString, FString> PerArticulationControlMode;
    /** Per-articulation xfrc_applied: prefix -> body_name -> [fx,fy,fz,tx,ty,tz]. */
    TMap<FString, TMap<FString, TArray<double>>> PerArticulationXfrc;
    /** Echo'd request envelope for downstream reply building. */
    FString Op;
};

/**
 * @struct FMjDirectStepCommand
 * @brief Heap-allocated wrapper passed by raw pointer through the SPSC queue
 *        in Direct mode. The RPC thread enqueues, the physics-thread custom
 *        step handler dequeues, drains the request, and signals via FEvent.
 *        Captures observations inline so the reply can be built off the
 *        physics thread without re-touching d.
 */
struct FMjDirectStepCommand
{
    FMjStepRequest Request;
    /** Set true by the handler when mj_step has completed. */
    bool bDone = false;
    /** Observations captured under the engine's CallbackMutex. */
    TSharedPtr<FJsonObject> Observations;
    TSharedPtr<FJsonObject> Entities;
    double ResultTime = 0.0;
    int64  ResultStep = 0;
    /** Physics thread signals this when the step has completed. */
    FEvent* Completion = nullptr;

    ~FMjDirectStepCommand()
    {
        if (Completion)
        {
            FPlatformProcess::ReturnSynchEventToPool(Completion);
            Completion = nullptr;
        }
    }
};

/**
 * @struct FMjPushStateRequest
 * @brief One Puppet-mode push-state request. The client owns the integrator;
 *        UE writes qpos/qvel and calls mj_forward.
 */
struct FMjPushStateRequest
{
    TArray<double> QPos;
    TArray<double> QVel;
    TArray<double> Ctrl;     // optional informational ctrl
    bool bIncludeCtrl = false;
    double Time = 0.0;
    int32 NSteps = 1;        // informational only in puppet
};

/**
 * @class UURLabZmqRpcTransport
 * @brief ZMQ REQ/REP adapter. Owned by `UURLabBridgeServer`; binds a
 *        REP socket on `StepEndpoint` and runs a polling worker thread.
 *        Wire framing only — dispatcher lookup + encoding live on the base.
 */
UCLASS()
class URLAB_API UURLabZmqRpcTransport : public UURLabRpcTransport
{
    GENERATED_BODY()

public:
    UURLabZmqRpcTransport();

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ZMQ")
    FString StepEndpoint = TEXT("tcp://0.0.0.0:5559");

    /** Polling interval for the REP socket in milliseconds. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ZMQ")
    int32 PollTimeoutMs = 50;

    // --- UURLabRpcTransport contract ---
    virtual bool TransportInit() override;
    virtual void TransportShutdown() override;
    virtual FString GetTransportName() const override { return TEXT("zmq"); }
    /** ZMQ accepts every op; SHM is the only transport that refuses. */
    virtual bool AcceptsEditorOps() const override { return true; }

private:
    void* ZmqContext = nullptr;
    void* ZmqRep = nullptr;
    FRunnableThread* WorkerThread = nullptr;
    std::atomic<bool> bStop{false};
    bool bIsInitialized = false;

    /** Worker thread loop. Runs zmq_poll on the REP socket and forwards each
     *  parsed request to the dispatcher; sends the reply back to the wire. */
    void RunPollLoop();
};
