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
#include "State/MjObservationLevel.h"
#include "Dom/JsonObject.h"
#include "HAL/Event.h"
#include "HAL/PlatformProcess.h"
#include <atomic>

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
	TMap<FString, TArray<TPair<FString, double>>> PerArticulationCtrl;
	/** wire key -> positional ctrl values, indexed in the entity's ascending mj-id
	 *  actuator order. Resolved against the compiled model at apply time. */
	TMap<FString, TArray<double>> PerArticulationCtrlPositional;
	/** Per-articulation control mode: "ue_controller" (default) or "raw". */
	TMap<FString, FString> PerArticulationControlMode;
	/** Per-articulation xfrc_applied: prefix -> body_name -> [fx,fy,fz,tx,ty,tz]. */
	TMap<FString, TMap<FString, TArray<double>>> PerArticulationXfrc;
	/** Echo'd request envelope for downstream reply building. */
	FString Op;
};

/**
 * @struct FMjDirectStepCommand
 * @brief Heap-allocated command passed as a shared pointer through the SPSC
 *        queue in Direct mode. The RPC thread enqueues, the physics-thread
 *        custom step handler dequeues, drains the request, and signals via FEvent.
 *        Captures observations inline so the reply can be built off the
 *        physics thread without re-touching d.
 */
struct FMjDirectStepCommand
{
	FMjStepRequest Request;
	/** Set true by the handler when mj_step has completed. */
	bool bDone = false;
	/** Set true by the RPC thread if it gives up (timeout / draining) before the
	 *  handler ran this command. The handler checks it after dequeue and discards
	 *  the command instead of stepping, so a timed-out request the client will
	 *  retry does not also execute here -- avoiding a double physics step. */
	std::atomic<bool> bAbandoned{false};
	/** Request-scoped observation verbosity. Threaded from the step request so
	 *  the worker builds this command's observations at the level THIS step
	 *  asked for, not whatever session level is current when the handler runs. */
	EObservationLevel ObservationLevel = EObservationLevel::Standard;
	/** Base step reply (op/time/step/clock/frame_id/arts/scene) built by the
	 *  handler under the engine's CallbackMutex from the state IR, so the reply's
	 *  observations reflect the just-stepped mjData. The RPC thread appends any
	 *  requested cameras before returning it. */
	TSharedPtr<FJsonObject> Reply;
	double ResultTime = 0.0;
	int64 ResultStep = 0;
	/** Post-step render-snapshot id, captured by the handler under
	 *  CallbackMutex right after the step. The reply returns this as the
	 *  camera frame_id; reading the engine's live counter from the RPC thread
	 *  would race the physics loop and yield the previous step's id. */
	uint64 ResultFrameId = 0;
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
	TArray<double> Ctrl; // optional informational ctrl
	bool bIncludeCtrl = false;
	double Time = 0.0;
	int32 NSteps = 1; // informational only in puppet
};
