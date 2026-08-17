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

#include "Bridge/RpcDispatcher.h"
#include "Bridge/RpcErrorCodes.h"
#include "Bridge/OpRegistry.h"
#include "Bridge/StepCommands.h"
#include "Bridge/MsgpackHelpers.h"
#include "State/MjStateCollector.h"
#include "State/MjMsgpackEncoder.h"
#include "State/MjCanonicalName.h"
#include "State/MjStateTypes.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Entity/MjControl.h"
#include "MuJoCo/Entity/MjControlIngress.h"
#include "MuJoCo/Entity/MjEntity.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Elements/MjActuatorRuntime.h"
#include "MuJoCo/Elements/MjSensorRuntime.h"
#include "MuJoCo/Elements/MjCamera.h"
#include "MuJoCo/Elements/MjJointRuntime.h"
#include "MuJoCo/Elements/MjBody.h"
#include "MuJoCo/Input/MjPerturbation.h"
#include "MuJoCo/Input/MjTwistController.h"
#include "Transport/NetworkManager.h"
#include "Transport/ShmPublishTransport.h"
#include "Transport/ShmRpcTransport.h"
#include "Transport/RpcTransport.h"
#include "Transport/ShmRegion.h" // FMjShmHeader (header_size in shm_rpc block)
#include "Bridge/BridgeServer.h"
#include "Replay/MjReplayManager.h"
#include "Kismet/GameplayStatics.h"
#include "Misc/Base64.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "Internationalization/Regex.h"
#include "HAL/FileManager.h"
#include "EngineUtils.h"
#include "Engine/World.h"
#include "Misc/Guid.h"
#include "Utils/URLabLogging.h"

namespace
{
/** Map pose source to wire-format string, matching the Python StepMode enum
 *  values. The wire tokens are the frozen client contract; the collapse onto
 *  EMjPoseSource must not change them. Mirror never reaches the active-mode
 *  paths, so it falls through to the "live" default rather than minting a token. */
FString StepModeToString(EMjPoseSource Mode)
{
	switch (Mode)
	{
		case EMjPoseSource::FreeRun:
			return TEXT("live");
		case EMjPoseSource::Stepped:
			return TEXT("direct");
		case EMjPoseSource::StatePushed:
			return TEXT("puppet");
		case EMjPoseSource::Mirror:
			break;
	}
	return TEXT("live");
}

bool StepModeFromString(const FString& Str, EMjPoseSource& OutMode)
{
	if (Str.Equals(TEXT("live"), ESearchCase::IgnoreCase) || Str.Equals(TEXT("streaming"), ESearchCase::IgnoreCase))
	{
		OutMode = EMjPoseSource::FreeRun;
		return true;
	}
	if (Str.Equals(TEXT("direct"), ESearchCase::IgnoreCase))
	{
		OutMode = EMjPoseSource::Stepped;
		return true;
	}
	if (Str.Equals(TEXT("puppet"), ESearchCase::IgnoreCase))
	{
		OutMode = EMjPoseSource::StatePushed;
		return true;
	}
	if (Str.Equals(TEXT("auto"), ESearchCase::IgnoreCase))
	{
		// "auto" is the client-picks promotion policy, not an axis value; it
		// resolves to FreeRun (matching the old Auto -> Live resolution).
		OutMode = EMjPoseSource::FreeRun;
		return true;
	}
	return false;
}

/** Write a client-pushed integration state (qpos/qvel/ctrl/time) into
 *  (m,d), recompute derived quantities, and fire OnPostStep. Shared by the
 *  puppet inline path and the puppet step handler. The caller must hold the
 *  engine's CallbackMutex. */
void ApplyPushedState(UMjPhysicsEngine* Engine, const FMjPushStateRequest& Push, mjModel* m, mjData* d)
{
	if (Push.QPos.Num() == m->nq)
		FMemory::Memcpy(d->qpos, Push.QPos.GetData(), m->nq * sizeof(mjtNum));
	if (Push.QVel.Num() == m->nv)
		FMemory::Memcpy(d->qvel, Push.QVel.GetData(), m->nv * sizeof(mjtNum));
	if (Push.bIncludeCtrl && Push.Ctrl.Num() == m->nu)
		FMemory::Memcpy(d->ctrl, Push.Ctrl.GetData(), m->nu * sizeof(mjtNum));
	d->time = Push.Time;
	mj_forward(m, d);
	if (Engine->OnPostStep)
		Engine->OnPostStep(m, d);
}

} // namespace

const FMjEntity* FURLabRpcDispatcher::ResolveEntityByWireKey(
	const UMjPhysicsEngine* Engine, const FString& WireKey)
{
	if (Engine == nullptr)
		return nullptr;
	for (const FMjEntity& Entity : Engine->GetEntityPartition())
	{
		if (Entity.Name.ToString() == WireKey || Entity.PublicName.ToString() == WireKey
			|| Entity.ActorId == WireKey)
			return &Entity;
	}
	return nullptr;
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleSetPaused(const TSharedPtr<FJsonObject>& Req)
{
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr || !Mgr->PhysicsEngine)
		return MakeError(URLabError::NotReady, TEXT("PhysicsEngine not initialised"));

	bool bPause = false;
	if (!Req->TryGetBoolField(TEXT("paused"), bPause))
		return MakeError(URLabError::MissingField, TEXT("set_paused requires 'paused' bool"));

	Mgr->PhysicsEngine->SetPaused(bPause);
	UE_LOG(LogURLabNet, Log, TEXT("FURLabRpcDispatcher: set_paused -> %s"),
		bPause ? TEXT("true") : TEXT("false"));

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("set_paused_ok"));
	Reply->SetBoolField(TEXT("paused"), Mgr->PhysicsEngine->bIsPaused);
	return Reply;
}

// =============================================================================
// step
// =============================================================================

// Parse the per_articulation control payload -- control_mode, positional ctrl
// array, named ctrl_map, and xfrc_applied -- into an FMjStepRequest. Shared by
// the live and direct step paths so both apply the full payload; the live
// branch previously parsed only the positional ctrl array and silently dropped
// ctrl_map / xfrc_applied.
static void ParseStepPerArticulation(const TSharedPtr<FJsonObject>& Req, FMjStepRequest& Out)
{
	const TSharedPtr<FJsonObject>* PerArt = nullptr;
	if (!Req->TryGetObjectField(TEXT("per_articulation"), PerArt) || !PerArt || !PerArt->IsValid())
	{
		return;
	}
	for (auto& Pair : (*PerArt)->Values)
	{
		const TSharedPtr<FJsonObject>* ArtObj = nullptr;
		if (!Pair.Value->TryGetObject(ArtObj) || !ArtObj || !ArtObj->IsValid())
			continue;

		// control_mode override
		FString CtlMode;
		if ((*ArtObj)->TryGetStringField(TEXT("control_mode"), CtlMode))
		{
			Out.PerArticulationControlMode.Add(Pair.Key, CtlMode);
		}

		// Positional ctrl array: kept positional and resolved against the compiled
		// model's actuator ids at apply time, so it needs no articulation here.
		const TArray<TSharedPtr<FJsonValue>>* CtrlList = nullptr;
		if ((*ArtObj)->TryGetArrayField(TEXT("ctrl"), CtrlList) && CtrlList)
		{
			TArray<double>& Positional = Out.PerArticulationCtrlPositional.FindOrAdd(Pair.Key);
			Positional.Reserve(CtrlList->Num());
			for (const TSharedPtr<FJsonValue>& V : *CtrlList)
				Positional.Add(V.IsValid() ? V->AsNumber() : 0.0);
		}

		// Named ctrl map alternative.
		const TSharedPtr<FJsonObject>* CtrlMap = nullptr;
		if ((*ArtObj)->TryGetObjectField(TEXT("ctrl_map"), CtrlMap) && CtrlMap && CtrlMap->IsValid())
		{
			for (auto& KV : (*CtrlMap)->Values)
			{
				Out.PerArticulationCtrl.FindOrAdd(Pair.Key).Add(
					{KV.Key, (float)KV.Value->AsNumber()});
			}
		}

		// xfrc_applied: { body_name: [fx,fy,fz,tx,ty,tz] }. Cleared after step.
		const TSharedPtr<FJsonObject>* XfrcMap = nullptr;
		if ((*ArtObj)->TryGetObjectField(TEXT("xfrc_applied"), XfrcMap) && XfrcMap && XfrcMap->IsValid())
		{
			TMap<FString, TArray<double>>& BodyMap = Out.PerArticulationXfrc.FindOrAdd(Pair.Key);
			for (auto& KV : (*XfrcMap)->Values)
			{
				const TArray<TSharedPtr<FJsonValue>>* Arr = nullptr;
				if (KV.Value->TryGetArray(Arr) && Arr && Arr->Num() == 6)
				{
					TArray<double>& Six = BodyMap.FindOrAdd(KV.Key);
					Six.SetNum(6);
					for (int i = 0; i < 6; ++i)
						Six[i] = (*Arr)[i]->AsNumber();
				}
			}
		}
	}
}

void FURLabRpcDispatcher::ParseStepCommon(const TSharedPtr<FJsonObject>& Req,
	AAMjManager* Mgr, FStepRequestCommon& Out) const
{
	// Per-step observations override: applies to THIS request only. Defaults to
	// the session level; the session default is only changed at hello, never
	// per-step, so a one-off `observations` field can't leak into later steps.
	Out.ObservationLevel = ActiveObservationLevel.load(std::memory_order_acquire);
	FString StepObs;
	if (Req->TryGetStringField(TEXT("observations"), StepObs))
	{
		if (StepObs.Equals(TEXT("minimal"), ESearchCase::IgnoreCase))
			Out.ObservationLevel = EObservationLevel::Minimal;
		else if (StepObs.Equals(TEXT("full"), ESearchCase::IgnoreCase))
			Out.ObservationLevel = EObservationLevel::Full;
		else if (StepObs.Equals(TEXT("standard"), ESearchCase::IgnoreCase))
			Out.ObservationLevel = EObservationLevel::Standard;
	}

	// Parse include_cameras. Per-camera value forms:
	//   "latest" / "sync"   -> latest available frame from the camera's history
	//   <number>            -> the frame showing post-step state >= that frame_id
	//   { "frame_id": N }   -> same as <number>
	// Or include_cameras: true  -> all registered cameras, latest.
	// Retrieval is non-blocking: a frame that isn't ready yet is omitted and
	// the client retries (or passes the step's frame_id to wait client-side).
	{
		const TSharedPtr<FJsonObject>* CamObj = nullptr;
		if (Req->TryGetObjectField(TEXT("include_cameras"), CamObj) && CamObj && CamObj->IsValid())
		{
			for (const auto& Kv : (*CamObj)->Values)
			{
				if (!Kv.Value.IsValid())
					continue;
				FString Mode;
				double Num = 0.0;
				const TSharedPtr<FJsonObject>* Obj = nullptr;
				if (Kv.Value->TryGetString(Mode))
				{
					ECameraInclude E = Mode.Equals(TEXT("sync"), ESearchCase::IgnoreCase)
										 ? ECameraInclude::Sync
										 : ECameraInclude::Latest;
					Out.CameraSpec.Add(Kv.Key, E);
				}
				else if (Kv.Value->TryGetNumber(Num))
				{
					Out.CameraSpec.Add(Kv.Key, ECameraInclude::Latest);
					if (Num > 0.0)
						Out.CameraMinFrameIds.Add(Kv.Key, static_cast<uint64>(Num));
				}
				else if (Kv.Value->TryGetObject(Obj) && Obj && Obj->IsValid())
				{
					Out.CameraSpec.Add(Kv.Key, ECameraInclude::Latest);
					double Fid = 0.0;
					if ((*Obj)->TryGetNumberField(TEXT("frame_id"), Fid) && Fid > 0.0)
						Out.CameraMinFrameIds.Add(Kv.Key, static_cast<uint64>(Fid));
				}
			}
		}
		else
		{
			bool bAll = false;
			if (Req->TryGetBoolField(TEXT("include_cameras"), bAll) && bAll && Mgr)
			{
				// CollectCameras is the single enumeration owner: render-view cameras
				// plus global cameras, keyed by canonical name, articulations alive or
				// gone. Kept in sync with BuildCameraNameMap so a camera the client asks
				// for here resolves there.
				TArray<UMjCamera*> Cameras;
				Mgr->CollectCameras(Cameras);
				for (UMjCamera* C : Cameras)
				{
					if (C)
						Out.CameraSpec.Add(C->GetCanonicalName(), ECameraInclude::Latest);
				}
			}
		}
	}

	// wait_cameras: block the reply server-side until the requested cameras
	// have the frame this step produced, instead of the client polling with a
	// min_frame_id and eating a round trip per miss.
	Req->TryGetBoolField(TEXT("wait_cameras"), Out.bWaitCameras);
	{
		double T = 0.0;
		if (Req->TryGetNumberField(TEXT("camera_timeout_ms"), T) && T > 0.0)
			Out.CameraTimeoutMs = static_cast<int32>(T);
	}
	// render: "sync" drives an immediate capture and waits for the fresh frame
	// (frame_id == this step). render: "async" kicks the capture but returns the
	// most-recently-completed frame instead of waiting, so back-to-back requests
	// overlap render and readback for higher throughput at the cost of a
	// one-step-stale frame. Both are for eval, not interactive use.
	{
		FString R;
		if (Req->TryGetStringField(TEXT("render"), R))
		{
			Out.bRenderSync = R.Equals(TEXT("sync"), ESearchCase::IgnoreCase);
			Out.bRenderAsync = R.Equals(TEXT("async"), ESearchCase::IgnoreCase);
		}
	}
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleStep(const TSharedPtr<FJsonObject>& Req)
{
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr || !Mgr->PhysicsEngine || !Mgr->PhysicsEngine->m_model)
	{
		return MakeError(URLabError::NotReady, TEXT("PhysicsEngine not initialised"));
	}

	// Gate step-carried control: any articulation whose payload carries
	// ctrl / ctrl_map / xfrc_applied must be owned by this request's control
	// source. Observation-only steps (no control payload) are never gated.
	{
		const TSharedPtr<FJsonObject>* PerArt = nullptr;
		if (Req->TryGetObjectField(TEXT("per_articulation"), PerArt) && PerArt && PerArt->IsValid())
		{
			const FString Source = ResolveControlSource(Req);
			for (const auto& Pair : (*PerArt)->Values)
			{
				const TSharedPtr<FJsonObject>* ArtObj = nullptr;
				if (!Pair.Value->TryGetObject(ArtObj) || !ArtObj || !ArtObj->IsValid())
					continue;
				const bool bCarriesControl =
					(*ArtObj)->HasField(TEXT("ctrl")) || (*ArtObj)->HasField(TEXT("ctrl_map")) || (*ArtObj)->HasField(TEXT("xfrc_applied"));
				if (!bCarriesControl)
					continue;

				const FMjEntity* Entity = ResolveEntityByWireKey(Mgr->PhysicsEngine, Pair.Key);
				const FName Key = Entity ? Entity->Name : FName(*Pair.Key);
				FString CurrentOwner;
				if (Mgr->PhysicsEngine->CheckControlWrite(Key, Source, CurrentOwner)
					!= FMjControlOwnership::EWriteCheck::Ok)
				{
					TSharedPtr<FJsonObject> Err = MakeError(TEXT("not_control_owner"),
						FString::Printf(TEXT("%s owned by %s"), *Key.ToString(), *CurrentOwner));
					Err->SetStringField(TEXT("owner"), CurrentOwner);
					return Err;
				}
			}
		}
	}

	FStepRequestCommon Common;
	ParseStepCommon(Req, Mgr, Common);

	// The pose source is the single selector for the step body. Load it once
	// (a plain atomic; no strategy object to keep alive across a concurrent
	// set_mode) and switch. A set_mode landing mid-step just picks the newer
	// source on the next request.
	switch (ActiveStepMode.load(std::memory_order_acquire))
	{
		case EMjPoseSource::Stepped:
			return StepStepped(Req, Common);
		case EMjPoseSource::StatePushed:
			return StepStatePushed(Req, Common);
		case EMjPoseSource::FreeRun:
		case EMjPoseSource::Mirror:
		default:
			return StepFreeRun(Req, Common);
	}
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::BuildStepReply(const FMjStateSnapshot& Snapshot,
	uint64 FrameId, EObservationLevel Level)
{
	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("step_ok"));
	Reply->SetNumberField(TEXT("time"), Snapshot.Time);
	Reply->SetNumberField(TEXT("step"), static_cast<double>(Snapshot.Step));
	AppendClockFields(Reply, Snapshot.Time);
	Reply->SetNumberField(TEXT("frame_id"), static_cast<double>(FrameId));
	Reply->SetObjectField(TEXT("arts"), FMjMsgpackEncoder::EncodeArts(Snapshot, Level));
	Reply->SetObjectField(TEXT("scene"), FMjMsgpackEncoder::EncodeScene(Snapshot));
	return Reply;
}

void FURLabRpcDispatcher::AppendCamerasBlock(TSharedPtr<FJsonObject>& Reply, AAMjManager* Mgr,
	const TMap<FString, ECameraInclude>& CameraSpec,
	const TMap<FString, uint64>& CameraMinFrameIds)
{
	if (!Reply.IsValid() || CameraSpec.Num() == 0)
		return;
	TSharedPtr<FJsonObject> Cams = BuildCamerasBlock(Mgr, CameraSpec, CameraMinFrameIds);
	if (Cams.IsValid() && Cams->Values.Num() > 0)
		Reply->SetObjectField(TEXT("cameras"), Cams);
}

void FURLabRpcDispatcher::ApplyStepCtrl(AAMjManager* Manager, const FMjStepRequest& Req,
	mjModel* m, mjData* d)
{
	if (!Manager)
		return;

	// Control resolves against the compiled model + entity partition, not the articulation: the
	// entity supplies the lease key (its compiled-prefix stem) and the actuator id slice, and every
	// setpoint routes through the one control ingress. The pre-step drain copies each touched
	// setpoint into d->ctrl per sub-step.
	UMjPhysicsEngine* Engine = Manager->PhysicsEngine;
	IMjControlIngress* Ingress = Engine ? Engine->GetControlIngress() : nullptr;
	if (Ingress && m)
	{
		const bool bRaw = Engine->IsRawModelInstalled();

		// Positional ctrl: write each value onto the entity's actuator ids in ascending mj-id order.
		for (const TPair<FString, TArray<double>>& Pair : Req.PerArticulationCtrlPositional)
		{
			const FMjEntity* Entity = ResolveEntityByWireKey(Engine, Pair.Key);
			if (!Entity)
				continue;
			const int32 Count = FMath::Min(Pair.Value.Num(), Entity->ActuatorIds.Num());
			for (int32 i = 0; i < Count; ++i)
				Ingress->WriteCtrl(Entity->Name, Entity->ActuatorIds[i], Pair.Value[i]);
		}

		// Named ctrl map: resolve each actuator by its compiled name (entity prefix + local name,
		// falling back to the name as given for a full or global name) against the model.
		for (const TPair<FString, TArray<TPair<FString, double>>>& Pair : Req.PerArticulationCtrl)
		{
			const FMjEntity* Entity = ResolveEntityByWireKey(Engine, Pair.Key);
			if (!Entity && !bRaw)
				continue;
			const FName EntityName = Entity ? Entity->Name : FName(*Pair.Key);
			const FString Prefix = (Entity && !Entity->Name.IsNone())
				? (Entity->Name.ToString() + TEXT("_")) : FString();
			for (const TPair<FString, double>& KV : Pair.Value)
			{
				int32 Id = -1;
				if (!Prefix.IsEmpty())
					Id = mj_name2id(m, mjOBJ_ACTUATOR, TCHAR_TO_ANSI(*(Prefix + KV.Key)));
				if (Id < 0)
					Id = mj_name2id(m, mjOBJ_ACTUATOR, TCHAR_TO_ANSI(*KV.Key));
				if (Id >= 0)
					Ingress->WriteCtrl(EntityName, Id, KV.Value);
			}
		}
	}

	// xfrc_applied writes: per_articulation -> body_name -> 6-vec.
	// MuJoCo clears d->xfrc_applied on every mj_step, so this is a one-shot
	// impulse for the next mj_step n_steps loop. Body name lookup tries both
	// the local (no-prefix) form and the prefixed full name. Interactive input is a
	// capability: an instance with AcceptInput off ignores pushed wrenches.
	if (m && d && Manager && Manager->HasCapability(EMjCapability::AcceptInput))
	{
		for (auto& APair : Req.PerArticulationXfrc)
		{
			FString ArtPrefix = APair.Key + TEXT("_");
			for (auto& BPair : APair.Value)
			{
				if (BPair.Value.Num() != 6)
					continue;
				FString FullName = ArtPrefix + BPair.Key;
				int Bid = mj_name2id(m, mjOBJ_BODY, TCHAR_TO_UTF8(*FullName));
				if (Bid < 0)
					Bid = mj_name2id(m, mjOBJ_BODY, TCHAR_TO_UTF8(*BPair.Key));
				if (Bid < 0 || Bid >= m->nbody)
					continue;
				for (int i = 0; i < 6; ++i)
					d->xfrc_applied[6 * Bid + i] = (mjtNum)BPair.Value[i];
			}
		}
	}
}

// =============================================================================
// reset / set_mode
// =============================================================================

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleReset(const TSharedPtr<FJsonObject>& Req)
{
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr || !Mgr->PhysicsEngine || !Mgr->PhysicsEngine->m_model)
	{
		return MakeError(URLabError::NotReady, TEXT("PhysicsEngine not initialised"));
	}

	int32 SeedVal = 0;
	if (Req->TryGetNumberField(TEXT("seed"), SeedVal))
	{
		Mgr->Seed = SeedVal;
		// Modern mjOption has no "seed" field; mj_step is deterministic and
		// doesn't depend on a stored seed (random elements come from
		// user-set noise inputs, not an integrator-internal RNG). The seed
		// is recorded on the manager so any RNG used by client code or by
		// the recording layer can mirror it for reproducibility. UE itself
		// does not reseed the integrator here.
	}

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	{
		FScopeLock Lock(&Mgr->PhysicsEngine->CallbackMutex);

		// Fetch model/data under the lock: a concurrent CompileModel frees them
		// under CallbackMutex.
		mjModel* m = Mgr->PhysicsEngine->GetModel();
		mjData* d = Mgr->PhysicsEngine->GetData();
		if (!m || !d)
			return MakeError(URLabError::NotReady, TEXT("PhysicsEngine not initialised"));

		FString KfName;
		if (Req->TryGetStringField(TEXT("keyframe_name"), KfName) && !KfName.IsEmpty())
		{
			int Kid = mj_name2id(m, mjOBJ_KEY, TCHAR_TO_UTF8(*KfName));
			if (Kid < 0)
				return MakeError(URLabError::UnknownKeyframe, KfName);
			mj_resetDataKeyframe(m, d, Kid);
		}
		else
		{
			mj_resetData(m, d);
		}

		// Per-articulation qpos overrides (joint-name -> value).
		const TSharedPtr<FJsonObject>* PerArt = nullptr;
		if (Req->TryGetObjectField(TEXT("per_articulation_qpos"), PerArt) && PerArt && PerArt->IsValid())
		{
			for (auto& APair : (*PerArt)->Values)
			{
				const FMjEntity* Entity = ResolveEntityByWireKey(Mgr->PhysicsEngine, APair.Key);
				if (!Entity)
					continue;
				const TSharedPtr<FJsonObject>* QObj = nullptr;
				if (!APair.Value->TryGetObject(QObj) || !QObj || !QObj->IsValid())
					continue;

				const FString Prefix = Entity->Name.IsNone()
					? FString() : (Entity->Name.ToString() + TEXT("_"));
				for (auto& JPair : (*QObj)->Values)
				{
					FString FullName = Prefix + JPair.Key;
					int Jid = mj_name2id(m, mjOBJ_JOINT, TCHAR_TO_UTF8(*FullName));
					if (Jid < 0)
						Jid = mj_name2id(m, mjOBJ_JOINT, TCHAR_TO_UTF8(*JPair.Key));
					if (Jid < 0)
						continue;
					int QAddr = m->jnt_qposadr[Jid];
					d->qpos[QAddr] = (mjtNum)JPair.Value->AsNumber();
				}
			}
		}
		mj_forward(m, d);

		StepCounter.store(0, std::memory_order_relaxed);

		// Build the reply fields under the lock: the worker wakes on its idle
		// timeout and can mutate d, tearing reads done after the lock releases.
		Reply->SetStringField(TEXT("op"), TEXT("reset_ok"));
		Reply->SetNumberField(TEXT("time"), d->time);
		Reply->SetNumberField(TEXT("step"), 0);
		AppendClockFields(Reply, d->time);
		const FMjStateSnapshot& Snap = Mgr->GetStateCollector().Collect(m, d, 0);
		Reply->SetObjectField(TEXT("arts"),
			FMjMsgpackEncoder::EncodeArts(Snap, ActiveObservationLevel.load(std::memory_order_acquire)));
	}
	return Reply;
}

// Run mj_forward (kinematics + dynamics, no integration) and return
// observations. Lets a client write qpos / qvel then read consistent
// derived state (xpos, sensors, contacts, ...) without advancing time.
TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleForward(const TSharedPtr<FJsonObject>& /*Req*/)
{
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr || !Mgr->PhysicsEngine || !Mgr->PhysicsEngine->m_model)
	{
		return MakeError(URLabError::NotReady, TEXT("PhysicsEngine not initialised"));
	}

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	{
		FScopeLock Lock(&Mgr->PhysicsEngine->CallbackMutex);
		// Fetch model/data under the lock: a concurrent CompileModel frees them
		// under CallbackMutex.
		mjModel* m = Mgr->PhysicsEngine->GetModel();
		mjData* d = Mgr->PhysicsEngine->GetData();
		if (!m || !d)
			return MakeError(URLabError::NotReady, TEXT("PhysicsEngine not initialised"));
		mj_forward(m, d);

		// Build the reply fields under the lock so the worker's idle-timeout
		// drain can't tear a read done after the lock releases.
		Reply->SetStringField(TEXT("op"), TEXT("forward_ok"));
		Reply->SetNumberField(TEXT("time"), d->time);
		Reply->SetNumberField(TEXT("step"), StepCounter.load(std::memory_order_relaxed));
		AppendClockFields(Reply, d->time);
		const FMjStateSnapshot& Snap =
			Mgr->GetStateCollector().Collect(m, d, StepCounter.load(std::memory_order_relaxed));
		Reply->SetObjectField(TEXT("arts"),
			FMjMsgpackEncoder::EncodeArts(Snap, ActiveObservationLevel.load(std::memory_order_acquire)));
	}
	return Reply;
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleSetMode(const TSharedPtr<FJsonObject>& Req)
{
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr)
		return MakeError(URLabError::NotReady, TEXT("Manager missing"));

	if (Mgr->bPinStepMode)
	{
		return MakeError(URLabError::ModeLockedByServer,
			FString::Printf(TEXT("Project pinned StepMode to %s"), *StepModeToString(Mgr->StepMode)));
	}

	FString ModeStr;
	if (!Req->TryGetStringField(TEXT("mode"), ModeStr))
		return MakeError(URLabError::MissingField, TEXT("set_mode requires 'mode'"));

	EMjPoseSource NewMode;
	if (!StepModeFromString(ModeStr, NewMode))
		return MakeError(URLabError::BadMode, FString::Printf(TEXT("Unknown mode '%s'"), *ModeStr));

	EMjPoseSource Prev = ActiveStepMode;
	SetActiveStepMode(NewMode);

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("set_mode_ok"));
	Reply->SetStringField(TEXT("previous_mode"), StepModeToString(Prev));
	Reply->SetStringField(TEXT("current_mode"), StepModeToString(ActiveStepMode));
	return Reply;
}

// The per-pose-source step bodies. Selected by ActiveStepMode in HandleStep;
// the mode-enter side effects (publisher pause, engine pose source, direct
// handler) live in EnterPoseSource. Camera publishers stream in every mode, so
// the mode-enter path clears FCameraZmqWorker::bPublishersPaused separately.

// UE drives its own physics: apply ctrl and read current state. n_steps is
// ignored (UE steps at its own rate). Cameras are served against the latest
// render snapshot id since free-run has no discrete stepped frame.
TSharedPtr<FJsonObject> FURLabRpcDispatcher::StepFreeRun(
	const TSharedPtr<FJsonObject>& Req, const FStepRequestCommon& Common)
{
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr || !Mgr->PhysicsEngine)
		return MakeError(URLabError::NotReady, TEXT("PhysicsEngine not initialised"));
	UMjPhysicsEngine* Engine = Mgr->PhysicsEngine;

	FMjStepRequest TmpReq;
	ParseStepPerArticulation(Req, TmpReq);

	TSharedPtr<FJsonObject> Reply;
	uint64 FrameId = 0;
	{
		// Fetch model/data AFTER acquiring CallbackMutex: a concurrent
		// CompileModel frees them under this lock, so a fetch before it
		// would dangle.
		FScopeLock Lock(&Engine->CallbackMutex);
		mjModel* m = Engine->GetModel();
		mjData* d = Engine->GetData();
		if (!m || !d)
			return MakeError(URLabError::NotReady, TEXT("PhysicsEngine not initialised"));
		ApplyStepCtrl(Mgr, TmpReq, m, d);
		// Most recently published snapshot id (UE's autonomous physics owns
		// stepping here), so the client can wait for a streamed frame >= this.
		FrameId = Engine->GetRenderFrameId();
		const int64 StepIdx = StepCounter.load(std::memory_order_relaxed);
		const FMjStateSnapshot& Snap = Mgr->GetStateCollector().Collect(m, d, StepIdx);
		Reply = BuildStepReply(Snap, FrameId, Common.ObservationLevel);
	}

	TMap<FString, uint64> CameraMinFrameIds = Common.CameraMinFrameIds;
	if (Common.bRenderSync && Common.CameraSpec.Num() > 0)
		RenderCamerasSync(Mgr, Common.CameraSpec, FrameId, Common.CameraTimeoutMs, CameraMinFrameIds);
	else if (Common.bRenderAsync && Common.CameraSpec.Num() > 0)
		RenderCamerasSync(Mgr, Common.CameraSpec, 0, Common.CameraTimeoutMs, CameraMinFrameIds, /*bWait=*/false);
	else if (Common.bWaitCameras && Common.CameraSpec.Num() > 0)
		WaitForCameraFrames(Mgr, Common.CameraSpec, FrameId, Common.CameraTimeoutMs, CameraMinFrameIds);

	AppendCamerasBlock(Reply, Mgr, Common.CameraSpec, CameraMinFrameIds);
	return Reply;
}

// Enqueue an FMjDirectStepCommand for the physics worker's step handler and
// wait for completion. If the worker isn't running (test / editor path), pump
// the handler inline under the engine lock.
TSharedPtr<FJsonObject> FURLabRpcDispatcher::StepStepped(
	const TSharedPtr<FJsonObject>& Req, const FStepRequestCommon& Common)
{
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr || !Mgr->PhysicsEngine)
		return MakeError(URLabError::NotReady, TEXT("PhysicsEngine not initialised"));
	UMjPhysicsEngine* Engine = Mgr->PhysicsEngine;

	TSharedPtr<FMjDirectStepCommand> Cmd = MakeShared<FMjDirectStepCommand>();
	int32 NSteps = 1;
	Req->TryGetNumberField(TEXT("n_steps"), NSteps);
	Cmd->Request.NSteps = NSteps > 0 ? NSteps : 1;
	Cmd->ObservationLevel = Common.ObservationLevel;
	ParseStepPerArticulation(Req, Cmd->Request);

	Cmd->Completion = FPlatformProcess::GetSynchEventFromPool(true);

	const bool bWorkerRunning = Engine->bWorkerRunning.load(std::memory_order_acquire);
	StepQueue.Enqueue(Cmd);
	if (Engine->StepRequestEvent)
		Engine->StepRequestEvent->Trigger();

	if (!bWorkerRunning)
	{
		// Test / editor path: pump the handler synchronously so we don't
		// block forever waiting for an engine that isn't ticking. The handler
		// mutates mjData, so hold the engine lock the handler contract
		// requires (the worker path already runs it under CallbackMutex).
		if (Engine->CustomStepHandler)
		{
			FScopeLock Lock(&Engine->CallbackMutex);
			Engine->CustomStepHandler(Engine->GetModel(), Engine->GetData());
		}
	}

	// 5-second hard cap so a wedged engine returns an error rather than
	// wedging the RPC thread. Polled in 50ms slices so the bDraining flag
	// (set when the bridge is being stopped) can short-circuit the wait.
	bool bSignaled = false;
	{
		const double Deadline = FPlatformTime::Seconds() + 5.0;
		while (FPlatformTime::Seconds() < Deadline)
		{
			if (bDraining.load(std::memory_order_acquire))
				break;
			if (Cmd->Completion->Wait(FTimespan::FromMilliseconds(50)))
			{
				bSignaled = true;
				break;
			}
		}
	}

	if (bSignaled && Cmd->bDone)
	{
		TMap<FString, uint64> CameraMinFrameIds = Common.CameraMinFrameIds;
		if (Common.bRenderSync && Common.CameraSpec.Num() > 0)
			RenderCamerasSync(Mgr, Common.CameraSpec, Cmd->ResultFrameId, Common.CameraTimeoutMs, CameraMinFrameIds);
		else if (Common.bRenderAsync && Common.CameraSpec.Num() > 0)
			RenderCamerasSync(Mgr, Common.CameraSpec, 0, Common.CameraTimeoutMs, CameraMinFrameIds, /*bWait=*/false);
		else if (Common.bWaitCameras && Common.CameraSpec.Num() > 0)
			WaitForCameraFrames(Mgr, Common.CameraSpec, Cmd->ResultFrameId, Common.CameraTimeoutMs, CameraMinFrameIds);
		// The base reply (arts/scene/time/step/frame_id) was built on the
		// physics thread from the state IR while the just-stepped mjData was
		// valid; append any requested cameras here.
		AppendCamerasBlock(Cmd->Reply, Mgr, Common.CameraSpec, CameraMinFrameIds);
		return Cmd->Reply;
	}

	// We stop waiting but the command is still queued. Mark it abandoned so
	// the handler discards it instead of stepping physics for a request the
	// client already saw fail (and may retry) -- otherwise the step executes
	// twice.
	Cmd->bAbandoned.store(true, std::memory_order_release);
	if (bDraining.load(std::memory_order_acquire))
		return MakeError(URLabError::ShuttingDown,
			TEXT("Bridge stopping; Direct-mode step abandoned"));
	return MakeError(URLabError::StepTimeout,
		TEXT("Direct-mode step did not complete within 5s"));
}

// Client owns the integrator: write the pushed qpos/qvel/ctrl/time into
// mjData, run mj_forward, and return the derived state.
TSharedPtr<FJsonObject> FURLabRpcDispatcher::StepStatePushed(
	const TSharedPtr<FJsonObject>& Req, const FStepRequestCommon& Common)
{
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr || !Mgr->PhysicsEngine)
		return MakeError(URLabError::NotReady, TEXT("PhysicsEngine not initialised"));
	UMjPhysicsEngine* Engine = Mgr->PhysicsEngine;

	FMjPushStateRequest Push;
	const TArray<TSharedPtr<FJsonValue>>* QPosArr = nullptr;
	const TArray<TSharedPtr<FJsonValue>>* QVelArr = nullptr;
	const TArray<TSharedPtr<FJsonValue>>* CtrlArr = nullptr;
	if (Req->TryGetArrayField(TEXT("qpos"), QPosArr))
	{
		Push.QPos.Reserve(QPosArr->Num());
		for (auto& V : *QPosArr)
			Push.QPos.Add(V->AsNumber());
	}
	if (Req->TryGetArrayField(TEXT("qvel"), QVelArr))
	{
		Push.QVel.Reserve(QVelArr->Num());
		for (auto& V : *QVelArr)
			Push.QVel.Add(V->AsNumber());
	}
	if (Req->TryGetArrayField(TEXT("ctrl"), CtrlArr))
	{
		Push.bIncludeCtrl = true;
		Push.Ctrl.Reserve(CtrlArr->Num());
		for (auto& V : *CtrlArr)
			Push.Ctrl.Add(V->AsNumber());
	}
	double TimeVal = 0.0;
	Req->TryGetNumberField(TEXT("time"), TimeVal);
	Push.Time = TimeVal;

	TSharedPtr<FJsonObject> Reply;
	uint64 PostFrameId = 0;
	{
		// Fetch model/data AFTER the lock: a concurrent CompileModel frees
		// them under CallbackMutex.
		FScopeLock Lock(&Engine->CallbackMutex);
		mjModel* m = Engine->GetModel();
		mjData* d = Engine->GetData();
		if (!m || !d)
			return MakeError(URLabError::NotReady, TEXT("PhysicsEngine not initialised"));
		ApplyPushedState(Engine, Push, m, d);

		// Publish the just-pushed pose to the render snapshot now, while we
		// still hold CallbackMutex (lock order CallbackMutex ->
		// RenderStateMutex). Without this the snapshot only refreshes on the
		// physics loop's idle timeout, so cameras and any render consumer
		// would lag the pushed state. The synchronous camera path below
		// relies on this snapshot being current.
		Engine->PushRenderState();

		// Build the reply from the state IR while still holding the lock. The
		// puppet-mode worker wakes on its idle timeout and can mutate d
		// (mocap/wrench drain), which would tear a read done after release.
		PostFrameId = Engine->GetRenderFrameId();
		const int64 StepIdx = StepCounter.fetch_add(1, std::memory_order_relaxed) + 1;
		const FMjStateSnapshot& Snap = Mgr->GetStateCollector().Collect(m, d, StepIdx);
		Reply = BuildStepReply(Snap, PostFrameId, Common.ObservationLevel);
	}

	// frame_id is the post-step state id: the client passes it back as a
	// camera frame_id to fetch the image showing this exact step's state.
	TMap<FString, uint64> CameraMinFrameIds = Common.CameraMinFrameIds;
	if (Common.bRenderSync && Common.CameraSpec.Num() > 0)
		RenderCamerasSync(Mgr, Common.CameraSpec, PostFrameId, Common.CameraTimeoutMs, CameraMinFrameIds);
	else if (Common.bRenderAsync && Common.CameraSpec.Num() > 0)
		RenderCamerasSync(Mgr, Common.CameraSpec, 0, Common.CameraTimeoutMs, CameraMinFrameIds, /*bWait=*/false);
	else if (Common.bWaitCameras && Common.CameraSpec.Num() > 0)
		WaitForCameraFrames(Mgr, Common.CameraSpec, PostFrameId, Common.CameraTimeoutMs, CameraMinFrameIds);

	AppendCamerasBlock(Reply, Mgr, Common.CameraSpec, CameraMinFrameIds);
	// Puppet-mode perturbation: include the latest sample so the client can
	// apply the editor click-drag widget's force to its own MjData.
	if (Mgr->Perturbation)
	{
		FMjPerturbationSample Sample = Mgr->Perturbation->GetLatestPerturbationSample();
		if (Sample.BodyId > 0)
		{
			TSharedPtr<FJsonObject> Pert = MakeShared<FJsonObject>();
			Pert->SetNumberField(TEXT("body_id"), Sample.BodyId);
			Pert->SetNumberField(TEXT("version"), Sample.Version);
			TArray<TSharedPtr<FJsonValue>> Six;
			for (int i = 0; i < 6; ++i)
				Six.Add(MakeShared<FJsonValueNumber>(Sample.Xfrc[i]));
			Pert->SetArrayField(TEXT("xfrc"), Six);
			Reply->SetObjectField(TEXT("perturbation"), Pert);
		}
	}
	return Reply;
}

void FURLabRpcDispatcher::EnterPoseSource(EMjPoseSource Mode, AAMjManager& Mgr)
{
	// The one place EMjPoseSource selects the clock behaviour. Client-driven
	// sources (Stepped / StatePushed) pause the state+ctrl publishers and push
	// their resolved source to the engine; only Stepped installs a custom step
	// handler. FreeRun and Mirror run UE's autonomous clock (Mirror is not an
	// engine clock, so it resolves to FreeRun). Camera publishers stream in
	// every mode and are cleared separately by the caller. Callers uninstall any
	// prior direct handler first and hold DispatchMutex.
	switch (Mode)
	{
		case EMjPoseSource::Stepped:
			Mgr.bPublishersPaused.store(true, std::memory_order_release);
			if (Mgr.PhysicsEngine)
				Mgr.PhysicsEngine->SetPoseSource(EMjPoseSource::Stepped);
			InstallDirectHandler();
			break;
		case EMjPoseSource::StatePushed:
			Mgr.bPublishersPaused.store(true, std::memory_order_release);
			if (Mgr.PhysicsEngine)
				Mgr.PhysicsEngine->SetPoseSource(EMjPoseSource::StatePushed);
			break;
		case EMjPoseSource::FreeRun:
		case EMjPoseSource::Mirror:
			Mgr.bPublishersPaused.store(false, std::memory_order_release);
			if (Mgr.PhysicsEngine)
				Mgr.PhysicsEngine->SetPoseSource(EMjPoseSource::FreeRun);
			break;
	}
}

void FURLabRpcDispatcher::SetActiveStepMode(EMjPoseSource Mode)
{
	// Serialises install/uninstall side effects against concurrent set_mode
	// calls; Dispatch releases DispatchMutex before handlers.
	FScopeLock Lock(&DispatchMutex);

	const EMjPoseSource CurMode = ActiveStepMode.load(std::memory_order_acquire);
	if (Mode == CurMode)
		return;

	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr)
		return;

	// Clear the previous source's direct handler (no-op unless it was Stepped),
	// then enter the new one.
	UninstallDirectHandler();
	DrainQueues();

	ActiveStepMode.store(Mode, std::memory_order_release);
	// Camera publishers stream in every mode (frames are decoupled from the
	// step reply), so they are never paused by mode.
	FCameraZmqWorker::bPublishersPaused.store(false, std::memory_order_release);

	EnterPoseSource(Mode, *Mgr);

	UE_LOG(LogURLabNet, Log, TEXT("FURLabRpcDispatcher: step mode -> %s"),
		*StepModeToString(Mode));
}

void FURLabRpcDispatcher::ReapplyActiveStepMode()
{
	FScopeLock Lock(&DispatchMutex);
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr)
		return;
	// Uninstall before re-entering so the direct handler's install guard
	// (bDirectHandlerInstalled) doesn't skip reinstalling onto the fresh engine
	// handler slot after a recompile.
	UninstallDirectHandler();
	EnterPoseSource(ActiveStepMode.load(std::memory_order_acquire), *Mgr);
}

void FURLabRpcDispatcher::InstallDirectHandler()
{
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr || !Mgr->PhysicsEngine)
		return;
	if (bDirectHandlerInstalled)
		return;

	UMjPhysicsEngine* Engine = Mgr->PhysicsEngine;
	DirectStepHandler = [this, Engine, Mgr](mjModel* m, mjData* d) -> bool {
		// Only the physics-engine async worker thread runs this handler,
		// so d is exclusively owned for its duration.
		TSharedPtr<FMjDirectStepCommand> Cmd;
		if (!StepQueue.Dequeue(Cmd) || !Cmd.IsValid())
			return false; // idle wake, no step requested — no advance

		// The RPC thread gave up on this command (timeout / draining) before we
		// got to it. Discard without stepping so a request the client already saw
		// fail does not also advance physics here (double step).
		if (Cmd->bAbandoned.load(std::memory_order_acquire))
			return false;

		ApplyStepCtrl(Mgr, Cmd->Request, m, d);

		for (int32 i = 0; i < Cmd->Request.NSteps; ++i)
		{
			Engine->DrainControlIntoData(m, d);
			mj_step(m, d);
			if (Engine->OnPostStep)
				Engine->OnPostStep(m, d);
		}
		Cmd->ResultTime = d->time;
		Cmd->ResultStep = StepCounter.fetch_add(Cmd->Request.NSteps, std::memory_order_relaxed)
						+ Cmd->Request.NSteps;
		// Publish the just-stepped state to the render snapshot now, while this
		// handler still owns d (it runs inside the engine's CallbackMutex), and
		// capture the resulting frame_id for the reply. Without this the RPC
		// thread would read GetRenderFrameId() after we Trigger() below but
		// before the physics loop's own PushRenderState() runs, returning the
		// previous step's id and breaking camera frame association.
		Engine->PushRenderState();
		Engine->bRenderStatePublishedThisStep = true; // loop tail must not republish
		Cmd->ResultFrameId = Engine->GetRenderFrameId();
		// Build the base reply (arts/scene/time/step/frame_id) from the state IR
		// here, where the just-stepped mjData is valid and the persistent snapshot
		// buffer is not racing another Collect (all callers hold CallbackMutex).
		const FMjStateSnapshot& Snap = Mgr->GetStateCollector().Collect(m, d, Cmd->ResultStep);
		Cmd->Reply = BuildStepReply(Snap, Cmd->ResultFrameId, Cmd->ObservationLevel);
		Cmd->bDone = true;
		if (Cmd->Completion)
			Cmd->Completion->Trigger();
		return true;
	};
	Engine->SetCustomStepHandler(DirectStepHandler);
	bDirectHandlerInstalled = true;
}

void FURLabRpcDispatcher::UninstallDirectHandler()
{
	if (!bDirectHandlerInstalled)
		return;
	if (AAMjManager* Mgr = OwnerMgr.Get())
	{
		if (Mgr->PhysicsEngine)
			Mgr->PhysicsEngine->ClearCustomStepHandler();
	}
	bDirectHandlerInstalled = false;
	DirectStepHandler = nullptr;
}
