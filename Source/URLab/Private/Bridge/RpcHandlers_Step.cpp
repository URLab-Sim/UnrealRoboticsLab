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
#include "Bridge/OpRegistry.h"
#include "Transport/ZmqRpcTransport.h"
#include "Bridge/MsgpackHelpers.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Components/Actuators/MjActuator.h"
#include "MuJoCo/Components/Sensors/MjSensor.h"
#include "MuJoCo/Components/Sensors/MjCamera.h"
#include "MuJoCo/Components/Joints/MjJoint.h"
#include "MuJoCo/Components/Bodies/MjBody.h"
#include "MuJoCo/Components/Controllers/MjArticulationController.h"
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
/** Map enum to wire-format string, matching the Python StepMode enum values. */
FString StepModeToString(EStepMode Mode)
{
	switch (Mode)
	{
		case EStepMode::Live:
			return TEXT("live");
		case EStepMode::Direct:
			return TEXT("direct");
		case EStepMode::Puppet:
			return TEXT("puppet");
		case EStepMode::Auto:
			return TEXT("auto");
	}
	return TEXT("live");
}

bool StepModeFromString(const FString& Str, EStepMode& OutMode)
{
	if (Str.Equals(TEXT("live"), ESearchCase::IgnoreCase) || Str.Equals(TEXT("streaming"), ESearchCase::IgnoreCase))
	{
		OutMode = EStepMode::Live;
		return true;
	}
	if (Str.Equals(TEXT("direct"), ESearchCase::IgnoreCase))
	{
		OutMode = EStepMode::Direct;
		return true;
	}
	if (Str.Equals(TEXT("puppet"), ESearchCase::IgnoreCase))
	{
		OutMode = EStepMode::Puppet;
		return true;
	}
	if (Str.Equals(TEXT("auto"), ESearchCase::IgnoreCase))
	{
		OutMode = EStepMode::Auto;
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

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleSetPaused(const TSharedPtr<FJsonObject>& Req)
{
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr || !Mgr->PhysicsEngine)
		return MakeError(TEXT("not_ready"), TEXT("PhysicsEngine not initialised"));

	bool bPause = false;
	if (!Req->TryGetBoolField(TEXT("paused"), bPause))
		return MakeError(TEXT("missing_field"), TEXT("set_paused requires 'paused' bool"));

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
static void ParseStepPerArticulation(const TSharedPtr<FJsonObject>& Req,
	AAMjManager* Mgr, FMjStepRequest& Out)
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

		// Positional ctrl array: indexed in articulation actuator order.
		const TArray<TSharedPtr<FJsonValue>>* CtrlList = nullptr;
		if ((*ArtObj)->TryGetArrayField(TEXT("ctrl"), CtrlList) && CtrlList)
		{
			if (AMjArticulation* Art = Cast<AMjArticulation>(Mgr->GetArticulation(Pair.Key)))
			{
				TArray<UMjActuator*> Acts = Art->GetActuators();
				for (int32 i = 0; i < CtrlList->Num() && i < Acts.Num(); ++i)
				{
					UMjActuator* A = Acts[i];
					if (!A)
						continue;
					FString LocalName = A->GetMjName();
					FString Prefix = Art->GetName() + TEXT("_");
					if (LocalName.StartsWith(Prefix))
						LocalName = LocalName.Mid(Prefix.Len());
					Out.PerArticulationCtrl.FindOrAdd(Pair.Key).Add(
						{LocalName, (float)(*CtrlList)[i]->AsNumber()});
				}
			}
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
				auto AddCamera = [&Out](UMjCamera* C) {
					if (!C || C->bIsDefault)
						return;
					Out.CameraSpec.Add(C->GetCanonicalName(), ECameraInclude::Latest);
				};
				for (AMjArticulation* Art : Mgr->GetAllArticulations())
				{
					if (!Art)
						continue;
					TArray<UMjCamera*> Cams;
					Art->GetComponents<UMjCamera>(Cams);
					for (UMjCamera* C : Cams)
						AddCamera(C);
				}
				// Global / manager-level cameras: components attached directly to
				// the manager actor rather than an articulation. The per-art walk
				// alone dropped these, so `include_cameras: true` covered only
				// robot-mounted cameras.
				TArray<UMjCamera*> GlobalCams;
				Mgr->GetComponents<UMjCamera>(GlobalCams);
				for (UMjCamera* C : GlobalCams)
					AddCamera(C);
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
		return MakeError(TEXT("not_ready"), TEXT("PhysicsEngine not initialised"));
	}

	FStepRequestCommon Common;
	ParseStepCommon(Req, Mgr, Common);

	// Snapshot the strategy under DispatchMutex, then dispatch outside it. The
	// TSharedPtr copy keeps the strategy alive even if a concurrent set_mode
	// swaps CurrentStepStrategy mid-step; strategies are stateless, so an
	// in-flight step finishing on the prior strategy is correct.
	TSharedPtr<FStepModeStrategy> Strategy;
	{
		FScopeLock Lock(&DispatchMutex);
		Strategy = CurrentStepStrategy;
	}
	if (!Strategy)
		return MakeError(TEXT("not_ready"), TEXT("no active step strategy"));
	return Strategy->HandleStep(*this, Req, Common);
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::BuildStepReply(double TimeSec, int64 StepIdx, uint64 FrameId,
	const TSharedPtr<FJsonObject>& Observations,
	const TSharedPtr<FJsonObject>& Entities,
	AAMjManager* Mgr,
	const TMap<FString, ECameraInclude>& CameraSpec,
	const TMap<FString, uint64>& CameraMinFrameIds)
{
	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("step_ok"));
	Reply->SetNumberField(TEXT("time"), TimeSec);
	Reply->SetNumberField(TEXT("step"), static_cast<double>(StepIdx));
	AppendClockFields(Reply, TimeSec);
	Reply->SetNumberField(TEXT("frame_id"), static_cast<double>(FrameId));
	if (Observations.IsValid())
		Reply->SetObjectField(TEXT("per_articulation"), Observations);
	if (Entities.IsValid())
		Reply->SetObjectField(TEXT("entities"), Entities);
	if (CameraSpec.Num() > 0)
	{
		TSharedPtr<FJsonObject> Cams = BuildCamerasBlock(Mgr, CameraSpec, CameraMinFrameIds);
		if (Cams.IsValid() && Cams->Values.Num() > 0)
			Reply->SetObjectField(TEXT("cameras"), Cams);
	}
	return Reply;
}

void FURLabRpcDispatcher::ApplyStepCtrl(AAMjManager* Manager, const FMjStepRequest& Req,
	mjModel* m, mjData* d)
{
	if (!Manager)
		return;
	for (auto& Pair : Req.PerArticulationCtrl)
	{
		AMjArticulation* Art = Manager->GetArticulation(Pair.Key);
		if (!Art)
			continue;

		const FString Prefix = Art->GetName() + TEXT("_");
		TMap<FString, UMjActuator*> ByName;
		ByName.Reserve(Art->GetActuators().Num() * 2);
		for (UMjActuator* A : Art->GetActuators())
		{
			if (!A)
				continue;
			FString FullName = A->GetMjName();
			FString Local = FullName.StartsWith(Prefix) ? FullName.Mid(Prefix.Len()) : FullName;
			ByName.Add(Local, A);
			ByName.Add(FullName, A);
		}

		// Stage to actuator NetworkValue; AMjArticulation::ApplyControls
		// copies it into d->ctrl every sub-step. Writing d->ctrl directly
		// would be overwritten on the next sub-step.
		for (const TPair<FString, float>& KV : Pair.Value)
		{
			UMjActuator** Found = ByName.Find(KV.Key);
			if (!Found || !*Found)
				continue;
			(*Found)->SetNetworkControl(KV.Value);
		}
	}

	// xfrc_applied writes: per_articulation -> body_name -> 6-vec.
	// MuJoCo clears d->xfrc_applied on every mj_step, so this is a one-shot
	// impulse for the next mj_step n_steps loop. Body name lookup tries both
	// the local (no-prefix) form and the prefixed full name.
	if (m && d)
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

TSharedPtr<FJsonObject> FURLabRpcDispatcher::BuildStepObservations(AAMjManager* Manager, mjModel* m, mjData* d,
	EObservationLevel Level)
{
	TSharedPtr<FJsonObject> PerArt = MakeShared<FJsonObject>();
	if (!Manager || !m || !d)
		return PerArt;

	const bool bWantStandard = (Level == EObservationLevel::Standard) || (Level == EObservationLevel::Full);
	const bool bWantFull = (Level == EObservationLevel::Full);

	for (AMjArticulation* Art : Manager->GetAllArticulations())
	{
		if (!Art)
			continue;
		TSharedPtr<FJsonObject> ArtObj = MakeShared<FJsonObject>();

		// qpos / qvel — present at every level. Joints are emitted in the
		// discovery order that GetJoints() returns, which matches the MjModel
		// jnt_id order at compile time. Per-joint qpos/qvel slot widths follow
		// jnt_type. Wire-side consumers should always rebuild full m.nq / m.nv
		// arrays from per-articulation slices in this same order.
		TArray<UMjJoint*> Joints = Art->GetJoints();
		TArray<TSharedPtr<FJsonValue>> QPos;
		TArray<TSharedPtr<FJsonValue>> QVel;
		for (UMjJoint* J : Joints)
		{
			if (!J)
				continue;
			int32 Id = J->GetMjID();
			if (Id < 0 || Id >= m->njnt)
				continue;
			int QAddr = m->jnt_qposadr[Id];
			int VAddr = m->jnt_dofadr[Id];
			int QSize = 1, VSize = 1;
			switch (m->jnt_type[Id])
			{
				case mjJNT_FREE:
					QSize = 7;
					VSize = 6;
					break;
				case mjJNT_BALL:
					QSize = 4;
					VSize = 3;
					break;
				case mjJNT_SLIDE:
				case mjJNT_HINGE:
					QSize = 1;
					VSize = 1;
					break;
			}
			for (int i = 0; i < QSize; ++i)
				QPos.Add(MakeShared<FJsonValueNumber>(d->qpos[QAddr + i]));
			for (int i = 0; i < VSize; ++i)
				QVel.Add(MakeShared<FJsonValueNumber>(d->qvel[VAddr + i]));
		}
		ArtObj->SetArrayField(TEXT("qpos"), QPos);
		ArtObj->SetArrayField(TEXT("qvel"), QVel);

		if (bWantStandard)
		{
			// ctrl positional array, same order as GetActuators().
			TArray<TSharedPtr<FJsonValue>> Ctrl;
			TArray<TSharedPtr<FJsonValue>> Act;
			for (UMjActuator* A : Art->GetActuators())
			{
				if (!A)
					continue;
				int32 Id = A->GetMjID();
				if (Id < 0 || Id >= m->nu)
					continue;
				Ctrl.Add(MakeShared<FJsonValueNumber>(d->ctrl[Id]));
				// Each actuator's "act" slot, if it has one (intvelocity, muscle, ...)
				int ActAddr = m->actuator_actadr ? m->actuator_actadr[Id] : -1;
				if (ActAddr >= 0 && ActAddr < m->na)
					Act.Add(MakeShared<FJsonValueNumber>(d->act[ActAddr]));
				else
					Act.Add(MakeShared<FJsonValueNumber>(0.0));
			}
			ArtObj->SetArrayField(TEXT("ctrl"), Ctrl);
			ArtObj->SetArrayField(TEXT("act"), Act);

			// sensors by name — use sensor MjID + dim.
			TSharedPtr<FJsonObject> Sensors = MakeShared<FJsonObject>();
			TArray<UMjSensor*> SensorComponents;
			Art->GetComponents<UMjSensor>(SensorComponents);
			FString Prefix = Art->GetName() + TEXT("_");
			for (UMjSensor* S : SensorComponents)
			{
				if (!S)
					continue;
				int32 Sid = S->GetMjID();
				if (Sid < 0 || Sid >= m->nsensor)
					continue;
				int Adr = m->sensor_adr[Sid];
				int Dim = m->sensor_dim[Sid];
				if (Adr < 0 || Dim <= 0 || (Adr + Dim) > m->nsensordata)
					continue;
				TArray<TSharedPtr<FJsonValue>> Vals;
				for (int i = 0; i < Dim; ++i)
					Vals.Add(MakeShared<FJsonValueNumber>(d->sensordata[Adr + i]));
				FString LocalName = S->GetMjName();
				if (LocalName.StartsWith(Prefix))
					LocalName = LocalName.Mid(Prefix.Len());
				Sensors->SetArrayField(LocalName, Vals);
			}
			ArtObj->SetObjectField(TEXT("sensors"), Sensors);
		}

		if (bWantFull)
		{
			// body xpos/xquat — discovered through articulation's MjBody components.
			TSharedPtr<FJsonObject> Bodies = MakeShared<FJsonObject>();
			TArray<UMjBody*> BodyComponents;
			Art->GetComponents<UMjBody>(BodyComponents);
			FString Prefix = Art->GetName() + TEXT("_");
			for (UMjBody* B : BodyComponents)
			{
				if (!B || B->bIsDefault)
					continue;
				int32 Bid = B->GetMjID();
				if (Bid < 0 || Bid >= m->nbody)
					continue;
				TSharedPtr<FJsonObject> Bo = MakeShared<FJsonObject>();
				TArray<TSharedPtr<FJsonValue>> XPos, XQuat;
				for (int i = 0; i < 3; ++i)
					XPos.Add(MakeShared<FJsonValueNumber>(d->xpos[Bid * 3 + i]));
				for (int i = 0; i < 4; ++i)
					XQuat.Add(MakeShared<FJsonValueNumber>(d->xquat[Bid * 4 + i]));
				Bo->SetArrayField(TEXT("xpos"), XPos);
				Bo->SetArrayField(TEXT("xquat"), XQuat);
				FString LocalName = B->GetMjName();
				if (LocalName.StartsWith(Prefix))
					LocalName = LocalName.Mid(Prefix.Len());
				Bodies->SetObjectField(LocalName, Bo);
			}
			ArtObj->SetObjectField(TEXT("bodies"), Bodies);

			// actuator_force per actuator (positional, same order as ctrl)
			TArray<TSharedPtr<FJsonValue>> AForce;
			for (UMjActuator* A : Art->GetActuators())
			{
				if (!A)
					continue;
				int32 Id = A->GetMjID();
				if (Id < 0 || Id >= m->nu)
					continue;
				AForce.Add(MakeShared<FJsonValueNumber>(d->actuator_force[Id]));
			}
			ArtObj->SetArrayField(TEXT("actuator_force"), AForce);
		}

		// geometry_msgs/Twist-aligned: (linear.x, linear.y, angular.z)
		// filled; rest stays zero. Only when a TwistController is attached.
		if (UMjTwistController* TwistCtrl = Art->FindComponentByClass<UMjTwistController>())
		{
			const FVector Twist = TwistCtrl->GetTwist(); // (Vx, Vy, YawRate)

			TArray<TSharedPtr<FJsonValue>> Linear;
			Linear.Add(MakeShared<FJsonValueNumber>(Twist.X));
			Linear.Add(MakeShared<FJsonValueNumber>(Twist.Y));
			Linear.Add(MakeShared<FJsonValueNumber>(0.0));

			TArray<TSharedPtr<FJsonValue>> Angular;
			Angular.Add(MakeShared<FJsonValueNumber>(0.0));
			Angular.Add(MakeShared<FJsonValueNumber>(0.0));
			Angular.Add(MakeShared<FJsonValueNumber>(Twist.Z));

			TSharedPtr<FJsonObject> TwistObj = MakeShared<FJsonObject>();
			TwistObj->SetArrayField(TEXT("linear"), Linear);
			TwistObj->SetArrayField(TEXT("angular"), Angular);
			ArtObj->SetObjectField(TEXT("twist"), TwistObj);

			ArtObj->SetNumberField(TEXT("actions"),
				static_cast<double>(TwistCtrl->GetActiveActions()));
		}

		PerArt->SetObjectField(Art->GetName(), ArtObj);
	}
	return PerArt;
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::BuildEntitiesBlock(AAMjManager* Manager, mjModel* m, mjData* d)
{
	TSharedPtr<FJsonObject> Scene = MakeShared<FJsonObject>();
	if (!Manager || !m || !d)
		return Scene;

	// Prefer the cached scene-body record table when populated. Avoids a
	// per-call TActorIterator walk on the physics thread.
	auto BuildFromBody = [&](int32 Id, const FString& Name) {
		if (Id < 0 || Id >= m->nbody)
			return;
		TSharedPtr<FJsonObject> Obj = MakeShared<FJsonObject>();
		TArray<TSharedPtr<FJsonValue>> XPos, XQuat;
		for (int i = 0; i < 3; ++i)
			XPos.Add(MakeShared<FJsonValueNumber>(d->xpos[Id * 3 + i]));
		for (int i = 0; i < 4; ++i)
			XQuat.Add(MakeShared<FJsonValueNumber>(d->xquat[Id * 4 + i]));
		Obj->SetArrayField(TEXT("xpos"), XPos);
		Obj->SetArrayField(TEXT("xquat"), XQuat);

		// Free-joint detection: a body with a single jntnum=1 of mjJNT_FREE
		// owns a 7-vec qpos and 6-vec qvel. Stream both. Other joint types
		// get xpos/xquat only — a kinematic-driven heightfield base, etc.
		if (Id < m->nbody && m->body_jntnum && m->body_jntadr)
		{
			int FirstJnt = m->body_jntadr[Id];
			int NumJnt = m->body_jntnum[Id];
			if (FirstJnt >= 0 && NumJnt > 0 && FirstJnt < m->njnt && m->jnt_type[FirstJnt] == mjJNT_FREE)
			{
				int QAddr = m->jnt_qposadr[FirstJnt];
				int VAddr = m->jnt_dofadr[FirstJnt];
				TArray<TSharedPtr<FJsonValue>> QPos, QVel;
				for (int i = 0; i < 7; ++i)
					QPos.Add(MakeShared<FJsonValueNumber>(d->qpos[QAddr + i]));
				for (int i = 0; i < 6; ++i)
					QVel.Add(MakeShared<FJsonValueNumber>(d->qvel[VAddr + i]));
				Obj->SetArrayField(TEXT("qpos"), QPos);
				Obj->SetArrayField(TEXT("qvel"), QVel);
			}
		}
		Scene->SetObjectField(Name, Obj);
	};

	// Cache fast path.
	const TArray<FMjEntityRecord>& Cache = Manager->GetEntities();
	if (Cache.Num() > 0)
	{
		for (const FMjEntityRecord& R : Cache)
			BuildFromBody(R.MjId, R.Name);
		return Scene;
	}

	// Fallback: walk the world via TActorIterator. ONLY safe from the game
	// thread -- the iterator asserts IsInGameThread(). DirectStepHandler runs
	// on the physics async thread, so when called from there with an empty
	// cache (no scene bodies were registered), return an empty block rather
	// than crashing. Tests / pre-cache callers on the game thread still use the
	// fallback path.
	if (!IsInGameThread())
		return Scene;

	UWorld* World = Manager->GetWorld();
	if (!World)
		return Scene;

	TSet<AMjArticulation*> ArticSet;
	for (AMjArticulation* A : Manager->GetAllArticulations())
		ArticSet.Add(A);

	for (TActorIterator<AActor> It(World); It; ++It)
	{
		AActor* Actor = *It;
		if (!Actor)
			continue;
		if (AMjArticulation* AsArt = Cast<AMjArticulation>(Actor))
		{
			if (ArticSet.Contains(AsArt))
				continue;
		}
		TArray<UMjBody*> Bodies;
		Actor->GetComponents<UMjBody>(Bodies);
		for (UMjBody* B : Bodies)
		{
			if (!B || B->bIsDefault)
				continue;
			BuildFromBody(B->GetMjID(), B->GetMjName());
		}
	}
	return Scene;
}

// =============================================================================
// reset / set_mode
// =============================================================================

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleReset(const TSharedPtr<FJsonObject>& Req)
{
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr || !Mgr->PhysicsEngine || !Mgr->PhysicsEngine->m_model)
	{
		return MakeError(TEXT("not_ready"), TEXT("PhysicsEngine not initialised"));
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
			return MakeError(TEXT("not_ready"), TEXT("PhysicsEngine not initialised"));

		FString KfName;
		if (Req->TryGetStringField(TEXT("keyframe_name"), KfName) && !KfName.IsEmpty())
		{
			int Kid = mj_name2id(m, mjOBJ_KEY, TCHAR_TO_UTF8(*KfName));
			if (Kid < 0)
				return MakeError(TEXT("unknown_keyframe"), KfName);
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
				AMjArticulation* Art = Mgr->GetArticulation(APair.Key);
				if (!Art)
					continue;
				const TSharedPtr<FJsonObject>* QObj = nullptr;
				if (!APair.Value->TryGetObject(QObj) || !QObj || !QObj->IsValid())
					continue;

				FString Prefix = Art->GetName() + TEXT("_");
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
		TSharedPtr<FJsonObject> Obs = BuildStepObservations(Mgr, m, d, ActiveObservationLevel);
		if (Obs.IsValid())
			Reply->SetObjectField(TEXT("per_articulation"), Obs);
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
		return MakeError(TEXT("not_ready"), TEXT("PhysicsEngine not initialised"));
	}

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	{
		FScopeLock Lock(&Mgr->PhysicsEngine->CallbackMutex);
		// Fetch model/data under the lock: a concurrent CompileModel frees them
		// under CallbackMutex.
		mjModel* m = Mgr->PhysicsEngine->GetModel();
		mjData* d = Mgr->PhysicsEngine->GetData();
		if (!m || !d)
			return MakeError(TEXT("not_ready"), TEXT("PhysicsEngine not initialised"));
		mj_forward(m, d);

		// Build the reply fields under the lock so the worker's idle-timeout
		// drain can't tear a read done after the lock releases.
		Reply->SetStringField(TEXT("op"), TEXT("forward_ok"));
		Reply->SetNumberField(TEXT("time"), d->time);
		Reply->SetNumberField(TEXT("step"), StepCounter.load(std::memory_order_relaxed));
		AppendClockFields(Reply, d->time);
		TSharedPtr<FJsonObject> Obs = BuildStepObservations(Mgr, m, d, ActiveObservationLevel);
		if (Obs.IsValid())
			Reply->SetObjectField(TEXT("per_articulation"), Obs);
	}
	return Reply;
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleSetMode(const TSharedPtr<FJsonObject>& Req)
{
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr)
		return MakeError(TEXT("not_ready"), TEXT("Manager missing"));

	if (Mgr->StepMode != EStepMode::Auto)
	{
		return MakeError(TEXT("mode_locked_by_server"),
			FString::Printf(TEXT("Project pinned StepMode to %s"), *StepModeToString(Mgr->StepMode)));
	}

	FString ModeStr;
	if (!Req->TryGetStringField(TEXT("mode"), ModeStr))
		return MakeError(TEXT("missing_field"), TEXT("set_mode requires 'mode'"));

	EStepMode NewMode;
	if (!StepModeFromString(ModeStr, NewMode))
		return MakeError(TEXT("bad_mode"), FString::Printf(TEXT("Unknown mode '%s'"), *ModeStr));

	EStepMode Prev = ActiveStepMode;
	SetActiveStepMode(NewMode);

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("set_mode_ok"));
	Reply->SetStringField(TEXT("previous_mode"), StepModeToString(Prev));
	Reply->SetStringField(TEXT("current_mode"), StepModeToString(ActiveStepMode));
	return Reply;
}

// Per-mode lifecycle + step body. OnEnter pauses/unpauses the state+ctrl
// publishers, sets the engine step mode (which unpauses the worker for
// client-driven modes), and installs the step handler; OnExit uninstalls it.
// HandleStep runs the per-step work. Camera publishers stream in every mode, so
// the caller clears FCameraZmqWorker::bPublishersPaused. These are named
// (not anonymous) so RpcDispatcher.h can friend them for internal access.
struct FLiveStepMode : FStepModeStrategy
{
	EStepMode Mode() const override { return EStepMode::Live; }
	void OnEnter(FURLabRpcDispatcher& /*D*/, AAMjManager& Mgr) override
	{
		Mgr.bPublishersPaused.store(false, std::memory_order_release);
		if (Mgr.PhysicsEngine)
			Mgr.PhysicsEngine->SetStepMode(EStepMode::Live);
	}
	void OnExit(FURLabRpcDispatcher& /*D*/, AAMjManager& /*Mgr*/) override {}

	// UE drives its own physics: apply ctrl and read current state. n_steps is
	// ignored (UE steps at its own rate). Cameras are served against the latest
	// render snapshot id since live has no discrete stepped frame.
	TSharedPtr<FJsonObject> HandleStep(FURLabRpcDispatcher& D,
		const TSharedPtr<FJsonObject>& Req, const FStepRequestCommon& Common) override
	{
		AAMjManager* Mgr = D.OwnerMgr.Get();
		if (!Mgr || !Mgr->PhysicsEngine)
			return FURLabRpcDispatcher::MakeError(TEXT("not_ready"), TEXT("PhysicsEngine not initialised"));
		UMjPhysicsEngine* Engine = Mgr->PhysicsEngine;

		FMjStepRequest TmpReq;
		ParseStepPerArticulation(Req, Mgr, TmpReq);

		TSharedPtr<FJsonObject> Obs;
		TSharedPtr<FJsonObject> Scene;
		uint64 FrameId = 0;
		double PostTime = 0.0;
		{
			// Fetch model/data AFTER acquiring CallbackMutex: a concurrent
			// CompileModel frees them under this lock, so a fetch before it
			// would dangle.
			FScopeLock Lock(&Engine->CallbackMutex);
			mjModel* m = Engine->GetModel();
			mjData* d = Engine->GetData();
			if (!m || !d)
				return FURLabRpcDispatcher::MakeError(TEXT("not_ready"), TEXT("PhysicsEngine not initialised"));
			FURLabRpcDispatcher::ApplyStepCtrl(Mgr, TmpReq, m, d);
			// Most recently published snapshot id (UE's autonomous physics owns
			// stepping here), so the client can wait for a streamed frame >= this.
			FrameId = Engine->GetRenderFrameId();
			PostTime = d->time;
			Obs = FURLabRpcDispatcher::BuildStepObservations(Mgr, m, d, Common.ObservationLevel);
			Scene = FURLabRpcDispatcher::BuildEntitiesBlock(Mgr, m, d);
		}

		TMap<FString, uint64> CameraMinFrameIds = Common.CameraMinFrameIds;
		if (Common.bRenderSync && Common.CameraSpec.Num() > 0)
			D.RenderCamerasSync(Mgr, Common.CameraSpec, FrameId, Common.CameraTimeoutMs, CameraMinFrameIds);
		else if (Common.bRenderAsync && Common.CameraSpec.Num() > 0)
			D.RenderCamerasSync(Mgr, Common.CameraSpec, 0, Common.CameraTimeoutMs, CameraMinFrameIds, /*bWait=*/false);
		else if (Common.bWaitCameras && Common.CameraSpec.Num() > 0)
			D.WaitForCameraFrames(Mgr, Common.CameraSpec, FrameId, Common.CameraTimeoutMs, CameraMinFrameIds);

		const int64 StepIdx = D.StepCounter.load(std::memory_order_relaxed);
		return D.BuildStepReply(PostTime, StepIdx, FrameId, Obs, Scene, Mgr,
			Common.CameraSpec, CameraMinFrameIds);
	}
};

struct FDirectStepMode : FStepModeStrategy
{
	EStepMode Mode() const override { return EStepMode::Direct; }
	void OnEnter(FURLabRpcDispatcher& D, AAMjManager& Mgr) override
	{
		Mgr.bPublishersPaused.store(true, std::memory_order_release);
		if (Mgr.PhysicsEngine)
			Mgr.PhysicsEngine->SetStepMode(EStepMode::Direct);
		D.InstallDirectHandler();
	}
	void OnExit(FURLabRpcDispatcher& D, AAMjManager& /*Mgr*/) override
	{
		D.UninstallDirectHandler();
	}

	// Enqueue an FMjDirectStepCommand for the physics worker's step handler and
	// wait for completion. If the worker isn't running (test / editor path), pump
	// the handler inline under the engine lock.
	TSharedPtr<FJsonObject> HandleStep(FURLabRpcDispatcher& D,
		const TSharedPtr<FJsonObject>& Req, const FStepRequestCommon& Common) override
	{
		AAMjManager* Mgr = D.OwnerMgr.Get();
		if (!Mgr || !Mgr->PhysicsEngine)
			return FURLabRpcDispatcher::MakeError(TEXT("not_ready"), TEXT("PhysicsEngine not initialised"));
		UMjPhysicsEngine* Engine = Mgr->PhysicsEngine;

		TSharedPtr<FMjDirectStepCommand> Cmd = MakeShared<FMjDirectStepCommand>();
		int32 NSteps = 1;
		Req->TryGetNumberField(TEXT("n_steps"), NSteps);
		Cmd->Request.NSteps = NSteps > 0 ? NSteps : 1;
		Cmd->ObservationLevel = Common.ObservationLevel;
		ParseStepPerArticulation(Req, Mgr, Cmd->Request);

		Cmd->Completion = FPlatformProcess::GetSynchEventFromPool(true);

		const bool bWorkerRunning = Engine->bWorkerRunning.load(std::memory_order_acquire);
		D.StepQueue.Enqueue(Cmd);
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
				if (D.bDraining.load(std::memory_order_acquire))
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
				D.RenderCamerasSync(Mgr, Common.CameraSpec, Cmd->ResultFrameId, Common.CameraTimeoutMs, CameraMinFrameIds);
			else if (Common.bRenderAsync && Common.CameraSpec.Num() > 0)
				D.RenderCamerasSync(Mgr, Common.CameraSpec, 0, Common.CameraTimeoutMs, CameraMinFrameIds, /*bWait=*/false);
			else if (Common.bWaitCameras && Common.CameraSpec.Num() > 0)
				D.WaitForCameraFrames(Mgr, Common.CameraSpec, Cmd->ResultFrameId, Common.CameraTimeoutMs, CameraMinFrameIds);
			return D.BuildStepReply(Cmd->ResultTime, Cmd->ResultStep, Cmd->ResultFrameId,
				Cmd->Observations, Cmd->Entities, Mgr, Common.CameraSpec, CameraMinFrameIds);
		}

		// We stop waiting but the command is still queued. Mark it abandoned so
		// the handler discards it instead of stepping physics for a request the
		// client already saw fail (and may retry) -- otherwise the step executes
		// twice.
		Cmd->bAbandoned.store(true, std::memory_order_release);
		if (D.bDraining.load(std::memory_order_acquire))
			return FURLabRpcDispatcher::MakeError(TEXT("shutting_down"),
				TEXT("Bridge stopping; Direct-mode step abandoned"));
		return FURLabRpcDispatcher::MakeError(TEXT("step_timeout"),
			TEXT("Direct-mode step did not complete within 5s"));
	}
};

struct FPuppetStepMode : FStepModeStrategy
{
	EStepMode Mode() const override { return EStepMode::Puppet; }
	void OnEnter(FURLabRpcDispatcher& /*D*/, AAMjManager& Mgr) override
	{
		Mgr.bPublishersPaused.store(true, std::memory_order_release);
		if (Mgr.PhysicsEngine)
			Mgr.PhysicsEngine->SetStepMode(EStepMode::Puppet);
	}
	void OnExit(FURLabRpcDispatcher& /*D*/, AAMjManager& /*Mgr*/) override {}

	// Client owns the integrator: write the pushed qpos/qvel/ctrl/time into
	// mjData, run mj_forward, and return the derived state.
	TSharedPtr<FJsonObject> HandleStep(FURLabRpcDispatcher& D,
		const TSharedPtr<FJsonObject>& Req, const FStepRequestCommon& Common) override
	{
		AAMjManager* Mgr = D.OwnerMgr.Get();
		if (!Mgr || !Mgr->PhysicsEngine)
			return FURLabRpcDispatcher::MakeError(TEXT("not_ready"), TEXT("PhysicsEngine not initialised"));
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

		TSharedPtr<FJsonObject> Obs;
		TSharedPtr<FJsonObject> Scene;
		uint64 PostFrameId = 0;
		double PostTime = 0.0;
		{
			// Fetch model/data AFTER the lock: a concurrent CompileModel frees
			// them under CallbackMutex.
			FScopeLock Lock(&Engine->CallbackMutex);
			mjModel* m = Engine->GetModel();
			mjData* d = Engine->GetData();
			if (!m || !d)
				return FURLabRpcDispatcher::MakeError(TEXT("not_ready"), TEXT("PhysicsEngine not initialised"));
			ApplyPushedState(Engine, Push, m, d);

			// Publish the just-pushed pose to the render snapshot now, while we
			// still hold CallbackMutex (lock order CallbackMutex ->
			// RenderStateMutex). Without this the snapshot only refreshes on the
			// physics loop's idle timeout, so cameras and any render consumer
			// would lag the pushed state. The synchronous camera path below
			// relies on this snapshot being current.
			Engine->PushRenderState();

			// Read observations/entities and the reply's time + frame id from d
			// while still holding the lock. The puppet-mode worker wakes on its
			// idle timeout and can mutate d (mocap/wrench drain), which would tear
			// a read done after the lock releases.
			PostFrameId = Engine->GetRenderFrameId();
			PostTime = d->time;
			Obs = FURLabRpcDispatcher::BuildStepObservations(Mgr, m, d, Common.ObservationLevel);
			Scene = FURLabRpcDispatcher::BuildEntitiesBlock(Mgr, m, d);
		}

		// frame_id is the post-step state id: the client passes it back as a
		// camera frame_id to fetch the image showing this exact step's state.
		TMap<FString, uint64> CameraMinFrameIds = Common.CameraMinFrameIds;
		if (Common.bRenderSync && Common.CameraSpec.Num() > 0)
			D.RenderCamerasSync(Mgr, Common.CameraSpec, PostFrameId, Common.CameraTimeoutMs, CameraMinFrameIds);
		else if (Common.bRenderAsync && Common.CameraSpec.Num() > 0)
			D.RenderCamerasSync(Mgr, Common.CameraSpec, 0, Common.CameraTimeoutMs, CameraMinFrameIds, /*bWait=*/false);
		else if (Common.bWaitCameras && Common.CameraSpec.Num() > 0)
			D.WaitForCameraFrames(Mgr, Common.CameraSpec, PostFrameId, Common.CameraTimeoutMs, CameraMinFrameIds);

		const int64 StepIdx = D.StepCounter.fetch_add(1, std::memory_order_relaxed) + 1;
		TSharedPtr<FJsonObject> Reply = D.BuildStepReply(PostTime, StepIdx, PostFrameId,
			Obs, Scene, Mgr, Common.CameraSpec, CameraMinFrameIds);
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
};

TSharedPtr<FStepModeStrategy> FURLabRpcDispatcher::MakeStepStrategy(EStepMode Mode)
{
	switch (Mode)
	{
		case EStepMode::Direct:
			return MakeShared<FDirectStepMode>();
		case EStepMode::Puppet:
			return MakeShared<FPuppetStepMode>();
		case EStepMode::Live:
		case EStepMode::Auto:
		default:
			return MakeShared<FLiveStepMode>();
	}
}

void FURLabRpcDispatcher::SetActiveStepMode(EStepMode NewMode)
{
	// Serialises install/uninstall side effects against concurrent set_mode
	// calls; Dispatch releases DispatchMutex before handlers.
	FScopeLock Lock(&DispatchMutex);

	const EStepMode Mode = (NewMode == EStepMode::Auto) ? EStepMode::Live : NewMode;
	const EStepMode CurMode = ActiveStepMode.load(std::memory_order_acquire);
	if (Mode == CurMode)
		return;

	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr)
		return;

	if (CurrentStepStrategy)
		CurrentStepStrategy->OnExit(*this, *Mgr);
	DrainQueues();

	ActiveStepMode.store(Mode, std::memory_order_release);
	// Camera publishers stream in every mode (frames are decoupled from the
	// step reply), so they are never paused by mode.
	FCameraZmqWorker::bPublishersPaused.store(false, std::memory_order_release);

	CurrentStepStrategy = MakeStepStrategy(Mode);
	CurrentStepStrategy->OnEnter(*this, *Mgr);

	UE_LOG(LogURLabNet, Log, TEXT("FURLabRpcDispatcher: step mode -> %s"),
		*StepModeToString(Mode));
}

void FURLabRpcDispatcher::ReapplyActiveStepMode()
{
	FScopeLock Lock(&DispatchMutex);
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr || !CurrentStepStrategy)
		return;
	// OnExit before OnEnter so the direct handler's install guard
	// (bDirectHandlerInstalled) doesn't skip reinstalling onto the fresh engine
	// handler slot after a recompile.
	CurrentStepStrategy->OnExit(*this, *Mgr);
	CurrentStepStrategy->OnEnter(*this, *Mgr);
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

		// control_mode="raw" per articulation bypasses the UE controller
		// (NetworkValue treated as direct ctrl setpoint). Name-keyed so
		// adding/removing articulations doesn't shift the mapping. The set is
		// fixed for this command (registration is compile-time), so read it once.
		const TArray<AMjArticulation*>& Arts = Mgr->GetAllArticulations();
		TMap<AMjArticulation*, bool> SkipController;
		for (AMjArticulation* Art : Arts)
		{
			if (!Art)
				continue;
			const FString* Mode = Cmd->Request.PerArticulationControlMode.Find(Art->GetName());
			const bool bRaw = Mode && Mode->Equals(TEXT("raw"), ESearchCase::IgnoreCase);
			SkipController.Add(Art, bRaw);
		}

		for (int32 i = 0; i < Cmd->Request.NSteps; ++i)
		{
			for (AMjArticulation* Art : Arts)
			{
				if (!Art)
					continue;
				const bool* bSkip = SkipController.Find(Art);
				Art->ApplyControls(bSkip != nullptr && *bSkip);
			}
			mj_step(m, d);
			if (Engine->OnPostStep)
				Engine->OnPostStep(m, d);
		}
		Cmd->ResultTime = d->time;
		Cmd->ResultStep = StepCounter.fetch_add(Cmd->Request.NSteps, std::memory_order_relaxed)
						+ Cmd->Request.NSteps;
		Cmd->Observations = BuildStepObservations(Mgr, m, d, Cmd->ObservationLevel);
		Cmd->Entities = BuildEntitiesBlock(Mgr, m, d);
		// Publish the just-stepped state to the render snapshot now, while this
		// handler still owns d (it runs inside the engine's CallbackMutex), and
		// capture the resulting frame_id for the reply. Without this the RPC
		// thread would read GetRenderFrameId() after we Trigger() below but
		// before the physics loop's own PushRenderState() runs, returning the
		// previous step's id and breaking camera frame association.
		Engine->PushRenderState();
		Engine->bRenderStatePublishedThisStep = true; // loop tail must not republish
		Cmd->ResultFrameId = Engine->GetRenderFrameId();
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

