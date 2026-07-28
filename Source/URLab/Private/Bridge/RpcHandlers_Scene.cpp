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
/** Resolve a save / load path. Bare filename -> <Project>/Saved/URLab/Replays/.
 *  Must match AMjReplayManager::SaveRecordingToFile so the
 *  recording_save_ok absolute_path actually resolves on the bridge. */
FString ResolveReplayPath(const FString& UserPath, const FString& DefaultBaseName = TEXT(""))
{
	FString BaseDir = FPaths::ProjectSavedDir() / TEXT("URLab") / TEXT("Replays");
	IFileManager::Get().MakeDirectory(*BaseDir, true);

	FString Path = UserPath;
	if (Path.IsEmpty())
	{
		Path = DefaultBaseName.IsEmpty() ? TEXT("recording.json") : DefaultBaseName;
	}

	if (FPaths::IsRelative(Path))
	{
		Path = FPaths::Combine(BaseDir, Path);
	}
	return FPaths::ConvertRelativePathToFull(Path);
}
} // namespace

// =============================================================================
// set_qpos — manager-required runtime write to a single articulation's qpos.
//
// Two write modes:
//   - Free-base 7-vec shortcut: len=7 and the first joint is mjJNT_FREE,
//     writes only the 7 free-joint slots (xyz + quat). Skips dof joints.
//   - Full per-articulation qpos: len matches the articulation's total qpos
//     dim (sum of per-joint slot widths in GetJoints() order). Writes the
//     whole slice.
// Always calls mj_forward after the write, mirroring the puppet push-state
// path so derived quantities (xpos, sensors) reflect the new state.
// =============================================================================
TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleSetQpos(const TSharedPtr<FJsonObject>& Req)
{
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr || !Mgr->PhysicsEngine || !Mgr->PhysicsEngine->IsInitialized())
		return MakeError(URLabError::NotReady, TEXT("Manager not initialised"));

	mjModel* m = Mgr->PhysicsEngine->GetModel();
	mjData* d = Mgr->PhysicsEngine->GetData();
	if (!m || !d)
		return MakeError(URLabError::NotReady, TEXT("MjModel/MjData missing"));

	// target/target_by wire shape. target_by="actor_name" looks up via
	// the manager's GetArticulation (UE name match); default
	// "actor_id" walks ActorId.
	FString Target, By;
	Req->TryGetStringField(TEXT("target"), Target);
	Req->TryGetStringField(TEXT("target_by"), By);
	if (Target.IsEmpty())
	{
		return MakeError(URLabError::MissingField,
			TEXT("set_qpos: missing 'target' field"));
	}
	const bool bByName = By.Equals(TEXT("actor_name"), ESearchCase::IgnoreCase);

	AMjArticulation* Art = nullptr;
	if (bByName)
	{
		Art = Mgr->GetArticulation(Target);
	}
	else
	{
		for (AMjArticulation* A : Mgr->GetAllArticulations())
		{
			if (A && A->ActorId.Equals(Target))
			{
				Art = A;
				break;
			}
		}
	}
	if (!Art)
	{
		return MakeError(URLabError::UnknownArticulation, Target);
	}

	if (TSharedPtr<FJsonObject> Denied = RejectIfNotControlOwner(FName(*Art->GetName()), Req))
		return Denied;

	const TArray<TSharedPtr<FJsonValue>>* QPosArr = nullptr;
	if (!Req->TryGetArrayField(TEXT("qpos"), QPosArr) || !QPosArr)
		return MakeError(URLabError::MissingField, TEXT("set_qpos requires 'qpos' array"));

	struct FJointSlot
	{
		int32 Adr;
		int32 Size;
		int32 Type;
	};
	TArray<FJointSlot> Slots;
	int32 ArtQDim = 0;
	for (UMjJoint* J : Art->GetJoints())
	{
		if (!J)
			continue;
		int32 Id = J->GetMjID();
		if (Id < 0 || Id >= m->njnt)
			continue;
		int32 Size = 1;
		switch (m->jnt_type[Id])
		{
			case mjJNT_FREE:
				Size = 7;
				break;
			case mjJNT_BALL:
				Size = 4;
				break;
			case mjJNT_SLIDE:
			case mjJNT_HINGE:
				Size = 1;
				break;
		}
		Slots.Add({m->jnt_qposadr[Id], Size, m->jnt_type[Id]});
		ArtQDim += Size;
	}

	if (Slots.Num() == 0)
		return MakeError(URLabError::NoJoints,
			TEXT("Articulation has no joints; nothing to write"));

	const int32 InN = QPosArr->Num();
	bool bFreeBaseShortcut = false;
	if (InN == 7 && Slots[0].Type == mjJNT_FREE && ArtQDim != 7)
		bFreeBaseShortcut = true;
	else if (InN != ArtQDim)
		return MakeError(URLabError::DimMismatch,
			FString::Printf(
				TEXT("qpos length %d != articulation qpos dim %d (free-base shortcut requires len=7 with FREE root)"),
				InN, ArtQDim));

	// Read-back of the written qpos must stay inside the lock: a concurrent
	// worker step (or a recompile) would otherwise tear the echoed values.
	TArray<TSharedPtr<FJsonValue>> Out;
	{
		FScopeLock Lock(&Mgr->PhysicsEngine->CallbackMutex);
		if (bFreeBaseShortcut)
		{
			const int32 Adr = Slots[0].Adr;
			for (int32 i = 0; i < 7; ++i)
				d->qpos[Adr + i] = (mjtNum)(*QPosArr)[i]->AsNumber();
		}
		else
		{
			int32 Cursor = 0;
			for (const FJointSlot& S : Slots)
			{
				for (int32 i = 0; i < S.Size; ++i, ++Cursor)
					d->qpos[S.Adr + i] = (mjtNum)(*QPosArr)[Cursor]->AsNumber();
			}
		}
		mj_forward(m, d);

		if (bFreeBaseShortcut)
		{
			const int32 Adr = Slots[0].Adr;
			for (int32 i = 0; i < 7; ++i)
				Out.Add(MakeShared<FJsonValueNumber>(d->qpos[Adr + i]));
		}
		else
		{
			for (const FJointSlot& S : Slots)
				for (int32 i = 0; i < S.Size; ++i)
					Out.Add(MakeShared<FJsonValueNumber>(d->qpos[S.Adr + i]));
		}
	}

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("set_qpos_ok"));
	// Echo back the resolved actor identifiers so the caller can
	// confirm which articulation actually got the write. `target`
	// matches the request's target field; `actor_name` is the UE
	// name (always present, even if actor_id was the lookup key).
	Reply->SetStringField(TEXT("target"), Target);
	Reply->SetStringField(TEXT("actor_name"), Art->GetName());
	if (!Art->ActorId.IsEmpty())
		Reply->SetStringField(TEXT("actor_id"), Art->ActorId);
	Reply->SetArrayField(TEXT("qpos"), Out);
	Reply->SetBoolField(TEXT("free_base_shortcut"), bFreeBaseShortcut);
	return Reply;
}

// =============================================================================
// set_mocap_pose / read_mocap_pose / get_contacts — runtime MJ-side reads/writes.
//
// All three operate directly on the live mjModel/mjData under the engine's
// CallbackMutex (same as set_qpos). Body name lookup uses mj_name2id with
// the full compiled MJ name (URLab prefixes are already included).
// =============================================================================

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleSetMocapPose(const TSharedPtr<FJsonObject>& Req)
{
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr || !Mgr->PhysicsEngine || !Mgr->PhysicsEngine->IsInitialized())
		return MakeError(URLabError::NotReady, TEXT("Manager not initialised"));

	mjModel* m = Mgr->PhysicsEngine->GetModel();
	mjData* d = Mgr->PhysicsEngine->GetData();
	if (!m || !d)
		return MakeError(URLabError::NotReady, TEXT("MjModel/MjData missing"));

	FString Body;
	Req->TryGetStringField(TEXT("body"), Body);
	if (Body.IsEmpty())
		return MakeError(URLabError::MissingField, TEXT("set_mocap_pose: missing 'body'"));

	const int32 BodyId = mj_name2id(m, mjOBJ_BODY, TCHAR_TO_UTF8(*Body));
	if (BodyId < 0)
		return MakeError(URLabError::UnknownBody, Body);

	{
		FName ArtKey;
		for (AMjArticulation* Art : Mgr->GetAllArticulations())
		{
			if (!Art)
				continue;
			for (UMjBody* B : Art->GetBodies())
			{
				if (B && B->GetMjName().Equals(Body))
				{
					ArtKey = FName(*Art->GetName());
					break;
				}
			}
			if (!ArtKey.IsNone())
				break;
		}
		if (!ArtKey.IsNone())
		{
			if (TSharedPtr<FJsonObject> Denied = RejectIfNotControlOwner(ArtKey, Req))
				return Denied;
		}
	}

	const int32 MocapId = m->body_mocapid[BodyId];
	if (MocapId < 0)
		return MakeError(URLabError::NotMocapBody,
			FString::Printf(TEXT("Body '%s' is not a mocap body"), *Body));

	const TArray<TSharedPtr<FJsonValue>>* PosArr = nullptr;
	const TArray<TSharedPtr<FJsonValue>>* QuatArr = nullptr;
	const bool bHasPos = Req->TryGetArrayField(TEXT("pos"), PosArr) && PosArr && PosArr->Num() == 3;
	const bool bHasQuat = Req->TryGetArrayField(TEXT("quat"), QuatArr) && QuatArr && QuatArr->Num() == 4;
	if (!bHasPos && !bHasQuat)
		return MakeError(URLabError::MissingField,
			TEXT("set_mocap_pose requires at least one of pos[3] or quat[4]"));

	// Read-back of the written mocap pose must stay inside the lock: a concurrent
	// worker step (or a recompile) would otherwise tear the echoed values.
	TArray<TSharedPtr<FJsonValue>> PosOut, QuatOut;
	{
		FScopeLock Lock(&Mgr->PhysicsEngine->CallbackMutex);
		if (bHasPos)
		{
			for (int32 i = 0; i < 3; ++i)
				d->mocap_pos[3 * MocapId + i] = (mjtNum)(*PosArr)[i]->AsNumber();
		}
		if (bHasQuat)
		{
			for (int32 i = 0; i < 4; ++i)
				d->mocap_quat[4 * MocapId + i] = (mjtNum)(*QuatArr)[i]->AsNumber();
		}

		for (int32 i = 0; i < 3; ++i)
			PosOut.Add(MakeShared<FJsonValueNumber>(d->mocap_pos[3 * MocapId + i]));
		for (int32 i = 0; i < 4; ++i)
			QuatOut.Add(MakeShared<FJsonValueNumber>(d->mocap_quat[4 * MocapId + i]));
	}

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("set_mocap_pose_ok"));
	Reply->SetStringField(TEXT("body"), Body);
	Reply->SetArrayField(TEXT("pos"), PosOut);
	Reply->SetArrayField(TEXT("quat"), QuatOut);
	return Reply;
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleReadMocapPose(const TSharedPtr<FJsonObject>& Req)
{
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr || !Mgr->PhysicsEngine || !Mgr->PhysicsEngine->IsInitialized())
		return MakeError(URLabError::NotReady, TEXT("Manager not initialised"));

	mjModel* m = Mgr->PhysicsEngine->GetModel();
	mjData* d = Mgr->PhysicsEngine->GetData();
	if (!m || !d)
		return MakeError(URLabError::NotReady, TEXT("MjModel/MjData missing"));

	FString Body;
	Req->TryGetStringField(TEXT("body"), Body);
	if (Body.IsEmpty())
		return MakeError(URLabError::MissingField, TEXT("read_mocap_pose: missing 'body'"));

	const int32 BodyId = mj_name2id(m, mjOBJ_BODY, TCHAR_TO_UTF8(*Body));
	if (BodyId < 0)
		return MakeError(URLabError::UnknownBody, Body);

	const int32 MocapId = m->body_mocapid[BodyId];
	if (MocapId < 0)
		return MakeError(URLabError::NotMocapBody,
			FString::Printf(TEXT("Body '%s' is not a mocap body"), *Body));

	TArray<TSharedPtr<FJsonValue>> PosOut, QuatOut;
	{
		FScopeLock Lock(&Mgr->PhysicsEngine->CallbackMutex);
		for (int32 i = 0; i < 3; ++i)
			PosOut.Add(MakeShared<FJsonValueNumber>(d->mocap_pos[3 * MocapId + i]));
		for (int32 i = 0; i < 4; ++i)
			QuatOut.Add(MakeShared<FJsonValueNumber>(d->mocap_quat[4 * MocapId + i]));
	}

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("read_mocap_pose_ok"));
	Reply->SetStringField(TEXT("body"), Body);
	Reply->SetArrayField(TEXT("pos"), PosOut);
	Reply->SetArrayField(TEXT("quat"), QuatOut);
	return Reply;
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleGetContacts(const TSharedPtr<FJsonObject>& Req)
{
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr || !Mgr->PhysicsEngine || !Mgr->PhysicsEngine->IsInitialized())
		return MakeError(URLabError::NotReady, TEXT("Manager not initialised"));

	mjModel* m = Mgr->PhysicsEngine->GetModel();
	mjData* d = Mgr->PhysicsEngine->GetData();
	if (!m || !d)
		return MakeError(URLabError::NotReady, TEXT("MjModel/MjData missing"));

	int32 MaxContacts = 64;
	{
		int32 Cap = 0;
		if (Req->TryGetNumberField(TEXT("max_contacts"), Cap) && Cap > 0)
			MaxContacts = Cap;
	}

	// Optional filter: {body1?, body2?, geom1?, geom2?}. AND across set fields.
	FString FBody1, FBody2, FGeom1, FGeom2;
	const TSharedPtr<FJsonObject>* FilterObj = nullptr;
	if (Req->TryGetObjectField(TEXT("filter"), FilterObj) && FilterObj && *FilterObj)
	{
		(*FilterObj)->TryGetStringField(TEXT("body1"), FBody1);
		(*FilterObj)->TryGetStringField(TEXT("body2"), FBody2);
		(*FilterObj)->TryGetStringField(TEXT("geom1"), FGeom1);
		(*FilterObj)->TryGetStringField(TEXT("geom2"), FGeom2);
	}

	auto NameOrEmpty = [&](int Type, int Id) -> FString {
		if (Id < 0)
			return FString();
		const char* p = mj_id2name(m, Type, Id);
		return p ? FString(UTF8_TO_TCHAR(p)) : FString();
	};

	TArray<TSharedPtr<FJsonValue>> Out;
	bool bTruncated = false;
	int32 Matched = 0;

	{
		FScopeLock Lock(&Mgr->PhysicsEngine->CallbackMutex);
		const int32 N = d->ncon;
		for (int32 i = 0; i < N; ++i)
		{
			const mjContact& c = d->contact[i];
			const int32 G1 = c.geom[0];
			const int32 G2 = c.geom[1];
			const int32 B1 = (G1 >= 0 && G1 < m->ngeom) ? m->geom_bodyid[G1] : -1;
			const int32 B2 = (G2 >= 0 && G2 < m->ngeom) ? m->geom_bodyid[G2] : -1;
			const FString G1Name = NameOrEmpty(mjOBJ_GEOM, G1);
			const FString G2Name = NameOrEmpty(mjOBJ_GEOM, G2);
			const FString B1Name = NameOrEmpty(mjOBJ_BODY, B1);
			const FString B2Name = NameOrEmpty(mjOBJ_BODY, B2);

			if (!FGeom1.IsEmpty() && !G1Name.Equals(FGeom1))
				continue;
			if (!FGeom2.IsEmpty() && !G2Name.Equals(FGeom2))
				continue;
			if (!FBody1.IsEmpty() && !B1Name.Equals(FBody1))
				continue;
			if (!FBody2.IsEmpty() && !B2Name.Equals(FBody2))
				continue;

			if (Matched >= MaxContacts)
			{
				bTruncated = true;
				break;
			}

			mjtNum Force[6] = {0};
			mj_contactForce(m, d, i, Force);

			TArray<TSharedPtr<FJsonValue>> Pos;
			for (int32 k = 0; k < 3; ++k)
				Pos.Add(MakeShared<FJsonValueNumber>(c.pos[k]));
			// First row of the contact frame is the contact normal.
			TArray<TSharedPtr<FJsonValue>> Normal;
			for (int32 k = 0; k < 3; ++k)
				Normal.Add(MakeShared<FJsonValueNumber>(c.frame[k]));
			TArray<TSharedPtr<FJsonValue>> ForceArr;
			for (int32 k = 0; k < 6; ++k)
				ForceArr.Add(MakeShared<FJsonValueNumber>(Force[k]));

			TSharedPtr<FJsonObject> CObj = MakeShared<FJsonObject>();
			CObj->SetStringField(TEXT("geom1"), G1Name);
			CObj->SetStringField(TEXT("geom2"), G2Name);
			CObj->SetStringField(TEXT("body1"), B1Name);
			CObj->SetStringField(TEXT("body2"), B2Name);
			CObj->SetArrayField(TEXT("pos"), Pos);
			CObj->SetArrayField(TEXT("normal"), Normal);
			CObj->SetNumberField(TEXT("dist"), c.dist);
			CObj->SetArrayField(TEXT("force"), ForceArr);
			Out.Add(MakeShared<FJsonValueObject>(CObj));
			++Matched;
		}
	}

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("get_contacts_ok"));
	Reply->SetNumberField(TEXT("n_contacts"), Matched);
	Reply->SetBoolField(TEXT("truncated"), bTruncated);
	Reply->SetArrayField(TEXT("contacts"), Out);
	return Reply;
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleListKeyframes(const TSharedPtr<FJsonObject>& /*Req*/)
{
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr || !Mgr->PhysicsEngine || !Mgr->PhysicsEngine->IsInitialized())
		return MakeError(URLabError::NotReady, TEXT("Manager not initialised"));

	mjModel* m = Mgr->PhysicsEngine->GetModel();
	if (!m)
		return MakeError(URLabError::NotReady, TEXT("MjModel missing"));

	auto Slice = [](const mjtNum* src, int32 stride, int32 idx, int32 width) {
		TArray<TSharedPtr<FJsonValue>> Out;
		if (!src || width <= 0)
			return Out;
		for (int32 k = 0; k < width; ++k)
			Out.Add(MakeShared<FJsonValueNumber>(src[idx * stride + k]));
		return Out;
	};

	TArray<TSharedPtr<FJsonValue>> Keys;
	for (int32 i = 0; i < m->nkey; ++i)
	{
		const char* NameC = mj_id2name(m, mjOBJ_KEY, i);
		TSharedPtr<FJsonObject> K = MakeShared<FJsonObject>();
		K->SetStringField(TEXT("name"), NameC ? UTF8_TO_TCHAR(NameC) : TEXT(""));
		K->SetNumberField(TEXT("time"), m->key_time ? m->key_time[i] : 0.0);
		K->SetArrayField(TEXT("qpos"), Slice(m->key_qpos, m->nq, i, m->nq));
		K->SetArrayField(TEXT("qvel"), Slice(m->key_qvel, m->nv, i, m->nv));
		K->SetArrayField(TEXT("ctrl"), Slice(m->key_ctrl, m->nu, i, m->nu));
		K->SetArrayField(TEXT("mocap_pos"), Slice(m->key_mpos, m->nmocap * 3, i, m->nmocap * 3));
		K->SetArrayField(TEXT("mocap_quat"), Slice(m->key_mquat, m->nmocap * 4, i, m->nmocap * 4));
		Keys.Add(MakeShared<FJsonValueObject>(K));
	}

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("list_keyframes_ok"));
	Reply->SetArrayField(TEXT("keyframes"), Keys);
	return Reply;
}

// =============================================================================
// recording_* / replay_*  delegate to AMjReplayManager
// =============================================================================

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleRecording(const FString& Op, const TSharedPtr<FJsonObject>& Req)
{
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr)
		return MakeError(URLabError::NotReady, TEXT("Manager missing"));
	// Use the game-thread-cached pointer; TActorIterator from this worker
	// thread would assert IsInGameThread() and crash.
	AMjReplayManager* RM = CachedReplayManager.Get();
	if (!RM)
		return MakeError(URLabError::NotReady, TEXT("AMjReplayManager not present in scene"));

	if (Op.Equals(TEXT("recording_start")))
	{
		if (RM->bIsRecording)
			return MakeError(URLabError::RecordingAlreadyActive, TEXT("Recording already active"));
		double MaxDur = 0.0;
		if (Req->TryGetNumberField(TEXT("max_duration_s"), MaxDur))
			RM->MaxRecordDuration = (float)MaxDur;
		else
			RM->MaxRecordDuration = FLT_MAX;
		RM->StartRecording();
		TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
		Reply->SetStringField(TEXT("op"), TEXT("recording_start_ok"));
		Reply->SetStringField(TEXT("name"), AMjReplayManager::LiveSessionName);
		Reply->SetNumberField(TEXT("max_duration_s"), RM->MaxRecordDuration);
		return Reply;
	}
	if (Op.Equals(TEXT("recording_stop")))
	{
		if (!RM->bIsRecording)
			return MakeError(URLabError::RecordingNotActive, TEXT("Recording is not active"));
		RM->StopRecording();
		TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
		Reply->SetStringField(TEXT("op"), TEXT("recording_stop_ok"));
		Reply->SetStringField(TEXT("name"), AMjReplayManager::LiveSessionName);
		// Populate the summary fields the Python client maps to
		// RecordingSummary. Recording has been stopped (StopRecording
		// above set bIsRecording=false) so the OnPostStep hook isn't
		// mutating Frames in parallel.
		Reply->SetNumberField(TEXT("frame_count"), static_cast<double>(RM->GetLiveFrameCount()));
		Reply->SetNumberField(TEXT("sim_duration_s"), RM->GetLiveSimDurationS());
		return Reply;
	}
	if (Op.Equals(TEXT("recording_save")))
	{
		FString Path;
		Req->TryGetStringField(TEXT("path"), Path);
		FString FileName = Path.IsEmpty() ? TEXT("recording.json") : FPaths::GetCleanFilename(Path);
		if (Path.IsEmpty())
			Path = FileName;
		bool bOk = RM->SaveRecordingToFile(FileName);
		// ResolveReplayPath now matches the manager's
		// ProjectSavedDir/URLab/Replays/ output dir, so the absolute_path
		// returned to the client points at the file the manager actually wrote.
		FString Abs = ResolveReplayPath(Path);
		TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
		Reply->SetStringField(TEXT("op"), bOk ? TEXT("recording_save_ok") : TEXT("error"));
		Reply->SetStringField(TEXT("absolute_path"), Abs);
		if (!bOk)
			Reply->SetStringField(TEXT("code"), URLabError::PathNotWritable);
		return Reply;
	}
	if (Op.Equals(TEXT("recording_clear")))
	{
		RM->ClearRecording();
		TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
		Reply->SetStringField(TEXT("op"), TEXT("recording_clear_ok"));
		return Reply;
	}
	return MakeError(URLabError::UnknownOp, Op);
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleReplay(const FString& Op, const TSharedPtr<FJsonObject>& Req)
{
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr)
		return MakeError(URLabError::NotReady, TEXT("Manager missing"));
	AMjReplayManager* RM = CachedReplayManager.Get();
	if (!RM)
		return MakeError(URLabError::NotReady, TEXT("AMjReplayManager not present in scene"));

	if (Op.Equals(TEXT("replay_load")))
	{
		FString P;
		if (!Req->TryGetStringField(TEXT("path"), P))
			return MakeError(URLabError::MissingField, TEXT("replay_load requires 'path'"));
		FString FileName = FPaths::GetCleanFilename(P);
		bool bOk = RM->LoadRecordingFromFile(FileName);
		if (!bOk)
			return MakeError(URLabError::PathNotReadable, P);
		TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
		Reply->SetStringField(TEXT("op"), TEXT("replay_load_ok"));
		Reply->SetStringField(TEXT("name"), FPaths::GetBaseFilename(FileName));
		return Reply;
	}
	if (Op.Equals(TEXT("replay_list_sessions")))
	{
		TArray<TSharedPtr<FJsonValue>> Names;
		for (const FString& N : RM->GetSessionNames())
			Names.Add(MakeShared<FJsonValueString>(N));
		TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
		Reply->SetStringField(TEXT("op"), TEXT("replay_list_sessions_ok"));
		Reply->SetArrayField(TEXT("sessions"), Names);
		return Reply;
	}
	if (Op.Equals(TEXT("replay_set_active")))
	{
		FString N;
		if (!Req->TryGetStringField(TEXT("name"), N))
			return MakeError(URLabError::MissingField, TEXT("replay_set_active requires 'name'"));
		if (!RM->Sessions.Contains(N))
			return MakeError(URLabError::ReplaySessionNotFound, N);
		RM->SetActiveSession(N);
		TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
		Reply->SetStringField(TEXT("op"), TEXT("replay_set_active_ok"));
		return Reply;
	}
	if (Op.Equals(TEXT("replay_start")))
	{
		if (ActiveStepMode == EStepMode::Live)
			return MakeError(URLabError::ReplayRequiresStepped,
				TEXT("Switch to direct or puppet before starting replay"));
		RM->StartReplay();
		int32 Total = RM->Sessions.Contains(RM->GetActiveSessionName())
						? RM->Sessions[RM->GetActiveSessionName()].Frames.Num()
						: 0;
		TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
		Reply->SetStringField(TEXT("op"), TEXT("replay_start_ok"));
		Reply->SetStringField(TEXT("active_session"), RM->GetActiveSessionName());
		Reply->SetNumberField(TEXT("total_frames"), Total);
		return Reply;
	}
	if (Op.Equals(TEXT("replay_stop")))
	{
		RM->StopReplay();
		TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
		Reply->SetStringField(TEXT("op"), TEXT("replay_stop_ok"));
		return Reply;
	}
	return MakeError(URLabError::UnknownOp, Op);
}
