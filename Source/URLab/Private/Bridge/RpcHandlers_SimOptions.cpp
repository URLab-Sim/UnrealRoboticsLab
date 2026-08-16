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
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Core/MjSceneOptions.h"
#include "MuJoCo/Gen/Elements/Options/MjFlag.gen.h"
#include "MuJoCo/Gen/Elements/Options/MjOption.gen.h"
#include "MuJoCo/Gen/MjKeywords.gen.h"
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

// =============================================================================
// set_sim_options
// =============================================================================

namespace
{
/** An MJCF keyword. The schema's spellings are ASCII by construction. */
FString KeywordToString(std::string_view Keyword)
{
	FString Out;
	Out.Reserve(static_cast<int32>(Keyword.size()));
	for (const char Character : Keyword)
	{
		Out.AppendChar(static_cast<TCHAR>(Character));
	}
	return Out;
}

/**
 * An MJCF keyword, matched without regard to case.
 *
 * The keyword tables are generated from the schema, so the spellings are
 * MuJoCo's own ("Euler", "RK4", "PGS"). The wire has always accepted any
 * casing, and the ordinal scan below stops at the first spelling the table
 * does not know, which is how it learns the enum's arity without one being
 * declared anywhere.
 */
template <class TEnum>
bool ParseKeyword(const FString& Text, TEnum& Out)
{
	for (int32 Ordinal = 0; Ordinal < 256; ++Ordinal)
	{
		const TEnum Candidate = static_cast<TEnum>(Ordinal);
		const std::string_view Keyword = ps::ue::ToMjcf(Candidate);
		if (Keyword.empty())
		{
			return false;
		}
		if (Text.Equals(KeywordToString(Keyword), ESearchCase::IgnoreCase))
		{
			Out = Candidate;
			return true;
		}
	}
	return false;
}

template <class TEnum>
FString KeywordOf(TEnum Value)
{
	return KeywordToString(ps::ue::ToMjcf(Value));
}

bool TryReadVec3(const TSharedPtr<FJsonObject>& Obj, const TCHAR* Key, double Out[3])
{
	const TArray<TSharedPtr<FJsonValue>>* Arr = nullptr;
	if (!Obj->TryGetArrayField(Key, Arr) || !Arr || Arr->Num() != 3)
		return false;
	Out[0] = (*Arr)[0]->AsNumber();
	Out[1] = (*Arr)[1]->AsNumber();
	Out[2] = (*Arr)[2]->AsNumber();
	return true;
}
} // namespace

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleSetSimOptions(const TSharedPtr<FJsonObject>& Req)
{
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr || !Mgr->PhysicsEngine)
		return MakeError(URLabError::NotReady, TEXT("PhysicsEngine not initialised"));

	const TSharedPtr<FJsonObject>* OptsPtr = nullptr;
	if (!Req->TryGetObjectField(TEXT("options"), OptsPtr) || !OptsPtr || !(*OptsPtr).IsValid())
		return MakeError(URLabError::MissingField, TEXT("set_sim_options requires 'options' object"));
	const TSharedPtr<FJsonObject>& Opts = *OptsPtr;

	UMjOption* const O = Mgr->SceneOption;
	UMjFlag* const F = Mgr->SceneFlags;
	if (O == nullptr || F == nullptr)
		return MakeError(URLabError::NotReady, TEXT("scene <option> missing"));

	// The wire is MJ-native SI and so is the spec, so nothing is converted
	// on the way in; setting a field is what marks it authored.
	double DNum = 0.0;
	if (Opts->TryGetNumberField(TEXT("timestep"), DNum))
	{
		O->Timestep = DNum;
	}

	double V3[3];
	if (TryReadVec3(Opts, TEXT("gravity"), V3))
	{
		O->Gravity = FMjDirection3(V3[0], V3[1], V3[2]);
	}
	if (TryReadVec3(Opts, TEXT("wind"), V3))
	{
		O->Wind = FMjDirection3(V3[0], V3[1], V3[2]);
	}
	if (TryReadVec3(Opts, TEXT("magnetic"), V3))
	{
		O->Magnetic = FMjDirection3(V3[0], V3[1], V3[2]);
	}

	if (Opts->TryGetNumberField(TEXT("density"), DNum))
	{
		O->Density = DNum;
	}
	if (Opts->TryGetNumberField(TEXT("viscosity"), DNum))
	{
		O->Viscosity = DNum;
	}
	if (Opts->TryGetNumberField(TEXT("impratio"), DNum))
	{
		O->Impratio = DNum;
	}
	if (Opts->TryGetNumberField(TEXT("tolerance"), DNum))
	{
		O->Tolerance = DNum;
	}

	int32 INum = 0;
	if (Opts->TryGetNumberField(TEXT("iterations"), INum))
	{
		O->Iterations = INum;
	}
	if (Opts->TryGetNumberField(TEXT("ls_iterations"), INum))
	{
		O->LsIterations = INum;
	}

	FString SNum;
	if (Opts->TryGetStringField(TEXT("integrator"), SNum))
	{
		EMjIntegrator E;
		if (!ParseKeyword(SNum, E))
			return MakeError(URLabError::BadValue, FString::Printf(TEXT("unknown integrator '%s'"), *SNum));
		O->Integrator = E;
	}
	if (Opts->TryGetStringField(TEXT("cone"), SNum))
	{
		EMjCone E;
		if (!ParseKeyword(SNum, E))
			return MakeError(URLabError::BadValue, FString::Printf(TEXT("unknown cone '%s'"), *SNum));
		O->Cone = E;
	}
	if (Opts->TryGetStringField(TEXT("solver"), SNum))
	{
		EMjSolverType E;
		if (!ParseKeyword(SNum, E))
			return MakeError(URLabError::BadValue, FString::Printf(TEXT("unknown solver '%s'"), *SNum));
		O->Solver = E;
	}

	if (Opts->TryGetNumberField(TEXT("noslip_iterations"), INum))
	{
		O->NoslipIterations = INum;
	}
	if (Opts->TryGetNumberField(TEXT("noslip_tolerance"), DNum))
	{
		O->NoslipTolerance = DNum;
	}
	if (Opts->TryGetNumberField(TEXT("ccd_iterations"), INum))
	{
		O->CcdIterations = INum;
	}
	if (Opts->TryGetNumberField(TEXT("ccd_tolerance"), DNum))
	{
		O->CcdTolerance = DNum;
	}

	bool BNum = false;
	if (Opts->TryGetBoolField(TEXT("enable_multiccd"), BNum))
	{
		F->Multiccd = BNum ? EMjEnable::enable : EMjEnable::disable;
	}
	if (Opts->TryGetBoolField(TEXT("enable_sleep"), BNum))
	{
		F->Sleep = BNum ? EMjEnable::enable : EMjEnable::disable;
	}
	if (Opts->TryGetNumberField(TEXT("sleep_tolerance"), DNum))
	{
		O->SleepTolerance = DNum;
	}

	// The physics worker may be mid mj_step on this same m/d on another thread.
	// Serialise every live-model write below (disable/enable flags,
	// ApplyOverridesToModel) and the threadpool rebuild against it under the
	// worker's step lock, and hold the lock across the reply's m->opt reads so
	// the echoed snapshot is coherent. Rebuilding mju_threadpool while a step is
	// in flight is otherwise a crash.
	FScopeLock ModelLock(&Mgr->PhysicsEngine->CallbackMutex);

	// Fetch the live model under the lock: a concurrent CompileModel frees and
	// reallocates m/d, so a pointer captured before the lock could dangle.
	mjModel* m = Mgr->PhysicsEngine->GetModel();
	if (!m)
		return MakeError(URLabError::NotReady, TEXT("mjModel not compiled"));

	// Raw disable / enable bit masks. Values are bitwise-ORs of
	// mujoco/mjmodel.h mjtDisableBit / mjtEnableBit constants.
	// Applied BEFORE the spec is pushed onto the model, so any named
	// bits the caller also set (enable_sleep / enable_multiccd) win on
	// top of the raw mask. Treat the raw masks as a coarse baseline.
	int32 DisableMask = 0;
	if (Opts->TryGetNumberField(TEXT("disableflags"), DisableMask))
	{
		m->opt.disableflags = DisableMask;
	}
	int32 EnableMask = 0;
	if (Opts->TryGetNumberField(TEXT("enableflags"), EnableMask))
	{
		m->opt.enableflags = EnableMask;
	}

	// The spec is the authority and the live model is the runtime effect,
	// so the write goes spec first and model second (Section 4.10).
	MjApplyOptionToModel(O, F, m);

	// Worker thread pool (mju_threadpool). Not a MuJoCo option-struct field —
	// it's a URLab engine setting applied to the live mjData. Clamped to the
	// detected CPU core count; ApplyThreadPool is idempotent.
	int32 NumThreads = 0;
	if (Opts->TryGetNumberField(TEXT("num_worker_threads"), NumThreads))
	{
		Mgr->PhysicsEngine->NumWorkerThreads =
			FMath::Clamp(NumThreads, 0, UMjPhysicsEngine::MaxWorkerThreads());
		Mgr->PhysicsEngine->ApplyThreadPool();
	}

	UE_LOG(LogURLabNet, Log,
		TEXT("FURLabRpcDispatcher: set_sim_options applied (timestep=%.5fs, gravity=[%.3f %.3f %.3f] m/s²)"),
		m->opt.timestep, m->opt.gravity[0], m->opt.gravity[1], m->opt.gravity[2]);

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("set_sim_options_ok"));

	TSharedPtr<FJsonObject> Out = MakeShared<FJsonObject>();
	Out->SetNumberField(TEXT("timestep"), m->opt.timestep);
	Out->SetNumberField(TEXT("num_worker_threads"), Mgr->PhysicsEngine->NumWorkerThreads);
	Out->SetNumberField(TEXT("max_worker_threads"), UMjPhysicsEngine::MaxWorkerThreads());
	{
		TArray<TSharedPtr<FJsonValue>> G;
		G.Add(MakeShared<FJsonValueNumber>(m->opt.gravity[0]));
		G.Add(MakeShared<FJsonValueNumber>(m->opt.gravity[1]));
		G.Add(MakeShared<FJsonValueNumber>(m->opt.gravity[2]));
		Out->SetArrayField(TEXT("gravity"), G);
		TArray<TSharedPtr<FJsonValue>> W;
		W.Add(MakeShared<FJsonValueNumber>(m->opt.wind[0]));
		W.Add(MakeShared<FJsonValueNumber>(m->opt.wind[1]));
		W.Add(MakeShared<FJsonValueNumber>(m->opt.wind[2]));
		Out->SetArrayField(TEXT("wind"), W);
		TArray<TSharedPtr<FJsonValue>> Mg;
		Mg.Add(MakeShared<FJsonValueNumber>(m->opt.magnetic[0]));
		Mg.Add(MakeShared<FJsonValueNumber>(m->opt.magnetic[1]));
		Mg.Add(MakeShared<FJsonValueNumber>(m->opt.magnetic[2]));
		Out->SetArrayField(TEXT("magnetic"), Mg);
	}
	Out->SetNumberField(TEXT("density"), m->opt.density);
	Out->SetNumberField(TEXT("viscosity"), m->opt.viscosity);
	Out->SetNumberField(TEXT("impratio"), m->opt.impratio);
	Out->SetNumberField(TEXT("tolerance"), m->opt.tolerance);
	Out->SetNumberField(TEXT("iterations"), m->opt.iterations);
	Out->SetNumberField(TEXT("ls_iterations"), m->opt.ls_iterations);
	Out->SetStringField(TEXT("integrator"), KeywordOf(static_cast<EMjIntegrator>(m->opt.integrator)));
	Out->SetStringField(TEXT("cone"), KeywordOf(static_cast<EMjCone>(m->opt.cone)));
	Out->SetStringField(TEXT("solver"), KeywordOf(static_cast<EMjSolverType>(m->opt.solver)));
	Out->SetNumberField(TEXT("noslip_iterations"), m->opt.noslip_iterations);
	Out->SetNumberField(TEXT("noslip_tolerance"), m->opt.noslip_tolerance);
	Out->SetNumberField(TEXT("ccd_iterations"), m->opt.ccd_iterations);
	Out->SetNumberField(TEXT("ccd_tolerance"), m->opt.ccd_tolerance);

	Out->SetBoolField(TEXT("enable_multiccd"), (m->opt.disableflags & mjDSBL_MULTICCD) == 0);
	Out->SetBoolField(TEXT("enable_sleep"), (m->opt.enableflags & mjENBL_SLEEP) != 0);
	Out->SetNumberField(TEXT("sleep_tolerance"), m->opt.sleep_tolerance);

	// Echo the raw bit masks so callers using disableflags / enableflags
	// can verify the final composed state (named overrides + raw mask).
	Out->SetNumberField(TEXT("disableflags"), (int32)m->opt.disableflags);
	Out->SetNumberField(TEXT("enableflags"), (int32)m->opt.enableflags);

	Reply->SetObjectField(TEXT("options"), Out);
	return Reply;
}

// =============================================================================
// set_sim_speed
// =============================================================================

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleSetSimSpeed(const TSharedPtr<FJsonObject>& Req)
{
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr || !Mgr->PhysicsEngine)
		return MakeError(URLabError::NotReady, TEXT("PhysicsEngine not initialised"));

	double Pct = 0.0;
	if (!Req->TryGetNumberField(TEXT("percent"), Pct))
		return MakeError(URLabError::MissingField, TEXT("set_sim_speed requires 'percent'"));

	// Engine clamps internally (5..100); echo back so the caller sees what stuck.
	Mgr->PhysicsEngine->SetSimSpeed((float)Pct);
	const float Effective = FMath::Clamp((float)Pct, 5.0f, 100.0f);

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("set_sim_speed_ok"));
	Reply->SetNumberField(TEXT("percent"), Effective);
	return Reply;
}

// =============================================================================
// set_twist
// =============================================================================

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleSetTwist(const TSharedPtr<FJsonObject>& Req)
{
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr)
		return MakeError(URLabError::NotReady, TEXT("Manager missing"));

	FString ArtName;
	if (!Req->TryGetStringField(TEXT("articulation"), ArtName))
		return MakeError(URLabError::MissingField, TEXT("set_twist requires 'articulation'"));

	AMjArticulation* Art = Mgr->GetArticulation(ArtName);
	if (!Art)
		return MakeError(URLabError::UnknownArticulation, ArtName);

	if (TSharedPtr<FJsonObject> Denied = RejectIfNotControlOwner(FName(*Art->GetName()), Req))
		return Denied;

	UMjTwistController* TC = Art->FindComponentByClass<UMjTwistController>();
	if (!TC)
		return MakeError(URLabError::NoTwistController,
			FString::Printf(TEXT("Articulation '%s' has no UMjTwistController"), *ArtName));

	// Wire format mirrors how the bridge already reads twist: linear is
	// (vx, vy, _) m/s, angular is (_, _, yaw_rate) rad/s. Tuple slots
	// beyond the ones used are accepted but ignored.
	auto ReadAxis = [](const TArray<TSharedPtr<FJsonValue>>* Arr, int32 Idx, float& Out) {
		if (Arr && Arr->IsValidIndex(Idx))
			Out = (float)(*Arr)[Idx]->AsNumber();
	};

	float Vx = 0.f, Vy = 0.f, YawRate = 0.f;
	const TArray<TSharedPtr<FJsonValue>>* LinArr = nullptr;
	const TArray<TSharedPtr<FJsonValue>>* AngArr = nullptr;
	Req->TryGetArrayField(TEXT("linear"), LinArr);
	Req->TryGetArrayField(TEXT("angular"), AngArr);
	ReadAxis(LinArr, 0, Vx);
	ReadAxis(LinArr, 1, Vy);
	ReadAxis(AngArr, 2, YawRate);

	TC->SetTwist(Vx, Vy, YawRate);

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("set_twist_ok"));
	Reply->SetStringField(TEXT("articulation"), ArtName);
	{
		TArray<TSharedPtr<FJsonValue>> L;
		L.Add(MakeShared<FJsonValueNumber>(Vx));
		L.Add(MakeShared<FJsonValueNumber>(Vy));
		L.Add(MakeShared<FJsonValueNumber>(0.0));
		Reply->SetArrayField(TEXT("linear"), L);
		TArray<TSharedPtr<FJsonValue>> A;
		A.Add(MakeShared<FJsonValueNumber>(0.0));
		A.Add(MakeShared<FJsonValueNumber>(0.0));
		A.Add(MakeShared<FJsonValueNumber>(YawRate));
		Reply->SetArrayField(TEXT("angular"), A);
	}
	return Reply;
}
