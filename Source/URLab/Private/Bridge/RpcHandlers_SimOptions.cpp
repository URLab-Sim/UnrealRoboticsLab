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

// =============================================================================
// configure_controller
// =============================================================================

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleConfigureController(const TSharedPtr<FJsonObject>& Req)
{
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr)
		return MakeError(TEXT("not_ready"), TEXT("Manager missing"));

	FString ArtName;
	if (!Req->TryGetStringField(TEXT("articulation"), ArtName))
		return MakeError(TEXT("missing_field"), TEXT("configure_controller requires 'articulation'"));

	AMjArticulation* Art = Mgr->GetArticulation(ArtName);
	if (!Art)
		return MakeError(TEXT("unknown_articulation"), ArtName);

	UMjArticulationController* Ctrl = Art->FindComponentByClass<UMjArticulationController>();
	if (!Ctrl)
		return MakeError(TEXT("no_controller"), FString::Printf(TEXT("Articulation '%s' has no controller"), *ArtName));

	const TSharedPtr<FJsonObject>* Params = nullptr;
	if (Req->TryGetObjectField(TEXT("params"), Params) && Params && Params->IsValid())
	{
		Ctrl->ApplyConfig(*Params);
	}

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("configure_controller_ok"));
	Reply->SetStringField(TEXT("articulation"), ArtName);

	TSharedPtr<FJsonObject> Out = MakeShared<FJsonObject>();
	Ctrl->GetCurrentConfig(Out);
	Reply->SetObjectField(TEXT("params"), Out);
	return Reply;
}

// =============================================================================
// set_sim_options
// =============================================================================

namespace
{
bool ParseIntegrator(const FString& S, EMjIntegrator& Out)
{
	if (S.Equals(TEXT("euler"), ESearchCase::IgnoreCase))
	{
		Out = EMjIntegrator::Euler;
		return true;
	}
	if (S.Equals(TEXT("rk4"), ESearchCase::IgnoreCase))
	{
		Out = EMjIntegrator::RK4;
		return true;
	}
	if (S.Equals(TEXT("implicit"), ESearchCase::IgnoreCase))
	{
		Out = EMjIntegrator::Implicit;
		return true;
	}
	if (S.Equals(TEXT("implicitfast"), ESearchCase::IgnoreCase))
	{
		Out = EMjIntegrator::ImplicitFast;
		return true;
	}
	return false;
}
FString IntegratorToString(EMjIntegrator I)
{
	switch (I)
	{
		case EMjIntegrator::Euler:
			return TEXT("euler");
		case EMjIntegrator::RK4:
			return TEXT("rk4");
		case EMjIntegrator::Implicit:
			return TEXT("implicit");
		case EMjIntegrator::ImplicitFast:
			return TEXT("implicitfast");
	}
	return TEXT("euler");
}
bool ParseCone(const FString& S, EMjCone& Out)
{
	if (S.Equals(TEXT("pyramidal"), ESearchCase::IgnoreCase))
	{
		Out = EMjCone::Pyramidal;
		return true;
	}
	if (S.Equals(TEXT("elliptic"), ESearchCase::IgnoreCase))
	{
		Out = EMjCone::Elliptic;
		return true;
	}
	return false;
}
FString ConeToString(EMjCone C)
{
	return C == EMjCone::Elliptic ? TEXT("elliptic") : TEXT("pyramidal");
}
bool ParseSolver(const FString& S, EMjSolver& Out)
{
	if (S.Equals(TEXT("pgs"), ESearchCase::IgnoreCase))
	{
		Out = EMjSolver::PGS;
		return true;
	}
	if (S.Equals(TEXT("cg"), ESearchCase::IgnoreCase))
	{
		Out = EMjSolver::CG;
		return true;
	}
	if (S.Equals(TEXT("newton"), ESearchCase::IgnoreCase))
	{
		Out = EMjSolver::Newton;
		return true;
	}
	return false;
}
FString SolverToString(EMjSolver S)
{
	switch (S)
	{
		case EMjSolver::PGS:
			return TEXT("pgs");
		case EMjSolver::CG:
			return TEXT("cg");
		case EMjSolver::Newton:
			return TEXT("newton");
	}
	return TEXT("newton");
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
		return MakeError(TEXT("not_ready"), TEXT("PhysicsEngine not initialised"));

	const TSharedPtr<FJsonObject>* OptsPtr = nullptr;
	if (!Req->TryGetObjectField(TEXT("options"), OptsPtr) || !OptsPtr || !(*OptsPtr).IsValid())
		return MakeError(TEXT("missing_field"), TEXT("set_sim_options requires 'options' object"));
	const TSharedPtr<FJsonObject>& Opts = *OptsPtr;

	FMjOptionGenerated& O = Mgr->PhysicsEngine->Options;

	double DNum = 0.0;
	if (Opts->TryGetNumberField(TEXT("timestep"), DNum))
	{
		O.Timestep = (float)DNum;
		O.bOverride_Timestep = true;
	}

	// Wire is MJ-native SI; FMjOptionGenerated stores UE cm/s² with Y-flip and
	// ApplyOverridesToModel reverses that, so pre-bake the inverse here.
	double V3[3];
	if (TryReadVec3(Opts, TEXT("gravity"), V3))
	{
		O.Gravity = FVector((float)(V3[0] * 100.0), (float)(-V3[1] * 100.0), (float)(V3[2] * 100.0));
		O.bOverride_Gravity = true;
	}
	if (TryReadVec3(Opts, TEXT("wind"), V3))
	{
		O.Wind = FVector((float)(V3[0] * 100.0), (float)(-V3[1] * 100.0), (float)(V3[2] * 100.0));
		O.bOverride_Wind = true;
	}
	if (TryReadVec3(Opts, TEXT("magnetic"), V3))
	{
		O.Magnetic = FVector((float)V3[0], (float)-V3[1], (float)V3[2]);
		O.bOverride_Magnetic = true;
	}

	if (Opts->TryGetNumberField(TEXT("density"), DNum))
	{
		O.Density = (float)DNum;
		O.bOverride_Density = true;
	}
	if (Opts->TryGetNumberField(TEXT("viscosity"), DNum))
	{
		O.Viscosity = (float)DNum;
		O.bOverride_Viscosity = true;
	}
	if (Opts->TryGetNumberField(TEXT("impratio"), DNum))
	{
		O.Impratio = (float)DNum;
		O.bOverride_Impratio = true;
	}
	if (Opts->TryGetNumberField(TEXT("tolerance"), DNum))
	{
		O.Tolerance = (float)DNum;
		O.bOverride_Tolerance = true;
	}

	int32 INum = 0;
	if (Opts->TryGetNumberField(TEXT("iterations"), INum))
	{
		O.Iterations = INum;
		O.bOverride_Iterations = true;
	}
	if (Opts->TryGetNumberField(TEXT("ls_iterations"), INum))
	{
		O.LsIterations = INum;
		O.bOverride_LsIterations = true;
	}

	FString SNum;
	if (Opts->TryGetStringField(TEXT("integrator"), SNum))
	{
		EMjIntegrator E;
		if (!ParseIntegrator(SNum, E))
			return MakeError(TEXT("bad_value"), FString::Printf(TEXT("unknown integrator '%s'"), *SNum));
		O.Integrator = E;
		O.bOverride_Integrator = true;
	}
	if (Opts->TryGetStringField(TEXT("cone"), SNum))
	{
		EMjCone E;
		if (!ParseCone(SNum, E))
			return MakeError(TEXT("bad_value"), FString::Printf(TEXT("unknown cone '%s'"), *SNum));
		O.Cone = E;
		O.bOverride_Cone = true;
	}
	if (Opts->TryGetStringField(TEXT("solver"), SNum))
	{
		EMjSolver E;
		if (!ParseSolver(SNum, E))
			return MakeError(TEXT("bad_value"), FString::Printf(TEXT("unknown solver '%s'"), *SNum));
		O.Solver = E;
		O.bOverride_Solver = true;
	}

	if (Opts->TryGetNumberField(TEXT("noslip_iterations"), INum))
	{
		O.NoslipIterations = INum;
		O.bOverride_NoslipIterations = true;
	}
	if (Opts->TryGetNumberField(TEXT("noslip_tolerance"), DNum))
	{
		O.NoslipTolerance = (float)DNum;
		O.bOverride_NoslipTolerance = true;
	}
	if (Opts->TryGetNumberField(TEXT("ccd_iterations"), INum))
	{
		O.CCD_Iterations = INum;
		O.bOverride_CCD_Iterations = true;
	}
	if (Opts->TryGetNumberField(TEXT("ccd_tolerance"), DNum))
	{
		O.CCD_Tolerance = (float)DNum;
		O.bOverride_CCD_Tolerance = true;
	}

	bool BNum = false;
	if (Opts->TryGetBoolField(TEXT("enable_multiccd"), BNum))
	{
		O.bEnableMultiCCD = BNum;
	}
	if (Opts->TryGetBoolField(TEXT("enable_sleep"), BNum))
	{
		O.bEnableSleep = BNum;
	}
	if (Opts->TryGetNumberField(TEXT("sleep_tolerance"), DNum))
	{
		O.SleepTolerance = (float)DNum;
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
		return MakeError(TEXT("not_ready"), TEXT("mjModel not compiled"));

	// Raw disable / enable bit masks. Values are bitwise-ORs of
	// mujoco/mjmodel.h mjtDisableBit / mjtEnableBit constants.
	// Applied BEFORE FMjOptionGenerated::ApplyOverridesToModel so any named
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

	O.ApplyOverridesToModel(m);

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
	Out->SetStringField(TEXT("integrator"), IntegratorToString((EMjIntegrator)m->opt.integrator));
	Out->SetStringField(TEXT("cone"), ConeToString((EMjCone)m->opt.cone));
	Out->SetStringField(TEXT("solver"), SolverToString((EMjSolver)m->opt.solver));
	Out->SetNumberField(TEXT("noslip_iterations"), m->opt.noslip_iterations);
	Out->SetNumberField(TEXT("noslip_tolerance"), m->opt.noslip_tolerance);
	Out->SetNumberField(TEXT("ccd_iterations"), m->opt.ccd_iterations);
	Out->SetNumberField(TEXT("ccd_tolerance"), m->opt.ccd_tolerance);

	constexpr int MJ_ENBL_MULTICCD = 1 << 4;
	constexpr int MJ_ENBL_SLEEP = 1 << 5;
	Out->SetBoolField(TEXT("enable_multiccd"), (m->opt.enableflags & MJ_ENBL_MULTICCD) != 0);
	Out->SetBoolField(TEXT("enable_sleep"), (m->opt.enableflags & MJ_ENBL_SLEEP) != 0);
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
		return MakeError(TEXT("not_ready"), TEXT("PhysicsEngine not initialised"));

	double Pct = 0.0;
	if (!Req->TryGetNumberField(TEXT("percent"), Pct))
		return MakeError(TEXT("missing_field"), TEXT("set_sim_speed requires 'percent'"));

	// Engine clamps internally (5..100); echo back so the caller sees what stuck.
	Mgr->PhysicsEngine->SetSimSpeed((float)Pct);
	const float Effective = FMath::Clamp((float)Pct, 5.0f, 100.0f);

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("set_sim_speed_ok"));
	Reply->SetNumberField(TEXT("percent"), Effective);
	return Reply;
}

// =============================================================================
// set_control_source
// =============================================================================

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleSetControlSource(const TSharedPtr<FJsonObject>& Req)
{
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr || !Mgr->PhysicsEngine)
		return MakeError(TEXT("not_ready"), TEXT("PhysicsEngine not initialised"));

	FString SourceStr;
	if (!Req->TryGetStringField(TEXT("source"), SourceStr))
		return MakeError(TEXT("missing_field"), TEXT("set_control_source requires 'source' (\"zmq\" | \"ui\")"));

	EControlSource NewSource;
	if (SourceStr.Equals(TEXT("zmq"), ESearchCase::IgnoreCase))
		NewSource = EControlSource::ZMQ;
	else if (SourceStr.Equals(TEXT("ui"), ESearchCase::IgnoreCase))
		NewSource = EControlSource::UI;
	else
		return MakeError(TEXT("bad_value"), FString::Printf(TEXT("unknown source '%s'"), *SourceStr));

	FString ArtName;
	Req->TryGetStringField(TEXT("articulation"), ArtName);

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("set_control_source_ok"));
	Reply->SetStringField(TEXT("source"), SourceStr.ToLower());

	if (ArtName.IsEmpty())
	{
		// Global: update engine + every articulation so the per-actor field
		// doesn't keep stale state after a global flip.
		Mgr->PhysicsEngine->SetControlSource(NewSource);
		for (AMjArticulation* Art : Mgr->GetAllArticulations())
		{
			if (Art)
				Art->ControlSource = (uint8)NewSource;
		}
		Reply->SetStringField(TEXT("scope"), TEXT("global"));
	}
	else
	{
		AMjArticulation* Art = Mgr->GetArticulation(ArtName);
		if (!Art)
			return MakeError(TEXT("unknown_articulation"), ArtName);
		Art->ControlSource = (uint8)NewSource;
		Reply->SetStringField(TEXT("scope"), TEXT("articulation"));
		Reply->SetStringField(TEXT("articulation"), ArtName);
	}
	return Reply;
}

// =============================================================================
// set_twist
// =============================================================================

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleSetTwist(const TSharedPtr<FJsonObject>& Req)
{
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr)
		return MakeError(TEXT("not_ready"), TEXT("Manager missing"));

	FString ArtName;
	if (!Req->TryGetStringField(TEXT("articulation"), ArtName))
		return MakeError(TEXT("missing_field"), TEXT("set_twist requires 'articulation'"));

	AMjArticulation* Art = Mgr->GetArticulation(ArtName);
	if (!Art)
		return MakeError(TEXT("unknown_articulation"), ArtName);

	UMjTwistController* TC = Art->FindComponentByClass<UMjTwistController>();
	if (!TC)
		return MakeError(TEXT("no_twist_controller"),
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

