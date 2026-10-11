// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#include "MuJoCo/Runtime/MjSimulationRuntime.h"
#include "MuJoCo/Render/MjRenderSceneAdapter.h"

#include "Misc/Guid.h"
#include "Misc/ScopeLock.h"
#include "Misc/ScopeExit.h"
#include "MuJoCo/Runtime/MjSimulationInteractionOwner.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

struct FMjSimulationRuntime::FInstalledModel
{
	struct FControlAuthority
	{
		TSharedPtr<FMjSimulationControlSessionLifetime, ESPMode::ThreadSafe> Session;
		FString ClaimToken;
		uint64 ModelGeneration = 0;
		EMjSimulationControlSourceKind SourceKind = EMjSimulationControlSourceKind::Remote;
		double TtlSeconds = 0.0;
		double LastActivitySeconds = 0.0;
	};

	~FInstalledModel()
	{
		if (CartesianScratch) mj_deleteData(CartesianScratch);
		if (Data) mj_deleteData(Data);
		if (Model) mj_deleteModel(Model);
	}

	mjModel* Model = nullptr;
	mjData* Data = nullptr;
	mjData* CartesianScratch = nullptr;
	FMjModelDescriptor Descriptor;
	FMjModelSourceInfo Source;
	uint64 ClockEpoch = 0;
	uint64 StepSequence = 0;
	uint64 StateRevision = 0;
	TArray<double> StagedControls;
	uint64 ControlRevision = 0;
	double ClockDebtSeconds = 0.0;
	double DroppedTimeSeconds = 0.0;
	uint64 DroppedStepCount = 0;
	TOptional<FControlAuthority> Authority;
};

FMjSimulationRuntime::FMjSimulationRuntime()
	: ControlClock(FMjSimulationControlClock::System())
{
}

FMjSimulationRuntime::FMjSimulationRuntime(
	TSharedRef<FMjSimulationControlClock, ESPMode::ThreadSafe> InControlClock)
	: ControlClock(MoveTemp(InControlClock))
{
}

FMjSimulationRuntime::~FMjSimulationRuntime()
{
	TUniquePtr<FInstalledModel> Retired;
	{
		FScopeLock Lock(&Mutex);
		Retired = MoveTemp(Installed);
	}
	// MuJoCo destruction can be nontrivial and does not need the publication lock.
	Retired.Reset();
}

void FMjSimulationRuntime::SetInteractionOwner(
	TSharedPtr<FMjSimulationInteractionOwner, ESPMode::ThreadSafe> Owner)
{
	FScopeLock Lock(&Mutex);
	if (InteractionOwner) InteractionOwner->RevokeGrant();
	InteractionOwner = MoveTemp(Owner);
}

TSharedPtr<FMjSimulationInteractionOwner, ESPMode::ThreadSafe>
FMjSimulationRuntime::GetInteractionOwner() const
{
	FScopeLock Lock(&Mutex);
	return InteractionOwner;
}

FURLabInteractionControlIdentity FMjSimulationRuntime::InteractionControlLocked(
	const FInstalledModel& Model) const
{
	FURLabInteractionControlIdentity Result;
	if (!Model.Authority) return Result;
	Result.AuthorityId = Model.Authority->Session
		? Model.Authority->Session->GetId() : FString();
	Result.Generation = Model.Authority->ModelGeneration;
	Result.OpaqueProof = Model.Authority->ClaimToken;
	return Result;
}

bool FMjSimulationRuntime::AuthorizeInteractionContext(
	const FMjSimulationControlHandle& Handle,
	const FURLabInteractionSourceIdentity& Source,
	FURLabInteractionOwnerContext& OutContext, FString& OutMessage) const
{
	FScopeLock Lock(&Mutex);
	if (!Installed)
	{
		OutMessage = TEXT("no model is installed");
		return false;
	}
	const double Now = ControlNowLocked();
	if (ValidateControlLocked(*Installed, Handle, Now, OutMessage)
		!= EMjSimulationControlError::None)
	{
		return false;
	}
	if (!Source.IsValid())
	{
		OutMessage = TEXT("interaction pose-stream source identity is invalid");
		return false;
	}
	OutContext.Source = Source;
	OutContext.Control = InteractionControlLocked(*Installed);
	OutContext.ClockEpoch = Installed->ClockEpoch;
	OutMessage.Reset();
	return OutContext.Control.IsValid();
}

FMjSimulationInstallResult FMjSimulationRuntime::Install(FMjOwnedModelCandidate&& Candidate)
{
	return Install(MoveTemp(Candidate),
		[](const FMjSimulationInstallResult&, FString&) { return true; });
}

FMjSimulationInstallResult FMjSimulationRuntime::Install(FMjOwnedModelCandidate&& Candidate,
	TFunctionRef<bool(const FMjSimulationInstallResult& Proposed, FString& OutError)> Finalize)
{
	FMjSimulationInstallResult Result;
	if (!Candidate.IsValid())
	{
		Result.Error = TEXT("candidate is not fully prepared");
		return Result;
	}
	FMjModelObservation Initial;
	if (!MjModelRuntime::CaptureObservation(Candidate.Model, Candidate.Data,
		Candidate.Descriptor.Identity, /*epoch=*/1, /*step sequence=*/0, Initial))
	{
		Result.Error = TEXT("candidate initial simulation observation is invalid or non-finite");
		return Result;
	}

	TUniquePtr<FInstalledModel> Replacement = MakeUnique<FInstalledModel>();
	Replacement->Model = Candidate.Model;
	Replacement->Data = Candidate.Data;
	Replacement->Descriptor = MoveTemp(Candidate.Descriptor);
	Replacement->Source = MoveTemp(Candidate.SourceInfo);
	Replacement->StagedControls.SetNumUninitialized(Replacement->Descriptor.Identity.Nu);
	for (int32 Index = 0; Index < Replacement->StagedControls.Num(); ++Index)
	{
		Replacement->StagedControls[Index] = Replacement->Data->ctrl[Index];
	}
	Candidate.Model = nullptr;
	Candidate.Data = nullptr;
	if (!FMath::IsFinite(Replacement->Model->opt.timestep)
		|| Replacement->Model->opt.timestep <= 0.0)
	{
		Result.Error = TEXT("installed model timestep must be finite and positive");
		return Result;
	}
	Replacement->CartesianScratch = mj_makeData(Replacement->Model);
	if (!Replacement->CartesianScratch)
	{
		Result.Error = TEXT("mj_makeData failed for Cartesian snapshot scratch state");
		return Result;
	}

	TUniquePtr<FInstalledModel> Retired;
	{
		FScopeLock Lock(&Mutex);
		if (NextGeneration == 0 || NextGeneration > static_cast<uint64>(MAX_int64)
			|| NextClockEpoch == 0 || NextClockEpoch > static_cast<uint64>(MAX_int64)
			|| NextStateRevision == 0 || NextStateRevision > static_cast<uint64>(MAX_int64))
		{
			Result.Error = TEXT("runtime identity counters exhausted their signed wire range");
			return Result;
		}

		FMjSimulationInstallResult Proposed;
		Proposed.Generation = NextGeneration;
		Proposed.ClockEpoch = NextClockEpoch;
		Proposed.StateRevision = NextStateRevision;
		Proposed.Identity = Replacement->Descriptor.Identity;
		Proposed.Identity.Generation = Proposed.Generation;
		Proposed.Source = Replacement->Source;
		FString FinalizeError;
		if (!Finalize(Proposed, FinalizeError))
		{
			Result.Error = FinalizeError.IsEmpty()
				? TEXT("simulation install precommit finalizer rejected the candidate")
				: MoveTemp(FinalizeError);
			return Result;
		}

		Replacement->Descriptor.Identity.Generation = NextGeneration++;
		Replacement->ClockEpoch = NextClockEpoch++;
		Replacement->StepSequence = 0;
		Replacement->StateRevision = NextStateRevision++;
		Retired = MoveTemp(Installed);
		Installed = MoveTemp(Replacement);

		Result.bCommitted = true;
		Result.Generation = Installed->Descriptor.Identity.Generation;
		Result.ClockEpoch = Installed->ClockEpoch;
		Result.StateRevision = Installed->StateRevision;
		Result.Identity = Installed->Descriptor.Identity;
		Result.Source = Installed->Source;
	}
	Retired.Reset();
	return Result;
}

bool FMjSimulationRuntime::HasModel() const
{
	FScopeLock Lock(&Mutex);
	return Installed.IsValid();
}

bool FMjSimulationRuntime::GetSourceInfo(FMjModelSourceInfo& OutSource) const
{
	FScopeLock Lock(&Mutex);
	if (!Installed) return false;
	OutSource = Installed->Source;
	return true;
}

bool FMjSimulationRuntime::GetDescriptor(FMjModelDescriptor& OutDescriptor) const
{
	FScopeLock Lock(&Mutex);
	if (!Installed) return false;
	OutDescriptor = Installed->Descriptor;
	return true;
}

bool FMjSimulationRuntime::GetObservation(FMjModelObservation& OutObservation) const
{
	FScopeLock Lock(&Mutex);
	return Installed && MjModelRuntime::CaptureObservation(Installed->Model, Installed->Data,
		Installed->Descriptor.Identity, Installed->ClockEpoch, Installed->StepSequence,
		OutObservation);
}

EMjModelStateError FMjSimulationRuntime::CaptureState(uint32 StateMask, FMjModelState& OutState) const
{
	FScopeLock Lock(&Mutex);
	if (!Installed) return EMjModelStateError::NoModel;
	return MjModelRuntime::CaptureState(Installed->Model, Installed->Data,
		Installed->Descriptor.Identity, Installed->ClockEpoch, StateMask, OutState);
}

double FMjSimulationRuntime::ControlNowLocked() const
{
	return ControlClock->NowSeconds();
}

bool FMjSimulationRuntime::ExpireControlLocked(FInstalledModel& Model, double Now) const
{
	if (!Model.Authority.IsSet()) return false;
	const FInstalledModel::FControlAuthority& Authority = Model.Authority.GetValue();
	const bool bSessionActive = Authority.Session.IsValid()
		&& Authority.Session->IsActiveAt(Now);
	const bool bLeaseActive = FMath::IsFinite(Now)
		&& (Authority.TtlSeconds == 0.0
			|| Now < Authority.LastActivitySeconds
			|| Now - Authority.LastActivitySeconds < Authority.TtlSeconds);
	if (bSessionActive && bLeaseActive) return false;
	Model.Authority.Reset();
	return true;
}

FMjSimulationControlStatus FMjSimulationRuntime::ControlStatusLocked(
	FInstalledModel& Model, double Now) const
{
	ExpireControlLocked(Model, Now);
	FMjSimulationControlStatus Status;
	Status.ModelGeneration = Model.Descriptor.Identity.Generation;
	Status.ControlRevision = Model.ControlRevision;
	if (!Model.Authority.IsSet()) return Status;

	const FInstalledModel::FControlAuthority& Authority = Model.Authority.GetValue();
	Status.bClaimed = true;
	Status.SourceKind = Authority.SourceKind;
	Status.TtlSeconds = Authority.TtlSeconds;
	Status.bExpires = Authority.TtlSeconds > 0.0;
	if (Status.bExpires)
	{
		const double Elapsed = FMath::Max(0.0, Now - Authority.LastActivitySeconds);
		Status.SecondsRemaining = FMath::Max(0.0, Authority.TtlSeconds - Elapsed);
	}
	return Status;
}

FMjSimulationControlStatus FMjSimulationRuntime::GetControlStatus() const
{
	FScopeLock Lock(&Mutex);
	if (!Installed) return FMjSimulationControlStatus();
	return ControlStatusLocked(*Installed, ControlNowLocked());
}

bool FMjSimulationRuntime::IsValidClockMode(EMjSimulationClockMode Mode) const
{
	return Mode == EMjSimulationClockMode::Stepped
		|| Mode == EMjSimulationClockMode::FreeRun;
}

FMjSimulationClockStatus FMjSimulationRuntime::ClockStatusLocked(
	const FInstalledModel* Model) const
{
	FMjSimulationClockStatus Status;
	Status.Mode = ClockMode;
	Status.MaxStepsPerAdvance = MaxFreeRunStepsPerAdvance;
	Status.MaxRetainedDebtSeconds = MaxFreeRunRetainedDebtSeconds;
	if (!Model) return Status;
	Status.ModelGeneration = Model->Descriptor.Identity.Generation;
	Status.TimeStepSeconds = Model->Model->opt.timestep;
	Status.MaxRetainedDebtSeconds = FMath::Max(
		MaxFreeRunRetainedDebtSeconds, Status.TimeStepSeconds);
	Status.AccumulatedDebtSeconds = Model->ClockDebtSeconds;
	Status.DroppedTimeSeconds = Model->DroppedTimeSeconds;
	Status.DroppedStepCount = Model->DroppedStepCount;
	return Status;
}

FMjSimulationClockStatus FMjSimulationRuntime::GetClockStatus() const
{
	FScopeLock Lock(&Mutex);
	return ClockStatusLocked(Installed.Get());
}

void FMjSimulationRuntime::ClearClockDebtLocked(FInstalledModel& Model)
{
	Model.ClockDebtSeconds = 0.0;
}

bool FMjSimulationRuntime::CanAdvanceStateRevisionLocked(FString& OutError) const
{
	if (NextStateRevision == 0 || NextStateRevision > static_cast<uint64>(MAX_int64))
	{
		OutError = TEXT("state revision exhausted its signed wire range");
		return false;
	}
	OutError.Reset();
	return true;
}

void FMjSimulationRuntime::CommitStateRevisionLocked(FInstalledModel& Model)
{
	check(NextStateRevision > 0 && NextStateRevision <= static_cast<uint64>(MAX_int64));
	Model.StateRevision = NextStateRevision++;
}

FMjSimulationClockChangeResult FMjSimulationRuntime::ConfigureLocalClockMode(
	EMjSimulationClockMode Mode, uint64 ExpectedModelGeneration)
{
	FScopeLock Lock(&Mutex);
	FMjSimulationClockChangeResult Result;
	if (!Installed)
	{
		Result.Error = EMjSimulationClockError::NoModel;
		Result.Message = TEXT("no model is installed");
	}
	else if (!IsValidClockMode(Mode))
	{
		Result.Error = EMjSimulationClockError::InvalidMode;
		Result.Message = TEXT("simulation clock mode is invalid");
	}
	else if (ExpectedModelGeneration != Installed->Descriptor.Identity.Generation)
	{
		Result.Error = EMjSimulationClockError::WrongModelGeneration;
		Result.Message = TEXT("clock configuration targets a different model generation");
	}
	else if (ClockMode != Mode)
	{
		ClockMode = Mode;
		ClearClockDebtLocked(*Installed);
		Result.bCommitted = true;
	}
	Result.Status = ClockStatusLocked(Installed.Get());
	return Result;
}

FMjSimulationClockChangeResult FMjSimulationRuntime::AuthorizedSetClockMode(
	const FMjSimulationControlHandle& Handle, EMjSimulationClockMode Mode)
{
	FScopeLock Lock(&Mutex);
	FMjSimulationClockChangeResult Result;
	if (!Installed)
	{
		Result.Error = EMjSimulationClockError::NoModel;
		Result.Message = TEXT("no model is installed");
		Result.Status = ClockStatusLocked(nullptr);
		return Result;
	}
	FInstalledModel& Model = *Installed;
	const double Now = ControlNowLocked();
	Result.AuthorityError = ValidateControlLocked(Model, Handle, Now, Result.Message);
	if (Result.AuthorityError == EMjSimulationControlError::None && !IsValidClockMode(Mode))
	{
		Result.Error = EMjSimulationClockError::InvalidMode;
		Result.Message = TEXT("simulation clock mode is invalid");
	}
	if (Result.IsOk() && ClockMode != Mode)
	{
		ClockMode = Mode;
		ClearClockDebtLocked(Model);
		RefreshControlLocked(Model, Now);
		Result.bCommitted = true;
	}
	Result.Status = ClockStatusLocked(&Model);
	return Result;
}

FMjSimulationAdvanceResult FMjSimulationRuntime::AdvanceFreeRun(double ElapsedSeconds)
{
	FScopeLock Lock(&Mutex);
	FMjSimulationAdvanceResult Result;
	if (!FMath::IsFinite(ElapsedSeconds) || ElapsedSeconds < 0.0)
	{
		Result.Error = EMjSimulationClockError::InvalidElapsed;
		Result.Message = TEXT("free-run elapsed time must be finite and nonnegative");
		Result.Status = ClockStatusLocked(Installed.Get());
		return Result;
	}
	if (!Installed)
	{
		Result.Error = EMjSimulationClockError::NoModel;
		Result.Message = TEXT("no model is installed");
		Result.Status = ClockStatusLocked(nullptr);
		return Result;
	}
	FInstalledModel& Model = *Installed;
	if (ClockMode != EMjSimulationClockMode::FreeRun)
	{
		Result.Error = EMjSimulationClockError::WrongMode;
		Result.Message = TEXT("simulation clock is in stepped mode");
		Result.Status = ClockStatusLocked(&Model);
		return Result;
	}
	const double TimeStep = Model.Model->opt.timestep;
	if (!FMath::IsFinite(TimeStep) || TimeStep <= 0.0)
	{
		Result.Error = EMjSimulationClockError::InvalidTimeStep;
		Result.Message = TEXT("installed model timestep is not finite and positive");
		Result.Status = ClockStatusLocked(&Model);
		return Result;
	}
	const double Available = Model.ClockDebtSeconds + ElapsedSeconds;
	if (!FMath::IsFinite(Available))
	{
		Result.Error = EMjSimulationClockError::InvalidElapsed;
		Result.Message = TEXT("free-run elapsed time overflows the clock accumulator");
		Result.Status = ClockStatusLocked(&Model);
		return Result;
	}
	const double StepQuotient = Available / TimeStep;
	if (!FMath::IsFinite(StepQuotient))
	{
		Result.Error = EMjSimulationClockError::CounterExhausted;
		Result.Message = TEXT("free-run available step count exhausted its range");
		Result.Status = ClockStatusLocked(&Model);
		return Result;
	}
	const double WholeSteps = FMath::FloorToDouble(StepQuotient);
	const uint32 Steps = WholeSteps >= static_cast<double>(MaxFreeRunStepsPerAdvance)
		? MaxFreeRunStepsPerAdvance : static_cast<uint32>(WholeSteps);
	const double Remaining = FMath::Max(0.0, Available - static_cast<double>(Steps) * TimeStep);
	const double EffectiveDebtCap = FMath::Max(MaxFreeRunRetainedDebtSeconds, TimeStep);
	const double Retained = FMath::Min(Remaining, EffectiveDebtCap);
	const double DroppedTime = FMath::Max(0.0, Remaining - Retained);
	const double NewDroppedTime = Model.DroppedTimeSeconds + DroppedTime;
	const double NewDroppedStepQuotient = NewDroppedTime / TimeStep;
	constexpr double SignedCounterLimit = 9223372036854775808.0; // exactly 2^63
	if (!FMath::IsFinite(NewDroppedTime) || !FMath::IsFinite(NewDroppedStepQuotient)
		|| NewDroppedStepQuotient >= SignedCounterLimit)
	{
		Result.Error = EMjSimulationClockError::CounterExhausted;
		Result.Message = TEXT("free-run dropped-time counters exhausted their range");
		Result.Status = ClockStatusLocked(&Model);
		return Result;
	}
	const uint64 NewDroppedStepCount = static_cast<uint64>(
		FMath::FloorToDouble(NewDroppedStepQuotient));
	if (NewDroppedStepCount < Model.DroppedStepCount)
	{
		Result.Error = EMjSimulationClockError::CounterExhausted;
		Result.Message = TEXT("free-run dropped-step counter moved backwards");
		Result.Status = ClockStatusLocked(&Model);
		return Result;
	}
	if (Steps > 0)
	{
		FString StepError;
		if (!StepLocked(Model, Steps, StepError))
		{
			Result.Error = StepError.Contains(TEXT("timestep"))
				? EMjSimulationClockError::InvalidTimeStep
				: EMjSimulationClockError::CounterExhausted;
			Result.Message = MoveTemp(StepError);
			Result.Status = ClockStatusLocked(&Model);
			return Result;
		}
	}
	Model.ClockDebtSeconds = Retained;
	Model.DroppedTimeSeconds = NewDroppedTime;
	const uint64 DroppedSteps = NewDroppedStepCount - Model.DroppedStepCount;
	Model.DroppedStepCount = NewDroppedStepCount;
	Result.bCommitted = ElapsedSeconds > 0.0 || Steps > 0 || DroppedTime > 0.0;
	Result.IntegratedStepCount = Steps;
	Result.DroppedTimeSeconds = DroppedTime;
	Result.DroppedStepCount = DroppedSteps;
	Result.Status = ClockStatusLocked(&Model);
	return Result;
}

EMjSimulationSnapshotError FMjSimulationRuntime::CaptureCartesianSnapshot(
	FMjSimulationCartesianSnapshot& OutSnapshot) const
{
	FScopeLock Lock(&Mutex);
	if (!Installed) return EMjSimulationSnapshotError::NoModel;
	FInstalledModel& Model = *Installed;
	mjData* Scratch = Model.CartesianScratch;
	if (!Model.Model || !Model.Data || !Scratch
		|| Model.Data->signature != Model.Model->signature
		|| Scratch->signature != Model.Model->signature
		|| Model.Model->nbody < 0 || Model.Model->nbody > MAX_int32 / 4
		|| Model.Model->ncam < 0 || Model.Model->ncam > MAX_int32 / 9
		|| Model.Model->nlight < 0 || Model.Model->nlight > MAX_int32 / 3
		|| Model.Model->nq < 0
		|| Model.Model->nmocap < 0 || !Model.Data->qpos || !Scratch->qpos)
	{
		return EMjSimulationSnapshotError::InvalidNativeState;
	}
	Scratch->time = Model.Data->time;
	mju_copy(Scratch->qpos, Model.Data->qpos, Model.Model->nq);
	if (Model.Model->nmocap > 0)
	{
		if (!Model.Data->mocap_pos || !Model.Data->mocap_quat
			|| !Scratch->mocap_pos || !Scratch->mocap_quat)
		{
			return EMjSimulationSnapshotError::InvalidNativeState;
		}
		mju_copy(Scratch->mocap_pos, Model.Data->mocap_pos, 3 * Model.Model->nmocap);
		mju_copy(Scratch->mocap_quat, Model.Data->mocap_quat, 4 * Model.Model->nmocap);
	}
	mj_kinematics(Model.Model, Scratch);
	mj_comPos(Model.Model, Scratch);
	mj_camlight(Model.Model, Scratch);
	if ((Model.Model->nbody > 0 && (!Scratch->xpos || !Scratch->xquat))
		|| (Model.Model->ncam > 0 && (!Scratch->cam_xpos || !Scratch->cam_xmat))
		|| (Model.Model->nlight > 0 && (!Scratch->light_xpos || !Scratch->light_xdir)))
	{
		return EMjSimulationSnapshotError::InvalidNativeState;
	}

	FMjSimulationCartesianSnapshot Candidate;
	Candidate.Identity.Model = Model.Descriptor.Identity;
	Candidate.Identity.ClockEpoch = Model.ClockEpoch;
	Candidate.Identity.StepSequence = Model.StepSequence;
	Candidate.Identity.StateRevision = Model.StateRevision;
	Candidate.Identity.Time = Model.Data->time;
	if (Model.Model->nbody > 0)
	{
		Candidate.BodyPositions.Append(Scratch->xpos, 3 * Model.Model->nbody);
		Candidate.BodyQuaternions.Append(Scratch->xquat, 4 * Model.Model->nbody);
	}
	if (Model.Model->ncam > 0)
	{
		Candidate.CameraPositions.Append(Scratch->cam_xpos, 3 * Model.Model->ncam);
	}
	Candidate.CameraQuaternions.SetNumUninitialized(4 * Model.Model->ncam);
	for (int32 CameraIndex = 0; CameraIndex < Model.Model->ncam; ++CameraIndex)
	{
		mju_mat2Quat(Candidate.CameraQuaternions.GetData() + 4 * CameraIndex,
			Scratch->cam_xmat + 9 * CameraIndex);
	}
	if (Model.Model->nlight > 0)
	{
		Candidate.LightPositions.Append(Scratch->light_xpos, 3 * Model.Model->nlight);
		Candidate.LightDirections.Append(Scratch->light_xdir, 3 * Model.Model->nlight);
	}
	auto AllFinite = [](const TArray<double>& Values)
	{
		for (const double Value : Values) if (!FMath::IsFinite(Value)) return false;
		return true;
	};
	if (!FMath::IsFinite(Candidate.Identity.Time)
		|| !AllFinite(Candidate.BodyPositions) || !AllFinite(Candidate.BodyQuaternions)
		|| !AllFinite(Candidate.CameraPositions) || !AllFinite(Candidate.CameraQuaternions)
		|| !AllFinite(Candidate.LightPositions) || !AllFinite(Candidate.LightDirections))
	{
		return EMjSimulationSnapshotError::NonFinite;
	}
	OutSnapshot = MoveTemp(Candidate);
	return EMjSimulationSnapshotError::None;
}

bool FMjSimulationRuntime::CaptureRenderFrame(FMjRenderSceneAdapter& Adapter,
	uint64 ExpectedGeneration, uint64 FrameId, FRenderFrame& OutFrame,
	FString& OutError) const
{
	FScopeLock Lock(&Mutex);
	if (!Installed || Installed->Descriptor.Identity.Generation != ExpectedGeneration
		|| !Installed->Data || !Installed->Model || !Installed->CartesianScratch
		|| Installed->Data->signature != Installed->Model->signature
		|| Installed->CartesianScratch->signature != Installed->Model->signature
		|| (Installed->Model->nq > 0 && (!Installed->Data->qpos
			|| !Installed->CartesianScratch->qpos))
		|| (Installed->Model->nv > 0 && (!Installed->Data->qvel
			|| !Installed->CartesianScratch->qvel))
		|| (Installed->Model->nu > 0 && (!Installed->Data->ctrl
			|| !Installed->CartesianScratch->ctrl))
		|| (Installed->Model->nbody > 0 && (!Installed->Data->xfrc_applied
			|| !Installed->CartesianScratch->xfrc_applied)))
	{
		OutError = TEXT("render frame model generation is no longer installed");
		return false;
	}
	// mj_step leaves position-dependent fields at the pre-integration qpos.
	// Copy the complete state once and refresh only position stages. This
	// neither calls controller callbacks nor mutates the live mjData.
	mjData* Scratch = Installed->CartesianScratch;
	const mjModel* Model = Installed->Model;
	if (!mj_copyData(Scratch, Model, Installed->Data))
	{
		OutError = TEXT("MuJoCo render scratch copy failed");
		return false;
	}
	mj_kinematics(Model, Scratch);
	mj_comPos(Model, Scratch);
	mj_camlight(Model, Scratch);
	mj_flex(Model, Scratch);
	FRenderFrame Candidate;
	if (!Adapter.FrameFromData(Scratch, FrameId, Candidate, OutError))
		return false;
	Candidate.Identity.ClockEpoch = Installed->ClockEpoch;
	OutFrame = MoveTemp(Candidate);
	OutError.Reset();
	return true;
}

bool FMjSimulationRuntime::ValidateControlTtl(double TtlSeconds,
	EMjSimulationControlSourceKind SourceKind, FString& OutMessage) const
{
	if (SourceKind != EMjSimulationControlSourceKind::Local
		&& SourceKind != EMjSimulationControlSourceKind::Remote)
	{
		OutMessage = TEXT("control source kind is invalid");
		return false;
	}
	if (!FMath::IsFinite(TtlSeconds) || TtlSeconds < 0.0)
	{
		OutMessage = TEXT("control TTL must be finite and nonnegative");
		return false;
	}
	if (SourceKind == EMjSimulationControlSourceKind::Remote
		&& (TtlSeconds <= 0.0 || TtlSeconds > MaxRemoteControlTtlSeconds))
	{
		OutMessage = TEXT("remote control TTL must be greater than zero and at most 300 seconds");
		return false;
	}
	OutMessage.Reset();
	return true;
}

EMjSimulationControlError FMjSimulationRuntime::ValidateControlLocked(
	FInstalledModel& Model, const FMjSimulationControlHandle& Handle,
	double Now, FString& OutMessage) const
{
	if (!Handle.IsValid())
	{
		OutMessage = TEXT("control handle is incomplete");
		return EMjSimulationControlError::InvalidToken;
	}
	if (Handle.ModelGeneration != Model.Descriptor.Identity.Generation)
	{
		OutMessage = TEXT("control handle targets a different model generation");
		return EMjSimulationControlError::WrongModelGeneration;
	}
	if (!Model.Authority.IsSet())
	{
		OutMessage = TEXT("no control source is currently claimed");
		return EMjSimulationControlError::NotClaimed;
	}

	const FInstalledModel::FControlAuthority& Authority = Model.Authority.GetValue();
	if (!Authority.Session.IsValid() || !Authority.Session->IsActiveAt(Now))
	{
		Model.Authority.Reset();
		OutMessage = TEXT("control session is inactive or expired");
		return EMjSimulationControlError::InactiveSession;
	}
	if (!FMath::IsFinite(Now)
		|| (Authority.TtlSeconds > 0.0
			&& Now >= Authority.LastActivitySeconds
			&& Now - Authority.LastActivitySeconds >= Authority.TtlSeconds))
	{
		Model.Authority.Reset();
		OutMessage = TEXT("control lease expired");
		return EMjSimulationControlError::Expired;
	}
	if (Authority.ModelGeneration != Handle.ModelGeneration
		|| Authority.Session->GetId() != Handle.SessionId
		|| Authority.ClaimToken != Handle.ClaimToken)
	{
		OutMessage = TEXT("control claim token is not authoritative");
		return EMjSimulationControlError::InvalidToken;
	}
	OutMessage.Reset();
	return EMjSimulationControlError::None;
}

FMjSimulationControlClaimResult FMjSimulationRuntime::ClaimControl(
	const FMjSimulationControlClaimRequest& Request)
{
	FScopeLock Lock(&Mutex);
	FMjSimulationControlClaimResult Result;
	if (!Installed)
	{
		Result.Error = EMjSimulationControlError::NoModel;
		Result.Message = TEXT("no model is installed");
		return Result;
	}

	FInstalledModel& Model = *Installed;
	const double Now = ControlNowLocked();
	if (Request.SourceKind != EMjSimulationControlSourceKind::Local
		&& Request.SourceKind != EMjSimulationControlSourceKind::Remote)
	{
		Result.Error = EMjSimulationControlError::InvalidSession;
		Result.Message = TEXT("control source kind is invalid");
	}
	else if (!Request.Session.IsValid() || Request.Session->GetId().IsEmpty()
		|| !Request.Session->UsesClock(ControlClock))
	{
		Result.Error = EMjSimulationControlError::InvalidSession;
		Result.Message = TEXT("control session is missing or uses a different clock");
	}
	else if ((Request.SourceKind == EMjSimulationControlSourceKind::Remote)
		!= Request.Session->IsExpiring())
	{
		Result.Error = EMjSimulationControlError::InvalidSession;
		Result.Message = TEXT("control source kind does not match session lifetime policy");
	}
	else if (!Request.Session->IsActiveAt(Now))
	{
		Result.Error = EMjSimulationControlError::InactiveSession;
		Result.Message = TEXT("control session is inactive or expired");
	}
	else if (Request.ExpectedModelGeneration != Model.Descriptor.Identity.Generation)
	{
		Result.Error = EMjSimulationControlError::WrongModelGeneration;
		Result.Message = TEXT("control claim targets a different model generation");
	}
	else if (!ValidateControlTtl(Request.TtlSeconds, Request.SourceKind, Result.Message))
	{
		Result.Error = EMjSimulationControlError::InvalidTtl;
	}
	else
	{
		ExpireControlLocked(Model, Now);
		if (Model.Authority.IsSet())
		{
			Result.Error = EMjSimulationControlError::AlreadyClaimed;
			Result.Message = TEXT("the installed model already has a control source");
		}
		else
		{
			FInstalledModel::FControlAuthority Authority;
			Authority.Session = Request.Session;
			Authority.ClaimToken = FGuid::NewGuid().ToString(EGuidFormats::DigitsWithHyphensLower);
			Authority.ModelGeneration = Model.Descriptor.Identity.Generation;
			Authority.SourceKind = Request.SourceKind;
			Authority.TtlSeconds = Request.TtlSeconds;
			Authority.LastActivitySeconds = Now;
			Model.Authority = MoveTemp(Authority);
			Result.Handle.SessionId = Request.Session->GetId();
			Result.Handle.ClaimToken = Model.Authority->ClaimToken;
			Result.Handle.ModelGeneration = Model.Descriptor.Identity.Generation;
		}
	}
	Result.Status = ControlStatusLocked(Model, Now);
	return Result;
}

FMjSimulationControlLeaseResult FMjSimulationRuntime::RenewControl(
	const FMjSimulationControlHandle& Handle, double TtlSeconds)
{
	FScopeLock Lock(&Mutex);
	FMjSimulationControlLeaseResult Result;
	if (!Installed)
	{
		Result.Error = EMjSimulationControlError::NoModel;
		Result.Message = TEXT("no model is installed");
		return Result;
	}
	FInstalledModel& Model = *Installed;
	const double Now = ControlNowLocked();
	Result.Error = ValidateControlLocked(Model, Handle, Now, Result.Message);
	if (Result.Error == EMjSimulationControlError::None
		&& !ValidateControlTtl(TtlSeconds, Model.Authority->SourceKind, Result.Message))
	{
		Result.Error = EMjSimulationControlError::InvalidTtl;
	}
	if (Result.Error == EMjSimulationControlError::None)
	{
		Model.Authority->TtlSeconds = TtlSeconds;
		Model.Authority->LastActivitySeconds = Now;
		Result.bChanged = true;
	}
	Result.Status = ControlStatusLocked(Model, Now);
	return Result;
}

FMjSimulationControlLeaseResult FMjSimulationRuntime::ReleaseControl(
	const FMjSimulationControlHandle& Handle)
{
	FScopeLock Lock(&Mutex);
	FMjSimulationControlLeaseResult Result;
	if (!Installed)
	{
		Result.Error = EMjSimulationControlError::NoModel;
		Result.Message = TEXT("no model is installed");
		return Result;
	}
	FInstalledModel& Model = *Installed;
	const double Now = ControlNowLocked();
	Result.Error = ValidateControlLocked(Model, Handle, Now, Result.Message);
	if (Result.Error == EMjSimulationControlError::None)
	{
		Model.Authority.Reset();
		Result.bChanged = true;
	}
	Result.Status = ControlStatusLocked(Model, Now);
	return Result;
}

EMjSimulationControlError FMjSimulationRuntime::AuthorizeControl(
	const FMjSimulationControlHandle& Handle, FString& OutMessage) const
{
	FScopeLock Lock(&Mutex);
	OutMessage.Reset();
	if (!Installed)
	{
		OutMessage = TEXT("no model is installed");
		return EMjSimulationControlError::NoModel;
	}
	FInstalledModel& Model = *Installed;
	return ValidateControlLocked(Model, Handle, ControlNowLocked(), OutMessage);
}

FMjSimulationControlLeaseResult FMjSimulationRuntime::RevokeControlForSession(
	const TSharedRef<FMjSimulationControlSessionLifetime, ESPMode::ThreadSafe>& Session)
{
	FScopeLock Lock(&Mutex);
	FMjSimulationControlLeaseResult Result;
	if (!Installed)
	{
		Result.Error = EMjSimulationControlError::NoModel;
		Result.Message = TEXT("no model is installed");
		return Result;
	}
	FInstalledModel& Model = *Installed;
	if (Model.Authority.IsSet() && Model.Authority->Session.IsValid()
		&& Model.Authority->Session.Get() == &Session.Get())
	{
		Model.Authority.Reset();
		Result.bChanged = true;
	}
	Result.Status = ControlStatusLocked(Model, ControlNowLocked());
	return Result;
}

void FMjSimulationRuntime::RefreshControlLocked(FInstalledModel& Model, double Now)
{
	if (Model.Authority.IsSet()) Model.Authority->LastActivitySeconds = Now;
}

bool FMjSimulationRuntime::CaptureMutationObservationLocked(FInstalledModel& Model,
	double Now, FMjSimulationMutationResult& OutResult) const
{
	OutResult.Status = ControlStatusLocked(Model, Now);
	if (MjModelRuntime::CaptureObservation(Model.Model, Model.Data,
		Model.Descriptor.Identity, Model.ClockEpoch, Model.StepSequence,
		OutResult.Observation))
	{
		return true;
	}
	OutResult.Error = EMjSimulationControlError::ObservationUnavailable;
	OutResult.Message = TEXT("mutation committed but its observation could not be captured");
	return false;
}

namespace
{
EMjSimulationControlError ValidateControlSpan(const FMjActuatorDescriptor& Actuator,
	TConstArrayView<double> Values, FString& OutMessage)
{
	if (Values.Num() != Actuator.ControlWidth
		|| Actuator.Controls.Num() != Actuator.ControlWidth)
	{
		OutMessage = FString::Printf(TEXT("actuator '%s' requires exactly %d controls"),
			*Actuator.Name, Actuator.ControlWidth);
		return EMjSimulationControlError::WrongControlWidth;
	}
	for (int32 Offset = 0; Offset < Values.Num(); ++Offset)
	{
		const double Value = Values[Offset];
		if (!FMath::IsFinite(Value))
		{
			OutMessage = FString::Printf(TEXT("actuator '%s' control %d is not finite"),
				*Actuator.Name, Offset);
			return EMjSimulationControlError::NonFinite;
		}
		const FMjActuatorControlDescriptor& Control = Actuator.Controls[Offset];
		if (Control.bLimited && (Value < Control.RangeMin || Value > Control.RangeMax))
		{
			OutMessage = FString::Printf(
				TEXT("actuator '%s' control %d is outside [%g, %g]"),
				*Actuator.Name, Offset, Control.RangeMin, Control.RangeMax);
			return EMjSimulationControlError::OutOfRange;
		}
	}
	return EMjSimulationControlError::None;
}
}

FMjSimulationMutationResult FMjSimulationRuntime::SetControlVector(
	const FMjSimulationControlHandle& Handle, TConstArrayView<double> Values)
{
	FScopeLock Lock(&Mutex);
	FMjSimulationMutationResult Result;
	if (!Installed)
	{
		Result.Error = EMjSimulationControlError::NoModel;
		Result.Message = TEXT("no model is installed");
		return Result;
	}
	FInstalledModel& Model = *Installed;
	const double Now = ControlNowLocked();
	Result.Error = ValidateControlLocked(Model, Handle, Now, Result.Message);
	if (Result.Error == EMjSimulationControlError::None
		&& Values.Num() != Model.Descriptor.Identity.Nu)
	{
		Result.Error = EMjSimulationControlError::WrongControlWidth;
		Result.Message = FString::Printf(TEXT("control vector requires exactly %d values"),
			Model.Descriptor.Identity.Nu);
	}
	if (Result.Error == EMjSimulationControlError::None)
	{
		for (const FMjActuatorDescriptor& Actuator : Model.Descriptor.Actuators)
		{
			Result.Error = ValidateControlSpan(Actuator,
				Values.Slice(Actuator.ControlAddress, Actuator.ControlWidth), Result.Message);
			if (Result.Error != EMjSimulationControlError::None) break;
		}
	}
	FString RevisionError;
	if (Result.Error == EMjSimulationControlError::None
		&& (Model.ControlRevision >= static_cast<uint64>(MAX_int64)
			|| !CanAdvanceStateRevisionLocked(RevisionError)))
	{
		Result.Error = EMjSimulationControlError::CounterExhausted;
		Result.Message = RevisionError.IsEmpty()
			? TEXT("control revision exhausted its signed wire range") : RevisionError;
	}
	if (Result.Error == EMjSimulationControlError::None)
	{
		Model.StagedControls.Reset(Values.Num());
		Model.StagedControls.Append(Values.GetData(), Values.Num());
		for (int32 Index = 0; Index < Values.Num(); ++Index) Model.Data->ctrl[Index] = Values[Index];
		++Model.ControlRevision;
		CommitStateRevisionLocked(Model);
		RefreshControlLocked(Model, Now);
		Result.bCommitted = true;
		CaptureMutationObservationLocked(Model, Now, Result);
	}
	else
	{
		Result.Status = ControlStatusLocked(Model, Now);
	}
	return Result;
}

FMjSimulationMutationResult FMjSimulationRuntime::SetNamedActuatorControls(
	const FMjSimulationControlHandle& Handle,
	const TMap<FString, TArray<double>>& ValuesByActuator)
{
	FScopeLock Lock(&Mutex);
	FMjSimulationMutationResult Result;
	if (!Installed)
	{
		Result.Error = EMjSimulationControlError::NoModel;
		Result.Message = TEXT("no model is installed");
		return Result;
	}
	FInstalledModel& Model = *Installed;
	const double Now = ControlNowLocked();
	Result.Error = ValidateControlLocked(Model, Handle, Now, Result.Message);
	TArray<double> Candidate = Model.StagedControls;
	if (Result.Error == EMjSimulationControlError::None && ValuesByActuator.Num() == 0)
	{
		Result.Error = EMjSimulationControlError::WrongControlWidth;
		Result.Message = TEXT("named actuator update is empty");
	}
	if (Result.Error == EMjSimulationControlError::None)
	{
		for (const TPair<FString, TArray<double>>& Entry : ValuesByActuator)
		{
			if (Entry.Key.IsEmpty())
			{
				Result.Error = EMjSimulationControlError::UnknownActuator;
				Result.Message = TEXT("named actuator update contains an empty name");
				break;
			}
			const FMjActuatorDescriptor* Match = nullptr;
			for (const FMjActuatorDescriptor& Actuator : Model.Descriptor.Actuators)
			{
				if (Actuator.Name.Equals(Entry.Key, ESearchCase::CaseSensitive))
				{
					Match = &Actuator;
					break;
				}
			}
			if (!Match)
			{
				Result.Error = EMjSimulationControlError::UnknownActuator;
				Result.Message = FString::Printf(TEXT("unknown actuator '%s'"), *Entry.Key);
				break;
			}
			Result.Error = ValidateControlSpan(*Match, Entry.Value, Result.Message);
			if (Result.Error != EMjSimulationControlError::None) break;
			for (int32 Offset = 0; Offset < Match->ControlWidth; ++Offset)
			{
				Candidate[Match->ControlAddress + Offset] = Entry.Value[Offset];
			}
		}
	}
	FString RevisionError;
	if (Result.Error == EMjSimulationControlError::None
		&& (Model.ControlRevision >= static_cast<uint64>(MAX_int64)
			|| !CanAdvanceStateRevisionLocked(RevisionError)))
	{
		Result.Error = EMjSimulationControlError::CounterExhausted;
		Result.Message = RevisionError.IsEmpty()
			? TEXT("control revision exhausted its signed wire range") : RevisionError;
	}
	if (Result.Error == EMjSimulationControlError::None)
	{
		Model.StagedControls = MoveTemp(Candidate);
		for (int32 Index = 0; Index < Model.StagedControls.Num(); ++Index)
		{
			Model.Data->ctrl[Index] = Model.StagedControls[Index];
		}
		++Model.ControlRevision;
		CommitStateRevisionLocked(Model);
		RefreshControlLocked(Model, Now);
		Result.bCommitted = true;
		CaptureMutationObservationLocked(Model, Now, Result);
	}
	else
	{
		Result.Status = ControlStatusLocked(Model, Now);
	}
	return Result;
}

EMjModelStateError FMjSimulationRuntime::ApplyStateLocked(
	FInstalledModel& Model, const FMjModelState& State)
{
	const EMjModelStateError Validation = MjModelRuntime::ValidateState(
		Model.Model, Model.Descriptor.Identity, State);
	if (Validation != EMjModelStateError::None) return Validation;
	const bool bReplacesControls = (State.StateMask & static_cast<uint32>(mjSTATE_CTRL)) != 0;
	FString RevisionError;
	if (NextClockEpoch == 0 || NextClockEpoch > static_cast<uint64>(MAX_int64)
		|| !CanAdvanceStateRevisionLocked(RevisionError)
		|| (bReplacesControls && Model.ControlRevision >= static_cast<uint64>(MAX_int64)))
	{
		return EMjModelStateError::CounterExhausted;
	}
	const EMjModelStateError Error = MjModelRuntime::ApplyState(Model.Model,
		Model.Data, Model.Descriptor.Identity, State);
	if (Error != EMjModelStateError::None) return Error;
	if (bReplacesControls)
	{
		for (int32 Index = 0; Index < Model.StagedControls.Num(); ++Index)
		{
			Model.StagedControls[Index] = Model.Data->ctrl[Index];
		}
		++Model.ControlRevision;
	}
	else
	{
		for (int32 Index = 0; Index < Model.StagedControls.Num(); ++Index)
		{
			Model.Data->ctrl[Index] = Model.StagedControls[Index];
		}
	}
	Model.ClockEpoch = NextClockEpoch++;
	Model.StepSequence = 0;
	ClearClockDebtLocked(Model);
	CommitStateRevisionLocked(Model);
	return EMjModelStateError::None;
}

EMjModelStateError FMjSimulationRuntime::ApplyState(const FMjModelState& State)
{
	FScopeLock Lock(&Mutex);
	if (!Installed) return EMjModelStateError::NoModel;
	return ApplyStateLocked(*Installed, State);
}

bool FMjSimulationRuntime::ResetLocked(FInstalledModel& Model, FString& OutError)
{
	if (NextClockEpoch == 0 || NextClockEpoch > static_cast<uint64>(MAX_int64))
	{
		OutError = TEXT("runtime clock epoch exhausted its signed wire range");
		return false;
	}
	if (Model.ControlRevision >= static_cast<uint64>(MAX_int64))
	{
		OutError = TEXT("control revision exhausted its signed wire range");
		return false;
	}
	if (!CanAdvanceStateRevisionLocked(OutError)) return false;
	mj_resetData(Model.Model, Model.Data);
	for (double& Value : Model.StagedControls) Value = 0.0;
	for (int32 Index = 0; Index < Model.StagedControls.Num(); ++Index)
	{
		Model.Data->ctrl[Index] = 0.0;
	}
	++Model.ControlRevision;
	mj_forward(Model.Model, Model.Data);
	Model.ClockEpoch = NextClockEpoch++;
	Model.StepSequence = 0;
	ClearClockDebtLocked(Model);
	CommitStateRevisionLocked(Model);
	OutError.Reset();
	return true;
}

bool FMjSimulationRuntime::Reset(FString& OutError)
{
	FScopeLock Lock(&Mutex);
	if (!Installed)
	{
		OutError = TEXT("no model is installed");
		return false;
	}
	return ResetLocked(*Installed, OutError);
}

bool FMjSimulationRuntime::StepLocked(FInstalledModel& Model, uint32 Count, FString& OutError)
{
	if (Count == 0)
	{
		OutError = TEXT("step count must be positive");
		return false;
	}
	if (Model.StepSequence > static_cast<uint64>(MAX_int64) - Count)
	{
		OutError = TEXT("step sequence would exceed the signed wire range");
		return false;
	}
	if (!CanAdvanceStateRevisionLocked(OutError)) return false;
	for (int32 ControlIndex = 0; ControlIndex < Model.StagedControls.Num(); ++ControlIndex)
	{
		Model.Data->ctrl[ControlIndex] = Model.StagedControls[ControlIndex];
	}
	for (uint32 Index = 0; Index < Count; ++Index)
	{
		const TSharedPtr<FMjSimulationInteractionOwner, ESPMode::ThreadSafe> Owner =
			InteractionOwner;
		FURLabInteractionOwnerContext InteractionContext;
		if (Owner)
		{
			InteractionContext.Control = InteractionControlLocked(Model);
			InteractionContext.ClockEpoch = Model.ClockEpoch;
			Owner->BeforeStepLocked(InteractionContext, Model.Model, Model.Data);
		}
		{
			ON_SCOPE_EXIT { if (Owner) Owner->AbortStepLocked(); };
			mj_step(Model.Model, Model.Data);
			++Model.StepSequence;
			if (Owner)
			{
				Owner->AfterStepLocked(InteractionContext, Model.StepSequence,
					Model.Data->time);
			}
		}
	}
	CommitStateRevisionLocked(Model);
	OutError.Reset();
	return true;
}

bool FMjSimulationRuntime::Step(uint32 Count, FString& OutError)
{
	FScopeLock Lock(&Mutex);
	if (!Installed)
	{
		OutError = TEXT("no model is installed");
		return false;
	}
	if (ClockMode != EMjSimulationClockMode::Stepped)
	{
		OutError = TEXT("explicit step is unavailable while the simulation clock is in free-run mode");
		return false;
	}
	return StepLocked(*Installed, Count, OutError);
}

FMjSimulationMutationResult FMjSimulationRuntime::AuthorizedStep(
	const FMjSimulationControlHandle& Handle, uint32 Count)
{
	FScopeLock Lock(&Mutex);
	FMjSimulationMutationResult Result;
	if (!Installed)
	{
		Result.Error = EMjSimulationControlError::NoModel;
		Result.Message = TEXT("no model is installed");
		return Result;
	}
	FInstalledModel& Model = *Installed;
	const double Now = ControlNowLocked();
	Result.Error = ValidateControlLocked(Model, Handle, Now, Result.Message);
	if (Result.Error == EMjSimulationControlError::None
		&& (Count == 0 || Count > MaxAuthorizedStepCount))
	{
		Result.Error = EMjSimulationControlError::InvalidStepCount;
		Result.Message = TEXT("authorized step count must be between 1 and 1024");
	}
	if (Result.Error == EMjSimulationControlError::None
		&& ClockMode != EMjSimulationClockMode::Stepped)
	{
		Result.Error = EMjSimulationControlError::WrongClockMode;
		Result.Message = TEXT("explicit step is unavailable while the simulation clock is in free-run mode");
	}
	if (Result.Error == EMjSimulationControlError::None)
	{
		FString StepError;
		if (!StepLocked(Model, Count, StepError))
		{
			Result.Error = StepError.Contains(TEXT("wire range"))
				? EMjSimulationControlError::CounterExhausted
				: EMjSimulationControlError::InvalidStepCount;
			Result.Message = MoveTemp(StepError);
		}
	}
	if (Result.Error == EMjSimulationControlError::None)
	{
		RefreshControlLocked(Model, Now);
		Result.bCommitted = true;
		CaptureMutationObservationLocked(Model, Now, Result);
	}
	else
	{
		Result.Status = ControlStatusLocked(Model, Now);
	}
	return Result;
}

FMjSimulationMutationResult FMjSimulationRuntime::AuthorizedForward(
	const FMjSimulationControlHandle& Handle)
{
	FScopeLock Lock(&Mutex);
	FMjSimulationMutationResult Result;
	if (!Installed)
	{
		Result.Error = EMjSimulationControlError::NoModel;
		Result.Message = TEXT("no model is installed");
		return Result;
	}
	FInstalledModel& Model = *Installed;
	const double Now = ControlNowLocked();
	Result.Error = ValidateControlLocked(Model, Handle, Now, Result.Message);
	if (Result.Error == EMjSimulationControlError::None)
	{
		FString RevisionError;
		if (!CanAdvanceStateRevisionLocked(RevisionError))
		{
			Result.Error = EMjSimulationControlError::CounterExhausted;
			Result.Message = MoveTemp(RevisionError);
		}
	}
	if (Result.Error == EMjSimulationControlError::None)
	{
		mj_forward(Model.Model, Model.Data);
		CommitStateRevisionLocked(Model);
		RefreshControlLocked(Model, Now);
		Result.bCommitted = true;
		CaptureMutationObservationLocked(Model, Now, Result);
	}
	else
	{
		Result.Status = ControlStatusLocked(Model, Now);
	}
	return Result;
}

FMjSimulationMutationResult FMjSimulationRuntime::AuthorizedReset(
	const FMjSimulationControlHandle& Handle)
{
	FScopeLock Lock(&Mutex);
	FMjSimulationMutationResult Result;
	if (!Installed)
	{
		Result.Error = EMjSimulationControlError::NoModel;
		Result.Message = TEXT("no model is installed");
		return Result;
	}
	FInstalledModel& Model = *Installed;
	const double Now = ControlNowLocked();
	Result.Error = ValidateControlLocked(Model, Handle, Now, Result.Message);
	if (Result.Error == EMjSimulationControlError::None)
	{
		FString ResetError;
		if (!ResetLocked(Model, ResetError))
		{
			Result.Error = EMjSimulationControlError::CounterExhausted;
			Result.Message = MoveTemp(ResetError);
		}
	}
	if (Result.Error == EMjSimulationControlError::None)
	{
		RefreshControlLocked(Model, Now);
		Result.bCommitted = true;
		CaptureMutationObservationLocked(Model, Now, Result);
	}
	else
	{
		Result.Status = ControlStatusLocked(Model, Now);
	}
	return Result;
}

FMjSimulationMutationResult FMjSimulationRuntime::AuthorizedApplyState(
	const FMjSimulationControlHandle& Handle, const FMjModelState& State)
{
	FScopeLock Lock(&Mutex);
	FMjSimulationMutationResult Result;
	if (!Installed)
	{
		Result.Error = EMjSimulationControlError::NoModel;
		Result.StateError = EMjModelStateError::NoModel;
		Result.Message = TEXT("no model is installed");
		return Result;
	}
	FInstalledModel& Model = *Installed;
	const double Now = ControlNowLocked();
	Result.Error = ValidateControlLocked(Model, Handle, Now, Result.Message);
	if (Result.Error == EMjSimulationControlError::None)
	{
		Result.StateError = ApplyStateLocked(Model, State);
		if (Result.StateError != EMjModelStateError::None)
		{
			Result.Error = Result.StateError == EMjModelStateError::CounterExhausted
				? EMjSimulationControlError::CounterExhausted
				: EMjSimulationControlError::InvalidState;
			Result.Message = TEXT("state payload was rejected");
		}
	}
	if (Result.Error == EMjSimulationControlError::None)
	{
		RefreshControlLocked(Model, Now);
		Result.bCommitted = true;
		CaptureMutationObservationLocked(Model, Now, Result);
	}
	else
	{
		Result.Status = ControlStatusLocked(Model, Now);
	}
	return Result;
}

uint64 FMjSimulationRuntime::GetModelGeneration() const
{
	FScopeLock Lock(&Mutex);
	return Installed ? Installed->Descriptor.Identity.Generation : 0;
}

uint64 FMjSimulationRuntime::GetClockEpoch() const
{
	FScopeLock Lock(&Mutex);
	return Installed ? Installed->ClockEpoch : 0;
}

uint64 FMjSimulationRuntime::GetStepSequence() const
{
	FScopeLock Lock(&Mutex);
	return Installed ? Installed->StepSequence : 0;
}

uint64 FMjSimulationRuntime::GetStateRevision() const
{
	FScopeLock Lock(&Mutex);
	return Installed ? Installed->StateRevision : 0;
}
