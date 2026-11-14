// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#include "MuJoCo/Render/MjRenderKinematics.h"

#include "Misc/ScopeLock.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

FMjRenderKinematicsCandidate::FMjRenderKinematicsCandidate() = default;

FMjRenderKinematicsCandidate::~FMjRenderKinematicsCandidate()
{
	Reset();
}

FMjRenderKinematicsCandidate::FMjRenderKinematicsCandidate(
	FMjRenderKinematicsCandidate&& Other) noexcept
	: ModelCandidate(MoveTemp(Other.ModelCandidate))
	, Adapter(MoveTemp(Other.Adapter))
	, ScratchData(Other.ScratchData)
	, SceneRevision(MoveTemp(Other.SceneRevision))
	, Scene(MoveTemp(Other.Scene))
	, Resources(MoveTemp(Other.Resources))
	, InitialFrame(MoveTemp(Other.InitialFrame))
	, bNeutralSceneTaken(Other.bNeutralSceneTaken)
{
	Other.ScratchData = nullptr;
	Other.bNeutralSceneTaken = false;
}

FMjRenderKinematicsCandidate& FMjRenderKinematicsCandidate::operator=(
	FMjRenderKinematicsCandidate&& Other) noexcept
{
	if (this != &Other)
	{
		Reset();
		ModelCandidate = MoveTemp(Other.ModelCandidate);
		Adapter = MoveTemp(Other.Adapter);
		ScratchData = Other.ScratchData;
		SceneRevision = MoveTemp(Other.SceneRevision);
		Scene = MoveTemp(Other.Scene);
		Resources = MoveTemp(Other.Resources);
		InitialFrame = MoveTemp(Other.InitialFrame);
		bNeutralSceneTaken = Other.bNeutralSceneTaken;
		Other.ScratchData = nullptr;
		Other.bNeutralSceneTaken = false;
	}
	return *this;
}

void FMjRenderKinematicsCandidate::Reset()
{
	if (ScratchData)
	{
		mj_deleteData(ScratchData);
		ScratchData = nullptr;
	}
	Adapter.Reset();
	ModelCandidate = FMjOwnedModelCandidate();
	SceneRevision.Reset();
	Scene = FRenderScene();
	Resources = FRenderSceneResources();
	InitialFrame = FRenderFrame();
	bNeutralSceneTaken = false;
}

bool FMjRenderKinematicsCandidate::IsValid() const
{
	return ModelCandidate.IsValid() && Adapter.IsValid() && ScratchData
		&& ScratchData->signature == ModelCandidate.Model->signature
		&& !SceneRevision.IsEmpty();
}

bool FMjRenderKinematicsCandidate::Prepare(FMjOwnedModelCandidate&& InModelCandidate,
	const FString& InSceneRevision, const FMjRenderKinematicsOptions& Options,
	FMjRenderKinematicsCandidate& OutCandidate, FString& OutError)
{
	OutError.Reset();
	if (!InModelCandidate.IsValid() || InSceneRevision.IsEmpty())
	{
		OutError = TEXT("render kinematics preparation requires a model and nonempty scene revision");
		return false;
	}

	FMjRenderKinematicsCandidate Candidate;
	Candidate.ModelCandidate = MoveTemp(InModelCandidate);
	Candidate.SceneRevision = InSceneRevision;
	Candidate.Adapter = MakeUnique<FMjRenderSceneAdapter>();
	Candidate.ScratchData = mj_makeData(Candidate.ModelCandidate.Model);
	if (!Candidate.ScratchData)
	{
		OutError = TEXT("mj_makeData failed for render kinematics scratch state");
		return false;
	}
	if (!Candidate.Adapter->Extract(Candidate.ModelCandidate.Model, Candidate.ModelCandidate.Data, InSceneRevision,
		Options.VisibleGroupMask, Options.CameraMaxHeight,
		Options.bIncludeUserCamera, &Options.CameraOverrides,
		Candidate.Scene, Candidate.Resources, OutError)
		|| !Candidate.Adapter->FrameFromData(Candidate.ModelCandidate.Data, 0,
			Candidate.InitialFrame, OutError))
	{
		return false;
	}
	OutCandidate = MoveTemp(Candidate);
	return true;
}

bool FMjRenderKinematicsCandidate::Prepare(FMjModelSourcePayload&& Source,
	const FString& SceneRevision, const FMjRenderKinematicsOptions& Options,
	FMjRenderKinematicsCandidate& OutCandidate, FString& OutError)
{
	FMjOwnedModelCandidate ModelCandidate;
	if (!FMjOwnedModelCandidate::FromBytes(Source.Kind, Source.PrimaryBytes,
		Source.Assets, ModelCandidate, OutError, Source.Label))
	{
		return false;
	}
	return Prepare(MoveTemp(ModelCandidate), SceneRevision, Options,
		OutCandidate, OutError);
}

bool FMjRenderKinematicsCandidate::TakeNeutralScene(FRenderScene& OutScene,
	FRenderSceneResources& OutResources, FRenderFrame& OutInitialFrame, FString& OutError)
{
	if (!IsValid() || bNeutralSceneTaken)
	{
		OutError = bNeutralSceneTaken
			? TEXT("render kinematics neutral scene has already been transferred")
			: TEXT("render kinematics candidate is invalid");
		return false;
	}
	OutScene = MoveTemp(Scene);
	OutResources = MoveTemp(Resources);
	OutInitialFrame = MoveTemp(InitialFrame);
	bNeutralSceneTaken = true;
	OutError.Reset();
	return true;
}

struct FMjRenderKinematics::FInstalled
{
	~FInstalled()
	{
		FMjRenderSceneAdapter::FreeFlexScratch(FlexScratch);
		if (ScratchData) mj_deleteData(ScratchData);
		if (Model) mj_deleteModel(Model);
	}

	mjModel* Model = nullptr;
	mjData* ScratchData = nullptr;
	mjvScene* FlexScratch = nullptr;
	/** Model-default mocap poses, 7 values per mocap body. */
	TArray<double> DefaultMocap;
	TUniquePtr<FMjRenderSceneAdapter> Adapter;
	FString SceneRevision;
	FMjModelSourceInfo SourceInfo;
};

FMjRenderKinematics::FMjRenderKinematics() = default;

FMjRenderKinematics::~FMjRenderKinematics()
{
	TUniquePtr<FInstalled> Retired;
	{
		FScopeLock Lock(&Mutex);
		Retired = MoveTemp(Installed);
	}
	Retired.Reset();
}

bool FMjRenderKinematics::Commit(FMjRenderKinematicsCandidate&& Candidate,
	const FString& ExpectedCurrentSceneRevision, FString& OutError)
{
	if (!Candidate.IsValid())
	{
		OutError = TEXT("render kinematics candidate is not fully prepared");
		return false;
	}
	TUniquePtr<FInstalled> Replacement = MakeUnique<FInstalled>();
	TUniquePtr<FInstalled> Retired;
	mjData* RetiredPreparedData = nullptr;
	{
		FScopeLock Lock(&Mutex);
		const bool bInitial = ExpectedCurrentSceneRevision.IsEmpty();
		if ((bInitial && Installed) || (!bInitial && (!Installed
			|| Installed->SceneRevision != ExpectedCurrentSceneRevision)))
		{
			OutError = bInitial
				? TEXT("render kinematics already has an installed model")
				: TEXT("expected render scene revision is not active");
			return false;
		}
		if (Installed && Candidate.SceneRevision == Installed->SceneRevision)
		{
			OutError = TEXT("replacement render scene revision must be new");
			return false;
		}
		Replacement->Model = Candidate.ModelCandidate.Model;
		Replacement->ScratchData = Candidate.ScratchData;
		Replacement->Adapter = MoveTemp(Candidate.Adapter);
		Replacement->SceneRevision = Candidate.SceneRevision;
		Replacement->SourceInfo = MoveTemp(Candidate.ModelCandidate.SourceInfo);
		Replacement->FlexScratch = FMjRenderSceneAdapter::MakeFlexScratch(Replacement->Model);
		mj_resetData(Replacement->Model, Replacement->ScratchData);
		Replacement->DefaultMocap.SetNumUninitialized(7 * Replacement->Model->nmocap);
		for (int32 Mocap = 0; Mocap < Replacement->Model->nmocap; ++Mocap)
		{
			FMemory::Memcpy(Replacement->DefaultMocap.GetData() + 7 * Mocap,
				Replacement->ScratchData->mocap_pos + 3 * Mocap, 3 * sizeof(double));
			FMemory::Memcpy(Replacement->DefaultMocap.GetData() + 7 * Mocap + 3,
				Replacement->ScratchData->mocap_quat + 4 * Mocap, 4 * sizeof(double));
		}
		Candidate.ModelCandidate.Model = nullptr;
		RetiredPreparedData = Candidate.ModelCandidate.Data;
		Candidate.ModelCandidate.Data = nullptr;
		Candidate.ScratchData = nullptr;
		Retired = MoveTemp(Installed);
		Installed = MoveTemp(Replacement);
	}
	if (RetiredPreparedData) mj_deleteData(RetiredPreparedData);
	Retired.Reset();
	OutError.Reset();
	return true;
}

bool FMjRenderKinematics::RebaseSceneRevision(const FString& ExpectedRevision,
	const FRenderScene& PatchedScene, FString& OutError)
{
	FScopeLock Lock(&Mutex);
	if (!Installed || Installed->SceneRevision != ExpectedRevision)
	{
		OutError = TEXT("render kinematics scene revision changed before patch commit");
		return false;
	}
	if (!Installed->Adapter->RebaseSceneRevision(ExpectedRevision, PatchedScene, OutError))
		return false;
	Installed->SceneRevision = PatchedScene.SceneRevision;
	OutError.Reset();
	return true;
}

bool FMjRenderKinematics::FrameFromQpos(const FString& SceneRevision,
	TConstArrayView<double> Qpos, uint64 FrameId, double SourceTime,
	bool bHasSourceTime, uint64 ClockEpoch, FRenderFrame& OutFrame, FString& OutError,
	const FMjRenderQposExtras& Extras)
{
	FScopeLock Lock(&Mutex);
	if (!Installed || SceneRevision.IsEmpty() || SceneRevision != Installed->SceneRevision)
	{
		OutError = TEXT("qpos frame does not target the active render scene revision");
		return false;
	}
	if (Qpos.Num() != Installed->Model->nq)
	{
		OutError = FString::Printf(TEXT("qpos frame has width %d; expected %d"),
			Qpos.Num(), Installed->Model->nq);
		return false;
	}
	if ((bHasSourceTime && !FMath::IsFinite(SourceTime)))
	{
		OutError = TEXT("qpos frame source time is non-finite");
		return false;
	}
	for (int32 Index = 0; Index < Qpos.Num(); ++Index)
	{
		if (!FMath::IsFinite(Qpos[Index]))
		{
			OutError = FString::Printf(TEXT("qpos value %d is non-finite"), Index);
			return false;
		}
	}

	const mjModel* Model = Installed->Model;
	mjData* Data = Installed->ScratchData;
	if (!Extras.Mocap.IsEmpty() && Extras.Mocap.Num() != 7 * Model->nmocap)
	{
		OutError = FString::Printf(TEXT("qpos frame has %d mocap values; expected %d"),
			Extras.Mocap.Num(), 7 * Model->nmocap);
		return false;
	}
	const TConstArrayView<double> MocapSource = Extras.Mocap.IsEmpty()
		? TConstArrayView<double>(Installed->DefaultMocap) : Extras.Mocap;
	TArray<double, TInlineAllocator<28>> Mocap;
	Mocap.Append(MocapSource.GetData(), MocapSource.Num());
	for (int32 Body = 0; Body < Model->nmocap; ++Body)
	{
		double* Pose = Mocap.GetData() + 7 * Body;
		double NormSquared = 0.0;
		for (int32 Index = 0; Index < 7; ++Index)
		{
			if (!FMath::IsFinite(Pose[Index]))
			{
				OutError = FString::Printf(TEXT("mocap pose %d is non-finite"), Body);
				return false;
			}
			if (Index >= 3) NormSquared += Pose[Index] * Pose[Index];
		}
		if (FMath::Abs(FMath::Sqrt(NormSquared) - 1.0) > 1.e-3)
		{
			OutError = FString::Printf(TEXT("mocap quaternion %d is not near unit length"), Body);
			return false;
		}
		for (int32 Index = 3; Index < 7; ++Index) Pose[Index] /= FMath::Sqrt(NormSquared);
	}
	if (Extras.UserCameraSceneTransform.IsSet() && !Installed->Adapter->HasUserCamera())
	{
		OutError = TEXT("qpos frame supplies a user camera pose but the scene has no user camera");
		return false;
	}

	if (Installed->Adapter->RequiresDynamics())
	{
		mj_resetData(Model, Data);
	}
	if (Qpos.Num() > 0)
	{
		FMemory::Memcpy(Data->qpos, Qpos.GetData(), Qpos.Num() * sizeof(double));
	}
	for (int32 Body = 0; Body < Model->nmocap; ++Body)
	{
		FMemory::Memcpy(Data->mocap_pos + 3 * Body, Mocap.GetData() + 7 * Body,
			3 * sizeof(double));
		FMemory::Memcpy(Data->mocap_quat + 4 * Body, Mocap.GetData() + 7 * Body + 3,
			4 * sizeof(double));
	}
	if (Installed->Adapter->RequiresDynamics())
	{
		mj_forward(Model, Data);
	}
	else
	{
		mj_kinematics(Model, Data);
		mj_comPos(Model, Data);
		mj_camlight(Model, Data);
		if (Model->nflex > 0) mj_flex(Model, Data);
	}
	FRenderFrame CandidateFrame;
	if (!Installed->Adapter->FrameFromData(Data, FrameId, CandidateFrame, OutError,
		Installed->FlexScratch))
	{
		return false;
	}
	if (Extras.UserCameraSceneTransform.IsSet())
		CandidateFrame.UserCameraSceneTransform = Extras.UserCameraSceneTransform;
	CandidateFrame.Identity.SourceTime = bHasSourceTime ? SourceTime : 0.0;
	CandidateFrame.Identity.bHasSourceTime = bHasSourceTime;
	CandidateFrame.Identity.ClockEpoch = ClockEpoch;
	OutFrame = MoveTemp(CandidateFrame);
	OutError.Reset();
	return true;
}

bool FMjRenderKinematics::HasModel() const
{
	FScopeLock Lock(&Mutex);
	return Installed.IsValid();
}

FString FMjRenderKinematics::GetSceneRevision() const
{
	FScopeLock Lock(&Mutex);
	return Installed ? Installed->SceneRevision : FString();
}

int32 FMjRenderKinematics::GetQposWidth() const
{
	FScopeLock Lock(&Mutex);
	return Installed ? Installed->Model->nq : 0;
}

int32 FMjRenderKinematics::GetMocapCount() const
{
	FScopeLock Lock(&Mutex);
	return Installed ? Installed->Model->nmocap : 0;
}

bool FMjRenderKinematics::GetSourceInfo(FMjModelSourceInfo& OutSource) const
{
	FScopeLock Lock(&Mutex);
	if (!Installed) return false;
	OutSource = Installed->SourceInfo;
	return true;
}
