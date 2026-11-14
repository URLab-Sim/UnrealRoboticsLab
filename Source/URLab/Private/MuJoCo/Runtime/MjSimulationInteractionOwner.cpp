#include "MuJoCo/Runtime/MjSimulationInteractionOwner.h"

#include "MuJoCo/Runtime/MjSimulationRuntime.h"
#include "Transport/PoseStreamProducer.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

struct FMjSimulationInteractionOwner::FImpl
{
	struct FSavedBody
	{
		int32 BodyIndex = INDEX_NONE;
		double Values[6] = {};
	};

	FURLabInteractionInbox Inbox;
	TOptional<FURLabInteractionSourceIdentity> Source;
	mjModel* ActiveModel = nullptr;
	mjData* ActiveData = nullptr;
	FURLabInteractionOwnerContext ActiveContext;
	TArray<FSavedBody> SavedBodies;
	TArray<FSavedBody> LastStepWrenches;
	TArray<uint64> ApplicationIds;
	bool bKinematicsFresh = false;
	TFunction<bool()> InputPermission;

	bool InputPermitted() const
	{
		return !InputPermission || InputPermission();
	}

	void Restore()
	{
		if (ActiveData)
		{
			for (const FSavedBody& Saved : SavedBodies)
			{
				FMemory::Memcpy(ActiveData->xfrc_applied + Saved.BodyIndex * 6,
					Saved.Values, sizeof(Saved.Values));
			}
		}
		ActiveModel = nullptr;
		ActiveData = nullptr;
		SavedBodies.Reset();
		bKinematicsFresh = false;
	}

	/** World-frame wrench about the body COM for one request at the current pose. */
	void WorldWrench(const FURLabInteractionWrenchRequest& Request, FVector3d& OutForce,
		FVector3d& OutTorque)
	{
		OutForce = Request.ForceNewton;
		OutTorque = Request.TorqueNewtonMetre;
		if (!Request.bBodyFrame && !Request.bHasPoint) return;
		if (!bKinematicsFresh)
		{
			mj_kinematics(ActiveModel, ActiveData);
			mj_comPos(ActiveModel, ActiveData);
			bKinematicsFresh = true;
		}
		const int32 Body = Request.BodyIndex;
		const mjtNum* R = ActiveData->xmat + 9 * Body;
		auto Rotate = [R](const FVector3d& V)
		{
			return FVector3d(R[0] * V.X + R[1] * V.Y + R[2] * V.Z,
				R[3] * V.X + R[4] * V.Y + R[5] * V.Z,
				R[6] * V.X + R[7] * V.Y + R[8] * V.Z);
		};
		if (Request.bBodyFrame)
		{
			OutForce = Rotate(OutForce);
			OutTorque = Rotate(OutTorque);
		}
		if (Request.bHasPoint)
		{
			const mjtNum* Origin = ActiveData->xpos + 3 * Body;
			const mjtNum* Com = ActiveData->xipos + 3 * Body;
			const FVector3d Point = Request.bBodyFrame
				? FVector3d(Origin[0], Origin[1], Origin[2]) + Rotate(Request.PointMetre)
				: Request.PointMetre;
			OutTorque += FVector3d::CrossProduct(
				Point - FVector3d(Com[0], Com[1], Com[2]), OutForce);
		}
	}
};

FMjSimulationInteractionOwner::FMjSimulationInteractionOwner()
	: Impl(MakeUnique<FImpl>())
{
}

FMjSimulationInteractionOwner::~FMjSimulationInteractionOwner() = default;

bool FMjSimulationInteractionOwner::BindSource(
	const FURLabPoseStreamDescriptor& Descriptor, FString& OutError)
{
	FURLabInteractionSourceIdentity Source;
	Source.ProducerBootId = Descriptor.ProducerBootId;
	Source.StreamId = Descriptor.StreamId;
	Source.StreamGeneration = Descriptor.StreamGeneration;
	Source.SourceRevision = Descriptor.SourceRevision;
	if (!Source.IsValid())
	{
		OutError = TEXT("simulation interaction requires a committed pose-stream source");
		return false;
	}
	AbortStepLocked();
	Impl->Inbox.RevokeGrant();
	Impl->Source = MoveTemp(Source);
	OutError.Reset();
	return true;
}

void FMjSimulationInteractionOwner::RevokeSource()
{
	AbortStepLocked();
	Impl->Inbox.RevokeGrant();
	Impl->Source.Reset();
}

bool FMjSimulationInteractionOwner::IssueGrant(FMjSimulationRuntime& Runtime,
	const FMjSimulationControlHandle& Control,
	const FURLabInteractionSourceIdentity& ExpectedSource,
	const FString& ViewerBootId,
	double TtlSeconds,
	FURLabInteractionGrant& OutGrant, FString& OutError)
{
	if (!Impl->Source || !(Impl->Source.GetValue() == ExpectedSource))
	{
		OutError = TEXT("simulation interaction pose publisher identity is unavailable or stale");
		return false;
	}
	if (!Impl->InputPermitted())
	{
		OutError = TEXT("accept_input is not granted");
		return false;
	}
	FURLabInteractionOwnerContext Context;
	if (!Runtime.AuthorizeInteractionContext(Control, Impl->Source.GetValue(), Context,
		OutError))
	{
		return false;
	}
	return Impl->Inbox.IssueGrant(Context, ViewerBootId, TtlSeconds, AsShared(),
		OutGrant, OutError);
}

void FMjSimulationInteractionOwner::RevokeGrant()
{
	Impl->LastStepWrenches.Reset();
	Impl->Inbox.RevokeGrant();
}

bool FMjSimulationInteractionOwner::RevokeGrant(FMjSimulationRuntime& Runtime,
	const FMjSimulationControlHandle& Control,
	const FURLabInteractionSourceIdentity& ExpectedSource,
	const FString& ExactToken, FString& OutError)
{
	if (!Impl->Source || !(Impl->Source.GetValue() == ExpectedSource))
	{
		OutError = TEXT("simulation interaction pose publisher identity is unavailable or stale");
		return false;
	}
	FURLabInteractionOwnerContext Context;
	if (!Runtime.AuthorizeInteractionContext(Control, ExpectedSource, Context, OutError))
		return false;
	if (!Impl->Inbox.RevokeGrant(ExactToken))
	{
		OutError = TEXT("interaction grant token is not current");
		return false;
	}
	OutError.Reset();
	return true;
}

FURLabInteractionAdmissionResult FMjSimulationInteractionOwner::Enqueue(
	const FURLabInteractionWrenchRequest& Request,
	TSharedPtr<IURLabInteractionCompletionSink, ESPMode::ThreadSafe> CompletionSink)
{
	return Impl->Inbox.Enqueue(Request, MoveTemp(CompletionSink));
}

bool FMjSimulationInteractionOwner::ValidateTarget(int32 BodyIndex, FString& OutError)
{
	if (!Impl->ActiveModel || !Impl->ActiveData || BodyIndex < 0
		|| BodyIndex >= Impl->ActiveModel->nbody)
	{
		OutError = TEXT("interaction body index is outside the installed model");
		return false;
	}
	return true;
}

bool FMjSimulationInteractionOwner::ApplyNextStepContribution(
	const FURLabInteractionWrenchRequest& Request, uint64 ApplicationId,
	FString& OutError)
{
	if (!ValidateTarget(Request.BodyIndex, OutError) || ApplicationId == 0) return false;
	FImpl::FSavedBody* Saved = Impl->SavedBodies.FindByPredicate(
		[&](const FImpl::FSavedBody& Item) { return Item.BodyIndex == Request.BodyIndex; });
	if (!Saved)
	{
		FImpl::FSavedBody& Added = Impl->SavedBodies.AddDefaulted_GetRef();
		Added.BodyIndex = Request.BodyIndex;
		FMemory::Memcpy(Added.Values,
			Impl->ActiveData->xfrc_applied + Request.BodyIndex * 6, sizeof(Added.Values));
	}
	FVector3d Force;
	FVector3d Torque;
	Impl->WorldWrench(Request, Force, Torque);
	if (!FMath::IsFinite(Force.X) || !FMath::IsFinite(Force.Y) || !FMath::IsFinite(Force.Z)
		|| !FMath::IsFinite(Torque.X) || !FMath::IsFinite(Torque.Y) || !FMath::IsFinite(Torque.Z))
	{
		OutError = TEXT("interaction wrench is not finite in the world frame");
		return false;
	}
	double* Wrench = Impl->ActiveData->xfrc_applied + Request.BodyIndex * 6;
	Wrench[0] += Force.X;
	Wrench[1] += Force.Y;
	Wrench[2] += Force.Z;
	Wrench[3] += Torque.X;
	Wrench[4] += Torque.Y;
	Wrench[5] += Torque.Z;
	FImpl::FSavedBody* Last = Impl->LastStepWrenches.FindByPredicate(
		[&](const FImpl::FSavedBody& Item) { return Item.BodyIndex == Request.BodyIndex; });
	if (!Last)
	{
		Last = &Impl->LastStepWrenches.AddDefaulted_GetRef();
		Last->BodyIndex = Request.BodyIndex;
	}
	Last->Values[0] += Force.X;
	Last->Values[1] += Force.Y;
	Last->Values[2] += Force.Z;
	Last->Values[3] += Torque.X;
	Last->Values[4] += Torque.Y;
	Last->Values[5] += Torque.Z;
	Impl->ApplicationIds.AddUnique(ApplicationId);
	return true;
}

void FMjSimulationInteractionOwner::BeforeStepLocked(
	const FURLabInteractionOwnerContext& Context, mjModel_* Model, mjData_* Data)
{
	AbortStepLocked();
	Impl->LastStepWrenches.Reset();
	Impl->ActiveModel = Model;
	Impl->ActiveData = Data;
	FURLabInteractionOwnerContext BoundContext = Context;
	if (Impl->Source) BoundContext.Source = Impl->Source.GetValue();
	if (Data) BoundContext.SourceTime = Data->time;
	Impl->ActiveContext = BoundContext;
	if (!Impl->InputPermitted()) Impl->Inbox.RevokeGrant();
	Impl->Inbox.Drain(BoundContext);
}

void FMjSimulationInteractionOwner::AfterStepLocked(
	const FURLabInteractionOwnerContext& Context, uint64 FrameId, double SourceTime)
{
	FURLabInteractionOwnerContext CompletedContext = Impl->ActiveContext;
	CompletedContext.Control = Context.Control;
	CompletedContext.ClockEpoch = Context.ClockEpoch;
	Impl->Restore();
	FURLabInteractionAppliedFrame Applied;
	Applied.FrameId = FrameId;
	Applied.ClockEpoch = Context.ClockEpoch;
	Applied.SourceTime = SourceTime;
	Applied.bHasSourceTime = true;
	FString Error;
	for (uint64 ApplicationId : Impl->ApplicationIds)
		Impl->Inbox.CompleteApplied(ApplicationId, CompletedContext, Applied, Error);
	Impl->ApplicationIds.Reset();
}

void FMjSimulationInteractionOwner::AbortStepLocked()
{
	if (Impl->ActiveData) Impl->LastStepWrenches.Reset();
	Impl->Restore();
	for (uint64 ApplicationId : Impl->ApplicationIds)
		Impl->Inbox.AbortApplied(ApplicationId, TEXT("authoritative step did not complete"));
	Impl->ApplicationIds.Reset();
}

int32 FMjSimulationInteractionOwner::NumHolds() const
{
	return Impl->Inbox.NumHolds();
}

int32 FMjSimulationInteractionOwner::NumGrants() const
{
	return Impl->Inbox.NumGrants();
}

void FMjSimulationInteractionOwner::SetInputPermission(TFunction<bool()> Predicate)
{
	Impl->InputPermission = MoveTemp(Predicate);
}

void FMjSimulationInteractionOwner::OverlayLastStepWrenchesLocked(
	const mjModel_* Model, mjData_* Scratch) const
{
	if (!Model || !Scratch || !Scratch->xfrc_applied) return;
	for (const FImpl::FSavedBody& Applied : Impl->LastStepWrenches)
	{
		if (Applied.BodyIndex < 0 || Applied.BodyIndex >= Model->nbody) continue;
		double* Wrench = Scratch->xfrc_applied + Applied.BodyIndex * 6;
		for (int32 Index = 0; Index < 6; ++Index) Wrench[Index] += Applied.Values[Index];
	}
}
