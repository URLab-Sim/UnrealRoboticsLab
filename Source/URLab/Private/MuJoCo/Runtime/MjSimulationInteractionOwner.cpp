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
	TArray<uint64> ApplicationIds;

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
	double* Wrench = Impl->ActiveData->xfrc_applied + Request.BodyIndex * 6;
	Wrench[0] += Request.ForceNewton.X;
	Wrench[1] += Request.ForceNewton.Y;
	Wrench[2] += Request.ForceNewton.Z;
	Wrench[3] += Request.TorqueNewtonMetre.X;
	Wrench[4] += Request.TorqueNewtonMetre.Y;
	Wrench[5] += Request.TorqueNewtonMetre.Z;
	Impl->ApplicationIds.Add(ApplicationId);
	return true;
}

void FMjSimulationInteractionOwner::BeforeStepLocked(
	const FURLabInteractionOwnerContext& Context, mjModel_* Model, mjData_* Data)
{
	AbortStepLocked();
	Impl->ActiveModel = Model;
	Impl->ActiveData = Data;
	FURLabInteractionOwnerContext BoundContext = Context;
	if (Impl->Source) BoundContext.Source = Impl->Source.GetValue();
	Impl->ActiveContext = BoundContext;
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
	Impl->Restore();
	Impl->ApplicationIds.Reset();
}
