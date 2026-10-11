#include "Workflows/InteractionInbox.h"

#include "Bridge/RequestLedger.h"
#include "Misc/Guid.h"
#include "Misc/ScopeLock.h"

static_assert(FURLabInteractionInbox::MaxPending ==
	FURLabRequestLedger::MaxPendingPerSession);
static_assert(FURLabInteractionInbox::MaxRetainedTickets ==
	FURLabRequestLedger::MaxEntriesPerSession);

namespace
{
bool FiniteVector(const FVector3d& Value)
{
	return FMath::IsFinite(Value.X) && FMath::IsFinite(Value.Y)
		&& FMath::IsFinite(Value.Z);
}

bool SameRequest(const FURLabInteractionWrenchRequest& A,
	const FURLabInteractionWrenchRequest& B)
{
	return A.GrantToken == B.GrantToken && A.Source == B.Source
		&& A.ViewerBootId == B.ViewerBootId && A.ClockEpoch == B.ClockEpoch
		&& A.BodyIndex == B.BodyIndex && A.ForceNewton == B.ForceNewton
		&& A.TorqueNewtonMetre == B.TorqueNewtonMetre;
}
}

bool FURLabInteractionSourceIdentity::IsValid() const
{
	return !ProducerBootId.IsEmpty() && !StreamId.IsEmpty()
		&& StreamGeneration != 0 && !SourceRevision.IsEmpty();
}

bool FURLabInteractionSourceIdentity::operator==(
	const FURLabInteractionSourceIdentity& Other) const
{
	return ProducerBootId == Other.ProducerBootId && StreamId == Other.StreamId
		&& StreamGeneration == Other.StreamGeneration
		&& SourceRevision == Other.SourceRevision;
}

bool FURLabInteractionControlIdentity::IsValid() const
{
	return !AuthorityId.IsEmpty() && Generation != 0 && !OpaqueProof.IsEmpty();
}

bool FURLabInteractionControlIdentity::operator==(
	const FURLabInteractionControlIdentity& Other) const
{
	return AuthorityId == Other.AuthorityId && Generation == Other.Generation
		&& OpaqueProof == Other.OpaqueProof;
}

bool FURLabInteractionGrant::IsValid() const
{
	return !Token.IsEmpty() && Source.IsValid() && !ViewerBootId.IsEmpty()
		&& FMath::IsFinite(ExpiresAtSeconds) && ExpiresAtSeconds > 0.0;
}

bool FURLabInteractionWrenchRequest::IsValid(FString& OutError) const
{
	OutError.Reset();
	if (Ticket.IsEmpty() || GrantToken.IsEmpty() || !Source.IsValid()
		|| ViewerBootId.IsEmpty() || BodyIndex < 0 || !FiniteVector(ForceNewton)
		|| !FiniteVector(TorqueNewtonMetre))
	{
		OutError = TEXT("interaction ticket, identity, target, or wrench is invalid");
		return false;
	}
	return true;
}

struct FURLabInteractionInbox::FState
{
	explicit FState(TSharedRef<FURLabRenderOwnerClock, ESPMode::ThreadSafe> InClock)
		: Clock(MoveTemp(InClock))
	{
	}

	struct FRecord
	{
		FURLabInteractionWrenchRequest Request;
		TSharedPtr<IURLabInteractionCompletionSink, ESPMode::ThreadSafe> CompletionSink;
		uint64 ApplicationId = 0;
		FURLabInteractionResult Result;
		bool bTerminal = false;
	};
	struct FGrantRecord
	{
		FURLabInteractionGrant Grant;
		FURLabInteractionControlIdentity Control;
	};

	mutable FCriticalSection Mutex;
	TSharedRef<FURLabRenderOwnerClock, ESPMode::ThreadSafe> Clock;
	TMap<FString, FGrantRecord> Grants;
	TSharedPtr<IURLabInteractionOwnerHandler, ESPMode::ThreadSafe> Handler;
	TArray<FString> Pending;
	TMap<FString, FRecord> Records;
	uint64 NextApplicationId = 1;
};

FURLabInteractionInbox::FURLabInteractionInbox(
	TSharedRef<FURLabRenderOwnerClock, ESPMode::ThreadSafe> InClock)
	: State(MakeUnique<FState>(MoveTemp(InClock)))
{
}

FURLabInteractionInbox::~FURLabInteractionInbox() = default;

bool FURLabInteractionInbox::IssueGrant(const FURLabInteractionOwnerContext& Context,
	const FString& ViewerBootId, double TtlSeconds,
	TSharedPtr<IURLabInteractionOwnerHandler, ESPMode::ThreadSafe> Handler,
	FURLabInteractionGrant& OutGrant, FString& OutError)
{
	OutError.Reset();
	if (!Context.Source.IsValid() || !Context.Control.IsValid()
		|| ViewerBootId.IsEmpty() || !Handler)
	{
		OutError = TEXT("interaction handler, completion sink, or identity is unavailable");
		return false;
	}
	if (!FMath::IsFinite(TtlSeconds) || TtlSeconds <= 0.0
		|| TtlSeconds > FURLabRenderOwnerAuthority::MaxLeaseSeconds)
	{
		OutError = TEXT("interaction grant TTL must be finite and in (0, 30] seconds");
		return false;
	}
	const double Now = State->Clock->NowSeconds();
	if (!FMath::IsFinite(Now) || !FMath::IsFinite(Now + TtlSeconds))
	{
		OutError = TEXT("interaction grant clock is invalid");
		return false;
	}
	FURLabInteractionGrant Grant;
	Grant.Token = FGuid::NewGuid().ToString(EGuidFormats::Digits).ToLower();
	Grant.Source = Context.Source;
	Grant.ViewerBootId = ViewerBootId;
	Grant.ClockEpoch = Context.ClockEpoch;
	Grant.ExpiresAtSeconds = Now + TtlSeconds;
	FScopeLock Lock(&State->Mutex);
	if (State->Grants.Num() >= MaxPending)
	{
		OutError = TEXT("interaction grant capacity is exhausted");
		return false;
	}
	FState::FGrantRecord GrantRecord;
	GrantRecord.Grant = Grant;
	GrantRecord.Control = Context.Control;
	State->Grants.Add(Grant.Token, MoveTemp(GrantRecord));
	State->Handler = MoveTemp(Handler);
	OutGrant = MoveTemp(Grant);
	return true;
}

void FURLabInteractionInbox::RevokeGrant()
{
	FScopeLock Lock(&State->Mutex);
	State->Grants.Reset();
}

bool FURLabInteractionInbox::RevokeGrant(const FString& ExactToken)
{
	FScopeLock Lock(&State->Mutex);
	return !ExactToken.IsEmpty() && State->Grants.Remove(ExactToken) == 1;
}

FURLabInteractionAdmissionResult FURLabInteractionInbox::Enqueue(
	const FURLabInteractionWrenchRequest& Request,
	TSharedPtr<IURLabInteractionCompletionSink, ESPMode::ThreadSafe> CompletionSink)
{
	FURLabInteractionAdmissionResult Result;
	if (!Request.IsValid(Result.Error)) return Result;
	FScopeLock Lock(&State->Mutex);
	if (!State->Handler)
	{
		Result.Admission = EURLabInteractionAdmission::Unsupported;
		Result.Error = TEXT("interaction handler is not enabled");
		return Result;
	}
	if (!CompletionSink)
	{
		Result.Error = TEXT("interaction completion sink is unavailable");
		return Result;
	}
	const FState::FGrantRecord* GrantRecord = State->Grants.Find(Request.GrantToken);
	if (!GrantRecord)
	{
		Result.Admission = EURLabInteractionAdmission::StaleGrant;
		Result.Error = TEXT("interaction grant has been revoked");
		return Result;
	}
	const FURLabInteractionGrant& Grant = GrantRecord->Grant;
	const double Now = State->Clock->NowSeconds();
	if (!FMath::IsFinite(Now) || Now >= Grant.ExpiresAtSeconds
		|| Request.GrantToken != Grant.Token || !(Request.Source == Grant.Source)
		|| Request.ViewerBootId != Grant.ViewerBootId
		|| Request.ClockEpoch != Grant.ClockEpoch)
	{
		Result.Admission = EURLabInteractionAdmission::StaleGrant;
		Result.Error = TEXT("interaction grant or source identity is stale");
		return Result;
	}
	if (const FState::FRecord* Existing = State->Records.Find(Request.Ticket))
	{
		if (!SameRequest(Existing->Request, Request))
		{
			Result.Admission = EURLabInteractionAdmission::Conflict;
			Result.Error = TEXT("interaction ticket was reused with different content");
		}
		else
		{
			Result.Admission = Existing->bTerminal
				? EURLabInteractionAdmission::ExistingTerminal
				: EURLabInteractionAdmission::ExistingPending;
			Result.ExistingResult = Existing->Result;
		}
		return Result;
	}
	if (State->Pending.Num() >= MaxPending || State->Records.Num() >= MaxRetainedTickets)
	{
		Result.Admission = EURLabInteractionAdmission::Capacity;
		Result.Error = TEXT("interaction inbox capacity is exhausted");
		return Result;
	}
	FState::FRecord Record;
	Record.Request = Request;
	Record.CompletionSink = MoveTemp(CompletionSink);
	State->Records.Add(Request.Ticket, MoveTemp(Record));
	State->Pending.Add(Request.Ticket);
	Result.Admission = EURLabInteractionAdmission::Accepted;
	return Result;
}

int32 FURLabInteractionInbox::Drain(const FURLabInteractionOwnerContext& CurrentContext,
	int32 MaxToDrain)
{
	int32 Drained = 0;
	while (Drained < FMath::Clamp(MaxToDrain, 0, MaxPending))
	{
		FString Ticket;
		FURLabInteractionWrenchRequest Request;
		TSharedPtr<IURLabInteractionOwnerHandler, ESPMode::ThreadSafe> Handler;
		TSharedPtr<IURLabInteractionCompletionSink, ESPMode::ThreadSafe> Sink;
		uint64 ApplicationId = 0;
		bool bStale = false;
		{
			FScopeLock Lock(&State->Mutex);
			if (State->Pending.IsEmpty()) break;
			Ticket = State->Pending[0];
			State->Pending.RemoveAt(0, 1, EAllowShrinking::No);
			FState::FRecord* Record = State->Records.Find(Ticket);
			if (!Record || Record->bTerminal) continue;
			Request = Record->Request;
			Handler = State->Handler;
			Sink = Record->CompletionSink;
			const double Now = State->Clock->NowSeconds();
			const FState::FGrantRecord* GrantRecord = State->Grants.Find(Request.GrantToken);
			bStale = !GrantRecord || !Handler || !Sink || !FMath::IsFinite(Now)
				|| Now >= GrantRecord->Grant.ExpiresAtSeconds
				|| !(Request.Source == GrantRecord->Grant.Source)
				|| Request.ViewerBootId != GrantRecord->Grant.ViewerBootId
				|| Request.ClockEpoch != GrantRecord->Grant.ClockEpoch
				|| !(CurrentContext.Source == GrantRecord->Grant.Source)
				|| !(CurrentContext.Control == GrantRecord->Control)
				|| CurrentContext.ClockEpoch != GrantRecord->Grant.ClockEpoch;
			if (!bStale)
			{
				if (State->NextApplicationId == 0) bStale = true;
				else ApplicationId = State->NextApplicationId++;
			}
		}

		FURLabInteractionResult Terminal;
		FString Error;
		if (bStale)
		{
			Terminal.Terminal = EURLabInteractionTerminal::RejectedStale;
			Terminal.Error = TEXT("interaction grant, source, control, or clock epoch is stale");
		}
		else if (!Handler->ValidateTarget(Request.BodyIndex, Error))
		{
			Terminal.Terminal = EURLabInteractionTerminal::RejectedTarget;
			Terminal.Error = MoveTemp(Error);
		}
		else if (!Handler->ApplyNextStepContribution(Request, ApplicationId, Error))
		{
			Terminal.Terminal = EURLabInteractionTerminal::ApplyFailed;
			Terminal.Error = MoveTemp(Error);
		}

		if (Terminal.Terminal != EURLabInteractionTerminal::None)
		{
			{
				FScopeLock Lock(&State->Mutex);
				if (FState::FRecord* Record = State->Records.Find(Ticket))
				{
					Record->Result = Terminal;
					Record->bTerminal = true;
				}
			}
			if (Sink) Sink->CompleteInteraction(Ticket, Terminal);
		}
		else
		{
			FScopeLock Lock(&State->Mutex);
			if (FState::FRecord* Record = State->Records.Find(Ticket))
				Record->ApplicationId = ApplicationId;
		}
		++Drained;
	}
	return Drained;
}

bool FURLabInteractionInbox::CompleteApplied(uint64 ApplicationId,
	const FURLabInteractionOwnerContext& CurrentContext,
	const FURLabInteractionAppliedFrame& AppliedFrame, FString& OutError)
{
	OutError.Reset();
	if (ApplicationId == 0 || AppliedFrame.FrameId == 0
		|| (AppliedFrame.bHasSourceTime && !FMath::IsFinite(AppliedFrame.SourceTime)))
	{
		OutError = TEXT("interaction applied frame is invalid");
		return false;
	}
	FString Ticket;
	TSharedPtr<IURLabInteractionCompletionSink, ESPMode::ThreadSafe> Sink;
	FURLabInteractionResult Result;
	{
		FScopeLock Lock(&State->Mutex);
		FState::FRecord* Match = nullptr;
		for (TPair<FString, FState::FRecord>& Entry : State->Records)
		{
			if (!Entry.Value.bTerminal && Entry.Value.ApplicationId == ApplicationId)
			{
				Ticket = Entry.Key;
				Match = &Entry.Value;
				break;
			}
		}
		if (!Match)
		{
			OutError = TEXT("interaction application is unknown or already complete");
			return false;
		}
		const double Now = State->Clock->NowSeconds();
		const FState::FGrantRecord* GrantRecord = State->Grants.Find(
			Match->Request.GrantToken);
		const bool bCurrent = GrantRecord && State->Handler
			&& FMath::IsFinite(Now) && Now < GrantRecord->Grant.ExpiresAtSeconds
			&& CurrentContext.Source == GrantRecord->Grant.Source
			&& CurrentContext.Control == GrantRecord->Control
			&& CurrentContext.ClockEpoch == GrantRecord->Grant.ClockEpoch
			&& AppliedFrame.ClockEpoch == CurrentContext.ClockEpoch;
		Result.Terminal = bCurrent ? EURLabInteractionTerminal::Applied
			: EURLabInteractionTerminal::RejectedStale;
		Result.AppliedFrame = AppliedFrame;
		if (!bCurrent)
			Result.Error = TEXT("interaction authority changed before applied completion");
		Match->Result = Result;
		Match->bTerminal = true;
		Sink = Match->CompletionSink;
	}
	if (Sink) Sink->CompleteInteraction(Ticket, Result);
	return true;
}

int32 FURLabInteractionInbox::NumPending() const
{
	FScopeLock Lock(&State->Mutex);
	return State->Pending.Num();
}
