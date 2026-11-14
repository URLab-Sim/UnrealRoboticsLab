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
constexpr double HoldTimeTolerance = 1e-9;

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
		&& A.TorqueNewtonMetre == B.TorqueNewtonMetre && A.bBodyFrame == B.bBodyFrame
		&& A.bHasPoint == B.bHasPoint && (!A.bHasPoint || A.PointMetre == B.PointMetre)
		&& A.DurationSeconds == B.DurationSeconds && A.Channel == B.Channel
		&& A.InteractionId == B.InteractionId;
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
		|| !FiniteVector(TorqueNewtonMetre) || (bHasPoint && !FiniteVector(PointMetre)))
	{
		OutError = TEXT("interaction ticket, identity, target, or wrench is invalid");
		return false;
	}
	if (!FMath::IsFinite(DurationSeconds) || DurationSeconds < 0.0
		|| DurationSeconds > MaxDurationSeconds)
	{
		OutError = TEXT("interaction duration_s must be finite and in [0, 30] seconds");
		return false;
	}
	return true;
}

FString FURLabInteractionWrenchRequest::IdentityKey() const
{
	return InteractionId.IsEmpty() ? TEXT("t:") + Ticket
		: TEXT("i:") + GrantToken + TEXT(":") + InteractionId;
}

struct FURLabInteractionInbox::FState
{
	explicit FState(TSharedRef<FURLabRenderOwnerClock, ESPMode::ThreadSafe> InClock)
		: Clock(MoveTemp(InClock))
	{
	}

	using FSinkPtr = TSharedPtr<IURLabInteractionCompletionSink, ESPMode::ThreadSafe>;

	struct FWaiter
	{
		FString Ticket;
		FSinkPtr Sink;
	};
	struct FRecord
	{
		FURLabInteractionWrenchRequest Request;
		TArray<FWaiter> Waiters;
		FURLabInteractionResult Result;
		bool bTerminal = false;
	};
	struct FGrantRecord
	{
		FURLabInteractionGrant Grant;
		FURLabInteractionControlIdentity Control;
	};
	struct FHold
	{
		uint64 HoldId = 0;
		FURLabInteractionWrenchRequest Request;
		uint64 ApplicationId = 0;
		double EndTime = 0.0;
	};

	mutable FCriticalSection Mutex;
	TSharedRef<FURLabRenderOwnerClock, ESPMode::ThreadSafe> Clock;
	TMap<FString, FGrantRecord> Grants;
	TSharedPtr<IURLabInteractionOwnerHandler, ESPMode::ThreadSafe> Handler;
	TArray<FString> Pending;
	TMap<FString, FRecord> Records;
	TArray<FString> TerminalOrder;
	TMap<uint64, FString> Applications;
	TArray<FHold> Holds;
	uint64 NextApplicationId = 1;
	uint64 NextHoldId = 1;

	/** Rejection for a request under the current grant table, or None when current. */
	EURLabInteractionTerminal CheckLocked(const FURLabInteractionWrenchRequest& Request,
		const FURLabInteractionOwnerContext* Current, double Now, FString& OutError) const
	{
		const FGrantRecord* GrantRecord = Grants.Find(Request.GrantToken);
		if (!GrantRecord || !Handler || Request.ViewerBootId != GrantRecord->Grant.ViewerBootId)
		{
			OutError = TEXT("interaction grant has been revoked");
			return EURLabInteractionTerminal::RejectedStale;
		}
		if (!FMath::IsFinite(Now) || Now >= GrantRecord->Grant.ExpiresAtSeconds)
		{
			OutError = TEXT("interaction grant has expired");
			return EURLabInteractionTerminal::RejectedExpired;
		}
		if (!(Request.Source == GrantRecord->Grant.Source)
			|| Request.ClockEpoch != GrantRecord->Grant.ClockEpoch
			|| (Current && (!(Current->Source == GrantRecord->Grant.Source)
				|| Current->ClockEpoch != GrantRecord->Grant.ClockEpoch)))
		{
			OutError = TEXT("interaction source or clock epoch is stale");
			return EURLabInteractionTerminal::RejectedSource;
		}
		if (Current && !(Current->Control == GrantRecord->Control))
		{
			OutError = TEXT("interaction control is stale");
			return EURLabInteractionTerminal::RejectedStale;
		}
		OutError.Reset();
		return EURLabInteractionTerminal::None;
	}

	void PruneExpiredGrantsLocked(double Now)
	{
		for (auto It = Grants.CreateIterator(); It; ++It)
		{
			if (!FMath::IsFinite(Now) || Now >= It.Value().Grant.ExpiresAtSeconds)
			{
				const FString Token = It.Key();
				Holds.RemoveAll([&](const FHold& Hold) { return Hold.Request.GrantToken == Token; });
				It.RemoveCurrent();
			}
		}
	}

	/** Marks a record terminal and returns its waiters; evicts the oldest terminals. */
	TArray<FWaiter> FinishLocked(const FString& Key, const FURLabInteractionResult& Result)
	{
		TArray<FWaiter> Waiters;
		FRecord* Record = Records.Find(Key);
		if (!Record || Record->bTerminal) return Waiters;
		Record->Result = Result;
		Record->bTerminal = true;
		Waiters = MoveTemp(Record->Waiters);
		Record->Waiters.Reset();
		TerminalOrder.Add(Key);
		while (TerminalOrder.Num() > MaxRetainedTickets)
		{
			Records.Remove(TerminalOrder[0]);
			TerminalOrder.RemoveAt(0, 1, EAllowShrinking::No);
		}
		return Waiters;
	}

	void DropTerminalLocked()
	{
		for (const FString& Key : TerminalOrder) Records.Remove(Key);
		TerminalOrder.Reset();
	}

	static void Notify(const TArray<FWaiter>& Waiters, const FURLabInteractionResult& Result)
	{
		for (const FWaiter& Waiter : Waiters)
		{
			if (Waiter.Sink) Waiter.Sink->CompleteInteraction(Waiter.Ticket, Result);
		}
	}
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
	State->PruneExpiredGrantsLocked(Now);
	if (State->Grants.Num() >= MaxGrants)
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
	State->Holds.Reset();
	State->DropTerminalLocked();
}

bool FURLabInteractionInbox::RevokeGrant(const FString& ExactToken)
{
	FScopeLock Lock(&State->Mutex);
	if (ExactToken.IsEmpty() || State->Grants.Remove(ExactToken) != 1) return false;
	State->Holds.RemoveAll([&](const FState::FHold& Hold)
	{
		return Hold.Request.GrantToken == ExactToken;
	});
	return true;
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
	const FString Key = Request.IdentityKey();
	if (FState::FRecord* Existing = State->Records.Find(Key))
	{
		if (!SameRequest(Existing->Request, Request))
		{
			Result.Admission = EURLabInteractionAdmission::Conflict;
			Result.Error = TEXT("interaction identity was reused with different content");
		}
		else if (Existing->bTerminal)
		{
			Result.Admission = EURLabInteractionAdmission::ExistingTerminal;
			Result.ExistingResult = Existing->Result;
		}
		else
		{
			Result.Admission = EURLabInteractionAdmission::ExistingPending;
			const bool bKnown = Existing->Waiters.ContainsByPredicate(
				[&](const FState::FWaiter& Waiter) { return Waiter.Ticket == Request.Ticket; });
			if (!bKnown) Existing->Waiters.Add({Request.Ticket, MoveTemp(CompletionSink)});
		}
		return Result;
	}
	const double Now = State->Clock->NowSeconds();
	const EURLabInteractionTerminal Rejection = State->CheckLocked(Request, nullptr, Now,
		Result.Error);
	if (Rejection != EURLabInteractionTerminal::None)
	{
		Result.Admission = Rejection == EURLabInteractionTerminal::RejectedExpired
			? EURLabInteractionAdmission::ExpiredGrant
			: Rejection == EURLabInteractionTerminal::RejectedSource
			? EURLabInteractionAdmission::StaleSource : EURLabInteractionAdmission::StaleGrant;
		return Result;
	}
	if (State->Pending.Num() >= MaxPending)
	{
		Result.Admission = EURLabInteractionAdmission::Capacity;
		Result.Error = TEXT("interaction inbox capacity is exhausted");
		return Result;
	}
	if (Request.DurationSeconds > 0.0 && Request.Channel.IsEmpty())
	{
		int32 Holding = State->Holds.Num();
		for (const FString& PendingKey : State->Pending)
		{
			const FState::FRecord* Queued = State->Records.Find(PendingKey);
			if (Queued && Queued->Request.DurationSeconds > 0.0) ++Holding;
		}
		if (Holding >= MaxHolds)
		{
			Result.Admission = EURLabInteractionAdmission::Capacity;
			Result.Error = TEXT("interaction hold capacity is exhausted");
			return Result;
		}
	}
	FState::FRecord Record;
	Record.Request = Request;
	Record.Waiters.Add({Request.Ticket, MoveTemp(CompletionSink)});
	State->Records.Add(Key, MoveTemp(Record));
	State->Pending.Add(Key);
	Result.Admission = EURLabInteractionAdmission::Accepted;
	return Result;
}

int32 FURLabInteractionInbox::Drain(const FURLabInteractionOwnerContext& CurrentContext,
	int32 MaxToDrain)
{
	struct FWork
	{
		FString Key;
		FURLabInteractionWrenchRequest Request;
		uint64 ApplicationId = 0;
		FURLabInteractionResult Terminal;
	};
	TArray<FWork> Work;
	TArray<FState::FHold> ActiveHolds;
	TSharedPtr<IURLabInteractionOwnerHandler, ESPMode::ThreadSafe> Handler;
	{
		FScopeLock Lock(&State->Mutex);
		Handler = State->Handler;
		const double Now = State->Clock->NowSeconds();
		const int32 Limit = FMath::Clamp(MaxToDrain, 0, MaxPending);
		while (Work.Num() < Limit && !State->Pending.IsEmpty())
		{
			const FString Key = State->Pending[0];
			State->Pending.RemoveAt(0, 1, EAllowShrinking::No);
			const FState::FRecord* Record = State->Records.Find(Key);
			if (!Record || Record->bTerminal) continue;
			FWork& Item = Work.AddDefaulted_GetRef();
			Item.Key = Key;
			Item.Request = Record->Request;
			Item.Terminal.Terminal = State->CheckLocked(Item.Request, &CurrentContext, Now,
				Item.Terminal.Error);
			if (Item.Terminal.Terminal == EURLabInteractionTerminal::None)
			{
				if (State->NextApplicationId == 0)
				{
					Item.Terminal.Terminal = EURLabInteractionTerminal::ApplyFailed;
					Item.Terminal.Error = TEXT("interaction application ids are exhausted");
				}
				else Item.ApplicationId = State->NextApplicationId++;
			}
		}
		FString Ignored;
		State->Holds.RemoveAll([&](const FState::FHold& Hold)
		{
			return CurrentContext.SourceTime + HoldTimeTolerance >= Hold.EndTime
				|| State->CheckLocked(Hold.Request, &CurrentContext, Now, Ignored)
					!= EURLabInteractionTerminal::None;
		});
		ActiveHolds = State->Holds;
	}

	for (FWork& Item : Work)
	{
		FString Error;
		if (Item.Terminal.Terminal == EURLabInteractionTerminal::None
			&& (!Handler || !Handler->ValidateTarget(Item.Request.BodyIndex, Error)))
		{
			Item.Terminal.Terminal = EURLabInteractionTerminal::RejectedTarget;
			Item.Terminal.Error = Error.IsEmpty()
				? FString(TEXT("interaction target is invalid")) : MoveTemp(Error);
		}
	}
	TSet<uint64> RemovedHolds;
	for (const FWork& Item : Work)
	{
		if (Item.Terminal.Terminal != EURLabInteractionTerminal::None
			|| Item.Request.Channel.IsEmpty())
		{
			continue;
		}
		for (const FState::FHold& Hold : ActiveHolds)
		{
			if (Hold.Request.GrantToken == Item.Request.GrantToken
				&& Hold.Request.Channel == Item.Request.Channel)
			{
				RemovedHolds.Add(Hold.HoldId);
			}
		}
	}
	for (const FState::FHold& Hold : ActiveHolds)
	{
		if (RemovedHolds.Contains(Hold.HoldId)) continue;
		FString Error;
		if (!Handler || !Handler->ApplyNextStepContribution(Hold.Request, Hold.ApplicationId, Error))
			RemovedHolds.Add(Hold.HoldId);
	}
	TArray<FState::FHold> NewHolds;
	int32 Drained = 0;
	for (FWork& Item : Work)
	{
		++Drained;
		if (Item.Terminal.Terminal != EURLabInteractionTerminal::None) continue;
		FString Error;
		if (!Handler->ApplyNextStepContribution(Item.Request, Item.ApplicationId, Error))
		{
			Item.Terminal.Terminal = EURLabInteractionTerminal::ApplyFailed;
			Item.Terminal.Error = Error.IsEmpty()
				? FString(TEXT("interaction contribution could not be applied")) : MoveTemp(Error);
			continue;
		}
		if (Item.Request.DurationSeconds > 0.0)
		{
			FState::FHold& Hold = NewHolds.AddDefaulted_GetRef();
			Hold.Request = Item.Request;
			Hold.ApplicationId = Item.ApplicationId;
			Hold.EndTime = CurrentContext.SourceTime + Item.Request.DurationSeconds;
		}
	}

	TArray<TPair<TArray<FState::FWaiter>, FURLabInteractionResult>> Notifications;
	{
		FScopeLock Lock(&State->Mutex);
		if (!RemovedHolds.IsEmpty())
		{
			State->Holds.RemoveAll([&](const FState::FHold& Hold)
			{
				return RemovedHolds.Contains(Hold.HoldId);
			});
		}
		for (FState::FHold& Hold : NewHolds)
		{
			if (!State->Grants.Contains(Hold.Request.GrantToken)
				|| State->Holds.Num() >= MaxHolds)
			{
				continue;
			}
			Hold.HoldId = State->NextHoldId++;
			State->Holds.Add(MoveTemp(Hold));
		}
		for (const FWork& Item : Work)
		{
			if (Item.Terminal.Terminal == EURLabInteractionTerminal::None)
				State->Applications.Add(Item.ApplicationId, Item.Key);
			else
				Notifications.Emplace(State->FinishLocked(Item.Key, Item.Terminal), Item.Terminal);
		}
	}
	for (const auto& Notification : Notifications)
		FState::Notify(Notification.Key, Notification.Value);
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
	TArray<FState::FWaiter> Waiters;
	FURLabInteractionResult Result;
	{
		FScopeLock Lock(&State->Mutex);
		FString Key;
		if (!State->Applications.RemoveAndCopyValue(ApplicationId, Key))
		{
			OutError = TEXT("interaction application is unknown or already complete");
			return false;
		}
		// The contribution was integrated by the step that just ran, so the
		// outcome is Applied even if authority changed during that step.
		Result.Terminal = EURLabInteractionTerminal::Applied;
		Result.AppliedFrame = AppliedFrame;
		Waiters = State->FinishLocked(Key, Result);
	}
	FState::Notify(Waiters, Result);
	return true;
}

bool FURLabInteractionInbox::AbortApplied(uint64 ApplicationId, const FString& Reason)
{
	TArray<FState::FWaiter> Waiters;
	FURLabInteractionResult Result;
	Result.Terminal = EURLabInteractionTerminal::ApplyFailed;
	Result.Error = Reason;
	{
		FScopeLock Lock(&State->Mutex);
		FString Key;
		if (!State->Applications.RemoveAndCopyValue(ApplicationId, Key)) return false;
		Waiters = State->FinishLocked(Key, Result);
	}
	FState::Notify(Waiters, Result);
	return true;
}

int32 FURLabInteractionInbox::NumPending() const
{
	FScopeLock Lock(&State->Mutex);
	return State->Pending.Num();
}

int32 FURLabInteractionInbox::NumGrants() const
{
	FScopeLock Lock(&State->Mutex);
	return State->Grants.Num();
}

int32 FURLabInteractionInbox::NumHolds() const
{
	FScopeLock Lock(&State->Mutex);
	return State->Holds.Num();
}

int32 FURLabInteractionInbox::NumRetained() const
{
	FScopeLock Lock(&State->Mutex);
	return State->Records.Num();
}
