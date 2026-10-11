// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#include "Bridge/RequestLedger.h"

namespace
{
using ETerminal = FURLabRequestLedger::ETerminal;

struct FRecord
{
	FURLabRequestDigest Digest;
	bool bPending = true;
	ETerminal Terminal = ETerminal::None;
	TSharedPtr<const TArray<uint8>, ESPMode::ThreadSafe> ReplyBytes;
	int64 AccountedBytes = FURLabRequestLedger::MaxInlineReplyBytes;
};

TSharedPtr<const TArray<uint8>, ESPMode::ThreadSafe> OwnedBytes(TArray<uint8>&& Bytes)
{
	TSharedPtr<TArray<uint8>, ESPMode::ThreadSafe> Owned =
		MakeShared<TArray<uint8>, ESPMode::ThreadSafe>(MoveTemp(Bytes));
	return Owned;
}

} // namespace

struct FURLabRequestLedger::FState
{
	mutable FCriticalSection Mutex;
	int32 Pending = 0;
	int64 RetainedBytes = 0;
};

class FURLabRequestLedger::FSession
{
public:
	explicit FSession(FString InId, const TSharedRef<FState, ESPMode::ThreadSafe>& InState)
		: Id(MoveTemp(InId)), SharedState(InState)
	{
	}
	~FSession()
	{
		FScopeLock Lock(&SharedState->Mutex);
		SharedState->Pending -= Pending;
		SharedState->RetainedBytes -= RetainedBytes;
	}

private:
	friend class FURLabRequestLedger;
	FString Id;
	TSharedRef<FState, ESPMode::ThreadSafe> SharedState;
	bool bOpen = true;
	uint64 RetiredWatermark = 0;
	uint64 HighWatermark = 0;
	int32 Pending = 0;
	int64 RetainedBytes = 0;
	TMap<uint64, FRecord> Records;
};

FURLabRequestLedger::FURLabRequestLedger()
	: State(MakeShared<FState, ESPMode::ThreadSafe>())
{
}

FURLabRequestLedger::~FURLabRequestLedger() = default;

FURLabRequestLedger::FSessionRef FURLabRequestLedger::OpenSession(const FString& SessionId)
{
	return MakeShared<FSession, ESPMode::ThreadSafe>(SessionId, State);
}

void FURLabRequestLedger::CloseSession(const FSessionRef& Session)
{
	if (&Session->SharedState.Get() != &State.Get())
		return;
	FState& Shared = *Session->SharedState;
	FScopeLock Lock(&Shared.Mutex);
	if (!Session->bOpen)
		return;
	Session->bOpen = false;
	for (auto It = Session->Records.CreateIterator(); It; ++It)
	{
		if (!It.Value().bPending)
		{
			Session->RetainedBytes -= It.Value().AccountedBytes;
			Shared.RetainedBytes -= It.Value().AccountedBytes;
			It.RemoveCurrent();
		}
	}
}

FURLabRequestLedger::FAdmissionResult FURLabRequestLedger::Admit(
	const FSessionRef& Session, int64 RequestId, const FURLabRequestDigest& Digest)
{
	FAdmissionResult Result;
	if (RequestId <= 0)
	{
		Result.Status = EAdmission::InvalidId;
		return Result;
	}
	if (&Session->SharedState.Get() != &State.Get())
	{
		Result.Status = EAdmission::Closed;
		return Result;
	}
	const uint64 Id = static_cast<uint64>(RequestId);
	FState& Shared = *Session->SharedState;
	FScopeLock Lock(&Shared.Mutex);
	if (!Session->bOpen)
	{
		Result.Status = EAdmission::Closed;
		return Result;
	}
	if (Id <= Session->RetiredWatermark)
	{
		Result.Status = EAdmission::Retired;
		return Result;
	}
	if (const FRecord* Existing = Session->Records.Find(Id))
	{
		if (!(Existing->Digest == Digest))
		{
			Result.Status = EAdmission::Conflict;
			return Result;
		}
		Result.Status = Existing->bPending
			? EAdmission::ExistingPending : EAdmission::ExistingTerminal;
		Result.Terminal = Existing->Terminal;
		Result.ReplyBytes = Existing->ReplyBytes;
		return Result;
	}
	// Count exhaustion has deterministic precedence over retirement pressure.
	// Duplicates and permanently retired IDs above still report their identity
	// result without consuming another pending slot.
	if (Session->Pending >= MaxPendingPerSession)
	{
		Result.Status = EAdmission::SessionCapacity;
		return Result;
	}
	if (Shared.Pending >= MaxPendingGlobal)
	{
		Result.Status = EAdmission::GlobalCapacity;
		return Result;
	}

	const uint64 ProposedHigh = FMath::Max(Session->HighWatermark, Id);
	const uint64 WindowFloor = ProposedHigh > WindowWidth ? ProposedHigh - WindowWidth : 0;
	uint64 ProposedRetired = FMath::Max(Session->RetiredWatermark, WindowFloor);
	TSet<uint64> Retire;
	int64 FreedBytes = 0;
	for (const TPair<uint64, FRecord>& Pair : Session->Records)
	{
		if (Pair.Key <= ProposedRetired)
		{
			if (Pair.Value.bPending)
			{
				Result.Status = EAdmission::PendingBlocksRetirement;
				return Result;
			}
			Retire.Add(Pair.Key);
			FreedBytes += Pair.Value.AccountedBytes;
		}
	}

	auto RemainingEntries = [&]() { return Session->Records.Num() - Retire.Num(); };
	auto SessionBytesAfter = [&]() {
		return Session->RetainedBytes - FreedBytes + MaxInlineReplyBytes;
	};
	auto GlobalBytesAfter = [&]() {
		return Shared.RetainedBytes - FreedBytes + MaxInlineReplyBytes;
	};
	while (RemainingEntries() >= MaxEntriesPerSession ||
		SessionBytesAfter() > MaxRetainedBytesPerSession ||
		GlobalBytesAfter() > MaxRetainedBytesGlobal)
	{
		uint64 Lowest = MAX_uint64;
		const FRecord* LowestRecord = nullptr;
		for (const TPair<uint64, FRecord>& Pair : Session->Records)
		{
			if (!Retire.Contains(Pair.Key) && Pair.Key < Lowest)
			{
				Lowest = Pair.Key;
				LowestRecord = &Pair.Value;
			}
		}
		if (LowestRecord == nullptr)
		{
			Result.Status = GlobalBytesAfter() > MaxRetainedBytesGlobal
				? EAdmission::GlobalCapacity : EAdmission::SessionCapacity;
			return Result;
		}
		if (LowestRecord->bPending)
		{
			Result.Status = EAdmission::PendingBlocksRetirement;
			return Result;
		}
		if (Lowest >= Id)
		{
			// Retiring this entry would also retire the incoming skipped ID.
			Result.Status = EAdmission::SessionCapacity;
			return Result;
		}
		ProposedRetired = FMath::Max(ProposedRetired, Lowest);
		for (const TPair<uint64, FRecord>& Pair : Session->Records)
		{
			if (Pair.Key <= ProposedRetired && !Retire.Contains(Pair.Key))
			{
				if (Pair.Value.bPending)
				{
					Result.Status = EAdmission::PendingBlocksRetirement;
					return Result;
				}
				Retire.Add(Pair.Key);
				FreedBytes += Pair.Value.AccountedBytes;
			}
		}
	}

	// Commit the complete retirement/reservation plan only after every check passed.
	for (uint64 RetiredId : Retire)
		Session->Records.Remove(RetiredId);
	Session->RetiredWatermark = ProposedRetired;
	Session->HighWatermark = ProposedHigh;
	Session->RetainedBytes = SessionBytesAfter();
	Shared.RetainedBytes = GlobalBytesAfter();
	++Session->Pending;
	++Shared.Pending;
	FRecord Record;
	Record.Digest = Digest;
	Session->Records.Add(Id, MoveTemp(Record));
	Result.Status = EAdmission::Accepted;
	return Result;
}

bool FURLabRequestLedger::Complete(const FSessionRef& Session, int64 RequestId,
	const FURLabRequestDigest& Digest, TArray<uint8>&& ReplyBytes)
{
	if (RequestId <= 0)
		return false;
	if (&Session->SharedState.Get() != &State.Get())
		return false;
	FState& Shared = *Session->SharedState;
	FScopeLock Lock(&Shared.Mutex);
	FRecord* Record = Session->Records.Find(static_cast<uint64>(RequestId));
	if (!Record || !Record->bPending || !(Record->Digest == Digest))
		return false;

	Record->bPending = false;
	--Session->Pending;
	--Shared.Pending;
	const bool bOversized = ReplyBytes.Num() > MaxInlineReplyBytes;
	Record->Terminal = bOversized ? ETerminal::ResultUnavailable : ETerminal::Completed;
	if (bOversized)
		Record->ReplyBytes.Reset();
	else
		Record->ReplyBytes = OwnedBytes(MoveTemp(ReplyBytes));
	const int64 ActualBytes = Record->ReplyBytes.IsValid() ? Record->ReplyBytes->Num() : 0;
	Session->RetainedBytes += ActualBytes - Record->AccountedBytes;
	Shared.RetainedBytes += ActualBytes - Record->AccountedBytes;
	Record->AccountedBytes = ActualBytes;

	if (!Session->bOpen)
	{
		Session->RetainedBytes -= Record->AccountedBytes;
		Shared.RetainedBytes -= Record->AccountedBytes;
		Session->Records.Remove(static_cast<uint64>(RequestId));
	}
	return true;
}

FURLabRequestLedger::FStats FURLabRequestLedger::GetStats() const
{
	FScopeLock Lock(&State->Mutex);
	return {State->Pending, State->RetainedBytes};
}

FURLabRequestLedger::FStats FURLabRequestLedger::GetSessionStats(const FSessionRef& Session) const
{
	if (&Session->SharedState.Get() != &State.Get())
		return {};
	FState& Shared = *Session->SharedState;
	FScopeLock Lock(&Shared.Mutex);
	return {Session->Pending, Session->RetainedBytes};
}

uint64 FURLabRequestLedger::GetRetiredWatermark(const FSessionRef& Session) const
{
	if (&Session->SharedState.Get() != &State.Get())
		return 0;
	FState& Shared = *Session->SharedState;
	FScopeLock Lock(&Shared.Mutex);
	return Session->RetiredWatermark;
}

uint64 FURLabRequestLedger::GetHighWatermark(const FSessionRef& Session) const
{
	if (&Session->SharedState.Get() != &State.Get())
		return 0;
	FState& Shared = *Session->SharedState;
	FScopeLock Lock(&Shared.Mutex);
	return Session->HighWatermark;
}
