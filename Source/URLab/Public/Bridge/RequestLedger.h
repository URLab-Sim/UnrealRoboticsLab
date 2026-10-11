// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#pragma once

#include "CoreMinimal.h"
#include "Wire/RequestDigest.h"

/** Bounded, instance-owned logical request ledger. No handler executes here.
 *  All session and aggregate state uses one shared, short-held instance mutex;
 *  there is no nested lock order and returned reply storage is const. */
class URLAB_API FURLabRequestLedger
{
public:
	static constexpr int32 MaxEntriesPerSession = 256;
	static constexpr uint64 WindowWidth = 1024;
	static constexpr int32 MaxPendingPerSession = 32;
	static constexpr int32 MaxPendingGlobal = 256;
	static constexpr int64 MaxInlineReplyBytes = 256 * 1024;
	static constexpr int64 MaxRetainedBytesPerSession = 8 * 1024 * 1024;
	static constexpr int64 MaxRetainedBytesGlobal = 64 * 1024 * 1024;

	enum EAdmission : uint8
	{
		Accepted,
		ExistingPending,
		ExistingTerminal,
		InvalidId,
		Closed,
		Retired,
		Conflict,
		PendingBlocksRetirement,
		SessionCapacity,
		GlobalCapacity
	};

	enum ETerminal : uint8
	{
		None,
		Completed,
		ResultUnavailable
	};

	struct FAdmissionResult
	{
		EAdmission Status = EAdmission::InvalidId;
		ETerminal Terminal = ETerminal::None;
		TSharedPtr<const TArray<uint8>, ESPMode::ThreadSafe> ReplyBytes;
	};

	struct FStats
	{
		int32 Pending = 0;
		int64 RetainedBytes = 0;
	};

	class FSession;
	using FSessionRef = TSharedRef<FSession, ESPMode::ThreadSafe>;

	FURLabRequestLedger();
	~FURLabRequestLedger();

	/** Creates a core accounting handle. The dispatcher owns session-ID
	 *  uniqueness/liveness; this handle cannot reopen or grant authority. */
	FSessionRef OpenSession(const FString& SessionId);
	void CloseSession(const FSessionRef& Session);

	FAdmissionResult Admit(const FSessionRef& Session, int64 RequestId,
		const FURLabRequestDigest& Digest);

	/** Completes an accepted Pending record. Returns false for a missing,
	 *  non-pending, or digest-conflicting record. Oversized bytes are discarded
	 *  and retained as ResultUnavailable. */
	bool Complete(const FSessionRef& Session, int64 RequestId,
		const FURLabRequestDigest& Digest, TArray<uint8>&& ReplyBytes);

	FStats GetStats() const;
	FStats GetSessionStats(const FSessionRef& Session) const;
	uint64 GetRetiredWatermark(const FSessionRef& Session) const;
	uint64 GetHighWatermark(const FSessionRef& Session) const;

private:
	struct FState;
	TSharedRef<FState, ESPMode::ThreadSafe> State;
};
