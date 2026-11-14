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
	static constexpr int64 MaxInlineReplyBytes = 64 * 1024 * 1024;
	static constexpr int64 MaxRetainedBytesPerSession = 512ll * 1024 * 1024;
	static constexpr int64 MaxRetainedBytesGlobal = 4096ll * 1024 * 1024;

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

	/** Blocks until an admitted record is terminal or the timeout elapses and
	 *  returns the admission view of the record at that point. */
	FAdmissionResult AwaitTerminal(const FSessionRef& Session, int64 RequestId,
		const FURLabRequestDigest& Digest, double TimeoutSeconds);

	FStats GetStats() const;
	FStats GetSessionStats(const FSessionRef& Session) const;
	uint64 GetRetiredWatermark(const FSessionRef& Session) const;
	uint64 GetHighWatermark(const FSessionRef& Session) const;

private:
	struct FState;
	TSharedRef<FState, ESPMode::ThreadSafe> State;
};

/**
 * Per-thread handoff that lets a transport ship a large retained reply without
 * copying it out of the ledger.
 *
 * A transport opens an FScope around one ProcessRequestBytes call. When the
 * settled reply is at least MinBytes, the dispatcher offers the ledger's shared,
 * immutable reply storage here and leaves the TArray out-parameter empty; the
 * transport then sends from the shared storage (ZMQ zero-copy, gRPC one copy).
 * Without an open scope nothing changes: the reply is copied as before.
 */
struct URLAB_API FURLabSharedReplySink
{
	using FBytesPtr = TSharedPtr<const TArray<uint8>, ESPMode::ThreadSafe>;
	static constexpr int32 MinBytes = 1024 * 1024;

	struct URLAB_API FScope
	{
		FScope();
		~FScope();
		FScope(const FScope&) = delete;
		FScope& operator=(const FScope&) = delete;
	};

	/** True when the current thread has an open scope that has not been filled. */
	static bool CanAccept(int64 NumBytes);
	static void Offer(FBytesPtr Bytes);
	/** Removes the offered reply, if any, from the current thread's scope. */
	static FBytesPtr Take();
};

