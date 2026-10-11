#pragma once

#include "CoreMinimal.h"
#include "Workflows/RenderOwnerAuthority.h"

/** Exact producer/source identity shared by grants, requests, and apply context. */
struct URLAB_API FURLabInteractionSourceIdentity
{
	FString ProducerBootId;
	FString StreamId;
	uint64 StreamGeneration = 0;
	FString SourceRevision;

	bool IsValid() const;
	bool operator==(const FURLabInteractionSourceIdentity& Other) const;
};

/** Opaque control identity supplied by the producer's authority implementation. */
struct URLAB_API FURLabInteractionControlIdentity
{
	FString AuthorityId;
	uint64 Generation = 0;
	FString OpaqueProof;

	bool IsValid() const;
	bool operator==(const FURLabInteractionControlIdentity& Other) const;
};

/** Owner-thread snapshot used both when issuing a grant and when applying work. */
struct URLAB_API FURLabInteractionOwnerContext
{
	FURLabInteractionSourceIdentity Source;
	FURLabInteractionControlIdentity Control;
	uint64 ClockEpoch = 0;
};

struct URLAB_API FURLabInteractionGrant
{
	FString Token;
	FURLabInteractionSourceIdentity Source;
	FString ViewerBootId;
	uint64 ClockEpoch = 0;
	double ExpiresAtSeconds = 0.0;

	bool IsValid() const;
};

/** One world-frame right-handed SI wrench for the next authoritative step. */
struct URLAB_API FURLabInteractionWrenchRequest
{
	FString Ticket;
	FString GrantToken;
	FURLabInteractionSourceIdentity Source;
	FString ViewerBootId;
	uint64 ClockEpoch = 0;
	int32 BodyIndex = INDEX_NONE;
	FVector3d ForceNewton = FVector3d::ZeroVector;
	FVector3d TorqueNewtonMetre = FVector3d::ZeroVector;

	bool IsValid(FString& OutError) const;
};

struct URLAB_API FURLabInteractionAppliedFrame
{
	uint64 FrameId = 0;
	uint64 ClockEpoch = 0;
	double SourceTime = 0.0;
	bool bHasSourceTime = false;
};

enum class EURLabInteractionAdmission : uint8
{
	Accepted,
	Unsupported,
	Invalid,
	StaleGrant,
	Conflict,
	ExistingPending,
	ExistingTerminal,
	Capacity,
};

enum class EURLabInteractionTerminal : uint8
{
	None,
	Applied,
	RejectedStale,
	RejectedTarget,
	ApplyFailed,
};

struct URLAB_API FURLabInteractionResult
{
	EURLabInteractionTerminal Terminal = EURLabInteractionTerminal::None;
	FURLabInteractionAppliedFrame AppliedFrame;
	FString Error;
};

struct URLAB_API FURLabInteractionAdmissionResult
{
	EURLabInteractionAdmission Admission = EURLabInteractionAdmission::Invalid;
	FURLabInteractionResult ExistingResult;
	FString Error;
};

/** Optional producer implementation. Every method is called only by its owner thread. */
class URLAB_API IURLabInteractionOwnerHandler
{
public:
	virtual ~IURLabInteractionOwnerHandler() = default;
	virtual bool ValidateTarget(int32 BodyIndex, FString& OutError) = 0;
	virtual bool ApplyNextStepContribution(const FURLabInteractionWrenchRequest& Request,
		uint64 ApplicationId, FString& OutError) = 0;
};

/** Completion seam for the correlated request ledger. Called on the owner thread. */
class URLAB_API IURLabInteractionCompletionSink
{
public:
	virtual ~IURLabInteractionCompletionSink() = default;
	virtual void CompleteInteraction(const FString& Ticket,
		const FURLabInteractionResult& Result) = 0;
};

/**
 * Bounded transport-independent interaction inbox. Network threads only enqueue;
 * the authority owner drains, applies, and completes work on its own thread.
 */
class URLAB_API FURLabInteractionInbox
{
public:
	static constexpr int32 MaxPending = 32;
	static constexpr int32 MaxRetainedTickets = 256;

	explicit FURLabInteractionInbox(
		TSharedRef<FURLabRenderOwnerClock, ESPMode::ThreadSafe> InClock =
			FURLabRenderOwnerClock::System());
	~FURLabInteractionInbox();

	/** Owner-thread only. Adds one independently revocable viewer grant. */
	bool IssueGrant(const FURLabInteractionOwnerContext& Context,
		const FString& ViewerBootId, double TtlSeconds,
		TSharedPtr<IURLabInteractionOwnerHandler, ESPMode::ThreadSafe> Handler,
		FURLabInteractionGrant& OutGrant, FString& OutError);
	/** Owner-thread only. Queued work remains available to drain as stale. */
	void RevokeGrant();
	bool RevokeGrant(const FString& ExactToken);
	/** Thread-safe admission; performs no handler or physics work. */
	FURLabInteractionAdmissionResult Enqueue(const FURLabInteractionWrenchRequest& Request,
		TSharedPtr<IURLabInteractionCompletionSink, ESPMode::ThreadSafe> CompletionSink);
	/** Owner-thread only. */
	int32 Drain(const FURLabInteractionOwnerContext& CurrentContext, int32 MaxToDrain = MaxPending);
	/** Owner-thread only, after the authoritative step has produced its identity. */
	bool CompleteApplied(uint64 ApplicationId,
		const FURLabInteractionOwnerContext& CurrentContext,
		const FURLabInteractionAppliedFrame& AppliedFrame, FString& OutError);
	int32 NumPending() const;

private:
	struct FState;
	TUniquePtr<FState> State;
};
