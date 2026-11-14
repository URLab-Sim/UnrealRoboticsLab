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

/**
 * Owner-thread snapshot used both when issuing a grant and when applying work.
 * SourceTime is the producer's simulation time at the start of the step being
 * drained; held wrenches measure their duration against it.
 */
struct URLAB_API FURLabInteractionOwnerContext
{
	FURLabInteractionSourceIdentity Source;
	FURLabInteractionControlIdentity Control;
	uint64 ClockEpoch = 0;
	double SourceTime = 0.0;
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

/**
 * One SI wrench in MuJoCo coordinates (right-handed, Z up, metres, newtons).
 *
 * With bBodyFrame unset, force, torque and the optional point are world-frame.
 * With bBodyFrame set they are expressed in the body frame (xpos/xmat) and are
 * re-rotated on every step the wrench is held. Without a point the force acts
 * at the body centre of mass; with a point the owner adds (point - com) x force.
 *
 * DurationSeconds == 0 applies the wrench to exactly one authoritative step.
 * A positive duration holds it on every step whose start time is before
 * first-step time + duration, so the integrated impulse is force * duration
 * rounded up to whole steps. A submission on a non-empty Channel replaces the
 * hold previously submitted on the same grant and channel; a zero-duration
 * submission on a channel therefore releases it. Holds end early when the
 * grant is revoked or expires, or the source, control or clock epoch changes.
 *
 * InteractionId, when set, is the idempotency key: a resubmission with the same
 * grant and InteractionId never applies twice, whatever its transport ticket.
 */
struct URLAB_API FURLabInteractionWrenchRequest
{
	static constexpr double MaxDurationSeconds = 30.0;

	FString Ticket;
	FString GrantToken;
	FURLabInteractionSourceIdentity Source;
	FString ViewerBootId;
	uint64 ClockEpoch = 0;
	int32 BodyIndex = INDEX_NONE;
	FVector3d ForceNewton = FVector3d::ZeroVector;
	FVector3d TorqueNewtonMetre = FVector3d::ZeroVector;
	bool bBodyFrame = false;
	bool bHasPoint = false;
	FVector3d PointMetre = FVector3d::ZeroVector;
	double DurationSeconds = 0.0;
	FString Channel;
	FString InteractionId;

	bool IsValid(FString& OutError) const;
	/** Inbox identity: grant-scoped InteractionId when present, else the ticket. */
	FString IdentityKey() const;
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
	ExpiredGrant,
	StaleSource,
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
	RejectedExpired,
	RejectedSource,
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
	/** Adds the wrench to the next step only; called again for every held step. */
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
 * Terminal records are retained for idempotent replay up to MaxRetainedTickets
 * and then evicted oldest first; they never consume admission capacity.
 */
class URLAB_API FURLabInteractionInbox
{
public:
	static constexpr int32 MaxPending = 32;
	static constexpr int32 MaxRetainedTickets = 256;
	static constexpr int32 MaxGrants = 32;
	static constexpr int32 MaxHolds = 32;

	explicit FURLabInteractionInbox(
		TSharedRef<FURLabRenderOwnerClock, ESPMode::ThreadSafe> InClock =
			FURLabRenderOwnerClock::System());
	~FURLabInteractionInbox();

	/** Owner-thread only. Adds one independently revocable viewer grant. */
	bool IssueGrant(const FURLabInteractionOwnerContext& Context,
		const FString& ViewerBootId, double TtlSeconds,
		TSharedPtr<IURLabInteractionOwnerHandler, ESPMode::ThreadSafe> Handler,
		FURLabInteractionGrant& OutGrant, FString& OutError);
	/** Owner-thread only. Ends holds immediately; queued work drains as stale. */
	void RevokeGrant();
	bool RevokeGrant(const FString& ExactToken);
	/** Thread-safe admission; performs no handler or physics work. */
	FURLabInteractionAdmissionResult Enqueue(const FURLabInteractionWrenchRequest& Request,
		TSharedPtr<IURLabInteractionCompletionSink, ESPMode::ThreadSafe> CompletionSink);
	/** Owner-thread only. Applies held wrenches, then up to MaxToDrain new ones. */
	int32 Drain(const FURLabInteractionOwnerContext& CurrentContext, int32 MaxToDrain = MaxPending);
	/** Owner-thread only, after the authoritative step has produced its identity. */
	bool CompleteApplied(uint64 ApplicationId,
		const FURLabInteractionOwnerContext& CurrentContext,
		const FURLabInteractionAppliedFrame& AppliedFrame, FString& OutError);
	/** Owner-thread only. Fails a drained application whose step did not run. */
	bool AbortApplied(uint64 ApplicationId, const FString& Reason);
	int32 NumPending() const;
	int32 NumGrants() const;
	int32 NumHolds() const;
	int32 NumRetained() const;

private:
	struct FState;
	TUniquePtr<FState> State;
};
