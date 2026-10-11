// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#pragma once

#include "CoreMinimal.h"

#include <atomic>

/** Errors from the transport-independent render-owner authority. */
enum class EURLabRenderOwnerAuthorityError : uint8
{
	None,
	Unavailable,
	InvalidScope,
	InactiveScope,
	InvalidClock,
	ClockMovedBackward,
	InvalidTtl,
	Occupied,
	Retiring,
	ShuttingDown,
	InvalidProof,
	Expired,
	OwnerStillActive,
	CounterExhausted,
	InvalidRetirement,
};

/** Shared monotonic time basis. The production instance uses FPlatformTime. */
class URLAB_API FURLabRenderOwnerClock
{
public:
	static TSharedRef<FURLabRenderOwnerClock, ESPMode::ThreadSafe> System();
	static TSharedRef<FURLabRenderOwnerClock, ESPMode::ThreadSafe> ForTest(
		TFunction<double()> ReadSeconds);

	double NowSeconds() const;

private:
	explicit FURLabRenderOwnerClock(TFunction<double()> InReadSeconds);
	TFunction<double()> ReadSeconds;
};

/**
 * One independently minted render-owner session scope. Its external label is
 * descriptive only: exact authority binds the unforgeable internal identity and
 * this scope's lifetime. Deactivation is one-way and may occur off the game thread.
 */
class URLAB_API FURLabRenderOwnerSessionScope
{
public:
	static TSharedPtr<FURLabRenderOwnerSessionScope, ESPMode::ThreadSafe> Create(
		FString ExternalSessionLabel, FString& OutError);

	const FString& GetExternalSessionLabel() const { return ExternalSessionLabel; }
	bool IsActive() const { return bActive.load(std::memory_order_acquire); }
	void Deactivate() { bActive.store(false, std::memory_order_release); }

private:
	friend class FURLabRenderOwnerAuthority;
	explicit FURLabRenderOwnerSessionScope(FString InExternalSessionLabel,
		FGuid InIdentity);

	const FString ExternalSessionLabel;
	const FGuid Identity;
	std::atomic<bool> bActive{true};
};

using FURLabRenderOwnerSessionScopePtr =
	TSharedPtr<FURLabRenderOwnerSessionScope, ESPMode::ThreadSafe>;

/** Exact owner proof returned only by attach or successful authorization. */
class URLAB_API FURLabRenderOwnerProof
{
public:
	FURLabRenderOwnerProof() = default;
	bool IsValid() const;
	uint64 GetGeneration() const { return AttachmentGeneration; }
	const FString& GetOpaqueToken() const { return OpaqueToken; }

private:
	friend class FURLabRenderOwnerAuthority;
	FGuid AuthorityIdentity;
	FURLabRenderOwnerSessionScopePtr Scope;
	FGuid ScopeIdentity;
	uint64 AttachmentGeneration = 0;
	FString OpaqueToken;
};

/** Exact proof of one in-progress two-phase owner retirement. */
class URLAB_API FURLabRenderOwnerRetirementToken
{
public:
	FURLabRenderOwnerRetirementToken() = default;
	bool IsValid() const;

private:
	friend class FURLabRenderOwnerAuthority;
	FGuid AuthorityIdentity;
	FGuid ScopeIdentity;
	uint64 AttachmentGeneration = 0;
	FString OwnerToken;
	FGuid RetirementIdentity;
};

/** Passive reader status. It never contains the owner token or session identity. */
struct URLAB_API FURLabRenderOwnerStatus
{
	bool bAvailable = true;
	bool bActive = false;
	bool bStale = false;
	bool bRetiring = false;
	uint64 AttachmentGeneration = 0;
	double SecondsRemaining = 0.0;
};

struct URLAB_API FURLabRenderOwnerStatusResult
{
	EURLabRenderOwnerAuthorityError Error = EURLabRenderOwnerAuthorityError::None;
	FURLabRenderOwnerStatus Status;
	FString Message;

	bool IsOk() const { return Error == EURLabRenderOwnerAuthorityError::None; }
};

struct URLAB_API FURLabRenderOwnerAttachResult
{
	EURLabRenderOwnerAuthorityError Error = EURLabRenderOwnerAuthorityError::None;
	FURLabRenderOwnerProof Proof;
	FURLabRenderOwnerStatus Status;
	FString Message;

	bool IsOk() const { return Error == EURLabRenderOwnerAuthorityError::None; }
};

struct URLAB_API FURLabRenderOwnerAuthorizationResult
{
	EURLabRenderOwnerAuthorityError Error = EURLabRenderOwnerAuthorityError::None;
	FURLabRenderOwnerProof Proof;
	FURLabRenderOwnerStatus Status;
	FString Message;

	bool IsOk() const { return Error == EURLabRenderOwnerAuthorityError::None; }
};

struct URLAB_API FURLabRenderOwnerAdmissionResult
{
	EURLabRenderOwnerAuthorityError Error = EURLabRenderOwnerAuthorityError::None;
	bool bAuthorized = false;
	FURLabRenderOwnerStatus Status;
	FString Message;

	bool IsOk() const
	{
		return bAuthorized && Error == EURLabRenderOwnerAuthorityError::None;
	}
};

struct URLAB_API FURLabRenderOwnerLeaseResult
{
	EURLabRenderOwnerAuthorityError Error = EURLabRenderOwnerAuthorityError::None;
	bool bChanged = false;
	FURLabRenderOwnerStatus Status;
	FString Message;

	bool IsOk() const { return Error == EURLabRenderOwnerAuthorityError::None; }
};

struct URLAB_API FURLabRenderOwnerRetirementResult
{
	EURLabRenderOwnerAuthorityError Error = EURLabRenderOwnerAuthorityError::None;
	bool bChanged = false;
	FURLabRenderOwnerRetirementToken Token;
	FURLabRenderOwnerStatus Status;
	FString Message;

	bool IsOk() const { return Error == EURLabRenderOwnerAuthorityError::None; }
};

/**
 * Exclusive render-consumer owner authority. All methods except session-scope
 * deactivation run on the game thread. The caller performs job cancellation and
 * publication revocation between Begin*Retirement and FinishRetirement.
 */
class URLAB_API FURLabRenderOwnerAuthority
{
public:
	static constexpr double DefaultLeaseSeconds = 30.0;
	static constexpr double MaxLeaseSeconds = 30.0;

	explicit FURLabRenderOwnerAuthority(
		TSharedRef<FURLabRenderOwnerClock, ESPMode::ThreadSafe> InClock =
			FURLabRenderOwnerClock::System());

	FURLabRenderOwnerStatusResult Status();
	FURLabRenderOwnerAttachResult Attach(
		const FURLabRenderOwnerSessionScopePtr& Scope,
		double TtlSeconds = DefaultLeaseSeconds);
	FURLabRenderOwnerAuthorizationResult Authorize(
		const FURLabRenderOwnerSessionScopePtr& Scope,
		uint64 AttachmentGeneration, const FString& OpaqueToken);
	/** Exact identity-only authorization for explicit release; does not read lease time. */
	FURLabRenderOwnerAuthorizationResult AuthorizeForRelease(
		const FURLabRenderOwnerSessionScopePtr& Scope,
		uint64 AttachmentGeneration, const FString& OpaqueToken);
	FURLabRenderOwnerAdmissionResult Validate(const FURLabRenderOwnerProof& Proof);
	FURLabRenderOwnerLeaseResult Renew(const FURLabRenderOwnerProof& Proof,
		double TtlSeconds = DefaultLeaseSeconds);

	FURLabRenderOwnerRetirementResult BeginRelease(
		const FURLabRenderOwnerProof& Proof);
	FURLabRenderOwnerRetirementResult BeginLifecycleRetirement(
		const FURLabRenderOwnerSessionScopePtr& Scope);
	FURLabRenderOwnerRetirementResult BeginShutdownRetirement();
	FURLabRenderOwnerLeaseResult FinishRetirement(
		const FURLabRenderOwnerRetirementToken& Token);

	FURLabRenderOwnerAuthority(const FURLabRenderOwnerAuthority&) = delete;
	FURLabRenderOwnerAuthority& operator=(const FURLabRenderOwnerAuthority&) = delete;

private:
	friend struct FURLabRenderOwnerAuthorityTestAccess;
	enum class ESlotPhase : uint8 { Available, Active, Retiring };

	bool ReadTimedNow(double& OutNow, EURLabRenderOwnerAuthorityError& OutError,
		FString& OutMessage);
	bool IsExactScope(const FURLabRenderOwnerSessionScopePtr& Scope) const;
	bool IsExactProof(const FURLabRenderOwnerProof& Proof) const;
	bool IsExpiredAt(double NowSeconds) const;
	FURLabRenderOwnerProof MakeProof() const;
	FURLabRenderOwnerStatus BuildStatusWithoutClock() const;
	FURLabRenderOwnerStatus BuildStatusAt(double NowSeconds) const;
	FURLabRenderOwnerRetirementResult BeginRetirement();
	FURLabRenderOwnerRetirementResult RetiringResult(bool bChanged) const;

	TSharedRef<FURLabRenderOwnerClock, ESPMode::ThreadSafe> Clock;
	const FGuid AuthorityIdentity;
	ESlotPhase Phase = ESlotPhase::Available;
	bool bShutdown = false;
	bool bHasClockFloor = false;
	double LastValidClockSeconds = 0.0;
	FURLabRenderOwnerSessionScopePtr ActiveScope;
	uint64 AttachmentGeneration = 0;
	FString ActiveOpaqueToken;
	double LeaseDeadlineSeconds = 0.0;
	FURLabRenderOwnerRetirementToken ActiveRetirement;
};
