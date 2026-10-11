// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#pragma once

#include "CoreMinimal.h"

#include "Bridge/URLabPayloadStore.h"
#include "Transport/ShmExactPayload.h"

enum class EURLabDataPolicy : uint8
{
	Network,
	Auto,
	Shm,
};

enum class EURLabDataTransfer : uint8
{
	Provisional,
	Network,
	Shm,
};

enum class EURLabDataSelectionError : uint8
{
	None,
	Inactive,
	InvalidClock,
	NegotiationExpired,
	SelectionConflict,
	ChallengeInvalid,
	BackingFailure,
};

struct URLAB_API FURLabDataSelectionStatus
{
	EURLabDataSelectionError Code = EURLabDataSelectionError::None;
	FString Message;

	bool IsOk() const { return Code == EURLabDataSelectionError::None; }
	explicit operator bool() const { return IsOk(); }
};

/** Immutable selection metadata safe to copy out of the session lock. */
struct URLAB_API FURLabDataSessionSnapshot
{
	EURLabDataPolicy Policy = EURLabDataPolicy::Network;
	EURLabDataTransfer Transfer = EURLabDataTransfer::Network;
	FString Adapter;
	FURLabPayloadOwner Owner;
	FURLabShmExactRegionDescriptor Region;
	FURLabShmExactChallengeOffer Challenge;
	double ExpiresInSeconds = 0.0;
	bool bActive = false;
};

/** Opaque proof that one exact selection is awaiting external challenge retirement. */
class URLAB_API FURLabDataSelectionCommit
{
public:
	FURLabDataSelectionCommit() = default;
	bool IsValid() const
	{
		return OwnerSession.IsValid() && DataSessionIdentity.IsValid() && Serial != 0;
	}

private:
	friend class FURLabDataSession;
	FURLabPayloadSessionPtr OwnerSession;
	FGuid DataSessionIdentity;
	uint64 Serial = 0;
	EURLabDataTransfer Transfer = EURLabDataTransfer::Provisional;
};

/**
 * One protocol-v2 connection's immutable data-plane selection.
 *
 * This state machine owns only selection authority. Payload bytes and their
 * lifetime remain in FURLabPayloadStore. Challenge verification is the SHM
 * primitive's in-memory comparison and is safe under this short mutex; challenge
 * retirement is returned to the caller and must run after dispatcher/session locks.
 */
class URLAB_API FURLabDataSession
{
public:
	static TSharedPtr<FURLabDataSession, ESPMode::ThreadSafe> CreateNetwork(
		FString Adapter, EURLabDataPolicy Policy, FURLabPayloadOwner Owner,
		FString& OutError);
	static TSharedPtr<FURLabDataSession, ESPMode::ThreadSafe> CreateProvisionalShm(
		FString Adapter, EURLabDataPolicy Policy, FURLabPayloadOwner Owner,
		FURLabShmExactRegionPtr Region, FURLabShmExactChallengePtr Challenge,
		double ExpiresAtSeconds, FString& OutError);

	FURLabDataSessionSnapshot Snapshot(double NowSeconds) const;
	bool IsSelected() const;
	bool IsActive() const;

	FURLabDataSelectionStatus PrepareSelectNetwork(double NowSeconds,
		FURLabDataSelectionCommit& OutCommit,
		FURLabShmExactChallengePtr& OutChallengeToRetire);
	FURLabDataSelectionStatus PrepareSelectShm(double NowSeconds,
		const FURLabShmExactChallengeProof& Proof,
		FURLabDataSelectionCommit& OutCommit,
		FURLabShmExactChallengePtr& OutChallengeToRetire);
	/** Publish the pending backend only after detached challenge retirement succeeds. */
	FURLabDataSelectionStatus CompleteSelection(
		const FURLabDataSelectionCommit& Commit, bool bChallengeRetired,
		const FString& RetirementError);
	/** Lazily detach an elapsed provisional challenge without closing the protocol session. */
	FURLabDataSelectionStatus ExpireChallenge(double NowSeconds,
		FURLabShmExactChallengePtr& OutChallengeToRetire);

	/** One-way admission close. Challenge cleanup remains caller-owned and lock-free. */
	void Deactivate(FURLabShmExactChallengePtr& OutChallengeToRetire);

	const FURLabPayloadOwner& GetOwner() const { return Owner; }
	const FURLabShmExactRegionPtr& GetRegion() const { return Region; }

private:
	FURLabDataSession(FString InAdapter, EURLabDataPolicy InPolicy,
		FURLabPayloadOwner InOwner, EURLabDataTransfer InTransfer,
		FURLabShmExactRegionPtr InRegion, FURLabShmExactChallengePtr InChallenge,
		double InExpiresAtSeconds);

	FURLabDataSelectionStatus ValidateSelectionLocked(double NowSeconds,
		EURLabDataTransfer RequestedTransfer,
		FURLabShmExactChallengePtr& OutChallengeToRetire);
	void ExpireChallengeLocked(FURLabShmExactChallengePtr& OutChallengeToRetire);

	mutable FCriticalSection Mutex;
	const FString Adapter;
	const EURLabDataPolicy Policy;
	const FURLabPayloadOwner Owner;
	const FURLabShmExactRegionPtr Region;
	const FGuid SelectionIdentity = FGuid::NewGuid();
	FURLabShmExactChallengePtr Challenge;
	const double ExpiresAtSeconds = 0.0;
	EURLabDataTransfer Transfer = EURLabDataTransfer::Network;
	bool bActive = true;
	bool bChallengeExpired = false;
	bool bSelectionPending = false;
	bool bSelectionFailed = false;
	FString SelectionFailure;
	EURLabDataTransfer PendingTransfer = EURLabDataTransfer::Provisional;
	uint64 PendingSelectionSerial = 0;
	uint64 NextSelectionSerial = 1;
};

using FURLabDataSessionPtr = TSharedPtr<FURLabDataSession, ESPMode::ThreadSafe>;
