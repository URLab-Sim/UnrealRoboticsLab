// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "Bridge/DataSession.h"

#include "Math/UnrealMathUtility.h"
#include "Misc/ScopeLock.h"

namespace
{
bool IsKnownPolicy(EURLabDataPolicy Policy)
{
	return Policy == EURLabDataPolicy::Network || Policy == EURLabDataPolicy::Auto
		|| Policy == EURLabDataPolicy::Shm;
}

bool IsCommandAdapter(const FString& Adapter)
{
	return Adapter == TEXT("zmq") || Adapter == TEXT("grpc");
}

bool IsExactOwner(const FURLabPayloadOwner& Owner)
{
	return Owner.Session && Owner.Session->IsActive()
		&& Owner.BootId == Owner.Session->GetBootId()
		&& Owner.SessionId == Owner.Session->GetSessionId()
		&& !Owner.BootId.IsEmpty() && !Owner.SessionId.IsEmpty();
}

FURLabDataSelectionStatus DataSessionOk()
{
	return {};
}

FURLabDataSelectionStatus DataSessionFail(EURLabDataSelectionError Code,
	FString Message)
{
	FURLabDataSelectionStatus Result;
	Result.Code = Code;
	Result.Message = MoveTemp(Message);
	return Result;
}
}

FURLabDataSession::FURLabDataSession(FString InAdapter, EURLabDataPolicy InPolicy,
	FURLabPayloadOwner InOwner, EURLabDataTransfer InTransfer,
	FURLabShmExactRegionPtr InRegion, FURLabShmExactChallengePtr InChallenge,
	double InExpiresAtSeconds)
	: Adapter(MoveTemp(InAdapter))
	, Policy(InPolicy)
	, Owner(MoveTemp(InOwner))
	, Region(MoveTemp(InRegion))
	, Challenge(MoveTemp(InChallenge))
	, ExpiresAtSeconds(InExpiresAtSeconds)
	, Transfer(InTransfer)
{
}

TSharedPtr<FURLabDataSession, ESPMode::ThreadSafe> FURLabDataSession::CreateNetwork(
	FString Adapter, EURLabDataPolicy Policy, FURLabPayloadOwner Owner,
	FString& OutError)
{
	if (!IsCommandAdapter(Adapter) || !IsKnownPolicy(Policy)
		|| Policy == EURLabDataPolicy::Shm || !IsExactOwner(Owner))
	{
		OutError = TEXT("invalid selected-network data session");
		return nullptr;
	}
	OutError.Reset();
	return MakeShareable(new FURLabDataSession(MoveTemp(Adapter), Policy,
		MoveTemp(Owner), EURLabDataTransfer::Network, nullptr, nullptr, 0.0));
}

TSharedPtr<FURLabDataSession, ESPMode::ThreadSafe>
FURLabDataSession::CreateProvisionalShm(FString Adapter, EURLabDataPolicy Policy,
	FURLabPayloadOwner Owner, FURLabShmExactRegionPtr Region,
	FURLabShmExactChallengePtr Challenge, double ExpiresAtSeconds, FString& OutError)
{
	if (Adapter != TEXT("zmq")
		|| (Policy != EURLabDataPolicy::Auto && Policy != EURLabDataPolicy::Shm)
		|| !IsExactOwner(Owner)
		|| !Region || !Challenge || !FMath::IsFinite(ExpiresAtSeconds)
		|| ExpiresAtSeconds <= 0.0)
	{
		OutError = TEXT("invalid provisional SHM data session");
		return nullptr;
	}
	const FURLabShmExactRegionDescriptor& RegionDescriptor = Region->GetDescriptor();
	const FURLabShmExactChallengeOffer& Offer = Challenge->GetOffer();
	if (RegionDescriptor.BootId != Owner.BootId
		|| RegionDescriptor.RegionId != Offer.RegionId
		|| RegionDescriptor.Generation != Offer.RegionGeneration
		|| Offer.RegionGeneration == 0
		|| Offer.RegionGeneration > static_cast<uint64>(MAX_int64)
		|| Offer.ChallengeName.IsEmpty() || Offer.ByteLength != 32)
	{
		OutError = TEXT("SHM challenge does not match its boot region");
		return nullptr;
	}
	OutError.Reset();
	return MakeShareable(new FURLabDataSession(MoveTemp(Adapter), Policy,
		MoveTemp(Owner), EURLabDataTransfer::Provisional, MoveTemp(Region),
		MoveTemp(Challenge), ExpiresAtSeconds));
}

FURLabDataSessionSnapshot FURLabDataSession::Snapshot(double NowSeconds) const
{
	FScopeLock Lock(&Mutex);
	FURLabDataSessionSnapshot Result;
	Result.Policy = Policy;
	Result.Transfer = Transfer;
	Result.Adapter = Adapter;
	Result.Owner = Owner;
	Result.bActive = bActive && Owner.Session && Owner.Session->IsActive();
	if (Region)
	{
		Result.Region = Region->GetDescriptor();
	}
	if (Challenge)
	{
		Result.Challenge = Challenge->GetOffer();
		if (FMath::IsFinite(NowSeconds))
		{
			Result.ExpiresInSeconds = FMath::Max(0.0, ExpiresAtSeconds - NowSeconds);
		}
	}
	return Result;
}

bool FURLabDataSession::IsSelected() const
{
	FScopeLock Lock(&Mutex);
	return bActive && Owner.Session && Owner.Session->IsActive()
		&& Transfer != EURLabDataTransfer::Provisional;
}

bool FURLabDataSession::IsActive() const
{
	FScopeLock Lock(&Mutex);
	return bActive && Owner.Session && Owner.Session->IsActive();
}

void FURLabDataSession::ExpireChallengeLocked(
	FURLabShmExactChallengePtr& OutChallengeToRetire)
{
	check(!OutChallengeToRetire);
	OutChallengeToRetire = MoveTemp(Challenge);
	bChallengeExpired = true;
}

FURLabDataSelectionStatus FURLabDataSession::ValidateSelectionLocked(double NowSeconds,
	EURLabDataTransfer RequestedTransfer,
	FURLabShmExactChallengePtr& OutChallengeToRetire)
{
	if (!bActive || !Owner.Session || !Owner.Session->IsActive())
	{
		return DataSessionFail(EURLabDataSelectionError::Inactive,
			TEXT("data session is inactive"));
	}
	if (!FMath::IsFinite(NowSeconds))
	{
		return DataSessionFail(EURLabDataSelectionError::InvalidClock,
			TEXT("data negotiation clock is invalid"));
	}
	if (bSelectionFailed)
	{
		return DataSessionFail(EURLabDataSelectionError::BackingFailure,
			SelectionFailure.IsEmpty()
				? TEXT("data selection was permanently disabled") : SelectionFailure);
	}
	if (bSelectionPending)
	{
		return DataSessionFail(EURLabDataSelectionError::SelectionConflict,
			TEXT("data transfer selection is awaiting challenge retirement"));
	}
	if (Transfer != EURLabDataTransfer::Provisional)
	{
		return DataSessionFail(EURLabDataSelectionError::SelectionConflict,
			TEXT("data transfer has already been selected"));
	}
	if (bChallengeExpired || NowSeconds >= ExpiresAtSeconds)
	{
		ExpireChallengeLocked(OutChallengeToRetire);
		return DataSessionFail(EURLabDataSelectionError::NegotiationExpired,
			TEXT("SHM challenge authority has expired"));
	}
	if (!Challenge || !Region)
	{
		return DataSessionFail(EURLabDataSelectionError::ChallengeInvalid,
			TEXT("SHM challenge is unavailable"));
	}
	if (RequestedTransfer == EURLabDataTransfer::Network
		&& Policy != EURLabDataPolicy::Auto)
	{
		return DataSessionFail(EURLabDataSelectionError::SelectionConflict,
			TEXT("required SHM selection cannot switch to network"));
	}
	return DataSessionOk();
}

FURLabDataSelectionStatus FURLabDataSession::PrepareSelectNetwork(double NowSeconds,
	FURLabDataSelectionCommit& OutCommit,
	FURLabShmExactChallengePtr& OutChallengeToRetire)
{
	OutCommit = {};
	OutChallengeToRetire.Reset();
	FScopeLock Lock(&Mutex);
	FURLabDataSelectionStatus Status = ValidateSelectionLocked(NowSeconds,
		EURLabDataTransfer::Network, OutChallengeToRetire);
	if (!Status) return Status;
	if (NextSelectionSerial == 0 || NextSelectionSerial == MAX_uint64)
	{
		return DataSessionFail(EURLabDataSelectionError::BackingFailure,
			TEXT("data selection identity space is exhausted"));
	}
	bSelectionPending = true;
	PendingTransfer = EURLabDataTransfer::Network;
	PendingSelectionSerial = NextSelectionSerial++;
	OutCommit.OwnerSession = Owner.Session;
	OutCommit.DataSessionIdentity = SelectionIdentity;
	OutCommit.Serial = PendingSelectionSerial;
	OutCommit.Transfer = PendingTransfer;
	OutChallengeToRetire = MoveTemp(Challenge);
	return DataSessionOk();
}

FURLabDataSelectionStatus FURLabDataSession::PrepareSelectShm(double NowSeconds,
	const FURLabShmExactChallengeProof& Proof,
	FURLabDataSelectionCommit& OutCommit,
	FURLabShmExactChallengePtr& OutChallengeToRetire)
{
	OutCommit = {};
	OutChallengeToRetire.Reset();
	FScopeLock Lock(&Mutex);
	FURLabDataSelectionStatus Status = ValidateSelectionLocked(NowSeconds,
		EURLabDataTransfer::Shm, OutChallengeToRetire);
	if (!Status) return Status;
	const FURLabShmExactStatus VerifyStatus = Region->VerifyChallenge(Owner, Challenge, Proof);
	if (!VerifyStatus)
	{
		return DataSessionFail(
			VerifyStatus.Code == EURLabShmExactError::ChallengeInvalid
				? EURLabDataSelectionError::ChallengeInvalid
				: EURLabDataSelectionError::BackingFailure,
			VerifyStatus.Message);
	}
	if (NextSelectionSerial == 0 || NextSelectionSerial == MAX_uint64)
	{
		return DataSessionFail(EURLabDataSelectionError::BackingFailure,
			TEXT("data selection identity space is exhausted"));
	}
	bSelectionPending = true;
	PendingTransfer = EURLabDataTransfer::Shm;
	PendingSelectionSerial = NextSelectionSerial++;
	OutCommit.OwnerSession = Owner.Session;
	OutCommit.DataSessionIdentity = SelectionIdentity;
	OutCommit.Serial = PendingSelectionSerial;
	OutCommit.Transfer = PendingTransfer;
	OutChallengeToRetire = MoveTemp(Challenge);
	return DataSessionOk();
}

FURLabDataSelectionStatus FURLabDataSession::CompleteSelection(
	const FURLabDataSelectionCommit& Commit, bool bChallengeRetired,
	const FString& RetirementError)
{
	FScopeLock Lock(&Mutex);
	if (!Commit.IsValid() || Commit.OwnerSession != Owner.Session
		|| Commit.DataSessionIdentity != SelectionIdentity || !bSelectionPending
		|| Commit.Serial != PendingSelectionSerial || Commit.Transfer != PendingTransfer)
	{
		return DataSessionFail(EURLabDataSelectionError::SelectionConflict,
			TEXT("data selection completion does not match the pending choice"));
	}

	bSelectionPending = false;
	PendingSelectionSerial = 0;
	const EURLabDataTransfer CompletedTransfer = PendingTransfer;
	PendingTransfer = EURLabDataTransfer::Provisional;
	if (!bActive || !Owner.Session || !Owner.Session->IsActive())
	{
		bSelectionFailed = true;
		SelectionFailure = TEXT("data selection owner became inactive before completion");
		return DataSessionFail(EURLabDataSelectionError::Inactive, SelectionFailure);
	}
	if (!bChallengeRetired)
	{
		bSelectionFailed = true;
		SelectionFailure = RetirementError.IsEmpty()
			? TEXT("SHM challenge retirement failed") : RetirementError;
		return DataSessionFail(EURLabDataSelectionError::BackingFailure, SelectionFailure);
	}
	if (CompletedTransfer != EURLabDataTransfer::Network
		&& CompletedTransfer != EURLabDataTransfer::Shm)
	{
		bSelectionFailed = true;
		SelectionFailure = TEXT("pending data selection carried an invalid backend");
		return DataSessionFail(EURLabDataSelectionError::BackingFailure, SelectionFailure);
	}
	Transfer = CompletedTransfer;
	SelectionFailure.Reset();
	return DataSessionOk();
}

FURLabDataSelectionStatus FURLabDataSession::ExpireChallenge(double NowSeconds,
	FURLabShmExactChallengePtr& OutChallengeToRetire)
{
	OutChallengeToRetire.Reset();
	FScopeLock Lock(&Mutex);
	if (!bActive || !Owner.Session || !Owner.Session->IsActive())
	{
		return DataSessionFail(EURLabDataSelectionError::Inactive,
			TEXT("data session is inactive"));
	}
	if (!FMath::IsFinite(NowSeconds))
	{
		return DataSessionFail(EURLabDataSelectionError::InvalidClock,
			TEXT("data negotiation clock is invalid"));
	}
	if (bChallengeExpired)
	{
		return DataSessionFail(EURLabDataSelectionError::NegotiationExpired,
			TEXT("SHM challenge authority has expired"));
	}
	if (bSelectionPending || bSelectionFailed
		|| Transfer != EURLabDataTransfer::Provisional || !Challenge
		|| NowSeconds < ExpiresAtSeconds)
	{
		return DataSessionOk();
	}
	ExpireChallengeLocked(OutChallengeToRetire);
	return DataSessionFail(EURLabDataSelectionError::NegotiationExpired,
		TEXT("SHM challenge authority has expired"));
}

void FURLabDataSession::Deactivate(FURLabShmExactChallengePtr& OutChallengeToRetire)
{
	OutChallengeToRetire.Reset();
	FScopeLock Lock(&Mutex);
	bActive = false;
	bSelectionPending = false;
	PendingSelectionSerial = 0;
	PendingTransfer = EURLabDataTransfer::Provisional;
	OutChallengeToRetire = MoveTemp(Challenge);
}
