// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "Bridge/URLabPayloadStore.h"

#include "Containers/ContainerAllocationPolicies.h"
#include "HAL/CriticalSection.h"
#include "HAL/PlatformTime.h"
#include "Misc/Guid.h"
#include "Misc/ScopeLock.h"
#include "Transport/ShmExactPayload.h"
#include "Utils/AssetCache.h"

class FURLabPayloadStoreIdentity
{
};

namespace
{
FURLabPayloadStoreStatus Ok()
{
	return {};
}

FURLabPayloadStoreStatus Fail(EURLabPayloadStoreError Code, const TCHAR* Message)
{
	FURLabPayloadStoreStatus Out;
	Out.Code = Code;
	Out.Message = Message;
	return Out;
}

FURLabPayloadStoreStatus Fail(EURLabPayloadStoreError Code, FString Message)
{
	FURLabPayloadStoreStatus Out;
	Out.Code = Code;
	Out.Message = MoveTemp(Message);
	return Out;
}

FURLabPayloadStoreStatus FromShm(const FURLabShmExactStatus& Status)
{
	if (Status) return Ok();
	switch (Status.Code)
	{
	case EURLabShmExactError::InvalidArgument:
		return Fail(EURLabPayloadStoreError::InvalidDeclaration, Status.Message);
	case EURLabShmExactError::QuotaExceeded:
		return Fail(EURLabPayloadStoreError::LimitExceeded, Status.Message);
	case EURLabShmExactError::NotFound:
		return Fail(EURLabPayloadStoreError::NotFound, Status.Message);
	case EURLabShmExactError::IdentityMismatch:
	case EURLabShmExactError::InvalidRegion:
		return Fail(EURLabPayloadStoreError::IdentityMismatch, Status.Message);
	case EURLabShmExactError::Bounds:
		return Fail(EURLabPayloadStoreError::OutOfBounds, Status.Message);
	case EURLabShmExactError::IntegrityMismatch:
		return Fail(EURLabPayloadStoreError::IntegrityMismatch, Status.Message);
	case EURLabShmExactError::Unsupported:
	case EURLabShmExactError::IoFailure:
	default:
		return Fail(EURLabPayloadStoreError::BackingFailure, Status.Message);
	}
}

struct FPhysicalUsage
{
	int64 Bytes = 0;
	int32 Count = 0;
};

struct FURLabPayloadAccounting : TSharedFromThis<FURLabPayloadAccounting, ESPMode::ThreadSafe>
{
	explicit FURLabPayloadAccounting(const FURLabPayloadStoreLimits& InLimits)
		: Limits(InLimits)
	{
	}

	bool Reserve(const FString& SessionId, int64 Bytes)
	{
		FScopeLock Lock(&Mutex);
		FPhysicalUsage& Session = Sessions.FindOrAdd(SessionId);
		if (Global.Bytes > Limits.MaxGlobalBytes - Bytes
			|| Session.Bytes > Limits.MaxSessionBytes - Bytes
			|| Global.Count >= Limits.MaxBlobCount
			|| Session.Count >= Limits.MaxSessionPhysicalObjectCount)
		{
			if (Session.Bytes == 0 && Session.Count == 0) Sessions.Remove(SessionId);
			return false;
		}
		Global.Bytes += Bytes;
		++Global.Count;
		Session.Bytes += Bytes;
		++Session.Count;
		return true;
	}

	void Release(const FString& SessionId, int64 Bytes)
	{
		FScopeLock Lock(&Mutex);
		Global.Bytes -= Bytes;
		--Global.Count;
		if (FPhysicalUsage* Session = Sessions.Find(SessionId))
		{
			Session->Bytes -= Bytes;
			--Session->Count;
			if (Session->Bytes == 0 && Session->Count == 0) Sessions.Remove(SessionId);
		}
	}

	FPhysicalUsage Get(const FString* SessionId = nullptr) const
	{
		FScopeLock Lock(&Mutex);
		if (!SessionId) return Global;
		if (const FPhysicalUsage* Usage = Sessions.Find(*SessionId)) return *Usage;
		return {};
	}

	const FURLabPayloadStoreLimits Limits;
	mutable FCriticalSection Mutex;
	FPhysicalUsage Global;
	TMap<FString, FPhysicalUsage> Sessions;
};
}

class FURLabPayloadPhysicalReservation
{
public:
	FURLabPayloadPhysicalReservation(
		TSharedRef<FURLabPayloadAccounting, ESPMode::ThreadSafe> InAccounting,
		FString InSessionId, int64 InBytes)
		: Accounting(MoveTemp(InAccounting)), SessionId(MoveTemp(InSessionId)), Bytes(InBytes)
	{
	}

	~FURLabPayloadPhysicalReservation()
	{
		Accounting->Release(SessionId, Bytes);
	}

private:
	TSharedRef<FURLabPayloadAccounting, ESPMode::ThreadSafe> Accounting;
	FString SessionId;
	int64 Bytes = 0;
};

FURLabPayloadProducer::FURLabPayloadProducer(FString InBlobId, int64 InDeclaredByteLength,
	FURLabPayloadPhysicalReservationPtr InReservation)
	: BlobId(MoveTemp(InBlobId)), DeclaredByteLength(InDeclaredByteLength),
	  Reservation(MoveTemp(InReservation))
{
}

FURLabPayloadProducer::FURLabPayloadProducer(FString InBlobId, int64 InDeclaredByteLength,
	FURLabShmExactRegionPtr InRegion,
	FURLabPayloadPhysicalReservationPtr InCpuReservation,
	FURLabPayloadPhysicalReservationPtr InShmReservation)
	: BackingKind(EURLabPayloadBackingKind::ShmSealedFile), BlobId(MoveTemp(InBlobId)),
	  DeclaredByteLength(InDeclaredByteLength), Reservation(MoveTemp(InCpuReservation)),
	  ShmReservation(MoveTemp(InShmReservation)), ShmRegion(MoveTemp(InRegion))
{
}

FURLabPayloadProducer::~FURLabPayloadProducer() = default;

FURLabImmutablePayload::FURLabImmutablePayload(const FURLabPayloadOwner& InOwner,
	TSharedRef<FURLabPayloadStoreIdentity, ESPMode::ThreadSafe> InStoreIdentity,
	TArray<uint8>&& InBytes, FString InSha256,
	FURLabPayloadPhysicalReservationPtr InReservation)
	: Owner(InOwner), StoreIdentity(MoveTemp(InStoreIdentity)), ByteLength(InBytes.Num()),
	  Reservation(MoveTemp(InReservation)), Bytes(MoveTemp(InBytes)), Sha256(MoveTemp(InSha256))
{
}

FURLabImmutablePayload::FURLabImmutablePayload(const FURLabPayloadOwner& InOwner,
	TSharedRef<FURLabPayloadStoreIdentity, ESPMode::ThreadSafe> InStoreIdentity,
	FString InSha256, int64 InByteLength,
	TSharedPtr<FURLabShmExactRegion, ESPMode::ThreadSafe> InRegion,
	TSharedPtr<FURLabShmExactSealedPayload, ESPMode::ThreadSafe> InSealed)
	: BackingKind(EURLabPayloadBackingKind::ShmSealedFile), Owner(InOwner),
	  StoreIdentity(MoveTemp(InStoreIdentity)), ByteLength(InByteLength),
	  Sha256(MoveTemp(InSha256)), ShmRegion(MoveTemp(InRegion)),
	  ShmSealed(MoveTemp(InSealed))
{
}

FURLabImmutablePayload::~FURLabImmutablePayload() = default;

bool FURLabImmutablePayload::TryGetNetworkBytes(TConstArrayView<uint8>& OutBytes) const
{
	OutBytes = TConstArrayView<uint8>();
	if (BackingKind != EURLabPayloadBackingKind::NetworkBytes) return false;
	OutBytes = Bytes;
	return true;
}

FURLabPayloadCopy::FURLabPayloadCopy(TArray<uint8>&& InBytes,
	FURLabPayloadPhysicalReservationPtr InReservation)
	: Reservation(MoveTemp(InReservation)), Bytes(MoveTemp(InBytes))
{
}

FURLabPayloadCopy::FURLabPayloadCopy(
	TSharedPtr<const FURLabShmExactCopiedBytes, ESPMode::ThreadSafe> InShmCopy)
	: ShmCopy(MoveTemp(InShmCopy))
{
}

FURLabPayloadCopy::~FURLabPayloadCopy() = default;

const TArray<uint8>& FURLabPayloadCopy::GetBytes() const
{
	return ShmCopy ? ShmCopy->GetBytes() : Bytes;
}

int64 FURLabPayloadCopy::Num() const
{
	return ShmCopy ? ShmCopy->Num() : Bytes.Num();
}

FURLabPayloadOwner FURLabPayloadOwner::FromSession(const FURLabPayloadSessionPtr& InSession)
{
	FURLabPayloadOwner Out;
	Out.Session = InSession;
	if (InSession)
	{
		Out.BootId = InSession->GetBootId();
		Out.SessionId = InSession->GetSessionId();
	}
	return Out;
}

namespace
{
struct FShmCatalogRetirement
{
	FURLabPayloadOwner Owner;
	FURLabShmExactRegionPtr Region;
	FURLabShmExactSealedPayloadPtr Sealed;
};

/** Declared before each catalog lock so destruction always performs file work after unlock. */
struct FPayloadDeferredReclaims
{
	~FPayloadDeferredReclaims()
	{
		for (FShmCatalogRetirement& Retirement : ShmRetirements)
		{
			if (Retirement.Region && Retirement.Sealed)
				Retirement.Region->Release(Retirement.Owner, Retirement.Sealed);
		}
	}

	TArray<FShmCatalogRetirement> ShmRetirements;
	TArray<FURLabShmExactStagingPtr> ShmStaging;
	TArray<FURLabPayloadPin> Payloads;
	TArray<FURLabPayloadPhysicalReservationPtr> Reservations;
	TArray<FURLabPayloadProducerPtr> Producers;
	TArray<TArray<uint8>> NetworkBytes;
};
}

struct FURLabPayloadStore::FState
{
	struct FRange
	{
		int64 Begin = 0;
		int64 End = 0;
	};

	struct FStaging
	{
		enum class EKind : uint8
		{
			Chunked,
			Producer,
			ShmProducerReady,
			ShmProducerSealing,
			ShmPreparing,
			ShmReady,
			ShmSealing
		};

		// Declared first so reverse destruction frees byte/interval storage before
		// returning its physical reservation to concurrent writers.
		FURLabPayloadPhysicalReservationPtr Reservation;
		FURLabPayloadProducerPtr Producer;
		FURLabShmExactRegionPtr ShmRegion;
		FURLabShmExactStagingPtr ShmStaging;
		FURLabPayloadOwner Owner;
		EKind Kind = EKind::Chunked;
		FString ExpectedSha256;
		TArray<uint8> Bytes;
		TArray<FRange> Ranges;
		int64 DeclaredByteLength = 0;
		int64 CoveredBytes = 0;
		double ExpiresAt = 0.0;
	};

	struct FSealed
	{
		FURLabPayloadOwner Owner;
		FURLabStoredPayloadDescriptor Descriptor;
		FURLabPayloadPin Payload;
		FURLabShmExactRegionPtr ShmRegion;
		FURLabShmExactSealedPayloadPtr ShmSealed;
		double ExpiresAt = 0.0;
		bool bAcknowledged = false;
	};

	struct FExpired
	{
		FString BootId;
		FString SessionId;
		TWeakPtr<FURLabPayloadSession, ESPMode::ThreadSafe> Session;
		double ForgetAt = 0.0;
	};

	FState(FString InBootId, FURLabPayloadStoreLimits InLimits,
		FURLabPayloadStore::FMonotonicClock InClock)
		: BootId(MoveTemp(InBootId)), Limits(InLimits),
		  Accounting(MakeShared<FURLabPayloadAccounting, ESPMode::ThreadSafe>(Limits)),
		  Identity(MakeShared<FURLabPayloadStoreIdentity, ESPMode::ThreadSafe>())
	{
		if (InClock)
			Clock = MoveTemp(InClock);
		else
			Clock = [] { return FPlatformTime::Seconds(); };
		if (BootId.IsEmpty())
		{
			LimitsError = TEXT("payload store boot identity must not be empty");
		}
		else if (Limits.MaxBlobBytes <= 0 || Limits.MaxBlobBytes > MAX_int32
			|| Limits.MaxChunkBytes <= 0 || Limits.MaxChunkBytes > Limits.MaxBlobBytes
			|| Limits.MaxChunksPerBlob <= 0 || Limits.MaxSessionBytes < Limits.MaxBlobBytes
			|| Limits.MaxGlobalBytes < Limits.MaxSessionBytes || Limits.MaxBlobCount <= 0
			|| Limits.MaxSessionBlobCount <= 0
			|| Limits.MaxSessionBlobCount > Limits.MaxBlobCount
			|| Limits.MaxSessionPhysicalObjectCount <= 0
			|| Limits.MaxSessionPhysicalObjectCount > Limits.MaxBlobCount
			|| Limits.MaxActiveSessions <= 0
			|| !FMath::IsFinite(Limits.StagingTtlSeconds) || Limits.StagingTtlSeconds <= 0.0
			|| !FMath::IsFinite(Limits.SealedTtlSeconds) || Limits.SealedTtlSeconds <= 0.0
			|| !FMath::IsFinite(Limits.ExpiredTombstoneTtlSeconds)
			|| Limits.ExpiredTombstoneTtlSeconds <= 0.0)
		{
			LimitsError = TEXT("payload store limits are invalid or inconsistent");
		}
	}

	FURLabPayloadStoreStatus ValidateOwner(const FURLabPayloadOwner& Owner) const
	{
		if (!LimitsError.IsEmpty())
			return Fail(EURLabPayloadStoreError::InvalidLimits, LimitsError);
		if (Owner.BootId != BootId || Owner.SessionId.IsEmpty() || !Owner.Session
			|| Owner.Session->GetBootId() != Owner.BootId
			|| Owner.Session->GetSessionId() != Owner.SessionId)
			return Fail(EURLabPayloadStoreError::InvalidIdentity,
				TEXT("payload owner must carry this boot's exact non-empty session handle"));
		const FURLabPayloadSessionPtr* Live = Sessions.Find(Owner.SessionId);
		if (!Owner.Session->IsActive() || !Live || *Live != Owner.Session)
			return Fail(EURLabPayloadStoreError::SessionRevoked,
				TEXT("payload session is inactive or has been revoked"));
		return Ok();
	}

	FURLabPayloadStoreStatus ReadNow(double& OutNow)
	{
		OutNow = Clock();
		if (!FMath::IsFinite(OutNow) || OutNow < 0.0
			|| (bHasClockSample && OutNow < LastClockSample))
			return Fail(EURLabPayloadStoreError::InvalidClock,
				TEXT("payload monotonic clock returned an invalid value"));
		LastClockSample = OutNow;
		bHasClockSample = true;
		return Ok();
	}

	int32 CountLiveEntries(const FString& SessionId) const
	{
		int32 Count = 0;
		for (const TPair<FString, FStaging>& Entry : Staging)
			if (Entry.Value.Owner.SessionId == SessionId) ++Count;
		for (const TPair<FString, FSealed>& Entry : Sealed)
			if (Entry.Value.Owner.SessionId == SessionId) ++Count;
		return Count;
	}

	FString MintBlobId() const
	{
		FString BlobId;
		do
		{
			BlobId = FGuid::NewGuid().ToString(EGuidFormats::DigitsWithHyphensLower);
		}
		while (Staging.Contains(BlobId) || Sealed.Contains(BlobId)
			|| Expired.Contains(BlobId));
		return BlobId;
	}

	void RecordExpired(const FString& BlobId, const FURLabPayloadOwner& Owner, double Now)
	{
		while (ExpiredOrder.Num() >= Limits.MaxBlobCount)
		{
			Expired.Remove(ExpiredOrder[0]);
			ExpiredOrder.RemoveAt(0, 1, EAllowShrinking::No);
		}
		FExpired Tombstone;
		Tombstone.BootId = Owner.BootId;
		Tombstone.SessionId = Owner.SessionId;
		Tombstone.Session = Owner.Session;
		Tombstone.ForgetAt = Now + Limits.ExpiredTombstoneTtlSeconds;
		Expired.Add(BlobId, MoveTemp(Tombstone));
		ExpiredOrder.Add(BlobId);
	}

	FURLabPayloadStoreStatus Missing(const FURLabPayloadOwner& Owner,
		const FString& BlobId) const
	{
		if (const FExpired* Tombstone = Expired.Find(BlobId))
		{
			if (Tombstone->BootId == Owner.BootId && Tombstone->SessionId == Owner.SessionId
				&& Tombstone->Session.Pin() == Owner.Session)
			{
				return Fail(EURLabPayloadStoreError::Expired, TEXT("payload has expired"));
			}
		}
		return Fail(EURLabPayloadStoreError::NotFound, TEXT("payload was not found"));
	}

	void DeferStaging(FStaging& Entry, FPayloadDeferredReclaims& Deferred)
	{
		Deferred.ShmStaging.Add(MoveTemp(Entry.ShmStaging));
		Deferred.Reservations.Add(MoveTemp(Entry.Reservation));
		Deferred.Producers.Add(MoveTemp(Entry.Producer));
		Deferred.NetworkBytes.Add(MoveTemp(Entry.Bytes));
	}

	void DeferSealed(FSealed& Entry, FPayloadDeferredReclaims& Deferred)
	{
		if (Entry.ShmRegion && Entry.ShmSealed)
		{
			FShmCatalogRetirement Retirement;
			Retirement.Owner = Entry.Owner;
			Retirement.Region = MoveTemp(Entry.ShmRegion);
			Retirement.Sealed = MoveTemp(Entry.ShmSealed);
			Deferred.ShmRetirements.Add(MoveTemp(Retirement));
		}
		Deferred.Payloads.Add(MoveTemp(Entry.Payload));
	}

	void SweepExpiredLocked(double Now, FURLabPayloadSweepResult& Out,
		FPayloadDeferredReclaims& Deferred)
	{
		for (auto It = Staging.CreateIterator(); It; ++It)
		{
			if (It.Value().ExpiresAt > Now) continue;
			RecordExpired(It.Key(), It.Value().Owner, Now);
			DeferStaging(It.Value(), Deferred);
			It.RemoveCurrent();
			++Out.StagingRemoved;
		}
		for (auto It = Sealed.CreateIterator(); It; ++It)
		{
			if (It.Value().ExpiresAt > Now) continue;
			RecordExpired(It.Key(), It.Value().Owner, Now);
			DeferSealed(It.Value(), Deferred);
			It.RemoveCurrent();
			++Out.SealedRemoved;
		}
		for (int32 Index = ExpiredOrder.Num() - 1; Index >= 0; --Index)
		{
			const FExpired* Tombstone = Expired.Find(ExpiredOrder[Index]);
			if (Tombstone && Tombstone->ForgetAt > Now) continue;
			Expired.Remove(ExpiredOrder[Index]);
			ExpiredOrder.RemoveAt(Index, 1, EAllowShrinking::No);
			++Out.TombstonesForgotten;
		}
	}

	FURLabStoredPayloadDescriptor Describe(const FSealed& Entry, double Now) const
	{
		FURLabStoredPayloadDescriptor Out = Entry.Descriptor;
		Out.ExpiresInSeconds = FMath::Max(0.0, Entry.ExpiresAt - Now);
		return Out;
	}

	FString BootId;
	FURLabPayloadStoreLimits Limits;
	FString LimitsError;
	FURLabPayloadStore::FMonotonicClock Clock;
	double LastClockSample = 0.0;
	bool bHasClockSample = false;
	TSharedRef<FURLabPayloadAccounting, ESPMode::ThreadSafe> Accounting;
	TSharedRef<FURLabPayloadStoreIdentity, ESPMode::ThreadSafe> Identity;
	mutable FCriticalSection Mutex;
	TMap<FString, FStaging> Staging;
	TMap<FString, FSealed> Sealed;
	TMap<FString, FExpired> Expired;
	TArray<FString> ExpiredOrder;
	TMap<FString, FURLabPayloadSessionPtr> Sessions;
};

FURLabPayloadStore::FURLabPayloadStore(FString InBootId, FURLabPayloadStoreLimits InLimits,
	FMonotonicClock InClock)
	: State(MakeUnique<FState>(MoveTemp(InBootId), InLimits, MoveTemp(InClock)))
{
}

FURLabPayloadStore::~FURLabPayloadStore()
{
	{
		FPayloadDeferredReclaims Deferred;
		{
			FScopeLock Lock(&State->Mutex);
			for (TPair<FString, FURLabPayloadSessionPtr>& Entry : State->Sessions)
			{
				if (Entry.Value) Entry.Value->Deactivate();
			}
			for (TPair<FString, FState::FStaging>& Entry : State->Staging)
				State->DeferStaging(Entry.Value, Deferred);
			for (TPair<FString, FState::FSealed>& Entry : State->Sealed)
				State->DeferSealed(Entry.Value, Deferred);
			State->Staging.Reset();
			State->Sealed.Reset();
			State->Sessions.Reset();
		}
	}
}

FURLabPayloadStoreStatus FURLabPayloadStore::OpenSession(const FString& BootId,
	const FString& SessionId, FURLabPayloadSessionPtr& OutSession)
{
	OutSession.Reset();
	FPayloadDeferredReclaims Deferred;
	FScopeLock Lock(&State->Mutex);
	double Now = 0.0;
	if (FURLabPayloadStoreStatus Status = State->ReadNow(Now); !Status) return Status;
	FURLabPayloadSweepResult Ignored;
	State->SweepExpiredLocked(Now, Ignored, Deferred);
	if (!State->LimitsError.IsEmpty())
		return Fail(EURLabPayloadStoreError::InvalidLimits, State->LimitsError);
	if (BootId != State->BootId || SessionId.IsEmpty())
		return Fail(EURLabPayloadStoreError::InvalidIdentity,
			TEXT("payload session must name this boot and a non-empty session"));
	if (const FURLabPayloadSessionPtr* Existing = State->Sessions.Find(SessionId))
	{
		if (!*Existing || !(*Existing)->IsActive())
		{
			return Fail(EURLabPayloadStoreError::SessionRevoked,
				TEXT("payload session is closing and must be reclaimed before reuse"));
		}
		OutSession = *Existing;
		return Ok();
	}
	if (State->Sessions.Num() >= State->Limits.MaxActiveSessions)
		return Fail(EURLabPayloadStoreError::LimitExceeded,
			TEXT("payload active-session limit is exhausted"));
	OutSession = MakeShareable(new FURLabPayloadSession(BootId, SessionId));
	State->Sessions.Add(SessionId, OutSession);
	return Ok();
}

FURLabPayloadStoreStatus FURLabPayloadStore::BeginWrite(const FURLabPayloadOwner& Owner,
	int64 ByteLength, const FString& ExpectedSha256, FString& OutBlobId)
{
	OutBlobId.Reset();
	FPayloadDeferredReclaims Deferred;
	FScopeLock Lock(&State->Mutex);
	double Now = 0.0;
	if (FURLabPayloadStoreStatus Status = State->ReadNow(Now); !Status) return Status;
	FURLabPayloadSweepResult Ignored;
	State->SweepExpiredLocked(Now, Ignored, Deferred);
	if (FURLabPayloadStoreStatus Status = State->ValidateOwner(Owner); !Status) return Status;
	if (State->CountLiveEntries(Owner.SessionId) >= State->Limits.MaxSessionBlobCount)
		return Fail(EURLabPayloadStoreError::LimitExceeded,
			TEXT("payload per-session live-object limit is exhausted"));
	if (ByteLength <= 0 || ByteLength > State->Limits.MaxBlobBytes)
		return Fail(EURLabPayloadStoreError::InvalidDeclaration,
			TEXT("payload byte length is outside the declared blob limit"));
	if (ExpectedSha256.Len() != 64)
		return Fail(EURLabPayloadStoreError::InvalidDeclaration,
			TEXT("expected SHA-256 must be exactly 64 hexadecimal characters"));
	const FString CanonicalSha = ExpectedSha256.ToLower();
	if (!FURLabAssetCache::IsValidSha256Hex(CanonicalSha))
		return Fail(EURLabPayloadStoreError::InvalidDeclaration,
			TEXT("expected SHA-256 contains a non-hexadecimal character"));
	if (!State->Accounting->Reserve(Owner.SessionId, ByteLength))
		return Fail(EURLabPayloadStoreError::LimitExceeded,
			TEXT("payload physical byte or blob-count budget is exhausted"));

	FURLabPayloadPhysicalReservationPtr Reservation =
		MakeShared<FURLabPayloadPhysicalReservation, ESPMode::ThreadSafe>(
			State->Accounting, Owner.SessionId, ByteLength);
	const FString BlobId = State->MintBlobId();

	FState::FStaging Entry;
	Entry.Owner = Owner;
	Entry.ExpectedSha256 = CanonicalSha;
	Entry.DeclaredByteLength = ByteLength;
	Entry.ExpiresAt = Now + State->Limits.StagingTtlSeconds;
	Entry.Bytes.Reserve(static_cast<int32>(ByteLength));
	Entry.Bytes.SetNumUninitialized(static_cast<int32>(ByteLength));
	Entry.Reservation = MoveTemp(Reservation);
	State->Staging.Add(BlobId, MoveTemp(Entry));
	OutBlobId = BlobId;
	return Ok();
}

FURLabPayloadStoreStatus FURLabPayloadStore::BeginShmWrite(const FURLabPayloadOwner& Owner,
	const FURLabShmExactRegionPtr& Region, int64 ByteLength,
	const FString& ExpectedSha256, FURLabPayloadShmStagingGrant& OutGrant)
{
	OutGrant = {};
	if (!Region)
		return Fail(EURLabPayloadStoreError::InvalidDeclaration,
			TEXT("SHM payload begin requires a live boot region"));

	FPayloadDeferredReclaims Deferred;
	FString BlobId;
	FString CanonicalSha;
	{
		FScopeLock Lock(&State->Mutex);
		double Now = 0.0;
		if (FURLabPayloadStoreStatus Status = State->ReadNow(Now); !Status) return Status;
		FURLabPayloadSweepResult Ignored;
		State->SweepExpiredLocked(Now, Ignored, Deferred);
		if (FURLabPayloadStoreStatus Status = State->ValidateOwner(Owner); !Status) return Status;
		if (State->CountLiveEntries(Owner.SessionId) >= State->Limits.MaxSessionBlobCount)
			return Fail(EURLabPayloadStoreError::LimitExceeded,
				TEXT("payload per-session live-object limit is exhausted"));
		if (ByteLength <= 0 || ByteLength > State->Limits.MaxBlobBytes)
			return Fail(EURLabPayloadStoreError::InvalidDeclaration,
				TEXT("SHM payload byte length is outside the declared blob limit"));
		if (ExpectedSha256.Len() != 64)
			return Fail(EURLabPayloadStoreError::InvalidDeclaration,
				TEXT("expected SHA-256 must be exactly 64 hexadecimal characters"));
		CanonicalSha = ExpectedSha256.ToLower();
		if (!FURLabAssetCache::IsValidSha256Hex(CanonicalSha))
			return Fail(EURLabPayloadStoreError::InvalidDeclaration,
				TEXT("expected SHA-256 contains a non-hexadecimal character"));

		BlobId = State->MintBlobId();
		FState::FStaging Entry;
		Entry.Owner = Owner;
		Entry.Kind = FState::FStaging::EKind::ShmPreparing;
		Entry.ExpectedSha256 = CanonicalSha;
		Entry.DeclaredByteLength = ByteLength;
		Entry.ExpiresAt = Now + State->Limits.StagingTtlSeconds;
		State->Staging.Add(BlobId, MoveTemp(Entry));
	}

	FURLabShmExactStagingPtr Prepared;
	const FURLabShmExactStatus PrepareStatus = Region->CreateStaging(
		*this, Owner, ByteLength, CanonicalSha, Prepared);
	if (!PrepareStatus)
	{
		FScopeLock Lock(&State->Mutex);
		if (FState::FStaging* Entry = State->Staging.Find(BlobId);
			Entry && Entry->Owner == Owner
			&& Entry->Kind == FState::FStaging::EKind::ShmPreparing)
		{
			State->Staging.Remove(BlobId);
		}
		return FromShm(PrepareStatus);
	}

	{
		FScopeLock Lock(&State->Mutex);
		double Now = 0.0;
		if (FURLabPayloadStoreStatus Status = State->ReadNow(Now); !Status) return Status;
		FURLabPayloadSweepResult Ignored;
		State->SweepExpiredLocked(Now, Ignored, Deferred);
		if (FURLabPayloadStoreStatus Status = State->ValidateOwner(Owner); !Status) return Status;
		FState::FStaging* Entry = State->Staging.Find(BlobId);
		if (!Entry) return State->Missing(Owner, BlobId);
		if (!(Entry->Owner == Owner)
			|| Entry->Kind != FState::FStaging::EKind::ShmPreparing)
		{
			return Fail(EURLabPayloadStoreError::IdentityMismatch,
				TEXT("SHM payload staging admission was replaced during preparation"));
		}
		Entry->Kind = FState::FStaging::EKind::ShmReady;
		Entry->ShmRegion = Region;
		Entry->ShmStaging = Prepared;
		const FURLabShmExactStagingGrant& Primitive = Prepared->GetGrant();
		OutGrant.BlobId = BlobId;
		OutGrant.RegionId = Primitive.RegionId;
		OutGrant.RegionGeneration = Primitive.Generation;
		OutGrant.ObjectName = Primitive.ObjectName;
		OutGrant.ByteLength = Primitive.ByteLength;
		OutGrant.Sha256 = Primitive.ExpectedSha256;
		OutGrant.ExpiresInSeconds = FMath::Max(0.0, Entry->ExpiresAt - Now);
	}
	return Ok();
}

FURLabPayloadStoreStatus FURLabPayloadStore::BeginProduce(const FURLabPayloadOwner& Owner,
	int64 ByteLength, FURLabPayloadProducerPtr& OutProducer)
{
	OutProducer.Reset();
	FPayloadDeferredReclaims Deferred;
	FScopeLock Lock(&State->Mutex);
	double Now = 0.0;
	if (FURLabPayloadStoreStatus Status = State->ReadNow(Now); !Status) return Status;
	FURLabPayloadSweepResult Ignored;
	State->SweepExpiredLocked(Now, Ignored, Deferred);
	if (FURLabPayloadStoreStatus Status = State->ValidateOwner(Owner); !Status) return Status;
	if (ByteLength <= 0 || ByteLength > State->Limits.MaxBlobBytes)
		return Fail(EURLabPayloadStoreError::InvalidDeclaration,
			TEXT("producer byte length is outside the declared blob limit"));
	if (State->CountLiveEntries(Owner.SessionId) >= State->Limits.MaxSessionBlobCount)
		return Fail(EURLabPayloadStoreError::LimitExceeded,
			TEXT("payload per-session live-object limit is exhausted"));
	if (!State->Accounting->Reserve(Owner.SessionId, ByteLength))
		return Fail(EURLabPayloadStoreError::LimitExceeded,
			TEXT("payload physical byte or object-count budget is exhausted"));

	FURLabPayloadPhysicalReservationPtr Reservation =
		MakeShared<FURLabPayloadPhysicalReservation, ESPMode::ThreadSafe>(
			State->Accounting, Owner.SessionId, ByteLength);
	const FString BlobId = State->MintBlobId();
	OutProducer = MakeShareable(new FURLabPayloadProducer(BlobId, ByteLength,
		MoveTemp(Reservation)));
	FState::FStaging Entry;
	Entry.Owner = Owner;
	Entry.Kind = FState::FStaging::EKind::Producer;
	Entry.Producer = OutProducer;
	Entry.DeclaredByteLength = ByteLength;
	Entry.ExpiresAt = Now + State->Limits.StagingTtlSeconds;
	State->Staging.Add(BlobId, MoveTemp(Entry));
	return Ok();
}

FURLabPayloadStoreStatus FURLabPayloadStore::BeginShmProduce(
	const FURLabPayloadOwner& Owner, const FURLabShmExactRegionPtr& Region,
	int64 ByteLength, FURLabPayloadProducerPtr& OutProducer)
{
	OutProducer.Reset();
	FPayloadDeferredReclaims Deferred;
	FScopeLock Lock(&State->Mutex);
	double Now = 0.0;
	if (FURLabPayloadStoreStatus Status = State->ReadNow(Now); !Status) return Status;
	FURLabPayloadSweepResult Ignored;
	State->SweepExpiredLocked(Now, Ignored, Deferred);
	if (FURLabPayloadStoreStatus Status = State->ValidateOwner(Owner); !Status) return Status;
	if (!Region || Region->GetDescriptor().BootId != Owner.BootId)
		return Fail(EURLabPayloadStoreError::InvalidIdentity,
			TEXT("SHM producer requires this boot's retained region"));
	if (ByteLength <= 0 || ByteLength > State->Limits.MaxBlobBytes)
		return Fail(EURLabPayloadStoreError::InvalidDeclaration,
			TEXT("SHM producer byte length is outside the declared blob limit"));
	if (State->CountLiveEntries(Owner.SessionId) >= State->Limits.MaxSessionBlobCount)
		return Fail(EURLabPayloadStoreError::LimitExceeded,
			TEXT("payload per-session live-object limit is exhausted"));
	if (!State->Accounting->Reserve(Owner.SessionId, ByteLength))
		return Fail(EURLabPayloadStoreError::LimitExceeded,
			TEXT("SHM producer CPU byte or object-count budget is exhausted"));
	if (!State->Accounting->Reserve(Owner.SessionId, ByteLength))
	{
		State->Accounting->Release(Owner.SessionId, ByteLength);
		return Fail(EURLabPayloadStoreError::LimitExceeded,
			TEXT("SHM producer sealed-file byte or object-count budget is exhausted"));
	}

	FURLabPayloadPhysicalReservationPtr CpuReservation =
		MakeShared<FURLabPayloadPhysicalReservation, ESPMode::ThreadSafe>(
			State->Accounting, Owner.SessionId, ByteLength);
	FURLabPayloadPhysicalReservationPtr ShmReservation =
		MakeShared<FURLabPayloadPhysicalReservation, ESPMode::ThreadSafe>(
			State->Accounting, Owner.SessionId, ByteLength);
	const FString BlobId = State->MintBlobId();
	OutProducer = MakeShareable(new FURLabPayloadProducer(BlobId, ByteLength, Region,
		MoveTemp(CpuReservation), MoveTemp(ShmReservation)));
	FState::FStaging Entry;
	Entry.Owner = Owner;
	Entry.Kind = FState::FStaging::EKind::ShmProducerReady;
	Entry.Producer = OutProducer;
	Entry.DeclaredByteLength = ByteLength;
	Entry.ExpiresAt = Now + State->Limits.StagingTtlSeconds;
	State->Staging.Add(BlobId, MoveTemp(Entry));
	return Ok();
}

FURLabPayloadStoreStatus FURLabPayloadStore::WriteChunk(const FURLabPayloadOwner& Owner,
	const FString& BlobId, int64 Offset, TConstArrayView<uint8> Bytes)
{
	FPayloadDeferredReclaims Deferred;
	FScopeLock Lock(&State->Mutex);
	double Now = 0.0;
	if (FURLabPayloadStoreStatus Status = State->ReadNow(Now); !Status) return Status;
	FURLabPayloadSweepResult Ignored;
	State->SweepExpiredLocked(Now, Ignored, Deferred);
	if (FURLabPayloadStoreStatus Status = State->ValidateOwner(Owner); !Status) return Status;
	FState::FStaging* Entry = State->Staging.Find(BlobId);
	if (!Entry)
		return State->Missing(Owner, BlobId);
	if (!(Entry->Owner == Owner))
		return Fail(EURLabPayloadStoreError::IdentityMismatch,
			TEXT("staging payload belongs to another owner"));
	if (Entry->Kind != FState::FStaging::EKind::Chunked)
		return Fail(EURLabPayloadStoreError::InvalidChunk,
			TEXT("direct-producer admission does not accept network chunks"));
	if (Bytes.Num() <= 0)
		return Fail(EURLabPayloadStoreError::InvalidChunk, TEXT("payload chunk must not be empty"));
	if (Bytes.Num() > State->Limits.MaxChunkBytes)
		return Fail(EURLabPayloadStoreError::ChunkTooLarge,
			TEXT("payload chunk exceeds the declared chunk limit"));
	if (Entry->Ranges.Num() >= State->Limits.MaxChunksPerBlob)
		return Fail(EURLabPayloadStoreError::ChunkLimitExceeded,
			TEXT("payload exceeds the declared chunk-count limit"));
	if (Offset < 0 || Offset > Entry->Bytes.Num()
		|| Bytes.Num() > static_cast<int64>(Entry->Bytes.Num()) - Offset)
		return Fail(EURLabPayloadStoreError::OutOfBounds,
			TEXT("payload chunk range is outside the declared blob"));

	const int64 End = Offset + Bytes.Num();
	int32 InsertAt = 0;
	while (InsertAt < Entry->Ranges.Num() && Entry->Ranges[InsertAt].Begin < Offset) ++InsertAt;
	if ((InsertAt > 0 && Entry->Ranges[InsertAt - 1].End > Offset)
		|| (InsertAt < Entry->Ranges.Num() && Entry->Ranges[InsertAt].Begin < End))
	{
		return Fail(EURLabPayloadStoreError::Overlap,
			TEXT("payload chunk overlaps bytes that were already written"));
	}

	FMemory::Memcpy(Entry->Bytes.GetData() + Offset, Bytes.GetData(), Bytes.Num());
	FState::FRange WrittenRange;
	WrittenRange.Begin = Offset;
	WrittenRange.End = End;
	Entry->Ranges.Insert(MoveTemp(WrittenRange), InsertAt);
	Entry->CoveredBytes += Bytes.Num();
	return Ok();
}

FURLabPayloadStoreStatus FURLabPayloadStore::Commit(const FURLabPayloadOwner& Owner,
	const FString& BlobId, FURLabStoredPayloadDescriptor& OutDescriptor, FURLabPayloadPin& OutPin)
{
	OutDescriptor = {};
	OutPin.Reset();
	FPayloadDeferredReclaims Deferred;
	FURLabShmExactRegionPtr ShmRegion;
	FURLabShmExactStagingPtr ShmStaging;
	bool bCommitShm = false;
	{
		FScopeLock Lock(&State->Mutex);
		double Now = 0.0;
		if (FURLabPayloadStoreStatus Status = State->ReadNow(Now); !Status) return Status;
		FURLabPayloadSweepResult Ignored;
		State->SweepExpiredLocked(Now, Ignored, Deferred);
		if (FURLabPayloadStoreStatus Status = State->ValidateOwner(Owner); !Status) return Status;
		FState::FStaging* Entry = State->Staging.Find(BlobId);
		if (!Entry) return State->Missing(Owner, BlobId);
		if (!(Entry->Owner == Owner))
			return Fail(EURLabPayloadStoreError::IdentityMismatch,
				TEXT("staging payload belongs to another owner"));

		if (Entry->Kind == FState::FStaging::EKind::ShmReady)
		{
			if (!Entry->ShmRegion || !Entry->ShmStaging)
				return Fail(EURLabPayloadStoreError::BackingFailure,
					TEXT("SHM staging catalog entry has no retained file backing"));
			Entry->Kind = FState::FStaging::EKind::ShmSealing;
			ShmRegion = Entry->ShmRegion;
			ShmStaging = Entry->ShmStaging;
			bCommitShm = true;
		}
		else if (Entry->Kind == FState::FStaging::EKind::ShmPreparing
			|| Entry->Kind == FState::FStaging::EKind::ShmSealing)
		{
			return Fail(EURLabPayloadStoreError::BackingFailure,
				TEXT("SHM payload staging is already being prepared or sealed"));
		}
		else if (Entry->Kind != FState::FStaging::EKind::Chunked)
		{
			return Fail(EURLabPayloadStoreError::InvalidDeclaration,
				TEXT("direct-producer admission must use SealProduced"));
		}
		else
		{
			if (Entry->CoveredBytes != Entry->Bytes.Num())
				return Fail(EURLabPayloadStoreError::Incomplete,
					TEXT("payload cannot commit until every declared byte is covered"));

			const FString ActualSha = FURLabAssetCache::Sha256Hex(Entry->Bytes);
			if (ActualSha != Entry->ExpectedSha256)
			{
				State->DeferStaging(*Entry, Deferred);
				State->Staging.Remove(BlobId);
				return Fail(EURLabPayloadStoreError::IntegrityMismatch,
					TEXT("payload SHA-256 does not match the declared digest"));
			}

			FURLabStoredPayloadDescriptor Descriptor;
			Descriptor.BlobId = BlobId;
			Descriptor.BootId = Owner.BootId;
			Descriptor.SessionId = Owner.SessionId;
			Descriptor.ByteLength = Entry->Bytes.Num();
			Descriptor.Sha256 = ActualSha;
			FURLabPayloadPin Payload = MakeShareable(new FURLabImmutablePayload(
				Owner, State->Identity, MoveTemp(Entry->Bytes), ActualSha,
				MoveTemp(Entry->Reservation)));
			State->DeferStaging(*Entry, Deferred);
			State->Staging.Remove(BlobId);
			FState::FSealed Sealed;
			Sealed.Owner = Owner;
			Sealed.Descriptor = Descriptor;
			Sealed.Payload = Payload;
			Sealed.ExpiresAt = Now + State->Limits.SealedTtlSeconds;
			State->Sealed.Add(BlobId, MoveTemp(Sealed));
			OutDescriptor = State->Describe(*State->Sealed.Find(BlobId), Now);
			OutPin = MoveTemp(Payload);
			return Ok();
		}
	}

	if (!bCommitShm)
		return Fail(EURLabPayloadStoreError::BackingFailure,
			TEXT("payload commit did not select a backing"));

	FURLabShmExactSealedPayloadPtr SealedCandidate;
	const FURLabShmExactStatus SealStatus = ShmRegion->SealStaging(
		*this, Owner, ShmStaging, SealedCandidate);
	{
		FScopeLock Lock(&State->Mutex);
		double Now = 0.0;
		if (FURLabPayloadStoreStatus Status = State->ReadNow(Now); !Status)
		{
			if (FState::FStaging* Entry = State->Staging.Find(BlobId);
				Entry && Entry->Owner == Owner
				&& Entry->Kind == FState::FStaging::EKind::ShmSealing)
			{
				State->DeferStaging(*Entry, Deferred);
				State->Staging.Remove(BlobId);
			}
			return Status;
		}
		FURLabPayloadSweepResult Ignored;
		State->SweepExpiredLocked(Now, Ignored, Deferred);
		FState::FStaging* Entry = State->Staging.Find(BlobId);
		if (!Entry) return State->Missing(Owner, BlobId);
		if (!(Entry->Owner == Owner) || Entry->Kind != FState::FStaging::EKind::ShmSealing
			|| Entry->ShmRegion != ShmRegion || Entry->ShmStaging != ShmStaging)
		{
			return Fail(EURLabPayloadStoreError::IdentityMismatch,
				TEXT("SHM payload staging changed while sealing"));
		}
		if (FURLabPayloadStoreStatus Status = State->ValidateOwner(Owner); !Status)
		{
			State->DeferStaging(*Entry, Deferred);
			State->Staging.Remove(BlobId);
			return Status;
		}
		if (!SealStatus)
		{
			if (SealStatus.Code == EURLabShmExactError::IntegrityMismatch)
			{
				State->DeferStaging(*Entry, Deferred);
				State->Staging.Remove(BlobId);
			}
			else
			{
				Entry->Kind = FState::FStaging::EKind::ShmReady;
			}
			return FromShm(SealStatus);
		}
		if (!SealedCandidate)
		{
			State->DeferStaging(*Entry, Deferred);
			State->Staging.Remove(BlobId);
			return Fail(EURLabPayloadStoreError::BackingFailure,
				TEXT("SHM seal succeeded without a retained sealed handle"));
		}

		const FURLabShmExactSealedDescriptor& Native = SealedCandidate->GetDescriptor();
		if (Native.RegionId != ShmRegion->GetDescriptor().RegionId
			|| Native.Generation != ShmRegion->GetDescriptor().Generation
			|| Native.ByteLength != Entry->DeclaredByteLength
			|| Native.Sha256 != Entry->ExpectedSha256)
		{
			State->DeferStaging(*Entry, Deferred);
			State->Staging.Remove(BlobId);
			return Fail(EURLabPayloadStoreError::IdentityMismatch,
				TEXT("SHM sealed backing disagrees with its exact catalog declaration"));
		}

		FURLabStoredPayloadDescriptor Descriptor;
		Descriptor.BackingKind = EURLabPayloadBackingKind::ShmSealedFile;
		Descriptor.BlobId = BlobId;
		Descriptor.BootId = Owner.BootId;
		Descriptor.SessionId = Owner.SessionId;
		Descriptor.ByteLength = Native.ByteLength;
		Descriptor.Sha256 = Native.Sha256;
		Descriptor.RegionId = Native.RegionId;
		Descriptor.RegionGeneration = Native.Generation;
		Descriptor.ObjectName = Native.ObjectName;
		FURLabPayloadPin Payload = MakeShareable(new FURLabImmutablePayload(
			Owner, State->Identity, Native.Sha256, Native.ByteLength,
			ShmRegion, SealedCandidate));
		State->DeferStaging(*Entry, Deferred);
		State->Staging.Remove(BlobId);
		FState::FSealed Sealed;
		Sealed.Owner = Owner;
		Sealed.Descriptor = Descriptor;
		Sealed.Payload = Payload;
		Sealed.ShmRegion = ShmRegion;
		Sealed.ShmSealed = SealedCandidate;
		Sealed.ExpiresAt = Now + State->Limits.SealedTtlSeconds;
		State->Sealed.Add(BlobId, MoveTemp(Sealed));
		OutDescriptor = State->Describe(*State->Sealed.Find(BlobId), Now);
		OutPin = MoveTemp(Payload);
	}
	return Ok();
}

FURLabPayloadStoreStatus FURLabPayloadStore::SealProduced(const FURLabPayloadOwner& Owner,
	const FURLabPayloadProducerPtr& Producer, TArray<uint8>&& Bytes,
	FURLabStoredPayloadDescriptor& OutDescriptor, FURLabPayloadPin& OutPin)
{
	OutDescriptor = {};
	OutPin.Reset();
	FPayloadDeferredReclaims Deferred;
	FURLabShmExactRegionPtr ShmRegion;
	bool bSealShm = false;
	{
		FScopeLock Lock(&State->Mutex);
		double Now = 0.0;
		if (FURLabPayloadStoreStatus Status = State->ReadNow(Now); !Status) return Status;
		FURLabPayloadSweepResult Ignored;
		State->SweepExpiredLocked(Now, Ignored, Deferred);
		if (FURLabPayloadStoreStatus Status = State->ValidateOwner(Owner); !Status) return Status;
		if (!Producer || Producer->BlobId.IsEmpty())
			return Fail(EURLabPayloadStoreError::InvalidIdentity,
				TEXT("producer handle must be minted by this payload store"));
		FState::FStaging* Entry = State->Staging.Find(Producer->BlobId);
		if (!Entry) return State->Missing(Owner, Producer->BlobId);
		const bool bNetworkEntry = Entry->Kind == FState::FStaging::EKind::Producer
			&& Producer->BackingKind == EURLabPayloadBackingKind::NetworkBytes;
		const bool bShmEntry = Entry->Kind == FState::FStaging::EKind::ShmProducerReady
			&& Producer->BackingKind == EURLabPayloadBackingKind::ShmSealedFile;
		if (!(Entry->Owner == Owner) || Entry->Producer != Producer
			|| (!bNetworkEntry && !bShmEntry))
		{
			return Fail(EURLabPayloadStoreError::IdentityMismatch,
				TEXT("producer admission belongs to another payload owner or generation"));
		}
		if (!Producer->Reservation || (bShmEntry
			&& (!Producer->ShmReservation || !Producer->ShmRegion)))
		{
			return Fail(EURLabPayloadStoreError::InvalidIdentity,
				TEXT("producer admission has already been consumed or lost its backing"));
		}
		if (Bytes.Num() != Entry->DeclaredByteLength
			|| Bytes.Num() != Producer->DeclaredByteLength)
		{
			return Fail(EURLabPayloadStoreError::InvalidDeclaration,
				TEXT("produced bytes do not match the admitted byte length"));
		}
		const int32 CanonicalCapacity = DefaultCalculateSlackReserve<int32>(
			Bytes.Num(), sizeof(uint8), true);
		if (Bytes.Max() > CanonicalCapacity)
		{
			return Fail(EURLabPayloadStoreError::InvalidDeclaration,
				TEXT("produced array retains capacity beyond canonical allocator rounding"));
		}

		if (bNetworkEntry)
		{
			const FString ActualSha = FURLabAssetCache::Sha256Hex(Bytes);
			FURLabStoredPayloadDescriptor Descriptor;
			Descriptor.BlobId = Producer->BlobId;
			Descriptor.BootId = Owner.BootId;
			Descriptor.SessionId = Owner.SessionId;
			Descriptor.ByteLength = Bytes.Num();
			Descriptor.Sha256 = ActualSha;
			FURLabPayloadPin Payload = MakeShareable(new FURLabImmutablePayload(
				Owner, State->Identity, MoveTemp(Bytes), ActualSha,
				MoveTemp(Producer->Reservation)));
			State->DeferStaging(*Entry, Deferred);
			State->Staging.Remove(Producer->BlobId);
			FState::FSealed Sealed;
			Sealed.Owner = Owner;
			Sealed.Descriptor = Descriptor;
			Sealed.Payload = Payload;
			Sealed.ExpiresAt = Now + State->Limits.SealedTtlSeconds;
			State->Sealed.Add(Descriptor.BlobId, MoveTemp(Sealed));
			OutDescriptor = State->Describe(*State->Sealed.Find(Descriptor.BlobId), Now);
			OutPin = MoveTemp(Payload);
			return Ok();
		}

		Entry->Kind = FState::FStaging::EKind::ShmProducerSealing;
		ShmRegion = Producer->ShmRegion;
		bSealShm = true;
	}

	if (!bSealShm)
		return Fail(EURLabPayloadStoreError::BackingFailure,
			TEXT("producer seal did not select a backing"));
	FURLabShmExactSealedPayloadPtr SealedCandidate;
	bool bBackingRetrySafe = true;
	const FURLabShmExactStatus SealStatus = ShmRegion->CreateFromBytesRetained(
		Owner, Bytes, Producer->ShmReservation, SealedCandidate, bBackingRetrySafe);
	bool bRestoreReadyAfterCandidateRetirement = false;
	FURLabPayloadStoreStatus FinalizeStatus = [&]() -> FURLabPayloadStoreStatus
	{
		FScopeLock Lock(&State->Mutex);
		double Now = 0.0;
		if (FURLabPayloadStoreStatus Status = State->ReadNow(Now); !Status)
		{
			bRestoreReadyAfterCandidateRetirement = true;
			return Status;
		}
		FURLabPayloadSweepResult Ignored;
		State->SweepExpiredLocked(Now, Ignored, Deferred);
		FState::FStaging* Entry = State->Staging.Find(Producer->BlobId);
		if (!Entry) return State->Missing(Owner, Producer->BlobId);
		if (!(Entry->Owner == Owner) || Entry->Producer != Producer
			|| Entry->Kind != FState::FStaging::EKind::ShmProducerSealing
			|| Producer->BackingKind != EURLabPayloadBackingKind::ShmSealedFile
			|| Producer->ShmRegion != ShmRegion)
		{
			return Fail(EURLabPayloadStoreError::IdentityMismatch,
				TEXT("SHM producer admission changed while publishing"));
		}
		if (FURLabPayloadStoreStatus Status = State->ValidateOwner(Owner); !Status)
		{
			State->DeferStaging(*Entry, Deferred);
			State->Staging.Remove(Producer->BlobId);
			return Status;
		}
		if (!SealStatus)
		{
			if (bBackingRetrySafe)
			{
				bRestoreReadyAfterCandidateRetirement = true;
			}
			else
			{
				// The failed file remains charged and tracked by its region. Retire
				// this admission because reusing its file reservation is unsafe.
				State->DeferStaging(*Entry, Deferred);
				State->Staging.Remove(Producer->BlobId);
			}
			return FromShm(SealStatus);
		}
		if (!SealedCandidate)
		{
			bRestoreReadyAfterCandidateRetirement = true;
			return Fail(EURLabPayloadStoreError::BackingFailure,
				TEXT("SHM producer publication returned no retained sealed handle"));
		}

		const FURLabShmExactSealedDescriptor& Native = SealedCandidate->GetDescriptor();
		if (Native.RegionId != ShmRegion->GetDescriptor().RegionId
			|| Native.Generation != ShmRegion->GetDescriptor().Generation
			|| Native.ByteLength != Producer->DeclaredByteLength
			|| Native.ByteLength != Entry->DeclaredByteLength
			|| !FURLabAssetCache::IsValidSha256Hex(Native.Sha256))
		{
			bRestoreReadyAfterCandidateRetirement = true;
			return Fail(EURLabPayloadStoreError::IdentityMismatch,
				TEXT("SHM produced backing disagrees with its exact admission"));
		}

		FURLabStoredPayloadDescriptor Descriptor;
		Descriptor.BackingKind = EURLabPayloadBackingKind::ShmSealedFile;
		Descriptor.BlobId = Producer->BlobId;
		Descriptor.BootId = Owner.BootId;
		Descriptor.SessionId = Owner.SessionId;
		Descriptor.ByteLength = Native.ByteLength;
		Descriptor.Sha256 = Native.Sha256;
		Descriptor.RegionId = Native.RegionId;
		Descriptor.RegionGeneration = Native.Generation;
		Descriptor.ObjectName = Native.ObjectName;
		FURLabPayloadPin Payload = MakeShareable(new FURLabImmutablePayload(
			Owner, State->Identity, Native.Sha256, Native.ByteLength,
			ShmRegion, SealedCandidate));
		FState::FSealed Sealed;
		Sealed.Owner = Owner;
		Sealed.Descriptor = Descriptor;
		Sealed.Payload = Payload;
		Sealed.ShmRegion = ShmRegion;
		Sealed.ShmSealed = SealedCandidate;
		Sealed.ExpiresAt = Now + State->Limits.SealedTtlSeconds;
		State->Sealed.Add(Descriptor.BlobId, MoveTemp(Sealed));
		// Publication is now non-failing. Retire the caller's CPU allocation and
		// producer admissions after the new catalog entry is complete; deferred
		// owners free memory outside the catalog mutex.
		Deferred.NetworkBytes.Add(MoveTemp(Bytes));
		Deferred.Reservations.Add(MoveTemp(Producer->Reservation));
		Deferred.Reservations.Add(MoveTemp(Producer->ShmReservation));
		Producer->ShmRegion.Reset();
		State->DeferStaging(*Entry, Deferred);
		State->Staging.Remove(Producer->BlobId);
		OutDescriptor = State->Describe(*State->Sealed.Find(Descriptor.BlobId), Now);
		OutPin = MoveTemp(Payload);
		return Ok();
	}();

	if (bRestoreReadyAfterCandidateRetirement)
	{
		// Keep this admission non-retryable until every candidate file and file
		// descriptor from the failed attempt has been retired. The producer's
		// retained region is immutable until a successful catalog publication.
		bool bCandidateRetired = bBackingRetrySafe;
		if (SealedCandidate)
		{
			bCandidateRetired = ShmRegion->Release(Owner, SealedCandidate).IsOk();
		}
		SealedCandidate.Reset();
		FScopeLock Lock(&State->Mutex);
		if (FState::FStaging* Entry = State->Staging.Find(Producer->BlobId);
			Entry && Entry->Owner == Owner && Entry->Producer == Producer
			&& Entry->Kind == FState::FStaging::EKind::ShmProducerSealing
			&& Producer->BackingKind == EURLabPayloadBackingKind::ShmSealedFile
			&& Producer->ShmRegion == ShmRegion)
		{
			if (bCandidateRetired)
			{
				Entry->Kind = FState::FStaging::EKind::ShmProducerReady;
			}
			else
			{
				State->DeferStaging(*Entry, Deferred);
				State->Staging.Remove(Producer->BlobId);
				FinalizeStatus = Fail(EURLabPayloadStoreError::BackingFailure,
					TEXT("failed SHM candidate could not be retired; producer admission was closed"));
			}
		}
	}
	return FinalizeStatus;
}

FURLabPayloadStoreStatus FURLabPayloadStore::Pin(const FURLabPayloadOwner& Owner,
	const FString& BlobId, FURLabPayloadPin& OutPin)
{
	OutPin.Reset();
	FPayloadDeferredReclaims Deferred;
	FScopeLock Lock(&State->Mutex);
	double Now = 0.0;
	if (FURLabPayloadStoreStatus Status = State->ReadNow(Now); !Status) return Status;
	FURLabPayloadSweepResult Ignored;
	State->SweepExpiredLocked(Now, Ignored, Deferred);
	if (FURLabPayloadStoreStatus Status = State->ValidateOwner(Owner); !Status) return Status;
	const FState::FSealed* Entry = State->Sealed.Find(BlobId);
	if (!Entry)
		return State->Missing(Owner, BlobId);
	if (!(Entry->Owner == Owner))
		return Fail(EURLabPayloadStoreError::IdentityMismatch,
			TEXT("sealed payload belongs to another owner"));
	OutPin = Entry->Payload;
	return Ok();
}

FURLabPayloadStoreStatus FURLabPayloadStore::GetDescriptor(const FURLabPayloadOwner& Owner,
	const FString& BlobId, FURLabStoredPayloadDescriptor& OutDescriptor)
{
	OutDescriptor = {};
	FPayloadDeferredReclaims Deferred;
	FScopeLock Lock(&State->Mutex);
	double Now = 0.0;
	if (FURLabPayloadStoreStatus Status = State->ReadNow(Now); !Status) return Status;
	FURLabPayloadSweepResult Ignored;
	State->SweepExpiredLocked(Now, Ignored, Deferred);
	if (FURLabPayloadStoreStatus Status = State->ValidateOwner(Owner); !Status) return Status;
	const FState::FSealed* Entry = State->Sealed.Find(BlobId);
	if (!Entry) return State->Missing(Owner, BlobId);
	if (!(Entry->Owner == Owner))
		return Fail(EURLabPayloadStoreError::IdentityMismatch,
			TEXT("sealed payload belongs to another owner"));
	OutDescriptor = State->Describe(*Entry, Now);
	return Ok();
}

FURLabPayloadStoreStatus FURLabPayloadStore::CopyRange(const FURLabPayloadOwner& Owner,
	const FString& BlobId, int64 Offset, int64 ByteLength, FURLabPayloadCopyPin& OutCopy)
{
	OutCopy.Reset();
	FPayloadDeferredReclaims Deferred;
	FURLabPayloadPin Payload;
	{
		FScopeLock Lock(&State->Mutex);
		double Now = 0.0;
		if (FURLabPayloadStoreStatus Status = State->ReadNow(Now); !Status) return Status;
		FURLabPayloadSweepResult Ignored;
		State->SweepExpiredLocked(Now, Ignored, Deferred);
		if (FURLabPayloadStoreStatus Status = State->ValidateOwner(Owner); !Status) return Status;
		const FState::FSealed* Entry = State->Sealed.Find(BlobId);
		if (!Entry) return State->Missing(Owner, BlobId);
		if (!(Entry->Owner == Owner))
			return Fail(EURLabPayloadStoreError::IdentityMismatch,
				TEXT("sealed payload belongs to another owner"));
		Payload = Entry->Payload;
	}
	return CopyPinnedRange(Payload, Offset, ByteLength, OutCopy);
}

FURLabPayloadStoreStatus FURLabPayloadStore::CopyPinnedRange(const FURLabPayloadPin& Pin,
	int64 Offset, int64 ByteLength, FURLabPayloadCopyPin& OutCopy)
{
	OutCopy.Reset();
	if (!Pin || Pin->StoreIdentity != State->Identity)
		return Fail(EURLabPayloadStoreError::InvalidIdentity,
			TEXT("payload pin was not minted by this store"));
	if (ByteLength > State->Limits.MaxChunkBytes)
		return Fail(EURLabPayloadStoreError::ChunkTooLarge,
			TEXT("payload read exceeds the declared chunk limit"));
	if (Offset < 0 || ByteLength <= 0 || Offset > Pin->ByteLength
		|| ByteLength > Pin->ByteLength - Offset)
	{
		return Fail(EURLabPayloadStoreError::OutOfBounds,
			TEXT("payload read range is outside the sealed blob"));
	}
	if (!State->Accounting->Reserve(Pin->Owner.SessionId, ByteLength))
		return Fail(EURLabPayloadStoreError::LimitExceeded,
			TEXT("payload copy physical byte or object-count budget is exhausted"));
	FURLabPayloadPhysicalReservationPtr Reservation =
		MakeShared<FURLabPayloadPhysicalReservation, ESPMode::ThreadSafe>(
			State->Accounting, Pin->Owner.SessionId, ByteLength);

	if (Pin->BackingKind == EURLabPayloadBackingKind::NetworkBytes)
	{
		TConstArrayView<uint8> Source;
		if (!Pin->TryGetNetworkBytes(Source) || Source.Num() != Pin->ByteLength)
			return Fail(EURLabPayloadStoreError::BackingFailure,
				TEXT("network payload pin has inconsistent immutable bytes"));
		TArray<uint8> Bytes;
		Bytes.Reserve(static_cast<int32>(ByteLength));
		Bytes.Append(Source.GetData() + Offset, static_cast<int32>(ByteLength));
		OutCopy = MakeShareable(new FURLabPayloadCopy(
			MoveTemp(Bytes), MoveTemp(Reservation)));
		return Ok();
	}
	if (Pin->BackingKind != EURLabPayloadBackingKind::ShmSealedFile
		|| !Pin->ShmRegion || !Pin->ShmSealed)
	{
		return Fail(EURLabPayloadStoreError::BackingFailure,
			TEXT("SHM payload pin has no retained sealed backing"));
	}
	FURLabShmExactCopiedBytesPtr ShmCopy;
	const FURLabShmExactStatus Status = Pin->ShmRegion->ReadRetainedCopied(
		Pin->ShmSealed, Offset, ByteLength, MoveTemp(Reservation), ShmCopy);
	if (!Status) return FromShm(Status);
	OutCopy = MakeShareable(new FURLabPayloadCopy(MoveTemp(ShmCopy)));
	return Ok();
}

FURLabPayloadStoreStatus FURLabPayloadStore::ReservePhysicalCapacity(
	const FURLabPayloadOwner& Owner, int64 ByteLength,
	FURLabPayloadPhysicalReservationPtr& OutReservation)
{
	OutReservation.Reset();
	FPayloadDeferredReclaims Deferred;
	FScopeLock Lock(&State->Mutex);
	double Now = 0.0;
	if (FURLabPayloadStoreStatus Status = State->ReadNow(Now); !Status) return Status;
	FURLabPayloadSweepResult Ignored;
	State->SweepExpiredLocked(Now, Ignored, Deferred);
	if (FURLabPayloadStoreStatus Status = State->ValidateOwner(Owner); !Status) return Status;
	if (ByteLength <= 0 || ByteLength > State->Limits.MaxBlobBytes)
		return Fail(EURLabPayloadStoreError::InvalidDeclaration,
			TEXT("physical reservation is outside the declared blob limit"));
	if (!State->Accounting->Reserve(Owner.SessionId, ByteLength))
		return Fail(EURLabPayloadStoreError::LimitExceeded,
			TEXT("payload physical byte or object-count budget is exhausted"));
	OutReservation = MakeShared<FURLabPayloadPhysicalReservation, ESPMode::ThreadSafe>(
		State->Accounting, Owner.SessionId, ByteLength);
	return Ok();
}

FURLabPayloadStoreStatus FURLabPayloadStore::Acknowledge(const FURLabPayloadOwner& Owner,
	const FString& BlobId, const FString& Sha256, bool& OutNewlyAcknowledged)
{
	OutNewlyAcknowledged = false;
	FPayloadDeferredReclaims Deferred;
	FScopeLock Lock(&State->Mutex);
	double Now = 0.0;
	if (FURLabPayloadStoreStatus Status = State->ReadNow(Now); !Status) return Status;
	FURLabPayloadSweepResult Ignored;
	State->SweepExpiredLocked(Now, Ignored, Deferred);
	if (FURLabPayloadStoreStatus Status = State->ValidateOwner(Owner); !Status) return Status;
	FState::FSealed* Entry = State->Sealed.Find(BlobId);
	if (!Entry) return State->Missing(Owner, BlobId);
	if (!(Entry->Owner == Owner))
		return Fail(EURLabPayloadStoreError::IdentityMismatch,
			TEXT("sealed payload belongs to another owner"));
	const FString CanonicalSha = Sha256.ToLower();
	if (!FURLabAssetCache::IsValidSha256Hex(CanonicalSha)
		|| CanonicalSha != Entry->Descriptor.Sha256)
	{
		return Fail(EURLabPayloadStoreError::IntegrityMismatch,
			TEXT("acknowledgement SHA-256 does not match the sealed payload"));
	}
	OutNewlyAcknowledged = !Entry->bAcknowledged;
	Entry->bAcknowledged = true;
	return Ok();
}

FURLabPayloadStoreStatus FURLabPayloadStore::Release(const FURLabPayloadOwner& Owner,
	const FString& BlobId)
{
	FPayloadDeferredReclaims Deferred;
	FScopeLock Lock(&State->Mutex);
	double Now = 0.0;
	if (FURLabPayloadStoreStatus Status = State->ReadNow(Now); !Status) return Status;
	FURLabPayloadSweepResult Ignored;
	State->SweepExpiredLocked(Now, Ignored, Deferred);
	if (FURLabPayloadStoreStatus Status = State->ValidateOwner(Owner); !Status) return Status;
	if (FState::FStaging* StagingEntry = State->Staging.Find(BlobId))
	{
		if (!(StagingEntry->Owner == Owner))
			return Fail(EURLabPayloadStoreError::IdentityMismatch,
				TEXT("staging payload belongs to another owner"));
		State->DeferStaging(*StagingEntry, Deferred);
		State->Staging.Remove(BlobId);
	}
	else if (FState::FSealed* SealedEntry = State->Sealed.Find(BlobId))
	{
		if (!(SealedEntry->Owner == Owner))
			return Fail(EURLabPayloadStoreError::IdentityMismatch,
				TEXT("sealed payload belongs to another owner"));
		State->DeferSealed(*SealedEntry, Deferred);
		State->Sealed.Remove(BlobId);
	}
	else
	{
		return State->Missing(Owner, BlobId);
	}
	return Ok();
}

FURLabPayloadStoreStatus FURLabPayloadStore::RevokeSession(const FURLabPayloadOwner& Owner,
	int32& OutRemovedEntries)
{
	OutRemovedEntries = 0;
	FPayloadDeferredReclaims Deferred;
	FScopeLock Lock(&State->Mutex);
	// Session closure is an identity operation, not a timed payload operation.
	// It must remain possible after admission has atomically deactivated the
	// scope and even when the injected expiry clock is no longer readable.
	if (Owner.BootId != State->BootId || Owner.SessionId.IsEmpty() || !Owner.Session
		|| Owner.Session->GetBootId() != Owner.BootId
		|| Owner.Session->GetSessionId() != Owner.SessionId)
	{
		return Fail(EURLabPayloadStoreError::InvalidIdentity,
			TEXT("payload revoke requires this boot's exact non-empty session handle"));
	}
	const FURLabPayloadSessionPtr* Exact = State->Sessions.Find(Owner.SessionId);
	if (!Exact || *Exact != Owner.Session)
	{
		return Fail(EURLabPayloadStoreError::SessionRevoked,
			TEXT("payload session has already been revoked or replaced"));
	}
	Owner.Session->Deactivate();
	State->Sessions.Remove(Owner.SessionId);
	for (auto It = State->Staging.CreateIterator(); It; ++It)
	{
		if (It.Value().Owner.SessionId != Owner.SessionId) continue;
		State->DeferStaging(It.Value(), Deferred);
		It.RemoveCurrent();
		++OutRemovedEntries;
	}
	for (auto It = State->Sealed.CreateIterator(); It; ++It)
	{
		if (It.Value().Owner.SessionId != Owner.SessionId) continue;
		State->DeferSealed(It.Value(), Deferred);
		It.RemoveCurrent();
		++OutRemovedEntries;
	}
	for (int32 Index = State->ExpiredOrder.Num() - 1; Index >= 0; --Index)
	{
		const FState::FExpired* Tombstone = State->Expired.Find(State->ExpiredOrder[Index]);
		if (!Tombstone || Tombstone->Session.Pin() != Owner.Session) continue;
		State->Expired.Remove(State->ExpiredOrder[Index]);
		State->ExpiredOrder.RemoveAt(Index, 1, EAllowShrinking::No);
	}
	return Ok();
}

FURLabPayloadStoreStatus FURLabPayloadStore::SweepExpired(FURLabPayloadSweepResult& OutResult)
{
	OutResult = {};
	FPayloadDeferredReclaims Deferred;
	FScopeLock Lock(&State->Mutex);
	double Now = 0.0;
	if (FURLabPayloadStoreStatus Status = State->ReadNow(Now); !Status) return Status;
	if (!State->LimitsError.IsEmpty())
		return Fail(EURLabPayloadStoreError::InvalidLimits, State->LimitsError);
	State->SweepExpiredLocked(Now, OutResult, Deferred);
	return Ok();
}

FURLabPayloadStoreStats FURLabPayloadStore::GetStats() const
{
	FURLabPayloadStoreStats Out;
	const FPhysicalUsage Usage = State->Accounting->Get();
	Out.PhysicalBytes = Usage.Bytes;
	Out.PhysicalBlobCount = Usage.Count;
	FScopeLock Lock(&State->Mutex);
	Out.StagingCount = State->Staging.Num();
	Out.SealedCount = State->Sealed.Num();
	for (const TPair<FString, FState::FSealed>& Entry : State->Sealed)
		if (Entry.Value.bAcknowledged) ++Out.AcknowledgedCount;
	Out.ExpiredTombstoneCount = State->Expired.Num();
	Out.ActiveSessionCount = State->Sessions.Num();
	return Out;
}

FURLabPayloadStoreStats FURLabPayloadStore::GetSessionStats(const FString& SessionId) const
{
	FURLabPayloadStoreStats Out;
	const FPhysicalUsage Usage = State->Accounting->Get(&SessionId);
	Out.PhysicalBytes = Usage.Bytes;
	Out.PhysicalBlobCount = Usage.Count;
	FScopeLock Lock(&State->Mutex);
	for (const TPair<FString, FState::FStaging>& Entry : State->Staging)
		if (Entry.Value.Owner.SessionId == SessionId) ++Out.StagingCount;
	for (const TPair<FString, FState::FSealed>& Entry : State->Sealed)
	{
		if (Entry.Value.Owner.SessionId != SessionId) continue;
		++Out.SealedCount;
		if (Entry.Value.bAcknowledged) ++Out.AcknowledgedCount;
	}
	for (const TPair<FString, FState::FExpired>& Entry : State->Expired)
		if (Entry.Value.SessionId == SessionId) ++Out.ExpiredTombstoneCount;
	Out.ActiveSessionCount = State->Sessions.Contains(SessionId) ? 1 : 0;
	return Out;
}

const FURLabPayloadStoreLimits& FURLabPayloadStore::GetLimits() const
{
	return State->Limits;
}
