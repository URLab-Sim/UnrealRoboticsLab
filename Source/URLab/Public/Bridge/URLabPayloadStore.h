// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#pragma once

#include "CoreMinimal.h"

#include <atomic>

/**
 * Conservative physical limits for exact payloads. Byte budgets count retained
 * network arrays, SHM files, and copied reads; allocator/filesystem metadata is
 * excluded. Every retained physical owner also consumes one object slot.
 */
struct URLAB_API FURLabPayloadStoreLimits
{
	int64 MaxBlobBytes = 64ll * 1024 * 1024;
	// Leaves room for a descriptor/envelope under the existing 256 KiB command reply cap.
	int32 MaxChunkBytes = 128 * 1024;
	int32 MaxChunksPerBlob = 4096;
	int64 MaxSessionBytes = 128ll * 1024 * 1024;
	int64 MaxGlobalBytes = 512ll * 1024 * 1024;
	int32 MaxBlobCount = 256;
	int32 MaxSessionBlobCount = 64;
	int32 MaxSessionPhysicalObjectCount = 64;
	int32 MaxActiveSessions = 16;
	double StagingTtlSeconds = 300.0;
	double SealedTtlSeconds = 300.0;
	double ExpiredTombstoneTtlSeconds = 300.0;
};

class FURLabPayloadStore;
class FURLabPayloadStoreIdentity;
class FURLabShmExactRegion;
class FURLabShmExactStaging;
class FURLabShmExactSealedPayload;
class FURLabShmExactCopiedBytes;

/** Store-minted session authority. Construction and active state are private to its store. */
class URLAB_API FURLabPayloadSession
{
public:
	const FString& GetBootId() const { return BootId; }
	const FString& GetSessionId() const { return SessionId; }
	bool IsActive() const { return bActive.load(std::memory_order_acquire); }
	/** One-way close gate used by dispatcher admission before store reclamation. */
	void Deactivate() { bActive.store(false, std::memory_order_release); }

private:
	friend class FURLabPayloadStore;
	explicit FURLabPayloadSession(FString InBootId, FString InSessionId)
		: BootId(MoveTemp(InBootId)), SessionId(MoveTemp(InSessionId))
	{
	}

	FString BootId;
	FString SessionId;
	std::atomic<bool> bActive{true};
};

using FURLabPayloadSessionPtr = TSharedPtr<FURLabPayloadSession, ESPMode::ThreadSafe>;

/** Authority for one payload operation. Both fields are exact, case-sensitive identities. */
struct URLAB_API FURLabPayloadOwner
{
	FString BootId;
	FString SessionId;
	FURLabPayloadSessionPtr Session;

	static FURLabPayloadOwner FromSession(const FURLabPayloadSessionPtr& InSession);

	bool operator==(const FURLabPayloadOwner& Other) const
	{
		return BootId == Other.BootId && SessionId == Other.SessionId
			&& Session == Other.Session;
	}
};

enum class EURLabPayloadStoreError : uint8
{
	None,
	InvalidLimits,
	InvalidClock,
	InvalidIdentity,
	SessionRevoked,
	InvalidDeclaration,
	LimitExceeded,
	NotFound,
	Expired,
	IdentityMismatch,
	InvalidChunk,
	ChunkTooLarge,
	ChunkLimitExceeded,
	OutOfBounds,
	Overlap,
	Incomplete,
	IntegrityMismatch,
	BackingFailure
};

struct URLAB_API FURLabPayloadStoreStatus
{
	EURLabPayloadStoreError Code = EURLabPayloadStoreError::None;
	FString Message;

	bool IsOk() const { return Code == EURLabPayloadStoreError::None; }
	explicit operator bool() const { return IsOk(); }
};

enum class EURLabPayloadBackingKind : uint8
{
	NetworkBytes,
	ShmSealedFile
};

struct URLAB_API FURLabStoredPayloadDescriptor
{
	EURLabPayloadBackingKind BackingKind = EURLabPayloadBackingKind::NetworkBytes;
	FString BlobId;
	FString BootId;
	FString SessionId;
	int64 ByteLength = 0;
	FString Sha256;
	double ExpiresInSeconds = 0.0;
	FString RegionId;
	uint64 RegionGeneration = 0;
	FString ObjectName;
};

/** Writable SHM staging object tied to one shared catalog blob identity. */
struct URLAB_API FURLabPayloadShmStagingGrant
{
	FString BlobId;
	FString RegionId;
	uint64 RegionGeneration = 0;
	FString ObjectName;
	int64 ByteLength = 0;
	FString Sha256;
	double ExpiresInSeconds = 0.0;
};

class FURLabPayloadPhysicalReservation;
using FURLabPayloadPhysicalReservationPtr =
	TSharedPtr<FURLabPayloadPhysicalReservation, ESPMode::ThreadSafe>;

/** Admission held by a direct producer before it allocates the declared bytes. */
class URLAB_API FURLabPayloadProducer
{
public:
	~FURLabPayloadProducer();

	const FString& GetBlobId() const { return BlobId; }
	int64 GetDeclaredByteLength() const { return DeclaredByteLength; }
	EURLabPayloadBackingKind GetBackingKind() const { return BackingKind; }

private:
	friend class FURLabPayloadStore;
	FURLabPayloadProducer(FString InBlobId, int64 InDeclaredByteLength,
		FURLabPayloadPhysicalReservationPtr InReservation);
	FURLabPayloadProducer(FString InBlobId, int64 InDeclaredByteLength,
		TSharedPtr<FURLabShmExactRegion, ESPMode::ThreadSafe> InRegion,
		FURLabPayloadPhysicalReservationPtr InCpuReservation,
		FURLabPayloadPhysicalReservationPtr InShmReservation);

	EURLabPayloadBackingKind BackingKind = EURLabPayloadBackingKind::NetworkBytes;
	FString BlobId;
	int64 DeclaredByteLength = 0;
	FURLabPayloadPhysicalReservationPtr Reservation;
	FURLabPayloadPhysicalReservationPtr ShmReservation;
	// Immutable for an admitted producer; the store clears it under its catalog
	// mutex only after the replacement sealed entry has been published.
	TSharedPtr<FURLabShmExactRegion, ESPMode::ThreadSafe> ShmRegion;
};

using FURLabPayloadProducerPtr = TSharedPtr<FURLabPayloadProducer, ESPMode::ThreadSafe>;

/** Immutable network bytes or retained sealed-file backing. The pin itself is read authority. */
class URLAB_API FURLabImmutablePayload
{
public:
	~FURLabImmutablePayload();

	EURLabPayloadBackingKind GetBackingKind() const { return BackingKind; }
	int64 Num() const { return ByteLength; }
	const FString& GetSha256() const { return Sha256; }
	/** Network-only zero-copy view. SHM callers use FURLabPayloadStore::CopyPinnedRange. */
	bool TryGetNetworkBytes(TConstArrayView<uint8>& OutBytes) const;

private:
	friend class FURLabPayloadStore;
	FURLabImmutablePayload(const FURLabPayloadOwner& InOwner,
		TSharedRef<FURLabPayloadStoreIdentity, ESPMode::ThreadSafe> InStoreIdentity,
		TArray<uint8>&& InBytes, FString InSha256,
		FURLabPayloadPhysicalReservationPtr InReservation);
	FURLabImmutablePayload(const FURLabPayloadOwner& InOwner,
		TSharedRef<FURLabPayloadStoreIdentity, ESPMode::ThreadSafe> InStoreIdentity,
		FString InSha256,
		int64 InByteLength,
		TSharedPtr<FURLabShmExactRegion, ESPMode::ThreadSafe> InRegion,
		TSharedPtr<FURLabShmExactSealedPayload, ESPMode::ThreadSafe> InSealed);

	EURLabPayloadBackingKind BackingKind = EURLabPayloadBackingKind::NetworkBytes;
	FURLabPayloadOwner Owner;
	TSharedRef<FURLabPayloadStoreIdentity, ESPMode::ThreadSafe> StoreIdentity;
	int64 ByteLength = 0;
	FURLabPayloadPhysicalReservationPtr Reservation;
	TArray<uint8> Bytes;
	FString Sha256;
	TSharedPtr<FURLabShmExactRegion, ESPMode::ThreadSafe> ShmRegion;
	TSharedPtr<FURLabShmExactSealedPayload, ESPMode::ThreadSafe> ShmSealed;
};

using FURLabPayloadPin = TSharedPtr<const FURLabImmutablePayload, ESPMode::ThreadSafe>;

/** A bounded copied range whose physical quota follows the returned bytes. */
class URLAB_API FURLabPayloadCopy
{
public:
	~FURLabPayloadCopy();

	const TArray<uint8>& GetBytes() const;
	int64 Num() const;

private:
	friend class FURLabPayloadStore;
	FURLabPayloadCopy(TArray<uint8>&& InBytes,
		FURLabPayloadPhysicalReservationPtr InReservation);
	explicit FURLabPayloadCopy(
		TSharedPtr<const FURLabShmExactCopiedBytes, ESPMode::ThreadSafe> InShmCopy);

	FURLabPayloadPhysicalReservationPtr Reservation;
	TArray<uint8> Bytes;
	TSharedPtr<const FURLabShmExactCopiedBytes, ESPMode::ThreadSafe> ShmCopy;
};

using FURLabPayloadCopyPin = TSharedPtr<const FURLabPayloadCopy, ESPMode::ThreadSafe>;

struct URLAB_API FURLabPayloadSweepResult
{
	int32 StagingRemoved = 0;
	int32 SealedRemoved = 0;
	int32 TombstonesForgotten = 0;
};

struct URLAB_API FURLabPayloadStoreStats
{
	int64 PhysicalBytes = 0;
	int32 PhysicalBlobCount = 0;
	int32 StagingCount = 0;
	int32 SealedCount = 0;
	int32 AcknowledgedCount = 0;
	int32 ExpiredTombstoneCount = 0;
	int32 ActiveSessionCount = 0;
};

/**
 * Bounded, boot-scoped exact payload storage.
 *
 * Writes remain private until complete coverage and SHA-256 verification. Released
 * entries cannot be looked up again, while pins already handed to accepted work
 * keep immutable network bytes or a retained sealed-file fd and its physical
 * budget reservation alive.
 */
class URLAB_API FURLabPayloadStore
{
public:
	/** The injected clock is sampled under the store mutex and must not re-enter this store. */
	using FMonotonicClock = TFunction<double()>;

	explicit FURLabPayloadStore(FString InBootId,
		FURLabPayloadStoreLimits InLimits = FURLabPayloadStoreLimits(),
		FMonotonicClock InClock = FMonotonicClock());
	~FURLabPayloadStore();

	FURLabPayloadStore(const FURLabPayloadStore&) = delete;
	FURLabPayloadStore& operator=(const FURLabPayloadStore&) = delete;

	FURLabPayloadStoreStatus OpenSession(const FString& BootId, const FString& SessionId,
		FURLabPayloadSessionPtr& OutSession);
	FURLabPayloadStoreStatus BeginWrite(const FURLabPayloadOwner& Owner, int64 ByteLength,
		const FString& ExpectedSha256, FString& OutBlobId);
	FURLabPayloadStoreStatus BeginShmWrite(const FURLabPayloadOwner& Owner,
		const TSharedPtr<FURLabShmExactRegion, ESPMode::ThreadSafe>& Region,
		int64 ByteLength, const FString& ExpectedSha256,
		FURLabPayloadShmStagingGrant& OutGrant);
	FURLabPayloadStoreStatus BeginProduce(const FURLabPayloadOwner& Owner, int64 ByteLength,
		FURLabPayloadProducerPtr& OutProducer);
	/** Reserve the producer's CPU bytes and future sealed SHM file before allocation. */
	FURLabPayloadStoreStatus BeginShmProduce(const FURLabPayloadOwner& Owner,
		const TSharedPtr<FURLabShmExactRegion, ESPMode::ThreadSafe>& Region,
		int64 ByteLength, FURLabPayloadProducerPtr& OutProducer);
	FURLabPayloadStoreStatus WriteChunk(const FURLabPayloadOwner& Owner, const FString& BlobId,
		int64 Offset, TConstArrayView<uint8> Bytes);
	FURLabPayloadStoreStatus Commit(const FURLabPayloadOwner& Owner, const FString& BlobId,
		FURLabStoredPayloadDescriptor& OutDescriptor, FURLabPayloadPin& OutPin);
	FURLabPayloadStoreStatus SealProduced(const FURLabPayloadOwner& Owner,
		const FURLabPayloadProducerPtr& Producer, TArray<uint8>&& Bytes,
		FURLabStoredPayloadDescriptor& OutDescriptor, FURLabPayloadPin& OutPin);

	FURLabPayloadStoreStatus Pin(const FURLabPayloadOwner& Owner, const FString& BlobId,
		FURLabPayloadPin& OutPin);
	FURLabPayloadStoreStatus GetDescriptor(const FURLabPayloadOwner& Owner, const FString& BlobId,
		FURLabStoredPayloadDescriptor& OutDescriptor);
	FURLabPayloadStoreStatus CopyRange(const FURLabPayloadOwner& Owner, const FString& BlobId,
		int64 Offset, int64 ByteLength, FURLabPayloadCopyPin& OutCopy);
	/** Copy from an already accepted immutable pin, independent of later session deactivation. */
	FURLabPayloadStoreStatus CopyPinnedRange(const FURLabPayloadPin& Pin,
		int64 Offset, int64 ByteLength, FURLabPayloadCopyPin& OutCopy);

	/**
	 * Reserve one physical byte owner before allocating it. The caller must keep
	 * the handle until that allocation is moved into store ownership or freed.
	 * Direct producers must shrink to canonical allocator capacity before
	 * SealProduced; errors leave the caller's array and reservation intact.
	 */
	FURLabPayloadStoreStatus ReservePhysicalCapacity(const FURLabPayloadOwner& Owner,
		int64 ByteLength, FURLabPayloadPhysicalReservationPtr& OutReservation);
	FURLabPayloadStoreStatus Acknowledge(const FURLabPayloadOwner& Owner,
		const FString& BlobId, const FString& Sha256, bool& OutNewlyAcknowledged);

	FURLabPayloadStoreStatus Release(const FURLabPayloadOwner& Owner, const FString& BlobId);
	FURLabPayloadStoreStatus RevokeSession(const FURLabPayloadOwner& Owner,
		int32& OutRemovedEntries);
	FURLabPayloadStoreStatus SweepExpired(FURLabPayloadSweepResult& OutResult);

	FURLabPayloadStoreStats GetStats() const;
	FURLabPayloadStoreStats GetSessionStats(const FString& SessionId) const;
	const FURLabPayloadStoreLimits& GetLimits() const;

private:
	struct FState;
	TUniquePtr<FState> State;
};
