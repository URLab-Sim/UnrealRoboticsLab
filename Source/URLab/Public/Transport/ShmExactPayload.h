// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#pragma once

#include "CoreMinimal.h"

#include "Bridge/URLabPayloadStore.h"

#include <atomic>

enum class EURLabShmExactError : uint8
{
	None,
	Unsupported,
	InvalidArgument,
	InvalidRegion,
	QuotaExceeded,
	NotFound,
	IdentityMismatch,
	Bounds,
	IntegrityMismatch,
	ChallengeInvalid,
	IoFailure
};

struct URLAB_API FURLabShmExactStatus
{
	EURLabShmExactError Code = EURLabShmExactError::None;
	FString Message;

	bool IsOk() const { return Code == EURLabShmExactError::None; }
	explicit operator bool() const { return IsOk(); }
};

struct URLAB_API FURLabShmExactRegionDescriptor
{
	FString BootId;
	FString RegionId;
	uint64 Generation = 0;
	FString DirectoryPath;
};

struct URLAB_API FURLabShmExactStagingGrant
{
	FString RegionId;
	uint64 Generation = 0;
	FString ObjectName;
	int64 ByteLength = 0;
	FString ExpectedSha256;
};

struct URLAB_API FURLabShmExactSealedDescriptor
{
	FString RegionId;
	uint64 Generation = 0;
	FString ObjectName;
	int64 ByteLength = 0;
	FString Sha256;
};

struct URLAB_API FURLabShmExactChallengeOffer
{
	FString RegionId;
	uint64 RegionGeneration = 0;
	FString ChallengeName;
	int64 ByteLength = 0;
};

struct URLAB_API FURLabShmExactChallengeProof
{
	FString RegionId;
	uint64 RegionGeneration = 0;
	FString ChallengeName;
	TArray<uint8> Nonce;
};

struct URLAB_API FURLabShmLiveObjectDescriptor
{
	FString RegionId;
	uint64 RegionGeneration = 0;
	FString ObjectName;
	int64 ByteLength = 0;
};

class FURLabShmExactRegion;
using FURLabShmExactRegionPtr = TSharedPtr<FURLabShmExactRegion, ESPMode::ThreadSafe>;
class FURLabShmRegionAccess;
using FURLabShmRegionAccessPtr = TSharedPtr<FURLabShmRegionAccess, ESPMode::ThreadSafe>;

class FURLabLockedLatestFrameWriter;
class FURLabLockedLatestFrameReader;

/** Opaque mutable region storage. It is not an immutable exact payload. */
class URLAB_API FURLabShmLiveObject
{
public:
	~FURLabShmLiveObject();
	const FURLabShmLiveObjectDescriptor& GetDescriptor() const { return Descriptor; }

private:
	friend class FURLabShmExactRegion;
	friend class FURLabShmRegionAccess;
	friend class FURLabLockedLatestFrameWriter;
	friend class FURLabLockedLatestFrameReader;
	FURLabShmLiveObject(FURLabShmExactRegionPtr InRegion,
		FURLabShmLiveObjectDescriptor InDescriptor, UPTRINT InNativeHandle, bool bInCreator);
	FURLabShmLiveObject(FURLabShmRegionAccessPtr InAccess,
		FURLabShmLiveObjectDescriptor InDescriptor, UPTRINT InNativeHandle);

	FURLabShmExactRegionPtr Region;
	FURLabShmRegionAccessPtr Access;
	FURLabShmLiveObjectDescriptor Descriptor;
	UPTRINT NativeHandle = ~UPTRINT(0);
	bool bCreator = false;
};

using FURLabShmLiveObjectPtr = TSharedPtr<FURLabShmLiveObject, ESPMode::ThreadSafe>;

/** Consumer-only validated access to a region proven during data negotiation. */
class URLAB_API FURLabShmRegionAccess
	: public TSharedFromThis<FURLabShmRegionAccess, ESPMode::ThreadSafe>
{
public:
	~FURLabShmRegionAccess();
	FURLabShmRegionAccess(const FURLabShmRegionAccess&) = delete;
	FURLabShmRegionAccess& operator=(const FURLabShmRegionAccess&) = delete;

	static FURLabShmExactStatus Open(const FString& DirectoryPath,
		const FString& ExpectedBootId, const FString& ExpectedRegionId,
		uint64 ExpectedGeneration, int64 MaxBlobBytes,
		FURLabShmRegionAccessPtr& OutAccess);
	FURLabShmExactStatus ReadChallenge(const FString& ChallengeName,
		int64 ByteLength, TArray<uint8>& OutNonce) const;
	FURLabShmExactStatus OpenLiveObject(const FURLabShmLiveObjectDescriptor& Descriptor,
		FURLabShmLiveObjectPtr& OutObject);

	const FString& GetRegionId() const { return RegionId; }
	uint64 GetRegionGeneration() const { return RegionGeneration; }

private:
	FURLabShmRegionAccess(FString InRegionId, uint64 InRegionGeneration,
		int64 InMaxBlobBytes, int InDirectoryFd, uint64 InDirectoryDevice,
		uint64 InDirectoryInode);

	FString RegionId;
	uint64 RegionGeneration = 0;
	int64 MaxBlobBytes = 0;
	int DirectoryFd = -1;
	uint64 DirectoryDevice = 0;
	uint64 DirectoryInode = 0;
	mutable FCriticalSection Mutex;
};

class URLAB_API FURLabShmExactStaging
{
public:
	~FURLabShmExactStaging();

	const FURLabShmExactStagingGrant& GetGrant() const { return Grant; }

private:
	friend class FURLabShmExactRegion;
	FURLabShmExactStaging(FURLabShmExactRegionPtr InRegion,
		FURLabPayloadOwner InOwner, FURLabShmExactStagingGrant InGrant, int InFileDescriptor,
		FURLabPayloadPhysicalReservationPtr InReservation);

	FURLabShmExactRegionPtr Region;
	FURLabPayloadOwner Owner;
	FURLabShmExactStagingGrant Grant;
	FURLabPayloadPhysicalReservationPtr Reservation;
	int FileDescriptor = -1;
	bool bRetired = false;
};

using FURLabShmExactStagingPtr = TSharedPtr<FURLabShmExactStaging, ESPMode::ThreadSafe>;

/** Retained read-only file identity. Its reservations survive unlink and store teardown. */
class URLAB_API FURLabShmExactSealedPayload
{
public:
	~FURLabShmExactSealedPayload();

	const FURLabShmExactSealedDescriptor& GetDescriptor() const { return Descriptor; }

private:
	friend class FURLabShmExactRegion;
	FURLabShmExactSealedPayload(FURLabShmExactRegionPtr InRegion,
		FURLabPayloadOwner InOwner, FURLabShmExactSealedDescriptor InDescriptor,
		int InFileDescriptor, FURLabPayloadPhysicalReservationPtr InReservation);

	FURLabShmExactRegionPtr Region;
	FURLabPayloadOwner Owner;
	FURLabShmExactSealedDescriptor Descriptor;
	FURLabPayloadPhysicalReservationPtr Reservation;
	int FileDescriptor = -1;
	bool bPublished = true;
};

using FURLabShmExactSealedPayloadPtr =
	TSharedPtr<FURLabShmExactSealedPayload, ESPMode::ThreadSafe>;

/** Immutable copied read whose store reservation owns its retained byte accounting. */
class URLAB_API FURLabShmExactCopiedBytes
{
public:
	~FURLabShmExactCopiedBytes();

	const TArray<uint8>& GetBytes() const { return Bytes; }
	int64 Num() const { return Bytes.Num(); }

private:
	friend class FURLabShmExactRegion;
	FURLabShmExactCopiedBytes(TArray<uint8>&& InBytes,
		FURLabPayloadPhysicalReservationPtr InReservation);

	FURLabPayloadPhysicalReservationPtr Reservation;
	TArray<uint8> Bytes;
};

using FURLabShmExactCopiedBytesPtr =
	TSharedPtr<const FURLabShmExactCopiedBytes, ESPMode::ThreadSafe>;

/** Opaque owner-bound bootstrap proof. The nonce is never exposed through this handle. */
class URLAB_API FURLabShmExactChallenge
{
public:
	~FURLabShmExactChallenge();

	const FURLabShmExactChallengeOffer& GetOffer() const { return Offer; }

private:
	friend class FURLabShmExactRegion;
	FURLabShmExactChallenge(FURLabShmExactRegionPtr InRegion,
		FURLabPayloadOwner InOwner, FURLabShmExactChallengeOffer InOffer,
		TArray<uint8>&& InNonce, int InFileDescriptor,
		FURLabPayloadPhysicalReservationPtr InReservation);

	FURLabShmExactRegionPtr Region;
	const FURLabPayloadOwner Owner;
	const FURLabShmExactChallengeOffer Offer;
	const TArray<uint8> Nonce;
	FURLabPayloadPhysicalReservationPtr Reservation;
	int FileDescriptor = -1;
	std::atomic<bool> bRetired{false};
};

using FURLabShmExactChallengePtr =
	TSharedPtr<FURLabShmExactChallenge, ESPMode::ThreadSafe>;

/**
 * Linux exact-file SHM primitive. Region and object names are creator-minted,
 * relative, and opened through retained directory descriptors with no symlink
 * following. Payload-byte accounting excludes filesystem/allocator metadata;
 * every staging file, sealed file, and retained copied read owns a separate
 * store physical reservation for its complete logical length.
 */
class URLAB_API FURLabShmExactRegion
	: public TSharedFromThis<FURLabShmExactRegion, ESPMode::ThreadSafe>
{
public:
	~FURLabShmExactRegion();

	FURLabShmExactRegion(const FURLabShmExactRegion&) = delete;
	FURLabShmExactRegion& operator=(const FURLabShmExactRegion&) = delete;

	static FURLabShmExactStatus Create(const FString& BootId, const FString& BaseDirectory,
		FURLabShmExactRegionPtr& OutRegion);
	/** Creates a fresh private region beneath the Linux user-scoped /dev/shm base. */
	static FURLabShmExactStatus CreateForBoot(const FString& BootId,
		FURLabShmExactRegionPtr& OutRegion);

	const FURLabShmExactRegionDescriptor& GetDescriptor() const;
	FURLabShmExactStatus CreateLiveObject(int64 ByteLength,
		FURLabShmLiveObjectPtr& OutObject);
	FURLabShmExactStatus OpenLiveObject(const FURLabShmLiveObjectDescriptor& Descriptor,
		FURLabShmLiveObjectPtr& OutObject);

	FURLabShmExactStatus CreateStaging(FURLabPayloadStore& Store,
		const FURLabPayloadOwner& Owner, int64 ByteLength, const FString& ExpectedSha256,
		FURLabShmExactStagingPtr& OutStaging);
	FURLabShmExactStatus SealStaging(FURLabPayloadStore& Store,
		const FURLabPayloadOwner& Owner, const FURLabShmExactStagingPtr& Staging,
		FURLabShmExactSealedPayloadPtr& OutSealed);
	/** Caller retains responsibility for accounting the input memory during this separately reserved copy. */
	FURLabShmExactStatus CreateFromBytes(FURLabPayloadStore& Store,
		const FURLabPayloadOwner& Owner, TConstArrayView<uint8> Bytes,
		FURLabShmExactSealedPayloadPtr& OutSealed);

	FURLabShmExactStatus CreateChallenge(FURLabPayloadStore& Store,
		const FURLabPayloadOwner& Owner, FURLabShmExactChallengePtr& OutChallenge);
	/** Pure in-memory proof validation; the caller separately checks session expiry. */
	FURLabShmExactStatus VerifyChallenge(const FURLabPayloadOwner& Owner,
		const FURLabShmExactChallengePtr& Challenge,
		const FURLabShmExactChallengeProof& Proof) const;
	FURLabShmExactStatus RetireChallenge(const FURLabPayloadOwner& Owner,
		const FURLabShmExactChallengePtr& Challenge);

	FURLabShmExactStatus ReadCopied(FURLabPayloadStore& Store,
		const FURLabPayloadOwner& Owner, const FURLabShmExactSealedDescriptor& Descriptor,
		int64 Offset, int64 ByteLength, FURLabShmExactCopiedBytesPtr& OutCopy);
	FURLabShmExactStatus ReadCopied(FURLabPayloadStore& Store,
		const FURLabPayloadOwner& Owner, const FURLabShmExactSealedPayloadPtr& Retained,
		int64 Offset, int64 ByteLength, FURLabShmExactCopiedBytesPtr& OutCopy);

	/** Remove future path lookup for the exact owner. Retained fd and reservations remain valid. */
	FURLabShmExactStatus Release(const FURLabPayloadOwner& Owner,
		const FURLabShmExactSealedPayloadPtr& Sealed);

private:
	friend class FURLabPayloadStore;
	friend class FURLabShmExactStaging;
	friend class FURLabShmExactSealedPayload;
	friend class FURLabShmExactChallenge;
	friend class FURLabShmLiveObject;
	FURLabShmExactStatus CreateFromBytesRetained(
		const FURLabPayloadOwner& Owner, TConstArrayView<uint8> Bytes,
		const FURLabPayloadPhysicalReservationPtr& Reservation,
		FURLabShmExactSealedPayloadPtr& OutSealed, bool& bOutRetrySafe);
	struct FState;
	explicit FURLabShmExactRegion(TUniquePtr<FState>&& InState);

	void RetireStaging(FURLabShmExactStaging& Staging);
	void RetireSealed(FURLabShmExactSealedPayload& Sealed);
	FURLabShmExactStatus RetireChallengeHandle(FURLabShmExactChallenge& Challenge);
	void RetireLiveObject(FURLabShmLiveObject& Object);
	FURLabShmExactStatus ReadRetainedCopied(
		const FURLabShmExactSealedPayloadPtr& Retained,
		int64 Offset, int64 ByteLength,
		FURLabPayloadPhysicalReservationPtr&& Reservation,
		FURLabShmExactCopiedBytesPtr& OutCopy);

	TUniquePtr<FState> State;
};
