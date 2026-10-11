// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "Transport/ShmExactPayload.h"

#include "HAL/FileManager.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"
#include "Misc/ScopeLock.h"
#include "Utils/AssetCache.h"

#if PLATFORM_LINUX
#include <cerrno>
#include <climits>
#include <cstdlib>
#include <fcntl.h>
#include <linux/magic.h>
#include <sys/mman.h>
#include <sys/random.h>
#include <sys/sendfile.h>
#include <sys/stat.h>
#include <sys/vfs.h>
#include <sys/types.h>
#include <unistd.h>
#endif

namespace
{
FURLabShmExactStatus ShmExactOk()
{
	return {};
}

FURLabShmExactStatus ShmExactFail(EURLabShmExactError Code, FString Message)
{
	FURLabShmExactStatus Out;
	Out.Code = Code;
	Out.Message = MoveTemp(Message);
	return Out;
}

FURLabShmExactStatus ShmExactUnsupported()
{
	return ShmExactFail(EURLabShmExactError::Unsupported,
		TEXT("exact payload SHM is currently supported only on Linux"));
}

FURLabShmExactStatus ShmExactFromStore(const FURLabPayloadStoreStatus& Status)
{
	if (Status.IsOk()) return ShmExactOk();
	EURLabShmExactError Code = EURLabShmExactError::IdentityMismatch;
	if (Status.Code == EURLabPayloadStoreError::LimitExceeded)
		Code = EURLabShmExactError::QuotaExceeded;
	else if (Status.Code == EURLabPayloadStoreError::InvalidDeclaration)
		Code = EURLabShmExactError::InvalidArgument;
	return ShmExactFail(Code, Status.Message);
}

FString ShmExactRandomToken()
{
	return FGuid::NewGuid().ToString(EGuidFormats::Digits).ToLower();
}

bool ShmExactIsObjectName(const FString& Name, const TCHAR* Prefix)
{
	if (!Name.StartsWith(Prefix) || !Name.EndsWith(TEXT(".bin")) || Name.Len() != 40)
		return false;
	for (int32 Index = 4; Index < 36; ++Index)
	{
		const TCHAR Char = Name[Index];
		if (!((Char >= TEXT('0') && Char <= TEXT('9'))
			|| (Char >= TEXT('a') && Char <= TEXT('f')))) return false;
	}
	return true;
}

#if PLATFORM_LINUX
bool ShmExactWriteAllAt(int FileDescriptor, const uint8* Data, int64 ByteLength)
{
	int64 Written = 0;
	while (Written < ByteLength)
	{
		const ssize_t Result = ::pwrite(FileDescriptor, Data + Written,
			static_cast<size_t>(ByteLength - Written), static_cast<off_t>(Written));
		if (Result < 0 && errno == EINTR) continue;
		if (Result <= 0) return false;
		Written += Result;
	}
	return true;
}

bool ShmExactGetRandom(uint8* Data, int64 ByteLength)
{
	int64 Filled = 0;
	while (Filled < ByteLength)
	{
		const ssize_t Result = ::getrandom(Data + Filled,
			static_cast<size_t>(ByteLength - Filled), 0);
		if (Result < 0 && errno == EINTR) continue;
		if (Result <= 0) return false;
		Filled += Result;
	}
	return true;
}

bool ShmExactReadAllAt(int FileDescriptor, uint8* Data, int64 Offset, int64 ByteLength)
{
	int64 Read = 0;
	while (Read < ByteLength)
	{
		const ssize_t Result = ::pread(FileDescriptor, Data + Read,
			static_cast<size_t>(ByteLength - Read), static_cast<off_t>(Offset + Read));
		if (Result < 0 && errno == EINTR) continue;
		if (Result <= 0) return false;
		Read += Result;
	}
	return true;
}

bool ShmExactCopyFile(int Source, int Destination, int64 ByteLength)
{
	off_t Offset = 0;
	int64 Copied = 0;
	while (Copied < ByteLength)
	{
		const ssize_t Result = ::sendfile(Destination, Source, &Offset,
			static_cast<size_t>(ByteLength - Copied));
		if (Result < 0 && errno == EINTR) continue;
		if (Result <= 0) return false;
		Copied += Result;
	}
	return true;
}

bool ShmExactHashFile(int FileDescriptor, int64 ByteLength, FString& OutSha256)
{
	OutSha256.Reset();
	if (ByteLength <= 0 || ByteLength > MAX_int32) return false;
	void* Mapping = ::mmap(nullptr, static_cast<size_t>(ByteLength), PROT_READ,
		MAP_PRIVATE, FileDescriptor, 0);
	if (Mapping == MAP_FAILED) return false;
	OutSha256 = FURLabAssetCache::Sha256Hex(static_cast<const uint8*>(Mapping),
		static_cast<int32>(ByteLength));
	::munmap(Mapping, static_cast<size_t>(ByteLength));
	return true;
}

bool ShmExactValidFileStat(const struct stat& Stat, int64 ExpectedLength,
	bool bRequireReadOnly)
{
	return S_ISREG(Stat.st_mode) && Stat.st_uid == ::geteuid()
		&& Stat.st_size == ExpectedLength && ExpectedLength > 0 && ExpectedLength <= MAX_int32
		&& (!bRequireReadOnly || (Stat.st_mode & (S_IWUSR | S_IWGRP | S_IWOTH)) == 0);
}
#endif
}

struct FURLabShmExactRegion::FState
{
	enum class EObjectKind : uint8 { Staging, Sealed, Challenge, Live };

	struct FObjectRecord
	{
		EObjectKind Kind = EObjectKind::Staging;
		FURLabPayloadOwner Owner;
		// The region retains the physical charge until the exact path is
		// successfully unlinked. Handles hold the same shared reservation.
		FURLabPayloadPhysicalReservationPtr Reservation;
		int64 ByteLength = 0;
		FString Sha256;
		uint64 Device = 0;
		uint64 Inode = 0;
	};

	FURLabShmExactRegionDescriptor Descriptor;
	FCriticalSection Mutex;
	TMap<FString, FObjectRecord> Objects;
#if PLATFORM_LINUX
	int BaseDirectoryFd = -1;
	int DirectoryFd = -1;
	FString RegionDirectoryName;
	uint64 DirectoryDevice = 0;
	uint64 DirectoryInode = 0;
	uint64 MarkerDevice = 0;
	uint64 MarkerInode = 0;

	bool ValidatePathIdentity(const FString& ObjectName, const FObjectRecord& Record,
		bool bRequireReadOnly, struct stat* OutStat = nullptr) const
	{
		FTCHARToUTF8 NameUtf8(*ObjectName);
		struct stat PathStat = {};
		if (::fstatat(DirectoryFd, NameUtf8.Get(), &PathStat, AT_SYMLINK_NOFOLLOW) != 0
			|| !ShmExactValidFileStat(PathStat, Record.ByteLength, bRequireReadOnly)
			|| PathStat.st_nlink != 1
			|| static_cast<uint64>(PathStat.st_dev) != Record.Device
			|| static_cast<uint64>(PathStat.st_ino) != Record.Inode)
		{
			return false;
		}
		if (OutStat) *OutStat = PathStat;
		return true;
	}

	int CreateObject(const TCHAR* Prefix, int64 ByteLength, FString& OutName,
		FObjectRecord& OutRecord)
	{
		OutName.Reset();
		for (int32 Attempt = 0; Attempt < 32; ++Attempt)
		{
			const FString Candidate = FString(Prefix) + ShmExactRandomToken() + TEXT(".bin");
			FTCHARToUTF8 NameUtf8(*Candidate);
			const int Fd = ::openat(DirectoryFd, NameUtf8.Get(),
				O_RDWR | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, S_IRUSR | S_IWUSR);
			if (Fd < 0)
			{
				if (errno == EEXIST) continue;
				return -1;
			}
			if (::ftruncate(Fd, static_cast<off_t>(ByteLength)) != 0)
			{
				::close(Fd);
				::unlinkat(DirectoryFd, NameUtf8.Get(), 0);
				return -1;
			}
			struct stat Stat = {};
			if (::fstat(Fd, &Stat) != 0
				|| !ShmExactValidFileStat(Stat, ByteLength, false) || Stat.st_nlink != 1)
			{
				::close(Fd);
				::unlinkat(DirectoryFd, NameUtf8.Get(), 0);
				return -1;
			}
			OutName = Candidate;
			OutRecord.ByteLength = ByteLength;
			OutRecord.Device = static_cast<uint64>(Stat.st_dev);
			OutRecord.Inode = static_cast<uint64>(Stat.st_ino);
			return Fd;
		}
		return -1;
	}

	bool UnlinkIfExact(const FString& ObjectName, const FObjectRecord& Record)
	{
		if (!ValidatePathIdentity(ObjectName, Record, false)) return false;
		FTCHARToUTF8 NameUtf8(*ObjectName);
		return ::unlinkat(DirectoryFd, NameUtf8.Get(), 0) == 0;
	}
#endif
};

FURLabShmLiveObject::FURLabShmLiveObject(FURLabShmExactRegionPtr InRegion,
	FURLabShmLiveObjectDescriptor InDescriptor, UPTRINT InNativeHandle, bool bInCreator)
	: Region(MoveTemp(InRegion)), Descriptor(MoveTemp(InDescriptor)),
	  NativeHandle(InNativeHandle), bCreator(bInCreator)
{
}

FURLabShmLiveObject::FURLabShmLiveObject(FURLabShmRegionAccessPtr InAccess,
	FURLabShmLiveObjectDescriptor InDescriptor, UPTRINT InNativeHandle)
	: Access(MoveTemp(InAccess)), Descriptor(MoveTemp(InDescriptor)),
	  NativeHandle(InNativeHandle)
{
}

FURLabShmLiveObject::~FURLabShmLiveObject()
{
	if (Region) Region->RetireLiveObject(*this);
#if PLATFORM_LINUX
	if (NativeHandle != ~UPTRINT(0)) ::close(static_cast<int>(NativeHandle));
#endif
	NativeHandle = ~UPTRINT(0);
	Region.Reset();
	Access.Reset();
}

FURLabShmRegionAccess::FURLabShmRegionAccess(FString InRegionId,
	uint64 InRegionGeneration, int64 InMaxBlobBytes, int InDirectoryFd,
	uint64 InDirectoryDevice, uint64 InDirectoryInode)
	: RegionId(MoveTemp(InRegionId)), RegionGeneration(InRegionGeneration),
	  MaxBlobBytes(InMaxBlobBytes), DirectoryFd(InDirectoryFd),
	  DirectoryDevice(InDirectoryDevice), DirectoryInode(InDirectoryInode)
{
}

FURLabShmRegionAccess::~FURLabShmRegionAccess()
{
#if PLATFORM_LINUX
	FScopeLock Lock(&Mutex);
	if (DirectoryFd >= 0) ::close(DirectoryFd);
#endif
	DirectoryFd = -1;
}

FURLabShmExactStatus FURLabShmRegionAccess::Open(const FString& DirectoryPath,
	const FString& ExpectedBootId, const FString& ExpectedRegionId,
	uint64 ExpectedGeneration, int64 MaxBlobBytes,
	FURLabShmRegionAccessPtr& OutAccess)
{
	OutAccess.Reset();
#if !PLATFORM_LINUX
	return ShmExactUnsupported();
#else
	FTCHARToUTF8 BootUtf8(*ExpectedBootId);
	FTCHARToUTF8 RegionUtf8(*ExpectedRegionId);
	if (DirectoryPath.IsEmpty() || ExpectedBootId.IsEmpty() || ExpectedRegionId.IsEmpty()
		|| BootUtf8.Length() > 256 || RegionUtf8.Length() > 256
		|| ExpectedGeneration == 0 || ExpectedGeneration > static_cast<uint64>(MAX_int64)
		|| MaxBlobBytes <= 0 || MaxBlobBytes > MAX_int32)
		return ShmExactFail(EURLabShmExactError::InvalidArgument,
			TEXT("consumer SHM region identity or limits are invalid"));
	FTCHARToUTF8 PathUtf8(*DirectoryPath);
	const int Fd = ::open(PathUtf8.Get(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
	if (Fd < 0)
		return ShmExactFail(EURLabShmExactError::InvalidRegion,
			TEXT("consumer SHM region could not be opened safely"));
	struct stat DirectoryStat = {};
	if (::fstat(Fd, &DirectoryStat) != 0 || !S_ISDIR(DirectoryStat.st_mode)
		|| DirectoryStat.st_uid != ::geteuid() || DirectoryStat.st_nlink < 1
		|| (DirectoryStat.st_mode & 077) != 0)
	{
		::close(Fd);
		return ShmExactFail(EURLabShmExactError::InvalidRegion,
			TEXT("consumer SHM region is not private and creator-owned"));
	}
	const FString ExpectedMarker = ExpectedBootId + TEXT("\n") + ExpectedRegionId
		+ TEXT("\n") + LexToString(ExpectedGeneration) + TEXT("\n");
	FTCHARToUTF8 MarkerName(TEXT(".urlab-owner"));
	const int MarkerFd = ::openat(Fd, MarkerName.Get(), O_RDONLY | O_NOFOLLOW | O_CLOEXEC);
	FTCHARToUTF8 ExpectedUtf8(*ExpectedMarker);
	TArray<uint8> Marker;
	Marker.SetNumUninitialized(ExpectedUtf8.Length());
	struct stat MarkerStat = {};
	const bool bMarkerValid = MarkerFd >= 0 && ::fstat(MarkerFd, &MarkerStat) == 0
		&& S_ISREG(MarkerStat.st_mode) && MarkerStat.st_uid == ::geteuid()
		&& MarkerStat.st_nlink == 1 && MarkerStat.st_size == ExpectedUtf8.Length()
		&& ShmExactReadAllAt(MarkerFd, Marker.GetData(), 0, Marker.Num())
		&& FMemory::Memcmp(Marker.GetData(), ExpectedUtf8.Get(), Marker.Num()) == 0;
	if (MarkerFd >= 0) ::close(MarkerFd);
	if (!bMarkerValid)
	{
		::close(Fd);
		return ShmExactFail(EURLabShmExactError::IdentityMismatch,
			TEXT("consumer SHM region marker identity does not match"));
	}
	OutAccess = MakeShareable(new FURLabShmRegionAccess(ExpectedRegionId,
		ExpectedGeneration, MaxBlobBytes, Fd, DirectoryStat.st_dev, DirectoryStat.st_ino));
	return ShmExactOk();
#endif
}

FURLabShmExactStatus FURLabShmRegionAccess::ReadChallenge(
	const FString& ChallengeName, int64 ByteLength, TArray<uint8>& OutNonce) const
{
	OutNonce.Reset();
#if !PLATFORM_LINUX
	return ShmExactUnsupported();
#else
	if (!ShmExactIsObjectName(ChallengeName, TEXT("chl-"))
		|| ByteLength <= 0 || ByteLength > 4096)
		return ShmExactFail(EURLabShmExactError::InvalidArgument,
			TEXT("consumer SHM challenge descriptor is invalid"));
	FScopeLock Lock(&Mutex);
	if (DirectoryFd < 0)
		return ShmExactFail(EURLabShmExactError::InvalidRegion,
			TEXT("consumer SHM region is closed"));
	FTCHARToUTF8 NameUtf8(*ChallengeName);
	const int Fd = ::openat(DirectoryFd, NameUtf8.Get(), O_RDONLY | O_NOFOLLOW | O_CLOEXEC);
	struct stat Stat = {};
	struct stat PathStat = {};
	const bool bValid = Fd >= 0 && ::fstat(Fd, &Stat) == 0
		&& ::fstatat(DirectoryFd, NameUtf8.Get(), &PathStat, AT_SYMLINK_NOFOLLOW) == 0
		&& ShmExactValidFileStat(Stat, ByteLength, true) && Stat.st_nlink == 1
		&& Stat.st_dev == PathStat.st_dev && Stat.st_ino == PathStat.st_ino;
	if (!bValid)
	{
		if (Fd >= 0) ::close(Fd);
		return ShmExactFail(EURLabShmExactError::IdentityMismatch,
			TEXT("consumer SHM challenge identity does not match"));
	}
	TArray<uint8> Local;
	Local.SetNumUninitialized(ByteLength);
	const bool bRead = ShmExactReadAllAt(Fd, Local.GetData(), 0, ByteLength);
	::close(Fd);
	if (!bRead)
		return ShmExactFail(EURLabShmExactError::IoFailure,
			TEXT("consumer SHM challenge read was incomplete"));
	OutNonce = MoveTemp(Local);
	return ShmExactOk();
#endif
}

FURLabShmExactStatus FURLabShmRegionAccess::OpenLiveObject(
	const FURLabShmLiveObjectDescriptor& Descriptor,
	FURLabShmLiveObjectPtr& OutObject)
{
	OutObject.Reset();
#if !PLATFORM_LINUX
	return ShmExactUnsupported();
#else
	if (Descriptor.RegionId != RegionId || Descriptor.RegionGeneration != RegionGeneration)
		return ShmExactFail(EURLabShmExactError::IdentityMismatch,
			TEXT("consumer live object region identity is stale"));
	if (!ShmExactIsObjectName(Descriptor.ObjectName, TEXT("liv-"))
		|| Descriptor.ByteLength <= 48 || Descriptor.ByteLength > MaxBlobBytes + 48)
		return ShmExactFail(EURLabShmExactError::Bounds,
			TEXT("consumer live object descriptor exceeds negotiated bounds"));
	FScopeLock Lock(&Mutex);
	if (DirectoryFd < 0)
		return ShmExactFail(EURLabShmExactError::InvalidRegion,
			TEXT("consumer SHM region is closed"));
	FTCHARToUTF8 NameUtf8(*Descriptor.ObjectName);
	const int Fd = ::openat(DirectoryFd, NameUtf8.Get(), O_RDONLY | O_NOFOLLOW | O_CLOEXEC);
	struct stat OpenStat = {};
	struct stat PathStat = {};
	const bool bValid = Fd >= 0 && ::fstat(Fd, &OpenStat) == 0
		&& ::fstatat(DirectoryFd, NameUtf8.Get(), &PathStat, AT_SYMLINK_NOFOLLOW) == 0
		&& ShmExactValidFileStat(OpenStat, Descriptor.ByteLength, false)
		&& S_ISREG(PathStat.st_mode) && PathStat.st_uid == ::geteuid()
		&& PathStat.st_nlink == 1 && PathStat.st_size == Descriptor.ByteLength
		&& OpenStat.st_dev == PathStat.st_dev && OpenStat.st_ino == PathStat.st_ino;
	if (!bValid)
	{
		if (Fd >= 0) ::close(Fd);
		return ShmExactFail(EURLabShmExactError::IdentityMismatch,
			TEXT("consumer live object path identity does not match"));
	}
	OutObject = MakeShareable(new FURLabShmLiveObject(AsShared(), Descriptor, Fd));
	return ShmExactOk();
#endif
}

FURLabShmExactStaging::FURLabShmExactStaging(FURLabShmExactRegionPtr InRegion,
	FURLabPayloadOwner InOwner, FURLabShmExactStagingGrant InGrant, int InFileDescriptor,
	FURLabPayloadPhysicalReservationPtr InReservation)
	: Region(MoveTemp(InRegion)), Owner(MoveTemp(InOwner)), Grant(MoveTemp(InGrant)),
	  Reservation(MoveTemp(InReservation)), FileDescriptor(InFileDescriptor)
{
}

FURLabShmExactStaging::~FURLabShmExactStaging()
{
	if (Region) Region->RetireStaging(*this);
	Reservation.Reset();
	Region.Reset();
}

FURLabShmExactSealedPayload::FURLabShmExactSealedPayload(
	FURLabShmExactRegionPtr InRegion, FURLabPayloadOwner InOwner,
	FURLabShmExactSealedDescriptor InDescriptor, int InFileDescriptor,
	FURLabPayloadPhysicalReservationPtr InReservation)
	: Region(MoveTemp(InRegion)), Owner(MoveTemp(InOwner)), Descriptor(MoveTemp(InDescriptor)),
	  Reservation(MoveTemp(InReservation)),
	  FileDescriptor(InFileDescriptor)
{
}

FURLabShmExactSealedPayload::~FURLabShmExactSealedPayload()
{
	if (Region) Region->RetireSealed(*this);
#if PLATFORM_LINUX
	if (FileDescriptor >= 0) ::close(FileDescriptor);
#endif
	FileDescriptor = -1;
	Reservation.Reset();
	Region.Reset();
}

FURLabShmExactCopiedBytes::FURLabShmExactCopiedBytes(TArray<uint8>&& InBytes,
	FURLabPayloadPhysicalReservationPtr InReservation)
	: Reservation(MoveTemp(InReservation)), Bytes(MoveTemp(InBytes))
{
}

FURLabShmExactCopiedBytes::~FURLabShmExactCopiedBytes() = default;

FURLabShmExactChallenge::FURLabShmExactChallenge(
	FURLabShmExactRegionPtr InRegion, FURLabPayloadOwner InOwner,
	FURLabShmExactChallengeOffer InOffer, TArray<uint8>&& InNonce,
	int InFileDescriptor, FURLabPayloadPhysicalReservationPtr InReservation)
	: Region(MoveTemp(InRegion)), Owner(MoveTemp(InOwner)), Offer(MoveTemp(InOffer)),
	  Nonce(MoveTemp(InNonce)), Reservation(MoveTemp(InReservation)),
	  FileDescriptor(InFileDescriptor)
{
}

FURLabShmExactChallenge::~FURLabShmExactChallenge()
{
	if (Region) Region->RetireChallengeHandle(*this);
#if PLATFORM_LINUX
	if (FileDescriptor >= 0) ::close(FileDescriptor);
#endif
	FileDescriptor = -1;
	Reservation.Reset();
	Region.Reset();
}

FURLabShmExactRegion::FURLabShmExactRegion(TUniquePtr<FState>&& InState)
	: State(MoveTemp(InState))
{
}

FURLabShmExactRegion::~FURLabShmExactRegion()
{
#if PLATFORM_LINUX
	FScopeLock Lock(&State->Mutex);
	for (const TPair<FString, FState::FObjectRecord>& Entry : State->Objects)
		State->UnlinkIfExact(Entry.Key, Entry.Value);
	State->Objects.Reset();
	if (State->DirectoryFd >= 0)
	{
		struct stat MarkerStat = {};
		if (::fstatat(State->DirectoryFd, ".urlab-owner", &MarkerStat,
				AT_SYMLINK_NOFOLLOW) == 0
			&& S_ISREG(MarkerStat.st_mode)
			&& static_cast<uint64>(MarkerStat.st_dev) == State->MarkerDevice
			&& static_cast<uint64>(MarkerStat.st_ino) == State->MarkerInode)
		{
			::unlinkat(State->DirectoryFd, ".urlab-owner", 0);
		}
		::close(State->DirectoryFd);
		State->DirectoryFd = -1;
	}
	if (State->BaseDirectoryFd >= 0)
	{
		FTCHARToUTF8 NameUtf8(*State->RegionDirectoryName);
		struct stat PathStat = {};
		if (::fstatat(State->BaseDirectoryFd, NameUtf8.Get(), &PathStat,
				AT_SYMLINK_NOFOLLOW) == 0
			&& S_ISDIR(PathStat.st_mode)
			&& static_cast<uint64>(PathStat.st_dev) == State->DirectoryDevice
			&& static_cast<uint64>(PathStat.st_ino) == State->DirectoryInode)
		{
			::unlinkat(State->BaseDirectoryFd, NameUtf8.Get(), AT_REMOVEDIR);
		}
		::close(State->BaseDirectoryFd);
		State->BaseDirectoryFd = -1;
	}
#endif
}

FURLabShmExactStatus FURLabShmExactRegion::Create(const FString& BootId,
	const FString& BaseDirectory, FURLabShmExactRegionPtr& OutRegion)
{
	OutRegion.Reset();
#if !PLATFORM_LINUX
	return ShmExactUnsupported();
#else
	if (BootId.IsEmpty() || BaseDirectory.IsEmpty())
		return ShmExactFail(EURLabShmExactError::InvalidArgument,
			TEXT("exact SHM region requires non-empty boot and base identities"));
	if (!IFileManager::Get().MakeDirectory(*BaseDirectory, true))
		return ShmExactFail(EURLabShmExactError::IoFailure,
			TEXT("exact SHM base directory could not be created"));
	FTCHARToUTF8 BaseUtf8(*BaseDirectory);
	char CanonicalBase[PATH_MAX] = {};
	if (!::realpath(BaseUtf8.Get(), CanonicalBase))
		return ShmExactFail(EURLabShmExactError::InvalidRegion,
			TEXT("exact SHM base directory could not be canonicalized"));
	const int BaseFd = ::open(CanonicalBase, O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
	if (BaseFd < 0)
		return ShmExactFail(EURLabShmExactError::InvalidRegion,
			TEXT("exact SHM base directory is not a no-follow directory"));

	FString RegionName;
	int DirectoryFd = -1;
	for (int32 Attempt = 0; Attempt < 32 && DirectoryFd < 0; ++Attempt)
	{
		RegionName = TEXT("urlab-") + ShmExactRandomToken();
		FTCHARToUTF8 NameUtf8(*RegionName);
		if (::mkdirat(BaseFd, NameUtf8.Get(), S_IRWXU) != 0)
		{
			if (errno == EEXIST) continue;
			::close(BaseFd);
			return ShmExactFail(EURLabShmExactError::IoFailure,
				TEXT("exact SHM private region directory creation failed"));
		}
		DirectoryFd = ::openat(BaseFd, NameUtf8.Get(),
			O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
		if (DirectoryFd < 0) ::unlinkat(BaseFd, NameUtf8.Get(), AT_REMOVEDIR);
	}
	if (DirectoryFd < 0)
	{
		::close(BaseFd);
		return ShmExactFail(EURLabShmExactError::IoFailure,
			TEXT("exact SHM could not mint an unpredictable private region"));
	}

	struct stat DirectoryStat = {};
	if (::fstat(DirectoryFd, &DirectoryStat) != 0 || !S_ISDIR(DirectoryStat.st_mode)
		|| DirectoryStat.st_uid != ::geteuid() || (DirectoryStat.st_mode & 077) != 0)
	{
		::close(DirectoryFd);
		FTCHARToUTF8 NameUtf8(*RegionName);
		::unlinkat(BaseFd, NameUtf8.Get(), AT_REMOVEDIR);
		::close(BaseFd);
		return ShmExactFail(EURLabShmExactError::InvalidRegion,
			TEXT("exact SHM region is not a private creator-owned directory"));
	}

	const int MarkerFd = ::openat(DirectoryFd, ".urlab-owner",
		O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, S_IRUSR | S_IWUSR);
	if (MarkerFd < 0)
	{
		::close(DirectoryFd);
		FTCHARToUTF8 NameUtf8(*RegionName);
		::unlinkat(BaseFd, NameUtf8.Get(), AT_REMOVEDIR);
		::close(BaseFd);
		return ShmExactFail(EURLabShmExactError::IoFailure,
			TEXT("exact SHM creator marker could not be created exclusively"));
	}
	const FString RegionId = ShmExactRandomToken();
	const FGuid GenerationGuid = FGuid::NewGuid();
	uint64 Generation = ((static_cast<uint64>(GenerationGuid.A) << 32) | GenerationGuid.B)
		& static_cast<uint64>(MAX_int64);
	if (Generation == 0) Generation = 1;
	const FString Marker = BootId + TEXT("\n") + RegionId + TEXT("\n")
		+ LexToString(Generation) + TEXT("\n");
	FTCHARToUTF8 MarkerUtf8(*Marker);
	const bool bMarkerWritten = ShmExactWriteAllAt(MarkerFd,
		reinterpret_cast<const uint8*>(MarkerUtf8.Get()), MarkerUtf8.Length());
	const bool bMarkerSynced = ::fsync(MarkerFd) == 0;
	struct stat MarkerStat = {};
	const bool bMarkerIdentity = ::fstat(MarkerFd, &MarkerStat) == 0
		&& S_ISREG(MarkerStat.st_mode) && MarkerStat.st_nlink == 1
		&& MarkerStat.st_uid == ::geteuid();
	::close(MarkerFd);
	if (!bMarkerWritten || !bMarkerSynced || !bMarkerIdentity)
	{
		::unlinkat(DirectoryFd, ".urlab-owner", 0);
		::close(DirectoryFd);
		FTCHARToUTF8 NameUtf8(*RegionName);
		::unlinkat(BaseFd, NameUtf8.Get(), AT_REMOVEDIR);
		::close(BaseFd);
		return ShmExactFail(EURLabShmExactError::IoFailure,
			TEXT("exact SHM creator marker could not be persisted"));
	}

	TUniquePtr<FState> NewState = MakeUnique<FState>();
	NewState->BaseDirectoryFd = BaseFd;
	NewState->DirectoryFd = DirectoryFd;
	NewState->RegionDirectoryName = RegionName;
	NewState->DirectoryDevice = static_cast<uint64>(DirectoryStat.st_dev);
	NewState->DirectoryInode = static_cast<uint64>(DirectoryStat.st_ino);
	NewState->MarkerDevice = static_cast<uint64>(MarkerStat.st_dev);
	NewState->MarkerInode = static_cast<uint64>(MarkerStat.st_ino);
	NewState->Descriptor.BootId = BootId;
	NewState->Descriptor.RegionId = RegionId;
	NewState->Descriptor.Generation = Generation;
	NewState->Descriptor.DirectoryPath = FPaths::Combine(
		UTF8_TO_TCHAR(CanonicalBase), RegionName);
	OutRegion = MakeShareable(new FURLabShmExactRegion(MoveTemp(NewState)));
	return ShmExactOk();
#endif
}

FURLabShmExactStatus FURLabShmExactRegion::CreateForBoot(const FString& BootId,
	FURLabShmExactRegionPtr& OutRegion)
{
	OutRegion.Reset();
#if !PLATFORM_LINUX
	return ShmExactUnsupported();
#else
	if (BootId.IsEmpty())
		return ShmExactFail(EURLabShmExactError::InvalidArgument,
			TEXT("exact SHM boot identity must not be empty"));

	struct statfs ShmFs = {};
	if (::statfs("/dev/shm", &ShmFs) != 0 || ShmFs.f_type != TMPFS_MAGIC)
		return ShmExactFail(EURLabShmExactError::Unsupported,
			TEXT("Linux shared-memory tmpfs is unavailable"));

	const FString BaseDirectory = FString::Printf(TEXT("/dev/shm/urlab-%u"),
		static_cast<uint32>(::geteuid()));
	FTCHARToUTF8 BaseUtf8(*BaseDirectory);
	if (::mkdir(BaseUtf8.Get(), S_IRWXU) != 0 && errno != EEXIST)
		return ShmExactFail(EURLabShmExactError::IoFailure,
			TEXT("private user-scoped SHM base could not be created"));
	const int BaseFd = ::open(BaseUtf8.Get(),
		O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
	struct stat BaseStat = {};
	const bool bPrivateBase = BaseFd >= 0 && ::fstat(BaseFd, &BaseStat) == 0
		&& S_ISDIR(BaseStat.st_mode) && BaseStat.st_uid == ::geteuid()
		&& (BaseStat.st_mode & 077) == 0;
	if (BaseFd >= 0) ::close(BaseFd);
	if (!bPrivateBase)
		return ShmExactFail(EURLabShmExactError::InvalidRegion,
			TEXT("user-scoped SHM base is not a private creator-owned directory"));
	return Create(BootId, BaseDirectory, OutRegion);
#endif
}

const FURLabShmExactRegionDescriptor& FURLabShmExactRegion::GetDescriptor() const
{
	return State->Descriptor;
}

FURLabShmExactStatus FURLabShmExactRegion::CreateLiveObject(int64 ByteLength,
	FURLabShmLiveObjectPtr& OutObject)
{
	OutObject.Reset();
#if !PLATFORM_LINUX
	return ShmExactUnsupported();
#else
	if (ByteLength <= 0 || ByteLength > MAX_int32)
		return ShmExactFail(EURLabShmExactError::Bounds,
			TEXT("live SHM object length is outside supported bounds"));
	FScopeLock Lock(&State->Mutex);
	FState::FObjectRecord Record;
	Record.Kind = FState::EObjectKind::Live;
	FString ObjectName;
	const int Fd = State->CreateObject(TEXT("liv-"), ByteLength, ObjectName, Record);
	if (Fd < 0)
		return ShmExactFail(EURLabShmExactError::IoFailure,
			TEXT("live SHM object creation failed"));
	State->Objects.Add(ObjectName, Record);
	FURLabShmLiveObjectDescriptor Descriptor;
	Descriptor.RegionId = State->Descriptor.RegionId;
	Descriptor.RegionGeneration = State->Descriptor.Generation;
	Descriptor.ObjectName = ObjectName;
	Descriptor.ByteLength = ByteLength;
	OutObject = MakeShareable(new FURLabShmLiveObject(AsShared(), MoveTemp(Descriptor), Fd, true));
	return ShmExactOk();
#endif
}

FURLabShmExactStatus FURLabShmExactRegion::OpenLiveObject(
	const FURLabShmLiveObjectDescriptor& Descriptor, FURLabShmLiveObjectPtr& OutObject)
{
	OutObject.Reset();
#if !PLATFORM_LINUX
	return ShmExactUnsupported();
#else
	if (Descriptor.RegionId != State->Descriptor.RegionId
		|| Descriptor.RegionGeneration != State->Descriptor.Generation)
		return ShmExactFail(EURLabShmExactError::IdentityMismatch,
			TEXT("live SHM object region identity is stale"));
	if (!ShmExactIsObjectName(Descriptor.ObjectName, TEXT("liv-"))
		|| Descriptor.ByteLength <= 0 || Descriptor.ByteLength > MAX_int32)
		return ShmExactFail(EURLabShmExactError::InvalidArgument,
			TEXT("live SHM object descriptor is invalid"));
	FScopeLock Lock(&State->Mutex);
	const FState::FObjectRecord* Record = State->Objects.Find(Descriptor.ObjectName);
	if (!Record || Record->Kind != FState::EObjectKind::Live
		|| Record->ByteLength != Descriptor.ByteLength
		|| !State->ValidatePathIdentity(Descriptor.ObjectName, *Record, false))
		return ShmExactFail(EURLabShmExactError::IdentityMismatch,
			TEXT("live SHM object path identity does not match"));
	FTCHARToUTF8 NameUtf8(*Descriptor.ObjectName);
	const int Fd = ::openat(State->DirectoryFd, NameUtf8.Get(),
		O_RDWR | O_NOFOLLOW | O_CLOEXEC);
	if (Fd < 0)
		return ShmExactFail(EURLabShmExactError::IoFailure,
			TEXT("live SHM object open failed"));
	struct stat OpenStat = {};
	if (::fstat(Fd, &OpenStat) != 0
		|| static_cast<uint64>(OpenStat.st_dev) != Record->Device
		|| static_cast<uint64>(OpenStat.st_ino) != Record->Inode)
	{
		::close(Fd);
		return ShmExactFail(EURLabShmExactError::IdentityMismatch,
			TEXT("opened live SHM object identity changed"));
	}
	OutObject = MakeShareable(new FURLabShmLiveObject(AsShared(), Descriptor, Fd, false));
	return ShmExactOk();
#endif
}

void FURLabShmExactRegion::RetireLiveObject(FURLabShmLiveObject& Object)
{
#if PLATFORM_LINUX
	if (!Object.bCreator) return;
	FScopeLock Lock(&State->Mutex);
	const FState::FObjectRecord* Record = State->Objects.Find(Object.Descriptor.ObjectName);
	if (!Record || Record->Kind != FState::EObjectKind::Live) return;
	const FState::FObjectRecord Copy = *Record;
	if (State->UnlinkIfExact(Object.Descriptor.ObjectName, Copy))
		State->Objects.Remove(Object.Descriptor.ObjectName);
#endif
}

FURLabShmExactStatus FURLabShmExactRegion::CreateStaging(FURLabPayloadStore& Store,
	const FURLabPayloadOwner& Owner, int64 ByteLength, const FString& ExpectedSha256,
	FURLabShmExactStagingPtr& OutStaging)
{
	OutStaging.Reset();
#if !PLATFORM_LINUX
	return ShmExactUnsupported();
#else
	const FString CanonicalSha = ExpectedSha256.ToLower();
	if (Owner.BootId != State->Descriptor.BootId || ByteLength <= 0 || ByteLength > MAX_int32
		|| !FURLabAssetCache::IsValidSha256Hex(CanonicalSha))
	{
		return ShmExactFail(EURLabShmExactError::InvalidArgument,
			TEXT("exact SHM staging requires bounded bytes and a valid SHA-256"));
	}
	FURLabPayloadPhysicalReservationPtr Reservation;
	if (const FURLabPayloadStoreStatus Status =
		Store.ReservePhysicalCapacity(Owner, ByteLength, Reservation); !Status)
	{
		return ShmExactFromStore(Status);
	}

	FScopeLock Lock(&State->Mutex);
	FString ObjectName;
	FState::FObjectRecord Record;
	Record.Kind = FState::EObjectKind::Staging;
	Record.Owner = Owner;
	Record.Reservation = Reservation;
	Record.Sha256 = CanonicalSha;
	const int Fd = State->CreateObject(TEXT("stg-"), ByteLength, ObjectName, Record);
	if (Fd < 0)
		return ShmExactFail(EURLabShmExactError::IoFailure,
			TEXT("exact SHM staging file could not be created exclusively"));
	State->Objects.Add(ObjectName, Record);
	FURLabShmExactStagingGrant Grant;
	Grant.RegionId = State->Descriptor.RegionId;
	Grant.Generation = State->Descriptor.Generation;
	Grant.ObjectName = ObjectName;
	Grant.ByteLength = ByteLength;
	Grant.ExpectedSha256 = CanonicalSha;
	OutStaging = MakeShareable(new FURLabShmExactStaging(AsShared(), Owner,
		MoveTemp(Grant), Fd, MoveTemp(Reservation)));
	return ShmExactOk();
#endif
}

FURLabShmExactStatus FURLabShmExactRegion::SealStaging(FURLabPayloadStore& Store,
	const FURLabPayloadOwner& Owner, const FURLabShmExactStagingPtr& Staging,
	FURLabShmExactSealedPayloadPtr& OutSealed)
{
	OutSealed.Reset();
#if !PLATFORM_LINUX
	return ShmExactUnsupported();
#else
	if (Owner.BootId != State->Descriptor.BootId || !Staging
		|| Staging->Region.Get() != this || !(Staging->Owner == Owner))
	{
		return ShmExactFail(EURLabShmExactError::IdentityMismatch,
			TEXT("exact SHM staging handle does not belong to this live region"));
	}
	int64 StagingByteLength = 0;
	{
		FScopeLock InitialLock(&State->Mutex);
		if (Staging->bRetired || !Staging->Reservation)
			return ShmExactFail(EURLabShmExactError::IdentityMismatch,
				TEXT("exact SHM staging handle is already retired"));
		StagingByteLength = Staging->Grant.ByteLength;
	}
	FURLabPayloadPhysicalReservationPtr SealedReservation;
	if (const FURLabPayloadStoreStatus Status = Store.ReservePhysicalCapacity(
			Owner, StagingByteLength, SealedReservation); !Status)
	{
		return ShmExactFromStore(Status);
	}

	FScopeLock Lock(&State->Mutex);
	if (Staging->bRetired || !Staging->Reservation)
		return ShmExactFail(EURLabShmExactError::IdentityMismatch,
			TEXT("exact SHM staging handle was retired during seal admission"));
	FState::FObjectRecord* SourceRecord = State->Objects.Find(Staging->Grant.ObjectName);
	if (!SourceRecord || SourceRecord->Kind != FState::EObjectKind::Staging
		|| !(SourceRecord->Owner == Owner)
		|| !State->ValidatePathIdentity(Staging->Grant.ObjectName, *SourceRecord, false))
	{
		return ShmExactFail(EURLabShmExactError::IdentityMismatch,
			TEXT("exact SHM staging path no longer has its minted file identity"));
	}
	struct stat OpenSourceStat = {};
	if (::fstat(Staging->FileDescriptor, &OpenSourceStat) != 0
		|| !ShmExactValidFileStat(OpenSourceStat, SourceRecord->ByteLength, false)
		|| OpenSourceStat.st_nlink != 1
		|| static_cast<uint64>(OpenSourceStat.st_dev) != SourceRecord->Device
		|| static_cast<uint64>(OpenSourceStat.st_ino) != SourceRecord->Inode)
	{
		return ShmExactFail(EURLabShmExactError::IdentityMismatch,
			TEXT("exact SHM staging descriptor identity changed"));
	}
	if (::fsync(Staging->FileDescriptor) != 0)
		return ShmExactFail(EURLabShmExactError::IoFailure,
			TEXT("exact SHM staging file could not be synchronized"));

	const FState::FObjectRecord SourceCopy = *SourceRecord;
	FString SealedName;
	FState::FObjectRecord SealedRecord;
	SealedRecord.Kind = FState::EObjectKind::Sealed;
	SealedRecord.Owner = Owner;
	SealedRecord.Reservation = SealedReservation;
	const int WritableFd = State->CreateObject(TEXT("obj-"), SourceCopy.ByteLength,
		SealedName, SealedRecord);
	if (WritableFd < 0)
		return ShmExactFail(EURLabShmExactError::IoFailure,
			TEXT("exact SHM sealed file could not be created exclusively"));
	FTCHARToUTF8 SealedNameUtf8(*SealedName);
	auto FailSealed = [&]
	{
		::close(WritableFd);
		State->UnlinkIfExact(SealedName, SealedRecord);
	};
	if (!ShmExactCopyFile(Staging->FileDescriptor, WritableFd, SourceCopy.ByteLength)
		|| ::fsync(WritableFd) != 0)
	{
		FailSealed();
		return ShmExactFail(EURLabShmExactError::IoFailure,
			TEXT("exact SHM staging bytes could not be copied and synchronized"));
	}
	FString ActualSha;
	if (!ShmExactHashFile(WritableFd, SourceCopy.ByteLength, ActualSha))
	{
		FailSealed();
		return ShmExactFail(EURLabShmExactError::IoFailure,
			TEXT("exact SHM sealed bytes could not be hashed"));
	}
	if (ActualSha != Staging->Grant.ExpectedSha256)
	{
		FailSealed();
		return ShmExactFail(EURLabShmExactError::IntegrityMismatch,
			TEXT("exact SHM staged bytes do not match the declared SHA-256"));
	}
	if (::fchmod(WritableFd, S_IRUSR) != 0 || ::fsync(WritableFd) != 0)
	{
		FailSealed();
		return ShmExactFail(EURLabShmExactError::IoFailure,
			TEXT("exact SHM sealed file could not be made read-only"));
	}
	::close(WritableFd);
	const int ReadOnlyFd = ::openat(State->DirectoryFd, SealedNameUtf8.Get(),
		O_RDONLY | O_NOFOLLOW | O_CLOEXEC);
	struct stat ReadOnlyStat = {};
	if (ReadOnlyFd < 0 || ::fstat(ReadOnlyFd, &ReadOnlyStat) != 0
		|| !ShmExactValidFileStat(ReadOnlyStat, SourceCopy.ByteLength, true)
		|| ReadOnlyStat.st_nlink != 1
		|| static_cast<uint64>(ReadOnlyStat.st_dev) != SealedRecord.Device
		|| static_cast<uint64>(ReadOnlyStat.st_ino) != SealedRecord.Inode)
	{
		if (ReadOnlyFd >= 0) ::close(ReadOnlyFd);
		State->UnlinkIfExact(SealedName, SealedRecord);
		return ShmExactFail(EURLabShmExactError::IdentityMismatch,
			TEXT("exact SHM read-only publication identity validation failed"));
	}

	SealedRecord.Sha256 = ActualSha;
	FURLabShmExactSealedDescriptor Descriptor;
	Descriptor.RegionId = State->Descriptor.RegionId;
	Descriptor.Generation = State->Descriptor.Generation;
	Descriptor.ObjectName = SealedName;
	Descriptor.ByteLength = SourceCopy.ByteLength;
	Descriptor.Sha256 = ActualSha;

	if (!State->UnlinkIfExact(Staging->Grant.ObjectName, SourceCopy))
	{
		::close(ReadOnlyFd);
		State->UnlinkIfExact(SealedName, SealedRecord);
		return ShmExactFail(EURLabShmExactError::IoFailure,
			TEXT("exact SHM mutable staging path could not be retired"));
	}
	State->Objects.Add(SealedName, SealedRecord);
	State->Objects.Remove(Staging->Grant.ObjectName);
	::close(Staging->FileDescriptor);
	Staging->FileDescriptor = -1;
	Staging->bRetired = true;
	Staging->Reservation.Reset();
	OutSealed = MakeShareable(new FURLabShmExactSealedPayload(AsShared(),
		Owner, MoveTemp(Descriptor), ReadOnlyFd, MoveTemp(SealedReservation)));
	return ShmExactOk();
#endif
}

FURLabShmExactStatus FURLabShmExactRegion::CreateFromBytes(FURLabPayloadStore& Store,
	const FURLabPayloadOwner& Owner, TConstArrayView<uint8> Bytes,
	FURLabShmExactSealedPayloadPtr& OutSealed)
{
	OutSealed.Reset();
#if !PLATFORM_LINUX
	return ShmExactUnsupported();
#else
	FURLabPayloadPhysicalReservationPtr Reservation;
	if (const FURLabPayloadStoreStatus Status =
		Store.ReservePhysicalCapacity(Owner, Bytes.Num(), Reservation); !Status)
	{
		return ShmExactFromStore(Status);
	}
	bool bRetrySafe = true;
	return CreateFromBytesRetained(Owner, Bytes, Reservation, OutSealed, bRetrySafe);
#endif
}

FURLabShmExactStatus FURLabShmExactRegion::CreateFromBytesRetained(
	const FURLabPayloadOwner& Owner, TConstArrayView<uint8> Bytes,
	const FURLabPayloadPhysicalReservationPtr& Reservation,
	FURLabShmExactSealedPayloadPtr& OutSealed, bool& bOutRetrySafe)
{
	OutSealed.Reset();
	bOutRetrySafe = true;
#if !PLATFORM_LINUX
	return ShmExactUnsupported();
#else
	if (Owner.BootId != State->Descriptor.BootId || Bytes.Num() <= 0 || !Reservation)
		return ShmExactFail(EURLabShmExactError::InvalidArgument,
			TEXT("exact SHM publication requires owned bytes and retained capacity"));
	FScopeLock Lock(&State->Mutex);
	FString ObjectName;
	FState::FObjectRecord Record;
	Record.Kind = FState::EObjectKind::Sealed;
	Record.Owner = Owner;
	Record.Reservation = Reservation;
	const int WritableFd = State->CreateObject(TEXT("obj-"), Bytes.Num(), ObjectName, Record);
	if (WritableFd < 0)
		return ShmExactFail(EURLabShmExactError::IoFailure,
			TEXT("exact SHM output file could not be created exclusively"));
	FTCHARToUTF8 NameUtf8(*ObjectName);
	FString ActualSha;
	auto RetireFailedObject = [&]
	{
		if (!State->UnlinkIfExact(ObjectName, Record))
		{
			// Keep the exact identity and its charge live when the kernel path
			// could not be retired. A later region cleanup can retry safely.
			State->Objects.Add(ObjectName, Record);
			bOutRetrySafe = false;
		}
	};
	if (!ShmExactWriteAllAt(WritableFd, Bytes.GetData(), Bytes.Num())
		|| ::fsync(WritableFd) != 0
		|| !ShmExactHashFile(WritableFd, Bytes.Num(), ActualSha)
		|| ::fchmod(WritableFd, S_IRUSR) != 0 || ::fsync(WritableFd) != 0)
	{
		::close(WritableFd);
		RetireFailedObject();
		return ShmExactFail(EURLabShmExactError::IoFailure,
			TEXT("exact SHM output bytes could not be persisted read-only"));
	}
	::close(WritableFd);
	const int ReadOnlyFd = ::openat(State->DirectoryFd, NameUtf8.Get(),
		O_RDONLY | O_NOFOLLOW | O_CLOEXEC);
	struct stat ReadOnlyStat = {};
	if (ReadOnlyFd < 0 || ::fstat(ReadOnlyFd, &ReadOnlyStat) != 0
		|| !ShmExactValidFileStat(ReadOnlyStat, Bytes.Num(), true)
		|| ReadOnlyStat.st_nlink != 1
		|| static_cast<uint64>(ReadOnlyStat.st_dev) != Record.Device
		|| static_cast<uint64>(ReadOnlyStat.st_ino) != Record.Inode)
	{
		if (ReadOnlyFd >= 0) ::close(ReadOnlyFd);
		RetireFailedObject();
		return ShmExactFail(EURLabShmExactError::IdentityMismatch,
			TEXT("exact SHM output publication identity validation failed"));
	}
	Record.Sha256 = MoveTemp(ActualSha);
	State->Objects.Add(ObjectName, Record);
	FURLabShmExactSealedDescriptor Descriptor;
	Descriptor.RegionId = State->Descriptor.RegionId;
	Descriptor.Generation = State->Descriptor.Generation;
	Descriptor.ObjectName = ObjectName;
	Descriptor.ByteLength = Bytes.Num();
	Descriptor.Sha256 = Record.Sha256;
	OutSealed = MakeShareable(new FURLabShmExactSealedPayload(AsShared(),
		Owner, MoveTemp(Descriptor), ReadOnlyFd, Reservation));
	return ShmExactOk();
#endif
}

FURLabShmExactStatus FURLabShmExactRegion::CreateChallenge(FURLabPayloadStore& Store,
	const FURLabPayloadOwner& Owner, FURLabShmExactChallengePtr& OutChallenge)
{
	OutChallenge.Reset();
#if !PLATFORM_LINUX
	return ShmExactUnsupported();
#else
	constexpr int64 ChallengeByteLength = 32;
	if (Owner.BootId != State->Descriptor.BootId || !Owner.Session)
		return ShmExactFail(EURLabShmExactError::IdentityMismatch,
			TEXT("SHM challenge owner does not belong to this boot region"));

	FURLabPayloadPhysicalReservationPtr Reservation;
	if (const FURLabPayloadStoreStatus Status = Store.ReservePhysicalCapacity(
			Owner, ChallengeByteLength, Reservation); !Status)
	{
		return ShmExactFromStore(Status);
	}
	TArray<uint8> Nonce;
	Nonce.SetNumUninitialized(ChallengeByteLength);
	if (!ShmExactGetRandom(Nonce.GetData(), Nonce.Num()))
		return ShmExactFail(EURLabShmExactError::IoFailure,
			TEXT("kernel random source could not fill the SHM challenge"));

	FScopeLock Lock(&State->Mutex);
	FString ChallengeName;
	FState::FObjectRecord Record;
	Record.Kind = FState::EObjectKind::Challenge;
	Record.Owner = Owner;
	Record.Reservation = Reservation;
	const int WritableFd = State->CreateObject(TEXT("chl-"), ChallengeByteLength,
		ChallengeName, Record);
	if (WritableFd < 0)
		return ShmExactFail(EURLabShmExactError::IoFailure,
			TEXT("SHM challenge file could not be created exclusively"));
	FTCHARToUTF8 NameUtf8(*ChallengeName);
	auto RetireFailedChallenge = [&]
	{
		if (!State->UnlinkIfExact(ChallengeName, Record))
		{
			// Preserve both the exact identity and its physical charge when
			// cleanup cannot unlink. Region cleanup can retry the same record.
			State->Objects.Add(ChallengeName, Record);
		}
	};
	if (!ShmExactWriteAllAt(WritableFd, Nonce.GetData(), Nonce.Num())
		|| ::fsync(WritableFd) != 0 || ::fchmod(WritableFd, S_IRUSR) != 0
		|| ::fsync(WritableFd) != 0)
	{
		::close(WritableFd);
		RetireFailedChallenge();
		return ShmExactFail(EURLabShmExactError::IoFailure,
			TEXT("SHM challenge could not be persisted read-only"));
	}
	::close(WritableFd);
	const int ReadOnlyFd = ::openat(State->DirectoryFd, NameUtf8.Get(),
		O_RDONLY | O_NOFOLLOW | O_CLOEXEC);
	struct stat ReadOnlyStat = {};
	if (ReadOnlyFd < 0 || ::fstat(ReadOnlyFd, &ReadOnlyStat) != 0
		|| !ShmExactValidFileStat(ReadOnlyStat, ChallengeByteLength, true)
		|| ReadOnlyStat.st_nlink != 1
		|| static_cast<uint64>(ReadOnlyStat.st_dev) != Record.Device
		|| static_cast<uint64>(ReadOnlyStat.st_ino) != Record.Inode)
	{
		if (ReadOnlyFd >= 0) ::close(ReadOnlyFd);
		RetireFailedChallenge();
		return ShmExactFail(EURLabShmExactError::IdentityMismatch,
			TEXT("SHM challenge read-only file identity validation failed"));
	}
	if (!Owner.Session->IsActive())
	{
		::close(ReadOnlyFd);
		RetireFailedChallenge();
		return ShmExactFail(EURLabShmExactError::IdentityMismatch,
			TEXT("SHM challenge owner was deactivated during creation"));
	}

	State->Objects.Add(ChallengeName, Record);
	FURLabShmExactChallengeOffer Offer;
	Offer.RegionId = State->Descriptor.RegionId;
	Offer.RegionGeneration = State->Descriptor.Generation;
	Offer.ChallengeName = ChallengeName;
	Offer.ByteLength = ChallengeByteLength;
	OutChallenge = MakeShareable(new FURLabShmExactChallenge(AsShared(), Owner,
		MoveTemp(Offer), MoveTemp(Nonce), ReadOnlyFd, MoveTemp(Reservation)));
	return ShmExactOk();
#endif
}

FURLabShmExactStatus FURLabShmExactRegion::VerifyChallenge(
	const FURLabPayloadOwner& Owner, const FURLabShmExactChallengePtr& Challenge,
	const FURLabShmExactChallengeProof& Proof) const
{
	if (!Challenge || Challenge->Region.Get() != this || !(Challenge->Owner == Owner)
		|| Proof.RegionId != Challenge->Offer.RegionId
		|| Proof.RegionGeneration != Challenge->Offer.RegionGeneration
		|| Proof.ChallengeName != Challenge->Offer.ChallengeName
		|| Proof.Nonce.Num() != Challenge->Nonce.Num())
	{
		return ShmExactFail(EURLabShmExactError::ChallengeInvalid,
			TEXT("SHM challenge proof identity is invalid"));
	}

	if (Challenge->bRetired.load(std::memory_order_acquire))
		return ShmExactFail(EURLabShmExactError::ChallengeInvalid,
			TEXT("SHM challenge is already retired"));
	uint8 Difference = 0;
	for (int32 Index = 0; Index < Challenge->Nonce.Num(); ++Index)
		Difference |= Challenge->Nonce[Index] ^ Proof.Nonce[Index];
	if (Difference != 0)
		return ShmExactFail(EURLabShmExactError::ChallengeInvalid,
			TEXT("SHM challenge nonce is invalid"));
	return ShmExactOk();
}

FURLabShmExactStatus FURLabShmExactRegion::RetireChallenge(
	const FURLabPayloadOwner& Owner, const FURLabShmExactChallengePtr& Challenge)
{
	if (!Challenge || Challenge->Region.Get() != this || !(Challenge->Owner == Owner))
		return ShmExactFail(EURLabShmExactError::ChallengeInvalid,
			TEXT("SHM challenge does not belong to this exact owner and region"));
	return RetireChallengeHandle(*Challenge);
}

FURLabShmExactStatus FURLabShmExactRegion::RetireChallengeHandle(
	FURLabShmExactChallenge& Challenge)
{
#if !PLATFORM_LINUX
	return ShmExactUnsupported();
#else
	FScopeLock Lock(&State->Mutex);
	if (Challenge.bRetired.load(std::memory_order_acquire)) return ShmExactOk();
	FState::FObjectRecord* Record = State->Objects.Find(Challenge.Offer.ChallengeName);
	if (!Record || Record->Kind != FState::EObjectKind::Challenge
		|| !(Record->Owner == Challenge.Owner))
	{
		return ShmExactFail(EURLabShmExactError::ChallengeInvalid,
			TEXT("SHM challenge retained identity is invalid"));
	}
	Challenge.bRetired.store(true, std::memory_order_release);
	if (!State->UnlinkIfExact(Challenge.Offer.ChallengeName, *Record))
	{
		Challenge.bRetired.store(false, std::memory_order_release);
		return ShmExactFail(EURLabShmExactError::IoFailure,
			TEXT("SHM challenge file could not be retired exactly"));
	}
	State->Objects.Remove(Challenge.Offer.ChallengeName);
	if (Challenge.FileDescriptor >= 0) ::close(Challenge.FileDescriptor);
	Challenge.FileDescriptor = -1;
	Challenge.Reservation.Reset();
	return ShmExactOk();
#endif
}

namespace
{
#if PLATFORM_LINUX
FURLabShmExactStatus ShmExactReadFdCopied(int FileDescriptor,
	const FURLabShmExactSealedDescriptor& Descriptor, int64 Offset, int64 ByteLength,
	TArray<uint8>& OutBytes)
{
	OutBytes.Reset();
	if (Offset < 0 || ByteLength <= 0 || Offset > Descriptor.ByteLength
		|| ByteLength > Descriptor.ByteLength - Offset)
	{
		return ShmExactFail(EURLabShmExactError::Bounds,
			TEXT("exact SHM copied read is outside the sealed object"));
	}
	struct stat Stat = {};
	if (::fstat(FileDescriptor, &Stat) != 0
		|| !ShmExactValidFileStat(Stat, Descriptor.ByteLength, true)
		|| Stat.st_nlink > 1)
	{
		return ShmExactFail(EURLabShmExactError::IdentityMismatch,
			TEXT("exact SHM retained file is not its declared regular read-only object"));
	}
	FString ActualSha;
	if (!ShmExactHashFile(FileDescriptor, Descriptor.ByteLength, ActualSha))
		return ShmExactFail(EURLabShmExactError::IoFailure,
			TEXT("exact SHM sealed object could not be mapped read-only for verification"));
	if (ActualSha != Descriptor.Sha256)
		return ShmExactFail(EURLabShmExactError::IntegrityMismatch,
			TEXT("exact SHM sealed object digest changed"));
	OutBytes.Reserve(static_cast<int32>(ByteLength));
	OutBytes.SetNumUninitialized(static_cast<int32>(ByteLength));
	if (!ShmExactReadAllAt(FileDescriptor, OutBytes.GetData(), Offset, ByteLength))
		return ShmExactFail(EURLabShmExactError::IoFailure,
			TEXT("exact SHM sealed range could not be copied"));
	return ShmExactOk();
}
#endif
}

FURLabShmExactStatus FURLabShmExactRegion::ReadCopied(FURLabPayloadStore& Store,
	const FURLabPayloadOwner& Owner, const FURLabShmExactSealedDescriptor& Descriptor,
	int64 Offset, int64 ByteLength, FURLabShmExactCopiedBytesPtr& OutCopy)
{
	OutCopy.Reset();
#if !PLATFORM_LINUX
	return ShmExactUnsupported();
#else
	if (Owner.BootId != State->Descriptor.BootId
		|| Descriptor.RegionId != State->Descriptor.RegionId
		|| Descriptor.Generation != State->Descriptor.Generation
		|| Descriptor.ByteLength <= 0 || Descriptor.ByteLength > MAX_int32
		|| !ShmExactIsObjectName(Descriptor.ObjectName, TEXT("obj-"))
		|| !FURLabAssetCache::IsValidSha256Hex(Descriptor.Sha256))
	{
		return ShmExactFail(EURLabShmExactError::InvalidRegion,
			TEXT("exact SHM descriptor does not name this region generation"));
	}
	int ReadOnlyFd = -1;
	{
		FScopeLock Lock(&State->Mutex);
		const FState::FObjectRecord* Record = State->Objects.Find(Descriptor.ObjectName);
		if (!Record || Record->Kind != FState::EObjectKind::Sealed)
			return ShmExactFail(EURLabShmExactError::NotFound,
				TEXT("exact SHM sealed object is not published"));
		if (!(Record->Owner == Owner) || Record->ByteLength != Descriptor.ByteLength
			|| Record->Sha256 != Descriptor.Sha256
			|| !State->ValidatePathIdentity(Descriptor.ObjectName, *Record, true))
		{
			return ShmExactFail(EURLabShmExactError::IdentityMismatch,
				TEXT("exact SHM descriptor disagrees with its minted object identity"));
		}
		FTCHARToUTF8 NameUtf8(*Descriptor.ObjectName);
		ReadOnlyFd = ::openat(State->DirectoryFd, NameUtf8.Get(),
			O_RDONLY | O_NOFOLLOW | O_CLOEXEC);
		struct stat OpenStat = {};
		if (ReadOnlyFd >= 0 && (::fstat(ReadOnlyFd, &OpenStat) != 0
			|| !ShmExactValidFileStat(OpenStat, Record->ByteLength, true)
			|| static_cast<uint64>(OpenStat.st_dev) != Record->Device
			|| static_cast<uint64>(OpenStat.st_ino) != Record->Inode))
		{
			::close(ReadOnlyFd);
			ReadOnlyFd = -1;
		}
	}
	if (ReadOnlyFd < 0)
		return ShmExactFail(EURLabShmExactError::NotFound,
			TEXT("exact SHM sealed object could not be opened without following links"));
	TArray<uint8> Bytes;
	FURLabPayloadPhysicalReservationPtr Reservation;
	if (const FURLabPayloadStoreStatus StoreStatus = Store.ReservePhysicalCapacity(
		Owner, ByteLength, Reservation); !StoreStatus)
	{
		::close(ReadOnlyFd);
		return ShmExactFromStore(StoreStatus);
	}
	const FURLabShmExactStatus Status = ShmExactReadFdCopied(ReadOnlyFd,
		Descriptor, Offset, ByteLength, Bytes);
	::close(ReadOnlyFd);
	if (Status) OutCopy = MakeShareable(new FURLabShmExactCopiedBytes(
		MoveTemp(Bytes), MoveTemp(Reservation)));
	return Status;
#endif
}

FURLabShmExactStatus FURLabShmExactRegion::ReadCopied(FURLabPayloadStore& Store,
	const FURLabPayloadOwner& Owner, const FURLabShmExactSealedPayloadPtr& Retained,
	int64 Offset, int64 ByteLength, FURLabShmExactCopiedBytesPtr& OutCopy)
{
	OutCopy.Reset();
#if !PLATFORM_LINUX
	return ShmExactUnsupported();
#else
	if (Owner.BootId != State->Descriptor.BootId || !Retained
		|| Retained->Region.Get() != this || !(Retained->Owner == Owner)
		|| Retained->FileDescriptor < 0)
		return ShmExactFail(EURLabShmExactError::IdentityMismatch,
			TEXT("exact SHM retained handle does not belong to this region"));
	const int ReadOnlyFd = ::fcntl(Retained->FileDescriptor, F_DUPFD_CLOEXEC, 0);
	if (ReadOnlyFd < 0)
		return ShmExactFail(EURLabShmExactError::IoFailure,
			TEXT("exact SHM retained descriptor could not be duplicated"));
	TArray<uint8> Bytes;
	FURLabPayloadPhysicalReservationPtr Reservation;
	if (const FURLabPayloadStoreStatus StoreStatus = Store.ReservePhysicalCapacity(
		Owner, ByteLength, Reservation); !StoreStatus)
	{
		::close(ReadOnlyFd);
		return ShmExactFromStore(StoreStatus);
	}
	const FURLabShmExactStatus Status = ShmExactReadFdCopied(ReadOnlyFd,
		Retained->Descriptor, Offset, ByteLength, Bytes);
	::close(ReadOnlyFd);
	if (Status) OutCopy = MakeShareable(new FURLabShmExactCopiedBytes(
		MoveTemp(Bytes), MoveTemp(Reservation)));
	return Status;
#endif
}

FURLabShmExactStatus FURLabShmExactRegion::ReadRetainedCopied(
	const FURLabShmExactSealedPayloadPtr& Retained, int64 Offset, int64 ByteLength,
	FURLabPayloadPhysicalReservationPtr&& Reservation,
	FURLabShmExactCopiedBytesPtr& OutCopy)
{
	OutCopy.Reset();
#if !PLATFORM_LINUX
	return ShmExactUnsupported();
#else
	if (!Retained || Retained->Region.Get() != this || Retained->FileDescriptor < 0
		|| !Reservation)
	{
		return ShmExactFail(EURLabShmExactError::IdentityMismatch,
			TEXT("exact SHM retained copy requires its sealed handle and reservation"));
	}
	const int ReadOnlyFd = ::fcntl(Retained->FileDescriptor, F_DUPFD_CLOEXEC, 0);
	if (ReadOnlyFd < 0)
		return ShmExactFail(EURLabShmExactError::IoFailure,
			TEXT("exact SHM retained descriptor could not be duplicated"));
	TArray<uint8> Bytes;
	const FURLabShmExactStatus Status = ShmExactReadFdCopied(ReadOnlyFd,
		Retained->Descriptor, Offset, ByteLength, Bytes);
	::close(ReadOnlyFd);
	if (Status)
	{
		OutCopy = MakeShareable(new FURLabShmExactCopiedBytes(
			MoveTemp(Bytes), MoveTemp(Reservation)));
	}
	return Status;
#endif
}

FURLabShmExactStatus FURLabShmExactRegion::Release(
	const FURLabPayloadOwner& Owner, const FURLabShmExactSealedPayloadPtr& Sealed)
{
#if !PLATFORM_LINUX
	return ShmExactUnsupported();
#else
	if (Owner.BootId != State->Descriptor.BootId || !Sealed
		|| Sealed->Region.Get() != this || !(Sealed->Owner == Owner))
		return ShmExactFail(EURLabShmExactError::IdentityMismatch,
			TEXT("exact SHM sealed handle does not belong to this region"));
	FScopeLock Lock(&State->Mutex);
	if (!Sealed->bPublished) return ShmExactOk();
	const FState::FObjectRecord* Record = State->Objects.Find(Sealed->Descriptor.ObjectName);
	if (!Record || Record->Kind != FState::EObjectKind::Sealed
		|| !(Record->Owner == Owner)
		|| Record->ByteLength != Sealed->Descriptor.ByteLength
		|| Record->Sha256 != Sealed->Descriptor.Sha256
		|| !State->ValidatePathIdentity(Sealed->Descriptor.ObjectName, *Record, true))
		return ShmExactFail(EURLabShmExactError::IdentityMismatch,
			TEXT("exact SHM sealed path no longer has its minted owner and identity"));
	const FState::FObjectRecord RecordCopy = *Record;
	if (!State->UnlinkIfExact(Sealed->Descriptor.ObjectName, RecordCopy))
		return ShmExactFail(EURLabShmExactError::IoFailure,
			TEXT("exact SHM sealed path could not be released"));
	State->Objects.Remove(Sealed->Descriptor.ObjectName);
	Sealed->bPublished = false;
	return ShmExactOk();
#endif
}

void FURLabShmExactRegion::RetireStaging(FURLabShmExactStaging& Staging)
{
#if PLATFORM_LINUX
	FScopeLock Lock(&State->Mutex);
	if (!Staging.bRetired)
	{
		if (const FState::FObjectRecord* Record = State->Objects.Find(Staging.Grant.ObjectName))
		{
			const FState::FObjectRecord RecordCopy = *Record;
			if (State->UnlinkIfExact(Staging.Grant.ObjectName, RecordCopy))
				State->Objects.Remove(Staging.Grant.ObjectName);
		}
		Staging.bRetired = true;
	}
	if (Staging.FileDescriptor >= 0)
	{
		::close(Staging.FileDescriptor);
		Staging.FileDescriptor = -1;
	}
#endif
}

void FURLabShmExactRegion::RetireSealed(FURLabShmExactSealedPayload& Sealed)
{
#if PLATFORM_LINUX
	FScopeLock Lock(&State->Mutex);
	if (!Sealed.bPublished) return;
	if (const FState::FObjectRecord* Record = State->Objects.Find(Sealed.Descriptor.ObjectName))
	{
		const FState::FObjectRecord RecordCopy = *Record;
		if (State->UnlinkIfExact(Sealed.Descriptor.ObjectName, RecordCopy))
			State->Objects.Remove(Sealed.Descriptor.ObjectName);
		else
			return;
	}
	Sealed.bPublished = false;
#endif
}
