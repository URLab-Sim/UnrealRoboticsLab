// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "CoreMinimal.h"
#include "HAL/FileManager.h"
#include "Misc/AutomationTest.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"

#include "Bridge/URLabPayloadStore.h"
#include "Transport/ShmExactPayload.h"
#include "Utils/AssetCache.h"

#if PLATFORM_LINUX
#include <cerrno>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

#if WITH_DEV_AUTOMATION_TESTS

namespace URLabShmExactPayloadTestPrivate
{
FURLabPayloadStoreLimits MakeLimits(int64 SessionBytes = 256)
{
	FURLabPayloadStoreLimits Limits;
	Limits.MaxBlobBytes = FMath::Min<int64>(64, SessionBytes);
	Limits.MaxChunkBytes = static_cast<int32>(FMath::Min<int64>(64, Limits.MaxBlobBytes));
	Limits.MaxChunksPerBlob = 8;
	Limits.MaxSessionBytes = SessionBytes;
	Limits.MaxGlobalBytes = 512;
	Limits.MaxBlobCount = 16;
	Limits.MaxSessionBlobCount = 8;
	Limits.MaxSessionPhysicalObjectCount = 16;
	Limits.MaxActiveSessions = 2;
	Limits.StagingTtlSeconds = 300.0;
	Limits.SealedTtlSeconds = 300.0;
	Limits.ExpiredTombstoneTtlSeconds = 300.0;
	return Limits;
}

FString MakeBaseDirectory()
{
	return FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("URLabTests"),
		TEXT("ShmExactPayload-") + FGuid::NewGuid().ToString(EGuidFormats::Digits));
}

TArray<uint8> MakePayload()
{
	return {0x10, 0x20, 0x30, 0x40, 0x51, 0x62, 0x73, 0x84};
}

bool OpenOwner(FAutomationTestBase& Test, FURLabPayloadStore& Store,
	const TCHAR* SessionId, FURLabPayloadOwner& OutOwner,
	const TCHAR* BootId = TEXT("boot-shm"))
{
	FURLabPayloadSessionPtr Session;
	const FURLabPayloadStoreStatus Status = Store.OpenSession(BootId, SessionId,
		Session);
	if (!Test.TestTrue(TEXT("exact SHM payload session opens"), Status.IsOk()))
	{
		Test.AddError(Status.Message);
		return false;
	}
	OutOwner = FURLabPayloadOwner::FromSession(Session);
	return true;
}

#if PLATFORM_LINUX
bool WriteFromChild(const FString& FullPath, TConstArrayView<uint8> Bytes)
{
	FTCHARToUTF8 PathUtf8(*FullPath);
	const pid_t Child = ::fork();
	if (Child < 0) return false;
	if (Child == 0)
	{
		const int Fd = ::open(PathUtf8.Get(), O_WRONLY | O_NOFOLLOW | O_CLOEXEC);
		if (Fd < 0) ::_exit(11);
		int64 Written = 0;
		while (Written < Bytes.Num())
		{
			const ssize_t Result = ::pwrite(Fd, Bytes.GetData() + Written,
				static_cast<size_t>(Bytes.Num() - Written), static_cast<off_t>(Written));
			if (Result < 0 && errno == EINTR) continue;
			if (Result <= 0)
			{
				::close(Fd);
				::_exit(12);
			}
			Written += Result;
		}
		const bool bSynced = ::fsync(Fd) == 0;
		::close(Fd);
		::_exit(bSynced ? 0 : 13);
	}
	int Status = 0;
	while (::waitpid(Child, &Status, 0) < 0)
	{
		if (errno != EINTR) return false;
	}
	return WIFEXITED(Status) && WEXITSTATUS(Status) == 0;
}

bool ReadChallengeFile(const FString& FullPath, TArray<uint8>& OutBytes)
{
	OutBytes.Reset();
	FTCHARToUTF8 PathUtf8(*FullPath);
	const int Fd = ::open(PathUtf8.Get(), O_RDONLY | O_NOFOLLOW | O_CLOEXEC);
	if (Fd < 0) return false;
	struct stat Stat = {};
	if (::fstat(Fd, &Stat) != 0 || !S_ISREG(Stat.st_mode) || Stat.st_size != 32)
	{
		::close(Fd);
		return false;
	}
	OutBytes.SetNumUninitialized(32);
	int64 Read = 0;
	while (Read < OutBytes.Num())
	{
		const ssize_t Result = ::pread(Fd, OutBytes.GetData() + Read,
			static_cast<size_t>(OutBytes.Num() - Read), static_cast<off_t>(Read));
		if (Result < 0 && errno == EINTR) continue;
		if (Result <= 0)
		{
			::close(Fd);
			return false;
		}
		Read += Result;
	}
	::close(Fd);
	return true;
}
#endif
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabShmBootstrapChallengeTest,
	"URLab.ShmExactPayload.BootstrapChallengeProofAndRetirement",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabShmBootstrapChallengeTest::RunTest(const FString&)
{
	using namespace URLabShmExactPayloadTestPrivate;
#if !PLATFORM_LINUX
	FURLabShmExactRegionPtr Unsupported;
	TestEqual(TEXT("boot-scoped SHM region is explicitly unsupported"),
		FURLabShmExactRegion::CreateForBoot(TEXT("boot-challenge"), Unsupported).Code,
		EURLabShmExactError::Unsupported);
	return true;
#else
	FURLabPayloadStore Store(TEXT("boot-challenge"), MakeLimits());
	FURLabPayloadOwner OwnerA;
	FURLabPayloadOwner OwnerB;
	if (!OpenOwner(*this, Store, TEXT("session-a"), OwnerA, TEXT("boot-challenge"))
		|| !OpenOwner(*this, Store, TEXT("session-b"), OwnerB,
			TEXT("boot-challenge"))) return false;

	FURLabShmExactRegionPtr Region;
	if (!TestTrue(TEXT("boot helper creates Linux SHM region"),
		FURLabShmExactRegion::CreateForBoot(TEXT("boot-challenge"), Region).IsOk()))
		return false;
	const FURLabShmExactRegionDescriptor RegionDescriptor = Region->GetDescriptor();
	TestTrue(TEXT("boot helper stays under /dev/shm"),
		RegionDescriptor.DirectoryPath.StartsWith(TEXT("/dev/shm/")));
	struct stat DirectoryStat = {};
	FTCHARToUTF8 DirectoryUtf8(*RegionDescriptor.DirectoryPath);
	TestTrue(TEXT("boot region directory is creator-private"),
		::stat(DirectoryUtf8.Get(), &DirectoryStat) == 0
		&& S_ISDIR(DirectoryStat.st_mode) && DirectoryStat.st_uid == ::geteuid()
		&& (DirectoryStat.st_mode & 077) == 0);

	FURLabShmExactChallengePtr Challenge;
	if (!TestTrue(TEXT("challenge creation validates and reserves exact owner"),
		Region->CreateChallenge(Store, OwnerA, Challenge).IsOk())) return false;
	const FURLabShmExactChallengeOffer Offer = Challenge->GetOffer();
	TestEqual(TEXT("challenge offer uses region identity"), Offer.RegionId,
		RegionDescriptor.RegionId);
	TestEqual(TEXT("challenge offer uses region generation"), Offer.RegionGeneration,
		RegionDescriptor.Generation);
	TestEqual(TEXT("challenge offer exposes exact fixed length"), Offer.ByteLength, 32ll);
	TestTrue(TEXT("challenge name has exact minted form"),
		Offer.ChallengeName.StartsWith(TEXT("chl-"))
		&& Offer.ChallengeName.EndsWith(TEXT(".bin"))
		&& Offer.ChallengeName.Len() == 40);
	TestEqual(TEXT("challenge owns one exact physical reservation"),
		Store.GetStats().PhysicalBytes, 32ll);

	const FString ChallengePath = FPaths::Combine(RegionDescriptor.DirectoryPath,
		Offer.ChallengeName);
	struct stat ChallengeStat = {};
	FTCHARToUTF8 ChallengeUtf8(*ChallengePath);
	TestTrue(TEXT("challenge is a private read-only regular file"),
		::lstat(ChallengeUtf8.Get(), &ChallengeStat) == 0
		&& S_ISREG(ChallengeStat.st_mode) && ChallengeStat.st_uid == ::geteuid()
		&& (ChallengeStat.st_mode & 0777) == S_IRUSR);
	TArray<uint8> Nonce;
	if (!TestTrue(TEXT("client reads exact challenge without following links"),
		ReadChallengeFile(ChallengePath, Nonce))) return false;
	FURLabShmExactChallengeProof Proof;
	Proof.RegionId = Offer.RegionId;
	Proof.RegionGeneration = Offer.RegionGeneration;
	Proof.ChallengeName = Offer.ChallengeName;
	Proof.Nonce = Nonce;
	TestTrue(TEXT("exact file proof verifies from retained memory"),
		Region->VerifyChallenge(OwnerA, Challenge, Proof).IsOk());

	FURLabShmExactChallengeProof WrongNonce = Proof;
	WrongNonce.Nonce[0] ^= 0xff;
	TestEqual(TEXT("wrong nonce is rejected"),
		Region->VerifyChallenge(OwnerA, Challenge, WrongNonce).Code,
		EURLabShmExactError::ChallengeInvalid);
	TestEqual(TEXT("wrong exact owner is rejected"),
		Region->VerifyChallenge(OwnerB, Challenge, Proof).Code,
		EURLabShmExactError::ChallengeInvalid);
	FURLabShmExactChallengeProof WrongRegion = Proof;
	WrongRegion.RegionId += TEXT("-other");
	TestEqual(TEXT("wrong region is rejected"),
		Region->VerifyChallenge(OwnerA, Challenge, WrongRegion).Code,
		EURLabShmExactError::ChallengeInvalid);
	FURLabShmExactChallengeProof WrongGeneration = Proof;
	++WrongGeneration.RegionGeneration;
	TestEqual(TEXT("wrong generation is rejected"),
		Region->VerifyChallenge(OwnerA, Challenge, WrongGeneration).Code,
		EURLabShmExactError::ChallengeInvalid);
	FURLabShmExactChallengeProof WrongName = Proof;
	WrongName.ChallengeName = TEXT("chl-00000000000000000000000000000000.bin");
	TestEqual(TEXT("wrong challenge name is rejected"),
		Region->VerifyChallenge(OwnerA, Challenge, WrongName).Code,
		EURLabShmExactError::ChallengeInvalid);
	FURLabShmExactRegionPtr OtherRegion;
	if (!TestTrue(TEXT("independent boot region creates"),
		FURLabShmExactRegion::CreateForBoot(TEXT("boot-challenge"), OtherRegion).IsOk()))
		return false;
	TestEqual(TEXT("another region cannot validate retained challenge"),
		OtherRegion->VerifyChallenge(OwnerA, Challenge, Proof).Code,
		EURLabShmExactError::ChallengeInvalid);
	OtherRegion.Reset();
	TestEqual(TEXT("another owner cannot retire retained challenge"),
		Region->RetireChallenge(OwnerB, Challenge).Code,
		EURLabShmExactError::ChallengeInvalid);

	TestTrue(TEXT("exact owner explicitly retires challenge"),
		Region->RetireChallenge(OwnerA, Challenge).IsOk());
	TestFalse(TEXT("retirement removes creator file"),
		IFileManager::Get().FileExists(*ChallengePath));
	TestEqual(TEXT("retirement invalidates retained proof"),
		Region->VerifyChallenge(OwnerA, Challenge, Proof).Code,
		EURLabShmExactError::ChallengeInvalid);
	TestEqual(TEXT("retirement returns physical quota"),
		Store.GetStats().PhysicalBytes, 0ll);

	FURLabShmExactChallengePtr RaiiChallenge;
	if (!TestTrue(TEXT("second challenge creates for RAII cleanup"),
		Region->CreateChallenge(Store, OwnerA, RaiiChallenge).IsOk())) return false;
	const FString RaiiPath = FPaths::Combine(RegionDescriptor.DirectoryPath,
		RaiiChallenge->GetOffer().ChallengeName);
	TestTrue(TEXT("RAII challenge file exists"), IFileManager::Get().FileExists(*RaiiPath));
	RaiiChallenge.Reset();
	TestFalse(TEXT("last creator handle retires challenge file"),
		IFileManager::Get().FileExists(*RaiiPath));
	TestEqual(TEXT("RAII retirement returns physical quota"),
		Store.GetStats().PhysicalBytes, 0ll);
	{
		FURLabPayloadStore QuotaStore(TEXT("boot-challenge"), MakeLimits(40));
		FURLabPayloadOwner QuotaOwner;
		if (!OpenOwner(*this, QuotaStore, TEXT("quota-session"), QuotaOwner,
			TEXT("boot-challenge"))) return false;
		const TArray<uint8> ExistingBytes = {
			0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
		FURLabShmExactSealedPayloadPtr Existing;
		if (!TestTrue(TEXT("quota fixture retains prior physical bytes"),
			Region->CreateFromBytes(QuotaStore, QuotaOwner, ExistingBytes, Existing).IsOk()))
			return false;
		FURLabShmExactChallengePtr Rejected;
		TestEqual(TEXT("challenge accounts its complete fixed allocation"),
			Region->CreateChallenge(QuotaStore, QuotaOwner, Rejected).Code,
			EURLabShmExactError::QuotaExceeded);
		Region->Release(QuotaOwner, Existing);
		Existing.Reset();
	}

	OwnerA.Session->Deactivate();
	FURLabShmExactChallengePtr Inactive;
	TestEqual(TEXT("inactive exact owner cannot create challenge"),
		Region->CreateChallenge(Store, OwnerA, Inactive).Code,
		EURLabShmExactError::IdentityMismatch);
	const FString RegionPath = RegionDescriptor.DirectoryPath;
	Challenge.Reset();
	Region.Reset();
	TestFalse(TEXT("boot region is creator-cleaned"),
		IFileManager::Get().DirectoryExists(*RegionPath));
	return !HasAnyErrors();
#endif
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabShmExactPayloadLocalTest,
	"URLab.ShmExactPayload.LocalPublicationIsolationAndAccounting",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabShmExactPayloadLocalTest::RunTest(const FString&)
{
	using namespace URLabShmExactPayloadTestPrivate;
#if !PLATFORM_LINUX
	FURLabShmExactRegionPtr Unsupported;
	TestEqual(TEXT("non-Linux exact SHM is explicit"),
		FURLabShmExactRegion::Create(TEXT("boot-shm"), MakeBaseDirectory(), Unsupported).Code,
		EURLabShmExactError::Unsupported);
	return true;
#else
	const FString BaseDirectory = MakeBaseDirectory();
	FURLabPayloadStore Store(TEXT("boot-shm"), MakeLimits());
	FURLabPayloadOwner OwnerA;
	FURLabPayloadOwner OwnerB;
	if (!OpenOwner(*this, Store, TEXT("session-a"), OwnerA)
		|| !OpenOwner(*this, Store, TEXT("session-b"), OwnerB)) return false;

	FURLabShmExactRegionPtr Region;
	const FURLabShmExactStatus RegionStatus = FURLabShmExactRegion::Create(
		TEXT("boot-shm"), BaseDirectory, Region);
	if (!TestTrue(TEXT("private exact SHM region creates"), RegionStatus.IsOk()))
	{
		AddError(RegionStatus.Message);
		return false;
	}
	const FURLabShmExactRegionDescriptor RegionDescriptor = Region->GetDescriptor();
	TestFalse(TEXT("region identity is non-empty"), RegionDescriptor.RegionId.IsEmpty());
	TestTrue(TEXT("region generation is nonzero"), RegionDescriptor.Generation != 0);
	TestTrue(TEXT("region generation fits the positive wire int64 domain"),
		RegionDescriptor.Generation <= static_cast<uint64>(MAX_int64));
	struct stat DirectoryStat = {};
	FTCHARToUTF8 DirectoryUtf8(*RegionDescriptor.DirectoryPath);
	TestTrue(TEXT("region directory is stat-able"), ::stat(DirectoryUtf8.Get(), &DirectoryStat) == 0);
	TestTrue(TEXT("region directory is private"), S_ISDIR(DirectoryStat.st_mode)
		&& (DirectoryStat.st_mode & 077) == 0);

	const TArray<uint8> Payload = MakePayload();
	FURLabShmExactSealedPayloadPtr Sealed;
	const FURLabShmExactStatus PublishStatus = Region->CreateFromBytes(Store, OwnerA,
		Payload, Sealed);
	if (!TestTrue(TEXT("bytes publish into a new sealed file"), PublishStatus.IsOk()))
	{
		AddError(PublishStatus.Message);
		return false;
	}
	const FURLabShmExactSealedDescriptor Descriptor = Sealed->GetDescriptor();
	TestEqual(TEXT("published SHA identifies exact bytes"), Descriptor.Sha256,
		FURLabAssetCache::Sha256Hex(Payload));
	TestFalse(TEXT("published name has no slash"), Descriptor.ObjectName.Contains(TEXT("/")));
	const FString ObjectPath = FPaths::Combine(RegionDescriptor.DirectoryPath,
		Descriptor.ObjectName);
	struct stat ObjectStat = {};
	FTCHARToUTF8 ObjectUtf8(*ObjectPath);
	TestTrue(TEXT("published object is a regular file"), ::lstat(ObjectUtf8.Get(), &ObjectStat) == 0
		&& S_ISREG(ObjectStat.st_mode));
	TestTrue(TEXT("published object is read-only"),
		(ObjectStat.st_mode & (S_IWUSR | S_IWGRP | S_IWOTH)) == 0);
	TestEqual(TEXT("one published file is charged"), Store.GetStats().PhysicalBytes, 8ll);
	TestEqual(TEXT("one published object is charged"), Store.GetStats().PhysicalBlobCount, 1);

	FURLabShmExactCopiedBytesPtr Copy;
	TestTrue(TEXT("bounded descriptor range copies"),
		Region->ReadCopied(Store, OwnerA, Descriptor, 2, 4, Copy).IsOk());
	TestEqual(TEXT("copied range length"), Copy->Num(), 4ll);
	TestEqual(TEXT("copied first byte"), Copy->GetBytes()[0], static_cast<uint8>(0x30));
	TestEqual(TEXT("copied last byte"), Copy->GetBytes()[3], static_cast<uint8>(0x62));
	TestEqual(TEXT("owned copied bytes are separately charged"),
		Store.GetStats().PhysicalBytes, 12ll);

	FURLabShmExactCopiedBytesPtr WrongOwnerCopy;
	TestEqual(TEXT("another exact session cannot read descriptor"),
		Region->ReadCopied(Store, OwnerB, Descriptor, 0, 1, WrongOwnerCopy).Code,
		EURLabShmExactError::IdentityMismatch);
	TestEqual(TEXT("another exact session cannot release handle"),
		Region->Release(OwnerB, Sealed).Code, EURLabShmExactError::IdentityMismatch);
	FURLabShmExactSealedDescriptor Traversal = Descriptor;
	Traversal.ObjectName = TEXT("../obj-00000000000000000000000000000000.bin");
	TestEqual(TEXT("relative traversal descriptor rejects"),
		Region->ReadCopied(Store, OwnerA, Traversal, 0, 1, WrongOwnerCopy).Code,
		EURLabShmExactError::InvalidRegion);

	TestTrue(TEXT("release removes only future lookup"), Region->Release(OwnerA, Sealed).IsOk());
	TestFalse(TEXT("released path is gone"), IFileManager::Get().FileExists(*ObjectPath));
	TestEqual(TEXT("released descriptor no longer resolves"),
		Region->ReadCopied(Store, OwnerA, Descriptor, 0, 1, WrongOwnerCopy).Code,
		EURLabShmExactError::NotFound);
	FURLabShmExactCopiedBytesPtr RetainedCopy;
	TestTrue(TEXT("retained read-only handle survives unlink"),
		Region->ReadCopied(Store, OwnerA, Sealed, 0, Payload.Num(), RetainedCopy).IsOk());
	TestEqual(TEXT("retained copy remains exact"), RetainedCopy->GetBytes(), Payload);

	RetainedCopy.Reset();
	Copy.Reset();
	Sealed.Reset();
	TestEqual(TEXT("last retained owners release all quota"), Store.GetStats().PhysicalBytes, 0ll);
	const FString RegionPath = RegionDescriptor.DirectoryPath;
	Region.Reset();
	TestFalse(TEXT("creator cleans its private region"), IFileManager::Get().DirectoryExists(*RegionPath));
	IFileManager::Get().DeleteDirectory(*BaseDirectory, false, true);
	return true;
#endif
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabShmExactPayloadStagingTest,
	"URLab.ShmExactPayload.CrossProcessStagingSealAndNoFollow",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabShmExactPayloadStagingTest::RunTest(const FString&)
{
	using namespace URLabShmExactPayloadTestPrivate;
#if !PLATFORM_LINUX
	return true;
#else
	const FString BaseDirectory = MakeBaseDirectory();
	FURLabPayloadStore Store(TEXT("boot-shm"), MakeLimits());
	FURLabPayloadOwner Owner;
	if (!OpenOwner(*this, Store, TEXT("session-a"), Owner)) return false;
	FURLabShmExactRegionPtr Region;
	if (!FURLabShmExactRegion::Create(TEXT("boot-shm"), BaseDirectory, Region)) return false;

	const TArray<uint8> Payload = MakePayload();
	const FString Sha = FURLabAssetCache::Sha256Hex(Payload);
	FURLabShmExactStagingPtr Staging;
	TestTrue(TEXT("staging reserves capacity before client write"),
		Region->CreateStaging(Store, Owner, Payload.Num(), Sha, Staging).IsOk());
	TestEqual(TEXT("unpublished staging is charged"), Store.GetStats().PhysicalBytes, 8ll);
	const FString StagingPath = FPaths::Combine(Region->GetDescriptor().DirectoryPath,
		Staging->GetGrant().ObjectName);
	if (!TestTrue(TEXT("separate process writes staging grant"), WriteFromChild(StagingPath, Payload)))
		return false;

	FURLabShmExactSealedPayloadPtr Sealed;
	TestTrue(TEXT("staging copies into and hashes a new sealed inode"),
		Region->SealStaging(Store, Owner, Staging, Sealed).IsOk());
	TestFalse(TEXT("mutable staging path retires"), IFileManager::Get().FileExists(*StagingPath));
	TestEqual(TEXT("only sealed physical file remains charged"),
		Store.GetStats().PhysicalBytes, 8ll);
	TestEqual(TEXT("only sealed physical object remains charged"),
		Store.GetStats().PhysicalBlobCount, 1);

	FURLabShmExactStagingPtr RetryStaging;
	TestTrue(TEXT("second staging creates"),
		Region->CreateStaging(Store, Owner, Payload.Num(), Sha, RetryStaging).IsOk());
	const FString RetryPath = FPaths::Combine(Region->GetDescriptor().DirectoryPath,
		RetryStaging->GetGrant().ObjectName);
	TArray<uint8> Wrong = Payload;
	Wrong[0] ^= 0xff;
	TestTrue(TEXT("separate process can write wrong staged bytes"), WriteFromChild(RetryPath, Wrong));
	FURLabShmExactSealedPayloadPtr RetrySealed;
	TestEqual(TEXT("hash mismatch never publishes copied inode"),
		Region->SealStaging(Store, Owner, RetryStaging, RetrySealed).Code,
		EURLabShmExactError::IntegrityMismatch);
	TestEqual(TEXT("failed seal retains only staging and existing sealed bytes"),
		Store.GetStats().PhysicalBytes, 16ll);
	TestTrue(TEXT("caller can repair retained staging"), WriteFromChild(RetryPath, Payload));
	TestTrue(TEXT("repaired staging seals"),
		Region->SealStaging(Store, Owner, RetryStaging, RetrySealed).IsOk());

	FURLabShmExactStagingPtr Attacked;
	TestTrue(TEXT("attack staging creates"),
		Region->CreateStaging(Store, Owner, Payload.Num(), Sha, Attacked).IsOk());
	const FString AttackPath = FPaths::Combine(Region->GetDescriptor().DirectoryPath,
		Attacked->GetGrant().ObjectName);
	const FString OutsidePath = FPaths::Combine(BaseDirectory, TEXT("outside.bin"));
	FTCHARToUTF8 AttackUtf8(*AttackPath);
	FTCHARToUTF8 OutsideUtf8(*OutsidePath);
	const int OutsideFd = ::open(OutsideUtf8.Get(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
	TestTrue(TEXT("outside sentinel creates"), OutsideFd >= 0);
	if (OutsideFd >= 0)
	{
		const uint8 Sentinel = 0x5a;
		::write(OutsideFd, &Sentinel, 1);
		::close(OutsideFd);
	}
	TestTrue(TEXT("attacker can replace its granted path for test"), ::unlink(AttackUtf8.Get()) == 0
		&& ::symlink(OutsideUtf8.Get(), AttackUtf8.Get()) == 0);
	FURLabShmExactSealedPayloadPtr AttackSeal;
	TestEqual(TEXT("seal refuses symlink replacement"),
		Region->SealStaging(Store, Owner, Attacked, AttackSeal).Code,
		EURLabShmExactError::IdentityMismatch);
	struct stat OutsideStat = {};
	TestTrue(TEXT("outside target remains untouched"), ::stat(OutsideUtf8.Get(), &OutsideStat) == 0
		&& OutsideStat.st_size == 1);
	Attacked.Reset();
	TestTrue(TEXT("cleanup removes only test-created replacement link"), ::unlink(AttackUtf8.Get()) == 0);
	::unlink(OutsideUtf8.Get());

	RetryStaging.Reset();
	Staging.Reset();
	RetrySealed.Reset();
	Sealed.Reset();
	// The attack replaced the minted inode, so handle retirement cannot prove it
	// removed the charged physical object. Keep that reservation conservatively
	// until the owning region itself retires its object registry.
	TestEqual(TEXT("unverifiable replaced path stays charged until region retirement"),
		Store.GetStats().PhysicalBytes, 8ll);
	Region.Reset();
	TestEqual(TEXT("region retirement releases the conservative reservation"),
		Store.GetStats().PhysicalBytes, 0ll);
	IFileManager::Get().DeleteDirectory(*BaseDirectory, false, true);
	return true;
#endif
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabShmExactPayloadPeakAdmissionTest,
	"URLab.ShmExactPayload.SealReservesPeakBeforeCopy",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabShmExactPayloadPeakAdmissionTest::RunTest(const FString&)
{
	using namespace URLabShmExactPayloadTestPrivate;
#if !PLATFORM_LINUX
	return true;
#else
	const FString BaseDirectory = MakeBaseDirectory();
	FURLabPayloadStore Store(TEXT("boot-shm"), MakeLimits(15));
	FURLabPayloadOwner Owner;
	if (!OpenOwner(*this, Store, TEXT("session-a"), Owner)) return false;
	FURLabShmExactRegionPtr Region;
	if (!FURLabShmExactRegion::Create(TEXT("boot-shm"), BaseDirectory, Region)) return false;
	const TArray<uint8> Payload = MakePayload();
	FURLabShmExactStagingPtr Staging;
	TestTrue(TEXT("first physical file fits"), Region->CreateStaging(Store, Owner,
		Payload.Num(), FURLabAssetCache::Sha256Hex(Payload), Staging).IsOk());
	const FString Path = FPaths::Combine(Region->GetDescriptor().DirectoryPath,
		Staging->GetGrant().ObjectName);
	TestTrue(TEXT("client fills staging"), WriteFromChild(Path, Payload));
	FURLabShmExactSealedPayloadPtr Sealed;
	TestEqual(TEXT("second full file is rejected before copy allocation"),
		Region->SealStaging(Store, Owner, Staging, Sealed).Code,
		EURLabShmExactError::QuotaExceeded);
	TestEqual(TEXT("failed peak admission leaves original staging charged"),
		Store.GetStats().PhysicalBytes, 8ll);
	Staging.Reset();
	Region.Reset();
	IFileManager::Get().DeleteDirectory(*BaseDirectory, false, true);
	return true;
#endif
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabShmPayloadCatalogLifecycleTest,
	"URLab.PayloadStore.ShmCatalogLifecycleAndRetainedPins",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabShmPayloadCatalogLifecycleTest::RunTest(const FString&)
{
	using namespace URLabShmExactPayloadTestPrivate;
#if !PLATFORM_LINUX
	return true;
#else
	const FString BaseDirectory = MakeBaseDirectory();
	FURLabPayloadStore Store(TEXT("boot-shm"), MakeLimits());
	FURLabPayloadOwner OwnerA;
	FURLabPayloadOwner OwnerB;
	if (!OpenOwner(*this, Store, TEXT("session-a"), OwnerA)
		|| !OpenOwner(*this, Store, TEXT("session-b"), OwnerB)) return false;
	FURLabShmExactRegionPtr Region;
	if (!FURLabShmExactRegion::Create(TEXT("boot-shm"), BaseDirectory, Region)) return false;

	const TArray<uint8> Payload = MakePayload();
	const FString Sha = FURLabAssetCache::Sha256Hex(Payload);
	FURLabPayloadShmStagingGrant Grant;
	if (!TestTrue(TEXT("catalog mints SHM blob and staging together"),
		Store.BeginShmWrite(OwnerA, Region, Payload.Num(), Sha, Grant).IsOk())) return false;
	TestFalse(TEXT("catalog blob id is nonempty"), Grant.BlobId.IsEmpty());
	TestEqual(TEXT("grant uses exact boot region"), Grant.RegionId,
		Region->GetDescriptor().RegionId);
	TestEqual(TEXT("grant uses exact region generation"), Grant.RegionGeneration,
		Region->GetDescriptor().Generation);
	const FString StagingPath = FPaths::Combine(Region->GetDescriptor().DirectoryPath,
		Grant.ObjectName);
	TestTrue(TEXT("client fills catalog staging grant"), WriteFromChild(StagingPath, Payload));
	FURLabStoredPayloadDescriptor Descriptor;
	FURLabPayloadPin Pin;
	TestEqual(TEXT("cross-session commit rejects exact catalog blob"),
		Store.Commit(OwnerB, Grant.BlobId, Descriptor, Pin).Code,
		EURLabPayloadStoreError::IdentityMismatch);
	if (!TestTrue(TEXT("catalog commits SHM backing"),
		Store.Commit(OwnerA, Grant.BlobId, Descriptor, Pin).IsOk())) return false;
	TestEqual(TEXT("descriptor identifies SHM backing"), Descriptor.BackingKind,
		EURLabPayloadBackingKind::ShmSealedFile);
	TestEqual(TEXT("descriptor retains shared blob id"), Descriptor.BlobId, Grant.BlobId);
	TestEqual(TEXT("descriptor region id is exact"), Descriptor.RegionId, Grant.RegionId);
	TestEqual(TEXT("descriptor region generation is exact"), Descriptor.RegionGeneration,
		Grant.RegionGeneration);
	TestFalse(TEXT("sealed object differs from mutable staging"),
		Descriptor.ObjectName == Grant.ObjectName);
	TestFalse(TEXT("mutable staging is retired at commit"),
		IFileManager::Get().FileExists(*StagingPath));
	TConstArrayView<uint8> NetworkView;
	TestFalse(TEXT("SHM pin cannot masquerade as network bytes"),
		Pin->TryGetNetworkBytes(NetworkView));

	FURLabPayloadCopyPin CatalogCopy;
	TestTrue(TEXT("catalog copied read supports SHM backing"),
		Store.CopyRange(OwnerA, Grant.BlobId, 2, 4, CatalogCopy).IsOk());
	TestEqual(TEXT("catalog copied range is exact"), CatalogCopy->GetBytes(),
		TArray<uint8>({0x30, 0x40, 0x51, 0x62}));
	const FString SealedPath = FPaths::Combine(Region->GetDescriptor().DirectoryPath,
		Descriptor.ObjectName);
	TestTrue(TEXT("sealed catalog path exists before release"),
		IFileManager::Get().FileExists(*SealedPath));
	TestTrue(TEXT("catalog release succeeds"), Store.Release(OwnerA, Grant.BlobId).IsOk());
	TestFalse(TEXT("catalog release unlinks sealed path immediately"),
		IFileManager::Get().FileExists(*SealedPath));
	int32 Removed = -1;
	TestTrue(TEXT("session revoke succeeds after catalog release"),
		Store.RevokeSession(OwnerA, Removed).IsOk());
	TestEqual(TEXT("released catalog left no live revoke entry"), Removed, 0);
	FURLabPayloadCopyPin RetainedCopy;
	TestTrue(TEXT("accepted SHM pin copies after session deactivation"),
		Store.CopyPinnedRange(Pin, 0, Payload.Num(), RetainedCopy).IsOk());
	TestEqual(TEXT("retained SHM pin bytes remain exact"), RetainedCopy->GetBytes(), Payload);

	RetainedCopy.Reset();
	CatalogCopy.Reset();
	Pin.Reset();
	TestEqual(TEXT("last retained pin and copies return physical quota"),
		Store.GetStats().PhysicalBytes, 0ll);
	const FString RegionPath = Region->GetDescriptor().DirectoryPath;
	Region.Reset();
	TestFalse(TEXT("region cleans after catalog pins retire"),
		IFileManager::Get().DirectoryExists(*RegionPath));
	IFileManager::Get().DeleteDirectory(*BaseDirectory, false, true);
	return !HasAnyErrors();
#endif
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabShmProducedCatalogTest,
	"URLab.PayloadStore.ShmDirectProducerPublicationAndRetry",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabShmProducedCatalogTest::RunTest(const FString&)
{
	using namespace URLabShmExactPayloadTestPrivate;
#if !PLATFORM_LINUX
	return true;
#else
	const TArray<uint8> Payload = MakePayload();
	{
		FURLabPayloadStore PeakStore(TEXT("boot-shm"), MakeLimits(15));
		FURLabPayloadOwner PeakOwner;
		if (!OpenOwner(*this, PeakStore, TEXT("peak"), PeakOwner)) return false;
		const FString PeakDirectory = MakeBaseDirectory();
		FURLabShmExactRegionPtr PeakRegion;
		if (!FURLabShmExactRegion::Create(TEXT("boot-shm"), PeakDirectory, PeakRegion))
			return false;
		FURLabPayloadProducerPtr Rejected;
		TestEqual(TEXT("producer rejects when CPU plus sealed-file peak exceeds quota"),
			PeakStore.BeginShmProduce(PeakOwner, PeakRegion, Payload.Num(), Rejected).Code,
			EURLabPayloadStoreError::LimitExceeded);
		TestFalse(TEXT("failed peak admission returns no producer"), Rejected.IsValid());
		TestEqual(TEXT("failed second reservation rolls back the first"),
			PeakStore.GetStats().PhysicalBytes, 0ll);
		TestEqual(TEXT("failed peak admission creates no staging catalog entry"),
			PeakStore.GetStats().StagingCount, 0);
		PeakRegion.Reset();
		IFileManager::Get().DeleteDirectory(*PeakDirectory, false, true);
	}

	const FString BaseDirectory = MakeBaseDirectory();
	double Now = 0.0;
	int32 ClockCalls = 0;
	int32 InvalidClockCall = INDEX_NONE;
	FURLabPayloadStoreLimits Limits = MakeLimits();
	Limits.StagingTtlSeconds = 2.0;
	FURLabPayloadStore Store(TEXT("boot-shm"), Limits, [&]
	{
		++ClockCalls;
		return ClockCalls == InvalidClockCall ? -1.0 : Now;
	});
	FURLabPayloadOwner OwnerA;
	FURLabPayloadOwner OwnerB;
	if (!OpenOwner(*this, Store, TEXT("session-a"), OwnerA)
		|| !OpenOwner(*this, Store, TEXT("session-b"), OwnerB)) return false;
	FURLabShmExactRegionPtr Region;
	if (!FURLabShmExactRegion::Create(TEXT("boot-shm"), BaseDirectory, Region)) return false;

	FURLabPayloadProducerPtr Producer;
	if (!TestTrue(TEXT("SHM producer reserves CPU and future file before allocation"),
		Store.BeginShmProduce(OwnerA, Region, Payload.Num(), Producer).IsOk())) return false;
	TestEqual(TEXT("producer reports SHM backing"), Producer->GetBackingKind(),
		EURLabPayloadBackingKind::ShmSealedFile);
	TestEqual(TEXT("two complete physical allocations are admitted"),
		Store.GetStats().PhysicalBytes, 2ll * Payload.Num());
	TestEqual(TEXT("CPU and future file occupy two physical objects"),
		Store.GetStats().PhysicalBlobCount, 2);

	TArray<uint8> Produced;
	Produced.Reserve(Payload.Num());
	Produced.Append(Payload);
	FURLabStoredPayloadDescriptor Descriptor;
	FURLabPayloadPin Pin;
	TestEqual(TEXT("cross-session producer use rejects exactly"),
		Store.SealProduced(OwnerB, Producer, MoveTemp(Produced), Descriptor, Pin).Code,
		EURLabPayloadStoreError::IdentityMismatch);
	TestEqual(TEXT("cross-session rejection preserves caller bytes"), Produced, Payload);

	const FString RegionDirectory = Region->GetDescriptor().DirectoryPath;
	FTCHARToUTF8 RegionDirectoryUtf8(*RegionDirectory);
	if (!TestTrue(TEXT("fixture temporarily removes region write permission"),
		::chmod(RegionDirectoryUtf8.Get(), S_IRUSR | S_IXUSR) == 0)) return false;
	const FURLabPayloadStoreStatus BackingFailure = Store.SealProduced(
		OwnerA, Producer, MoveTemp(Produced), Descriptor, Pin);
	const bool bRestored = ::chmod(RegionDirectoryUtf8.Get(), S_IRWXU) == 0;
	if (!TestTrue(TEXT("fixture restores private region permission"), bRestored)) return false;
	TestEqual(TEXT("recoverable backing failure is explicit"), BackingFailure.Code,
		EURLabPayloadStoreError::BackingFailure);
	TestEqual(TEXT("backing failure preserves caller bytes"), Produced, Payload);
	TestEqual(TEXT("backing failure preserves both retry reservations"),
		Store.GetStats().PhysicalBytes, 2ll * Payload.Num());
	const int32 CallsBeforePostIoFailure = ClockCalls;
	InvalidClockCall = ClockCalls + 2;
	const FURLabPayloadStoreStatus ClockFailure = Store.SealProduced(
		OwnerA, Producer, MoveTemp(Produced), Descriptor, Pin);
	InvalidClockCall = INDEX_NONE;
	TestEqual(TEXT("post-I/O clock failure is explicit"), ClockFailure.Code,
		EURLabPayloadStoreError::InvalidClock);
	TestEqual(TEXT("clock fault occurred on finalization after initial admission"),
		ClockCalls - CallsBeforePostIoFailure, 2);
	TestEqual(TEXT("post-I/O clock failure preserves caller bytes"), Produced, Payload);
	TestEqual(TEXT("failed candidate retires before restoring retry admission"),
		Store.GetStats().PhysicalBytes, 2ll * Payload.Num());
	TArray<FString> FailedCandidateFiles;
	IFileManager::Get().FindFiles(FailedCandidateFiles,
		*FPaths::Combine(RegionDirectory, TEXT("obj-*.bin")), true, false);
	TestEqual(TEXT("retry admission is restored only after candidate path cleanup"),
		FailedCandidateFiles.Num(), 0);

	if (!TestTrue(TEXT("same admitted producer retries into existing catalog"),
		Store.SealProduced(OwnerA, Producer, MoveTemp(Produced), Descriptor, Pin).IsOk()))
		return false;
	TestEqual(TEXT("successful SHM publication consumes CPU array"), Produced.Num(), 0);
	TestEqual(TEXT("successful publication retains only sealed file bytes"),
		Store.GetStats().PhysicalBytes, static_cast<int64>(Payload.Num()));
	TestEqual(TEXT("produced descriptor selects SHM backing"), Descriptor.BackingKind,
		EURLabPayloadBackingKind::ShmSealedFile);
	TestEqual(TEXT("produced descriptor retains producer blob identity"),
		Descriptor.BlobId, Producer->GetBlobId());
	TestEqual(TEXT("produced descriptor hashes exact caller bytes"), Descriptor.Sha256,
		FURLabAssetCache::Sha256Hex(Payload));
	TConstArrayView<uint8> NetworkView;
	TestFalse(TEXT("produced SHM pin has no network array"),
		Pin->TryGetNetworkBytes(NetworkView));
	FURLabPayloadCopyPin Copy;
	if (!TestTrue(TEXT("produced SHM pin supports bounded catalog copy"),
		Store.CopyPinnedRange(Pin, 0, Payload.Num(), Copy).IsOk())) return false;
	TestEqual(TEXT("produced SHM catalog bytes are exact"), Copy->GetBytes(), Payload);
	Copy.Reset();
	TestTrue(TEXT("produced SHM catalog entry releases"),
		Store.Release(OwnerA, Descriptor.BlobId).IsOk());
	TestEqual(TEXT("accepted pin retains sealed bytes after catalog release"),
		Store.GetStats().PhysicalBytes, static_cast<int64>(Payload.Num()));
	Pin.Reset();
	TestEqual(TEXT("last produced pin returns file quota"),
		Store.GetStats().PhysicalBytes, 0ll);

	Now = 3.0;
	FURLabPayloadProducerPtr Expiring;
	if (!TestTrue(TEXT("expiring SHM producer admits both allocations"),
		Store.BeginShmProduce(OwnerA, Region, Payload.Num(), Expiring).IsOk())) return false;
	Now = 5.0;
	FURLabPayloadSweepResult Sweep;
	TestTrue(TEXT("SHM producer expiry sweep succeeds"), Store.SweepExpired(Sweep).IsOk());
	TestEqual(TEXT("expiry removes the producer catalog entry"), Sweep.StagingRemoved, 1);
	TestEqual(TEXT("caller-held expired producer retains both reservations"),
		Store.GetStats().PhysicalBytes, 2ll * Payload.Num());
	TArray<uint8> ExpiredBytes;
	ExpiredBytes.Reserve(Payload.Num());
	ExpiredBytes.Append(Payload);
	TestEqual(TEXT("expired SHM producer reports exact expiry"),
		Store.SealProduced(OwnerA, Expiring, MoveTemp(ExpiredBytes), Descriptor, Pin).Code,
		EURLabPayloadStoreError::Expired);
	TestEqual(TEXT("expired producer preserves caller bytes"), ExpiredBytes, Payload);
	Expiring.Reset();
	TestEqual(TEXT("last expired producer handle returns both reservations"),
		Store.GetStats().PhysicalBytes, 0ll);

	Producer.Reset();
	int32 RemovedA = 0;
	int32 RemovedB = 0;
	Store.RevokeSession(OwnerA, RemovedA);
	Store.RevokeSession(OwnerB, RemovedB);
	const FString RegionPath = Region->GetDescriptor().DirectoryPath;
	Region.Reset();
	TestFalse(TEXT("produced SHM region retires without leaked files"),
		IFileManager::Get().DirectoryExists(*RegionPath));
	IFileManager::Get().DeleteDirectory(*BaseDirectory, false, true);
	return !HasAnyErrors();
#endif
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabShmPayloadCatalogExpiryPeakTest,
	"URLab.PayloadStore.ShmCatalogExpiryRevokeAndPeakBudget",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabShmPayloadCatalogExpiryPeakTest::RunTest(const FString&)
{
	using namespace URLabShmExactPayloadTestPrivate;
#if !PLATFORM_LINUX
	return true;
#else
	const TArray<uint8> Payload = MakePayload();
	const FString Sha = FURLabAssetCache::Sha256Hex(Payload);
	{
		const FString BaseDirectory = MakeBaseDirectory();
		double Now = 0.0;
		FURLabPayloadStoreLimits Limits = MakeLimits(15);
		Limits.StagingTtlSeconds = 1.0;
		FURLabPayloadStore Store(TEXT("boot-shm"), Limits, [&Now] { return Now; });
		FURLabPayloadOwner Owner;
		if (!OpenOwner(*this, Store, TEXT("session-a"), Owner)) return false;
		FURLabShmExactRegionPtr Region;
		if (!FURLabShmExactRegion::Create(TEXT("boot-shm"), BaseDirectory, Region)) return false;
		FURLabPayloadShmStagingGrant Expiring;
		TestTrue(TEXT("expiring SHM catalog staging begins"),
			Store.BeginShmWrite(Owner, Region, Payload.Num(), Sha, Expiring).IsOk());
		const FString ExpiringPath = FPaths::Combine(Region->GetDescriptor().DirectoryPath,
			Expiring.ObjectName);
		Now = 1.0;
		FURLabPayloadSweepResult Sweep;
		TestTrue(TEXT("staging expiry sweep succeeds"), Store.SweepExpired(Sweep).IsOk());
		TestEqual(TEXT("staging expiry removes one catalog entry"), Sweep.StagingRemoved, 1);
		TestFalse(TEXT("staging expiry unlinks mutable file"),
			IFileManager::Get().FileExists(*ExpiringPath));
		TestEqual(TEXT("staging expiry returns physical quota"),
			Store.GetStats().PhysicalBytes, 0ll);

		Now = 2.0;
		FURLabPayloadShmStagingGrant Peak;
		TestTrue(TEXT("peak fixture staging begins"),
			Store.BeginShmWrite(Owner, Region, Payload.Num(), Sha, Peak).IsOk());
		const FString PeakPath = FPaths::Combine(Region->GetDescriptor().DirectoryPath,
			Peak.ObjectName);
		TestTrue(TEXT("peak fixture staging fills"), WriteFromChild(PeakPath, Payload));
		FURLabStoredPayloadDescriptor Descriptor;
		FURLabPayloadPin Pin;
		TestEqual(TEXT("catalog seal reserves second full file before copy"),
			Store.Commit(Owner, Peak.BlobId, Descriptor, Pin).Code,
			EURLabPayloadStoreError::LimitExceeded);
		TestTrue(TEXT("failed peak admission preserves writable staging"),
			IFileManager::Get().FileExists(*PeakPath));
		TestEqual(TEXT("failed peak admission retains only staging quota"),
			Store.GetStats().PhysicalBytes, 8ll);
		TestTrue(TEXT("failed peak staging remains releasable"),
			Store.Release(Owner, Peak.BlobId).IsOk());
		TestEqual(TEXT("peak staging release returns quota"),
			Store.GetStats().PhysicalBytes, 0ll);
		Region.Reset();
		IFileManager::Get().DeleteDirectory(*BaseDirectory, false, true);
	}

	{
		const FString BaseDirectory = MakeBaseDirectory();
		double Now = 10.0;
		FURLabPayloadStoreLimits Limits = MakeLimits();
		Limits.SealedTtlSeconds = 1.0;
		FURLabPayloadStore Store(TEXT("boot-shm"), Limits, [&Now] { return Now; });
		FURLabPayloadOwner OldOwner;
		if (!OpenOwner(*this, Store, TEXT("same-id"), OldOwner)) return false;
		FURLabShmExactRegionPtr Region;
		if (!FURLabShmExactRegion::Create(TEXT("boot-shm"), BaseDirectory, Region)) return false;
		FURLabPayloadShmStagingGrant Grant;
		Store.BeginShmWrite(OldOwner, Region, Payload.Num(), Sha, Grant);
		const FString StagingPath = FPaths::Combine(Region->GetDescriptor().DirectoryPath,
			Grant.ObjectName);
		if (!WriteFromChild(StagingPath, Payload)) return false;
		FURLabStoredPayloadDescriptor Descriptor;
		FURLabPayloadPin AcceptedPin;
		if (!Store.Commit(OldOwner, Grant.BlobId, Descriptor, AcceptedPin)) return false;
		const FString SealedPath = FPaths::Combine(Region->GetDescriptor().DirectoryPath,
			Descriptor.ObjectName);
		Now = 11.0;
		FURLabPayloadSweepResult Sweep;
		Store.SweepExpired(Sweep);
		TestEqual(TEXT("sealed expiry removes one catalog entry"), Sweep.SealedRemoved, 1);
		TestFalse(TEXT("sealed expiry unlinks catalog path"),
			IFileManager::Get().FileExists(*SealedPath));
		int32 Removed = -1;
		TestTrue(TEXT("expired session scope revokes"),
			Store.RevokeSession(OldOwner, Removed).IsOk());
		FURLabPayloadOwner NewOwner;
		TestTrue(TEXT("same textual session can reopen after revoke"),
			OpenOwner(*this, Store, TEXT("same-id"), NewOwner));
		FURLabPayloadShmStagingGrant Rejected;
		TestEqual(TEXT("stale owner cannot begin SHM catalog entry"),
			Store.BeginShmWrite(OldOwner, Region, Payload.Num(), Sha, Rejected).Code,
			EURLabPayloadStoreError::SessionRevoked);
		FURLabPayloadCopyPin RetainedCopy;
		TestTrue(TEXT("expired accepted pin copies after scope replacement"),
			Store.CopyPinnedRange(AcceptedPin, 0, Payload.Num(), RetainedCopy).IsOk());
		TestEqual(TEXT("expired retained pin bytes are exact"), RetainedCopy->GetBytes(), Payload);
		RetainedCopy.Reset();
		AcceptedPin.Reset();
		TestEqual(TEXT("expired retained backing returns quota"),
			Store.GetStats().PhysicalBytes, 0ll);
		int32 NewRemoved = 0;
		Store.RevokeSession(NewOwner, NewRemoved);
		Region.Reset();
		IFileManager::Get().DeleteDirectory(*BaseDirectory, false, true);
	}
	return !HasAnyErrors();
#endif
}

#endif
