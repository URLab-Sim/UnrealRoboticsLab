// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "CoreMinimal.h"
#include "HAL/FileManager.h"
#include "Misc/AutomationTest.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"

#include "Bridge/URLabPayloadStore.h"
#include "Transport/ShmExactPayload.h"

#if PLATFORM_LINUX
#include <cstdlib>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

#if WITH_DEV_AUTOMATION_TESTS

namespace URLabShmChallengeLifetimeTestPrivate
{
FURLabPayloadStoreLimits MakeChallengeLifetimeLimits()
{
	FURLabPayloadStoreLimits Limits;
	Limits.MaxBlobBytes = 64;
	Limits.MaxChunkBytes = 64;
	Limits.MaxChunksPerBlob = 8;
	Limits.MaxSessionBytes = 64;
	Limits.MaxGlobalBytes = 64;
	Limits.MaxBlobCount = 4;
	Limits.MaxSessionBlobCount = 4;
	Limits.MaxSessionPhysicalObjectCount = 4;
	Limits.MaxActiveSessions = 1;
	Limits.StagingTtlSeconds = 300.0;
	Limits.SealedTtlSeconds = 300.0;
	Limits.ExpiredTombstoneTtlSeconds = 300.0;
	return Limits;
}

#if PLATFORM_LINUX
int32 CountOpenDescriptorsForInode(uint64 Device, uint64 Inode)
{
	int32 Count = 0;
	DIR* Directory = ::opendir("/proc/self/fd");
	if (!Directory) return -1;
	while (const dirent* Entry = ::readdir(Directory))
	{
		if (Entry->d_name[0] < '0' || Entry->d_name[0] > '9') continue;
		char* End = nullptr;
		const long Descriptor = ::strtol(Entry->d_name, &End, 10);
		if (!End || *End != '\0' || Descriptor < 0) continue;
		struct stat Stat = {};
		if (::fstat(static_cast<int>(Descriptor), &Stat) == 0
			&& static_cast<uint64>(Stat.st_dev) == Device
			&& static_cast<uint64>(Stat.st_ino) == Inode)
		{
			++Count;
		}
	}
	::closedir(Directory);
	return Count;
}
#endif
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabShmChallengeFailedRetirementLifetimeTest,
	"URLab.ShmExactPayload.ChallengeFailedRetirementClosesDescriptor",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabShmChallengeFailedRetirementLifetimeTest::RunTest(const FString&)
{
	using namespace URLabShmChallengeLifetimeTestPrivate;
#if !PLATFORM_LINUX
	return true;
#else
	const FString BootId = TEXT("boot-challenge-lifetime");
	const FString BaseDirectory = FPaths::Combine(FPaths::ProjectSavedDir(),
		TEXT("URLabTests"), TEXT("ShmChallengeLifetime-")
		+ FGuid::NewGuid().ToString(EGuidFormats::Digits));
	FURLabPayloadStore Store(BootId, MakeChallengeLifetimeLimits());
	FURLabPayloadSessionPtr Session;
	if (!TestTrue(TEXT("challenge session opens"),
		Store.OpenSession(BootId, TEXT("session-a"), Session).IsOk())) return false;
	const FURLabPayloadOwner Owner = FURLabPayloadOwner::FromSession(Session);

	FURLabShmExactRegionPtr Region;
	if (!TestTrue(TEXT("challenge region creates"),
		FURLabShmExactRegion::Create(BootId, BaseDirectory, Region).IsOk())) return false;
	const FString RegionPath = Region->GetDescriptor().DirectoryPath;
	FURLabShmExactChallengePtr Challenge;
	if (!TestTrue(TEXT("challenge creates"),
		Region->CreateChallenge(Store, Owner, Challenge).IsOk())) return false;
	const FString ChallengePath = FPaths::Combine(RegionPath,
		Challenge->GetOffer().ChallengeName);
	FTCHARToUTF8 ChallengeUtf8(*ChallengePath);
	struct stat ChallengeStat = {};
	if (!TestTrue(TEXT("challenge path identity is available"),
		::stat(ChallengeUtf8.Get(), &ChallengeStat) == 0)) return false;
	TestTrue(TEXT("challenge handle retains its minted descriptor"),
		CountOpenDescriptorsForInode(static_cast<uint64>(ChallengeStat.st_dev),
			static_cast<uint64>(ChallengeStat.st_ino)) >= 1);

	FTCHARToUTF8 RegionUtf8(*RegionPath);
	const bool bMadeReadOnly = ::chmod(RegionUtf8.Get(), S_IRUSR | S_IXUSR) == 0;
	TestTrue(TEXT("fixture removes directory write permission"), bMadeReadOnly);
	if (!bMadeReadOnly)
	{
		Challenge.Reset();
		Region.Reset();
		IFileManager::Get().DeleteDirectory(*BaseDirectory, false, true);
		return false;
	}

	Challenge.Reset();
	TestEqual(TEXT("failed retirement still closes the handle descriptor"),
		CountOpenDescriptorsForInode(static_cast<uint64>(ChallengeStat.st_dev),
			static_cast<uint64>(ChallengeStat.st_ino)), 0);
	TestTrue(TEXT("failed retirement retains the exact creator path"),
		IFileManager::Get().FileExists(*ChallengePath));
	TestEqual(TEXT("failed retirement retains physical charge in the region catalog"),
		Store.GetStats().PhysicalBytes, 32ll);

	TestTrue(TEXT("fixture restores directory cleanup permission"),
		::chmod(RegionUtf8.Get(), S_IRWXU) == 0);
	Region.Reset();
	TestFalse(TEXT("region cleanup retries the retained exact path"),
		IFileManager::Get().FileExists(*ChallengePath));
	TestEqual(TEXT("region cleanup returns retained physical charge"),
		Store.GetStats().PhysicalBytes, 0ll);
	TestFalse(TEXT("region cleanup removes the private directory"),
		IFileManager::Get().DirectoryExists(*RegionPath));
	IFileManager::Get().DeleteDirectory(*BaseDirectory, false, true);
	return !HasAnyErrors();
#endif
}

#endif
