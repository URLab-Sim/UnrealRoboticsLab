// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
#include "CoreMinimal.h"
#include "HAL/PlatformProcess.h"
#include "Misc/AutomationTest.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"
#include "Misc/ScopeLock.h"
#include "HAL/FileManager.h"

#include "Transport/LockedLatestFrame.h"
#include "Transport/PoseStreamFrameCodec.h"
#include "Transport/PoseStreamProducer.h"
#include "Transport/PoseStreamZmqReader.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabPoseStreamProducerLifecycleTest,
	"URLab.Transport.PoseStream.ProducerLifecycle",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabPoseStreamProducerLifecycleTest::RunTest(const FString& Parameters)
{
#if !PLATFORM_LINUX
	AddWarning(TEXT("native exact-region producer test requires Linux"));
	return true;
#else
	const FString Base = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("URLabTests"),
		TEXT("PoseProducer-") + FGuid::NewGuid().ToString(EGuidFormats::Digits));
	FURLabShmExactRegionPtr Region;
	if (!TestTrue(TEXT("region creates"),
		FURLabShmExactRegion::Create(TEXT("boot-region"), Base, Region).IsOk())) return false;
	FURLabPoseStreamProducerConfig Config;
	Config.BindEndpoint = TEXT("tcp://127.0.0.1:*");
	Config.MaxPayloadBytes = 4096;
	Config.ShmRegion = Region;
	FString Error;
	TUniquePtr<FURLabPoseStreamProducer> Producer =
		FURLabPoseStreamProducer::Create(Config, Error);
	if (!TestTrue(TEXT("producer starts"), Producer.IsValid())) { AddError(Error); return false; }
	FURLabPoseStreamDescription Description;
	TestFalse(TEXT("describe is unavailable before local commit"),
		Producer->DescribeForSession(true, Description, Error));
	FURLabPoseStreamDescriptor First;
	TestTrue(TEXT("source commits"), Producer->CommitSource(TEXT("scene-a"), First, Error));
	FURLabPoseStreamDescriptor Same;
	Producer->CommitSource(TEXT("scene-a"), Same, Error);
	TestEqual(TEXT("same revision is idempotent"), Same.StreamGeneration, First.StreamGeneration);
	FURLabPoseStreamDescriptor Replaced;
	Producer->CommitSource(TEXT("scene-b"), Replaced, Error);
	TestEqual(TEXT("replacement advances generation"), Replaced.StreamGeneration,
		First.StreamGeneration + 1);
	TestEqual(TEXT("stream identity survives replacement"), Replaced.StreamId, First.StreamId);

	TestTrue(TEXT("network-only session describes"),
		Producer->DescribeForSession(false, Description, Error));
	TestTrue(TEXT("ZMQ is ready"), Description.Zmq.IsSet());
	TestFalse(TEXT("unproven session cannot see SHM"), Description.Shm.IsSet());
	TestTrue(TEXT("SHM-proven session describes"),
		Producer->DescribeForSession(true, Description, Error));
	if (!TestTrue(TEXT("SHM is ready"), Description.Shm.IsSet())) return false;

	FURLabShmLiveObjectPtr Consumer;
	if (!TestTrue(TEXT("consumer opens exact live object"),
		Region->OpenLiveObject(Description.Shm->Object, Consumer).IsOk())) return false;
	TUniquePtr<FURLabLockedLatestFrameReader> ShmReader =
		FURLabLockedLatestFrameReader::Create(Consumer, 4096, Error);
	TArray<uint8> ZmqFrame;
	FCriticalSection ZmqMutex;
	TUniquePtr<FURLabPoseStreamZmqReader> ZmqReader = FURLabPoseStreamZmqReader::Create(
		Description.Zmq->Endpoint, Description.Zmq->Topic, 4096,
		[&](TArray<uint8>&& Bytes) { FScopeLock Lock(&ZmqMutex); ZmqFrame = MoveTemp(Bytes); }, Error);
	if (!TestTrue(TEXT("ZMQ reader starts"), ZmqReader.IsValid())) return false;
	FPlatformProcess::SleepNoStats(0.05f);
	const double Pose[7] = {1, 2, 3, 1, 0, 0, 0};
	for (uint64 FrameId = 1; FrameId <= 3; ++FrameId)
	{
		TestTrue(TEXT("publication is accepted"), Producer->Publish(
			FrameId, 0.25, true, 4, Pose, Error));
		FPlatformProcess::SleepNoStats(0.01f);
	}
	TArray<uint8> ShmFrame;
	EURLabLatestFrameRead ShmResult = EURLabLatestFrameRead::NoFrame;
	for (int32 Attempt = 0; Attempt < 200 && ShmResult == EURLabLatestFrameRead::NoFrame; ++Attempt)
	{
		ShmResult = ShmReader->ReadLatest(ShmFrame, Error);
		FPlatformProcess::SleepNoStats(0.001f);
	}
	for (int32 Attempt = 0; Attempt < 200; ++Attempt)
	{
		{ FScopeLock Lock(&ZmqMutex); if (!ZmqFrame.IsEmpty()) break; }
		FPlatformProcess::SleepNoStats(0.001f);
	}
	FURLabPoseStreamFrame Decoded;
	TestEqual(TEXT("locked SHM returns a complete frame"), ShmResult,
		EURLabLatestFrameRead::Frame);
	TestTrue(TEXT("SHM frame decodes"),
		FURLabPoseStreamFrameCodec::Decode(ShmFrame, Decoded, Error, sizeof(Pose)));
	TestEqual(TEXT("SHM frame uses replacement generation"), Decoded.StreamGeneration,
		Replaced.StreamGeneration);
	{ FScopeLock Lock(&ZmqMutex);
		TestTrue(TEXT("ZMQ frame coexists"), !ZmqFrame.IsEmpty()
			&& FURLabPoseStreamFrameCodec::Decode(ZmqFrame, Decoded, Error, sizeof(Pose))); }
	ZmqReader->Stop(); ShmReader.Reset(); Consumer.Reset(); Producer->Stop(); Producer.Reset();
	Region.Reset();
	IFileManager::Get().DeleteDirectory(*Base, false, true);
	return true;
#endif
}

#endif
