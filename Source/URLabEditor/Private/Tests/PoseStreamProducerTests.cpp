// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
#include "CoreMinimal.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"
#include "Misc/ScopeLock.h"
#include "HAL/FileManager.h"

#include "Transport/LockedLatestFrame.h"
#include "Transport/PoseStreamFrameCodec.h"
#include "Transport/PoseStreamProducer.h"
#include "Transport/PoseStreamZmqReader.h"
#include "Wire/RequestEnvelope.h"

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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabPoseStreamGenerationBarrierTest,
	"URLab.Transport.PoseStream.GenerationBarrier",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabPoseStreamGenerationBarrierTest::RunTest(const FString& Parameters)
{
#if !PLATFORM_LINUX
	AddWarning(TEXT("native exact-region producer test requires Linux"));
	return true;
#else
	const FString Base = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("URLabTests"),
		TEXT("PoseBarrier-") + FGuid::NewGuid().ToString(EGuidFormats::Digits));
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
	FURLabPoseStreamDescriptor First;
	TestTrue(TEXT("source commits"), Producer->CommitSource(TEXT("scene-a"), First, Error));
	const double Pose[7] = {1, 2, 3, 1, 0, 0, 0};
	TestTrue(TEXT("first generation publishes"), Producer->Publish(1, 0.5, true, 0, Pose, Error));
	FURLabPoseStreamDescription Description;
	if (!TestTrue(TEXT("describes"), Producer->DescribeForSession(true, Description, Error))
		|| !TestTrue(TEXT("SHM is ready"), Description.Shm.IsSet())) return false;

	auto ReadFresh = [&](TArray<uint8>& OutBytes) -> EURLabLatestFrameRead
	{
		FURLabShmLiveObjectPtr Consumer;
		if (!Region->OpenLiveObject(Description.Shm->Object, Consumer).IsOk())
			return EURLabLatestFrameRead::Error;
		TUniquePtr<FURLabLockedLatestFrameReader> Reader =
			FURLabLockedLatestFrameReader::Create(Consumer, 4096, Error);
		if (!Reader) return EURLabLatestFrameRead::Error;
		EURLabLatestFrameRead Result = EURLabLatestFrameRead::NoFrame;
		for (int32 Attempt = 0; Attempt < 100 && Result == EURLabLatestFrameRead::NoFrame; ++Attempt)
		{
			Result = Reader->ReadLatest(OutBytes, Error);
			if (Result == EURLabLatestFrameRead::NoFrame) FPlatformProcess::SleepNoStats(0.001f);
		}
		return Result;
	};
	auto ReadZmqLate = [&](float WaitSeconds, TArray<uint8>& OutBytes) -> bool
	{
		TArray<uint8> Received;
		FCriticalSection Mutex;
		TUniquePtr<FURLabPoseStreamZmqReader> Reader = FURLabPoseStreamZmqReader::Create(
			Description.Zmq->Endpoint, Description.Zmq->Topic, 4096,
			[&](TArray<uint8>&& Bytes) { FScopeLock Lock(&Mutex); Received = MoveTemp(Bytes); }, Error);
		if (!Reader) return false;
		const double Deadline = FPlatformTime::Seconds() + WaitSeconds;
		bool bReceived = false;
		while (!bReceived && FPlatformTime::Seconds() < Deadline)
		{
			FPlatformProcess::SleepNoStats(0.005f);
			FScopeLock Lock(&Mutex);
			bReceived = !Received.IsEmpty();
			if (bReceived) OutBytes = MoveTemp(Received);
		}
		Reader->Stop();
		return bReceived;
	};

	TArray<uint8> Bytes;
	FURLabPoseStreamFrame Decoded;
	TestEqual(TEXT("fresh SHM reader sees the first generation"), ReadFresh(Bytes),
		EURLabLatestFrameRead::Frame);
	TestTrue(TEXT("late ZMQ subscriber receives the idle latest frame"), ReadZmqLate(2.5f, Bytes)
		&& FURLabPoseStreamFrameCodec::Decode(Bytes, Decoded, Error, sizeof(Pose))
		&& Decoded.StreamGeneration == First.StreamGeneration);

	FURLabPoseStreamDescriptor Second;
	TestTrue(TEXT("replacement commits"), Producer->CommitSource(TEXT("scene-b"), Second, Error));
	Bytes.Reset();
	TestEqual(TEXT("fresh SHM reader sees no stale generation after commit"), ReadFresh(Bytes),
		EURLabLatestFrameRead::NoFrame);
	TestFalse(TEXT("late ZMQ subscriber receives no stale generation after commit"),
		ReadZmqLate(1.5f, Bytes));

	TestTrue(TEXT("second generation publishes"), Producer->Publish(2, 1.0, true, 0, Pose, Error));
	TestEqual(TEXT("fresh SHM reader sees the second generation"), ReadFresh(Bytes),
		EURLabLatestFrameRead::Frame);
	TestTrue(TEXT("second generation decodes"),
		FURLabPoseStreamFrameCodec::Decode(Bytes, Decoded, Error, sizeof(Pose)));
	TestEqual(TEXT("SHM exposes the committed generation"), Decoded.StreamGeneration,
		Second.StreamGeneration);
	TestEqual(TEXT("sequence restarts for the new generation"), Decoded.Sequence, uint64(1));

	Producer->Stop();
	Producer.Reset();
	Region.Reset();
	IFileManager::Get().DeleteDirectory(*Base, false, true);
	return !HasAnyErrors();
#endif
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabPoseStreamCodecCompatibilityTest,
	"URLab.Transport.PoseStream.CodecCompatibility",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabPoseStreamCodecCompatibilityTest::RunTest(const FString& Parameters)
{
	using FWire = FURLabWireValue;
	TArray<uint8> Payload;
	Payload.SetNumZeroed(56);
	auto Frame = [&](FWire SourceTime, TOptional<FWire> Extra) -> TArray<uint8>
	{
		TArray<FString> Keys = {TEXT("producer_boot_id"), TEXT("stream_id"),
			TEXT("stream_generation"), TEXT("source_revision"), TEXT("sequence"),
			TEXT("frame_id"), TEXT("has_source_time"), TEXT("source_time"),
			TEXT("clock_epoch"), TEXT("payload")};
		TArray<FWire> Values = {FWire::MakeString(TEXT("boot")), FWire::MakeString(TEXT("stream")),
			FWire::MakeInt(1), FWire::MakeString(TEXT("rev")), FWire::MakeInt(1), FWire::MakeInt(0),
			FWire::MakeBool(true), MoveTemp(SourceTime), FWire::MakeInt(0), FWire::MakeBinary(Payload)};
		if (Extra.IsSet())
		{
			Keys.Add(TEXT("visualization"));
			Values.Add(MoveTemp(Extra.GetValue()));
		}
		TArray<uint8> Encoded;
		FURLabRequestEnvelopeCodec::EncodeValue(FWire::MakeMap(MoveTemp(Keys), MoveTemp(Values)), Encoded);
		return Encoded;
	};
	FURLabPoseStreamFrame Decoded;
	FString Error;
	TestTrue(TEXT("integer source_time decodes"),
		FURLabPoseStreamFrameCodec::Decode(Frame(FWire::MakeInt(3), {}), Decoded, Error));
	TestEqual(TEXT("integer source_time widens to float64"), Decoded.SourceTime, 3.0);
	TestTrue(TEXT("float source_time decodes"),
		FURLabPoseStreamFrameCodec::Decode(Frame(FWire::MakeFloat(0.25), {}), Decoded, Error));
	TestFalse(TEXT("string source_time is rejected"),
		FURLabPoseStreamFrameCodec::Decode(Frame(FWire::MakeString(TEXT("1")), {}), Decoded, Error));

	TArray<uint8> Positions;
	Positions.SetNumZeroed(36);
	auto Deformable = [&](const TCHAR* Name, bool bExtraKey) -> FWire
	{
		TArray<FString> Keys = {TEXT("name"), TEXT("vertex_positions_cm")};
		TArray<FWire> Values = {FWire::MakeString(Name), FWire::MakeBinary(Positions)};
		if (bExtraKey) { Keys.Add(TEXT("colour")); Values.Add(FWire::MakeInt(1)); }
		return FWire::MakeMap(MoveTemp(Keys), MoveTemp(Values));
	};
	auto Visualization = [&](TArray<FWire> Rows) -> FWire
	{
		return FWire::MakeMap({TEXT("deformables")}, {FWire::MakeArray(MoveTemp(Rows))});
	};
	TestTrue(TEXT("unique deformable names decode"), FURLabPoseStreamFrameCodec::Decode(
		Frame(FWire::MakeFloat(0.0), Visualization({Deformable(TEXT("a"), false),
			Deformable(TEXT("b"), false)})), Decoded, Error));
	TestFalse(TEXT("duplicate deformable names are rejected"), FURLabPoseStreamFrameCodec::Decode(
		Frame(FWire::MakeFloat(0.0), Visualization({Deformable(TEXT("a"), false),
			Deformable(TEXT("a"), false)})), Decoded, Error));
	TestFalse(TEXT("unknown deformable keys are rejected"), FURLabPoseStreamFrameCodec::Decode(
		Frame(FWire::MakeFloat(0.0), Visualization({Deformable(TEXT("a"), true)})), Decoded, Error));
	TestFalse(TEXT("unknown visualization keys are rejected"), FURLabPoseStreamFrameCodec::Decode(
		Frame(FWire::MakeFloat(0.0), FWire::MakeMap({TEXT("deformables"), TEXT("extra")},
			{FWire::MakeArray({Deformable(TEXT("a"), false)}), FWire::MakeInt(1)})), Decoded, Error));
	TestFalse(TEXT("non-float rgba is rejected"), FURLabPoseStreamFrameCodec::Decode(
		Frame(FWire::MakeFloat(0.0), FWire::MakeMap({TEXT("debug_meshes")}, {FWire::MakeArray({
			FWire::MakeMap({TEXT("name"), TEXT("rgba"), TEXT("positions_cm"), TEXT("triangles")},
				{FWire::MakeString(TEXT("m")), FWire::MakeArray({FWire::MakeString(TEXT("r")),
					FWire::MakeFloat(0), FWire::MakeFloat(0), FWire::MakeFloat(1)}),
				FWire::MakeBinary(Positions), FWire::MakeBinary(TArray<uint8>({0, 0, 0, 0, 1, 0, 0, 0,
					2, 0, 0, 0}))})})})), Decoded, Error));

	FURLabPoseStreamFrame Duplicate;
	Duplicate.ProducerBootId = TEXT("boot");
	Duplicate.StreamId = TEXT("stream");
	Duplicate.StreamGeneration = 1;
	Duplicate.SourceRevision = TEXT("rev");
	Duplicate.Sequence = 1;
	Duplicate.Payload = Payload;
	FRenderFrame::FDeformable Mesh;
	Mesh.Name = TEXT("cloth");
	Mesh.Positions = {FVector::ZeroVector, FVector::ZeroVector, FVector::ZeroVector};
	Duplicate.Deformables = {Mesh, Mesh};
	TArray<uint8> Encoded;
	TestFalse(TEXT("encoder rejects duplicate dynamic mesh names"),
		FURLabPoseStreamFrameCodec::Encode(Duplicate, Encoded, Error));
	return !HasAnyErrors();
}

#endif
