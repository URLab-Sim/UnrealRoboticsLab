// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"

#include "Transport/LockedLatestFrame.h"
#include "Transport/PoseStreamFrameCodec.h"

#if PLATFORM_LINUX
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>
#endif

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
TArray<uint8> HexBytes(const TCHAR* Hex)
{
	TArray<uint8> Out;
	FString Text(Hex);
	Out.Reserve(Text.Len() / 2);
	for (int32 Index = 0; Index < Text.Len(); Index += 2)
	{
		Out.Add(static_cast<uint8>(FCString::Strtoi(*Text.Mid(Index, 2), nullptr, 16)));
	}
	return Out;
}

bool ContainsBytes(TConstArrayView<uint8> Haystack, TConstArrayView<uint8> Needle)
{
	for (int32 Offset = 0; Offset <= Haystack.Num() - Needle.Num(); ++Offset)
		if (FMemory::Memcmp(Haystack.GetData() + Offset, Needle.GetData(), Needle.Num()) == 0)
			return true;
	return false;
}

FURLabPoseStreamFrame MakeFrame(uint64 Sequence)
{
	FURLabPoseStreamFrame Frame;
	Frame.ProducerBootId = TEXT("boot");
	Frame.StreamId = TEXT("poses");
	Frame.StreamGeneration = 2;
	Frame.SourceRevision = TEXT("rev");
	Frame.Sequence = Sequence;
	Frame.FrameId = Sequence + 1;
	Frame.bHasSourceTime = true;
	Frame.SourceTime = 1.25;
	Frame.ClockEpoch = 5;
	const double Values[7] = {1, 2, 3, 1, 0, 0, 0};
	Frame.Payload.Append(reinterpret_cast<const uint8*>(Values), sizeof(Values));
	return Frame;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabPoseStreamCodecInteroperabilityTest,
	"URLab.Transport.PoseStream.CodecPythonInteroperability",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabPoseStreamCodecInteroperabilityTest::RunTest(const FString& Parameters)
{
	const TArray<uint8> PythonBytes = HexBytes(
		TEXT("8ab070726f64756365725f626f6f745f6964a4626f6f74a973747265616d5f6964a5706f736573b173747265616d5f67656e65726174696f6e02af736f757263655f7265766973696f6ea3726576a873657175656e636503a86672616d655f696404af6861735f736f757263655f74696d65c3ab736f757263655f74696d65cb3ff4000000000000ab636c6f636b5f65706f636805a77061796c6f6164c438000000000000f03f00000000000000400000000000000840000000000000f03f000000000000000000000000000000000000000000000000"));
	FURLabPoseStreamFrame Decoded;
	FString Error;
	TestTrue(TEXT("Python MessagePack decodes"),
		FURLabPoseStreamFrameCodec::Decode(PythonBytes, Decoded, Error, 56));
	TestEqual(TEXT("Python sequence preserved"), Decoded.Sequence, uint64(3));
	TArray<uint8> NativeBytes;
	TestTrue(TEXT("native frame encodes"),
		FURLabPoseStreamFrameCodec::Encode(Decoded, NativeBytes, Error));
	TestEqual(TEXT("canonical native encoding matches Python"), NativeBytes, PythonBytes);

	Decoded.bHasSourceTime = false;
	Decoded.SourceTime = 0;
	TestTrue(TEXT("source-time absence encodes"),
		FURLabPoseStreamFrameCodec::Encode(Decoded, NativeBytes, Error));
	FURLabPoseStreamFrame RoundTrip;
	TestTrue(TEXT("source-time absence decodes"),
		FURLabPoseStreamFrameCodec::Decode(NativeBytes, RoundTrip, Error, 56));
	TestFalse(TEXT("source-time remains absent"), RoundTrip.bHasSourceTime);

	FURLabPoseStreamFrame Dynamic = MakeFrame(4);
	FRenderFrame::FDeformable& Surface = Dynamic.Deformables.AddDefaulted_GetRef();
	Surface.Name = TEXT("cloth");
	Surface.Positions = {FVector(0, 0, 0), FVector(12, -34, 56), FVector(0, 100, 0)};
	Surface.Normals = {FVector(0.6, -0.8, 0), FVector::UpVector, FVector::UpVector};
	FRenderFrame::FDebugMesh& Mesh = Dynamic.DebugMeshes.AddDefaulted_GetRef();
	Mesh.Name = TEXT("contact"); Mesh.Color = FLinearColor(1, 0, 0, 1);
	Mesh.Positions = Surface.Positions; Mesh.Triangles = {0, 1, 2};
	TestTrue(TEXT("dynamic stream frame encodes"),
		FURLabPoseStreamFrameCodec::Encode(Dynamic, NativeBytes, Error));
	// Independent RH wire literals. Dense poses above are metres; dynamic vertices
	// are centimetres and normals remain unit vectors. Only the wire Y flips.
	TestTrue(TEXT("encoded dynamic position is RH centimetres"),
		ContainsBytes(NativeBytes, HexBytes(TEXT("000040410000084200006042"))));
	TestTrue(TEXT("encoded dynamic normal is RH and unscaled"),
		ContainsBytes(NativeBytes, HexBytes(TEXT("9a99193fcdcc4c3f00000000"))));
	FURLabPoseStreamFrame DynamicDecoded;
	TestTrue(TEXT("dynamic stream frame decodes"),
		FURLabPoseStreamFrameCodec::Decode(NativeBytes, DynamicDecoded, Error, 56));
	TestEqual(TEXT("deformable count survives"), DynamicDecoded.Deformables.Num(), 1);
	TestEqual(TEXT("debug mesh count survives"), DynamicDecoded.DebugMeshes.Num(), 1);
	if (DynamicDecoded.Deformables.Num() == 1)
	{
		const FRenderFrame::FDeformable& DecodedSurface = DynamicDecoded.Deformables[0];
		TestTrue(TEXT("asymmetric centimetre position survives RH wire conversion"),
			DecodedSurface.Positions[1].Equals(FVector(12, -34, 56), 1.e-5));
		TestTrue(TEXT("unit normal survives RH wire conversion without position scaling"),
			DecodedSurface.Normals[0].Equals(FVector(0.6, -0.8, 0), 1.e-5)
			&& FMath::IsNearlyEqual(DecodedSurface.Normals[0].Size(), 1.0, 1.e-5));
	}
	if (DynamicDecoded.DebugMeshes.Num() == 1)
		TestEqual(TEXT("debug triangle survives"), DynamicDecoded.DebugMeshes[0].Triangles[1], 1);
	Dynamic.Deformables[0].Name=FString::ChrN(1025,TEXT('x'));
	TestFalse(TEXT("oversized dynamic name is rejected before encoding"),
		FURLabPoseStreamFrameCodec::Encode(Dynamic, NativeBytes, Error));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabLockedLatestFrameTest,
	"URLab.Transport.PoseStream.LockedLatest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabLockedLatestFrameTest::RunTest(const FString& Parameters)
{
#if !PLATFORM_LINUX
	AddWarning(TEXT("Windows LockFileEx implementation has not been built or executed on Windows"));
	return true;
#else
	const FString Base = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("URLabTests"),
		TEXT("PoseStream-") + FGuid::NewGuid().ToString(EGuidFormats::Digits));
	FURLabShmExactRegionPtr Region;
	FURLabShmExactStatus Status = FURLabShmExactRegion::Create(TEXT("boot"), Base, Region);
	if (!TestTrue(TEXT("exact region creates"), Status.IsOk())) return false;
	FURLabShmLiveObjectPtr Creator;
	Status = Region->CreateLiveObject(48 + 4096, Creator);
	if (!TestTrue(TEXT("live object creates"), Status.IsOk())) return false;
	const FURLabShmLiveObjectDescriptor Descriptor = Creator->GetDescriptor();
	FURLabShmRegionAccessPtr Access;
	Status = FURLabShmRegionAccess::Open(Region->GetDescriptor().DirectoryPath,
		Region->GetDescriptor().BootId, Region->GetDescriptor().RegionId,
		Region->GetDescriptor().Generation, 4096, Access);
	if (!TestTrue(TEXT("consumer validates negotiated region"), Status.IsOk())) return false;
	FURLabShmLiveObjectPtr Consumer;
	Status = Access->OpenLiveObject(Descriptor, Consumer);
	if (!TestTrue(TEXT("live object exact-open succeeds"), Status.IsOk())) return false;
	FURLabShmLiveObjectDescriptor Stale = Descriptor;
	++Stale.RegionGeneration;
	FURLabShmLiveObjectPtr Rejected;
	TestFalse(TEXT("stale generation is rejected"), Access->OpenLiveObject(Stale, Rejected).IsOk());

	FString Error;
	TUniquePtr<FURLabLockedLatestFrameWriter> Writer =
		FURLabLockedLatestFrameWriter::Create(Creator, 4096, Error);
	TUniquePtr<FURLabLockedLatestFrameReader> Reader =
		FURLabLockedLatestFrameReader::Create(Consumer, 4096, Error);
	if (!TestTrue(TEXT("writer and reader create"), Writer && Reader)) return false;

	const FString FullPath = FPaths::Combine(Region->GetDescriptor().DirectoryPath,
		Descriptor.ObjectName);
	FTCHARToUTF8 Path(*FullPath);
	const int BlockFd = ::open(Path.Get(), O_RDWR | O_NOFOLLOW | O_CLOEXEC);
	if (!TestTrue(TEXT("test lock fd opens"), BlockFd >= 0)) return false;
	TestEqual(TEXT("test takes exclusive lock"), ::flock(BlockFd, LOCK_EX | LOCK_NB), 0);
	TArray<uint8> Old;
	TArray<uint8> New;
	FURLabPoseStreamFrameCodec::Encode(MakeFrame(1), Old, Error);
	FURLabPoseStreamFrameCodec::Encode(MakeFrame(2), New, Error);
	TestTrue(TEXT("old snapshot submits without waiting"), Writer->Submit(
		MakeShared<TArray<uint8>, ESPMode::ThreadSafe>(MoveTemp(Old)), Error));
	TestTrue(TEXT("new snapshot coalesces without waiting"), Writer->Submit(
		MakeShared<TArray<uint8>, ESPMode::ThreadSafe>(MoveTemp(New)), Error));
	FPlatformProcess::SleepNoStats(0.01f);
	::flock(BlockFd, LOCK_UN);
	::close(BlockFd);

	TArray<uint8> Read;
	EURLabLatestFrameRead Result = EURLabLatestFrameRead::NoFrame;
	for (int32 Attempt = 0; Attempt < 200 && Result == EURLabLatestFrameRead::NoFrame; ++Attempt)
	{
		FPlatformProcess::SleepNoStats(0.001f);
		Result = Reader->ReadLatest(Read, Error);
	}
	TestEqual(TEXT("reader obtains complete coalesced frame"), Result,
		EURLabLatestFrameRead::Frame);
	FURLabPoseStreamFrame Decoded;
	TestTrue(TEXT("coalesced bytes decode after copy"),
		FURLabPoseStreamFrameCodec::Decode(Read, Decoded, Error, 56));
	TestEqual(TEXT("newest snapshot wins"), Decoded.Sequence, uint64(2));
	Writer->Stop();
	TArray<uint8> AfterStop = {1};
	TestFalse(TEXT("stopped writer rejects submit"), Writer->Submit(
		MakeShared<TArray<uint8>, ESPMode::ThreadSafe>(MoveTemp(AfterStop)), Error));
	Writer.Reset(); Reader.Reset(); Consumer.Reset(); Access.Reset(); Creator.Reset(); Region.Reset();
	IFileManager::Get().DeleteDirectory(*Base, false, true);
	return true;
#endif
}

#endif
