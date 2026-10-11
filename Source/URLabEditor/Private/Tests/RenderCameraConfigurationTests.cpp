// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Render/RenderCameraComponent.h"
#include "Render/RenderCameraSubsystem.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRenderCameraEditorTickPolicyTest,
	"URLab.Render.CameraConfiguration.EditorWorldSchedulerTicks",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FRenderCameraEditorTickPolicyTest::RunTest(const FString& Parameters)
{
	URenderCameraSubsystem* Scheduler =
		NewObject<URenderCameraSubsystem>(GetTransientPackage());
	if (!TestNotNull(TEXT("camera scheduler"), Scheduler)) return false;
	TestTrue(TEXT("live editor worlds drive camera scheduling and readback harvesting"),
		Scheduler->IsTickableInEditor());
	TestTrue(TEXT("camera scheduler supports editor viewer worlds"),
		Scheduler->DoesSupportWorldType(EWorldType::Editor));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRenderCameraCheckedSizingTest,
	"URLab.Render.CameraConfiguration.CheckedSizing",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FRenderCameraCheckedSizingTest::RunTest(const FString& Parameters)
{
	auto CheckMode = [this](ERenderCameraMode Mode, int32 ExpectedElements,
		int64 ExpectedLogicalBytes, const TCHAR* Label)
	{
		int32 Elements = 0;
		int64 LogicalBytes = 0;
		int64 PhysicalBytes = 0;
		FString Error;
		const bool bSized = RenderCheckedCameraBufferSizing(Mode, FIntPoint(64, 32),
			Elements, LogicalBytes, PhysicalBytes, Error);
		TestTrue(FString::Printf(TEXT("%s sizing succeeds"), Label), bSized);
		TestEqual(FString::Printf(TEXT("%s element count"), Label), Elements,
			ExpectedElements);
		TestEqual(FString::Printf(TEXT("%s logical bytes"), Label), LogicalBytes,
			ExpectedLogicalBytes);
		TestTrue(FString::Printf(TEXT("%s physical covers logical"), Label),
			PhysicalBytes >= LogicalBytes);

		if (Mode == ERenderCameraMode::Real)
		{
			TArray<FColor> Values;
			Values.Reserve(Elements);
			Values.SetNumUninitialized(Elements);
			TestTrue(TEXT("colour allocation is within bound"),
				static_cast<int64>(Values.GetAllocatedSize()) <= PhysicalBytes);
		}
		else if (Mode == ERenderCameraMode::Depth)
		{
			TArray<float> Values;
			Values.Reserve(Elements);
			Values.SetNumUninitialized(Elements);
			TestTrue(TEXT("depth allocation is within bound"),
				static_cast<int64>(Values.GetAllocatedSize()) <= PhysicalBytes);
		}
		else
		{
			TArray<int32> Values;
			Values.Reserve(Elements);
			Values.SetNumUninitialized(Elements);
			TestTrue(TEXT("segmentation allocation is within bound"),
				static_cast<int64>(Values.GetAllocatedSize()) <= PhysicalBytes);
		}
	};

	constexpr int32 Pixels = 64 * 32;
	CheckMode(ERenderCameraMode::Real, Pixels, Pixels * sizeof(FColor), TEXT("colour"));
	CheckMode(ERenderCameraMode::Depth, Pixels, Pixels * sizeof(float), TEXT("depth"));
	CheckMode(ERenderCameraMode::SemanticSegmentation, Pixels * 2,
		Pixels * 2 * sizeof(int32), TEXT("segmentation"));

	int32 Elements = 7;
	int64 LogicalBytes = 7;
	int64 PhysicalBytes = 7;
	FString Error;
	TestFalse(TEXT("zero dimension rejected"), RenderCheckedCameraBufferSizing(
		ERenderCameraMode::Real, FIntPoint(0, 32), Elements, LogicalBytes,
		PhysicalBytes, Error));
	TestFalse(TEXT("pixel count exceeding int32 rejected"), RenderCheckedCameraBufferSizing(
		ERenderCameraMode::Depth, FIntPoint(MAX_int32, MAX_int32), Elements,
		LogicalBytes, PhysicalBytes, Error));
	TestFalse(TEXT("segmentation pair count exceeding int32 rejected"),
		RenderCheckedCameraBufferSizing(ERenderCameraMode::InstanceSegmentation,
			FIntPoint(MAX_int32 / 2 + 1, 1), Elements, LogicalBytes, PhysicalBytes, Error));
	TestFalse(TEXT("unknown mode rejected"), RenderCheckedCameraBufferSizing(
		static_cast<ERenderCameraMode>(255), FIntPoint(1, 1), Elements,
		LogicalBytes, PhysicalBytes, Error));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRenderCameraConfigurationSnapshotTest,
	"URLab.Render.CameraConfiguration.AtomicSnapshot",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FRenderCameraConfigurationSnapshotTest::RunTest(const FString& Parameters)
{
	URenderCameraComponent* Camera =
		NewObject<URenderCameraComponent>(GetTransientPackage());
	if (!TestNotNull(TEXT("camera"), Camera))
	{
		return false;
	}

	FRenderCameraDesc Desc;
	Desc.CanonicalName = TEXT("snapshot-a");
	Desc.Optics.Resolution = FIntPoint(64, 32);
	Desc.Capture.Mode = ERenderCameraMode::Real;
	FString Error;
	TestTrue(TEXT("first configuration accepted"), Camera->Configure(Desc, Error));
	FRenderCameraConfigurationSnapshot First;
	TestTrue(TEXT("first snapshot valid"), Camera->GetConfigurationSnapshot(First, Error));
	TestEqual(TEXT("first name"), First.CanonicalName, FString(TEXT("snapshot-a")));
	TestEqual(TEXT("first mode"), First.Mode, ERenderCameraMode::Real);
	TestEqual(TEXT("first resolution"), First.Resolution, FIntPoint(64, 32));
	TestEqual(TEXT("first logical bytes"), First.LogicalPixelBytes,
		static_cast<int64>(64 * 32 * sizeof(FColor)));
	Camera->CaptureMode = ERenderCameraMode::Depth;
	FRenderCameraConfigurationSnapshot EffectiveMode;
	TestTrue(TEXT("effective-mode snapshot valid"),
		Camera->GetConfigurationSnapshot(EffectiveMode, Error));
	TestEqual(TEXT("snapshot reads effective public mode"), EffectiveMode.Mode,
		ERenderCameraMode::Depth);
	TestEqual(TEXT("direct mode write does not forge a revision"),
		EffectiveMode.ConfigurationRevision, First.ConfigurationRevision);
	TestEqual(TEXT("effective-mode logical bytes"), EffectiveMode.LogicalPixelBytes,
		static_cast<int64>(64 * 32 * sizeof(float)));

	Desc.CanonicalName = TEXT("snapshot-b");
	Desc.Optics.Resolution = FIntPoint(32, 16);
	Desc.Capture.Mode = ERenderCameraMode::Depth;
	TestTrue(TEXT("replacement configuration accepted"), Camera->Configure(Desc, Error));
	FRenderCameraConfigurationSnapshot Second;
	TestTrue(TEXT("replacement snapshot valid"), Camera->GetConfigurationSnapshot(Second, Error));
	TestEqual(TEXT("replacement name"), Second.CanonicalName, FString(TEXT("snapshot-b")));
	TestEqual(TEXT("replacement mode"), Second.Mode, ERenderCameraMode::Depth);
	TestEqual(TEXT("replacement resolution"), Second.Resolution, FIntPoint(32, 16));
	TestTrue(TEXT("configuration revision advanced"),
		Second.ConfigurationRevision > First.ConfigurationRevision);
	TestEqual(TEXT("replacement logical bytes"), Second.LogicalPixelBytes,
		static_cast<int64>(32 * 16 * sizeof(float)));
	return true;
}

#endif
