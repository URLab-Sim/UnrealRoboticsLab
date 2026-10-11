// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "Misc/AutomationTest.h"
#include "Render/RenderCameraSubsystem.h"

#if WITH_DEV_AUTOMATION_TESTS
namespace
{
class FCountingHistoryLease final : public IRenderCameraCaptureLifetimeLease
{
public:
	explicit FCountingHistoryLease(int32& InReleased) : Released(InReleased) {}
	~FCountingHistoryLease() override { ++Released; }
private:
	int32& Released;
};

FURLabRenderExactCapturePlan Plan(const FGuid& Context, int64 PhysicalBytes)
{
	FURLabRenderExactCapturePlan Value; Value.RetentionContext = Context;
	Value.SceneRevision = TEXT("history-scene"); Value.AggregateLogicalPixelBytes = 4;
	Value.AggregatePhysicalArrayBytes = PhysicalBytes;
	FRenderCameraEffectiveConfiguration Camera; Camera.CanonicalName = TEXT("camera");
	Camera.Mode = ERenderCameraMode::Real; Camera.BaselineConfigurationRevision = 1;
	Camera.Optics.Resolution = FIntPoint(1, 1); Camera.LogicalPixelBytes = 4;
	Camera.PhysicalArrayBytes = PhysicalBytes; Value.Cameras.Add(Camera); return Value;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRenderExactHistoryEvictionAndLeaseTest,
	"URLab.RenderCapture.History.EvictionAndLeaseLifetime",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRenderExactHistoryEvictionAndLeaseTest::RunTest(const FString&)
{
	URenderCameraSubsystem* Scheduler = NewObject<URenderCameraSubsystem>();
	const FGuid Context = FGuid::NewGuid(); int32 Released = 0;
	TSharedPtr<const FRenderCameraFrame> HeldOldest;
	for (int32 Index = 0; Index < 65; ++Index)
	{
		const FURLabRenderExactCapturePlan CapturePlan = Plan(Context, 2ll * 1024ll * 1024ll);
		TSharedPtr<FRenderCameraFrame> Frame = MakeShared<FRenderCameraFrame>();
		Frame->Identity.SceneRevision = CapturePlan.SceneRevision;
		Frame->Identity.FrameId = Index + 1; Frame->Identity.ClockEpoch = 7;
		Frame->Identity.bHasSourceTime = true; Frame->Identity.SourceTime = Index;
		Frame->PixelLifetimeLease = MakeShared<FCountingHistoryLease, ESPMode::ThreadSafe>(Released);
		TArray<TSharedPtr<const FRenderCameraFrame>> Frames{Frame};
		if (Index == 0) HeldOldest = Frame;
		TestTrue(TEXT("in-budget complete group is retained"),
			Scheduler->SeedExactCaptureHistoryForTest(CapturePlan, Frame->Identity, Frames));
	}
	TestEqual(TEXT("oldest complete group evicted at cap"),
		Scheduler->GetExactCaptureHistoryGroupCountForTest(), 64);
	TestEqual(TEXT("shared references count physical bytes once"),
		Scheduler->GetExactCaptureHistoryPhysicalBytesForTest(), 128ll * 1024ll * 1024ll);
	TestEqual(TEXT("external result keeps evicted lease alive"), Released, 0);
	HeldOldest.Reset(); TestEqual(TEXT("evicted lease releases with final reference"), Released, 1);
	Scheduler->RetireExactCaptureHistory(Context);
	TestEqual(TEXT("context retirement releases retained leases"), Released, 65);

	const FURLabRenderExactCapturePlan Oversized = Plan(
		FGuid::NewGuid(), 128ll * 1024ll * 1024ll + 1);
	TSharedPtr<FRenderCameraFrame> Fresh = MakeShared<FRenderCameraFrame>();
	Fresh->Identity.SceneRevision = Oversized.SceneRevision;
	Fresh->PixelLifetimeLease = MakeShared<FCountingHistoryLease, ESPMode::ThreadSafe>(Released);
	TArray<TSharedPtr<const FRenderCameraFrame>> Frames{Fresh};
	TestFalse(TEXT("oversized group is skipped"),
		Scheduler->SeedExactCaptureHistoryForTest(Oversized, Fresh->Identity, Frames));
	TestTrue(TEXT("oversized fresh result remains valid"), Fresh.IsValid());
	Fresh.Reset(); Frames.Reset();
	TestEqual(TEXT("skipped result releases normally"), Released, 66);
	return true;
}
#endif
