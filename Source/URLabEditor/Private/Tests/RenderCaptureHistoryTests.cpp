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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRenderExactHistoryNearestSelectionTest,
	"URLab.RenderCapture.History.NearestSelection",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRenderExactHistoryNearestSelectionTest::RunTest(const FString&)
{
	URenderCameraSubsystem* Scheduler = NewObject<URenderCameraSubsystem>();
	const FGuid Context = FGuid::NewGuid();
	FURLabRenderExactCapturePlan CapturePlan = Plan(Context, 4);
	CapturePlan.GeometryConfigurationId = TEXT("geometry");
	auto Seed = [&](uint64 FrameId, double SourceTime)
	{
		TSharedPtr<FRenderCameraFrame> Frame = MakeShared<FRenderCameraFrame>();
		Frame->Identity.SceneRevision = CapturePlan.SceneRevision;
		Frame->Identity.FrameId = FrameId; Frame->Identity.ClockEpoch = 3;
		Frame->Identity.bHasSourceTime = true; Frame->Identity.SourceTime = SourceTime;
		TArray<TSharedPtr<const FRenderCameraFrame>> Frames{Frame};
		Scheduler->SeedExactCaptureHistoryForTest(CapturePlan, Frame->Identity, Frames);
	};
	Seed(1, 1.0);
	Seed(2, 0.5);
	Seed(3, 2.0);
	Seed(4, 0.5);

	FURLabRenderHistoryQuery Query;
	Query.RetentionContext = Context; Query.ExpectedSceneRevision = CapturePlan.SceneRevision;
	Query.GeometryConfigurationId = CapturePlan.GeometryConfigurationId; Query.ClockEpoch = 3;
	Query.Cameras.Add({TEXT("camera"), RenderCameraConfigurationId(CapturePlan.Cameras[0])});
	auto Resolve = [&](double Target, FRenderFrameIdentity& OutIdentity) -> bool
	{
		Query.TargetSourceTime = Target;
		FURLabRenderExactCapturePlan Resolved; FString Error;
		return Scheduler->ResolveExactCaptureHistory(Query, Resolved, OutIdentity, Error);
	};
	FRenderFrameIdentity Identity;
	TestTrue(TEXT("target between captures resolves"), Resolve(1.2, Identity));
	TestEqual(TEXT("nearest earlier capture wins over a newer insertion"), Identity.FrameId, uint64(1));
	TestTrue(TEXT("exact target resolves"), Resolve(2.0, Identity));
	TestEqual(TEXT("exact source time is selected"), Identity.FrameId, uint64(3));
	TestTrue(TEXT("tied source time resolves"), Resolve(0.7, Identity));
	TestEqual(TEXT("ties prefer the newest insertion"), Identity.FrameId, uint64(4));
	TestFalse(TEXT("target before every capture misses"), Resolve(0.1, Identity));

	Query.TargetSourceTime = 1.2;
	FURLabRenderCaptureJobAdmission Admission = Scheduler->QueryExactCaptureHistory(Query);
	TestTrue(TEXT("query admits the nearest group"), Admission.bAccepted);
	const FURLabRenderCaptureJobSnapshot Snapshot = Admission.bAccepted
		? Admission.Job.Poll() : FURLabRenderCaptureJobSnapshot();
	if (Snapshot.Frames.Num() == 1 && Snapshot.Frames[0].IsValid())
	{
		TestEqual(TEXT("query returns the same group as resolve"),
			Snapshot.Frames[0]->Identity.FrameId, uint64(1));
	}
	else
	{
		AddError(TEXT("query did not return the selected frame"));
	}

	Query.ClockEpoch = 0;
	TestFalse(TEXT("foreign clock epoch misses"), Resolve(1.2, Identity));
	Query.ClockEpoch = 3;
	Query.Cameras[0].ConfigurationId = TEXT("other");
	TestFalse(TEXT("foreign camera configuration misses"), Resolve(1.2, Identity));
	Scheduler->RetireExactCaptureHistory(Context);
	return !HasAnyErrors();
}
#endif
