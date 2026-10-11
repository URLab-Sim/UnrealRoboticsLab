// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "CoreMinimal.h"
#include "Editor.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "HAL/PlatformProcess.h"
#include "Misc/App.h"
#include "Misc/AutomationTest.h"
#include "RenderingThread.h"

#include "Render/RenderCameraComponent.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabRenderCameraOverrideNormalizationTest,
	"URLab.Render.Camera.OverrideNormalizationAndRestore",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabRenderCameraOverrideNormalizationTest::RunTest(const FString& Parameters)
{
	URenderCameraComponent* Camera = NewObject<URenderCameraComponent>();
	FRenderCameraDesc Desc;
	Desc.CanonicalName = TEXT("override-camera");
	Desc.Optics.Resolution = FIntPoint(640, 480);
	Desc.Optics.bHasIntrinsics = true;
	Desc.Optics.FocalPixelsX = 400.0f;
	Desc.Optics.FocalPixelsY = 420.0f;
	Desc.Optics.PrincipalPixelsX = 320.0f;
	Desc.Optics.PrincipalPixelsY = 240.0f;
	FString Error;
	TestTrue(TEXT("baseline configures"), Camera->Configure(Desc, Error));
	FRenderCameraCaptureOverride Override;
	Override.Mode = ERenderCameraMode::Depth;
	Override.Resolution = FIntPoint(320, 240);
	Override.DepthNearCm = 5.0f;
	Override.DepthFarCm = 500.0f;
	FRenderCameraEffectiveConfiguration Effective;
	TestTrue(TEXT("override normalizes"),
		Camera->ResolveCaptureConfiguration(Override, Effective, Error));
	TestEqual(TEXT("x intrinsics scale"), Effective.Optics.FocalPixelsX, 200.0f);
	TestEqual(TEXT("y intrinsics scale"), Effective.Optics.FocalPixelsY, 210.0f);
	FRenderCameraOverrideToken Token;
	TestTrue(TEXT("override begins"), Camera->BeginCaptureOverride(Effective, Token, Error));
	TestEqual(TEXT("runtime resolution overridden"), Camera->CaptureResolution(), FIntPoint(320, 240));
	FRenderCameraConfigurationSnapshot Baseline;
	TestTrue(TEXT("baseline remains describable"), Camera->GetConfigurationSnapshot(Baseline, Error));
	TestEqual(TEXT("describe remains baseline"), Baseline.Resolution, FIntPoint(640, 480));
	TestTrue(TEXT("exact restore succeeds"), Camera->RestoreCaptureOverride(Token, Error));
	TestEqual(TEXT("runtime baseline restored"), Camera->CaptureResolution(), FIntPoint(640, 480));
	TestFalse(TEXT("double restore rejects"), Camera->RestoreCaptureOverride(Token, Error));
	return true;
}

namespace URLabRenderCameraReceiptTestPrivate
{
struct FCountingCaptureLease final : IRenderCameraCaptureLifetimeLease
{
	explicit FCountingCaptureLease(
		TSharedRef<std::atomic<int32>, ESPMode::ThreadSafe> InLiveCount)
		: LiveCount(MoveTemp(InLiveCount))
	{
		LiveCount->fetch_add(1, std::memory_order_relaxed);
	}

	virtual ~FCountingCaptureLease() override
	{
		LiveCount->fetch_sub(1, std::memory_order_relaxed);
	}

	TSharedRef<std::atomic<int32>, ESPMode::ThreadSafe> LiveCount;
};

FRenderFrameIdentity ReceiptIdentity(uint64 FrameId)
{
	FRenderFrameIdentity Identity;
	Identity.SceneRevision = TEXT("camera-receipt-scene");
	Identity.FrameId = FrameId;
	Identity.SourceTime = static_cast<double>(FrameId) * 0.01;
	Identity.ClockEpoch = 9;
	Identity.bHasSourceTime = true;
	return Identity;
}

struct FReceiptCameraHost
{
	UWorld* World = nullptr;
	AActor* Actor = nullptr;
	URenderCameraComponent* Camera = nullptr;

	bool Init()
	{
		World = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr;
		if (!World) return false;
		Actor = World->SpawnActor<AActor>();
		if (!Actor) return false;
		USceneComponent* Root = NewObject<USceneComponent>(Actor);
		Actor->SetRootComponent(Root);
		Root->RegisterComponent();
		Camera = NewObject<URenderCameraComponent>(Actor);
		FRenderCameraDesc Desc;
		Desc.CanonicalName = TEXT("receipt-camera");
		Desc.Optics.Resolution = FIntPoint(64, 32);
		Desc.Capture.Mode = ERenderCameraMode::Real;
		FString Error;
		if (!Camera || !Camera->Configure(Desc, Error)) return false;
		Camera->SetupAttachment(Root);
		Camera->RegisterComponent();
		Camera->SetSceneVisibility(true);
		Camera->SetAppliedFrameIdentity(ReceiptIdentity(10));
		Camera->SetStreamingEnabled(true);
		return true;
	}

	void Cleanup()
	{
		if (Actor) Actor->Destroy();
		Camera = nullptr;
		Actor = nullptr;
		World = nullptr;
	}

	~FReceiptCameraHost() { Cleanup(); }
};

FRenderCameraCaptureIssueResult IssueReady(
	URenderCameraComponent& Camera, FRenderCameraCaptureLifetimeLeasePtr Lease)
{
	FRenderCameraCaptureIssueResult Result =
		Camera.TryIssueExactCapture(Lease);
	for (int32 Attempt = 0; Attempt < 4 && Result.IsRetryable(); ++Attempt)
	{
		FlushRenderingCommands();
		Result = Camera.TryIssueExactCapture(Lease);
	}
	return Result;
}

FRenderCameraCaptureCompletion HarvestReceipt(
	URenderCameraComponent& Camera, const FRenderCameraCaptureReceipt& Receipt)
{
	FlushRenderingCommands();
	Camera.WaitAndHarvestReadbacks(1.0);
	FlushRenderingCommands();
	Camera.HarvestCompletedReadbacks();
	return Receipt.Poll();
}

class FWaitForDetachedReceiptDrain final : public IAutomationLatentCommand
{
public:
	FWaitForDetachedReceiptDrain(FAutomationTestBase& InTest,
		FRenderCameraCaptureReceipt InReceipt,
		TSharedRef<std::atomic<int32>, ESPMode::ThreadSafe> InLiveLeases,
		TSharedRef<std::atomic<bool>, ESPMode::ThreadSafe> InGateExited,
		FEvent* InRenderGate, double InDeadlineSeconds)
		: Test(InTest)
		, Receipt(MoveTemp(InReceipt))
		, LiveLeases(MoveTemp(InLiveLeases))
		, GateExited(MoveTemp(InGateExited))
		, RenderGate(InRenderGate)
		, DeadlineSeconds(InDeadlineSeconds)
	{
	}

	virtual bool Update() override
	{
		const bool bDrained = Receipt.IsGpuWriterDrained();
		const int32 LeaseCount = LiveLeases->load(std::memory_order_relaxed);
		if (bDrained && LeaseCount == 0)
		{
			const FRenderCameraCaptureCompletion Completion = Receipt.Poll();
			Test.TestEqual(TEXT("detached completion remains teardown after camera destruction"),
				Completion.Outcome, ERenderCameraCaptureOutcome::Teardown);
			Test.TestTrue(TEXT("detached completion reports its GPU writer drained"),
				Completion.bGpuWriterDrained);
			ReleaseGate();
			return true;
		}
		if (FPlatformTime::Seconds() < DeadlineSeconds)
		{
			return false;
		}
		Test.AddError(FString::Printf(
			TEXT("detached camera writer did not drain before deadline (drained=%d leases=%d)"),
			bDrained ? 1 : 0, LeaseCount));
		ReleaseGate();
		return true;
	}

	virtual ~FWaitForDetachedReceiptDrain() override
	{
		ReleaseGate();
	}

private:
	void ReleaseGate()
	{
		if (RenderGate)
		{
			// A failed test must not recycle the event while the queued render
			// command could still hold its raw pointer. Successful GPU drain proves
			// this is true; keep the explicit flag for the timeout path.
			if (GateExited->load(std::memory_order_acquire))
			{
				FPlatformProcess::ReturnSynchEventToPool(RenderGate);
			}
			RenderGate = nullptr;
		}
	}

	FAutomationTestBase& Test;
	FRenderCameraCaptureReceipt Receipt;
	TSharedRef<std::atomic<int32>, ESPMode::ThreadSafe> LiveLeases;
	TSharedRef<std::atomic<bool>, ESPMode::ThreadSafe> GateExited;
	FEvent* RenderGate = nullptr;
	double DeadlineSeconds = 0.0;
};
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabRenderCameraExactReceiptTest,
	"URLab.CameraReceipt.ExactCompletionCancellationAndConfiguration",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FURLabRenderCameraExactReceiptTest::RunTest(const FString&)
{
	using namespace URLabRenderCameraReceiptTestPrivate;
	FReceiptCameraHost Host;
	if (!TestTrue(TEXT("receipt camera host creates"), Host.Init())) return false;
	if (!FApp::CanEverRender())
	{
		AddInfo(TEXT("No RHI: exact GPU receipt completion requires an offscreen render run"));
		return true;
	}
	FlushRenderingCommands();

	auto LiveLeases = MakeShared<std::atomic<int32>, ESPMode::ThreadSafe>(0);
	auto NewLease = [&]() -> FRenderCameraCaptureLifetimeLeasePtr
	{
		return MakeShared<FCountingCaptureLease, ESPMode::ThreadSafe>(LiveLeases);
	};

	// Hold the render thread ahead of the accepted capture so cancellation is
	// deterministically terminal while its GPU writer is still outstanding.
	FEvent* RenderGate = FPlatformProcess::GetSynchEventFromPool(true);
	ENQUEUE_RENDER_COMMAND(URLabCameraReceiptHoldWriter)
	([RenderGate](FRHICommandListImmediate&) { RenderGate->Wait(5000); });
	FRenderCameraCaptureIssueResult CancelIssue =
		Host.Camera->TryIssueExactCapture(NewLease());
	if (!TestTrue(TEXT("first exact capture is accepted"), CancelIssue.IsAccepted()))
	{
		RenderGate->Trigger();
		FlushRenderingCommands();
		FPlatformProcess::ReturnSynchEventToPool(RenderGate);
		return false;
	}
	FRenderCameraCaptureReceipt CancelReceipt = MoveTemp(CancelIssue.Receipt);
	TestTrue(TEXT("accepted receipt is complete"), CancelReceipt.IsValid());
	TestEqual(TEXT("receipt snapshots canonical camera"), CancelReceipt.CameraName,
		FString(TEXT("receipt-camera")));
	TestEqual(TEXT("receipt snapshots capture mode"), CancelReceipt.Mode,
		ERenderCameraMode::Real);
	TestEqual(TEXT("receipt snapshots width"), CancelReceipt.Resolution.X, 64);
	TestEqual(TEXT("receipt snapshots height"), CancelReceipt.Resolution.Y, 32);
	TestEqual(TEXT("receipt snapshots full source frame"), CancelReceipt.Identity.FrameId,
		static_cast<uint64>(10));
	FRenderCameraCaptureCompletion Cancelled = CancelReceipt.Poll();
	TestEqual(TEXT("accepted receipt begins pending"), Cancelled.Outcome,
		ERenderCameraCaptureOutcome::Pending);
	TestFalse(TEXT("accepted writer has not drained before harvest"),
		Cancelled.bGpuWriterDrained);
	TestFalse(TEXT("atomic receipt drain query agrees before harvest"),
		CancelReceipt.IsGpuWriterDrained());
	TestTrue(TEXT("receipt cancellation becomes terminal"), CancelReceipt.Cancel());
	Cancelled = CancelReceipt.Poll();
	TestEqual(TEXT("cancellation has an exact terminal outcome"), Cancelled.Outcome,
		ERenderCameraCaptureOutcome::Cancelled);
	TestFalse(TEXT("logical cancellation does not imply writer drain"),
		Cancelled.bGpuWriterDrained);
	RenderGate->Trigger();
	FlushRenderingCommands();
	FPlatformProcess::ReturnSynchEventToPool(RenderGate);
	Cancelled = HarvestReceipt(*Host.Camera, CancelReceipt);
	TestEqual(TEXT("harvest cannot overwrite cancellation"), Cancelled.Outcome,
		ERenderCameraCaptureOutcome::Cancelled);
	TestTrue(TEXT("cancelled GPU writer eventually drains"), Cancelled.bGpuWriterDrained);
	TestTrue(TEXT("atomic receipt drain query survives terminal cancellation"),
		CancelReceipt.IsGpuWriterDrained());
	TestFalse(TEXT("cancelled receipt exposes no pixels"), Cancelled.Frame.IsValid());
	TestEqual(TEXT("cancelled writer releases its capacity lease"),
		LiveLeases->load(std::memory_order_relaxed), 0);

	Host.Camera->SetAppliedFrameIdentity(ReceiptIdentity(11));
	FRenderCameraCaptureIssueResult ChangedIssue = IssueReady(*Host.Camera, NewLease());
	if (!TestTrue(TEXT("configuration fixture capture is accepted"),
		ChangedIssue.IsAccepted())) return false;
	FRenderCameraCaptureReceipt ChangedReceipt = MoveTemp(ChangedIssue.Receipt);
	Host.Camera->SetCaptureRate(false, 17.0f);
	FRenderCameraCaptureCompletion Changed = ChangedReceipt.Poll();
	TestEqual(TEXT("effective mutation invalidates exact receipt"), Changed.Outcome,
		ERenderCameraCaptureOutcome::ConfigurationChanged);
	Changed = HarvestReceipt(*Host.Camera, ChangedReceipt);
	TestEqual(TEXT("late pixels cannot relabel changed configuration"), Changed.Outcome,
		ERenderCameraCaptureOutcome::ConfigurationChanged);
	TestTrue(TEXT("changed-configuration writer drains"), Changed.bGpuWriterDrained);
	TestFalse(TEXT("changed configuration exposes no exact pixels"), Changed.Frame.IsValid());
	TestEqual(TEXT("discarded changed pixels release their lease"),
		LiveLeases->load(std::memory_order_relaxed), 0);

	Host.Camera->SetAppliedFrameIdentity(ReceiptIdentity(12));
	FRenderCameraCaptureIssueResult SuccessIssue = IssueReady(*Host.Camera, NewLease());
	if (!TestTrue(TEXT("successful exact capture is accepted"), SuccessIssue.IsAccepted()))
		return false;
	FRenderCameraCaptureReceipt SuccessReceipt = MoveTemp(SuccessIssue.Receipt);
	FRenderCameraCaptureCompletion Success = HarvestReceipt(*Host.Camera, SuccessReceipt);
	if (TestEqual(TEXT("exact receipt succeeds"), Success.Outcome,
		ERenderCameraCaptureOutcome::Succeeded)
		&& TestTrue(TEXT("successful exact pixels are retained"), Success.Frame.IsValid()))
	{
		TestTrue(TEXT("successful writer is drained"), Success.bGpuWriterDrained);
		TestEqual(TEXT("frame retains issued serial"), Success.Frame->CaptureSerial,
			SuccessReceipt.CaptureSerial);
		TestEqual(TEXT("frame retains issued configuration"),
			Success.Frame->CameraConfigurationRevision,
			SuccessReceipt.ConfigurationRevision);
		TestEqual(TEXT("frame retains issued mode"), Success.Frame->Mode, SuccessReceipt.Mode);
		TestEqual(TEXT("frame retains exact identity"), Success.Frame->Identity.FrameId,
			static_cast<uint64>(12));
		TestEqual(TEXT("frame retains exact pixel count"), Success.Frame->Color.Num(), 2048);
	}
	TestEqual(TEXT("successful pixels retain their capacity lease"),
		LiveLeases->load(std::memory_order_relaxed), 1);
	Host.Camera->ClearHistory();
	Success = {};
	SuccessReceipt = {};
	TestEqual(TEXT("last exact pixel reference releases capacity lease"),
		LiveLeases->load(std::memory_order_relaxed), 0);

	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabRenderCameraDetachedReceiptDrainTest,
	"URLab.CameraReceipt.DetachedTeardownDrain",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FURLabRenderCameraDetachedReceiptDrainTest::RunTest(const FString&)
{
	using namespace URLabRenderCameraReceiptTestPrivate;
	FReceiptCameraHost Host;
	if (!TestTrue(TEXT("teardown receipt camera host creates"), Host.Init())) return false;
	if (!FApp::CanEverRender())
	{
		AddInfo(TEXT("No RHI: detached GPU receipt drain requires an offscreen render run"));
		return true;
	}
	FlushRenderingCommands();

	auto LiveLeases = MakeShared<std::atomic<int32>, ESPMode::ThreadSafe>(0);
	FRenderCameraCaptureLifetimeLeasePtr Lease =
		MakeShared<FCountingCaptureLease, ESPMode::ThreadSafe>(LiveLeases);
	FEvent* RenderGate = FPlatformProcess::GetSynchEventFromPool(true);
	auto GateExited = MakeShared<std::atomic<bool>, ESPMode::ThreadSafe>(false);
	ENQUEUE_RENDER_COMMAND(URLabCameraReceiptHoldTeardownWriter)
	([RenderGate, GateExited](FRHICommandListImmediate&)
	{
		RenderGate->Wait(5000);
		GateExited->store(true, std::memory_order_release);
	});
	FRenderCameraCaptureIssueResult Issue = Host.Camera->TryIssueExactCapture(Lease);
	Lease.Reset();
	if (!TestTrue(TEXT("teardown exact capture is accepted"), Issue.IsAccepted()))
	{
		RenderGate->Trigger();
		FlushRenderingCommands();
		FPlatformProcess::ReturnSynchEventToPool(RenderGate);
		return false;
	}
	FRenderCameraCaptureReceipt Receipt = MoveTemp(Issue.Receipt);

	const double DisableStarted = FPlatformTime::Seconds();
	Host.Camera->SetStreamingEnabled(false);
	const double DisableElapsed = FPlatformTime::Seconds() - DisableStarted;
	TestTrue(TEXT("teardown returns without waiting for the blocked render thread"),
		DisableElapsed < 0.5);
	const FRenderCameraCaptureCompletion Teardown = Receipt.Poll();
	TestEqual(TEXT("streaming teardown is immediately terminal"), Teardown.Outcome,
		ERenderCameraCaptureOutcome::Teardown);
	TestFalse(TEXT("terminal teardown does not claim the blocked writer drained"),
		Teardown.bGpuWriterDrained);
	TestEqual(TEXT("teardown retains capacity until the writer drains"),
		LiveLeases->load(std::memory_order_relaxed), 1);

	// The detached owner has no UObject dependency. Retain only the receipt, retire
	// the actor/component, and let ordinary engine frames drive OnEndFrameRT.
	Host.Cleanup();
	RenderGate->Trigger();
	ADD_LATENT_AUTOMATION_COMMAND(FWaitForDetachedReceiptDrain(
		*this, MoveTemp(Receipt), LiveLeases, GateExited, RenderGate,
		FPlatformTime::Seconds() + 10.0));
	return true;
}

#endif
