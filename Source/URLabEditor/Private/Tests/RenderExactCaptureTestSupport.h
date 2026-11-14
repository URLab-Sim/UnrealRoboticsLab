// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
//
// Drives one camera through the exact-capture path the render server uses
// (TryIssueExactCapture plus the async readback pipeline) and waits for the
// receipt to settle.

#pragma once

#include "CoreMinimal.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "Render/RenderCameraComponent.h"
#include "RenderingThread.h"

namespace URLabRenderTest
{
/** Issue an exact capture, retrying while the camera reports a retryable state. */
inline FRenderCameraCaptureIssueResult IssueExactCapture(URenderCameraComponent& Camera,
	FRenderCameraCaptureLifetimeLeasePtr Lease = {})
{
	FRenderCameraCaptureIssueResult Result = Camera.TryIssueExactCapture(Lease);
	for (int32 Attempt = 0; Attempt < 4 && Result.IsRetryable(); ++Attempt)
	{
		FlushRenderingCommands();
		Result = Camera.TryIssueExactCapture(Lease);
	}
	return Result;
}

/** Pump the readback pipeline until the receipt is terminal and its GPU writer has
 *  drained, or the timeout elapses. */
inline FRenderCameraCaptureCompletion WaitForReceipt(URenderCameraComponent& Camera,
	const FRenderCameraCaptureReceipt& Receipt, double TimeoutSeconds)
{
	const double Deadline = FPlatformTime::Seconds() + TimeoutSeconds;
	FRenderCameraCaptureCompletion Completion = Receipt.Poll();
	auto Settled = [](const FRenderCameraCaptureCompletion& C)
	{
		return C.IsTerminal() && C.bGpuWriterDrained;
	};
	while (!Settled(Completion) && FPlatformTime::Seconds() < Deadline)
	{
		FlushRenderingCommands();
		Camera.HarvestCompletedReadbacks();
		Completion = Receipt.Poll();
		if (!Settled(Completion))
		{
			FPlatformProcess::SleepNoStats(0.0005f);
		}
	}
	return Completion;
}

/** Issue one exact capture and wait for it. Returns a non-terminal completion when
 *  the capture was never accepted. */
inline FRenderCameraCaptureCompletion CaptureExact(URenderCameraComponent& Camera,
	double TimeoutSeconds)
{
	const FRenderCameraCaptureIssueResult Issue = IssueExactCapture(Camera);
	if (!Issue.IsAccepted())
	{
		FRenderCameraCaptureCompletion Rejected;
		Rejected.Message = Issue.Message;
		return Rejected;
	}
	return WaitForReceipt(Camera, Issue.Receipt, TimeoutSeconds);
}
} // namespace URLabRenderTest
