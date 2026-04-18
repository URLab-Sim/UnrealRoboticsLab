// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
// trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Tests/MjTestHelpers.h"
#include "MuJoCo/Components/Sensors/MjCamera.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"

namespace
{
    /**
     * Spawn a UMjCamera on the test articulation with the given mode and enable streaming.
     * Returns the camera so callers can inspect state after configuration.
     */
    UMjCamera* SpawnCameraAndStream(FMjUESession& Sess, EMjCameraMode Mode)
    {
        UMjCamera* Cam = NewObject<UMjCamera>(Sess.Robot, TEXT("TestCamera"));
        Cam->CaptureMode = Mode;
        Cam->RegisterComponent();
        Cam->AttachToComponent(Sess.Body, FAttachmentTransformRules::KeepRelativeTransform);
        Cam->SetStreamingEnabled(true);
        return Cam;
    }
}

// ============================================================================
// URLab.Camera.RealMode_ConfiguresFinalColorBGRA
//   Default Real mode → RT is RGBA8, CaptureSource is SCS_FinalToneCurveHDR.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCameraRealModeConfig,
    "URLab.Camera.RealMode_ConfiguresFinalColorBGRA",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCameraRealModeConfig::RunTest(const FString& Parameters)
{
    FMjUESession S;
    if (!S.Init())
    {
        AddError(FString::Printf(TEXT("FMjUESession::Init failed: %s"), *S.LastError));
        return false;
    }

    UMjCamera* Cam = SpawnCameraAndStream(S, EMjCameraMode::Real);
    if (!TestNotNull(TEXT("camera"), Cam)) { S.Cleanup(); return false; }

    if (!TestNotNull(TEXT("RT"), Cam->RenderTarget)) { S.Cleanup(); return false; }
    TestEqual(TEXT("RT format"), (int32)Cam->RenderTarget->RenderTargetFormat, (int32)ETextureRenderTargetFormat::RTF_RGBA8);

    if (TestNotNull(TEXT("capture component"), Cam->CaptureComponent))
    {
        TestEqual(TEXT("capture source"),
            (int32)Cam->CaptureComponent->CaptureSource,
            (int32)ESceneCaptureSource::SCS_FinalToneCurveHDR);
    }

    S.Cleanup();
    return true;
}

// ============================================================================
// URLab.Camera.DepthMode_ConfiguresSceneDepthFloat
//   Depth mode → RT is R32f, CaptureSource is SCS_SceneDepth, near clip overridden.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCameraDepthModeConfig,
    "URLab.Camera.DepthMode_ConfiguresSceneDepthFloat",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCameraDepthModeConfig::RunTest(const FString& Parameters)
{
    FMjUESession S;
    if (!S.Init())
    {
        AddError(FString::Printf(TEXT("FMjUESession::Init failed: %s"), *S.LastError));
        return false;
    }

    UMjCamera* Cam = SpawnCameraAndStream(S, EMjCameraMode::Depth);
    if (!TestNotNull(TEXT("camera"), Cam)) { S.Cleanup(); return false; }

    if (!TestNotNull(TEXT("RT"), Cam->RenderTarget)) { S.Cleanup(); return false; }
    TestEqual(TEXT("RT format"), (int32)Cam->RenderTarget->RenderTargetFormat, (int32)ETextureRenderTargetFormat::RTF_R32f);

    if (TestNotNull(TEXT("capture component"), Cam->CaptureComponent))
    {
        TestEqual(TEXT("capture source"),
            (int32)Cam->CaptureComponent->CaptureSource,
            (int32)ESceneCaptureSource::SCS_SceneDepth);
        TestTrue(TEXT("near clip overridden"), Cam->CaptureComponent->bOverride_CustomNearClippingPlane);
        TestEqual(TEXT("near clip value"),
            Cam->CaptureComponent->CustomNearClippingPlane, Cam->DepthNearCm);
    }

    S.Cleanup();
    return true;
}

// ============================================================================
// URLab.Camera.ModeCycle_RebuildsRenderTarget
//   Toggling streaming off then on after a mode change rebuilds the RT with
//   the new format. Real → Depth should switch RTF_RGBA8 → RTF_R32f.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCameraModeCycleRebuildsRT,
    "URLab.Camera.ModeCycle_RebuildsRenderTarget",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCameraModeCycleRebuildsRT::RunTest(const FString& Parameters)
{
    FMjUESession S;
    if (!S.Init())
    {
        AddError(FString::Printf(TEXT("FMjUESession::Init failed: %s"), *S.LastError));
        return false;
    }

    UMjCamera* Cam = SpawnCameraAndStream(S, EMjCameraMode::Real);
    if (!TestNotNull(TEXT("camera"), Cam)) { S.Cleanup(); return false; }
    TestEqual(TEXT("initial format"), (int32)Cam->RenderTarget->RenderTargetFormat, (int32)ETextureRenderTargetFormat::RTF_RGBA8);

    Cam->SetStreamingEnabled(false);
    TestNull(TEXT("RT cleared after disable"), Cam->RenderTarget);

    Cam->CaptureMode = EMjCameraMode::Depth;
    Cam->SetStreamingEnabled(true);
    if (!TestNotNull(TEXT("RT rebuilt"), Cam->RenderTarget)) { S.Cleanup(); return false; }
    TestEqual(TEXT("new format"), (int32)Cam->RenderTarget->RenderTargetFormat, (int32)ETextureRenderTargetFormat::RTF_R32f);

    S.Cleanup();
    return true;
}
