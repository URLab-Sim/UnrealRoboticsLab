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

#include "UI/MjCameraFeedEntry.h"
#include "Components/TextBlock.h"
#include "Components/Image.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/Material.h"
#include "Styling/SlateTypes.h"
#include "Fonts/SlateFontInfo.h"

void UMjCameraFeedEntry::BindToCamera(UMjCamera* InCamera)
{
    if (!InCamera) return;
    BoundCamera = InCamera;

    InCamera->SetStreamingEnabled(true);

    if (CameraNameText)
    {
        CameraNameText->SetText(FText::FromString(InCamera->MjName));
        FSlateFontInfo FontInfo = CameraNameText->GetFont();
        FontInfo.Size = 12;
        FontInfo.TypefaceFontName = TEXT("Bold");
        CameraNameText->SetFont(FontInfo);
        CameraNameText->SetColorAndOpacity(FSlateColor(FLinearColor(0.8f, 0.9f, 1.0f, 1.0f)));
    }

    RefreshBrush();
}

void UMjCameraFeedEntry::RefreshBrush()
{
    if (!BoundCamera || !FeedImage) return;

    UTextureRenderTarget2D* RT = BoundCamera->RenderTarget;
    if (!RT) return;

    const float W = 320.f;
    const float H = (BoundCamera->Resolution.X > 0)
        ? W * static_cast<float>(BoundCamera->Resolution.Y) / static_cast<float>(BoundCamera->Resolution.X)
        : W * 0.75f;

    // Set the render target DIRECTLY as the brush resource.
    // This works without any material and has no domain restriction.
    // (SetBrushFromMaterial silently renders nothing if the material domain
    //  is not "User Interface" — avoid that path unless you're sure.)
    FeedImage->SetBrushResourceObject(RT);

    // Patch the rest of the brush properties on the copy UImage holds internally
    FSlateBrush Brush = FeedImage->GetBrush();
    Brush.DrawAs   = ESlateBrushDrawType::Image;
    Brush.ImageType = ESlateBrushImageType::FullColor;
    Brush.Tiling   = ESlateBrushTileType::NoTile;
    Brush.ImageSize = FVector2D(W, H);
    FeedImage->SetBrush(Brush);

    UE_LOG(LogURLab, Log, TEXT("[MjCameraFeedEntry] Brush set: '%s' RT=%dx%d display=%.0fx%.0f"),
        *BoundCamera->MjName, RT->SizeX, RT->SizeY, W, H);
}

void UMjCameraFeedEntry::UnbindCamera()
{
    if (BoundCamera)
    {
        BoundCamera->SetStreamingEnabled(false);
        BoundCamera = nullptr;
    }
    FeedMID = nullptr;
    if (FeedImage) FeedImage->SetBrush(FSlateBrush());
}

void UMjCameraFeedEntry::UpdateFeed()
{
    if (BoundCamera && FeedImage && BoundCamera->RenderTarget)
    {
        if (!FeedImage->GetBrush().GetResourceObject())
        {
            RefreshBrush();
        }
    }
}
