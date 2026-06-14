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

#pragma once

#include "CoreMinimal.h"
#include "mujoco/mjspec.h"
#include "MuJoCo/Components/MjComponent.h"
#include "MuJoCo/Utils/MjOrientationUtils.h"
#include "MjFrame.generated.h"

/**
 * @class UMjFrame
 * @brief Represents a MuJoCo <frame> (mjsFrame).
 *
 * Frames supply a coordinate offset that the MuJoCo compiler bakes into the
 * pos/quat of their child elements; they don't survive into mjModel/mjData, so
 * no runtime view or Bind work is needed. Spec creation goes through
 * MjFrame::Setup, which is dispatched by the parent UMjBody during its own
 * Setup walk and recursively visits the frame's UE child components.
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class URLAB_API UMjFrame : public UMjComponent
{
    GENERATED_BODY()

public:
    // --- CODEGEN_PROPERTIES_START ---
    UPROPERTY(EditAnywhere, Category = "MuJoCo|Frame|Spatial Pose", meta=(InlineEditConditionToggle))
    bool bOverride_Pos = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Frame|Spatial Pose", meta=(EditCondition="false", EditConditionHides))
    FVector Pos = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Frame|Orientation", meta=(InlineEditConditionToggle))
    bool bOverride_Quat = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Frame|Orientation", meta=(EditCondition="false", EditConditionHides))
    FQuat Quat = FQuat::Identity;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Frame", meta=(InlineEditConditionToggle))
    bool bOverride_childclass = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Frame", meta=(EditCondition="bOverride_childclass"))
    FString childclass = TEXT("");
    // --- CODEGEN_PROPERTIES_END ---

    UMjFrame();

    /**
     * @brief Recursive spec registration entry point for frames. Creates the
     *        mjsFrame from the UE relative transform (the source of truth)
     *        and dispatches RegisterToSpec on the frame's child components.
     */
    virtual void Setup(USceneComponent* Parent, mjsBody* ParentBody, class FMujocoSpecWrapper* Wrapper);

    /**
     * @brief Codegen-owned: writes the frame's hand-rolled childclass and any
     *        future per-attr fields. Pos/Quat come from the UE transform via
     *        MjSpecWrapper::CreateFrame and are deliberately skipped here.
     */
    virtual void ExportTo(mjsFrame* Element, mjsDefault* Default = nullptr);

    /** @brief Empty: frames are compiled away. */
    virtual void Bind(mjModel* Model, mjData* Data, const FString& Prefix = TEXT("")) override;

    /** @brief Frame registration is driven by Setup (called from UMjBody). */
    virtual void RegisterToSpec(class FMujocoSpecWrapper& Wrapper, mjsBody* ParentBody = nullptr) override;

    /**
     * @brief Codegen-owned: imports pos/orientation/childclass from <frame>.
     *        Drives SetRelativeLocation/SetRelativeRotation from the codegen-
     *        emitted Pos/Quat UPROPERTYs so the UE editor matches the data.
     */
    void ImportFromXml(const class FXmlNode* Node, const struct FMjCompilerSettings& CompilerSettings = FMjCompilerSettings{});

private:
    /** @brief Cached list of child elements for registration. */
    UPROPERTY()
    TArray<TScriptInterface<IMjSpecElement>> m_SpecElements;
};
