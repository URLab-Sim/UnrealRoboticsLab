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
#include "MjKeyframe.generated.h"

/**
 * @class UMjKeyframe
 * @brief Component representing a MuJoCo keyframe.
 *
 * Keyframes store simulation state (time, qpos, qvel, act, ctrl, mocap) at a specific moment.
 */
UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class URLAB_API UMjKeyframe : public UMjComponent
{
    GENERATED_BODY()

public:
    // --- CODEGEN_PROPERTIES_START ---
    UPROPERTY(EditAnywhere, Category = "MuJoCo|Keyframe", meta=(InlineEditConditionToggle))
    bool bOverride_Time = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Keyframe", meta=(EditCondition="bOverride_Time"))
    float Time = 0.0f;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Keyframe", meta=(InlineEditConditionToggle))
    bool bOverride_Qpos = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Keyframe", meta=(EditCondition="bOverride_Qpos"))
    TArray<float> Qpos = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Keyframe", meta=(InlineEditConditionToggle))
    bool bOverride_Qvel = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Keyframe", meta=(EditCondition="bOverride_Qvel"))
    TArray<float> Qvel = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Keyframe", meta=(InlineEditConditionToggle))
    bool bOverride_Act = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Keyframe", meta=(EditCondition="bOverride_Act"))
    TArray<float> Act = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Keyframe", meta=(InlineEditConditionToggle))
    bool bOverride_Mpos = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Keyframe", meta=(EditCondition="bOverride_Mpos"))
    TArray<float> Mpos = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Keyframe", meta=(InlineEditConditionToggle))
    bool bOverride_Mquat = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Keyframe", meta=(EditCondition="bOverride_Mquat"))
    TArray<float> Mquat = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Keyframe", meta=(InlineEditConditionToggle))
    bool bOverride_Ctrl = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Keyframe", meta=(EditCondition="bOverride_Ctrl"))
    TArray<float> Ctrl = {};
    // --- CODEGEN_PROPERTIES_END ---

    UMjKeyframe();

    /**
     * @brief Exports properties to a pre-created MuJoCo spec keyframe structure.
     *        Codegen-owned: writes Time + the six mjDoubleVec* fields (qpos/
     *        qvel/act/ctrl/mpos/mquat) from the UPROPERTY values. RegisterToSpec
     *        applies freejoint-aware padding for qpos/qvel on top.
     * @param Key  Pointer to the target mjsKey structure.
     */
    void ExportTo(mjsKey* Element, mjsDefault* Default = nullptr);

    /**
     * @brief Registers this keyframe to the MuJoCo spec.
     * @param Wrapper The spec wrapper instance.
     * @param ParentBody Unused (keyframes are global).
     */
    virtual void RegisterToSpec(class FMujocoSpecWrapper& Wrapper, mjsBody* ParentBody = nullptr) override;

    /**
     * @brief Imports properties from a raw XML node. Codegen-owned.
     * @param Node             The <key> XML node inside <keyframe>.
     * @param CompilerSettings MJCF compiler-level settings (unused here, but
     *                         matches the standard signature so codegen can
     *                         emit a tagged import block).
     */
    void ImportFromXml(const class FXmlNode* Node, const struct FMjCompilerSettings& CompilerSettings = FMjCompilerSettings{});
};
