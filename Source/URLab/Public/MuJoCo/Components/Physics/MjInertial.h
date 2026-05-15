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
#include "Components/SceneComponent.h"
#include "mujoco/mujoco.h"
#include "MuJoCo/Components/MjComponent.h"
#include "MuJoCo/Utils/MjOrientationUtils.h"
#include "MjInertial.generated.h"



UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class URLAB_API UMjInertial : public UMjComponent
{
	GENERATED_BODY()

public:
    // --- CODEGEN_PROPERTIES_START ---
    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjInertial|Spatial Pose", meta=(InlineEditConditionToggle))
    bool bOverride_Pos = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjInertial|Spatial Pose", meta=(EditCondition="bOverride_Pos"))
    FVector Pos = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjInertial|Orientation", meta=(InlineEditConditionToggle))
    bool bOverride_Quat = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjInertial|Orientation", meta=(EditCondition="bOverride_Quat"))
    FQuat Quat = FQuat::Identity;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjInertial", meta=(InlineEditConditionToggle))
    bool bOverride_mass = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjInertial", meta=(EditCondition="bOverride_mass"))
    float mass = 0.0f;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjInertial", meta=(InlineEditConditionToggle))
    bool bOverride_diaginertia = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjInertial", meta=(EditCondition="bOverride_diaginertia"))
    TArray<float> diaginertia = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjInertial", meta=(InlineEditConditionToggle))
    bool bOverride_fullinertia = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjInertial", meta=(EditCondition="bOverride_fullinertia"))
    TArray<float> fullinertia = {};
    // --- CODEGEN_PROPERTIES_END ---

    /** @brief Default constructor. */
    UMjInertial();







    /**
     * @brief Imports properties from a MuJoCo XML node.
     * @param Node Pointer to the XML node.
     */
    void ImportFromXml(const class FXmlNode* Node, const struct FMjCompilerSettings& CompilerSettings);

    // Note: Inertial usually doesn't have a direct 'mjsInertial' struct in mjspec.h same as others,
    // it's often part of body or explicit inertial element. 
    // mjsBody has 'mass', 'inertia', 'fullinertia'.
    
    /**
     * @brief Binds this component to the live MuJoCo simulation.
     */
    virtual void Bind(mjModel* model, mjData* data, const FString& Prefix = TEXT("")) override;

    /**
     * @brief Registers/Applies this inertial to the parent body in the spec.
     * @param Wrapper The spec wrapper instance.
     * @param ParentBody The parent body to apply inertial properties to.
     */
    virtual void RegisterToSpec(class FMujocoSpecWrapper& Wrapper, mjsBody* ParentBody) override;

};
