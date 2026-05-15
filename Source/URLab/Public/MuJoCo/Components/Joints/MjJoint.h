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
#include "mujoco/mujoco.h"
#include "MuJoCo/Utils/MjBind.h"
#include "MuJoCo/Core/MjTypes.h"
#include "MuJoCo/Components/MjComponent.h"
#include "MuJoCo/Components/Defaults/MjDefault.h"
#include "MuJoCo/Utils/MjOrientationUtils.h"
#include "MjJoint.generated.h"


/**
 * @enum EMjJointType
 * @brief Defines the kinematic constraints of a MuJoCo joint.
 */
UENUM(BlueprintType)
enum class EMjJointType : uint8
{
    Free    UMETA(DisplayName = "Free"),
    Ball    UMETA(DisplayName = "Ball"),
    Slide   UMETA(DisplayName = "Slide"),
    Hinge   UMETA(DisplayName = "Hinge")
};


/**
 * @class UMjJoint
 * @brief Component representing a MuJoCo joint.
 * 
 * Joints connect bodies and define their relative motion (degrees of freedom).
 * This component mirrors the `joint` element in MuJoCo XML.
 */
UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class URLAB_API UMjJoint : public UMjComponent
{
	GENERATED_BODY()

public:	
    // --- CODEGEN_PROPERTIES_START ---
    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjJoint", meta=(InlineEditConditionToggle))
    bool bOverride_group = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjJoint", meta=(EditCondition="bOverride_group"))
    int32 group = 0;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjJoint", meta=(InlineEditConditionToggle))
    bool bOverride_springdamper = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjJoint", meta=(EditCondition="bOverride_springdamper"))
    TArray<float> springdamper = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjJoint", meta=(InlineEditConditionToggle))
    bool bOverride_limited = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjJoint", meta=(EditCondition="bOverride_limited"))
    bool limited = false;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjJoint", meta=(InlineEditConditionToggle))
    bool bOverride_actuatorfrclimited = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjJoint", meta=(EditCondition="bOverride_actuatorfrclimited"))
    bool actuatorfrclimited = false;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjJoint", meta=(InlineEditConditionToggle))
    bool bOverride_solreflimit = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjJoint", meta=(EditCondition="bOverride_solreflimit"))
    TArray<float> solreflimit = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjJoint", meta=(InlineEditConditionToggle))
    bool bOverride_solimplimit = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjJoint", meta=(EditCondition="bOverride_solimplimit"))
    TArray<float> solimplimit = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjJoint", meta=(InlineEditConditionToggle))
    bool bOverride_solreffriction = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjJoint", meta=(EditCondition="bOverride_solreffriction"))
    TArray<float> solreffriction = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjJoint", meta=(InlineEditConditionToggle))
    bool bOverride_solimpfriction = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjJoint", meta=(EditCondition="bOverride_solimpfriction"))
    TArray<float> solimpfriction = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjJoint", meta=(InlineEditConditionToggle))
    bool bOverride_stiffness = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjJoint", meta=(EditCondition="bOverride_stiffness"))
    TArray<float> stiffness = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjJoint", meta=(InlineEditConditionToggle))
    bool bOverride_range = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjJoint", meta=(EditCondition="bOverride_range"))
    TArray<float> range = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjJoint", meta=(InlineEditConditionToggle))
    bool bOverride_actuatorfrcrange = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjJoint", meta=(EditCondition="bOverride_actuatorfrcrange"))
    TArray<float> actuatorfrcrange = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjJoint", meta=(InlineEditConditionToggle))
    bool bOverride_actuatorgravcomp = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjJoint", meta=(EditCondition="bOverride_actuatorgravcomp"))
    float actuatorgravcomp = 0.0f;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjJoint", meta=(InlineEditConditionToggle))
    bool bOverride_margin = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjJoint", meta=(EditCondition="bOverride_margin"))
    float margin = 0.0f;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjJoint", meta=(InlineEditConditionToggle))
    bool bOverride_ref = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjJoint", meta=(EditCondition="bOverride_ref"))
    float ref = 0.0f;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjJoint", meta=(InlineEditConditionToggle))
    bool bOverride_springref = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjJoint", meta=(EditCondition="bOverride_springref"))
    float springref = 0.0f;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjJoint", meta=(InlineEditConditionToggle))
    bool bOverride_armature = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjJoint", meta=(EditCondition="bOverride_armature"))
    float armature = 0.0f;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjJoint", meta=(InlineEditConditionToggle))
    bool bOverride_damping = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjJoint", meta=(EditCondition="bOverride_damping"))
    TArray<float> damping = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjJoint", meta=(InlineEditConditionToggle))
    bool bOverride_frictionloss = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjJoint", meta=(EditCondition="bOverride_frictionloss"))
    float frictionloss = 0.0f;
    // --- CODEGEN_PROPERTIES_END ---

	UMjJoint();


    virtual void ExportTo(mjsJoint* Element, mjsDefault* Default = nullptr);

    virtual void Bind(mjModel* Model, mjData* Data, const FString& Prefix = TEXT("")) override;
    
    /**
     * @brief Imports properties and override flags directly from the raw XML node.
     * @param Node Pointer to the corresponding FXmlNode.
     * @param CompilerSettings Compiler settings (autolimits, angle units, euler sequence).
     */
    virtual void ImportFromXml(const class FXmlNode* Node, const FMjCompilerSettings& CompilerSettings = FMjCompilerSettings{});

    /**
     * @brief Registers this joint to the MuJoCo spec.
     * @param Wrapper The spec wrapper instance.
     * @param ParentBody The parent body to attach to.
     */
    virtual void RegisterToSpec(class FMujocoSpecWrapper& Wrapper, mjsBody* ParentBody) override;

    /** @brief The runtime view of the MuJoCo joint. Valid only after Bind() is called. */
    JointView m_JointView;
    
    /** @brief Semantic accessor for raw MuJoCo data and helper methods. */
    JointView& GetMj() { return m_JointView; }
    const JointView& GetMj() const { return m_JointView; }

    // --- Runtime Accessors ---

    /** 
     * @brief Gets the current joint position (angle/slide). 
     * Only valid for 1-DOF joints (Hinge/Slide). Returns 0 for complex joints.
     */
    UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
    float GetPosition() const;

    /** 
     * @brief Sets the joint position (angle/slide) directly in mjData.
     * Be careful: this teleports the joint.
     */
    UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
    void SetPosition(float NewPosition);

    /** @brief Gets the current joint velocity. */
    UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
    float GetVelocity() const;
    
    /** @brief Sets the joint velocity. */
    UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
    void SetVelocity(float NewVelocity);

    /** @brief Gets the joint acceleration (qacc). Valid for 1-DOF joints (Hinge/Slide). Returns 0 for others. */
    UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
    float GetAcceleration() const;

    /** @brief Gets the joint range [min, max] from the compiled model. Returns (0,0) if limits are disabled. */
    UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
    FVector2D GetJointRange() const;

    virtual void BuildBinaryPayload(FBufferArchive& OutBuffer) const override;
    virtual FString GetTelemetryTopicName() const override;

    /** @brief Gets the complete runtime state (Pos, Vel, Accel) for this joint. */
    UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
    FMuJoCoJointState GetJointState() const;

    /** @brief Gets the world-space anchor position of this joint (UE coordinates, cm). */
    UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
    FVector GetWorldAnchor() const;

    /** @brief Gets the world-space axis of this joint (unit vector, UE coordinates). */
    UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
    FVector GetWorldAxis() const;

    /** @brief Gets the full prefixed name of this joint as it appears in the compiled MuJoCo model. */
    virtual FString GetMjName() const override;

    /** @brief Optional MuJoCo class name to inherit defaults from (string fallback). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Joint", meta=(GetOptions="GetDefaultClassOptions"))
    FString MjClassName;

#if WITH_EDITOR
    UFUNCTION()
    TArray<FString> GetDefaultClassOptions() const;
#endif

    /** @brief Reference to a UMjDefault component for default class inheritance. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Joint")
    class UMjDefault* DefaultClass = nullptr;

    virtual FString GetMjClassName() const override
    {
        return MjClassName;
    }

    // --- Editor-time resolved accessors ---
    // Walk the default class chain to resolve effective property values.
    // If bound (runtime), uses compiled model data. Otherwise, resolves
    // through UMjDefault hierarchy with MuJoCo builtin fallbacks.

    /** Returns the resolved joint type. */
    EMjJointType GetResolvedType() const;

    /** Returns the resolved axis (UE coordinates). */
    FVector GetResolvedAxis() const;

    /** Returns the resolved range [min, max]. (0,0) if unlimited. */
    FVector2D GetResolvedRange() const;

    /** Returns whether limits are resolved as enabled. */
    bool GetResolvedLimited() const;

    /** @brief Override toggle for Type. */
    UPROPERTY(EditAnywhere, Category = "MuJoCo|Joint", meta=(InlineEditConditionToggle))
    bool bOverride_Type = false;

    /** @brief The type of joint (Hinge, Slide, Ball, Free). Default: Hinge (MuJoCo builtin). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Joint", meta=(EditCondition="bOverride_Type"))
    EMjJointType Type = EMjJointType::Hinge;

    /** @brief Override toggle for Axis. */
    UPROPERTY(EditAnywhere, Category = "MuJoCo|Joint", meta=(InlineEditConditionToggle))
    bool bOverride_Axis = false;

    /** @brief Local joint axis vector (relative to parent body). Ignored for Free/Ball joints. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Joint", meta=(EditCondition="bOverride_Axis"))
    FVector Axis = FVector(0.0f, 0.0f, 1.0f);



    // --- Physics Properties (with override toggles) ---




    






    // --- Limits (with override toggles) ---




	




    // --- Solver Parameters (with override toggles) ---









    // --- group ---



};
