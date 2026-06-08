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

#include <mujoco/mjspec.h>

#include "CoreMinimal.h"
#include "mujoco/mujoco.h"
#include "MuJoCo/Core/Spec/MjSpecElement.h"
#include "MuJoCo/Components/MjComponent.h"
#include "MuJoCo/Components/Defaults/MjDefault.h"
#include "MuJoCo/Utils/MjOrientationUtils.h"
#include <atomic>
#include "MjActuator.generated.h"

/**
 * @enum EMjActuatorType
 * @brief Defines the type of actuator dynamics for MuJoCo.
 */
UENUM(BlueprintType)
enum class EMjActuatorType : uint8
{
    Motor,
    Position,
    Velocity,
    IntVelocity,
    Damper,
    Cylinder,
    Muscle,
    Adhesion,
    DcMotor
};

/**
 * @enum EMjActuatorTrnType
 * @brief Defines the type of transmission for the actuator.
 */
UENUM(BlueprintType)
enum class EMjActuatorTrnType : uint8
{
    Joint,
    JointInParent,
    SliderCrank,
    Tendon,
    Site,
    Body,
    Undefined
};

/**
 * @enum EMjGainType
 * @brief MJCF actuator gain function. Mirrors MuJoCo's mjtGain.
 *        Most actuator subtypes hardcode this via mjs_setToX; useful on
 *        ``<general>`` to express the raw gainprm formula directly.
 */
UENUM(BlueprintType)
enum class EMjGainType : uint8
{
    Fixed,
    Affine,
    Muscle,
    User,
    DcMotor
};

/**
 * @enum EMjBiasType
 * @brief MJCF actuator bias function. Mirrors MuJoCo's mjtBias.
 */
UENUM(BlueprintType)
enum class EMjBiasType : uint8
{
    None,
    Affine,
    Muscle,
    User,
    DcMotor
};

/**
 * @enum EMjDynType
 * @brief MJCF actuator activation dynamics. Mirrors MuJoCo's mjtDyn.
 */
UENUM(BlueprintType)
enum class EMjDynType : uint8
{
    None,
    Integrator,
    Filter,
    FilterExact,
    Muscle,
    User,
    DcMotor
};

/**
 * @class UMjActuator
 * @brief Component representing a MuJoCo actuator.
 * 
 * An actuator generates force/torque and applies it to the simulation.
 * It corresponds to the `actuator` elements in MuJoCo (motor, position, velocity, etc.).
 */
UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class URLAB_API UMjActuator : public UMjComponent
{
	GENERATED_BODY()

public:	
    // --- CODEGEN_PROPERTIES_START ---
    UPROPERTY(EditAnywhere, Category = "MuJoCo|Actuator", meta=(InlineEditConditionToggle))
    bool bOverride_group = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta=(EditCondition="bOverride_group"))
    int32 group = 0;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Actuator", meta=(InlineEditConditionToggle))
    bool bOverride_nsample = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta=(EditCondition="bOverride_nsample"))
    int32 nsample = 0;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Actuator", meta=(InlineEditConditionToggle))
    bool bOverride_interp = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta=(EditCondition="bOverride_interp"))
    int32 interp = 0;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Actuator", meta=(InlineEditConditionToggle))
    bool bOverride_delay = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta=(EditCondition="bOverride_delay"))
    float delay = 0.0f;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Actuator", meta=(InlineEditConditionToggle))
    bool bOverride_ctrllimited = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta=(EditCondition="bOverride_ctrllimited"))
    bool ctrllimited = false;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Actuator", meta=(InlineEditConditionToggle))
    bool bOverride_forcelimited = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta=(EditCondition="bOverride_forcelimited"))
    bool forcelimited = false;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Actuator", meta=(InlineEditConditionToggle))
    bool bOverride_actlimited = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta=(EditCondition="bOverride_actlimited"))
    bool actlimited = false;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Actuator", meta=(InlineEditConditionToggle))
    bool bOverride_ctrlrange = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta=(EditCondition="bOverride_ctrlrange"))
    TArray<float> ctrlrange = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Actuator", meta=(InlineEditConditionToggle))
    bool bOverride_forcerange = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta=(EditCondition="bOverride_forcerange"))
    TArray<float> forcerange = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Actuator", meta=(InlineEditConditionToggle))
    bool bOverride_actrange = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta=(EditCondition="bOverride_actrange"))
    TArray<float> actrange = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Actuator", meta=(InlineEditConditionToggle))
    bool bOverride_lengthrange = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta=(EditCondition="bOverride_lengthrange"))
    TArray<float> lengthrange = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Actuator", meta=(InlineEditConditionToggle))
    bool bOverride_gear = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta=(EditCondition="bOverride_gear"))
    TArray<float> gear = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Actuator", meta=(InlineEditConditionToggle))
    bool bOverride_damping = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta=(EditCondition="bOverride_damping"))
    TArray<float> damping = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Actuator", meta=(InlineEditConditionToggle))
    bool bOverride_armature = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta=(EditCondition="bOverride_armature"))
    float armature = 0.0f;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Actuator", meta=(InlineEditConditionToggle))
    bool bOverride_cranklength = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta=(EditCondition="bOverride_cranklength"))
    float cranklength = 0.0f;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Actuator", meta=(InlineEditConditionToggle))
    bool bOverride_gainprm = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta=(EditCondition="bOverride_gainprm"))
    TArray<float> gainprm = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Actuator", meta=(InlineEditConditionToggle))
    bool bOverride_biasprm = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta=(EditCondition="bOverride_biasprm"))
    TArray<float> biasprm = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Actuator", meta=(InlineEditConditionToggle))
    bool bOverride_dynprm = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta=(EditCondition="bOverride_dynprm"))
    TArray<float> dynprm = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Actuator", meta=(InlineEditConditionToggle))
    bool bOverride_actdim = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta=(EditCondition="bOverride_actdim"))
    int32 actdim = 0;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Actuator", meta=(InlineEditConditionToggle))
    bool bOverride_GainType = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta=(EditCondition="bOverride_GainType"))
    EMjGainType GainType = EMjGainType::Fixed;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Actuator", meta=(InlineEditConditionToggle))
    bool bOverride_BiasType = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta=(EditCondition="bOverride_BiasType"))
    EMjBiasType BiasType = EMjBiasType::None;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Actuator", meta=(InlineEditConditionToggle))
    bool bOverride_DynType = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta=(EditCondition="bOverride_DynType"))
    EMjDynType DynType = EMjDynType::None;
    // --- CODEGEN_PROPERTIES_END ---

    UMjActuator();

    /** @brief The type of actuator dynamics (e.g. Motor, Position). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator")
    EMjActuatorType Type;

    /** @brief The transmission type connecting the actuator to the system. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator")
    EMjActuatorTrnType TransmissionType;

    // gaintype / biastype / dyntype UPROPERTYs are codegen-emitted inside
    // CODEGEN_PROPERTIES via xml_enum_attrs.emit_property_decl. Useful
    // primarily on <general> actuators; for motor/position/velocity/etc.
    // subtypes the mjs_setToX preset writes these fields and overrides
    // the user's choice.

    /** @brief Optional MuJoCo class name to inherit defaults from (string fallback). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta=(GetOptions="GetDefaultClassOptions"))
    FString MjClassName;

    /** @brief Reference to a UMjDefault component for default class inheritance. Hidden from the
     *  Details panel — synced from MjClassName via the editor dropdown. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator",
              meta=(EditCondition="false", EditConditionHides))
    UMjDefault* DefaultClass = nullptr;

    virtual FString GetMjClassName() const override
    {
        return MjClassName;
    }

    virtual void ExportTo(mjsActuator* Element, mjsDefault* Default = nullptr);

    /**
     * @brief Imports properties and override flags directly from the raw XML node.
     * @param Node Pointer to the corresponding FXmlNode.
     */
    virtual void ImportFromXml(const class FXmlNode* Node, const struct FMjCompilerSettings& CompilerSettings = FMjCompilerSettings{});

    // --- Runtime Binding ---
    /**
     * @brief Registers this actuator to the MuJoCo spec.
     * @param Wrapper The spec wrapper instance.
     */
    virtual void RegisterToSpec(class FMujocoSpecWrapper& Wrapper, mjsBody* ParentBody = nullptr) override;

    /**
     * @brief Binds this component to the live MuJoCo simulation.
     */
    virtual void Bind(mjModel* Model, mjData* Data, const FString& Prefix = TEXT("")) override;

    /**
     * @brief Sets the internal control input (ctrl) for this actuator.
     * Updates the lock-free InternalValue atomic register.
     */
    UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
    void SetControl(float Value);

    /**
     * @brief Resets the control input (ctrl) for this actuator to zero.
     * Synchronizes both the physics buffer and the UI control map.
     */
    UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
    void ResetControl();

    /** @brief Gets the current resolved control value being applied. */
    UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
    float GetControl() const;

    float GetMjControl() const;
    
    /** @brief Gets the current force generated by this actuator. */
    UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
    float GetForce() const;

    /** @brief Gets the current length of the actuator. */
    UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
    float GetLength() const;

    /** @brief Gets the current velocity of the actuator. */
    UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
    float GetVelocity() const;

    /** @brief Gets the control range [min, max] from the compiled model. Returns ZeroVector if ctrl is not limited. */
    UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
    FVector2D GetControlRange() const;

    /** @brief Gets the current activation state (for stateful actuators like muscle/intvelocity). Returns 0 if stateless. */
    UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
    float GetActivation() const;

    /** @brief Gets the full prefixed name of this actuator as it appears in the compiled MuJoCo model. */
    virtual FString GetMjName() const override;

    /** @brief Sets the gear ratio (expert use). */
    UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
    void SetGear(const TArray<float>& NewGear);

    /** @brief Gets the gear ratio. */
    UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
    TArray<float> GetGear() const;

    /** 
     * @brief Resolves the final control value to apply based on the specified source.
     * This is called by the Articulation during the async physics step.
     */
    float ResolveDesiredControl(uint8 Source) const;

    /** @brief Sets the control value from the ZMQ networked stream. */
    void SetNetworkControl(float Value);

    ActuatorView m_ActuatorView;

protected:
    /** @brief Internal control value (from UI or Blueprints). */
    std::atomic<float> InternalValue{0.0f};

    /** @brief External control value (from ZMQ). */
    std::atomic<float> NetworkValue{0.0f};

public:

    /** @brief Name of the target element (Joint, Tendon, Site, etc.) this actuator acts upon. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta=(GetOptions="GetTargetNameOptions"))
    FString TargetName;











    // Transmission specific

    /** @brief Name of the slider site for slider-crank transmission. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta=(GetOptions="GetSliderSiteOptions"))
    FString SliderSite;

    /** @brief Reference site name. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta=(GetOptions="GetRefSiteOptions"))
    FString RefSite;



#if WITH_EDITOR
    UFUNCTION()
    TArray<FString> GetTargetNameOptions() const;
    UFUNCTION()
    TArray<FString> GetSliderSiteOptions() const;
    UFUNCTION()
    TArray<FString> GetRefSiteOptions() const;
    UFUNCTION()
    TArray<FString> GetDefaultClassOptions() const;
#endif

	virtual void BeginPlay() override;
};
