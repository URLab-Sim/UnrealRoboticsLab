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
#include "MjEquality.generated.h"

/**
 * @enum EMjEqualityType
 * @brief Defines the type of equality constraint.
 */
UENUM(BlueprintType)
enum class EMjEqualityType : uint8
{
    Connect     = 0,
    Weld        = 1,
    Joint       = 2,
    Tendon      = 3,
    Flex        = 4,
    FlexVert    = 5,
    FlexStrain  = 6
};

/**
 * @class UMjEquality
 * @brief Component representing a MuJoCo equality constraint.
 *
 * Equality constraints (connect, weld, joint, tendon) enforce kinematic relationships
 * between bodies, joints, or tendons.
 */
UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class URLAB_API UMjEquality : public UMjComponent
{
    GENERATED_BODY()

public:
    // --- CODEGEN_PROPERTIES_START ---
    UPROPERTY(EditAnywhere, Category = "MuJoCo|Equality", meta=(InlineEditConditionToggle))
    bool bOverride_relpose = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Equality", meta=(EditCondition="bOverride_relpose"))
    TArray<float> relpose = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Equality", meta=(InlineEditConditionToggle))
    bool bOverride_anchor = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Equality", meta=(EditCondition="bOverride_anchor", MjUnit="cm"))
    TArray<float> anchor = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Equality", meta=(InlineEditConditionToggle))
    bool bOverride_site1 = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Equality", meta=(EditCondition="bOverride_site1"))
    FString site1 = TEXT("");

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Equality", meta=(InlineEditConditionToggle))
    bool bOverride_site2 = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Equality", meta=(EditCondition="bOverride_site2"))
    FString site2 = TEXT("");

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Equality", meta=(InlineEditConditionToggle))
    bool bOverride_active = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Equality", meta=(EditCondition="bOverride_active"))
    float active = 0.0f;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Equality", meta=(InlineEditConditionToggle))
    bool bOverride_solref = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Equality", meta=(EditCondition="bOverride_solref"))
    TArray<float> solref = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Equality", meta=(InlineEditConditionToggle))
    bool bOverride_solimp = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Equality", meta=(EditCondition="bOverride_solimp"))
    TArray<float> solimp = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Equality", meta=(InlineEditConditionToggle))
    bool bOverride_torquescale = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Equality", meta=(EditCondition="bOverride_torquescale"))
    float torquescale = 0.0f;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Equality", meta=(InlineEditConditionToggle))
    bool bOverride_polycoef = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Equality", meta=(EditCondition="bOverride_polycoef"))
    TArray<float> polycoef = {};
    // --- CODEGEN_PROPERTIES_END ---

    UMjEquality();

    /**
     * @brief Exports properties to a pre-created MuJoCo spec equality structure.
     * @param Eq Pointer to the target mjsEquality structure.
     */
    void ExportTo(mjsEquality* Element);

    /**
     * @brief Registers this equality constraint to the MuJoCo spec.
     * @param Wrapper The spec wrapper instance.
     * @param ParentBody Unused (equalities are global in the spec).
     */
    virtual void RegisterToSpec(class FMujocoSpecWrapper& Wrapper, mjsBody* ParentBody = nullptr) override;

    /**
     * @brief Imports properties from a raw XML node.
     * @param Node The <connect>, <weld>, <joint>, or <tendon> XML child node inside <equality>.
     */
    void ImportFromXml(const class FXmlNode* Node, const struct FMjCompilerSettings& CompilerSettings = FMjCompilerSettings{});

    /** @brief The type of equality constraint. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Equality")
    EMjEqualityType EqualityType = EMjEqualityType::Weld;

    /** @brief Name of the first object (body, joint, or tendon). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Equality", meta=(GetOptions="GetObjOptions"))
    FString Obj1;

    /** @brief Name of the second object (body, joint, or tendon). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Equality", meta=(GetOptions="GetObjOptions"))
    FString Obj2;

#if WITH_EDITOR
    UFUNCTION()
    TArray<FString> GetObjOptions() const;
#endif


    // --- Solver Parameters ---





};

// --- Multi-UCLASS subclasses --------------------------------------------------
// One UCLASS per MJCF equality kind. State (anchor/relpose/polycoef/torquescale
// /Obj1/Obj2/active/solref/solimp) lives on the base UMjEquality and is
// codegen-owned. Each subclass pins EqualityType so the Add Component picker
// reads "MuJoCo Weld Equality" / "MuJoCo Joint Equality" / etc., and
// GetObjOptions filters by EqualityType to show the right target dropdown.

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent, DisplayName="MuJoCo Connect Equality"))
class URLAB_API UMjConnectEquality : public UMjEquality
{
    GENERATED_BODY()
public:
    UMjConnectEquality();
};

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent, DisplayName="MuJoCo Weld Equality"))
class URLAB_API UMjWeldEquality : public UMjEquality
{
    GENERATED_BODY()
public:
    UMjWeldEquality();
};

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent, DisplayName="MuJoCo Joint Equality"))
class URLAB_API UMjJointEquality : public UMjEquality
{
    GENERATED_BODY()
public:
    UMjJointEquality();
};

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent, DisplayName="MuJoCo Tendon Equality"))
class URLAB_API UMjTendonEquality : public UMjEquality
{
    GENERATED_BODY()
public:
    UMjTendonEquality();
};

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent, DisplayName="MuJoCo Flex Equality"))
class URLAB_API UMjFlexEquality : public UMjEquality
{
    GENERATED_BODY()
public:
    UMjFlexEquality();
};

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent, DisplayName="MuJoCo FlexVert Equality"))
class URLAB_API UMjFlexVertEquality : public UMjEquality
{
    GENERATED_BODY()
public:
    UMjFlexVertEquality();
};

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent, DisplayName="MuJoCo FlexStrain Equality"))
class URLAB_API UMjFlexStrainEquality : public UMjEquality
{
    GENERATED_BODY()
public:
    UMjFlexStrainEquality();
};
