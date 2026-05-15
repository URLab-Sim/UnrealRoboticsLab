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
#include "mujoco/mjspec.h"
#include "MuJoCo/Utils/MjBind.h"
#include "MuJoCo/Components/MjComponent.h"
#include "MuJoCo/Components/Defaults/MjDefault.h"
#include "MuJoCo/Utils/MjOrientationUtils.h"
#include "MjTendon.generated.h"


/**
 * @enum EMjTendonWrapType
 * @brief Defines the type of each wrap entry within a tendon.
 */
UENUM(BlueprintType)
enum class EMjTendonWrapType : uint8
{
    Joint   UMETA(DisplayName = "Joint (Fixed)"),
    Site    UMETA(DisplayName = "Site (Spatial)"),
    Geom    UMETA(DisplayName = "Geom (Spatial)"),
    Pulley  UMETA(DisplayName = "Pulley (Spatial)")
};


/**
 * @struct FMjTendonWrap
 * @brief Represents a single wrap entry in a MuJoCo tendon.
 *
 * A tendon is composed of an ordered list of wraps. For fixed tendons, use Joint wraps.
 * For spatial tendons, use Site, Geom, and Pulley wraps.
 */
USTRUCT(BlueprintType)
struct FMjTendonWrap
{
    GENERATED_BODY()

    /** @brief Type of this wrap entry. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Tendon|Wrap")
    EMjTendonWrapType Type = EMjTendonWrapType::Joint;

    /**
     * @brief Name of the target joint, site, or geom.
     * Ignored for Pulley wraps.
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Tendon|Wrap")
    FString TargetName;

    /**
     * @brief Gear coefficient for Joint wraps (how much of joint movement maps to tendon length).
     * Ignored for Site, Geom, Pulley wraps.
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Tendon|Wrap")
    float Coef = 1.0f;

    /**
     * @brief Name of the side-site for Geom wraps.
     * The side-site disambiguates which side of the cylinder to wrap around.
     * Ignored for Joint, Site, Pulley wraps.
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Tendon|Wrap")
    FString SideSite;

    /**
     * @brief The pulley divisor for Pulley wraps.
     * Divides the tendon's effective length change; use for compound pulleys.
     * Ignored for Joint, Site, Geom wraps.
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Tendon|Wrap")
    float Divisor = 1.0f;
};


/**
 * @class UMjTendon
 * @brief Component representing a MuJoCo tendon.
 *
 * Tendons are kinematic coupling mechanisms that can:
 * - Connect joints (fixed tendon: uses Joint wraps with gear coefficients)
 * - Route through space (spatial tendon: uses Site, Geom, and Pulley wraps)
 *
 * This component mirrors the `<fixed>` and `<spatial>` elements inside `<tendon>` in MuJoCo XML.
 * Attach this component directly to the AMjArticulation root (not to a body).
 */
UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class URLAB_API UMjTendon : public UMjComponent
{
    GENERATED_BODY()

public:
    // --- CODEGEN_PROPERTIES_START ---
    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjTendon", meta=(InlineEditConditionToggle))
    bool bOverride_group = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjTendon", meta=(EditCondition="bOverride_group"))
    int32 group = 0;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjTendon", meta=(InlineEditConditionToggle))
    bool bOverride_limited = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjTendon", meta=(EditCondition="bOverride_limited"))
    bool limited = false;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjTendon", meta=(InlineEditConditionToggle))
    bool bOverride_actuatorfrclimited = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjTendon", meta=(EditCondition="bOverride_actuatorfrclimited"))
    bool actuatorfrclimited = false;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjTendon", meta=(InlineEditConditionToggle))
    bool bOverride_range = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjTendon", meta=(EditCondition="bOverride_range"))
    TArray<float> range = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjTendon", meta=(InlineEditConditionToggle))
    bool bOverride_actuatorfrcrange = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjTendon", meta=(EditCondition="bOverride_actuatorfrcrange"))
    TArray<float> actuatorfrcrange = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjTendon", meta=(InlineEditConditionToggle))
    bool bOverride_solreflimit = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjTendon", meta=(EditCondition="bOverride_solreflimit"))
    TArray<float> solreflimit = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjTendon", meta=(InlineEditConditionToggle))
    bool bOverride_solimplimit = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjTendon", meta=(EditCondition="bOverride_solimplimit"))
    TArray<float> solimplimit = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjTendon", meta=(InlineEditConditionToggle))
    bool bOverride_solreffriction = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjTendon", meta=(EditCondition="bOverride_solreffriction"))
    TArray<float> solreffriction = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjTendon", meta=(InlineEditConditionToggle))
    bool bOverride_solimpfriction = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjTendon", meta=(EditCondition="bOverride_solimpfriction"))
    TArray<float> solimpfriction = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjTendon", meta=(InlineEditConditionToggle))
    bool bOverride_frictionloss = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjTendon", meta=(EditCondition="bOverride_frictionloss"))
    float frictionloss = 0.0f;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjTendon", meta=(InlineEditConditionToggle))
    bool bOverride_springlength = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjTendon", meta=(EditCondition="bOverride_springlength"))
    TArray<float> springlength = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjTendon", meta=(InlineEditConditionToggle))
    bool bOverride_width = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjTendon", meta=(EditCondition="bOverride_width"))
    float width = 0.0f;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjTendon", meta=(InlineEditConditionToggle))
    bool bOverride_material = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjTendon", meta=(EditCondition="bOverride_material"))
    FString material = TEXT("");

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjTendon", meta=(InlineEditConditionToggle))
    bool bOverride_margin = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjTendon", meta=(EditCondition="bOverride_margin"))
    float margin = 0.0f;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjTendon", meta=(InlineEditConditionToggle))
    bool bOverride_stiffness = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjTendon", meta=(EditCondition="bOverride_stiffness"))
    TArray<float> stiffness = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjTendon", meta=(InlineEditConditionToggle))
    bool bOverride_damping = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjTendon", meta=(EditCondition="bOverride_damping"))
    TArray<float> damping = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjTendon", meta=(InlineEditConditionToggle))
    bool bOverride_armature = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjTendon", meta=(EditCondition="bOverride_armature"))
    float armature = 0.0f;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjTendon", meta=(InlineEditConditionToggle))
    bool bOverride_rgba = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjTendon", meta=(EditCondition="bOverride_rgba"))
    FLinearColor rgba = FLinearColor::White;
    // --- CODEGEN_PROPERTIES_END ---

    UMjTendon();

    /**
     * @brief Exports properties to a pre-created MuJoCo spec tendon structure.
     * @param Tendon Pointer to the target mjsTendon structure.
     * @param def Optional default structure for optimized export.
     */
    void ExportTo(mjsTendon* Element, mjsDefault* def = nullptr);

    /**
     * @brief Registers this tendon to the MuJoCo spec.
     * Calls mjs_addTendon, ExportTo, then mjs_wrapJoint/Site/Geom/Pulley for each Wrap entry.
     * @param Wrapper The spec wrapper instance.
     * @param ParentBody Unused for tendons (tendons are spec-level, not body-children).
     */
    virtual void RegisterToSpec(class FMujocoSpecWrapper& Wrapper, mjsBody* ParentBody = nullptr) override;

    /**
     * @brief Binds this component to the live MuJoCo simulation.
     * Resolves the tendon ID by prefixed name and populates m_TendonView.
     */
    virtual void Bind(mjModel* model, mjData* data, const FString& Prefix = TEXT("")) override;

    /**
     * @brief Imports properties from a raw XML node.
     * @param Node The <fixed> or <spatial> XML child node inside <tendon>.
     */
    void ImportFromXml(const class FXmlNode* Node, const struct FMjCompilerSettings& CompilerSettings = FMjCompilerSettings{});

    /** @brief The runtime view of the MuJoCo tendon. Valid only after Bind() is called. */
    TendonView m_TendonView;

    /** @brief Semantic accessor for raw MuJoCo data and helper methods. */
    TendonView& GetMj() { return m_TendonView; }
    const TendonView& GetMj() const { return m_TendonView; }

    // --- Runtime Accessors ---

    /**
     * @brief Gets the current tendon length (meters).
     * Valid only for 1D tendons after Bind(). Returns 0 if not bound.
     */
    UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
    float GetLength() const;

    /**
     * @brief Gets the current tendon velocity (m/s).
     * Returns 0 if not bound.
     */
    UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
    float GetVelocity() const;

    /** @brief Gets the full prefixed name of this tendon in the compiled MuJoCo model. */
    virtual FString GetMjName() const override;

    // --- Wrap Entries ---

    /**
     * @brief Ordered list of wrap entries that define the tendon path.
     * For fixed tendons: add Joint wraps.
     * For spatial tendons: add Site, Geom, and/or Pulley wraps.
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Tendon")
    TArray<FMjTendonWrap> Wraps;

    /** @brief Optional MuJoCo default class name to inherit from (string fallback). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Tendon", meta=(GetOptions="GetDefaultClassOptions"))
    FString MjClassName;

    /** @brief Reference to a UMjDefault component for default class inheritance. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Tendon")
    UMjDefault* DefaultClass = nullptr;

    virtual FString GetMjClassName() const override
    {
        return MjClassName;
    }

#if WITH_EDITOR
    UFUNCTION()
    TArray<FString> GetDefaultClassOptions() const;
#endif

    // --- Physics Properties ---




    /**
     * @brief Spring resting length [min, max]. Use (-1, -1) to compute from qpos_spring.
     * For fixed tendons this is usually a single value; store in X, leave Y as -1.
     */







    // --- Limits ---







    // --- Actuator Limits ---

    /** @brief Override toggle for bActFrcLimited. */
    UPROPERTY(EditAnywhere, Category = "MuJoCo|Tendon|Limits", meta=(InlineEditConditionToggle))
    bool bOverride_ActFrcLimited = false;

    /** @brief Whether the tendon has actuator force limits. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Tendon|Limits", meta=(EditCondition="bOverride_ActFrcLimited"))
    bool bActFrcLimited = false;

    /** @brief Override toggle for ActFrcRange. */
    UPROPERTY(EditAnywhere, Category = "MuJoCo|Tendon|Limits", meta=(InlineEditConditionToggle))
    bool bOverride_ActFrcRange = false;

    /** @brief Tendon actuator force limits [min, max]. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Tendon|Limits", meta=(EditCondition="bOverride_ActFrcRange"))
    TArray<float> ActFrcRange = {0.0f, 0.0f};

    // --- Solver Parameters ---









    // --- Visuals ---







protected:
    virtual void BeginPlay() override;
};

// --- Multi-UCLASS subclasses --------------------------------------------------
// MJCF distinguishes <spatial> vs <fixed> by the wrap entries (Joint wraps =
// fixed; Site/Geom/Pulley wraps = spatial). URLab keeps a single mjsTendon
// spec API; these subclasses are pure Blueprint UX so the user picks the
// flavour they're authoring directly from "Add Component". Same data shape.

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent, DisplayName="MuJoCo Spatial Tendon"))
class URLAB_API UMjSpatialTendon : public UMjTendon
{
    GENERATED_BODY()
public:
    UMjSpatialTendon();
};

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent, DisplayName="MuJoCo Fixed Tendon"))
class URLAB_API UMjFixedTendon : public UMjTendon
{
    GENERATED_BODY()
public:
    UMjFixedTendon();
};
