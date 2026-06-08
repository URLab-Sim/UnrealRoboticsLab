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
#include "Components/SceneComponent.h"
#include "MuJoCo/Core/Spec/MjSpecElement.h"
#include "MuJoCo/Utils/MjOrientationUtils.h"
#include "MjContactPair.generated.h"

/**
 * @class UMjContactPair
 * @brief Component representing a MuJoCo contact pair.
 * 
 * Explicitly defines contact between two geoms with custom properties.
 * Corresponds to the <pair> element in MuJoCo XML.
 */
UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class URLAB_API UMjContactPair : public USceneComponent, public IMjSpecElement
{
	GENERATED_BODY()

public:	
    // --- CODEGEN_PROPERTIES_START ---
    UPROPERTY(EditAnywhere, Category = "MuJoCo|ContactPair", meta=(InlineEditConditionToggle))
    bool bOverride_geom1 = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|ContactPair", meta=(EditCondition="bOverride_geom1"))
    FString geom1 = TEXT("");

    UPROPERTY(EditAnywhere, Category = "MuJoCo|ContactPair", meta=(InlineEditConditionToggle))
    bool bOverride_geom2 = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|ContactPair", meta=(EditCondition="bOverride_geom2"))
    FString geom2 = TEXT("");

    UPROPERTY(EditAnywhere, Category = "MuJoCo|ContactPair", meta=(InlineEditConditionToggle))
    bool bOverride_condim = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|ContactPair", meta=(EditCondition="bOverride_condim"))
    int32 condim = 0;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|ContactPair", meta=(InlineEditConditionToggle))
    bool bOverride_friction = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|ContactPair", meta=(EditCondition="bOverride_friction"))
    TArray<float> friction = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|ContactPair", meta=(InlineEditConditionToggle))
    bool bOverride_solref = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|ContactPair", meta=(EditCondition="bOverride_solref"))
    TArray<float> solref = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|ContactPair", meta=(InlineEditConditionToggle))
    bool bOverride_solreffriction = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|ContactPair", meta=(EditCondition="bOverride_solreffriction"))
    TArray<float> solreffriction = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|ContactPair", meta=(InlineEditConditionToggle))
    bool bOverride_solimp = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|ContactPair", meta=(EditCondition="bOverride_solimp"))
    TArray<float> solimp = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|ContactPair", meta=(InlineEditConditionToggle))
    bool bOverride_gap = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|ContactPair", meta=(EditCondition="bOverride_gap"))
    float gap = 0.0f;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|ContactPair", meta=(InlineEditConditionToggle))
    bool bOverride_margin = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|ContactPair", meta=(EditCondition="bOverride_margin"))
    float margin = 0.0f;
    // --- CODEGEN_PROPERTIES_END ---

    /** @brief Default constructor. */
	UMjContactPair();

    /** @brief Name of the contact pair (optional). Hidden from the Details panel — synced from MjName. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Contact Pair",
              meta=(EditCondition="false", EditConditionHides))
    FString Name;









    /**
     * @brief Imports contact pair settings from a MuJoCo XML <pair> node.
     * @param Node Pointer to the FXmlNode representing the <pair> element.
     */
    void ImportFromXml(const class FXmlNode* Node, const struct FMjCompilerSettings& CompilerSettings = FMjCompilerSettings{});

    /**
     * @brief Exports contact pair settings to a MuJoCo mjsPair structure.
     * @param pair Pointer to the mjsPair structure to populate.
     */

    void ExportTo(mjsPair* Element);

    /**
     * @brief Registers this contact pair to the MuJoCo spec.
     * @param Wrapper The spec wrapper instance.
     */
    virtual void RegisterToSpec(class FMujocoSpecWrapper& Wrapper, mjsBody* ParentBody = nullptr) override;

    /** No-op: contact pairs are global static data, no per-instance runtime binding. Required by IMjSpecElement. */
    virtual void Bind(mjModel* model, mjData* data, const FString& Prefix = TEXT("")) override {}

#if WITH_EDITOR
    UFUNCTION()
    TArray<FString> GetGeomOptions() const;
#endif

protected:
    /** @brief Called when the game starts. */
	virtual void BeginPlay() override;
};
