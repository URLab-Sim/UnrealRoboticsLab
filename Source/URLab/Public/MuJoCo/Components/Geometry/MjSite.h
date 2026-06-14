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
#include "MuJoCo/Components/Defaults/MjDefault.h"
#include "MuJoCo/Utils/MjOrientationUtils.h"
#include "MjSite.generated.h"

/**
 * @enum EMjSiteType
 * @brief Defines the geometric shape of a MuJoCo site. Mirrors the
 * geom-subset MuJoCo permits on <site>. Plane and SDF stay omitted
 * (infinite extent / heavy data, not what a site spawn surface
 * wants).
 */
UENUM(BlueprintType)
enum class EMjSiteType : uint8
{
	Sphere UMETA(DisplayName = "Sphere"),
	Capsule UMETA(DisplayName = "Capsule"),
	Ellipsoid UMETA(DisplayName = "Ellipsoid"),
	Cylinder UMETA(DisplayName = "Cylinder"),
	Box UMETA(DisplayName = "Box"),
	Mesh UMETA(DisplayName = "Mesh"),
	Hfield UMETA(DisplayName = "Hfield"),
};

/**
 * @class UMjSite
 * @brief Represents a MuJoCo site element within Unreal Engine.
 *
 * Sites are interesting locations on a body, used for sensor attachment, constraint definition, or visualization.
 * This component mirrors the `site` element in MuJoCo XML.
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class URLAB_API UMjSite : public UMjComponent
{
	GENERATED_BODY()

public:
	// --- CODEGEN_PROPERTIES_START ---
	UPROPERTY(EditAnywhere, Category = "MuJoCo|Site|Spatial Pose", meta = (InlineEditConditionToggle))
	bool bOverride_Pos = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Site|Spatial Pose", meta = (EditCondition = "false", EditConditionHides))
	FVector Pos = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, Category = "MuJoCo|Site|Orientation", meta = (InlineEditConditionToggle))
	bool bOverride_Quat = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Site|Orientation", meta = (EditCondition = "false", EditConditionHides))
	FQuat Quat = FQuat::Identity;

	UPROPERTY(EditAnywhere, Category = "MuJoCo|Site", meta = (InlineEditConditionToggle))
	bool bOverride_group = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Site", meta = (EditCondition = "bOverride_group"))
	int32 group = 0;

	UPROPERTY(EditAnywhere, Category = "MuJoCo|Site", meta = (InlineEditConditionToggle))
	bool bOverride_material = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Site", meta = (EditCondition = "bOverride_material"))
	FString material = TEXT("");

	UPROPERTY(EditAnywhere, Category = "MuJoCo|Site", meta = (InlineEditConditionToggle))
	bool bOverride_size = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Site", meta = (EditCondition = "false", EditConditionHides))
	TArray<float> size = {};

	UPROPERTY(EditAnywhere, Category = "MuJoCo|Site", meta = (InlineEditConditionToggle))
	bool bOverride_rgba = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Site", meta = (EditCondition = "bOverride_rgba"))
	FLinearColor rgba = FLinearColor::White;
	// --- CODEGEN_PROPERTIES_END ---

	/**
	 * @brief Exports properties to a MuJoCo spec site structure.
	 * @param site Pointer to the target mjsSite structure.
	 * @param def Optional default structure for optimized export.
	 */
	void ExportTo(mjsSite* Element, mjsDefault* def = nullptr);

	/**
	 * @brief Binds this component to the live MuJoCo simulation.
	 */
	virtual void Bind(mjModel* model, mjData* data, const FString& Prefix = TEXT("")) override;

	/**
	 * @brief Registers this site to the MuJoCo spec.
	 * @param Wrapper The spec wrapper instance.
	 * @param ParentBody The parent body to attach to.
	 */
	virtual void RegisterToSpec(class FMujocoSpecWrapper& Wrapper, mjsBody* ParentBody) override;

	/**
	 * @brief Imports properties from a MuJoCo XML node.
	 * @param Node Pointer to the XML node.
	 */
	void ImportFromXml(const class FXmlNode* Node, const struct FMjCompilerSettings& CompilerSettings);

public:
	/** @brief Default constructor. */
	UMjSite();

public:
	// --- Geometric Properties ---
	/** @brief The geometric shape type of the site. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Site")
	EMjSiteType Type = EMjSiteType::Sphere;

	// size is codegen-owned (TArray<float>) — see CODEGEN_PROPERTIES block.
	// Interpretation depends on Type (sphere: size[0]; capsule: [0]+[1]; box: [0..2]).

	/** @brief Optional MuJoCo class name to inherit defaults from (string fallback). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Site", meta = (GetOptions = "GetDefaultClassOptions"))
	FString MjClassName;

#if WITH_EDITOR
	UFUNCTION()
	TArray<FString> GetDefaultClassOptions() const;
#endif

	/** @brief Reference to a UMjDefault component for default class inheritance. Hidden from the
	 *  Details panel — synced from MjClassName via the editor dropdown. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Site",
		meta = (EditCondition = "false", EditConditionHides))
	UMjDefault* DefaultClass = nullptr;

	virtual FString GetMjClassName() const override
	{
		return MjClassName;
	}

	// --- Visuals ---

	/** @brief The runtime view of the MuJoCo site. Valid only after Bind() is called. */
	SiteView m_SiteView;

	/** @brief Semantic accessor for raw MuJoCo data and helper methods. */
	SiteView& GetMj() { return m_SiteView; }
	const SiteView& GetMj() const { return m_SiteView; }
};

// --- Multi-UCLASS subclasses --------------------------------------------------
// One UCLASS per MJCF site type, declared in this header so the Blueprint
// "Add Component" picker lists each as a distinct item (e.g. "MuJoCo Box Site")
// rather than forcing the user to spawn a generic UMjSite and toggle Type.
// All schema attrs (group/material/rgba/Pos/Quat/size) live on the base
// UMjSite via codegen — the subclasses only set the Type enum.

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent, DisplayName = "MuJoCo Box Site"))
class URLAB_API UMjBoxSite : public UMjSite
{
	GENERATED_BODY()
public:
	UMjBoxSite();
};

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent, DisplayName = "MuJoCo Sphere Site"))
class URLAB_API UMjSphereSite : public UMjSite
{
	GENERATED_BODY()
public:
	UMjSphereSite();
};

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent, DisplayName = "MuJoCo Capsule Site"))
class URLAB_API UMjCapsuleSite : public UMjSite
{
	GENERATED_BODY()
public:
	UMjCapsuleSite();
};

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent, DisplayName = "MuJoCo Cylinder Site"))
class URLAB_API UMjCylinderSite : public UMjSite
{
	GENERATED_BODY()
public:
	UMjCylinderSite();
};

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent, DisplayName = "MuJoCo Ellipsoid Site"))
class URLAB_API UMjEllipsoidSite : public UMjSite
{
	GENERATED_BODY()
public:
	UMjEllipsoidSite();
};
