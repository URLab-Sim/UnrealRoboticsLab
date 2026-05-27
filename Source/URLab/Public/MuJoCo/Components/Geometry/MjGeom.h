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
#include "MuJoCo/Components/Defaults/MjDefault.h"
#include "mujoco/mujoco.h"
#include "MuJoCo/Utils/MjBind.h"
#include "MuJoCo/Components/MjComponent.h"
#include "MuJoCo/Utils/MjOrientationUtils.h"
#include "MjGeom.generated.h"




/**
 * @enum EMjGeomType
 * @brief Defines the geometric primitive type for the MuJoCo geom.
 */
UENUM(BlueprintType)
enum class EMjGeomType : uint8
{
	Plane		UMETA(DisplayName = "Plane"),
	Hfield		UMETA(DisplayName = "Hfield"),
	Sphere		UMETA(DisplayName = "Sphere"),
	Capsule		UMETA(DisplayName = "Capsule"),
	Ellipsoid	UMETA(DisplayName = "Ellipsoid"),
	Cylinder	UMETA(DisplayName = "Cylinder"),
	Box			UMETA(DisplayName = "Box"),
	Mesh		UMETA(DisplayName = "Mesh"),
	SDF			UMETA(DisplayName = "SDF")
};

/**
 * @enum EMjGeomInertia
 * @brief MJCF ``<geom shellinertia="...">``. Mirrors MuJoCo's mjtGeomInertia.
 *
 *  - Volume: inertia computed from solid volume (MuJoCo default).
 *  - Shell:  inertia computed from a thin shell (mass distributed on surface).
 */
UENUM(BlueprintType)
enum class EMjGeomInertia : uint8
{
	Volume UMETA(DisplayName = "Volume"),
	Shell  UMETA(DisplayName = "Shell"),
};

/**
 * @enum EMjFluidShape
 * @brief MJCF ``<geom fluidshape="...">``. Selects the ellipsoid-fluid
 *        interaction model in MuJoCo. Stored as 0.0/1.0 in
 *        mjsGeom.fluid_ellipsoid.
 *
 *  - None:      no ellipsoid fluid model (MuJoCo default).
 *  - Ellipsoid: enable the ellipsoid-fluid interaction model.
 */
UENUM(BlueprintType)
enum class EMjFluidShape : uint8
{
	None      UMETA(DisplayName = "None"),
	Ellipsoid UMETA(DisplayName = "Ellipsoid"),
};

/**
 * @class UMjGeom
 * @brief Component representing a collision geometry (geom) in MuJoCo.
 * 
 * Geoms define the shape and collision properties of a body.
 * This class corresponds to the `geom` element in MuJoCo XML.
 */
// ...

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class URLAB_API UMjGeom : public UMjComponent
{
	GENERATED_BODY()

public:
    // --- CODEGEN_PROPERTIES_START ---
    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjGeom|Spatial Pose", meta=(InlineEditConditionToggle))
    bool bOverride_Pos = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjGeom|Spatial Pose", meta=(EditCondition="bOverride_Pos"))
    FVector Pos = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjGeom|Orientation", meta=(InlineEditConditionToggle))
    bool bOverride_Quat = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjGeom|Orientation", meta=(EditCondition="bOverride_Quat"))
    FQuat Quat = FQuat::Identity;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjGeom", meta=(InlineEditConditionToggle))
    bool bOverride_contype = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjGeom", meta=(EditCondition="bOverride_contype"))
    int32 contype = 0;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjGeom", meta=(InlineEditConditionToggle))
    bool bOverride_conaffinity = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjGeom", meta=(EditCondition="bOverride_conaffinity"))
    int32 conaffinity = 0;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjGeom", meta=(InlineEditConditionToggle))
    bool bOverride_condim = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjGeom", meta=(EditCondition="bOverride_condim"))
    int32 condim = 0;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjGeom", meta=(InlineEditConditionToggle))
    bool bOverride_group = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjGeom", meta=(EditCondition="bOverride_group"))
    int32 group = 0;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjGeom", meta=(InlineEditConditionToggle))
    bool bOverride_priority = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjGeom", meta=(EditCondition="bOverride_priority"))
    int32 priority = 0;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjGeom", meta=(InlineEditConditionToggle))
    bool bOverride_size = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjGeom", meta=(EditCondition="bOverride_size"))
    TArray<float> size = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjGeom", meta=(InlineEditConditionToggle))
    bool bOverride_material = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjGeom", meta=(EditCondition="bOverride_material"))
    FString material = TEXT("");

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjGeom", meta=(InlineEditConditionToggle))
    bool bOverride_friction = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjGeom", meta=(EditCondition="bOverride_friction"))
    TArray<float> friction = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjGeom", meta=(InlineEditConditionToggle))
    bool bOverride_mass = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjGeom", meta=(EditCondition="bOverride_mass"))
    float mass = 0.0f;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjGeom", meta=(InlineEditConditionToggle))
    bool bOverride_density = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjGeom", meta=(EditCondition="bOverride_density"))
    float density = 0.0f;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjGeom", meta=(InlineEditConditionToggle))
    bool bOverride_solmix = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjGeom", meta=(EditCondition="bOverride_solmix"))
    float solmix = 0.0f;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjGeom", meta=(InlineEditConditionToggle))
    bool bOverride_solref = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjGeom", meta=(EditCondition="bOverride_solref"))
    TArray<float> solref = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjGeom", meta=(InlineEditConditionToggle))
    bool bOverride_solimp = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjGeom", meta=(EditCondition="bOverride_solimp"))
    TArray<float> solimp = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjGeom", meta=(InlineEditConditionToggle))
    bool bOverride_margin = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjGeom", meta=(EditCondition="bOverride_margin"))
    float margin = 0.0f;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjGeom", meta=(InlineEditConditionToggle))
    bool bOverride_gap = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjGeom", meta=(EditCondition="bOverride_gap"))
    float gap = 0.0f;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjGeom", meta=(InlineEditConditionToggle))
    bool bOverride_hfield = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjGeom", meta=(EditCondition="bOverride_hfield"))
    FString hfield = TEXT("");

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjGeom", meta=(InlineEditConditionToggle))
    bool bOverride_mesh = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjGeom", meta=(EditCondition="bOverride_mesh"))
    FString mesh = TEXT("");

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjGeom", meta=(InlineEditConditionToggle))
    bool bOverride_fitscale = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjGeom", meta=(EditCondition="bOverride_fitscale"))
    bool fitscale = false;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjGeom", meta=(InlineEditConditionToggle))
    bool bOverride_rgba = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjGeom", meta=(EditCondition="bOverride_rgba"))
    FLinearColor rgba = FLinearColor::White;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjGeom", meta=(InlineEditConditionToggle))
    bool bOverride_fluidcoef = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjGeom", meta=(EditCondition="bOverride_fluidcoef"))
    TArray<float> fluidcoef = {};
    // --- CODEGEN_PROPERTIES_END ---

     UMjGeom();

    virtual void ExportTo(mjsGeom* Element, mjsDefault* Default = nullptr);

    virtual void Bind(mjModel* Model, mjData* Data, const FString& Prefix = TEXT("")) override;
    
    virtual void ImportFromXml(const class FXmlNode* Node, const struct FMjCompilerSettings& CompilerSettings);

    /** 
     * @brief Synchronizes the Unreal Component transform (Scale/Location/Rotation) 
     * from the live MuJoCo view. Called after successful Bind().
     */
    virtual void SyncUnrealTransformFromMj();
    
    /** @brief Sets visibility for this geom and its child visual components. */
    virtual void SetGeomVisibility(bool bNewVisibility);

#if WITH_EDITOR
    virtual void PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

    /** @brief Returns the built-in visualizer mesh if it exists. */
    virtual class UStaticMeshComponent* GetVisualizerMesh() const { return nullptr; }

    /** @brief Apply a material to the visual mesh. Override in subclasses with direct member access. */
    virtual void ApplyOverrideMaterial(class UMaterialInterface* Material);

    /** @brief The runtime view of the MuJoCo geom. Valid only after Bind() is called. */
    GeomView m_GeomView;

    /** @brief Semantic accessor for raw MuJoCo data and helper methods. */
    GeomView& GetMj() { return m_GeomView; }
    const GeomView& GetMj() const { return m_GeomView; }

    // --- Runtime Accessors ---

    /** 
     * @brief Updates the Unreal SceneComponent transform to match the MuJoCo simulation state.
     * Call this in Tick if you want the visual to follow the physics.
     */
    UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
    void UpdateGlobalTransform();

    /** @brief Gets the current world location from MuJoCo. */
    UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
    FVector GetWorldLocation() const;

    /** 
     * @brief Sets the sliding friction at runtime.
     * @param NewFriction The new friction value (only the first dimension is usually used for sliding).
     */
    UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
    void SetFriction(float NewFriction);

public:
	// --- Geometric Properties ---
    /** @brief The geometric shape type of the geom. */
    UPROPERTY(EditAnywhere, Category = "MuJoCo|Geom", meta=(InlineEditConditionToggle))
    bool bOverride_Type = false;
	
	

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Geom", meta=(EditCondition="bOverride_Type"))
	EMjGeomType Type = EMjGeomType::Sphere;

	// Hand-declared because the UPROPERTY type is a URLab enum. Codegen owns
	// the XML "shellinertia" attr <-> EMjGeomInertia mapping + write to
	// mjsGeom.typeinertia via xml_enum_attrs in codegen_rules.json. Default
	// = Volume (matches MuJoCo's mjINERTIA_VOLUME).
	UPROPERTY(EditAnywhere, Category = "MuJoCo|Geom", meta=(InlineEditConditionToggle))
	bool bOverride_ShellInertia = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Geom",
		meta=(EditCondition="bOverride_ShellInertia"))
	EMjGeomInertia ShellInertia = EMjGeomInertia::Volume;

	// Hand-declared because the UPROPERTY type is a URLab enum. Codegen owns
	// the XML "fluidshape" attr <-> EMjFluidShape mapping + write to
	// mjsGeom.fluid_ellipsoid via xml_enum_attrs. Default = None (no
	// ellipsoid-fluid interaction).
	UPROPERTY(EditAnywhere, Category = "MuJoCo|Geom", meta=(InlineEditConditionToggle))
	bool bOverride_FluidShape = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Geom",
		meta=(EditCondition="bOverride_FluidShape"))
	EMjFluidShape FluidShape = EMjFluidShape::None;

	/** @brief Codegen-owned ``TArray<float> size`` lives in the CODEGEN_PROPERTIES
	 * block above. Sphere uses size[0] (radius); Capsule uses size[0]/size[1]
	 * (radius / half-length); Box uses size[0..2] (x/y/z half-extents). The
	 * fromto canonicalisation writes into size[1]/[2] depending on shape.
	 */

    /** True when this component was created via ImportFromXml. Lets export
     *  preserve "size inherited from default class" semantics: for an
     *  imported geom with no explicit size attr (bOverride_size=false) we
     *  must NOT derive size from RelativeScale3D on export, otherwise
     *  MuJoCo's default-class inheritance won't fire. */
    UPROPERTY()
    bool bWasImported = false;

    /** @brief Name of the mesh asset if Type is mesh. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|Geom")
	FString MeshName;

    /** @brief Legacy: use complex collision mesh. Hidden — use "Decompose mesh" button instead.
     *  Still used internally by QuickConvert and as import/compile fallback. */
    UPROPERTY(BlueprintReadWrite, Category = "MuJoCo|Geom")
    bool bComplexMeshRequired = false;

    /** @brief CoACD decomposition threshold. Lower = more accurate but more hulls. Range [0.01, 1.0]. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Geom",
        meta=(EditCondition="Type == EMjGeomType::Mesh", EditConditionHides, ClampMin="0.01", ClampMax="1.0",
        ToolTip="CoACD concavity threshold. Lower values produce more accurate (but more) convex hulls. Default 0.05."))
    float CoACDThreshold = 0.05f;

    /** @brief True if this geom was created by CoACD decomposition of another geom. */
    UPROPERTY()
    bool bIsDecomposedHull = false;

    /** @brief True if this geom's mesh was decomposed into hull sub-geoms.
     *  When set, this geom is skipped during RegisterToSpec. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Geom")
    bool bDisabledByDecomposition = false;

    /** @brief Runs CoACD decomposition on this geom's mesh and creates persistent hull sub-geom components. */
    UFUNCTION(BlueprintCallable, Category = "MuJoCo|Geom")
    void DecomposeMesh();

    /** @brief Removes all hull sub-geoms created by decomposition and re-enables this geom. */
    UFUNCTION(BlueprintCallable, Category = "MuJoCo|Geom")
    void RemoveDecomposition();

    /** @brief Optional MuJoCo class name to inherit defaults from (string fallback). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Geom", meta=(GetOptions="GetDefaultClassOptions"))
    FString MjClassName;
    virtual FString GetMjClassName() const override { return MjClassName; }

#if WITH_EDITOR
    UFUNCTION()
    TArray<FString> GetDefaultClassOptions() const;
#endif

    /** @brief Resolves the codegen-owned ``material`` FString through the default class chain. Returns empty if none found. */
    FString GetResolvedMaterialName() const;

	// --- Physics Properties (with override toggles) ---
    














    // --- Contact Dimension ---


	// --- Collision Filtering (with override toggles) ---
    








    /** @brief Number of size parameters explicitly provided in XML.
     *  Now derived from ``size.Num()`` (codegen-owned TArray<float>). */
    int32 SizeParamsCount() const { return size.Num(); }

	// --- Visuals (with override toggle) ---
    

	
    /** @brief Optional Unreal material override for primitive visualizer meshes (Box/Sphere/Cylinder). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Geom|Visual",
        meta=(EditCondition="Type != EMjGeomType::Mesh", EditConditionHides))
    TObjectPtr<UMaterialInterface> OverrideMaterial;

    /** @brief Reference to a UMjDefault component for default class inheritance. Set via detail customization dropdown. */
	UPROPERTY(BlueprintReadWrite, Category = "MuJoCo|Geom")
	UMjDefault* DefaultClass;

    /**
     * @brief Registers this geom to the MuJoCo spec.
     * @param Wrapper The spec wrapper instance.
     * @param ParentBody The parent body to attach to.
     */
    virtual void RegisterToSpec(class FMujocoSpecWrapper& Wrapper, mjsBody* ParentBody) override;

};
