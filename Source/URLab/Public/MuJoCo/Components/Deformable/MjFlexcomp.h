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
#include "MuJoCo/Components/MjComponent.h"
#include <mujoco/mjspec.h>
#include <mujoco/mujoco.h>
#include "MuJoCo/Utils/MjOrientationUtils.h"
#include "MjFlexcomp.generated.h"

class UDynamicMeshComponent;

UENUM(BlueprintType)
enum class EMjFlexcompType : uint8
{
    Grid       UMETA(DisplayName = "Grid"),
    Box        UMETA(DisplayName = "Box"),
    Cylinder   UMETA(DisplayName = "Cylinder"),
    Ellipsoid  UMETA(DisplayName = "Ellipsoid"),
    Square     UMETA(DisplayName = "Square"),
    Disc       UMETA(DisplayName = "Disc"),
    Circle     UMETA(DisplayName = "Circle"),
    Mesh       UMETA(DisplayName = "Mesh"),
    Gmsh       UMETA(DisplayName = "GMSH (.msh)"),
    Direct     UMETA(DisplayName = "Direct (manual point + element)"),
};

UENUM(BlueprintType)
enum class EMjFlexcompDof : uint8
{
    Full       UMETA(DisplayName = "Full"),
    Radial     UMETA(DisplayName = "Radial"),
    Trilinear  UMETA(DisplayName = "Trilinear"),
    Quadratic  UMETA(DisplayName = "Quadratic"),
    TwoD       UMETA(DisplayName = "2D"),
};

/**
 * @class UMjFlexcomp
 * @brief Component representing a MuJoCo flexcomp deformable body.
 *
 * Flexcomp generates a deformable soft body: ropes (dim=1), cloth (dim=2),
 * or volumetric (dim=3). At spec registration, it expands into a flex element
 * plus child bodies with slider joints for non-pinned vertices.
 */
UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class URLAB_API UMjFlexcomp : public UMjComponent
{
    GENERATED_BODY()

public:
    // --- CODEGEN_PROPERTIES_START ---
    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjFlexcomp|Spatial Pose", meta=(InlineEditConditionToggle))
    bool bOverride_Pos = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjFlexcomp|Spatial Pose", meta=(EditCondition="bOverride_Pos"))
    FVector Pos = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjFlexcomp|Orientation", meta=(InlineEditConditionToggle))
    bool bOverride_Quat = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjFlexcomp|Orientation", meta=(EditCondition="bOverride_Quat"))
    FQuat Quat = FQuat::Identity;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjFlexcomp", meta=(InlineEditConditionToggle))
    bool bOverride_group = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjFlexcomp", meta=(EditCondition="bOverride_group"))
    int32 group = 0;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjFlexcomp", meta=(InlineEditConditionToggle))
    bool bOverride_dim = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjFlexcomp", meta=(EditCondition="bOverride_dim"))
    int32 dim = 0;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjFlexcomp", meta=(InlineEditConditionToggle))
    bool bOverride_count = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjFlexcomp", meta=(EditCondition="bOverride_count"))
    TArray<int32> count = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjFlexcomp", meta=(InlineEditConditionToggle))
    bool bOverride_spacing = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjFlexcomp", meta=(EditCondition="bOverride_spacing"))
    TArray<float> spacing = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjFlexcomp", meta=(InlineEditConditionToggle))
    bool bOverride_radius = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjFlexcomp", meta=(EditCondition="bOverride_radius"))
    float radius = 0.0f;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjFlexcomp", meta=(InlineEditConditionToggle))
    bool bOverride_rigid = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjFlexcomp", meta=(EditCondition="bOverride_rigid"))
    bool rigid = false;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjFlexcomp", meta=(InlineEditConditionToggle))
    bool bOverride_mass = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjFlexcomp", meta=(EditCondition="bOverride_mass"))
    float mass = 0.0f;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjFlexcomp", meta=(InlineEditConditionToggle))
    bool bOverride_inertiabox = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjFlexcomp", meta=(EditCondition="bOverride_inertiabox"))
    float inertiabox = 0.0f;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjFlexcomp", meta=(InlineEditConditionToggle))
    bool bOverride_scale = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjFlexcomp", meta=(EditCondition="bOverride_scale"))
    TArray<float> scale = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjFlexcomp", meta=(InlineEditConditionToggle))
    bool bOverride_file = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjFlexcomp", meta=(EditCondition="bOverride_file"))
    FString file = TEXT("");

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjFlexcomp", meta=(InlineEditConditionToggle))
    bool bOverride_point = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjFlexcomp", meta=(EditCondition="bOverride_point"))
    TArray<float> point = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjFlexcomp", meta=(InlineEditConditionToggle))
    bool bOverride_element = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjFlexcomp", meta=(EditCondition="bOverride_element"))
    TArray<int32> element = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjFlexcomp", meta=(InlineEditConditionToggle))
    bool bOverride_texcoord = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjFlexcomp", meta=(EditCondition="bOverride_texcoord"))
    TArray<float> texcoord = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjFlexcomp", meta=(InlineEditConditionToggle))
    bool bOverride_material = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjFlexcomp", meta=(EditCondition="bOverride_material"))
    FString material = TEXT("");

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjFlexcomp", meta=(InlineEditConditionToggle))
    bool bOverride_rgba = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjFlexcomp", meta=(EditCondition="bOverride_rgba"))
    FLinearColor rgba = FLinearColor::White;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjFlexcomp", meta=(InlineEditConditionToggle))
    bool bOverride_flatskin = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjFlexcomp", meta=(EditCondition="bOverride_flatskin"))
    bool flatskin = false;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjFlexcomp", meta=(InlineEditConditionToggle))
    bool bOverride_origin = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjFlexcomp", meta=(EditCondition="bOverride_origin"))
    TArray<float> origin = {};
    // --- CODEGEN_PROPERTIES_END ---

    UMjFlexcomp();

    // --- Core Properties: enum UPROPERTYs only ---
    // The codegen owns every other flexcomp attr (group, dim, count, spacing,
    // scale, radius, mass, file, point, element, texcoord, material, rgba,
    // rigid, flatskin, origin, ...) via the CODEGEN_PROPERTIES block above
    // plus xml_enum_attrs in codegen_rules.json. The two enum UPROPERTYs
    // below are hand-declared because codegen only owns the XML-string <->
    // enum-value bridge, not the UE-enum UPROPERTY itself.

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjFlexcomp", meta=(InlineEditConditionToggle))
    bool bOverride_FlexcompType = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjFlexcomp", meta=(EditCondition="bOverride_FlexcompType"))
    EMjFlexcompType FlexcompType = EMjFlexcompType::Grid;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjFlexcomp", meta=(InlineEditConditionToggle))
    bool bOverride_FlexcompDof = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjFlexcomp", meta=(EditCondition="bOverride_FlexcompDof"))
    EMjFlexcompDof FlexcompDof = EMjFlexcompDof::Full;

    // --- Contact Properties ---

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Flexcomp|Contact", meta=(InlineEditConditionToggle))
    bool bOverride_ConType = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Flexcomp|Contact", meta=(EditCondition="bOverride_ConType"))
    int32 ConType = 1;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Flexcomp|Contact", meta=(InlineEditConditionToggle))
    bool bOverride_ConAffinity = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Flexcomp|Contact", meta=(EditCondition="bOverride_ConAffinity"))
    int32 ConAffinity = 1;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Flexcomp|Contact", meta=(InlineEditConditionToggle))
    bool bOverride_ConDim = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Flexcomp|Contact", meta=(EditCondition="bOverride_ConDim"))
    int32 ConDim = 3;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Flexcomp|Contact", meta=(InlineEditConditionToggle))
    bool bOverride_Priority = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Flexcomp|Contact", meta=(EditCondition="bOverride_Priority"))
    int32 Priority = 0;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Flexcomp|Contact", meta=(InlineEditConditionToggle))
    bool bOverride_Margin = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Flexcomp|Contact", meta=(EditCondition="bOverride_Margin"))
    float Margin = 0.0f;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Flexcomp|Contact", meta=(InlineEditConditionToggle))
    bool bOverride_Gap = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Flexcomp|Contact", meta=(EditCondition="bOverride_Gap"))
    float Gap = 0.0f;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Flexcomp|Contact", meta=(InlineEditConditionToggle))
    bool bOverride_SelfCollide = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Flexcomp|Contact", meta=(EditCondition="bOverride_SelfCollide"))
    int32 SelfCollide = 0;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Flexcomp|Contact", meta=(InlineEditConditionToggle))
    bool bOverride_Internal = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Flexcomp|Contact", meta=(EditCondition="bOverride_Internal"))
    bool bInternal = false;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Flexcomp|Contact", meta=(InlineEditConditionToggle))
    bool bOverride_Friction = false;

    /** Contact friction: {tangential, torsional, rolling}. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Flexcomp|Contact", meta=(EditCondition="bOverride_Friction"))
    TArray<float> Friction;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Flexcomp|Contact", meta=(InlineEditConditionToggle))
    bool bOverride_SolMix = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Flexcomp|Contact", meta=(EditCondition="bOverride_SolMix"))
    float SolMix = 1.0f;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Flexcomp|Contact", meta=(InlineEditConditionToggle))
    bool bOverride_ContactSolRef = false;

    /** Contact solref: {time_constant, damping_ratio}. Leave empty for MuJoCo defaults. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Flexcomp|Contact", meta=(EditCondition="bOverride_ContactSolRef"))
    TArray<float> ContactSolRef;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Flexcomp|Contact", meta=(InlineEditConditionToggle))
    bool bOverride_ContactSolImp = false;

    /** Contact solimp: 5-element impedance vector. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Flexcomp|Contact", meta=(EditCondition="bOverride_ContactSolImp"))
    TArray<float> ContactSolImp;

    // --- Edge Properties ---

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Flexcomp|Edge", meta=(InlineEditConditionToggle))
    bool bOverride_EdgeStiffness = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Flexcomp|Edge", meta=(EditCondition="bOverride_EdgeStiffness"))
    float EdgeStiffness = 0.0f;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Flexcomp|Edge", meta=(InlineEditConditionToggle))
    bool bOverride_EdgeDamping = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Flexcomp|Edge", meta=(EditCondition="bOverride_EdgeDamping"))
    float EdgeDamping = 0.0f;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Flexcomp|Edge", meta=(InlineEditConditionToggle))
    bool bOverride_EdgeEquality = false;

    /** Emit equality constraints per edge. Pairs with EdgeSolRef/EdgeSolImp. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Flexcomp|Edge", meta=(EditCondition="bOverride_EdgeEquality"))
    bool bEdgeEquality = false;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Flexcomp|Edge", meta=(InlineEditConditionToggle))
    bool bOverride_EdgeSolRef = false;

    /** Edge solref: {time_constant, damping_ratio}. Only has effect when bEdgeEquality=true. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Flexcomp|Edge", meta=(EditCondition="bOverride_EdgeSolRef"))
    TArray<float> EdgeSolRef;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Flexcomp|Edge", meta=(InlineEditConditionToggle))
    bool bOverride_EdgeSolImp = false;

    /** Edge solimp: 5-element impedance vector. Only has effect when bEdgeEquality=true. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Flexcomp|Edge", meta=(EditCondition="bOverride_EdgeSolImp"))
    TArray<float> EdgeSolImp;

    // --- Elasticity Properties ---

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Flexcomp|Elasticity", meta=(InlineEditConditionToggle))
    bool bOverride_Young = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Flexcomp|Elasticity", meta=(EditCondition="bOverride_Young"))
    float Young = 0.0f;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Flexcomp|Elasticity", meta=(InlineEditConditionToggle))
    bool bOverride_Poisson = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Flexcomp|Elasticity", meta=(EditCondition="bOverride_Poisson"))
    float Poisson = 0.0f;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Flexcomp|Elasticity", meta=(InlineEditConditionToggle))
    bool bOverride_Damping = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Flexcomp|Elasticity", meta=(EditCondition="bOverride_Damping"))
    float Damping = 0.0f;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Flexcomp|Elasticity", meta=(InlineEditConditionToggle))
    bool bOverride_Thickness = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Flexcomp|Elasticity", meta=(EditCondition="bOverride_Thickness"))
    float Thickness = 0.0f;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Flexcomp|Elasticity", meta=(InlineEditConditionToggle))
    bool bOverride_Elastic2D = false;

    /** @brief 2D passive forces mode: 0=none, 1=bending, 2=stretching, 3=both. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Flexcomp|Elasticity", meta=(EditCondition="bOverride_Elastic2D"))
    int32 Elastic2D = 0;

    // --- Pin Data ---

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Flexcomp|Pin")
    TArray<int32> PinIds;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Flexcomp|Pin")
    TArray<int32> PinGridRange;

    // --- Import/Export/Bind ---

    void ImportFromXml(const class FXmlNode* Node, const struct FMjCompilerSettings& CompilerSettings = FMjCompilerSettings{});

    /**
     * @brief Build the `<flexcomp ...>` attribute string from codegen-owned
     *        UPROPERTYs. Method body is codegen-emitted between
     *        ``CODEGEN_XML_PASSTHROUGH_*`` markers in MjFlexcomp.cpp so new
     *        schema attrs flow through without hand-edits. Called by
     *        ``BuildFlexcompXml`` which still hand-rolls the surrounding
     *        ``<flexcomp>`` tag + sub-elements (``<contact>`` / ``<edge>``
     *        / ``<elasticity>`` / ``<pin>``).
     */
    FString BuildSchemaAttrsXml() const;

    virtual void RegisterToSpec(class FMujocoSpecWrapper& Wrapper, mjsBody* ParentBody = nullptr) override;

    virtual void Bind(mjModel* Model, mjData* Data, const FString& Prefix = TEXT("")) override;

    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
    int32 FlexId = -1;
    int32 FlexVertAdr = 0;
    int32 FlexVertNum = 0;

    /** Set once RegisterToSpec succeeds — prevents double-registration from
     *  both the articulation-level loop and MjBody's child iteration. */
    bool bIsRegistered = false;

    UPROPERTY()
    UDynamicMeshComponent* DynamicMesh = nullptr;

    /** Remap from raw UE vertex index → welded MuJoCo flex vertex index.
     *  Used each tick: raw_pos[i] = flexvert_xpos[RawToWelded[i]]. */
    TArray<int32> RawToWelded;

    /** Number of raw UE vertices. */
    int32 NumRenderVerts = 0;

    void CreateProceduralMesh();
    void UpdateProceduralMesh();

    /** Captures raw UE vertex data (UVs, tangents, indices) from the child
     *  static mesh for rendering, plus builds the welded remap for physics. */
    void CaptureRenderData();

    /** Serializes this flexcomp's properties back to an <flexcomp> MJCF fragment. */
    FString BuildFlexcompXml(const FString& MeshAssetName) const;

    /** Exports the child StaticMeshComponent's mesh to an OBJ file in the wrapper's VFS.
     *  Returns the filename used in the VFS, or empty string on failure.
     *  Also populates RawToWelded, RenderUVs, RenderTangents, RenderTriangles for
     *  the procedural-mesh visualization path. */
    FString ExportMeshToVFS(class FMujocoSpecWrapper& Wrapper);
};
