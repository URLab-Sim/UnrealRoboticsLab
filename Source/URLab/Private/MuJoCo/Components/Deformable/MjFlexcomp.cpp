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

#include "MuJoCo/Components/Deformable/MjFlexcomp.h"
#include "MuJoCo/Core/Spec/MjSpecWrapper.h"
#include "MuJoCo/Utils/MjXmlUtils.h"
#include "MuJoCo/Utils/MjUtils.h"
#include "Utils/URLabLogging.h"
#include "XmlNode.h"
#include "Engine/StaticMesh.h"
#include "Components/StaticMeshComponent.h"
#include "StaticMeshResources.h"
#include "Components/DynamicMeshComponent.h"
#include "UDynamicMesh.h"
#include "DynamicMesh/DynamicMesh3.h"
#include "DynamicMesh/DynamicMeshAttributeSet.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/ScopeTryLock.h"
#include "HAL/FileManager.h"
#include "Interfaces/IPluginManager.h"

UMjFlexcomp::UMjFlexcomp()
{
    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.bStartWithTickEnabled = false;
}

// ============================================================================
// Import
// ============================================================================

void UMjFlexcomp::ImportFromXml(const FXmlNode* Node, const FMjCompilerSettings& CompilerSettings)
{
    if (!Node) return;

    MjXmlUtils::ReadAttrString(Node, TEXT("name"), MjName);

    // --- CODEGEN_IMPORT_START ---
    { // xml_enum: type -> EMjFlexcompType
        FString S = Node->GetAttribute(TEXT("type"));
        S = S.ToLower();
        if      (S == TEXT("grid")) FlexcompType = EMjFlexcompType::Grid;
        else if (S == TEXT("box")) FlexcompType = EMjFlexcompType::Box;
        else if (S == TEXT("cylinder")) FlexcompType = EMjFlexcompType::Cylinder;
        else if (S == TEXT("ellipsoid")) FlexcompType = EMjFlexcompType::Ellipsoid;
        else if (S == TEXT("square")) FlexcompType = EMjFlexcompType::Square;
        else if (S == TEXT("disc")) FlexcompType = EMjFlexcompType::Disc;
        else if (S == TEXT("circle")) FlexcompType = EMjFlexcompType::Circle;
        else if (S == TEXT("mesh")) FlexcompType = EMjFlexcompType::Mesh;
        else if (S == TEXT("gmsh")) FlexcompType = EMjFlexcompType::Gmsh;
        else if (S == TEXT("direct")) FlexcompType = EMjFlexcompType::Direct;
        if (!S.IsEmpty()) bOverride_FlexcompType = true;
    }
    { // xml_enum: dof -> EMjFlexcompDof
        FString S = Node->GetAttribute(TEXT("dof"));
        S = S.ToLower();
        if      (S == TEXT("full")) FlexcompDof = EMjFlexcompDof::Full;
        else if (S == TEXT("radial")) FlexcompDof = EMjFlexcompDof::Radial;
        else if (S == TEXT("trilinear")) FlexcompDof = EMjFlexcompDof::Trilinear;
        else if (S == TEXT("quadratic")) FlexcompDof = EMjFlexcompDof::Quadratic;
        else if (S == TEXT("2d")) FlexcompDof = EMjFlexcompDof::TwoD;
        if (!S.IsEmpty()) bOverride_FlexcompDof = true;
    }
    MjXmlUtils::ReadAttrInt(Node, TEXT("group"), group, bOverride_group);
    MjXmlUtils::ReadAttrInt(Node, TEXT("dim"), dim, bOverride_dim);
    MjXmlUtils::ReadAttrIntArray(Node, TEXT("count"), count, bOverride_count);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("spacing"), spacing, bOverride_spacing);
    MjXmlUtils::ReadAttrFloat(Node, TEXT("radius"), radius, bOverride_radius);
    MjXmlUtils::ReadAttrBool(Node, TEXT("rigid"), rigid, bOverride_rigid);
    MjXmlUtils::ReadAttrFloat(Node, TEXT("mass"), mass, bOverride_mass);
    MjXmlUtils::ReadAttrFloat(Node, TEXT("inertiabox"), inertiabox, bOverride_inertiabox);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("scale"), scale, bOverride_scale);
    if (MjXmlUtils::ReadAttrString(Node, TEXT("file"), file)) bOverride_file = true;
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("point"), point, bOverride_point);
    MjXmlUtils::ReadAttrIntArray(Node, TEXT("element"), element, bOverride_element);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("texcoord"), texcoord, bOverride_texcoord);
    if (MjXmlUtils::ReadAttrString(Node, TEXT("material"), material)) bOverride_material = true;
    MjXmlUtils::ReadAttrColor(Node, TEXT("rgba"), rgba, bOverride_rgba);
    MjXmlUtils::ReadAttrBool(Node, TEXT("flatskin"), flatskin, bOverride_flatskin);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("origin"), origin, bOverride_origin);
    MjUtils::ReadVec3InMeters(Node, TEXT("pos"), Pos, bOverride_Pos);
    { // canonicalize orientation (quat/euler/axisangle/xyaxes/zaxis)
        double TmpQuat[4] = {1.0, 0.0, 0.0, 0.0};
        if (MjOrientationUtils::OrientationToMjQuat(Node, CompilerSettings, TmpQuat))
        {
            Quat = MjUtils::MjToUERotation(TmpQuat);
            bOverride_Quat = true;
        }
    }
    if (bOverride_Pos)  SetRelativeLocation(Pos);
    if (bOverride_Quat) SetRelativeRotation(Quat);
    // --- CODEGEN_IMPORT_END ---

    // Sub-elements — these aren't auto-codegen yet because the codegen
    // doesn't model per-sub-element UPROPERTY groups. Stays hand-rolled.
    for (const FXmlNode* Child : Node->GetChildrenNodes())
    {
        FString ChildTag = Child->GetTag();

        if (ChildTag == TEXT("contact"))
        {
            FString ConTypeStr = Child->GetAttribute(TEXT("contype"));
            if (!ConTypeStr.IsEmpty()) { ConType = FCString::Atoi(*ConTypeStr); bOverride_ConType = true; }

            FString ConAffStr = Child->GetAttribute(TEXT("conaffinity"));
            if (!ConAffStr.IsEmpty()) { ConAffinity = FCString::Atoi(*ConAffStr); bOverride_ConAffinity = true; }

            FString ConDimStr = Child->GetAttribute(TEXT("condim"));
            if (!ConDimStr.IsEmpty()) { ConDim = FCString::Atoi(*ConDimStr); bOverride_ConDim = true; }

            FString PriorityStr = Child->GetAttribute(TEXT("priority"));
            if (!PriorityStr.IsEmpty()) { Priority = FCString::Atoi(*PriorityStr); bOverride_Priority = true; }

            MjXmlUtils::ReadAttrFloat(Child, TEXT("margin"), Margin, bOverride_Margin);
            MjXmlUtils::ReadAttrFloat(Child, TEXT("gap"), Gap, bOverride_Gap);

            FString SelfStr = Child->GetAttribute(TEXT("selfcollide"));
            if (!SelfStr.IsEmpty())
            {
                if (SelfStr == TEXT("none")) SelfCollide = 0;
                else if (SelfStr == TEXT("auto")) SelfCollide = 1;
                else if (SelfStr == TEXT("all")) SelfCollide = 2;
                bOverride_SelfCollide = true;
            }

            FString IntStr = Child->GetAttribute(TEXT("internal"));
            if (!IntStr.IsEmpty()) { bInternal = IntStr.ToBool(); bOverride_Internal = true; }

            MjXmlUtils::ReadAttrFloatArray(Child, TEXT("friction"), Friction, bOverride_Friction);
            MjXmlUtils::ReadAttrFloat     (Child, TEXT("solmix"),   SolMix,   bOverride_SolMix);
            MjXmlUtils::ReadAttrFloatArray(Child, TEXT("solref"),   ContactSolRef, bOverride_ContactSolRef);
            MjXmlUtils::ReadAttrFloatArray(Child, TEXT("solimp"),   ContactSolImp, bOverride_ContactSolImp);
        }
        else if (ChildTag == TEXT("edge"))
        {
            MjXmlUtils::ReadAttrFloat(Child, TEXT("stiffness"), EdgeStiffness, bOverride_EdgeStiffness);
            MjXmlUtils::ReadAttrFloat(Child, TEXT("damping"),   EdgeDamping,   bOverride_EdgeDamping);

            FString EqStr = Child->GetAttribute(TEXT("equality"));
            if (!EqStr.IsEmpty()) { bEdgeEquality = EqStr.ToBool(); bOverride_EdgeEquality = true; }

            MjXmlUtils::ReadAttrFloatArray(Child, TEXT("solref"), EdgeSolRef, bOverride_EdgeSolRef);
            MjXmlUtils::ReadAttrFloatArray(Child, TEXT("solimp"), EdgeSolImp, bOverride_EdgeSolImp);
        }
        else if (ChildTag == TEXT("elasticity"))
        {
            MjXmlUtils::ReadAttrFloat(Child, TEXT("young"), Young, bOverride_Young);
            MjXmlUtils::ReadAttrFloat(Child, TEXT("poisson"), Poisson, bOverride_Poisson);
            MjXmlUtils::ReadAttrFloat(Child, TEXT("damping"), Damping, bOverride_Damping);
            MjXmlUtils::ReadAttrFloat(Child, TEXT("thickness"), Thickness, bOverride_Thickness);

            FString E2DStr = Child->GetAttribute(TEXT("elastic2d"));
            if (!E2DStr.IsEmpty())
            {
                if (E2DStr == TEXT("none"))       Elastic2D = 0;
                else if (E2DStr == TEXT("bend"))  Elastic2D = 1;
                else if (E2DStr == TEXT("stretch")) Elastic2D = 2;
                else if (E2DStr == TEXT("both"))  Elastic2D = 3;
                bOverride_Elastic2D = true;
            }
        }
        else if (ChildTag == TEXT("pin"))
        {
            FString IdStr = Child->GetAttribute(TEXT("id"));
            if (!IdStr.IsEmpty())
            {
                TArray<FString> Parts;
                IdStr.ParseIntoArray(Parts, TEXT(" "), true);
                for (const FString& P : Parts) PinIds.Add(FCString::Atoi(*P));
            }

            FString GridRangeStr = Child->GetAttribute(TEXT("gridrange"));
            if (!GridRangeStr.IsEmpty())
            {
                TArray<FString> Parts;
                GridRangeStr.ParseIntoArray(Parts, TEXT(" "), true);
                for (const FString& P : Parts) PinGridRange.Add(FCString::Atoi(*P));
            }
        }
    }

    UE_LOG(LogURLab, Log, TEXT("[MjFlexcomp] Imported '%s' (dim=%d, count.Num()=%d)"),
        *MjName, dim, count.Num());
}

// ============================================================================
// Mesh Export (for mesh-type flexcomp)
// ============================================================================

FString UMjFlexcomp::ExportMeshToVFS(FMujocoSpecWrapper& Wrapper)
{
    TArray<USceneComponent*> Children;
    GetChildrenComponents(false, Children);

    UStaticMeshComponent* SMC = nullptr;
    for (USceneComponent* Child : Children)
    {
        SMC = Cast<UStaticMeshComponent>(Child);
        if (SMC && SMC->GetStaticMesh()) break;
        SMC = nullptr;
    }

    if (!SMC || !SMC->GetStaticMesh()) return FString();

    UStaticMesh* Mesh = SMC->GetStaticMesh();
    const FStaticMeshLODResources& LOD = Mesh->GetRenderData()->LODResources[0];
    const FStaticMeshVertexBuffer& VB = LOD.VertexBuffers.StaticMeshVertexBuffer;

    // UE splits vertices per-face (for normals/UVs). MuJoCo needs welded
    // vertices for flex. We build a remap table so the visualization can keep
    // UE's per-face UVs/tangents while physics uses welded positions.
    int32 NumRawVerts = LOD.VertexBuffers.PositionVertexBuffer.GetNumVertices();
    NumRenderVerts = NumRawVerts;
    TArray<FVector3f> UniquePositions;
    RawToWelded.SetNum(NumRawVerts);

    const float WeldTolerance = 1e-5f;
    const int32 HashSize = 1 << 14;
    const int32 HashMask = HashSize - 1;
    TArray<TArray<int32>> HashBuckets;
    HashBuckets.SetNum(HashSize);

    auto HashPos = [HashMask](const FVector3f& P)
    {
        uint32 H = (uint32)(P.X * 73856093.f) ^ (uint32)(P.Y * 19349663.f) ^ (uint32)(P.Z * 83492791.f);
        return (int32)(H & HashMask);
    };

    for (int32 i = 0; i < NumRawVerts; i++)
    {
        FVector3f LocalPos = LOD.VertexBuffers.PositionVertexBuffer.VertexPosition(i);
        int32 Bucket = HashPos(LocalPos);
        int32 Found = INDEX_NONE;
        for (int32 C : HashBuckets[Bucket])
        {
            if (UniquePositions[C].Equals(LocalPos, WeldTolerance)) { Found = C; break; }
        }
        if (Found == INDEX_NONE)
        {
            Found = UniquePositions.Add(LocalPos);
            HashBuckets[Bucket].Add(Found);
        }
        RawToWelded[i] = Found;
    }

    FIndexArrayView Indices = LOD.IndexBuffer.GetArrayView();

    // Write OBJ with welded unique positions: UE cm -> MuJoCo m, flip Y, reverse winding.
    FString ObjContent;
    ObjContent.Reserve(UniquePositions.Num() * 40);
    for (const FVector3f& P : UniquePositions)
    {
        ObjContent += FString::Printf(TEXT("v %f %f %f\n"),
            P.X / 100.0f, -P.Y / 100.0f, P.Z / 100.0f);
    }

    for (int32 i = 0; i + 2 < Indices.Num(); i += 3)
    {
        int32 A = RawToWelded[Indices[i]];
        int32 B = RawToWelded[Indices[i + 1]];
        int32 C = RawToWelded[Indices[i + 2]];
        if (A == B || B == C || A == C) continue;
        // OBJ is 1-indexed; swap last two to fix winding after Y-flip.
        ObjContent += FString::Printf(TEXT("f %d %d %d\n"), A + 1, C + 1, B + 1);
    }

    int32 NumVerts = UniquePositions.Num();

    // Write OBJ to a temp file so MuJoCo's parser can load it
    FString FlexName = MjName.IsEmpty() ? GetName() : MjName;
    FString ObjFileName = FString::Printf(TEXT("flexcomp_%s.obj"), *FlexName);
    FString TempDir = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("URLab/FlexcompMeshes"));
    IFileManager::Get().MakeDirectory(*TempDir, true);
    FString FullPath = FPaths::ConvertRelativePathToFull(FPaths::Combine(TempDir, ObjFileName));

    if (!FFileHelper::SaveStringToFile(ObjContent, *FullPath))
    {
        UE_LOG(LogURLab, Warning, TEXT("[MjFlexcomp] Failed to write OBJ to '%s'"), *FullPath);
        return FString();
    }

    // Add to VFS so MuJoCo's XML parser can resolve file="<ObjFileName>"
    FString Dir = FPaths::GetPath(FullPath);
    FString FileName = FPaths::GetCleanFilename(FullPath);
    int Result = mj_addFileVFS(Wrapper.VFS, TCHAR_TO_UTF8(*Dir), TCHAR_TO_UTF8(*FileName));
    if (Result != 0)
    {
        UE_LOG(LogURLab, Warning, TEXT("[MjFlexcomp] mj_addFileVFS returned %d for '%s'"), Result, *FullPath);
    }

    UE_LOG(LogURLab, Log, TEXT("[MjFlexcomp] Exported mesh to VFS: %s (%d verts, %d tris)"),
        *FileName, NumVerts, Indices.Num() / 3);
    return FileName;
}

// ============================================================================
// XML Serialization
// ============================================================================

// Codegen-emitted helper: walks every codegen-owned UPROPERTY on UMjFlexcomp
// and writes `name="value"` XML attr fragments for each one whose
// bOverride_X toggle is true. Body is auto-generated between the
// CODEGEN_XML_PASSTHROUGH markers below; new MJCF attrs flow through with
// zero hand-edits.
FString UMjFlexcomp::BuildSchemaAttrsXml() const
{
    // --- CODEGEN_XML_PASSTHROUGH_START ---
    FString Out;
    if (bOverride_Pos)
    {
        double Tmp[3];
        MjUtils::UEToMjPosition(Pos, Tmp);
        Out += FString::Printf(TEXT(" pos=\"%f %f %f\""), Tmp[0], Tmp[1], Tmp[2]);
    }
    if (bOverride_Quat)
    {
        double Tmp[4];
        MjUtils::UEToMjRotation(Quat, Tmp);
        Out += FString::Printf(TEXT(" quat=\"%f %f %f %f\""), Tmp[0], Tmp[1], Tmp[2], Tmp[3]);
    }
    if (bOverride_FlexcompType)
    {
        if      (FlexcompType == EMjFlexcompType::Grid) Out += TEXT(" type=\"grid\"");
        else if (FlexcompType == EMjFlexcompType::Box) Out += TEXT(" type=\"box\"");
        else if (FlexcompType == EMjFlexcompType::Cylinder) Out += TEXT(" type=\"cylinder\"");
        else if (FlexcompType == EMjFlexcompType::Ellipsoid) Out += TEXT(" type=\"ellipsoid\"");
        else if (FlexcompType == EMjFlexcompType::Square) Out += TEXT(" type=\"square\"");
        else if (FlexcompType == EMjFlexcompType::Disc) Out += TEXT(" type=\"disc\"");
        else if (FlexcompType == EMjFlexcompType::Circle) Out += TEXT(" type=\"circle\"");
        else if (FlexcompType == EMjFlexcompType::Mesh) Out += TEXT(" type=\"mesh\"");
        else if (FlexcompType == EMjFlexcompType::Gmsh) Out += TEXT(" type=\"gmsh\"");
        else if (FlexcompType == EMjFlexcompType::Direct) Out += TEXT(" type=\"direct\"");
    }
    if (bOverride_FlexcompDof)
    {
        if      (FlexcompDof == EMjFlexcompDof::Full) Out += TEXT(" dof=\"full\"");
        else if (FlexcompDof == EMjFlexcompDof::Radial) Out += TEXT(" dof=\"radial\"");
        else if (FlexcompDof == EMjFlexcompDof::Trilinear) Out += TEXT(" dof=\"trilinear\"");
        else if (FlexcompDof == EMjFlexcompDof::Quadratic) Out += TEXT(" dof=\"quadratic\"");
        else if (FlexcompDof == EMjFlexcompDof::TwoD) Out += TEXT(" dof=\"2d\"");
    }
    if (bOverride_group) Out += FString::Printf(TEXT(" group=\"%d\""), group);
    if (bOverride_dim) Out += FString::Printf(TEXT(" dim=\"%d\""), dim);
    if (bOverride_count && count.Num() > 0)
    {
        Out += TEXT(" count=\"");
        for (int32 i = 0; i < count.Num(); ++i)
        {
            if (i > 0) Out += TEXT(" ");
            Out += FString::Printf(TEXT("%d"), count[i]);
        }
        Out += TEXT("\"");
    }
    if (bOverride_spacing && spacing.Num() > 0)
    {
        Out += TEXT(" spacing=\"");
        for (int32 i = 0; i < spacing.Num(); ++i)
        {
            if (i > 0) Out += TEXT(" ");
            Out += FString::Printf(TEXT("%f"), spacing[i]);
        }
        Out += TEXT("\"");
    }
    if (bOverride_radius) Out += FString::Printf(TEXT(" radius=\"%f\""), radius);
    if (bOverride_rigid) Out += FString::Printf(TEXT(" rigid=\"%s\""), rigid ? TEXT("true") : TEXT("false"));
    if (bOverride_mass) Out += FString::Printf(TEXT(" mass=\"%f\""), mass);
    if (bOverride_inertiabox) Out += FString::Printf(TEXT(" inertiabox=\"%f\""), inertiabox);
    if (bOverride_scale && scale.Num() > 0)
    {
        Out += TEXT(" scale=\"");
        for (int32 i = 0; i < scale.Num(); ++i)
        {
            if (i > 0) Out += TEXT(" ");
            Out += FString::Printf(TEXT("%f"), scale[i]);
        }
        Out += TEXT("\"");
    }
    if (bOverride_file && !file.IsEmpty()) Out += FString::Printf(TEXT(" file=\"%s\""), *file);
    if (bOverride_point && point.Num() > 0)
    {
        Out += TEXT(" point=\"");
        for (int32 i = 0; i < point.Num(); ++i)
        {
            if (i > 0) Out += TEXT(" ");
            Out += FString::Printf(TEXT("%f"), point[i]);
        }
        Out += TEXT("\"");
    }
    if (bOverride_element && element.Num() > 0)
    {
        Out += TEXT(" element=\"");
        for (int32 i = 0; i < element.Num(); ++i)
        {
            if (i > 0) Out += TEXT(" ");
            Out += FString::Printf(TEXT("%d"), element[i]);
        }
        Out += TEXT("\"");
    }
    if (bOverride_texcoord && texcoord.Num() > 0)
    {
        Out += TEXT(" texcoord=\"");
        for (int32 i = 0; i < texcoord.Num(); ++i)
        {
            if (i > 0) Out += TEXT(" ");
            Out += FString::Printf(TEXT("%f"), texcoord[i]);
        }
        Out += TEXT("\"");
    }
    if (bOverride_material && !material.IsEmpty()) Out += FString::Printf(TEXT(" material=\"%s\""), *material);
    if (bOverride_rgba) Out += FString::Printf(TEXT(" rgba=\"%f %f %f %f\""), rgba.R, rgba.G, rgba.B, rgba.A);
    if (bOverride_flatskin) Out += FString::Printf(TEXT(" flatskin=\"%s\""), flatskin ? TEXT("true") : TEXT("false"));
    if (bOverride_origin && origin.Num() > 0)
    {
        Out += TEXT(" origin=\"");
        for (int32 i = 0; i < origin.Num(); ++i)
        {
            if (i > 0) Out += TEXT(" ");
            Out += FString::Printf(TEXT("%f"), origin[i]);
        }
        Out += TEXT("\"");
    }
    return Out;
    // --- CODEGEN_XML_PASSTHROUGH_END ---
}

FString UMjFlexcomp::BuildFlexcompXml(const FString& MeshAssetName) const
{
    FString FlexName = MjName.IsEmpty() ? GetName() : MjName;

    // Build the `<flexcomp ...>` attribute string from codegen-owned
    // UPROPERTYs first, then layer on a few attrs the codegen doesn't
    // know about: the element name (we own the MjName resolution), the
    // pos derived from the UE component transform (the SceneComponent
    // location, not the UPROPERTY), and the mesh `file=` substitution
    // (we just exported the OBJ to the VFS — name comes from the caller).
    FString Attrs = FString::Printf(TEXT(" name=\"%s\""), *FlexName);
    Attrs += BuildSchemaAttrsXml();

    // Derive pos from the UE component transform, NOT the Pos UPROPERTY.
    // BuildSchemaAttrsXml will emit Pos if bOverride_Pos is set, but the
    // UE transform is the authoritative source for placement at runtime.
    {
        FVector UEPos = GetRelativeLocation();
        double MjPos[3];
        MjUtils::UEToMjPosition(UEPos, MjPos);
        // Strip any pos=".." the codegen helper wrote — the UE transform wins.
        int32 PosIdx = Attrs.Find(TEXT(" pos=\""), ESearchCase::CaseSensitive);
        if (PosIdx >= 0)
        {
            int32 EndIdx = Attrs.Find(TEXT("\""),
                ESearchCase::CaseSensitive, ESearchDir::FromStart, PosIdx + 7);
            if (EndIdx > PosIdx) { Attrs.RemoveAt(PosIdx, EndIdx - PosIdx + 1); }
        }
        Attrs += FString::Printf(TEXT(" pos=\"%f %f %f\""), MjPos[0], MjPos[1], MjPos[2]);
    }

    // Mesh-type flexcomp uses the OBJ that ExportMeshToVFS just registered.
    // Overrides any file=".." the codegen helper produced.
    if (FlexcompType == EMjFlexcompType::Mesh && !MeshAssetName.IsEmpty())
    {
        int32 FileIdx = Attrs.Find(TEXT(" file=\""), ESearchCase::CaseSensitive);
        if (FileIdx >= 0)
        {
            int32 EndIdx = Attrs.Find(TEXT("\""),
                ESearchCase::CaseSensitive, ESearchDir::FromStart, FileIdx + 8);
            if (EndIdx > FileIdx) { Attrs.RemoveAt(FileIdx, EndIdx - FileIdx + 1); }
        }
        Attrs += FString::Printf(TEXT(" file=\"%s\""), *MeshAssetName);
    }

    // Sub-elements — emit each only when at least one attribute is overridden,
    // and only emit the overridden attributes inside.
    FString SubElements;

    // Small helper: join a float array into a space-separated string.
    auto JoinFloats = [](const TArray<float>& Arr) -> FString
    {
        FString Out;
        for (int32 i = 0; i < Arr.Num(); ++i)
        {
            if (i > 0) Out += TEXT(" ");
            Out += FString::Printf(TEXT("%f"), Arr[i]);
        }
        return Out;
    };

    // <contact>
    {
        FString ContactAttrs;
        if (bOverride_ConType)     ContactAttrs += FString::Printf(TEXT(" contype=\"%d\""), ConType);
        if (bOverride_ConAffinity) ContactAttrs += FString::Printf(TEXT(" conaffinity=\"%d\""), ConAffinity);
        if (bOverride_ConDim)      ContactAttrs += FString::Printf(TEXT(" condim=\"%d\""), ConDim);
        if (bOverride_Priority)    ContactAttrs += FString::Printf(TEXT(" priority=\"%d\""), Priority);
        if (bOverride_Margin)      ContactAttrs += FString::Printf(TEXT(" margin=\"%f\""), Margin);
        if (bOverride_Gap)         ContactAttrs += FString::Printf(TEXT(" gap=\"%f\""), Gap);
        if (bOverride_SelfCollide)
        {
            const TCHAR* SelfStr = TEXT("auto");
            if (SelfCollide == 0) SelfStr = TEXT("none");
            else if (SelfCollide == 1) SelfStr = TEXT("auto");
            else if (SelfCollide == 2) SelfStr = TEXT("all");
            ContactAttrs += FString::Printf(TEXT(" selfcollide=\"%s\""), SelfStr);
        }
        if (bOverride_Internal)
        {
            ContactAttrs += FString::Printf(TEXT(" internal=\"%s\""), bInternal ? TEXT("true") : TEXT("false"));
        }
        if (bOverride_Friction && Friction.Num() > 0)
            ContactAttrs += FString::Printf(TEXT(" friction=\"%s\""), *JoinFloats(Friction));
        if (bOverride_SolMix)
            ContactAttrs += FString::Printf(TEXT(" solmix=\"%f\""), SolMix);
        if (bOverride_ContactSolRef && ContactSolRef.Num() > 0)
            ContactAttrs += FString::Printf(TEXT(" solref=\"%s\""), *JoinFloats(ContactSolRef));
        if (bOverride_ContactSolImp && ContactSolImp.Num() > 0)
            ContactAttrs += FString::Printf(TEXT(" solimp=\"%s\""), *JoinFloats(ContactSolImp));
        if (!ContactAttrs.IsEmpty())
        {
            SubElements += FString::Printf(TEXT("<contact%s/>"), *ContactAttrs);
        }
    }

    // <edge>
    {
        FString EdgeAttrs;
        if (bOverride_EdgeStiffness) EdgeAttrs += FString::Printf(TEXT(" stiffness=\"%f\""), EdgeStiffness);
        if (bOverride_EdgeDamping)   EdgeAttrs += FString::Printf(TEXT(" damping=\"%f\""), EdgeDamping);
        if (bOverride_EdgeEquality)
            EdgeAttrs += FString::Printf(TEXT(" equality=\"%s\""), bEdgeEquality ? TEXT("true") : TEXT("false"));
        if (bOverride_EdgeSolRef && EdgeSolRef.Num() > 0)
            EdgeAttrs += FString::Printf(TEXT(" solref=\"%s\""), *JoinFloats(EdgeSolRef));
        if (bOverride_EdgeSolImp && EdgeSolImp.Num() > 0)
            EdgeAttrs += FString::Printf(TEXT(" solimp=\"%s\""), *JoinFloats(EdgeSolImp));
        if (!EdgeAttrs.IsEmpty())
        {
            SubElements += FString::Printf(TEXT("<edge%s/>"), *EdgeAttrs);
        }
    }

    // <elasticity>
    {
        FString ElasticAttrs;
        if (bOverride_Young)     ElasticAttrs += FString::Printf(TEXT(" young=\"%f\""), Young);
        if (bOverride_Poisson)   ElasticAttrs += FString::Printf(TEXT(" poisson=\"%f\""), Poisson);
        if (bOverride_Damping)   ElasticAttrs += FString::Printf(TEXT(" damping=\"%f\""), Damping);
        if (bOverride_Thickness) ElasticAttrs += FString::Printf(TEXT(" thickness=\"%f\""), Thickness);
        if (bOverride_Elastic2D)
        {
            const TCHAR* E2DStr = TEXT("none");
            if (Elastic2D == 1) E2DStr = TEXT("bend");
            else if (Elastic2D == 2) E2DStr = TEXT("stretch");
            else if (Elastic2D == 3) E2DStr = TEXT("both");
            ElasticAttrs += FString::Printf(TEXT(" elastic2d=\"%s\""), E2DStr);
        }
        if (!ElasticAttrs.IsEmpty())
        {
            SubElements += FString::Printf(TEXT("<elasticity%s/>"), *ElasticAttrs);
        }
    }

    // <pin>
    if (PinIds.Num() > 0 || PinGridRange.Num() >= 6)
    {
        SubElements += TEXT("<pin");
        if (PinIds.Num() > 0)
        {
            SubElements += TEXT(" id=\"");
            for (int32 i = 0; i < PinIds.Num(); i++)
            {
                if (i > 0) SubElements += TEXT(" ");
                SubElements += FString::FromInt(PinIds[i]);
            }
            SubElements += TEXT("\"");
        }
        if (PinGridRange.Num() >= 6)
        {
            SubElements += TEXT(" gridrange=\"");
            for (int32 i = 0; i < PinGridRange.Num(); i++)
            {
                if (i > 0) SubElements += TEXT(" ");
                SubElements += FString::FromInt(PinGridRange[i]);
            }
            SubElements += TEXT("\"");
        }
        SubElements += TEXT("/>");
    }

    return FString::Printf(TEXT("<flexcomp %s>%s</flexcomp>"), *Attrs, *SubElements);
}

// ============================================================================
// Spec Registration (Path 2: XML parse + attach)
// ============================================================================

void UMjFlexcomp::RegisterToSpec(FMujocoSpecWrapper& Wrapper, mjsBody* ParentBody)
{
    if (!ParentBody) return;
    if (bIsRegistered) return;

    FString FlexName = MjName.IsEmpty() ? GetName() : MjName;

    // 1. For mesh type, export child static mesh to an OBJ in the VFS
    FString MeshAssetName;
    if (FlexcompType == EMjFlexcompType::Mesh)
    {
        MeshAssetName = ExportMeshToVFS(Wrapper);
        if (MeshAssetName.IsEmpty())
        {
            UE_LOG(LogURLab, Warning, TEXT("[MjFlexcomp] '%s': mesh export failed"), *FlexName);
            return;
        }
    }

    // 2. Build standalone MJCF containing just this flexcomp
    FString FlexcompXml = BuildFlexcompXml(MeshAssetName);
    FString FullXml = FString::Printf(
        TEXT("<mujoco><worldbody>%s</worldbody></mujoco>"), *FlexcompXml);

    // Dump the exact fragment we hand to MuJoCo so we can verify
    // override emission + see what the compile is actually seeing.
    UE_LOG(LogURLab, Log,
        TEXT("[MjFlexcomp] '%s' MJCF fragment:\n%s"),
        *FlexName, *FlexcompXml);

    // 3. Parse into temp spec — MuJoCo expands the flexcomp macro
    char ErrBuf[1000] = "";
    mjSpec* TempSpec = mj_parseXMLString(TCHAR_TO_UTF8(*FullXml), Wrapper.VFS, ErrBuf, sizeof(ErrBuf));
    if (!TempSpec)
    {
        UE_LOG(LogURLab, Error, TEXT("[MjFlexcomp] '%s': mj_parseXMLString failed: %hs"),
            *FlexName, ErrBuf);
        return;
    }

    // 4. Attach temp spec's worldbody into our parent via a new frame
    mjsFrame* AttachFrame = mjs_addFrame(ParentBody, nullptr);
    mjsBody* TempWorld = mjs_findBody(TempSpec, "world");
    if (!TempWorld)
    {
        UE_LOG(LogURLab, Error, TEXT("[MjFlexcomp] '%s': temp spec has no worldbody"), *FlexName);
        mj_deleteSpec(TempSpec);
        return;
    }

    mjsElement* Attached = mjs_attach(AttachFrame->element, TempWorld->element, "", "");
    if (!Attached)
    {
        UE_LOG(LogURLab, Error, TEXT("[MjFlexcomp] '%s': mjs_attach failed"), *FlexName);
    }
    else
    {
        bIsRegistered = true;
        UE_LOG(LogURLab, Log, TEXT("[MjFlexcomp] '%s': attached via XML+parse+attach"), *FlexName);
    }

    mj_deleteSpec(TempSpec);
}

void UMjFlexcomp::Bind(mjModel* Model, mjData* Data, const FString& Prefix)
{
    Super::Bind(Model, Data, Prefix);

    FString FlexName = MjName.IsEmpty() ? GetName() : MjName;
    FString PrefixedName = Prefix + FlexName;

    for (int i = 0; i < Model->nflex; i++)
    {
        const char* Name = mj_id2name(Model, mjOBJ_FLEX, i);
        if (Name && FString(UTF8_TO_TCHAR(Name)) == PrefixedName)
        {
            FlexId = i;
            FlexVertAdr = Model->flex_vertadr[i];
            FlexVertNum = Model->flex_vertnum[i];

            // Cache shell/triangle indices once at Bind (static, no thread race)
            int32 ShellNum = Model->flex_shellnum[i];
            int32 ShellDataAdr = Model->flex_shelldataadr[i];
            int32 FlexDim = Model->flex_dim[i];

            UE_LOG(LogURLab, Log, TEXT("[MjFlexcomp] Bound '%s' to flex ID %d: %d flex verts, %d render verts"),
                *FlexName, FlexId, FlexVertNum, NumRenderVerts);

            CreateProceduralMesh();
            SetComponentTickEnabled(true);
            break;
        }
    }
}

void UMjFlexcomp::CreateProceduralMesh()
{
    if (FlexId < 0 || !m_Model || !m_Data || NumRenderVerts == 0) return;

    AActor* Owner = GetOwner();
    if (!Owner) return;

    // Find source static mesh (we'll build the dynamic mesh from its render data)
    TArray<USceneComponent*> Children;
    GetChildrenComponents(false, Children);
    UStaticMeshComponent* SourceSMC = nullptr;
    for (USceneComponent* Child : Children)
    {
        if (UStaticMeshComponent* SMC = Cast<UStaticMeshComponent>(Child))
        {
            if (SMC->GetStaticMesh()) { SourceSMC = SMC; break; }
        }
    }
    if (!SourceSMC) return;

    DynamicMesh = NewObject<UDynamicMeshComponent>(Owner, TEXT("FlexMesh"));
    DynamicMesh->SetupAttachment(Owner->GetRootComponent());
    DynamicMesh->RegisterComponent();
    DynamicMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    // Let DynamicMesh compute tangents itself via MikkTSpace from normals + UVs.
    DynamicMesh->SetTangentsType(EDynamicMeshComponentTangentsMode::AutoCalculated);

    // Reuse the source mesh's material
    if (UMaterialInterface* SourceMat = SourceSMC->GetMaterial(0))
    {
        DynamicMesh->SetMaterial(0, SourceMat);
    }
    SourceSMC->SetVisibility(false);
    SourceSMC->SetHiddenInGame(true);

    // Build the dynamic mesh from the static mesh's render data (vertex + index + UV + normal + tangent)
    const FStaticMeshLODResources& LOD = SourceSMC->GetStaticMesh()->GetRenderData()->LODResources[0];
    const FStaticMeshVertexBuffer& VB = LOD.VertexBuffers.StaticMeshVertexBuffer;
    const FPositionVertexBuffer& PB = LOD.VertexBuffers.PositionVertexBuffer;
    FIndexArrayView Indices = LOD.IndexBuffer.GetArrayView();
    bool bHasUVs = VB.GetNumTexCoords() > 0;

    DynamicMesh->EditMesh([&](UE::Geometry::FDynamicMesh3& Mesh)
    {
        Mesh.Clear();
        Mesh.EnableAttributes();
        Mesh.Attributes()->SetNumNormalLayers(1);
        if (bHasUVs) Mesh.Attributes()->SetNumUVLayers(1);

        UE::Geometry::FDynamicMeshNormalOverlay* NormalOverlay = Mesh.Attributes()->PrimaryNormals();
        UE::Geometry::FDynamicMeshUVOverlay* UVOverlay = bHasUVs ? Mesh.Attributes()->PrimaryUV() : nullptr;

        // Add vertices (positions in UE local space; at t=0 these come from the static mesh directly)
        for (int32 i = 0; i < NumRenderVerts; i++)
        {
            FVector3f P = PB.VertexPosition(i);
            Mesh.AppendVertex(FVector3d(P.X, P.Y, P.Z));

            FVector4f Nz = VB.VertexTangentZ(i);
            NormalOverlay->AppendElement(FVector3f(Nz.X, Nz.Y, Nz.Z));
            if (UVOverlay)
            {
                FVector2f UV = VB.GetVertexUV(i, 0);
                UVOverlay->AppendElement(FVector2f(UV.X, UV.Y));
            }
        }

        // Add triangles — use raw UE indices and set overlays to same element indices
        for (int32 i = 0; i + 2 < Indices.Num(); i += 3)
        {
            int32 A = Indices[i], B = Indices[i + 1], C = Indices[i + 2];
            if (A == B || B == C || A == C) continue;
            int32 TriId = Mesh.AppendTriangle(A, B, C);
            if (TriId >= 0)
            {
                NormalOverlay->SetTriangle(TriId, UE::Geometry::FIndex3i(A, B, C));
                if (UVOverlay) UVOverlay->SetTriangle(TriId, UE::Geometry::FIndex3i(A, B, C));
            }
        }
    }, EDynamicMeshComponentRenderUpdateMode::FullUpdate);

    UpdateProceduralMesh();
}

void UMjFlexcomp::UpdateProceduralMesh()
{
    if (!DynamicMesh || FlexId < 0 || !m_Data || NumRenderVerts == 0) return;

    FTransform ParentTransform = DynamicMesh->GetAttachParent()
        ? DynamicMesh->GetAttachParent()->GetComponentTransform()
        : FTransform::Identity;

    // Read welded flex positions under the physics mutex to avoid torn reads.
    TArray<FVector> WeldedPositions;
    WeldedPositions.SetNum(FlexVertNum);

    AAMjManager* Manager = AAMjManager::GetManager();
    UMjPhysicsEngine* Engine = Manager ? Manager->PhysicsEngine : nullptr;

    {
        FScopeTryLock ScopeLock(Engine ? &Engine->CallbackMutex : nullptr);
        if (Engine && !ScopeLock.IsLocked())
        {
            return;
        }
        for (int32 i = 0; i < FlexVertNum; i++)
        {
            const int32 Idx = (FlexVertAdr + i) * 3;
            const FVector WorldPos = MjUtils::MjToUEPosition(&m_Data->flexvert_xpos[Idx]);
            WeldedPositions[i] = ParentTransform.InverseTransformPosition(WorldPos);
        }
    }

    DynamicMesh->EditMesh([&](UE::Geometry::FDynamicMesh3& Mesh)
    {
        for (int32 i = 0; i < NumRenderVerts; i++)
        {
            const int32 W = RawToWelded[i];
            const FVector P = (W >= 0 && W < FlexVertNum) ? WeldedPositions[W] : FVector::ZeroVector;
            Mesh.SetVertex(i, FVector3d(P.X, P.Y, P.Z));
        }
    }, EDynamicMeshComponentRenderUpdateMode::NoUpdate);

    DynamicMesh->FastNotifyPositionsUpdated(/*bNormals=*/false, /*bColors=*/false, /*bUVs=*/false);
}

void UMjFlexcomp::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
    UpdateProceduralMesh();
}


