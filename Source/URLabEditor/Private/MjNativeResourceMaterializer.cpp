// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#include "MjNativeResourceMaterializer.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Engine/Blueprint.h"
#include "Engine/StaticMesh.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#include "MeshDescription.h"
#include "Misc/Paths.h"
#include "PackageTools.h"
#include "StaticMeshAttributes.h"
#include "MuJoCo/Gen/Elements/Bodies/MjBody.gen.h"
#include "MuJoCo/Gen/Elements/Assets/MjHfield.gen.h"
#include "MuJoCo/Gen/Elements/Assets/MjSkin.gen.h"
#include "MuJoCo/Gen/Elements/Assets/MjSkinBone.gen.h"
#include "MuJoCo/Gen/Elements/Deformable/MjFlexcomp.gen.h"
#include "MuJoCo/Elements/MjFlexcomp.h"
#include "MuJoCo/Spec/MjAssetResolve.h"
#include "MuJoCo/Spec/MjAssetSink.h"
#include "MuJoCo/Spec/MjNodeFactories.h"
#include "MuJoCo/Spec/MjTreeAdapters.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "Utils/CoordinateConversion.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace
{
struct FNativeModelGuard
{
	mjModel* Model = nullptr;
	~FNativeModelGuard()
	{
		if (Model != nullptr) mj_deleteModel(Model);
	}
};

struct FNativeSpecGuard
{
	mjSpec* Spec = nullptr;
	~FNativeSpecGuard()
	{
		if (Spec != nullptr) mj_deleteSpec(Spec);
	}
};

bool FlexcompRotation(const UMjFlexcomp& Flexcomp, const mjSpec& Spec, mjtNum OutQuat[4], FString& OutError)
{
	if (Flexcomp.Quat.IsSet())
	{
		const FMjQuatRot Q = Flexcomp.Quat.GetValue();
		OutQuat[0] = Q.W; OutQuat[1] = Q.X; OutQuat[2] = Q.Y; OutQuat[3] = Q.Z;
		mju_normalize4(OutQuat);
		return true;
	}
	if (Flexcomp.Euler.IsSet())
	{
		const FMjVec3 Euler = Flexcomp.Euler.GetValue();
		mjtNum Angles[3] = {Euler.X, Euler.Y, Euler.Z};
		if (Spec.compiler.degree)
		{
			for (mjtNum& Angle : Angles) Angle *= mjPI / 180.0;
		}
		mju_euler2Quat(OutQuat, Angles, Spec.compiler.eulerseq);
		return true;
	}
	if (Flexcomp.Axisangle.IsSet())
	{
		const TArray<double>& AxisAngle = Flexcomp.Axisangle.GetValue();
		if (AxisAngle.Num() != 4)
		{
			OutError = FString::Printf(TEXT("flexcomp '%s' has malformed axisangle data"), *MjAssetElementName(Flexcomp));
			return false;
		}
		mjtNum Axis[3] = {AxisAngle[0], AxisAngle[1], AxisAngle[2]};
		mjtNum Angle = AxisAngle[3] * (Spec.compiler.degree ? mjPI / 180.0 : 1.0);
		mju_axisAngle2Quat(OutQuat, Axis, Angle);
		return true;
	}
	if (Flexcomp.Xyaxes.IsSet())
	{
		const TArray<double>& Axes = Flexcomp.Xyaxes.GetValue();
		if (Axes.Num() != 6)
		{
			OutError = FString::Printf(TEXT("flexcomp '%s' has malformed xyaxes data"), *MjAssetElementName(Flexcomp));
			return false;
		}
		mjtNum X[3] = {Axes[0], Axes[1], Axes[2]};
		mjtNum Y[3] = {Axes[3], Axes[4], Axes[5]};
		if (mju_normalize3(X) < mjEPS)
		{
			OutError = TEXT("flexcomp xyaxes contains a zero x axis"); return false;
		}
		const mjtNum Projection = mju_dot3(X, Y);
		for (int32 Axis = 0; Axis < 3; ++Axis) Y[Axis] -= X[Axis] * Projection;
		if (mju_normalize3(Y) < mjEPS)
		{
			OutError = TEXT("flexcomp xyaxes contains parallel axes"); return false;
		}
		mjtNum Z[3]; mju_cross(Z, X, Y);
		mjtNum Matrix[9] = {X[0], Y[0], Z[0], X[1], Y[1], Z[1], X[2], Y[2], Z[2]};
		mju_mat2Quat(OutQuat, Matrix);
		return true;
	}
	if (Flexcomp.Zaxis.IsSet())
	{
		const FMjDirection3 Value = Flexcomp.Zaxis.GetValue();
		mjtNum Z[3] = {Value.X, Value.Y, Value.Z};
		if (mju_normalize3(Z) < mjEPS)
		{
			OutError = TEXT("flexcomp zaxis is zero"); return false;
		}
		mjtNum Axis[3] = {-Z[1], Z[0], 0.0};
		mjtNum Angle = std::acos(FMath::Clamp(Z[2], -1.0, 1.0));
		if (mju_normalize3(Axis) < mjEPS) { Axis[0] = 1.0; Axis[1] = 0.0; Axis[2] = 0.0; }
		mju_axisAngle2Quat(OutQuat, Axis, Angle);
		return true;
	}
	OutQuat[0] = 1.0; OutQuat[1] = OutQuat[2] = OutQuat[3] = 0.0;
	return true;
}

int32 ParentBodyId(urlab::spec::FMjScsScope& Scope, const UMjFlexcomp& Flexcomp,
	const mjModel& Model, FString& OutError)
{
	USCS_Node* Parent = Scope.ParentNodeOf(Flexcomp);
	while (Parent != nullptr)
	{
		if (const UMjBodyBase* Body = Cast<UMjBodyBase>(Scope.TemplateFor(*Parent)))
		{
			const FString BodyName = MjAssetElementName(*Body);
			const int32 BodyId = mj_name2id(&Model, mjOBJ_BODY, TCHAR_TO_UTF8(*BodyName));
			if (BodyId >= 0) return BodyId;
			OutError = FString::Printf(TEXT("flexcomp '%s' parent body '%s' is missing from the native model"),
				*MjAssetElementName(Flexcomp), *BodyName);
			return -1;
		}
		UMjNodeComponent* Template = Cast<UMjNodeComponent>(Parent->ComponentTemplate);
		if (Template == nullptr) return 0;
		Parent = Scope.ParentNodeOf(*Template);
	}
	return 0;
}

bool CopyFlexMesh(const mjModel& Model, const mjSpec& Spec, int32 FlexId, int32 ParentId,
	UMjFlexcomp& Flexcomp, FString& OutError)
{
	if (Model.flex_dim[FlexId] != 2)
	{
		OutError = FString::Printf(TEXT("file-backed flexcomp '%s' mesh has dim=%d; only surface dim=2 can be materialized as an editable static mesh"),
			*MjAssetElementName(Flexcomp), Model.flex_dim[FlexId]);
		return false;
	}
	const int32 VertAddress = Model.flex_vertadr[FlexId];
	const int32 VertCount = Model.flex_vertnum[FlexId];
	const int32 ElemDataAddress = Model.flex_elemdataadr[FlexId];
	const int32 ElemCount = Model.flex_elemnum[FlexId];
	const int32 ElemWidth = Model.flex_dim[FlexId] + 1;
	const int32 TexcoordAddress = Model.flex_texcoordadr[FlexId];
	if (VertAddress < 0 || VertCount < 3 || VertAddress > Model.nflexvert - VertCount ||
		ElemDataAddress < 0 || ElemCount < 1 || ElemDataAddress > Model.nflexelemdata - ElemCount * ElemWidth ||
		Model.flex_vert == nullptr || Model.flex_vertbodyid == nullptr || Model.flex_elem == nullptr)
	{
		OutError = FString::Printf(TEXT("flexcomp '%s' has incomplete native surface arrays"), *MjAssetElementName(Flexcomp));
		return false;
	}
	if (TexcoordAddress >= 0 && (Model.flex_texcoord == nullptr || Model.flex_elemtexcoord == nullptr))
	{
		OutError = FString::Printf(TEXT("flexcomp '%s' has incomplete native UV arrays"), *MjAssetElementName(Flexcomp));
		return false;
	}

	mjtNum Rotation[4];
	if (!FlexcompRotation(Flexcomp, Spec, Rotation, OutError)) return false;
	mjtNum InverseRotation[4];
	mju_negQuat(InverseRotation, Rotation);
	const FMjPosition3 Position = Flexcomp.Pos.Get(FMjPosition3());
	const FMjVec3 Scale = Flexcomp.Scale.Get(FMjVec3(1.0, 1.0, 1.0));
	if (FMath::Abs(Scale.X) < UE_DOUBLE_SMALL_NUMBER || FMath::Abs(Scale.Y) < UE_DOUBLE_SMALL_NUMBER ||
		FMath::Abs(Scale.Z) < UE_DOUBLE_SMALL_NUMBER)
	{
		OutError = FString::Printf(TEXT("flexcomp '%s' has a zero scale axis"), *MjAssetElementName(Flexcomp));
		return false;
	}

	TArray<FVector> Positions;
	Positions.Reserve(VertCount);
	for (int32 Vertex = 0; Vertex < VertCount; ++Vertex)
	{
		const int32 NativeVertex = VertAddress + Vertex;
		const int32 BodyId = Model.flex_vertbodyid[NativeVertex];
		if (BodyId < 0 || BodyId >= Model.nbody)
		{
			OutError = FString::Printf(TEXT("flexcomp '%s' vertex %d references an invalid body"), *MjAssetElementName(Flexcomp), Vertex);
			return false;
		}
		mjtNum ParentPoint[3] = {
			Model.flex_vert[3 * NativeVertex], Model.flex_vert[3 * NativeVertex + 1], Model.flex_vert[3 * NativeVertex + 2]};
		if (BodyId != ParentId)
		{
			if (Model.body_parentid[BodyId] != ParentId)
			{
				OutError = FString::Printf(TEXT("flexcomp '%s' vertex %d no longer maps to a direct generated body"),
					*MjAssetElementName(Flexcomp), Vertex);
				return false;
			}
			mjtNum Rotated[3];
			mju_rotVecQuat(Rotated, ParentPoint, Model.body_quat + 4 * BodyId);
			for (int32 Axis = 0; Axis < 3; ++Axis) ParentPoint[Axis] = Model.body_pos[3 * BodyId + Axis] + Rotated[Axis];
		}
		mjtNum Shifted[3] = {ParentPoint[0] - Position.X, ParentPoint[1] - Position.Y, ParentPoint[2] - Position.Z};
		mjtNum SourcePoint[3];
		mju_rotVecQuat(SourcePoint, Shifted, InverseRotation);
		SourcePoint[0] /= Scale.X; SourcePoint[1] /= Scale.Y; SourcePoint[2] /= Scale.Z;
		if (!FMath::IsFinite(SourcePoint[0]) || !FMath::IsFinite(SourcePoint[1]) || !FMath::IsFinite(SourcePoint[2]))
		{
			OutError = FString::Printf(TEXT("flexcomp '%s' native vertex %d is not finite"), *MjAssetElementName(Flexcomp), Vertex);
			return false;
		}
		Positions.Add(URLabCoordinateConversion::RightHandedMetresPositionToUnrealCentimetres(SourcePoint));
	}

	const FString AssetName = MjSanitizeAssetName(TEXT("Flex_") + MjAssetElementName(Flexcomp));
	const FString PackageName = UPackageTools::SanitizePackageName(
		MjImportedMeshPath(FSpecRef::OverBlueprint(*Flexcomp.GetTypedOuter<UBlueprint>())) / AssetName);
	UPackage* Package = CreatePackage(*PackageName);
	if (Package == nullptr)
	{
		OutError = FString::Printf(TEXT("flexcomp '%s' could not create mesh package '%s'"), *MjAssetElementName(Flexcomp), *PackageName);
		return false;
	}
	UStaticMesh* Mesh = LoadObject<UStaticMesh>(nullptr, *PackageName);
	const bool bNewAsset = Mesh == nullptr;
	if (bNewAsset) Mesh = NewObject<UStaticMesh>(Package, *AssetName, RF_Public | RF_Standalone);
	if (Mesh == nullptr)
	{
		OutError = FString::Printf(TEXT("flexcomp '%s' could not create its editable UStaticMesh"), *MjAssetElementName(Flexcomp));
		return false;
	}
	Mesh->SetNumSourceModels(1);
	FMeshDescription* Description = Mesh->CreateMeshDescription(0);
	if (Description == nullptr)
	{
		OutError = FString::Printf(TEXT("flexcomp '%s' could not create a MeshDescription"), *MjAssetElementName(Flexcomp));
		return false;
	}
	FStaticMeshAttributes Attributes(*Description);
	Attributes.Register();
	Attributes.GetVertexInstanceUVs().SetNumChannels(TexcoordAddress >= 0 ? 1 : 0);
	TVertexAttributesRef<FVector3f> VertexPositions = Attributes.GetVertexPositions();
	TVertexInstanceAttributesRef<FVector3f> Normals = Attributes.GetVertexInstanceNormals();
	TVertexInstanceAttributesRef<FVector2f> UVs = Attributes.GetVertexInstanceUVs();
	TArray<FVertexID> VertexIds;
	VertexIds.Reserve(VertCount);
	for (const FVector& PositionValue : Positions)
	{
		const FVertexID VertexId = Description->CreateVertex();
		VertexPositions[VertexId] = FVector3f(PositionValue);
		VertexIds.Add(VertexId);
	}
	const FPolygonGroupID Group = Description->CreatePolygonGroup();
	for (int32 Elem = 0; Elem < ElemCount; ++Elem)
	{
		TArray<FVertexInstanceID> Instances;
		for (int32 Corner = ElemWidth - 1; Corner >= 0; --Corner)
		{
			const int32 DataIndex = ElemDataAddress + Elem * ElemWidth + Corner;
			const int32 Vertex = Model.flex_elem[DataIndex];
			if (Vertex < 0 || Vertex >= VertCount)
			{
				OutError = FString::Printf(TEXT("flexcomp '%s' element %d has an invalid vertex id"), *MjAssetElementName(Flexcomp), Elem);
				return false;
			}
			const FVertexInstanceID Instance = Description->CreateVertexInstance(VertexIds[Vertex]);
			Normals[Instance] = FVector3f::ZeroVector;
			if (TexcoordAddress >= 0)
			{
				const int32 Texcoord = Model.flex_elemtexcoord[DataIndex];
				if (Texcoord < 0 || Texcoord >= Model.nflextexcoord)
				{
					OutError = FString::Printf(TEXT("flexcomp '%s' element %d has an invalid texture-coordinate id"), *MjAssetElementName(Flexcomp), Elem);
					return false;
				}
				UVs.Set(Instance, 0, FVector2f(Model.flex_texcoord[2 * (TexcoordAddress + Texcoord)],
					Model.flex_texcoord[2 * (TexcoordAddress + Texcoord) + 1]));
			}
			Instances.Add(Instance);
		}
		Description->CreatePolygon(Group, Instances);
	}
	Mesh->CommitMeshDescription(0);
	Mesh->Build(false);
	Mesh->PostEditChange();
	if (bNewAsset)
	{
		FAssetRegistryModule::AssetCreated(Mesh);
	}
	Package->MarkPackageDirty();
	Flexcomp.MeshAsset = Mesh;
	Flexcomp.File.Reset();
	return true;
}

bool HasNativeResourceNodes(const TArray<UMjNodeComponent*>& Nodes, FString& OutError)
{
	for (const UMjNodeComponent* Node : Nodes)
	{
		if (const UMjFlexcomp* Flexcomp = Cast<UMjFlexcomp>(Node))
		{
			if (Flexcomp->File.IsSet() && !Flexcomp->File->IsEmpty())
			{
				const FString Name = Flexcomp->MjName.Get(FString(TEXT("<unnamed>")));
				if (Flexcomp->GetType() == EMjFlexcompType::mesh)
				{
					if (Flexcomp->MeshAsset == nullptr)
					{
						OutError = FString::Printf(TEXT("flexcomp '%s' did not produce its editable surface mesh asset"), *Name);
						return false;
					}
				}
				else
				{
					OutError = FString::Printf(
						TEXT("file-backed flexcomp '%s' of type '%s' has no supported UE mesh materialization"),
						*Name, *UEnum::GetValueAsString(Flexcomp->GetType()));
					return false;
				}
			}
		}
	}
	return true;
}

bool HasFileBackedHfieldOrSkin(const TArray<UMjNodeComponent*>& Nodes)
{
	for (const UMjNodeComponent* Node : Nodes)
	{
		if (const UMjHfield* Hfield = Cast<UMjHfield>(Node))
		{
			if (Hfield->File.IsSet() && !Hfield->File->IsEmpty()) return true;
		}
		if (const UMjSkin* Skin = Cast<UMjSkin>(Node))
		{
			if (Skin->File.IsSet() && !Skin->File->IsEmpty()) return true;
		}
	}
	return false;
}

bool CopyHfield(const mjModel& Model, int32 Id, UMjHfield& Hfield, FString& OutError)
{
	const int32 Rows = Model.hfield_nrow[Id];
	const int32 Cols = Model.hfield_ncol[Id];
	const int32 Address = Model.hfield_adr[Id];
	const int64 Count = static_cast<int64>(Rows) * Cols;
	if (Rows < 1 || Cols < 1 || Count > MAX_int32 || Address < 0 || Count > Model.nhfielddata ||
		Address > Model.nhfielddata - Count || Model.hfield_data == nullptr || Model.hfield_size == nullptr)
	{
		OutError = FString::Printf(TEXT("heightfield '%s' has invalid native dimensions or sample storage"),
			*MjAssetElementName(Hfield));
		return false;
	}

	TArray<double> Elevation;
	Elevation.Reserve(static_cast<int32>(Count));
	for (int64 Index = 0; Index < Count; ++Index)
	{
		const float Sample = Model.hfield_data[Address + Index];
		if (!FMath::IsFinite(Sample) || Sample < 0.0f || Sample > 1.0f)
		{
			OutError = FString::Printf(TEXT("heightfield '%s' has an invalid normalized native sample"),
				*MjAssetElementName(Hfield));
			return false;
		}
		Elevation.Add(static_cast<double>(Sample));
	}

	TArray<double> Size;
	Size.Reserve(4);
	for (int32 Axis = 0; Axis < 4; ++Axis)
	{
		const double Value = Model.hfield_size[4 * Id + Axis];
		if (!FMath::IsFinite(Value) || Value <= 0.0)
		{
			OutError = FString::Printf(TEXT("heightfield '%s' has invalid native size data"),
				*MjAssetElementName(Hfield));
			return false;
		}
		Size.Add(Value);
	}

	Hfield.Nrow = Rows;
	Hfield.Ncol = Cols;
	Hfield.Size = MoveTemp(Size);
	Hfield.Elevation = MoveTemp(Elevation);
	Hfield.File.Reset();
	return true;
}

bool CopySkin(const mjModel& Model, int32 Id, UMjSkin& Skin, UBlueprint& Blueprint, FString& OutError)
{
	const int32 VertexAddress = Model.skin_vertadr[Id];
	const int32 VertexCount = Model.skin_vertnum[Id];
	const int32 TexcoordAddress = Model.skin_texcoordadr[Id];
	const int32 FaceAddress = Model.skin_faceadr[Id];
	const int32 FaceCount = Model.skin_facenum[Id];
	const int32 BoneAddress = Model.skin_boneadr[Id];
	const int32 BoneCount = Model.skin_bonenum[Id];
	if (VertexAddress < 0 || VertexCount < 1 || VertexAddress > Model.nskinvert - VertexCount ||
		FaceAddress < 0 || FaceCount < 1 || FaceAddress > Model.nskinface - FaceCount ||
		BoneAddress < 0 || BoneCount < 1 || BoneAddress > Model.nskinbone - BoneCount ||
		Model.skin_vert == nullptr || Model.skin_face == nullptr || Model.skin_bonebodyid == nullptr ||
		Model.skin_bonebindpos == nullptr || Model.skin_bonebindquat == nullptr ||
		Model.skin_bonevertadr == nullptr || Model.skin_bonevertnum == nullptr ||
		Model.skin_bonevertid == nullptr || Model.skin_bonevertweight == nullptr)
	{
		OutError = FString::Printf(TEXT("skin '%s' has incomplete native mesh or bone arrays"),
			*MjAssetElementName(Skin));
		return false;
	}
	if (TexcoordAddress >= 0 && (TexcoordAddress > Model.nskintexvert - VertexCount || Model.skin_texcoord == nullptr))
	{
		OutError = FString::Printf(TEXT("skin '%s' has invalid native texture-coordinate storage"),
			*MjAssetElementName(Skin));
		return false;
	}

	TArray<float> Vertices;
	Vertices.Append(Model.skin_vert + 3 * VertexAddress, 3 * VertexCount);
	TArray<int32> Faces;
	Faces.Append(Model.skin_face + 3 * FaceAddress, 3 * FaceCount);
	for (const int32 VertexId : Faces)
	{
		if (VertexId < 0 || VertexId >= VertexCount)
		{
			OutError = FString::Printf(TEXT("skin '%s' has an out-of-range native face index"),
				*MjAssetElementName(Skin));
			return false;
		}
	}

	Skin.Vertex = MoveTemp(Vertices);
	Skin.Face = MoveTemp(Faces);
	if (TexcoordAddress >= 0)
	{
		TArray<float> Texcoords;
		Texcoords.Append(Model.skin_texcoord + 2 * TexcoordAddress, 2 * VertexCount);
		Skin.Texcoord = MoveTemp(Texcoords);
	}
	else
	{
		Skin.Texcoord.Reset();
	}

	for (int32 BoneIndex = 0; BoneIndex < BoneCount; ++BoneIndex)
	{
		const int32 NativeBone = BoneAddress + BoneIndex;
		const int32 BodyId = Model.skin_bonebodyid[NativeBone];
		const int32 InfluenceAddress = Model.skin_bonevertadr[NativeBone];
		const int32 InfluenceCount = Model.skin_bonevertnum[NativeBone];
		if (BodyId < 0 || BodyId >= Model.nbody || InfluenceAddress < 0 || InfluenceCount < 1 ||
			InfluenceAddress > Model.nskinbonevert - InfluenceCount)
		{
			OutError = FString::Printf(TEXT("skin '%s' has invalid native bone mapping %d"),
				*MjAssetElementName(Skin), BoneIndex);
			return false;
		}
		const char* BodyName = mj_id2name(&Model, mjOBJ_BODY, BodyId);
		if (BodyName == nullptr || BodyName[0] == '\0')
		{
			OutError = FString::Printf(TEXT("skin '%s' bone %d refers to a body without an authorable name"),
				*MjAssetElementName(Skin), BoneIndex);
			return false;
		}

		UMjSkinBone& Bone = urlab::spec::FScsNodeFactory::Create<UMjSkinBone>(Skin);
		Bone.MjName = FString::Printf(TEXT("%s_Bone_%d"), *MjAssetElementName(Skin), BoneIndex);
		Bone.Body = UTF8_TO_TCHAR(BodyName);
		Bone.Bindpos = FMjPosition3(Model.skin_bonebindpos[3 * NativeBone],
			Model.skin_bonebindpos[3 * NativeBone + 1], Model.skin_bonebindpos[3 * NativeBone + 2]);
		Bone.Bindquat = FMjQuatRot(Model.skin_bonebindquat[4 * NativeBone],
			Model.skin_bonebindquat[4 * NativeBone + 1], Model.skin_bonebindquat[4 * NativeBone + 2],
			Model.skin_bonebindquat[4 * NativeBone + 3]);

		TArray<double> VertexIds;
		TArray<double> Weights;
		VertexIds.Reserve(InfluenceCount);
		Weights.Reserve(InfluenceCount);
		for (int32 Influence = 0; Influence < InfluenceCount; ++Influence)
		{
			const int32 VertexId = Model.skin_bonevertid[InfluenceAddress + Influence];
			const float Weight = Model.skin_bonevertweight[InfluenceAddress + Influence];
			if (VertexId < 0 || VertexId >= VertexCount || !FMath::IsFinite(Weight))
			{
				OutError = FString::Printf(TEXT("skin '%s' has an invalid bone influence"),
					*MjAssetElementName(Skin));
				return false;
			}
			VertexIds.Add(static_cast<double>(VertexId));
			Weights.Add(static_cast<double>(Weight));
		}
		Bone.Vertid = MoveTemp(VertexIds);
		Bone.Vertweight = MoveTemp(Weights);
	}

	Skin.File.Reset();
	return true;
}

} // namespace

bool MjMaterializeNativeFileAssets(UBlueprint& Blueprint, const FString& NativeSourcePath, FString& OutError)
{
	OutError.Reset();
#if URLAB_MJ_GEN && WITH_EDITOR
	urlab::spec::FMjScsScope ScsScope(Blueprint);
	TArray<UMjNodeComponent*> Nodes;
	if (Blueprint.SimpleConstructionScript != nullptr)
	{
		for (const USCS_Node* SCSNode : Blueprint.SimpleConstructionScript->GetAllNodes())
		{
			if (SCSNode != nullptr)
			{
				if (UMjNodeComponent* Node = Cast<UMjNodeComponent>(SCSNode->ComponentTemplate))
				{
					Nodes.Add(Node);
				}
			}
		}
	}
	if (!HasNativeResourceNodes(Nodes, OutError)) return false;
	if (!HasFileBackedHfieldOrSkin(Nodes)) return true;
	if (NativeSourcePath.IsEmpty() || !FPaths::FileExists(NativeSourcePath))
	{
		OutError = TEXT("file-backed heightfields and skins require the original MJCF path during import so MuJoCo can decode them");
		return false;
	}

	char NativeError[2048] = {};
	FNativeModelGuard Native;
	Native.Model = mj_loadXML(TCHAR_TO_UTF8(*NativeSourcePath), nullptr, NativeError, sizeof(NativeError));
	if (Native.Model == nullptr)
	{
		OutError = FString::Printf(TEXT("MuJoCo could not decode file-backed authoring resources: %s"),
			UTF8_TO_TCHAR(NativeError));
		return false;
	}

	for (UMjNodeComponent* Node : Nodes)
	{
		if (UMjHfield* Hfield = Cast<UMjHfield>(Node))
		{
			if (!Hfield->File.IsSet() || Hfield->File->IsEmpty()) continue;
			const FString Name = MjAssetElementName(*Hfield);
			const int32 Id = mj_name2id(Native.Model, mjOBJ_HFIELD, TCHAR_TO_UTF8(*Name));
			if (Id < 0 || Id >= Native.Model->nhfield)
			{
				OutError = FString::Printf(TEXT("MuJoCo did not compile file-backed heightfield '%s'"), *Name);
				return false;
			}
			if (!CopyHfield(*Native.Model, Id, *Hfield, OutError)) return false;
		}
		else if (UMjSkin* Skin = Cast<UMjSkin>(Node))
		{
			if (!Skin->File.IsSet() || Skin->File->IsEmpty()) continue;
			const FString Name = MjAssetElementName(*Skin);
			const int32 Id = mj_name2id(Native.Model, mjOBJ_SKIN, TCHAR_TO_UTF8(*Name));
			if (Id < 0 || Id >= Native.Model->nskin)
			{
				OutError = FString::Printf(TEXT("MuJoCo did not compile file-backed skin '%s'"), *Name);
				return false;
			}
			if (!CopySkin(*Native.Model, Id, *Skin, Blueprint, OutError)) return false;
		}
	}
	return true;
#else
	(void)Blueprint;
	(void)NativeSourcePath;
	OutError = TEXT("native file-backed resource materialization requires editor MuJoCo support");
	return false;
#endif
}

#else

bool MjMaterializeNativeFileAssets(UBlueprint& Blueprint, const FString& NativeSourcePath, FString& OutError)
{
	(void)Blueprint;
	(void)NativeSourcePath;
	OutError = TEXT("native file-backed resource materialization requires editor MuJoCo support");
	return false;
}

#endif // URLAB_MJ_GEN && WITH_EDITOR
