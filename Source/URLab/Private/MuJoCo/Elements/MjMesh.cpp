// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Elements/MjMesh.h"

#include "Engine/StaticMesh.h"
#include "Misc/Paths.h"

#if WITH_EDITOR
#include "MeshDescription.h"
#include "StaticMeshAttributes.h"
#endif

#include "MuJoCo/Spec/MjAssetFiles.h"
#include "MuJoCo/Spec/MjAssetResolve.h"
#include "MuJoCo/Spec/MjAssetSink.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "Utils/MeshUtils.h"
#include "Utils/URLabLogging.h"

bool UMjMesh::IsFileStale() const
{
	return MeshAsset != nullptr && FSoftObjectPath(MeshAsset.Get()) != FileAsset;
}

bool UMjMesh::DumpAssetToFile(const FString& BaseDirectory, const FString& OutputName)
{
#if !WITH_EDITOR
	UE_LOG(LogURLab, Error, TEXT("[MjMesh] visual mesh export requires editor MeshDescription data"));
	return false;
#else
	if (MeshAsset == nullptr)
	{
		return false;
	}

	// The selected mesh is visual geometry. Chaos collision is a separate,
	// optional cooking product and can have different triangles or no triangles.
	FMeshDescription* Description = MeshAsset->GetMeshDescription(0);
	if (Description == nullptr)
	{
		UE_LOG(LogURLab, Error, TEXT("[MjMesh] '%s' has no editable LOD0 MeshDescription on '%s'"),
			*MjAssetElementName(*this), *MeshAsset->GetName());
		return false;
	}
	FStaticMeshAttributes Attributes(*Description);
	const TVertexAttributesRef<FVector3f> VertexPositions = Attributes.GetVertexPositions();
	const TVertexInstanceAttributesRef<FVector3f> InstanceNormals = Attributes.GetVertexInstanceNormals();
	const TVertexInstanceAttributesRef<FVector2f> InstanceUVs = Attributes.GetVertexInstanceUVs();
	const int32 UVChannels = InstanceUVs.GetNumChannels();
	if (UVChannels > 1)
	{
		UE_LOG(LogURLab, Warning,
			TEXT("[MjMesh] '%s' exports LOD0 UV channel 0; MuJoCo OBJ mesh assets do not represent additional UV channels"),
			*MjAssetElementName(*this));
	}

	TArray<FVector> Positions;
	TMap<int32, int32> VertexIndices;
	Positions.Reserve(Description->Vertices().Num());
	for (const FVertexID VertexId : Description->Vertices().GetElementIDs())
	{
		VertexIndices.Add(VertexId.GetValue(), Positions.Add(FVector(VertexPositions[VertexId])));
	}

	TArray<FIntVector> Triangles;
	TArray<FVector> CornerNormals;
	TArray<FVector2D> CornerUVs;
	bool bHasValidCornerNormal = false;
	bool bHasMissingCornerNormal = false;
	for (const FPolygonID PolygonId : Description->Polygons().GetElementIDs())
	{
		for (const FTriangleID TriangleId : Description->GetPolygonTriangleIDs(PolygonId))
		{
			const TArrayView<const FVertexInstanceID> Instances =
				Description->GetTriangleVertexInstances(TriangleId);
			if (Instances.Num() != 3)
			{
				UE_LOG(LogURLab, Error, TEXT("[MjMesh] '%s' has a non-triangle LOD0 tessellation"),
					*MjAssetElementName(*this));
				return false;
			}
			int32 TriangleVertices[3];
			FVector SourceCornerNormals[3];
			FVector2D SourceCornerUVs[3];
			for (int32 Corner = 0; Corner < 3; ++Corner)
			{
				const FVertexInstanceID InstanceId = Instances[Corner];
				const FVertexID VertexId = Description->GetVertexInstanceVertex(InstanceId);
				const int32* VertexIndex = VertexIndices.Find(VertexId.GetValue());
				const FVector Normal(InstanceNormals[InstanceId]);
				if (VertexIndex == nullptr)
				{
					UE_LOG(LogURLab, Error,
						TEXT("[MjMesh] '%s' has an invalid LOD0 corner vertex or normal"),
						*MjAssetElementName(*this));
					return false;
				}
				if (Normal.ContainsNaN())
				{
					UE_LOG(LogURLab, Error, TEXT("[MjMesh] '%s' has an invalid LOD0 corner normal"),
						*MjAssetElementName(*this));
					return false;
				}
				TriangleVertices[Corner] = *VertexIndex;
				SourceCornerNormals[Corner] = Normal;
				bHasValidCornerNormal |= !Normal.IsNearlyZero();
				bHasMissingCornerNormal |= Normal.IsNearlyZero();
				if (UVChannels > 0)
				{
					const FVector2f UV = InstanceUVs.Get(InstanceId, 0);
					SourceCornerUVs[Corner] = FVector2D(UV.X, UV.Y);
				}
			}
			for (int32 Corner = 0; Corner < 3; ++Corner)
			{
				const FVector Normal = SourceCornerNormals[Corner];
				if (!Normal.IsNearlyZero()) CornerNormals.Add(Normal.GetSafeNormal());
				if (UVChannels > 0) CornerUVs.Add(SourceCornerUVs[Corner]);
			}
			Triangles.Emplace(TriangleVertices[0], TriangleVertices[1], TriangleVertices[2]);
		}
	}
	if (Positions.Num() < 3 || Triangles.IsEmpty())
	{
		UE_LOG(LogURLab, Error, TEXT("[MjMesh] '%s' has no visual triangles in LOD0"),
			*MjAssetElementName(*this));
		return false;
	}
	if (bHasMissingCornerNormal && bHasValidCornerNormal)
	{
		UE_LOG(LogURLab, Error,
			TEXT("[MjMesh] '%s' has a partial LOD0 normal stream; UE corner normals must be complete or absent"),
			*MjAssetElementName(*this));
		return false;
	}
	if (!bHasValidCornerNormal)
	{
		// Keep the source's lack of a normal stream. MuJoCo's OBJ reader derives
		// normals from the faces; inventing flat normals here would change shading.
		CornerNormals.Reset();
		UE_LOG(LogURLab, Warning,
			TEXT("[MjMesh] '%s' has no LOD0 corner normals; OBJ omits vn and MuJoCo derives normals"),
			*MjAssetElementName(*this));
	}

	const FString Name = MjSanitizeAssetName(OutputName.IsEmpty()
		? MjAssetElementName(*this) : OutputName);
	if (Name.IsEmpty())
	{
		return false;
	}
	const FMjDumpTarget Target = MjDumpTargetFor(BaseDirectory, Name, TEXT(".obj"));

	// The common MeshUtils writer applies the same UE-centimetre/LH to
	// MuJoCo-metre/RH conversion and reverses each reflected triangle. OBJ's
	// split corner indices preserve this MeshDescription's UV seams and normals.
	const int32 Written = MeshUtils::SaveVisualMeshAsOBJ(
		Target.FullPath, Positions, Triangles, CornerNormals, CornerUVs);
	if (Written == 0)
	{
		UE_LOG(LogURLab, Error, TEXT("[MjMesh] could not write '%s'"), *Target.FullPath);
		return false;
	}
	if (!MeshAsset->GetStaticMaterials().IsEmpty())
	{
		UE_LOG(LogURLab, Warning,
			TEXT("[MjMesh] OBJ exports geometry and UVs only for '%s'; the existing MJCF material node remains authoritative and Unreal material graphs are not converted"),
			*MjAssetElementName(*this));
	}

	// MJCF admits a file or inline vertices, never both, and the file is now the
	// geometry. An element that authored its own triangles and then had an asset
	// dropped on it would otherwise fail to compile.
	Vertex.Reset();
	Normal.Reset();
	Texcoord.Reset();
	Face.Reset();

	File = Target.FileAttribute;
	FileAsset = FSoftObjectPath(MeshAsset.Get());
	return true;
#endif // WITH_EDITOR
}

#if WITH_EDITOR

void UMjMesh::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

	const FName Changed = PropertyChangedEvent.GetPropertyName();
	const FName Member = PropertyChangedEvent.MemberProperty != nullptr
						   ? PropertyChangedEvent.MemberProperty->GetFName()
						   : NAME_None;
	static const FName MeshAssetName(GET_MEMBER_NAME_CHECKED(UMjMesh, MeshAsset));
	if (Changed == MeshAssetName || Member == MeshAssetName)
	{
		NameFromAssetIfUnnamed();
	}
}

void UMjMesh::NameFromAssetIfUnnamed()
{
	if (MeshAsset == nullptr)
	{
		return;
	}

	// The mesh already answers to a name of its own -- an authored MJCF `name`, or
	// the basename of a `file` the way MuJoCo derives one -- so it is already in
	// the picker and nothing here renames it. This is the whole of what keeps the
	// import pass out: it sets `file` before `MeshAsset`, so a mesh it produced
	// never has an empty element name at the moment an asset lands on it.
	if (!MjAssetElementName(*this).IsEmpty())
	{
		return;
	}

	const FString Base = MjSanitizeAssetName(MeshAsset->GetName());
	if (Base.IsEmpty())
	{
		return;
	}

	// Unique among the model's other meshes, so a second hand-added cube does not
	// take the first one's name and shadow it in the picker.
	const TArray<FString> Taken = FSpecRef::OverOwner(this).NamesOfType(UMjMesh::StaticClass());
	FString Candidate = Base;
	for (int32 Ordinal = 2; Taken.Contains(Candidate); ++Ordinal)
	{
		Candidate = FString::Printf(TEXT("%s_%d"), *Base, Ordinal);
	}

	Modify();
	MjName = Candidate;
}

#endif // WITH_EDITOR
