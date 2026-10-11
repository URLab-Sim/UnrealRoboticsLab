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

#include "Chaos/ArrayCollectionArray.h"

#include "EngineUtils.h"
#include "Misc/FileHelper.h"
#include "Utils/URLabLogging.h"
#include "CoreMinimal.h"
#include "Engine/Texture2D.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "LandscapeComponent.h"
#include "Utils/MJHelper.h"
#include "CoACD/CoacdInterface.h"
#include "Utils/CoordinateConversion.h"
namespace MeshUtils
{

struct TotalLandscapeData
{

	TArray<float> HeightData;
	TArray<float> HeightDataNormalised;
	MJHelper::HeightFieldData HeightFieldData;
	int NumRows;
};

ALandscape* FindLandscapeActor(UWorld* World);

template <typename VertexType, typename IndexType>
int SaveMeshAsOBJSimple(const FString& FilePath, const Chaos::TArrayCollectionArray<VertexType>& Vertices,
	const TArray<Chaos::TVector<IndexType, 3>>& Indices)
{
	FString OutputString;

	// Write vertices
	for (const auto& Vertex : Vertices)
	{
		const FVector UEPosition(static_cast<double>(Vertex.X), static_cast<double>(Vertex.Y),
			static_cast<double>(Vertex.Z));
		double MjPosition[3];
		URLabCoordinateConversion::UnrealCentimetresPositionToRightHandedMetres(UEPosition, MjPosition);
		OutputString += FString::Printf(TEXT("v %.9f %.9f %.9f\n"), MjPosition[0], MjPosition[1], MjPosition[2]);
	}

	// Write indices
	for (const auto& Index : Indices)
	{
		OutputString += FString::Printf(TEXT("f %d %d %d\n"), Index.Z + 1, Index.Y + 1, Index.X + 1);
	}

	return FFileHelper::SaveStringToFile(OutputString, *FilePath) ? 1 : 0;
}

/** Export visual positions with per-corner UVs/normals to a MuJoCo-ready OBJ.
 *
 * Triangles index Positions; CornerNormals and optional CornerUVs are laid out
 * three entries per triangle in the same source-corner order. Reflection across
 * Y and its corresponding winding reversal match SaveMeshAsOBJSimple. OBJ's
 * independent corner indices keep UV seams and hard normals intact.
 */
inline int SaveVisualMeshAsOBJ(const FString& FilePath,
	const TArray<FVector>& Positions, const TArray<FIntVector>& Triangles,
	const TArray<FVector>& CornerNormals, const TArray<FVector2D>& CornerUVs)
{
	if (Positions.IsEmpty() || Triangles.IsEmpty() ||
		(!CornerNormals.IsEmpty() && CornerNormals.Num() != Triangles.Num() * 3) ||
		(!CornerUVs.IsEmpty() && CornerUVs.Num() != Triangles.Num() * 3))
	{
		return 0;
	}

	FString OutputString;
	for (const FVector& Position : Positions)
	{
		double MjPosition[3];
		URLabCoordinateConversion::UnrealCentimetresPositionToRightHandedMetres(Position, MjPosition);
		OutputString += FString::Printf(TEXT("v %.9f %.9f %.9f\n"), MjPosition[0], MjPosition[1], MjPosition[2]);
	}
	for (const FVector2D& UV : CornerUVs)
	{
		// MuJoCo's OBJ decoder flips V while reading. Write the inverse mapping
		// so native mesh texcoords (and the UE render adapter) recover this LOD's
		// authored Unreal UV convention.
		OutputString += FString::Printf(TEXT("vt %.9f %.9f\n"), UV.X, 1.0 - UV.Y);
	}
	for (const FVector& Normal : CornerNormals)
	{
		double MjNormal[3];
		URLabCoordinateConversion::UnrealDirectionToRightHanded(Normal.GetSafeNormal(), MjNormal);
		OutputString += FString::Printf(TEXT("vn %.9f %.9f %.9f\n"), MjNormal[0], MjNormal[1], MjNormal[2]);
	}

	for (int32 TriangleIndex = 0; TriangleIndex < Triangles.Num(); ++TriangleIndex)
	{
		const FIntVector& Triangle = Triangles[TriangleIndex];
		if (!Positions.IsValidIndex(Triangle.X) || !Positions.IsValidIndex(Triangle.Y) ||
			!Positions.IsValidIndex(Triangle.Z))
		{
			return 0;
		}
		const int32 VertexIndices[3] = {Triangle.X, Triangle.Y, Triangle.Z};
		OutputString += TEXT("f");
		for (int32 ObjCorner = 0; ObjCorner < 3; ++ObjCorner)
		{
			const int32 SourceCorner = 2 - ObjCorner;
			const int32 VertexIndex = VertexIndices[SourceCorner] + 1;
			const int32 AttributeIndex = TriangleIndex * 3 + SourceCorner + 1;
			if (CornerUVs.IsEmpty() && CornerNormals.IsEmpty())
			{
				OutputString += FString::Printf(TEXT(" %d"), VertexIndex);
			}
			else if (CornerUVs.IsEmpty())
			{
				OutputString += FString::Printf(TEXT(" %d//%d"), VertexIndex, AttributeIndex);
			}
			else if (CornerNormals.IsEmpty())
			{
				OutputString += FString::Printf(TEXT(" %d/%d"), VertexIndex, AttributeIndex);
			}
			else
			{
				OutputString += FString::Printf(TEXT(" %d/%d/%d"), VertexIndex,
					AttributeIndex, AttributeIndex);
			}
		}
		OutputString += TEXT("\n");
	}
	return FFileHelper::SaveStringToFile(OutputString, *FilePath) ? 1 : 0;
}

template <typename VertexType, typename IndexType>
int SaveMeshAsOBJComplex(const FString& FilePath, const Chaos::TArrayCollectionArray<VertexType>& Vertices,
	const TArray<Chaos::TVector<IndexType, 3>>& Indices, float Threshold = 0.05f,
	bool bExtrude = false, double ExtrudeMargin = 0.01)
{
	CoACD_Mesh inputMesh = CoacdInterface::ConvertToCoACDMesh(Vertices, Indices);

	CoACD_MeshArray result = CoACD_run(inputMesh, Threshold, -1, preprocess_auto, 30, 1000, 20, 150, 3, false, true, false, 100,
		bExtrude, ExtrudeMargin, apx_ch, 0, false);
	int MeshCount = result.meshes_count;

	UE_LOG(LogURLab, Log, TEXT("CoACD: %d convex hulls created (threshold=%.3f, extrude=%s margin=%.3f)"),
		result.meshes_count, Threshold, bExtrude ? TEXT("on") : TEXT("off"), ExtrudeMargin);

	CoacdInterface::SaveCoACDMeshArrayAsOBJ(result, FilePath);
	CoACD_freeMeshArray(result);
	return MeshCount;
}

template <typename VertexType, typename IndexType>
int SaveMesh(const FString& FilePath, const Chaos::TArrayCollectionArray<VertexType>& Vertices,
	const TArray<Chaos::TVector<IndexType, 3>>& Indices, bool ComplexMeshRequired, float Threshold = 0.05f,
	bool bExtrude = false, double ExtrudeMargin = 0.01)
{
	if (ComplexMeshRequired)
		return MeshUtils::SaveMeshAsOBJComplex(FilePath, Vertices, Indices, Threshold, bExtrude, ExtrudeMargin);

	return MeshUtils::SaveMeshAsOBJSimple(FilePath, Vertices, Indices);
}

} // namespace MeshUtils
