// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Elements/MjMesh.h"

#include "Chaos/TriangleMeshImplicitObject.h"
#include "Engine/StaticMesh.h"
#include "Misc/Paths.h"
#include "PhysicsEngine/BodySetup.h"

#include "MuJoCo/Spec/MjAssetFiles.h"
#include "MuJoCo/Spec/MjAssetResolve.h"
#include "MuJoCo/Spec/MjAssetSink.h"
#include "Utils/MeshUtils.h"
#include "Utils/URLabLogging.h"

bool UMjMesh::IsFileStale() const
{
	return MeshAsset != nullptr && FSoftObjectPath(MeshAsset.Get()) != FileAsset;
}

bool UMjMesh::DumpAssetToFile(const FString& BaseDirectory)
{
	if (MeshAsset == nullptr)
	{
		return false;
	}

	// The triangles Chaos cooked, which is the only vertex-and-index form a
	// UStaticMesh offers outside the editor's own mesh description API. A mesh
	// with no cooked collision has none, and there is nothing to write.
	UBodySetup* BodySetup = MeshAsset->GetBodySetup();
	if (BodySetup == nullptr || BodySetup->TriMeshGeometries.Num() == 0)
	{
		UE_LOG(LogURLab, Warning, TEXT("[MjMesh] '%s' has no collision geometry to export from '%s'"),
			*MjAssetElementName(*this), *MeshAsset->GetName());
		return false;
	}

	const FString Name = MjSanitizeAssetName(MjAssetElementName(*this));
	if (Name.IsEmpty())
	{
		return false;
	}
	const FMjDumpTarget Target = MjDumpTargetFor(BaseDirectory, Name, TEXT(".obj"));

	// SaveMeshAsOBJSimple is the export the convex-decomposition path already
	// uses: centimetres to metres, Unreal's left-handed Y flipped, and the
	// winding reversed to match. Reused rather than reimplemented.
	auto& TriGeom = BodySetup->TriMeshGeometries[0];
	auto& Vertices = TriGeom.GetReference()->Particles().X();
	const int32 Written = TriGeom.GetReference()->Elements().RequiresLargeIndices()
		? MeshUtils::SaveMeshAsOBJSimple(
			  Target.FullPath, Vertices, TriGeom.GetReference()->Elements().GetLargeIndexBuffer())
		: MeshUtils::SaveMeshAsOBJSimple(
			  Target.FullPath, Vertices, TriGeom.GetReference()->Elements().GetSmallIndexBuffer());
	if (Written == 0)
	{
		UE_LOG(LogURLab, Error, TEXT("[MjMesh] could not write '%s'"), *Target.FullPath);
		return false;
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
}
