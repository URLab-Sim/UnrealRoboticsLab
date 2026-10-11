// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "Render/RenderFrame.h"
#include "Render/RenderScene.h"

namespace
{
bool IsRigidTransform(const FTransform& Transform)
{
	const FVector Translation = Transform.GetTranslation();
	const FVector Scale = Transform.GetScale3D();
	const FQuat Rotation = Transform.GetRotation();
	return !Translation.ContainsNaN() && !Scale.ContainsNaN() && !Rotation.ContainsNaN()
		&& Scale.Equals(FVector::OneVector, KINDA_SMALL_NUMBER)
		&& Rotation.SizeSquared() > SMALL_NUMBER
		&& FMath::IsNearlyEqual(Rotation.SizeSquared(), 1.0, KINDA_SMALL_NUMBER);
}
}

bool FRenderFrame::IsValid(const FRenderScene& Scene, FString& OutError) const
{
	OutError.Reset();
	if (Identity.SceneRevision.IsEmpty() || Identity.SceneRevision != Scene.SceneRevision)
	{
		OutError = FString::Printf(TEXT("frame revision '%s' does not match scene revision '%s'"),
			*Identity.SceneRevision, *Scene.SceneRevision);
		return false;
	}
	if (Identity.bHasSourceTime && !FMath::IsFinite(Identity.SourceTime))
	{
		OutError = TEXT("frame source time is not finite");
		return false;
	}
	if (BodySceneTransforms.Num() != Scene.Bodies.Num())
	{
		OutError = FString::Printf(TEXT("frame has %d body transforms; expected exactly %d"),
			BodySceneTransforms.Num(), Scene.Bodies.Num());
		return false;
	}
	if (CameraSceneTransforms.Num() != Scene.Cameras.Num()
		|| UserCameraSceneTransform.IsSet() != Scene.UserCamera.IsSet())
	{
		OutError = FString::Printf(TEXT("frame has %d model camera transforms and user=%d; expected %d and user=%d"),
			CameraSceneTransforms.Num(), UserCameraSceneTransform.IsSet(), Scene.Cameras.Num(),
			Scene.UserCamera.IsSet());
		return false;
	}
	for (int32 BodyIndex = 0; BodyIndex < BodySceneTransforms.Num(); ++BodyIndex)
	{
		if (!IsRigidTransform(BodySceneTransforms[BodyIndex]))
		{
			OutError = FString::Printf(TEXT("body transform %d is non-finite, scaled, or non-unit"),
				BodyIndex);
			return false;
		}
	}
	for (int32 CameraIndex = 0; CameraIndex < CameraSceneTransforms.Num(); ++CameraIndex)
	{
		if (!IsRigidTransform(CameraSceneTransforms[CameraIndex]))
		{
			OutError = FString::Printf(TEXT("camera transform %d is non-finite, scaled, or non-unit"),
				CameraIndex);
			return false;
		}
	}
	if (UserCameraSceneTransform.IsSet() && !IsRigidTransform(UserCameraSceneTransform.GetValue()))
	{
		OutError = TEXT("user camera transform is non-finite, scaled, or non-unit");
		return false;
	}
	if (LightSceneTransforms.Num() != Scene.Lights.Num())
	{
		OutError = FString::Printf(TEXT("frame has %d light transforms; expected exactly %d"),
			LightSceneTransforms.Num(), Scene.Lights.Num());
		return false;
	}
	for (int32 LightIndex = 0; LightIndex < LightSceneTransforms.Num(); ++LightIndex)
	{
		if (!IsRigidTransform(LightSceneTransforms[LightIndex]))
		{
			OutError = FString::Printf(TEXT("light transform %d is non-finite, scaled, or non-unit"),
				LightIndex);
			return false;
		}
	}
	if (Deformables.Num() != Scene.Deformables.Num())
	{
		OutError = FString::Printf(TEXT("frame has %d deformables; expected exactly %d"),
			Deformables.Num(), Scene.Deformables.Num());
		return false;
	}
	if (DebugMeshes.Num() > 4096 || Deformables.Num() > 4096)
	{
		OutError = TEXT("render frame exceeds the dynamic mesh count limit");
		return false;
	}
	int64 TotalVertices = 0, TotalIndices = 0;
	for (int32 Index = 0; Index < Deformables.Num(); ++Index)
	{
		const FDeformable& Dynamic = Deformables[Index];
		const FRenderDeformableDesc& Static = Scene.Deformables[Index];
		if (Dynamic.Name != Static.Name)
		{
			OutError = FString::Printf(TEXT("deformable frame %d does not match installed scene order"), Index);
			return false;
		}
		if (Static.VertexCount < 3 || Dynamic.Positions.Num() != Static.VertexCount
			|| (!Dynamic.Normals.IsEmpty()
			&& Dynamic.Normals.Num() != Dynamic.Positions.Num()))
		{
			OutError = FString::Printf(TEXT("deformable '%s' has invalid positions or normals"), *Dynamic.Name);
			return false;
		}
		TotalVertices += Dynamic.Positions.Num();
		TotalIndices += Static.Triangles.Num();
		for (int32 Vertex = 0; Vertex < Dynamic.Positions.Num(); ++Vertex)
		{
			if (Dynamic.Positions[Vertex].ContainsNaN()
				|| (!Dynamic.Normals.IsEmpty() && Dynamic.Normals[Vertex].ContainsNaN()))
			{
				OutError = FString::Printf(TEXT("deformable '%s' has non-finite vertex data"), *Dynamic.Name);
				return false;
			}
		}
	}
	if (TotalVertices > 1000000 || TotalIndices > 3000000
		|| (TotalVertices * 24 + TotalIndices * 4) > 128ll * 1024 * 1024)
	{
		OutError = TEXT("render frame exceeds the dynamic mesh size limit");
		return false;
	}
	for (const FDebugMesh& Mesh : DebugMeshes)
	{
		TotalVertices += Mesh.Positions.Num();
		TotalIndices += Mesh.Triangles.Num();
		if (TotalVertices > 1000000 || TotalIndices > 3000000
			|| (TotalVertices * 24 + TotalIndices * 4) > 128ll * 1024 * 1024)
		{
			OutError = TEXT("render frame exceeds the dynamic mesh size limit");
			return false;
		}
		if (Mesh.Name.IsEmpty() || Mesh.Positions.Num() < 3 || Mesh.Triangles.Num() < 3
			|| Mesh.Triangles.Num() % 3 != 0 || !FMath::IsFinite(Mesh.Color.R)
			|| !FMath::IsFinite(Mesh.Color.G) || !FMath::IsFinite(Mesh.Color.B)
			|| !FMath::IsFinite(Mesh.Color.A))
		{
			OutError = TEXT("debug mesh has invalid name, topology, or colour count");
			return false;
		}
		for (const FVector& Position : Mesh.Positions)
			if (Position.ContainsNaN()) { OutError = TEXT("debug mesh position is non-finite"); return false; }
		for (const int32 Triangle : Mesh.Triangles)
			if (!Mesh.Positions.IsValidIndex(Triangle)) { OutError = TEXT("debug mesh index is out of range"); return false; }
	}
	return true;
}

bool FRenderFrame::IsValid(int32 ExpectedBodies, int32 ExpectedCameras,
	bool bExpectedUserCamera, int32 ExpectedLights, const FString& ExpectedRevision,
	FString& OutError) const
{
	FRenderScene StructuralScene;
	StructuralScene.SceneRevision = ExpectedRevision;
	StructuralScene.Bodies.SetNum(ExpectedBodies);
	StructuralScene.Cameras.SetNum(ExpectedCameras);
	if (bExpectedUserCamera) StructuralScene.UserCamera.Emplace();
	StructuralScene.Lights.SetNum(ExpectedLights);
	for (const FDeformable& Dynamic : Deformables)
	{
		FRenderDeformableDesc& Static = StructuralScene.Deformables.AddDefaulted_GetRef();
		Static.Name = Dynamic.Name;
		Static.VertexCount = Dynamic.Positions.Num();
		Static.Triangles = {0, 1, 2};
	}
	return IsValid(StructuralScene, OutError);
}
