// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
#pragma once

#include "CoreMinimal.h"

struct FRenderScene;

/** Identity shared by every payload applied to one installed render scene revision. */
struct FRenderFrameIdentity
{
	FString SceneRevision;
	uint64 FrameId = 0;
	double SourceTime = 0.0;
	uint64 ClockEpoch = 0;
	bool bHasSourceTime = false;
};

/**
 * One rigid Cartesian frame. BodySceneTransforms is indexed by the installed scene's
 * body array and contains exactly one rigid, unit-scale transform per body. Transforms
 * are relative to the render-scene actor root, in Unreal centimetres and left-handed
 * axes. The actor transform carries the scene origin exactly once.
 *
 * Camera, deformable and overlay payloads will extend this top-level frame after their
 * existing host implementations migrate; they share Identity rather than
 * introducing independent revision or clock fields.
 */
struct URLAB_API FRenderFrame
{
	FRenderFrameIdentity Identity;
	TArray<FTransform> BodySceneTransforms;
	TArray<FTransform> CameraSceneTransforms;
	TOptional<FTransform> UserCameraSceneTransform;
	/** Dense scene-root-relative transforms in the installed scene light order. */
	TArray<FTransform> LightSceneTransforms;
	/** Dynamic surface positions and optional normals, indexed by installed deformable. */
	struct FDeformable
	{
		FString Name;
		TArray<FVector> Positions;
		TArray<FVector> Normals;
	};
	TArray<FDeformable> Deformables;
	/** Adapter-produced generic drawable meshes; colours are linear per vertex or default white. */
	struct FDebugMesh
	{
		FString Name;
		FLinearColor Color = FLinearColor::White;
		TArray<FVector> Positions;
		TArray<int32> Triangles;
	};
	TArray<FDebugMesh> DebugMeshes;

	bool IsValid(const FRenderScene& Scene, FString& OutError) const;
	/** Structural check for camera/history helpers which only know rigid array sizes.
	 *  Scene installation ingress must use the complete-scene overload above. */
	bool IsValid(int32 ExpectedBodies, int32 ExpectedCameras, bool bExpectedUserCamera,
		int32 ExpectedLights, const FString& ExpectedRevision, FString& OutError) const;
};
