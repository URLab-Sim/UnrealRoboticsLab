// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
#pragma once

#include "CoreMinimal.h"

/**
 * Immutable CPU-side inputs for renderer asset construction. Mesh positions are Unreal
 * centimetres in Unreal's left-handed axes; normals are unitless in the same axes and UVs
 * use the source image's row order. Every array and string owns its storage.
 * Indices are scoped to SceneRevision.
 */

/** Texture slots understood by URLab's renderer master material. */
enum class ERenderMaterialRole : uint8
{
	Rgb,
	Occlusion,
	Roughness,
	Metallic,
	Normal,
	Opacity,
	Emissive,
	Rgba,
	Orm,
	Count
};

struct FRenderTextureBinding
{
	int32 TextureIndex = INDEX_NONE;
	bool bSRGB = false;
	bool bNormal = false;
};

struct FRenderMaterialResource
{
	FLinearColor Rgba = FLinearColor::White;
	float Emission = 0.0f, Specular = 0.5f, Shininess = 0.5f, Reflectance = 0.0f;
	/** Final renderer values. Source-specific unset sentinels are resolved by its adapter. */
	float Metallic = 0.0f, Roughness = 0.8f;
	FVector2D TexRepeat = FVector2D(1.0, 1.0);
	bool bTexUniform = false;
	FRenderTextureBinding Textures[static_cast<int32>(ERenderMaterialRole::Count)];
};

struct FRenderTextureResource
{
	int32 Width = 0, Height = 0;
	TArray<uint8> Rgba8;
};

struct FRenderMeshResource
{
	TArray<FVector> Positions, Normals;
	TArray<FVector2D> UVs;
	TArray<int32> Triangles;
};

/** Resolved appearance of a drawable whose material may be absent. */
struct FRenderSurfaceBinding
{
	int32 MaterialIndex = INDEX_NONE;
	FLinearColor Rgba = FLinearColor::White;
	/** Final dimensionless multiplier used only by spatial texuniform mapping. */
	FVector2D TexUniformScale = FVector2D::ZeroVector;
};

struct URLAB_API FRenderSceneResources
{
	FString SceneRevision;
	TArray<FRenderMeshResource> Meshes;
	TArray<FRenderTextureResource> Textures;
	TArray<FRenderMaterialResource> Materials;
	/** Indexed in the caller's geom index space for this SceneRevision. */
	TArray<FRenderSurfaceBinding> Geoms;
	/** Surface appearance entries referenced by FRenderDeformableDesc::SurfaceIndex. */
	TArray<FRenderSurfaceBinding> Deformables;

	bool IsValid(FString& OutError) const;
	/** Stable field-wise hash of render-affecting resources; excludes SceneRevision. */
	FString ContentHash() const;
};
