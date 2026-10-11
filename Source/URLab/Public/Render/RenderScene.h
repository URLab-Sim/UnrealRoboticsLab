// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
#pragma once

#include "CoreMinimal.h"
#include "Render/RenderCameraTypes.h"
#include "Render/RenderFrame.h"
#include "Render/RenderSceneResources.h"

enum class ERenderGeomKind : uint8
{
	Unsupported,
	Plane,
	Sphere,
	Ellipsoid,
	Cylinder,
	Capsule,
	Box,
	Mesh,
};

enum class ERenderLightKind : uint8
{
	Directional,
	Point,
	Spot,
};

class UTexture2D;

/** Physics-independent, fully resolved light authored in Unreal rendering units. */
struct URLAB_API FRenderLightDesc
{
	FString Name;
	ERenderLightKind Kind = ERenderLightKind::Point;
	bool bEnabled = true;
	FLinearColor Color = FLinearColor::White;
	/** Lux for directional lights; candelas for point and spot lights. */
	float Intensity = 0.0f;
	bool bCastShadows = false;
	float RangeCm = 0.0f;
	float SourceRadiusCm = 0.0f;
	float OuterConeDegrees = 0.0f;
	float InnerConeDegrees = 0.0f;
};

struct FRenderBodyDesc
{
	FString Name;
};

/** Resolved material parameters used by explicit renderer debug overrides. */
struct FRenderResolvedMaterialOverride
{
	bool bEnabled = false;
	FRenderMaterialResource Material;
	FLinearColor BaseColor = FLinearColor::White;
	FVector2D TexUniformScale = FVector2D::ZeroVector;
	TArray<TObjectPtr<UTexture2D>> RoleTextures;
};

struct FRenderGeomDesc
{
	int32 BodyIndex = INDEX_NONE;
	int32 Group = 0;
	ERenderGeomKind Kind = ERenderGeomKind::Unsupported;
	/** Body-local rigid transform, finalized for the selected drawable, UE cm/LHS. */
	FTransform BodyLocalTransform = FTransform::Identity;
	/** Primitive half dimensions in UE centimetres. */
	FVector SizeCm = FVector::ZeroVector;
	/** Render resource mesh index for the procedural/fallback mesh path. */
	int32 MeshResourceIndex = INDEX_NONE;
	FString Name;
	FString MeshName;
};

/** Physics-neutral deformable surface. Its topology is an owned render resource;
 *  each frame supplies the current Cartesian vertex positions. */
struct URLAB_API FRenderDeformableDesc
{
	FString Name;
	int32 VertexCount = 0;
	TArray<int32> Triangles;
	/** Immutable per-vertex UVs; empty means this surface has no texture coordinates. */
	TArray<FVector2D> UVs;
	int32 SurfaceIndex = INDEX_NONE;
};

/** Named feature selection for adapter-produced debug drawables. Empty means none. */
struct URLAB_API FRenderVisualizationConfig
{
	TArray<FString> Features;
	bool IsEnabled(const TCHAR* Feature) const { return Features.ContainsByPredicate(
		[Feature](const FString& Value) { return Value == Feature; }); }
};

/** Renderer-owned environment description, independent of the source physics engine. */
struct URLAB_API FRenderEnvironmentDesc
{
	FLinearColor AmbientColor = FLinearColor::Black;
	float AmbientIntensity = 0.0f;
	FLinearColor SkyTop = FLinearColor::Black;
	FLinearColor SkyBottom = FLinearColor::Black;
	bool bSkyboxEnabled = false;
	bool bHeadlightEnabled = false;
	FLinearColor HeadlightColor = FLinearColor::White;
	float HeadlightIntensity = 0.0f;
	/** Six source texture indices in +X,-X,+Y,-Y,+Z,-Z order; empty means gradient only. */
	TArray<int32> SkyboxFaceTextureIndices;
};

/** One sparse, capture-scoped body-local geometry pose in UE centimetres/LHS. */
struct URLAB_API FRenderGeomPoseOverride
{
	int32 GeomIndex = INDEX_NONE;
	FTransform BodyLocalTransform = FTransform::Identity;
};

/** Opaque token for restoring one exact scene's active geometry override scope. */
struct URLAB_API FRenderGeomPoseOverrideToken
{
	FString SceneRevision;
	uint64 Serial = 0;
	TArray<FRenderGeomPoseOverride> Baseline;
	bool IsValid() const { return Serial != 0; }
};

struct URLAB_API FRenderScene
{
	FString SceneRevision;
	TArray<FRenderBodyDesc> Bodies;
	TArray<FRenderGeomDesc> Geoms;
	TArray<FRenderDeformableDesc> Deformables;
	TArray<FRenderCameraDesc> Cameras;
	/** Operator camera identity is separate from model-camera indices. */
	TOptional<FRenderCameraDesc> UserCamera;
	/** Ordered scene lights; frame transforms use this exact order. */
	TArray<FRenderLightDesc> Lights;
	FRenderVisualizationConfig Visualization;
	FRenderEnvironmentDesc Environment;

	bool IsValid(const FRenderSceneResources& Resources, int32 VisibleGroupMask,
		FString& OutError) const;
};

class URLAB_API IRenderSceneResourceLifetimeLease
{
public:
	virtual ~IRenderSceneResourceLifetimeLease() = default;
};
using FRenderSceneResourceLifetimeLeasePtr =
	TSharedPtr<const IRenderSceneResourceLifetimeLease, ESPMode::ThreadSafe>;

/** Move-only complete install input; arrays are destroyed before the optional lease. */
struct URLAB_API FRenderSceneInstallPayload
{
	FRenderSceneResourceLifetimeLeasePtr LifetimeLease;
	FRenderScene Scene;
	FRenderSceneResources Resources;
	FRenderFrame InitialFrame;

	FRenderSceneInstallPayload() = default;
	FRenderSceneInstallPayload(FRenderScene&& InScene, FRenderSceneResources&& InResources,
		FRenderFrame&& InInitialFrame,
		FRenderSceneResourceLifetimeLeasePtr InLifetimeLease = nullptr)
		: LifetimeLease(MoveTemp(InLifetimeLease)), Scene(MoveTemp(InScene)),
		  Resources(MoveTemp(InResources)), InitialFrame(MoveTemp(InInitialFrame)) {}
	FRenderSceneInstallPayload(const FRenderSceneInstallPayload&) = delete;
	FRenderSceneInstallPayload& operator=(const FRenderSceneInstallPayload&) = delete;
	FRenderSceneInstallPayload(FRenderSceneInstallPayload&&) = default;
	FRenderSceneInstallPayload& operator=(FRenderSceneInstallPayload&& Other)
	{
		if (this == &Other) return *this;
		InitialFrame = FRenderFrame();
		Resources = FRenderSceneResources();
		Scene = FRenderScene();
		LifetimeLease.Reset();
		LifetimeLease = MoveTemp(Other.LifetimeLease);
		Scene = MoveTemp(Other.Scene);
		Resources = MoveTemp(Other.Resources);
		InitialFrame = MoveTemp(Other.InitialFrame);
		return *this;
	}
};

struct FRenderSceneInstallOptions
{
	/** Scene actor world placement. Its translation contains SceneOrigin exactly once. */
	FTransform SceneTransform = FTransform::Identity;
	int32 VisibleGroupMask = 0b0000111;
	bool bForceRebuildAssets = false;
	/** Editor preview actors remain saveable; runtime actors are transient. */
	bool bPersistentEditorActors = false;
	/** False lets an owning host commit its related state before exposing the new tree. */
	bool bInitiallyVisible = true;
};
