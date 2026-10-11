#pragma once

#include "CoreMinimal.h"
#include "Render/RenderCameraTypes.h"
#include "Render/RenderScene.h"

struct URLAB_API FURLabRenderMaterialPatch
{
	int32 Index = INDEX_NONE;
	TOptional<FLinearColor> Rgba;
	TOptional<float> Emission, Specular, Shininess, Reflectance, Metallic, Roughness;
	TOptional<FVector2D> TexRepeat;
	TOptional<bool> bTexUniform;
};

struct URLAB_API FURLabRenderGeomSurfacePatch
{
	int32 Index = INDEX_NONE;
	TOptional<int32> MaterialIndex;
	TOptional<FLinearColor> Rgba;
	TOptional<FVector2D> TexUniformScale;
};

struct URLAB_API FURLabRenderLightPatch
{
	FString Name;
	TOptional<bool> bEnabled, bCastShadows;
	TOptional<FLinearColor> Color;
	TOptional<float> Intensity, RangeCm, SourceRadiusCm, InnerConeDegrees, OuterConeDegrees;
};

struct URLAB_API FURLabRenderCameraPatch
{
	FString Name;
	TOptional<ERenderCameraMode> Mode;
	TOptional<FRenderCameraOptics> Optics;
	TOptional<float> DepthNearCm, DepthFarCm;
};

struct URLAB_API FURLabRenderGeomPosePatch
{
	int32 Index = INDEX_NONE;
	FTransform BodyLocalTransform = FTransform::Identity;
};

struct URLAB_API FURLabRenderEnvironmentPatch
{
	TOptional<FLinearColor> AmbientColor, SkyTop, SkyBottom, HeadlightColor;
	TOptional<float> AmbientIntensity, HeadlightIntensity;
	TOptional<bool> bSkyboxEnabled, bHeadlightEnabled;
};

/** Complete atomic patch DTO. An omitted section leaves that installed value intact. */
struct URLAB_API FURLabRenderScenePatch
{
	TArray<FURLabRenderMaterialPatch> Materials;
	TArray<FURLabRenderGeomSurfacePatch> GeomSurfaces;
	TArray<FURLabRenderLightPatch> Lights;
	TArray<FURLabRenderCameraPatch> Cameras;
	TArray<FURLabRenderGeomPosePatch> GeomPoses;
	TOptional<FURLabRenderEnvironmentPatch> Environment;
	TOptional<TArray<FString>> VisualizationFeatures;

	bool Apply(FRenderScene& Scene, FRenderSceneResources& Resources,
		int32 VisibleGroupMask, FString& OutError) const;
};
