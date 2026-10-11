// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "Render/RenderScene.h"


namespace
{
bool IsRigidTransform(const FTransform& Transform)
{
	const FQuat Rotation = Transform.GetRotation();
	return !Transform.GetTranslation().ContainsNaN() && !Transform.GetScale3D().ContainsNaN()
		&& !Rotation.ContainsNaN()
		&& Transform.GetScale3D().Equals(FVector::OneVector, KINDA_SMALL_NUMBER)
		&& Rotation.SizeSquared() > SMALL_NUMBER
		&& FMath::IsNearlyEqual(Rotation.SizeSquared(), 1.0, KINDA_SMALL_NUMBER);
}

bool IsFiniteVector(const FVector& Value)
{
	return FMath::IsFinite(Value.X) && FMath::IsFinite(Value.Y) && FMath::IsFinite(Value.Z);
}

bool ValidPrimitiveSize(ERenderGeomKind Kind, const FVector& Size)
{
	if (!IsFiniteVector(Size)) return false;
	switch (Kind)
	{
		case ERenderGeomKind::Plane: return Size.X >= 0.0 && Size.Y >= 0.0;
		case ERenderGeomKind::Sphere: return Size.X > 0.0;
		case ERenderGeomKind::Ellipsoid:
		case ERenderGeomKind::Box: return Size.X > 0.0 && Size.Y > 0.0 && Size.Z > 0.0;
		case ERenderGeomKind::Cylinder: return Size.X > 0.0 && Size.Y > 0.0;
		case ERenderGeomKind::Capsule: return Size.X > 0.0 && Size.Y >= 0.0;
		default: return true;
	}
}

bool ValidCamera(const FRenderCameraDesc& Camera)
{
	const FRenderCameraOptics& Optics = Camera.Optics;
	const FRenderCameraCaptureSettings& Capture = Camera.Capture;
	return !Camera.CanonicalName.IsEmpty() && Optics.Resolution.X > 0 && Optics.Resolution.Y > 0
		&& static_cast<uint8>(Optics.Projection)
			<= static_cast<uint8>(ERenderCameraProjection::Orthographic)
		&& static_cast<uint8>(Capture.Mode)
			<= static_cast<uint8>(ERenderCameraMode::InstanceSegmentation)
		&& FMath::IsFinite(Optics.VerticalFovDegrees) && Optics.VerticalFovDegrees > 0.0f
		&& (Optics.Projection != ERenderCameraProjection::Perspective
			|| Optics.VerticalFovDegrees < 180.0f)
		&& (Optics.Projection != ERenderCameraProjection::Orthographic
			|| (FMath::IsFinite(Optics.OrthographicWidthCm) && Optics.OrthographicWidthCm > 0.0f))
		&& FMath::IsFinite(Optics.NearClipCm)
		&& FMath::IsFinite(Optics.FarClipCm) && Optics.NearClipCm > 0.0f
		&& Optics.FarClipCm > Optics.NearClipCm
		&& (!Optics.bHasIntrinsics || (FMath::IsFinite(Optics.FocalPixelsX)
			&& FMath::IsFinite(Optics.FocalPixelsY) && FMath::IsFinite(Optics.PrincipalPixelsX)
			&& FMath::IsFinite(Optics.PrincipalPixelsY) && Optics.FocalPixelsX > 0.0f
			&& Optics.FocalPixelsY > 0.0f))
		&& FMath::IsFinite(Capture.CaptureMaxFps) && Capture.CaptureMaxFps >= 0.0f
		&& FMath::IsFinite(Capture.DepthNearCm) && FMath::IsFinite(Capture.DepthFarCm)
		&& Capture.DepthNearCm > 0.0f && Capture.DepthFarCm > Capture.DepthNearCm
		&& Capture.HistoryCapacity >= 1
		&& Capture.HistoryCapacity <= FRenderCameraHistory::MaxHistoryCapacity
		&& FMath::IsFinite(Capture.RequestActiveTtlSeconds)
		&& Capture.RequestActiveTtlSeconds >= 0.0f && FMath::IsFinite(Capture.DelaySeconds)
		&& Capture.DelaySeconds >= 0.0f && FMath::IsFinite(Capture.DelayJitterSeconds)
		&& Capture.DelayJitterSeconds >= 0.0f && FMath::IsFinite(Capture.StreamingBoost)
		&& Capture.StreamingBoost > 0.0f;
}

bool ValidLight(const FRenderLightDesc& Light)
{
	const bool bColorValid = FMath::IsFinite(Light.Color.R) && FMath::IsFinite(Light.Color.G)
		&& FMath::IsFinite(Light.Color.B) && FMath::IsFinite(Light.Color.A)
		&& Light.Color.R >= 0.0f && Light.Color.R <= 1.0f
		&& Light.Color.G >= 0.0f && Light.Color.G <= 1.0f
		&& Light.Color.B >= 0.0f && Light.Color.B <= 1.0f
		&& Light.Color.A >= 0.0f && Light.Color.A <= 1.0f;
	if (!bColorValid || !FMath::IsFinite(Light.Intensity) || Light.Intensity < 0.0f
		|| !FMath::IsFinite(Light.RangeCm) || Light.RangeCm < 0.0f
		|| !FMath::IsFinite(Light.SourceRadiusCm) || Light.SourceRadiusCm < 0.0f
		|| !FMath::IsFinite(Light.InnerConeDegrees)
		|| !FMath::IsFinite(Light.OuterConeDegrees)) return false;
	switch (Light.Kind)
	{
	case ERenderLightKind::Directional:
		return Light.RangeCm == 0.0f && Light.SourceRadiusCm == 0.0f
			&& Light.InnerConeDegrees == 0.0f && Light.OuterConeDegrees == 0.0f;
	case ERenderLightKind::Point:
		return Light.RangeCm > 0.0f && Light.SourceRadiusCm <= Light.RangeCm
			&& Light.InnerConeDegrees == 0.0f && Light.OuterConeDegrees == 0.0f;
	case ERenderLightKind::Spot:
		return Light.RangeCm > 0.0f && Light.SourceRadiusCm <= Light.RangeCm
			&& Light.InnerConeDegrees >= 0.0f
			&& Light.OuterConeDegrees >= Light.InnerConeDegrees
			&& Light.OuterConeDegrees < 90.0f;
	default: return false;
	}
}
}

bool FRenderScene::IsValid(const FRenderSceneResources& Resources, int32 VisibleGroupMask,
	FString& OutError) const
{
	OutError.Reset();
	if (SceneRevision.IsEmpty() || SceneRevision != Resources.SceneRevision)
	{
		OutError = TEXT("scene and resource revisions must be equal and nonempty");
		return false;
	}
	if (Geoms.Num() != Resources.Geoms.Num())
	{
		OutError = FString::Printf(TEXT("scene has %d geoms but resources have %d bindings"),
			Geoms.Num(), Resources.Geoms.Num());
		return false;
	}
	TSet<FString> CameraNames;
	for (int32 CameraIndex = 0; CameraIndex < Cameras.Num(); ++CameraIndex)
	{
		if (!ValidCamera(Cameras[CameraIndex]) || CameraNames.Contains(Cameras[CameraIndex].CanonicalName))
		{
			OutError = FString::Printf(TEXT("model camera %d is invalid or has an ambiguous name"),
				CameraIndex);
			return false;
		}
		CameraNames.Add(Cameras[CameraIndex].CanonicalName);
	}
	if (UserCamera.IsSet())
	{
		if (!ValidCamera(UserCamera.GetValue())
			|| CameraNames.Contains(UserCamera.GetValue().CanonicalName))
		{
			OutError = TEXT("user camera is invalid or collides with a model camera name");
			return false;
		}
	}
	for (int32 LightIndex = 0; LightIndex < Lights.Num(); ++LightIndex)
	{
		if (!ValidLight(Lights[LightIndex]))
		{
			OutError = FString::Printf(TEXT("light %d has invalid physical parameters"), LightIndex);
			return false;
		}
	}
	auto ValidColor = [](const FLinearColor& C)
	{
		return FMath::IsFinite(C.R) && FMath::IsFinite(C.G) && FMath::IsFinite(C.B)
			&& FMath::IsFinite(C.A) && C.R >= 0.0f && C.G >= 0.0f && C.B >= 0.0f
			&& C.A >= 0.0f && C.R <= 1.0f && C.G <= 1.0f && C.B <= 1.0f && C.A <= 1.0f;
	};
	if (!ValidColor(Environment.AmbientColor) || !ValidColor(Environment.SkyTop)
		|| !ValidColor(Environment.SkyBottom) || !ValidColor(Environment.HeadlightColor)
		|| !FMath::IsFinite(Environment.AmbientIntensity) || Environment.AmbientIntensity < 0.0f
		|| !FMath::IsFinite(Environment.HeadlightIntensity) || Environment.HeadlightIntensity < 0.0f
		|| (!Environment.SkyboxFaceTextureIndices.IsEmpty()
			&& Environment.SkyboxFaceTextureIndices.Num() != 6))
	{
		OutError = TEXT("render environment contains invalid colors, intensities, or skybox faces");
		return false;
	}
	for (const int32 TextureIndex : Environment.SkyboxFaceTextureIndices)
	{
		if (!Resources.Textures.IsValidIndex(TextureIndex))
		{
			OutError = TEXT("render environment references a missing skybox face texture");
			return false;
		}
	}
	TSet<FString> DeformableNames;
	for (int32 Index = 0; Index < Deformables.Num(); ++Index)
	{
		const FRenderDeformableDesc& Deformable = Deformables[Index];
		if (Deformable.Name.IsEmpty() || DeformableNames.Contains(Deformable.Name)
			|| Deformable.VertexCount < 3 || Deformable.Triangles.Num() < 3
			|| Deformable.Triangles.Num() % 3 != 0
			|| (!Deformable.UVs.IsEmpty() && Deformable.UVs.Num() != Deformable.VertexCount))
		{
			OutError = FString::Printf(TEXT("deformable %d has invalid name, vertex count, or topology"), Index);
			return false;
		}
		for (const int32 Vertex : Deformable.Triangles)
		{
			if (Vertex < 0 || Vertex >= Deformable.VertexCount)
			{
				OutError = FString::Printf(TEXT("deformable '%s' has an out-of-range triangle index"), *Deformable.Name);
				return false;
			}
		}
		for (const FVector2D& UV : Deformable.UVs)
		{
			if (!FMath::IsFinite(UV.X) || !FMath::IsFinite(UV.Y))
			{
				OutError = FString::Printf(TEXT("deformable '%s' has non-finite UVs"), *Deformable.Name);
				return false;
			}
		}
		if (Deformable.SurfaceIndex != INDEX_NONE
			&& !Resources.Deformables.IsValidIndex(Deformable.SurfaceIndex))
		{
			OutError = FString::Printf(TEXT("deformable '%s' has an invalid surface binding"), *Deformable.Name);
			return false;
		}
		DeformableNames.Add(Deformable.Name);
	}
	TSet<FString> Features;
	for (const FString& Feature : Visualization.Features)
	{
		if (Feature.IsEmpty() || Features.Contains(Feature))
		{
			OutError = TEXT("visualization feature names must be nonempty and unique");
			return false;
		}
		Features.Add(Feature);
	}
	for (int32 GeomIndex = 0; GeomIndex < Geoms.Num(); ++GeomIndex)
	{
		const FRenderGeomDesc& Geom = Geoms[GeomIndex];
		if (static_cast<uint8>(Geom.Kind) > static_cast<uint8>(ERenderGeomKind::Mesh))
		{
			OutError = FString::Printf(TEXT("geom %d has an invalid kind"), GeomIndex);
			return false;
		}
		if (!Bodies.IsValidIndex(Geom.BodyIndex))
		{
			OutError = FString::Printf(TEXT("geom %d has invalid body index %d"), GeomIndex,
				Geom.BodyIndex);
			return false;
		}
		if (Geom.Group < 0 || Geom.Group > 30 || !IsRigidTransform(Geom.BodyLocalTransform))
		{
			OutError = FString::Printf(TEXT("geom %d has invalid group or local transform"), GeomIndex);
			return false;
		}
		if (!ValidPrimitiveSize(Geom.Kind, Geom.SizeCm))
		{
			OutError = FString::Printf(TEXT("geom %d has invalid primitive dimensions"), GeomIndex);
			return false;
		}
		const bool bVisible = (VisibleGroupMask & (1 << Geom.Group)) != 0
			&& Resources.Geoms[GeomIndex].Rgba.A > 0.0f;
		if (bVisible && Geom.Kind == ERenderGeomKind::Unsupported)
		{
			OutError = FString::Printf(TEXT("visible geom %d has an unsupported kind"), GeomIndex);
			return false;
		}
		if (Geom.Kind != ERenderGeomKind::Mesh && Geom.MeshResourceIndex != INDEX_NONE)
		{
			OutError = FString::Printf(TEXT("primitive geom %d carries an ignored mesh index"),
				GeomIndex);
			return false;
		}
		if (Geom.Kind == ERenderGeomKind::Mesh
			&& !Resources.Meshes.IsValidIndex(Geom.MeshResourceIndex))
		{
			OutError = FString::Printf(TEXT("mesh geom %d has no compiled mesh resource"),
				GeomIndex);
			return false;
		}
	}
	return true;
}
