#include "Workflows/RenderScenePatch.h"

namespace
{
bool ValidColor(const FLinearColor& Color)
{
	return FMath::IsFinite(Color.R) && FMath::IsFinite(Color.G)
		&& FMath::IsFinite(Color.B) && FMath::IsFinite(Color.A)
		&& Color.R >= 0.0f && Color.R <= 1.0f
		&& Color.G >= 0.0f && Color.G <= 1.0f
		&& Color.B >= 0.0f && Color.B <= 1.0f
		&& Color.A >= 0.0f && Color.A <= 1.0f;
}

bool ValidVector2(const FVector2D& Value)
{
	return FMath::IsFinite(Value.X) && FMath::IsFinite(Value.Y)
		&& Value.X >= 0.0 && Value.Y >= 0.0;
}

bool HasMaterialValue(const FURLabRenderMaterialPatch& Patch)
{
	return Patch.Rgba.IsSet() || Patch.Emission.IsSet() || Patch.Specular.IsSet()
		|| Patch.Shininess.IsSet() || Patch.Reflectance.IsSet() || Patch.Metallic.IsSet()
		|| Patch.Roughness.IsSet() || Patch.TexRepeat.IsSet() || Patch.bTexUniform.IsSet();
}

bool HasGeomSurfaceValue(const FURLabRenderGeomSurfacePatch& Patch)
{
	return Patch.MaterialIndex.IsSet() || Patch.Rgba.IsSet() || Patch.TexUniformScale.IsSet();
}

bool HasLightValue(const FURLabRenderLightPatch& Patch)
{
	return Patch.bEnabled.IsSet() || Patch.bCastShadows.IsSet() || Patch.Color.IsSet()
		|| Patch.Intensity.IsSet() || Patch.RangeCm.IsSet() || Patch.SourceRadiusCm.IsSet()
		|| Patch.InnerConeDegrees.IsSet() || Patch.OuterConeDegrees.IsSet();
}

bool HasCameraValue(const FURLabRenderCameraPatch& Patch)
{
	return Patch.Mode.IsSet() || Patch.Optics.IsSet() || Patch.DepthNearCm.IsSet()
		|| Patch.DepthFarCm.IsSet();
}

bool ValidMaterialPatch(const FURLabRenderMaterialPatch& Patch)
{
	if (Patch.Index < 0 || !HasMaterialValue(Patch)) return false;
	if (Patch.Rgba.IsSet() && !ValidColor(Patch.Rgba.GetValue())) return false;
	if (Patch.TexRepeat.IsSet() && !ValidVector2(Patch.TexRepeat.GetValue())) return false;
	const TOptional<float>* Scalars[] = {&Patch.Emission, &Patch.Specular, &Patch.Shininess,
		&Patch.Reflectance, &Patch.Metallic, &Patch.Roughness};
	for (const TOptional<float>* Value : Scalars)
		if (Value->IsSet() && (!FMath::IsFinite(Value->GetValue()) || Value->GetValue() < 0.0f))
			return false;
	return !Patch.Metallic.IsSet() || Patch.Metallic.GetValue() <= 1.0f
		? (!Patch.Roughness.IsSet() || Patch.Roughness.GetValue() <= 1.0f) : false;
}

bool ValidEnvironmentPatch(const FURLabRenderEnvironmentPatch& Patch)
{
	if (Patch.AmbientColor.IsSet() && !ValidColor(Patch.AmbientColor.GetValue())) return false;
	if (Patch.SkyTop.IsSet() && !ValidColor(Patch.SkyTop.GetValue())) return false;
	if (Patch.SkyBottom.IsSet() && !ValidColor(Patch.SkyBottom.GetValue())) return false;
	if (Patch.HeadlightColor.IsSet() && !ValidColor(Patch.HeadlightColor.GetValue())) return false;
	if (Patch.AmbientIntensity.IsSet() && (!FMath::IsFinite(Patch.AmbientIntensity.GetValue())
		|| Patch.AmbientIntensity.GetValue() < 0.0f)) return false;
	if (Patch.HeadlightIntensity.IsSet() && (!FMath::IsFinite(Patch.HeadlightIntensity.GetValue())
		|| Patch.HeadlightIntensity.GetValue() < 0.0f)) return false;
	return Patch.AmbientColor.IsSet() || Patch.SkyTop.IsSet() || Patch.SkyBottom.IsSet()
		|| Patch.HeadlightColor.IsSet() || Patch.AmbientIntensity.IsSet()
		|| Patch.HeadlightIntensity.IsSet() || Patch.bSkyboxEnabled.IsSet()
		|| Patch.bHeadlightEnabled.IsSet();
}

bool IsSupportedFeature(const FString& Feature)
{
	static const TSet<FString> Features = {
		TEXT("collision"), TEXT("sites"), TEXT("com"), TEXT("inertia"),
		TEXT("contact_points"), TEXT("contact_forces"), TEXT("contact_split"),
		TEXT("perturbation"), TEXT("camera_markers"), TEXT("light_markers"),
		TEXT("actuators"), TEXT("tendons"), TEXT("rangefinders"),
		TEXT("constraints"), TEXT("static_bodies"), TEXT("auto_connect"), TEXT("wrenches")};
	return Features.Contains(Feature);
}
}

bool FURLabRenderScenePatch::Apply(FRenderScene& Scene, FRenderSceneResources& Resources,
	int32 VisibleGroupMask, FString& OutError) const
{
	OutError.Reset();
	if (Materials.IsEmpty() && GeomSurfaces.IsEmpty() && Lights.IsEmpty()
		&& Cameras.IsEmpty() && GeomPoses.IsEmpty() && !Environment.IsSet()
		&& !VisualizationFeatures.IsSet())
	{
		OutError = TEXT("scene patch contains no changes");
		return false;
	}
	FRenderScene Candidate = Scene;
	FRenderSceneResources CandidateResources = Resources;

	TSet<int32> MaterialTargets;
	for (const FURLabRenderMaterialPatch& Patch : Materials)
	{
		if (!ValidMaterialPatch(Patch) || Patch.Index >= CandidateResources.Materials.Num()
			|| MaterialTargets.Contains(Patch.Index))
		{
			OutError = TEXT("material patch has an invalid or duplicate material target/value");
			return false;
		}
		MaterialTargets.Add(Patch.Index);
		FRenderMaterialResource& Value = CandidateResources.Materials[Patch.Index];
		if (Patch.Rgba) Value.Rgba = Patch.Rgba.GetValue();
		if (Patch.Emission) Value.Emission = Patch.Emission.GetValue();
		if (Patch.Specular) Value.Specular = Patch.Specular.GetValue();
		if (Patch.Shininess) Value.Shininess = Patch.Shininess.GetValue();
		if (Patch.Reflectance) Value.Reflectance = Patch.Reflectance.GetValue();
		if (Patch.Metallic) Value.Metallic = Patch.Metallic.GetValue();
		if (Patch.Roughness) Value.Roughness = Patch.Roughness.GetValue();
		if (Patch.TexRepeat) Value.TexRepeat = Patch.TexRepeat.GetValue();
		if (Patch.bTexUniform) Value.bTexUniform = Patch.bTexUniform.GetValue();
	}

	TSet<int32> GeomSurfaceTargets;
	for (const FURLabRenderGeomSurfacePatch& Patch : GeomSurfaces)
	{
		if (Patch.Index < 0 || Patch.Index >= CandidateResources.Geoms.Num()
			|| CandidateResources.Geoms.Num() != Candidate.Geoms.Num()
			|| GeomSurfaceTargets.Contains(Patch.Index) || !HasGeomSurfaceValue(Patch)
			|| (Patch.MaterialIndex.IsSet() && (Patch.MaterialIndex.GetValue() < 0
				|| Patch.MaterialIndex.GetValue() >= CandidateResources.Materials.Num()))
			|| (Patch.Rgba.IsSet() && !ValidColor(Patch.Rgba.GetValue()))
			|| (Patch.TexUniformScale.IsSet() && !ValidVector2(Patch.TexUniformScale.GetValue())))
		{
			OutError = TEXT("geometry surface patch has an invalid or duplicate target/value");
			return false;
		}
		GeomSurfaceTargets.Add(Patch.Index);
		FRenderSurfaceBinding& Value = CandidateResources.Geoms[Patch.Index];
		if (Patch.MaterialIndex) Value.MaterialIndex = Patch.MaterialIndex.GetValue();
		if (Patch.Rgba) Value.Rgba = Patch.Rgba.GetValue();
		if (Patch.TexUniformScale) Value.TexUniformScale = Patch.TexUniformScale.GetValue();
	}

	TSet<FString> LightTargets;
	for (const FURLabRenderLightPatch& Patch : Lights)
	{
		if (Patch.Name.IsEmpty() || LightTargets.Contains(Patch.Name) || !HasLightValue(Patch))
		{
			OutError = TEXT("light patch has an empty/duplicate target or no changes");
			return false;
		}
		FRenderLightDesc* Target = Candidate.Lights.FindByPredicate(
			[&](const FRenderLightDesc& Item) { return Item.Name == Patch.Name; });
		if (!Target)
		{
			OutError = FString::Printf(TEXT("light '%s' does not exist"), *Patch.Name);
			return false;
		}
		LightTargets.Add(Patch.Name);
		if (Patch.bEnabled) Target->bEnabled = Patch.bEnabled.GetValue();
		if (Patch.bCastShadows) Target->bCastShadows = Patch.bCastShadows.GetValue();
		if (Patch.Color) Target->Color = Patch.Color.GetValue();
		if (Patch.Intensity) Target->Intensity = Patch.Intensity.GetValue();
		if (Patch.RangeCm) Target->RangeCm = Patch.RangeCm.GetValue();
		if (Patch.SourceRadiusCm) Target->SourceRadiusCm = Patch.SourceRadiusCm.GetValue();
		if (Patch.InnerConeDegrees) Target->InnerConeDegrees = Patch.InnerConeDegrees.GetValue();
		if (Patch.OuterConeDegrees) Target->OuterConeDegrees = Patch.OuterConeDegrees.GetValue();
	}

	TSet<FString> CameraTargets;
	for (const FURLabRenderCameraPatch& Patch : Cameras)
	{
		if (Patch.Name.IsEmpty() || CameraTargets.Contains(Patch.Name) || !HasCameraValue(Patch))
		{
			OutError = TEXT("camera patch has an empty/duplicate target or no changes");
			return false;
		}
		FRenderCameraDesc* Target = Patch.Name == TEXT("user") && Candidate.UserCamera.IsSet()
			? &Candidate.UserCamera.GetValue()
			: Candidate.Cameras.FindByPredicate([&](const FRenderCameraDesc& Item)
				{ return Item.CanonicalName == Patch.Name; });
		if (!Target)
		{
			OutError = FString::Printf(TEXT("camera '%s' does not exist"), *Patch.Name);
			return false;
		}
		CameraTargets.Add(Patch.Name);
		if (Patch.Mode) Target->Capture.Mode = Patch.Mode.GetValue();
		if (Patch.Optics) Target->Optics = Patch.Optics.GetValue();
		if (Patch.DepthNearCm) Target->Capture.DepthNearCm = Patch.DepthNearCm.GetValue();
		if (Patch.DepthFarCm) Target->Capture.DepthFarCm = Patch.DepthFarCm.GetValue();
	}

	TSet<int32> GeomPoseTargets;
	for (const FURLabRenderGeomPosePatch& Patch : GeomPoses)
	{
		if (Patch.Index < 0 || Patch.Index >= Candidate.Geoms.Num()
			|| GeomPoseTargets.Contains(Patch.Index) || !Patch.BodyLocalTransform.IsValid()
			|| !Patch.BodyLocalTransform.GetScale3D().Equals(FVector::OneVector, 1.e-4f))
		{
			OutError = TEXT("geometry pose patch has an invalid or duplicate target/rigid transform");
			return false;
		}
		GeomPoseTargets.Add(Patch.Index);
		Candidate.Geoms[Patch.Index].BodyLocalTransform = Patch.BodyLocalTransform;
	}

	if (Environment.IsSet())
	{
		const FURLabRenderEnvironmentPatch& Patch = Environment.GetValue();
		if (!ValidEnvironmentPatch(Patch))
		{
			OutError = TEXT("environment patch has invalid values or no changes");
			return false;
		}
		FRenderEnvironmentDesc& Value = Candidate.Environment;
		if (Patch.AmbientColor) Value.AmbientColor = Patch.AmbientColor.GetValue();
		if (Patch.AmbientIntensity) Value.AmbientIntensity = Patch.AmbientIntensity.GetValue();
		if (Patch.SkyTop) Value.SkyTop = Patch.SkyTop.GetValue();
		if (Patch.SkyBottom) Value.SkyBottom = Patch.SkyBottom.GetValue();
		if (Patch.bSkyboxEnabled) Value.bSkyboxEnabled = Patch.bSkyboxEnabled.GetValue();
		if (Patch.bHeadlightEnabled) Value.bHeadlightEnabled = Patch.bHeadlightEnabled.GetValue();
		if (Patch.HeadlightColor) Value.HeadlightColor = Patch.HeadlightColor.GetValue();
		if (Patch.HeadlightIntensity) Value.HeadlightIntensity = Patch.HeadlightIntensity.GetValue();
	}

	if (VisualizationFeatures.IsSet())
	{
		TSet<FString> Seen;
		for (const FString& Feature : VisualizationFeatures.GetValue())
			if (!IsSupportedFeature(Feature) || Seen.Contains(Feature))
			{
				OutError = TEXT("visualization feature list contains an unsupported or duplicate feature");
				return false;
			}
			else Seen.Add(Feature);
		Candidate.Visualization.Features = VisualizationFeatures.GetValue();
	}

	FString Validation;
	if (!CandidateResources.IsValid(Validation)
		|| !Candidate.IsValid(CandidateResources, VisibleGroupMask, Validation))
	{
		OutError = FString::Printf(TEXT("scene patch result is invalid: %s"), *Validation);
		return false;
	}
	Scene = MoveTemp(Candidate);
	Resources = MoveTemp(CandidateResources);
	return true;
}
