// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#include "MuJoCo/Fast/MjSkyImporter.h"

#include "MuJoCo/Utils/URLabAxisConv.h"
#include "Utils/URLabLogging.h"

#include "Engine/DirectionalLight.h"
#include "Engine/SpotLight.h"
#include "Engine/PointLight.h"
#include "Engine/Light.h"
#include "Engine/SkyLight.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/SpotLightComponent.h"
#include "Components/PointLightComponent.h"
#include "Components/SkyLightComponent.h"
#include "Engine/TextureCube.h"
#include "Engine/World.h"
#include "Engine/StaticMesh.h"
#include "Engine/StaticMeshActor.h"
#include "Components/StaticMeshComponent.h"
#include "EngineUtils.h"
#include "UObject/Package.h"
#include "UObject/ConstructorHelpers.h"

#if WITH_EDITOR
#include "MaterialEditingLibrary.h"
#include "Materials/Material.h"
#include "Materials/MaterialExpressionCameraVectorWS.h"
#include "Materials/MaterialExpressionComponentMask.h"
#include "Materials/MaterialExpressionMultiply.h"
#include "Materials/MaterialExpressionAdd.h"
#include "Materials/MaterialExpressionClamp.h"
#include "Materials/MaterialExpressionConstant3Vector.h"
#include "Materials/MaterialExpressionLinearInterpolate.h"
#endif

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace MjSkyImporter
{
namespace
{
// Tag on every actor this importer spawns, so a model reload can clear the old set.
const FName kImportedTag(TEXT("URLabMjEnv"));

// MuJoCo's lighting is an ad-hoc OpenGL model with 0..1 diffuse; UE lights are
// physical. There is no exact mapping, so ONE conversion scale per light kind turns a
// MuJoCo diffuse magnitude into a UE intensity. These are the only tunables, and they
// are unit conversions -- not per-scene guesses. (Tune against a MuJoCo reference.)
constexpr float kDirLux = 6.0f;          // directional: lux at diffuse 1.0
constexpr float kSpotCandela = 42.0f;    // spot: candela at diffuse 1.0 (tuned vs MuJoCo, lights ~2.5m away)
constexpr float kPointCandela = 42.0f;   // point: candela at diffuse 1.0
constexpr float kLightRadiusCm = 6000.0f;

// Live brightness trim for imported lights: set URLAB_LIGHT_SCALE=0.5 (etc.) to scale
// every imported light's intensity without a rebuild, for tuning against MuJoCo.
float ImportedLightScale()
{
	const FString S = FPlatformMisc::GetEnvironmentVariable(TEXT("URLAB_LIGHT_SCALE"));
	if (S.IsEmpty()) { return 1.0f; }
	const float V = FCString::Atof(*S);
	return V > 0.0f ? V : 1.0f;
}

float Luminance(const float* Rgb)
{
	// Perceptual-ish brightness of the MuJoCo diffuse colour.
	return 0.2126f * Rgb[0] + 0.7152f * Rgb[1] + 0.0722f * Rgb[2];
}

// Build a UTextureCube (BGRA8, six slices) from a mjtexSKYBOX texture: six cube faces
// stacked vertically in tex_data (height == 6 * width), RGB(A).
UTextureCube* BuildCubeFromSkybox(const mjModel* Model, int32 TexId)
{
#if WITH_EDITOR
	const int32 W = Model->tex_width[TexId];
	const int32 H = Model->tex_height[TexId];
	const int32 NCh = Model->tex_nchannel[TexId];
	if (W <= 0 || H != 6 * W || (NCh != 3 && NCh != 4))
	{
		UE_LOG(LogURLab, Warning, TEXT("[MjSky] skybox tex %d layout %dx%d nch=%d unsupported"),
			TexId, W, H, NCh);
		return nullptr;
	}
	const uint8* Src = Model->tex_data + Model->tex_adr[TexId];
	TArray<uint8> Bgra;
	Bgra.SetNumUninitialized(6 * W * W * 4);
	for (int32 Px = 0; Px < 6 * W * W; ++Px)
	{
		Bgra[Px * 4 + 0] = Src[Px * NCh + 2];                    // B
		Bgra[Px * 4 + 1] = Src[Px * NCh + 1];                    // G
		Bgra[Px * 4 + 2] = Src[Px * NCh + 0];                    // R
		Bgra[Px * 4 + 3] = (NCh == 4) ? Src[Px * NCh + 3] : 255; // A
	}
	UTextureCube* Cube = NewObject<UTextureCube>(GetTransientPackage(), NAME_None, RF_Transient);
	Cube->Source.Init(W, W, 6, 1, TSF_BGRA8, Bgra.GetData());
	Cube->SRGB = true;
	Cube->CompressionNone = true;
	Cube->MipGenSettings = TMGS_NoMipmaps;
	Cube->UpdateResource();
	return Cube;
#else
	return nullptr;
#endif
}

// Pull the two gradient endpoint colours out of a MuJoCo skybox texture. MuJoCo bakes the
// gradient into the faces (rgb1 at the top of the sky, rgb2 at the bottom), so the brightest
// and darkest texels ARE the endpoints -- order-independent, so we need not know the cube
// face layout. Returns false if the texture is degenerate.
bool ExtractGradientEndpoints(const mjModel* Model, int32 TexId, FLinearColor& OutTop, FLinearColor& OutBottom)
{
	const int32 W = Model->tex_width[TexId];
	const int32 H = Model->tex_height[TexId];
	const int32 NCh = Model->tex_nchannel[TexId];
	if (W <= 0 || H <= 0 || (NCh != 3 && NCh != 4))
	{
		return false;
	}
	const uint8* Src = Model->tex_data + Model->tex_adr[TexId];
	float MaxL = -1.0f, MinL = 2.0f;
	for (int64 P = 0; P < (int64)W * H; ++P)
	{
		const float R = Src[P * NCh + 0] / 255.0f;
		const float G = Src[P * NCh + 1] / 255.0f;
		const float B = Src[P * NCh + 2] / 255.0f;
		const float L = 0.2126f * R + 0.7152f * G + 0.0722f * B;
		if (L > MaxL) { MaxL = L; OutTop = FLinearColor(R, G, B); }
		if (L < MinL) { MinL = L; OutBottom = FLinearColor(R, G, B); }
	}
	return MaxL >= 0.0f;
}

// Draw the skybox as a VISIBLE background: a large inverted sphere with an unlit material
// that reproduces MuJoCo's vertical gradient procedurally -- lerp(bottom, top, saturate(0.5 -
// 0.5*viewDir.z)) using the outward view ray. Reads as sky at infinity (parallax-free) since it
// keys off direction, not position. The SkyLight handles ambient/reflection separately.
void BuildSkyDome(UWorld& World, const FLinearColor& Top, const FLinearColor& Bottom, const FVector& SceneOrigin)
{
#if WITH_EDITOR
	UMaterial* Mat = NewObject<UMaterial>(GetTransientPackage(), NAME_None, RF_Transient);
	Mat->SetShadingModel(MSM_Unlit);
	Mat->TwoSided = true;

	auto Make = [Mat](UClass* C) { return UMaterialEditingLibrary::CreateMaterialExpression(Mat, C); };
	auto* Cam  = Cast<UMaterialExpressionCameraVectorWS>(Make(UMaterialExpressionCameraVectorWS::StaticClass()));
	auto* MaskZ = Cast<UMaterialExpressionComponentMask>(Make(UMaterialExpressionComponentMask::StaticClass()));
	auto* MulZ = Cast<UMaterialExpressionMultiply>(Make(UMaterialExpressionMultiply::StaticClass()));
	auto* AddH = Cast<UMaterialExpressionAdd>(Make(UMaterialExpressionAdd::StaticClass()));
	auto* ClampT = Cast<UMaterialExpressionClamp>(Make(UMaterialExpressionClamp::StaticClass()));
	auto* CTop = Cast<UMaterialExpressionConstant3Vector>(Make(UMaterialExpressionConstant3Vector::StaticClass()));
	auto* CBot = Cast<UMaterialExpressionConstant3Vector>(Make(UMaterialExpressionConstant3Vector::StaticClass()));
	auto* Lerp = Cast<UMaterialExpressionLinearInterpolate>(Make(UMaterialExpressionLinearInterpolate::StaticClass()));
	if (!Cam || !MaskZ || !MulZ || !AddH || !ClampT || !CTop || !CBot || !Lerp)
	{
		return;
	}
	// t = saturate(0.5 - 0.5 * CameraVectorWS.z); CameraVectorWS points surface->camera, so its
	// -z is the outward view ray's z (up). t=1 looking up (->Top), t=0 looking down (->Bottom).
	MaskZ->R = false; MaskZ->G = false; MaskZ->B = true; MaskZ->A = false;
	MaskZ->Input.Expression = Cam;
	MulZ->A.Expression = MaskZ; MulZ->ConstB = -0.5f;
	AddH->A.Expression = MulZ; AddH->ConstB = 0.5f;
	ClampT->Input.Expression = AddH; ClampT->MinDefault = 0.0f; ClampT->MaxDefault = 1.0f;
	CTop->Constant = Top;
	CBot->Constant = Bottom;
	Lerp->A.Expression = CBot; Lerp->B.Expression = CTop; Lerp->Alpha.Expression = ClampT;
	Mat->GetEditorOnlyData()->EmissiveColor.Expression = Lerp;
	// NOT UMaterialEditingLibrary::RecompileMaterial -- it rebuilds open material-editor windows
	// via an editor subsystem that does not exist under -game (SIGSEGV). PostEditChange recompiles
	// the shader for rendering without touching editor UI.
	Mat->PostEditChange();

	UStaticMesh* Sphere = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Sphere.Sphere"));
	if (!Sphere)
	{
		UE_LOG(LogURLab, Warning, TEXT("[MjSky] sky dome: /Engine/BasicShapes/Sphere not found"));
		return;
	}
	AStaticMeshActor* Dome = World.SpawnActor<AStaticMeshActor>(AStaticMeshActor::StaticClass());
	if (!Dome)
	{
		return;
	}
	Dome->Tags.Add(kImportedTag);
	UStaticMeshComponent* SM = Dome->GetStaticMeshComponent();
	SM->SetMobility(EComponentMobility::Movable);
	SM->SetStaticMesh(Sphere);
	SM->SetMaterial(0, Mat);
	SM->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	SM->SetCastShadow(false);
	SM->bReceivesDecals = false;
	SM->SetWorldLocation(SceneOrigin);
	SM->SetWorldScale3D(FVector(200.0f)); // BasicShapes/Sphere is ~1m dia -> ~100m radius dome
#endif
}

void ClearImported(UWorld& World)
{
	TArray<AActor*> Old;
	for (TActorIterator<AActor> It(&World); It; ++It)
	{
		if (It->Tags.Contains(kImportedTag))
		{
			Old.Add(*It);
		}
	}
	for (AActor* A : Old)
	{
		A->Destroy();
	}
}
} // namespace

void ApplyMjEnvironment(UWorld& World, const mjModel* Model, const mjData* Data, bool bBaseLevel,
	const FVector& SceneOrigin)
{
#if WITH_EDITOR
	if (Model == nullptr || bBaseLevel)
	{
		return;
	}
	ClearImported(World);
	const float Scale = ImportedLightScale();

	// --- Scene lights: one UE light per active MuJoCo <light>, at its resolved world
	// pose (Data->light_xpos/xdir account for targetbody tracking). ---
	int32 NLit = 0;
	for (int32 i = 0; i < Model->nlight; ++i)
	{
		if (Model->light_active[i] == 0)
		{
			continue;
		}
		const double* Pos = (Data ? Data->light_xpos : Model->light_pos) + 3 * i;
		const double* Dir = (Data ? Data->light_xdir : Model->light_dir) + 3 * i;
		const FVector UePos = URLabAxisConv::MjPositionToUe(Pos) + SceneOrigin;
		const FVector UeDir = URLabAxisConv::MjDirectionToUe(Dir).GetSafeNormal();
		const FRotator Rot = UeDir.IsNearlyZero() ? FRotator::ZeroRotator : UeDir.Rotation();
		const float* Dif = Model->light_diffuse + 3 * i;
		const FLinearColor Color(Dif[0], Dif[1], Dif[2]);
		const float Lum = FMath::Max(Luminance(Dif), 1e-3f);
		const int32 Type = Model->light_type[i];
		const bool bShadow = Model->light_castshadow[i] != 0;

		AActor* Actor = nullptr;
		ULightComponent* LC = nullptr;
		if (Type == mjLIGHT_DIRECTIONAL)
		{
			ADirectionalLight* D = World.SpawnActor<ADirectionalLight>(
				ADirectionalLight::StaticClass(), FTransform(Rot));
			if (D)
			{
				Actor = D;
				LC = D->GetLightComponent();
				LC->SetIntensity(kDirLux * Lum * Scale);
			}
		}
		else if (Type == mjLIGHT_SPOT)
		{
			ASpotLight* S = World.SpawnActor<ASpotLight>(
				ASpotLight::StaticClass(), FTransform(Rot, UePos));
			if (S)
			{
				Actor = S;
				USpotLightComponent* SC = Cast<USpotLightComponent>(S->GetLightComponent());
				SC->SetMobility(EComponentMobility::Movable);
				SC->SetOuterConeAngle(FMath::Clamp((float)Model->light_cutoff[i], 1.0f, 80.0f));
				SC->SetInnerConeAngle(0.0f);
				SC->SetIntensityUnits(ELightUnits::Candelas);
				SC->SetIntensity(kSpotCandela * Lum * Scale);
				SC->SetAttenuationRadius(kLightRadiusCm);
				LC = SC;
			}
		}
		else // point (and image lights fall back to point)
		{
			APointLight* P = World.SpawnActor<APointLight>(
				APointLight::StaticClass(), FTransform(Rot, UePos));
			if (P)
			{
				Actor = P;
				UPointLightComponent* PC = Cast<UPointLightComponent>(P->GetLightComponent());
				PC->SetMobility(EComponentMobility::Movable);
				PC->SetIntensityUnits(ELightUnits::Candelas);
				PC->SetIntensity(kPointCandela * Lum * Scale);
				PC->SetAttenuationRadius(kLightRadiusCm);
				LC = PC;
			}
		}
		if (LC)
		{
			LC->SetMobility(EComponentMobility::Movable);
			LC->SetLightColor(Color);
			LC->SetCastShadows(bShadow);
		}
		if (Actor)
		{
			// The spawn transform's ROTATION does not reliably stick on a light actor
			// whose component starts Static; re-apply it now that the component is Movable
			// so spot/directional beams actually face their MuJoCo direction.
			Actor->SetActorLocationAndRotation(UePos, Rot);
			Actor->Tags.Add(kImportedTag);
			++NLit;
		}
	}

	// --- SkyLight: the model's skybox as the ambient/reflection source, at an intensity
	// taken from the headlight (MuJoCo's camera light is mostly an even fill): ambient +
	// half the diffuse approximates that omnidirectional fill without a magic constant. ---
	int32 SkyTex = -1;
	for (int32 T = 0; T < Model->ntex; ++T)
	{
		if (Model->tex_type[T] == mjTEXTURE_SKYBOX)
		{
			SkyTex = T;
			break;
		}
	}
	const auto& HL = Model->vis.headlight;
	const float AmbLum = 0.2126f * HL.ambient[0] + 0.7152f * HL.ambient[1] + 0.0722f * HL.ambient[2];
	const float DifLum = 0.2126f * HL.diffuse[0] + 0.7152f * HL.diffuse[1] + 0.0722f * HL.diffuse[2];
	const float SkyIntensity = HL.active ? (AmbLum + 0.5f * DifLum) : AmbLum;

	// Build the skybox cubemap once; it feeds both the SkyLight (ambient) and the sky dome.
	UTextureCube* SkyCube = (SkyTex >= 0) ? BuildCubeFromSkybox(Model, SkyTex) : nullptr;

	ASkyLight* Sky = nullptr;
	for (TActorIterator<ASkyLight> It(&World); It; ++It) { Sky = *It; break; }
	if (!Sky) { Sky = World.SpawnActor<ASkyLight>(ASkyLight::StaticClass()); }
	if (Sky)
	{
		if (USkyLightComponent* C = Sky->GetLightComponent())
		{
			C->SetMobility(EComponentMobility::Movable);
			C->bLowerHemisphereIsBlack = false;
			if (SkyCube)
			{
				C->SourceType = SLS_SpecifiedCubemap;
				C->Cubemap = SkyCube;
			}
			C->SetIntensity(FMath::Max(SkyIntensity, 0.0f));
			C->MarkRenderStateDirty();
			C->RecaptureSky();
		}
	}

	// Visible background: draw the skybox gradient on a sky dome around the scene.
	if (SkyTex >= 0)
	{
		FLinearColor Top(0, 0, 0), Bottom(0, 0, 0);
		if (ExtractGradientEndpoints(Model, SkyTex, Top, Bottom))
		{
			BuildSkyDome(World, Top, Bottom, SceneOrigin);
		}
	}

	UE_LOG(LogURLab, Log,
		TEXT("[MjSky] imported %d light(s), headlight fill=%.2f, skybox=%s"),
		NLit, SkyIntensity, SkyTex >= 0 ? TEXT("yes") : TEXT("none"));
#endif
}
} // namespace MjSkyImporter
