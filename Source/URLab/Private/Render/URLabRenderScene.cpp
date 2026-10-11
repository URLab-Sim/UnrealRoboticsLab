// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "Render/URLabRenderScene.h"

#include "Render/RenderAssetBuilder.h"
#include "Render/RenderPrimitiveBuilder.h"
#include "Render/RenderComponentUtils.h"
#include "Render/RenderCameraComponent.h"
#include "Render/RenderCameraRig.h"
#include "Render/RenderMaterial.h"

#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/PointLightComponent.h"
#include "Components/SpotLightComponent.h"
#include "Components/SkyLightComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/Texture2D.h"
#include "Engine/TextureCube.h"
#include "TextureResource.h"
#include "Engine/World.h"
#include "Misc/ScopeExit.h"
#include "Misc/SecureHash.h"
#include "GameFramework/Actor.h"
#include "Materials/MaterialInterface.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "ProceduralMeshComponent.h"
#include "UObject/StrongObjectPtr.h"

namespace
{
TArray<FProcMeshTangent> DeformableTangents(const TArray<FVector>& Positions,
	const TArray<FVector>& Normals, const TArray<int32>& Triangles,
	const TArray<FVector2D>& UVs)
{
	TArray<FProcMeshTangent> Result;
	Result.Init(FProcMeshTangent(), Positions.Num());
	if (UVs.Num() != Positions.Num()) return Result;
	TArray<FVector> TangentSum, BitangentSum;
	TangentSum.Init(FVector::ZeroVector, Positions.Num());
	BitangentSum.Init(FVector::ZeroVector, Positions.Num());
	for (int32 I = 0; I + 2 < Triangles.Num(); I += 3)
	{
		const int32 A = Triangles[I], B = Triangles[I + 1], C = Triangles[I + 2];
		const FVector Edge1 = Positions[B] - Positions[A], Edge2 = Positions[C] - Positions[A];
		const FVector2D D1 = UVs[B] - UVs[A], D2 = UVs[C] - UVs[A];
		const double Determinant = D1.X * D2.Y - D2.X * D1.Y;
		if (FMath::Abs(Determinant) <= SMALL_NUMBER) continue;
		const FVector T = (Edge1 * D2.Y - Edge2 * D1.Y) / Determinant;
		const FVector Bv = (Edge2 * D1.X - Edge1 * D2.X) / Determinant;
		TangentSum[A] += T; TangentSum[B] += T; TangentSum[C] += T;
		BitangentSum[A] += Bv; BitangentSum[B] += Bv; BitangentSum[C] += Bv;
	}
	for (int32 I = 0; I < Positions.Num(); ++I)
	{
		const FVector N = Normals[I].GetSafeNormal();
		FVector T = TangentSum[I] - N * FVector::DotProduct(N, TangentSum[I]);
		if (!T.Normalize())
		{
			T = FVector::CrossProduct(FMath::Abs(N.Z) < 0.9 ? FVector::UpVector : FVector::RightVector, N).GetSafeNormal();
		}
		const bool bFlipY = FVector::DotProduct(FVector::CrossProduct(N, T), BitangentSum[I]) < 0.0;
		Result[I] = FProcMeshTangent(T, bFlipY);
	}
	return Result;
}

FName IndexedTag(const TCHAR* Prefix, int32 Index)
{
	return FName(*FString::Printf(TEXT("%s%d"), Prefix, Index));
}

bool IsRenderable(const FRenderGeomDesc& Geom, const FRenderSurfaceBinding& Surface,
	int32 VisibleGroupMask)
{
	return Geom.Group >= 0 && Geom.Group <= 30
		&& (VisibleGroupMask & (1 << Geom.Group)) != 0 && Surface.Rgba.A > 0.0f;
}

FLinearColor EncodeSegmentationId(int32 ObjectId)
{
	if (ObjectId < 0 || ObjectId >= 0xFFFFFF) return FLinearColor::Black;
	const uint32 Encoded = static_cast<uint32>(ObjectId) + 1u;
	return FLinearColor(static_cast<float>(Encoded & 0xFFu),
		static_cast<float>((Encoded >> 8) & 0xFFu),
		static_cast<float>((Encoded >> 16) & 0xFFu), 1.0f);
}

bool IsRigidGeomPose(const FTransform& Transform)
{
	const FQuat Rotation = Transform.GetRotation();
	return !Transform.GetTranslation().ContainsNaN()
		&& !Transform.GetScale3D().ContainsNaN() && !Rotation.ContainsNaN()
		&& Transform.GetScale3D().Equals(FVector::OneVector, KINDA_SMALL_NUMBER)
		&& FMath::IsNearlyEqual(Rotation.SizeSquared(), 1.0, KINDA_SMALL_NUMBER);
}

void CanonicalizeQuaternion(FQuat& Rotation)
{
	Rotation.Normalize();
	if (Rotation.W < 0.0
		|| (Rotation.W == 0.0 && (Rotation.X < 0.0
			|| (Rotation.X == 0.0 && (Rotation.Y < 0.0
				|| (Rotation.Y == 0.0 && Rotation.Z < 0.0))))))
	{
		Rotation.X = -Rotation.X; Rotation.Y = -Rotation.Y;
		Rotation.Z = -Rotation.Z; Rotation.W = -Rotation.W;
	}
}

FString GeometryConfigurationId(const FString& Revision,
	const TArray<FRenderGeomPoseOverride>& Overrides)
{
	FSHA1 Hash;
	auto Append = [&Hash](const FString& Value)
	{
		FTCHARToUTF8 Utf8(*Value);
		Hash.Update(reinterpret_cast<const uint8*>(Utf8.Get()), Utf8.Length());
	};
	Append(Revision);
	for (const FRenderGeomPoseOverride& Override : Overrides)
	{
		const FVector P = Override.BodyLocalTransform.GetTranslation();
		const FQuat Q = Override.BodyLocalTransform.GetRotation();
		Append(FString::Printf(TEXT("|%d|%.17g|%.17g|%.17g|%.17g|%.17g|%.17g|%.17g"),
			Override.GeomIndex, P.X, P.Y, P.Z, Q.X, Q.Y, Q.Z, Q.W));
	}
	Hash.Final();
	uint8 Digest[FSHA1::DigestSize]; Hash.GetHash(Digest);
	return BytesToHex(Digest, UE_ARRAY_COUNT(Digest)).ToLower();
}
}

AURLabRenderScene::AURLabRenderScene()
{
	PrimaryActorTick.bCanEverTick = false;
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("RenderSceneRoot"));
}

void AURLabRenderScene::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	TeardownScene();
	Super::EndPlay(EndPlayReason);
}

void AURLabRenderScene::Destroyed()
{
	TeardownScene();
	Super::Destroyed();
}

bool AURLabRenderScene::InstallScene(FRenderSceneInstallPayload&& Payload,
	const FRenderSceneInstallOptions& Options, FString& OutError)
{
	FRenderSceneInstallPayload OwnedPayload = MoveTemp(Payload);
	FRenderScene& CandidateScene = OwnedPayload.Scene;
	FRenderSceneResources& CandidateResources = OwnedPayload.Resources;
	FRenderFrame& InitialFrame = OwnedPayload.InitialFrame;
	OutError.Reset();
	if (!CandidateScene.IsValid(CandidateResources, Options.VisibleGroupMask, OutError))
	{
		return false;
	}
	if (!InitialFrame.IsValid(CandidateScene, OutError))
	{
		return false;
	}
	const FQuat SceneRotation = Options.SceneTransform.GetRotation();
	if (Options.SceneTransform.GetTranslation().ContainsNaN()
		|| Options.SceneTransform.GetScale3D().ContainsNaN() || SceneRotation.ContainsNaN()
		|| !Options.SceneTransform.GetScale3D().Equals(FVector::OneVector, KINDA_SMALL_NUMBER)
		|| !FMath::IsNearlyEqual(SceneRotation.SizeSquared(), 1.0, KINDA_SMALL_NUMBER))
	{
		OutError = TEXT("scene transform must be finite, rigid and unit scale");
		return false;
	}

	TStrongObjectPtr<URenderAssetBuilder> CandidateBuilder(
		NewObject<URenderAssetBuilder>(this));
	bool bBuilderCommitted = false;
	ON_SCOPE_EXIT
	{
		if (!bBuilderCommitted && CandidateBuilder.IsValid()) CandidateBuilder->Reset();
	};
	if (!CandidateBuilder.IsValid()
		|| !CandidateBuilder->InitResources(MoveTemp(CandidateResources),
			Options.bForceRebuildAssets, OutError))
	{
		return false;
	}
	FCandidateTree CandidateTree;
	if (!BuildCandidate(CandidateScene, CandidateBuilder->GetResources(), *CandidateBuilder, Options,
		CandidateTree, OutError))
	{
		DestroyTree(CandidateTree);
		return false;
	}
	if (!BuildDynamicMeshes(CandidateScene, InitialFrame, *CandidateBuilder,
		CandidateTree, OutError))
	{
		DestroyTree(CandidateTree);
		return false;
	}
	if (!BuildEnvironment(CandidateScene, CandidateBuilder->GetResources(),
		CandidateTree, OutError))
	{
		DestroyTree(CandidateTree);
		return false;
	}
	TArray<UMaterialInterface*> CandidateMaterials;
	for (UPrimitiveComponent* Component : CandidateTree.GeomComps)
	{
		if (Component)
		{
			Component->GetUsedMaterials(CandidateMaterials);
		}
	}
	for (UPrimitiveComponent* Component : CandidateTree.InstanceSegComps)
		if (Component) Component->GetUsedMaterials(CandidateMaterials);
	for (UPrimitiveComponent* Component : CandidateTree.SemanticSegComps)
		if (Component) Component->GetUsedMaterials(CandidateMaterials);
	for (UPrimitiveComponent* Component : CandidateTree.Deformables)
		if (Component) Component->GetUsedMaterials(CandidateMaterials);
	for (UPrimitiveComponent* Component : CandidateTree.DebugMeshes)
		if (Component) Component->GetUsedMaterials(CandidateMaterials);
	if (CandidateTree.SkyDome)
		CandidateTree.SkyDome->GetUsedMaterials(CandidateMaterials);
	if (!EnsureRenderMaterialsReadyForWorld(*GetWorld(), CandidateMaterials, OutError))
	{
		DestroyTree(CandidateTree);
		return false;
	}
	ApplyFrameToBodies(InitialFrame, CandidateTree.Bodies);
	ApplyFrameToLights(InitialFrame, CandidateTree.Lights);
	CandidateTree.CameraRig = NewObject<URenderCameraRig>(this);
	if (!CandidateTree.CameraRig || !CandidateTree.CameraRig->Build(GetRootComponent(),
		CandidateScene, InitialFrame, /*bInitiallyVisible=*/false, OutError))
	{
		DestroyTree(CandidateTree);
		return false;
	}
	TArray<UPrimitiveComponent*> InstanceSubjects;
	TArray<UPrimitiveComponent*> SemanticSubjects;
	for (UPrimitiveComponent* Component : CandidateTree.InstanceSegComps)
		if (Component) InstanceSubjects.Add(Component);
	for (UPrimitiveComponent* Component : CandidateTree.SemanticSegComps)
		if (Component) SemanticSubjects.Add(Component);
	TArray<URenderCameraComponent*> CandidateCameras;
	CandidateTree.CameraRig->GetAllCameras(CandidateCameras);
	for (URenderCameraComponent* Camera : CandidateCameras)
		Camera->SetNeutralSegmentationComponents(InstanceSubjects, SemanticSubjects);

	FCandidateTree OldTree;
	OldTree.Bodies = MoveTemp(BodyActors);
	OldTree.GeomActors = MoveTemp(GeomActors);
	OldTree.GeomComps = MoveTemp(GeomComps);
	OldTree.InstanceSegComps = MoveTemp(InstanceSegComps);
	OldTree.SemanticSegComps = MoveTemp(SemanticSegComps);
	OldTree.Lights = MoveTemp(LightComponents);
	OldTree.SkyLight = SkyLightComponent;
	OldTree.SkyDome = SkyDomeComponent;
	OldTree.Deformables = MoveTemp(DeformableComponents);
	OldTree.DebugMeshes = MoveTemp(DebugMeshComponents);
	OldTree.CameraRig = CameraRig;
	URenderAssetBuilder* OldBuilder = AssetBuilder;
	FRenderSceneResourceLifetimeLeasePtr RetiredLease = MoveTemp(ResourceLifetimeLease);
	SetTreeHidden(OldTree, true);

	SetActorTransform(Options.SceneTransform);
	AssetBuilder = CandidateBuilder.Get();
	Scene = MoveTemp(OwnedPayload.Scene);
	AppliedFrame = MoveTemp(OwnedPayload.InitialFrame);
	ResourceLifetimeLease = MoveTemp(OwnedPayload.LifetimeLease);
	InstalledOptions = Options;
	BodyActors = MoveTemp(CandidateTree.Bodies);
	GeomActors = MoveTemp(CandidateTree.GeomActors);
	GeomComps = MoveTemp(CandidateTree.GeomComps);
	InstanceSegComps = MoveTemp(CandidateTree.InstanceSegComps);
	SemanticSegComps = MoveTemp(CandidateTree.SemanticSegComps);
	LightComponents = MoveTemp(CandidateTree.Lights);
	SkyLightComponent = CandidateTree.SkyLight;
	SkyDomeComponent = CandidateTree.SkyDome;
	DeformableComponents = MoveTemp(CandidateTree.Deformables);
	DebugMeshComponents = MoveTemp(CandidateTree.DebugMeshes);
	CameraRig = CandidateTree.CameraRig;
	SetSceneVisible(Options.bInitiallyVisible);
	bBuilderCommitted = true;
	DestroyTree(OldTree);
	if (OldBuilder) OldBuilder->Reset();
	RetiredLease.Reset();
	return true;
}

bool AURLabRenderScene::ApplyFrame(const FRenderFrame& Candidate, FString& OutError)
{
	if (!Candidate.IsValid(Scene, OutError))
	{
		return false;
	}
	ApplyFrameToBodies(Candidate, BodyActors);
	ApplyFrameToLights(Candidate, LightComponents);
	if (!ApplyDynamicMeshes(Scene, Candidate, DeformableComponents,
		DebugMeshComponents, OutError)) return false;
	if (CameraRig && !CameraRig->ApplyFrame(Candidate, OutError)) return false;
	AppliedFrame = Candidate;
	return true;
}

bool AURLabRenderScene::CopyInstalledPayload(FRenderSceneInstallPayload& OutPayload,
	FRenderSceneInstallOptions& OutOptions, FString& OutError) const
{
	OutError.Reset();
	if (!AssetBuilder || Scene.SceneRevision.IsEmpty()
		|| AssetBuilder->GetResources().SceneRevision != Scene.SceneRevision)
	{
		OutError = TEXT("there is no complete installed neutral scene to copy");
		return false;
	}
	FRenderScene SceneCopy = Scene;
	FRenderSceneResources ResourcesCopy = AssetBuilder->GetResources();
	FRenderFrame FrameCopy = AppliedFrame;
	OutPayload = FRenderSceneInstallPayload(MoveTemp(SceneCopy), MoveTemp(ResourcesCopy),
		MoveTemp(FrameCopy), ResourceLifetimeLease);
	OutOptions = InstalledOptions;
	OutOptions.bInitiallyVisible = !IsHidden();
	return true;
}

bool AURLabRenderScene::PlanGeomPoseOverrides(
	const TArray<FRenderGeomPoseOverride>& Overrides,
	TArray<FRenderGeomPoseOverride>& OutCanonical, FString& OutConfigurationId,
	FString& OutError) const
{
	OutCanonical.Reset(); OutConfigurationId.Reset(); OutError.Reset();
	if (Overrides.Num() > Scene.Geoms.Num())
	{
		OutError = TEXT("geometry override count exceeds the installed geometry count");
		return false;
	}
	OutCanonical = Overrides;
	OutCanonical.Sort([](const FRenderGeomPoseOverride& A, const FRenderGeomPoseOverride& B)
	{
		return A.GeomIndex < B.GeomIndex;
	});
	for (int32 Index = 0; Index < OutCanonical.Num(); ++Index)
	{
		FRenderGeomPoseOverride& Override = OutCanonical[Index];
		if (!Scene.Geoms.IsValidIndex(Override.GeomIndex))
		{
			OutError = FString::Printf(TEXT("geometry override index %d is out of range"),
				Override.GeomIndex);
			return false;
		}
		if (Index > 0 && OutCanonical[Index - 1].GeomIndex == Override.GeomIndex)
		{
			OutError = FString::Printf(TEXT("geometry override index %d is duplicated"),
				Override.GeomIndex);
			return false;
		}
		if (!IsRigidGeomPose(Override.BodyLocalTransform))
		{
			OutError = FString::Printf(TEXT("geometry override index %d is not a finite rigid pose"),
				Override.GeomIndex);
			return false;
		}
		FQuat Rotation = Override.BodyLocalTransform.GetRotation();
		CanonicalizeQuaternion(Rotation);
		Override.BodyLocalTransform.SetRotation(Rotation);
		Override.BodyLocalTransform.SetScale3D(FVector::OneVector);
	}
	OutConfigurationId = GeometryConfigurationId(Scene.SceneRevision, OutCanonical);
	return true;
}

bool AURLabRenderScene::BeginGeomPoseOverrides(
	const TArray<FRenderGeomPoseOverride>& CanonicalOverrides,
	FRenderGeomPoseOverrideToken& OutToken, FString& OutError)
{
	OutToken = {}; OutError.Reset();
	if (ActiveGeomOverrideSerial != 0)
	{
		OutError = TEXT("geometry pose overrides are already active"); return false;
	}
	TArray<FRenderGeomPoseOverride> Verified;
	FString IgnoredId;
	if (!PlanGeomPoseOverrides(CanonicalOverrides, Verified, IgnoredId, OutError)) return false;
	if (NextGeomOverrideSerial == 0 || NextGeomOverrideSerial == MAX_uint64)
	{
		OutError = TEXT("geometry override token space is exhausted"); return false;
	}
	for (const FRenderGeomPoseOverride& Override : Verified)
	{
		AActor* Actor = GeomActors.IsValidIndex(Override.GeomIndex)
			? GeomActors[Override.GeomIndex].Get() : nullptr;
		const FRenderGeomDesc& Desc = Scene.Geoms[Override.GeomIndex];
		const bool bExpectedBuilt = AssetBuilder
			&& AssetBuilder->GetResources().Geoms.IsValidIndex(Override.GeomIndex)
			&& IsRenderable(Desc, AssetBuilder->GetResources().Geoms[Override.GeomIndex],
				InstalledOptions.VisibleGroupMask);
		if (bExpectedBuilt && !Actor)
		{
			OutError = FString::Printf(TEXT("visible geometry %d has no built actor"),
				Override.GeomIndex); return false;
		}
		if (Actor)
		{
			FRenderGeomPoseOverride& Baseline = OutToken.Baseline.AddDefaulted_GetRef();
			Baseline.GeomIndex = Override.GeomIndex;
			Baseline.BodyLocalTransform = Actor->GetRootComponent()->GetRelativeTransform();
		}
	}
	for (const FRenderGeomPoseOverride& Override : Verified)
	{
		if (AActor* Actor = GeomActors[Override.GeomIndex].Get())
			Actor->SetActorRelativeTransform(Override.BodyLocalTransform);
	}
	OutToken.SceneRevision = Scene.SceneRevision;
	OutToken.Serial = NextGeomOverrideSerial++;
	ActiveGeomOverrideSerial = OutToken.Serial;
	return true;
}

bool AURLabRenderScene::RestoreGeomPoseOverrides(
	const FRenderGeomPoseOverrideToken& Token, FString& OutError)
{
	OutError.Reset();
	if (!Token.IsValid() || Token.SceneRevision != Scene.SceneRevision
		|| Token.Serial != ActiveGeomOverrideSerial)
	{
		OutError = TEXT("geometry override token is stale or belongs to another scene");
		return false;
	}
	for (const FRenderGeomPoseOverride& Baseline : Token.Baseline)
	{
		AActor* Actor = GeomActors.IsValidIndex(Baseline.GeomIndex)
			? GeomActors[Baseline.GeomIndex].Get() : nullptr;
		if (!Actor)
		{
			OutError = FString::Printf(TEXT("geometry %d disappeared before override restoration"),
				Baseline.GeomIndex); return false;
		}
	}
	for (const FRenderGeomPoseOverride& Baseline : Token.Baseline)
		GeomActors[Baseline.GeomIndex]->SetActorRelativeTransform(Baseline.BodyLocalTransform);
	ActiveGeomOverrideSerial = 0;
	return true;
}

bool AURLabRenderScene::BuildCandidate(const FRenderScene& CandidateScene,
	const FRenderSceneResources& CandidateResources, URenderAssetBuilder& CandidateBuilder,
	const FRenderSceneInstallOptions& Options, FCandidateTree& OutTree, FString& OutError)
{
	UWorld* World = GetWorld();
	if (!World)
	{
		OutError = TEXT("render scene has no world");
		return false;
	}
	OutTree.Bodies.SetNum(CandidateScene.Bodies.Num());
	OutTree.GeomActors.SetNum(CandidateScene.Geoms.Num());
	OutTree.GeomComps.SetNum(CandidateScene.Geoms.Num());
	OutTree.InstanceSegComps.SetNum(CandidateScene.Geoms.Num());
	OutTree.SemanticSegComps.SetNum(CandidateScene.Geoms.Num());
	OutTree.Lights.SetNum(CandidateScene.Lights.Num());
	UMaterialInterface* SegmentationMaterial = LoadObject<UMaterialInterface>(nullptr,
		TEXT("/UnrealRoboticsLab/Materials/M_MjSegId.M_MjSegId"));
	if (!SegmentationMaterial)
	{
		OutError = TEXT("neutral segmentation id material is unavailable");
		return false;
	}
	CandidateBuilder.RetainResolvedObject(SegmentationMaterial);
	for (int32 BodyIndex = 0; BodyIndex < CandidateScene.Bodies.Num(); ++BodyIndex)
	{
		FActorSpawnParameters Params;
		Params.Owner = this;
		if (!Options.bPersistentEditorActors) Params.ObjectFlags |= RF_Transient;
		AActor* Body = World->SpawnActor<AActor>(AActor::StaticClass(), FTransform::Identity, Params);
		if (!Body)
		{
			OutError = FString::Printf(TEXT("failed to spawn body actor %d"), BodyIndex);
			return false;
		}
		OutTree.Bodies[BodyIndex] = Body;
		USceneComponent* BodyRoot = NewObject<USceneComponent>(Body,
			*FString::Printf(TEXT("RenderBody_%d"), BodyIndex));
		if (!BodyRoot)
		{
			OutError = FString::Printf(TEXT("failed to create body root %d"), BodyIndex);
			return false;
		}
		Body->SetRootComponent(BodyRoot);
		BodyRoot->RegisterComponent();
		Body->AttachToActor(this, FAttachmentTransformRules::KeepRelativeTransform);
		Body->Tags.Add(IndexedTag(TEXT("URLabRenderBody="), BodyIndex));
		Body->SetActorHiddenInGame(true);
#if WITH_EDITOR
		if (Options.bPersistentEditorActors && !CandidateScene.Bodies[BodyIndex].Name.IsEmpty())
		{
			Body->SetActorLabel(CandidateScene.Bodies[BodyIndex].Name);
		}
#endif
	}

	for (int32 GeomIndex = 0; GeomIndex < CandidateScene.Geoms.Num(); ++GeomIndex)
	{
		if (!IsRenderable(CandidateScene.Geoms[GeomIndex], CandidateResources.Geoms[GeomIndex],
				Options.VisibleGroupMask))
		{
			continue;
		}
		const FRenderGeomDesc& Geom = CandidateScene.Geoms[GeomIndex];
		AActor* Body = OutTree.Bodies[Geom.BodyIndex];
		FActorSpawnParameters Params;
		Params.Owner = this;
		if (!Options.bPersistentEditorActors) Params.ObjectFlags |= RF_Transient;
		AActor* GeomActor = World->SpawnActor<AActor>(AActor::StaticClass(), FTransform::Identity, Params);
		if (!GeomActor)
		{
			OutError = FString::Printf(TEXT("failed to spawn geom actor %d"), GeomIndex);
			return false;
		}
		OutTree.GeomActors[GeomIndex] = GeomActor;
		USceneComponent* GeomRoot = NewObject<USceneComponent>(GeomActor, TEXT("RenderGeomRoot"));
		if (!GeomRoot)
		{
			OutError = FString::Printf(TEXT("failed to create geom root %d"), GeomIndex);
			return false;
		}
		GeomActor->SetRootComponent(GeomRoot);
		GeomRoot->RegisterComponent();
		GeomActor->AttachToActor(Body, FAttachmentTransformRules::KeepRelativeTransform);
		GeomActor->SetActorRelativeTransform(Geom.BodyLocalTransform);
		GeomActor->Tags.Add(IndexedTag(TEXT("URLabRenderGeom="), GeomIndex));
		GeomActor->SetActorHiddenInGame(true);
#if WITH_EDITOR
		if (Options.bPersistentEditorActors)
		{
			GeomActor->SetActorLabel(!Geom.Name.IsEmpty() ? Geom.Name
				: FString::Printf(TEXT("geom_%d"), GeomIndex));
		}
#endif
		UPrimitiveComponent* Component = BuildGeom(GeomIndex, CandidateScene,
			CandidateBuilder, *GeomActor);
		if (!Component)
		{
			OutError = FString::Printf(TEXT("failed to build visible geom %d"), GeomIndex);
			return false;
		}
		Component->ComponentTags.Add(IndexedTag(TEXT("URLabRenderGeom="), GeomIndex));
		OutTree.GeomComps[GeomIndex] = Component;
		UPrimitiveComponent* Instance = BuildSegmentationGeom(GeomIndex, GeomIndex,
			CandidateScene, CandidateBuilder, *GeomActor, *SegmentationMaterial);
		UPrimitiveComponent* Semantic = BuildSegmentationGeom(GeomIndex, Geom.BodyIndex,
			CandidateScene, CandidateBuilder, *GeomActor, *SegmentationMaterial);
		if (!Instance || !Semantic)
		{
			OutError = FString::Printf(TEXT("failed to build segmentation geom %d"), GeomIndex);
			return false;
		}
		Instance->ComponentTags.Add(FName(TEXT("URLab_Seg_Instance")));
		Semantic->ComponentTags.Add(FName(TEXT("URLab_Seg_Semantic")));
		OutTree.InstanceSegComps[GeomIndex] = Instance;
		OutTree.SemanticSegComps[GeomIndex] = Semantic;
	}
	for (int32 LightIndex = 0; LightIndex < CandidateScene.Lights.Num(); ++LightIndex)
	{
		const FRenderLightDesc& Desc = CandidateScene.Lights[LightIndex];
		ULightComponent* Light = nullptr;
		switch (Desc.Kind)
		{
		case ERenderLightKind::Directional:
			Light = NewObject<UDirectionalLightComponent>(this); break;
		case ERenderLightKind::Point:
			Light = NewObject<UPointLightComponent>(this); break;
		case ERenderLightKind::Spot:
			Light = NewObject<USpotLightComponent>(this); break;
		}
		if (!Light)
		{
			OutError = FString::Printf(TEXT("failed to create light %d"), LightIndex);
			return false;
		}
		Light->SetupAttachment(GetRootComponent());
		Light->SetMobility(EComponentMobility::Movable);
		Light->SetLightColor(Desc.Color);
		Light->SetIntensity(Desc.Intensity);
		Light->SetCastShadows(Desc.bCastShadows);
		Light->SetVisibility(false);
		if (UPointLightComponent* Point = Cast<UPointLightComponent>(Light))
		{
			Point->IntensityUnits = ELightUnits::Candelas;
			Point->SetAttenuationRadius(Desc.RangeCm);
			Point->SetSourceRadius(Desc.SourceRadiusCm);
		}
		if (USpotLightComponent* Spot = Cast<USpotLightComponent>(Light))
		{
			Spot->SetInnerConeAngle(Desc.InnerConeDegrees);
			Spot->SetOuterConeAngle(Desc.OuterConeDegrees);
		}
		Light->RegisterComponent();
		Light->SetVisibility(false);
		Light->SetActive(Desc.bEnabled);
		OutTree.Lights[LightIndex] = Light;
	}
	return true;
}

bool AURLabRenderScene::BuildDynamicMeshes(const FRenderScene& CandidateScene,
	const FRenderFrame& InitialFrame, URenderAssetBuilder& CandidateBuilder,
	FCandidateTree& OutTree, FString& OutError)
{
	OutTree.Deformables.SetNum(CandidateScene.Deformables.Num());
	OutTree.DebugMeshes.SetNum(InitialFrame.DebugMeshes.Num());
	auto CreateSection = [this, &OutError](const FString& Name,
		const TArray<FVector>& Positions, const TArray<FVector>& SuppliedNormals,
		const TArray<int32>& Triangles, const TArray<FVector2D>& StaticUVs,
		const FLinearColor& Color, bool bCastShadow,
		TObjectPtr<UProceduralMeshComponent>& OutComponent) -> bool
	{
		if (Positions.Num() < 3 || Triangles.Num() < 3 || Triangles.Num() % 3 != 0)
		{
			OutError = FString::Printf(TEXT("dynamic render mesh '%s' has invalid topology"), *Name);
			return false;
		}
		TArray<FVector> Normals;
		Normals.Init(FVector::ZeroVector, Positions.Num());
		for (int32 I = 0; I + 2 < Triangles.Num(); I += 3)
		{
			const int32 A = Triangles[I], B = Triangles[I + 1], C = Triangles[I + 2];
			if (!Positions.IsValidIndex(A) || !Positions.IsValidIndex(B) || !Positions.IsValidIndex(C))
			{
				OutError = FString::Printf(TEXT("dynamic render mesh '%s' has an invalid index"), *Name);
				return false;
			}
			const FVector N = FVector::CrossProduct(Positions[C] - Positions[A],
				Positions[B] - Positions[A]);
			Normals[A] += N; Normals[B] += N; Normals[C] += N;
		}
		for (FVector& Normal : Normals) Normal.Normalize();
		if (SuppliedNormals.Num() == Normals.Num()) Normals = SuppliedNormals;
		TArray<FVector2D> UVs = StaticUVs;
		if (UVs.IsEmpty()) UVs.Init(FVector2D::ZeroVector, Positions.Num());
		TArray<FColor> Colors; Colors.Init(Color.ToFColor(true), Positions.Num());
		TArray<FProcMeshTangent> Tangents = DeformableTangents(Positions, Normals, Triangles, StaticUVs);
		UProceduralMeshComponent* Component = NewObject<UProceduralMeshComponent>(this);
		if (!Component)
		{
			OutError = FString::Printf(TEXT("failed to create dynamic render mesh '%s'"), *Name);
			return false;
		}
		Component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Component->SetCastShadow(bCastShadow);
		Component->SetupAttachment(GetRootComponent());
		Component->SetVisibility(false);
		Component->RegisterComponent();
		Component->CreateMeshSection(0, Positions, Triangles, Normals, UVs, Colors, Tangents, false);
		OutComponent = Component;
		return true;
	};

	for (int32 Index = 0; Index < CandidateScene.Deformables.Num(); ++Index)
	{
		const FRenderDeformableDesc& Desc = CandidateScene.Deformables[Index];
		const FRenderFrame::FDeformable& FrameData = InitialFrame.Deformables[Index];
		if (!CreateSection(Desc.Name, FrameData.Positions, FrameData.Normals, Desc.Triangles, Desc.UVs,
			FLinearColor::White, true, OutTree.Deformables[Index])) return false;
		if (Desc.SurfaceIndex != INDEX_NONE)
			CandidateBuilder.ApplyDeformableMaterial(OutTree.Deformables[Index], Desc.SurfaceIndex);
	}
	for (int32 Index = 0; Index < InitialFrame.DebugMeshes.Num(); ++Index)
	{
		const FRenderFrame::FDebugMesh& Mesh = InitialFrame.DebugMeshes[Index];
		if (!CreateSection(Mesh.Name, Mesh.Positions, {}, Mesh.Triangles, {}, Mesh.Color,
			false, OutTree.DebugMeshes[Index])) return false;
		FRenderResolvedMaterialOverride Override;
		Override.bEnabled = true;
		Override.BaseColor = Mesh.Color;
		Override.Material.Rgba = Mesh.Color;
		Override.Material.Emission = 3.0f;
		Override.Material.Roughness = 1.0f;
		CandidateBuilder.ApplyResolvedMaterialOverride(OutTree.DebugMeshes[Index], Override);
	}
	return true;
}

bool AURLabRenderScene::BuildEnvironment(const FRenderScene& CandidateScene,
	const FRenderSceneResources& Resources, FCandidateTree& OutTree, FString& OutError)
{
	const FRenderEnvironmentDesc& Env = CandidateScene.Environment;
	const float Fill = Env.AmbientIntensity
		+ (Env.bHeadlightEnabled ? Env.HeadlightIntensity : 0.0f);
	if (Fill > 0.0f || Env.bSkyboxEnabled)
	{
		USkyLightComponent* Sky = NewObject<USkyLightComponent>(this);
		if (!Sky) { OutError = TEXT("failed to create neutral sky light"); return false; }
		OutTree.SkyLight = Sky;
		Sky->SetMobility(EComponentMobility::Movable);
		Sky->SetupAttachment(GetRootComponent());
		Sky->bLowerHemisphereIsBlack = false;
		Sky->SetLightColor(Env.AmbientColor + (Env.bHeadlightEnabled
			? Env.HeadlightColor * Env.HeadlightIntensity : FLinearColor::Black));
		Sky->SetIntensity(FMath::Max(Fill, 0.0f));
		if (Env.SkyboxFaceTextureIndices.Num() == 6)
		{
			const FRenderTextureResource& First =
				Resources.Textures[Env.SkyboxFaceTextureIndices[0]];
			const int32 Size = First.Width;
			UTextureCube* Cube = UTextureCube::CreateTransient(Size, Size, PF_B8G8R8A8);
			if (!Cube || !Cube->GetPlatformData() || Cube->GetPlatformData()->Mips.IsEmpty())
			{
				OutError = TEXT("failed to create neutral skybox cubemap"); return false;
			}
			Cube->SRGB = true;
			Cube->NeverStream = true;
			FTexture2DMipMap& Mip = Cube->GetPlatformData()->Mips[0];
			const int64 Bytes = static_cast<int64>(Size) * Size * 6 * 4;
			if (Mip.BulkData.GetBulkDataSize() < Bytes)
			{
				OutError = TEXT("neutral skybox cube storage is incomplete"); return false;
			}
			uint8* Dest = static_cast<uint8*>(Mip.BulkData.Lock(LOCK_READ_WRITE));
			for (int32 Face = 0; Face < 6; ++Face)
			{
				const FRenderTextureResource& Texture =
					Resources.Textures[Env.SkyboxFaceTextureIndices[Face]];
				if (Texture.Width != Size || Texture.Height != Size
					|| Texture.Rgba8.Num() != Size * Size * 4)
				{
					Mip.BulkData.Unlock();
					OutError = TEXT("neutral skybox faces must have equal square RGBA8 dimensions");
					return false;
				}
				for (int32 Pixel = 0; Pixel < Size * Size; ++Pixel)
				{
					const uint8* Src = Texture.Rgba8.GetData() + 4 * Pixel;
					uint8* Dst = Dest + (Face * Size * Size + Pixel) * 4;
					Dst[0] = Src[2]; Dst[1] = Src[1]; Dst[2] = Src[0]; Dst[3] = Src[3];
				}
			}
			Mip.BulkData.Unlock();
			Cube->UpdateResource();
			Sky->SourceType = SLS_SpecifiedCubemap;
			Sky->Cubemap = Cube;
		}
		Sky->SetVisibility(false);
		Sky->RegisterComponent();
	}
	if (Env.bSkyboxEnabled)
	{
		UMaterialInterface* Base = LoadObject<UMaterialInterface>(nullptr,
			TEXT("/UnrealRoboticsLab/Materials/M_MjSkyDome.M_MjSkyDome"));
		UStaticMesh* Sphere = LoadObject<UStaticMesh>(nullptr,
			TEXT("/Engine/BasicShapes/Sphere.Sphere"));
		if (!Base || !Sphere)
		{
			OutError = TEXT("neutral sky dome material or sphere is missing"); return false;
		}
		UMaterialInstanceDynamic* Material = UMaterialInstanceDynamic::Create(Base, this);
		if (!Material) { OutError = TEXT("failed to create neutral sky dome material"); return false; }
		Material->SetVectorParameterValue(TEXT("Top"), Env.SkyTop);
		Material->SetVectorParameterValue(TEXT("Bottom"), Env.SkyBottom);
		UStaticMeshComponent* Dome = NewObject<UStaticMeshComponent>(this);
		if (!Dome) { OutError = TEXT("failed to create neutral sky dome"); return false; }
		OutTree.SkyDome = Dome;
		Dome->SetMobility(EComponentMobility::Movable);
		Dome->SetupAttachment(GetRootComponent());
		Dome->SetStaticMesh(Sphere);
		Dome->SetMaterial(0, Material);
		// The camera lives inside this sphere. Reverse its winding so the sky
		// remains visible even when the authored material is single-sided.
		Dome->SetRelativeScale3D(FVector(-200.0, 200.0, 200.0));
		Dome->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Dome->SetCastShadow(false);
		Dome->SetVisibility(false);
		Dome->RegisterComponent();
	}
	return true;
}

bool AURLabRenderScene::ApplyDynamicMeshes(const FRenderScene& CandidateScene,
	const FRenderFrame& Frame, TArray<TObjectPtr<UProceduralMeshComponent>>& Deformables,
	TArray<TObjectPtr<UProceduralMeshComponent>>& DebugMeshes, FString& OutError)
{
	for (int32 Index = 0; Index < Frame.Deformables.Num(); ++Index)
	{
		UProceduralMeshComponent* Component = Deformables.IsValidIndex(Index)
			? Deformables[Index].Get() : nullptr;
		if (!Component || !CandidateScene.Deformables.IsValidIndex(Index))
		{
			OutError = TEXT("installed deformable mesh is missing"); return false;
		}
		const FRenderFrame::FDeformable& Dynamic = Frame.Deformables[Index];
		const FRenderDeformableDesc& Static = CandidateScene.Deformables[Index];
		TArray<FVector> Normals;
		Normals.Init(FVector::ZeroVector, Dynamic.Positions.Num());
		for (int32 I = 0; I + 2 < Static.Triangles.Num(); I += 3)
		{
			const int32 A = Static.Triangles[I], B = Static.Triangles[I + 1], C = Static.Triangles[I + 2];
			const FVector N = FVector::CrossProduct(Dynamic.Positions[C] - Dynamic.Positions[A],
				Dynamic.Positions[B] - Dynamic.Positions[A]);
			Normals[A] += N; Normals[B] += N; Normals[C] += N;
		}
		for (FVector& Normal : Normals) Normal.Normalize();
		if (Dynamic.Normals.Num() == Normals.Num()) Normals = Dynamic.Normals;
		TArray<FVector2D> UVs = Static.UVs;
		if (UVs.IsEmpty()) UVs.Init(FVector2D::ZeroVector, Dynamic.Positions.Num());
		TArray<FColor> Colors; Colors.Init(FColor::White, Dynamic.Positions.Num());
		TArray<FProcMeshTangent> Tangents = DeformableTangents(Dynamic.Positions, Normals, Static.Triangles, Static.UVs);
		Component->UpdateMeshSection(0, Dynamic.Positions, Normals, UVs, Colors, Tangents);
	}
	while (DebugMeshes.Num() > Frame.DebugMeshes.Num())
	{
		if (DebugMeshes.Last()) DebugMeshes.Last()->DestroyComponent();
		DebugMeshes.Pop();
	}
	while (DebugMeshes.Num() < Frame.DebugMeshes.Num())
	{
		UProceduralMeshComponent* Component = NewObject<UProceduralMeshComponent>(this);
		if (!Component) { OutError = TEXT("failed to allocate debug drawable mesh"); return false; }
		Component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Component->SetCastShadow(false);
		Component->SetupAttachment(GetRootComponent());
		Component->SetVisibility(bSceneVisible);
		Component->RegisterComponent();
		DebugMeshes.Add(Component);
	}
	for (int32 Index = 0; Index < Frame.DebugMeshes.Num(); ++Index)
	{
		UProceduralMeshComponent* Component = DebugMeshes[Index].Get();
		if (!Component) { OutError = TEXT("installed debug mesh is missing"); return false; }
		const FRenderFrame::FDebugMesh& Mesh = Frame.DebugMeshes[Index];
		TArray<FVector> Normals; Normals.Init(FVector::ZeroVector, Mesh.Positions.Num());
		for (int32 I = 0; I + 2 < Mesh.Triangles.Num(); I += 3)
		{
			const int32 A = Mesh.Triangles[I], B = Mesh.Triangles[I + 1], C = Mesh.Triangles[I + 2];
			const FVector N = FVector::CrossProduct(Mesh.Positions[C] - Mesh.Positions[A],
				Mesh.Positions[B] - Mesh.Positions[A]);
			Normals[A] += N; Normals[B] += N; Normals[C] += N;
		}
		for (FVector& Normal : Normals) Normal.Normalize();
		TArray<FVector2D> UVs; UVs.Init(FVector2D::ZeroVector, Mesh.Positions.Num());
		TArray<FColor> Colors; Colors.Init(Mesh.Color.ToFColor(true), Mesh.Positions.Num());
		TArray<FProcMeshTangent> Tangents; Tangents.Init(FProcMeshTangent(), Mesh.Positions.Num());
		Component->ClearAllMeshSections();
		Component->CreateMeshSection(0, Mesh.Positions, Mesh.Triangles, Normals,
			UVs, Colors, Tangents, false);
		FRenderResolvedMaterialOverride Override;
		Override.bEnabled = true;
		Override.BaseColor = Mesh.Color;
		Override.Material.Rgba = Mesh.Color;
		Override.Material.Emission = 3.0f;
		Override.Material.Roughness = 1.0f;
		if (AssetBuilder) AssetBuilder->ApplyResolvedMaterialOverride(Component, Override);
	}
	return true;
}

UPrimitiveComponent* AURLabRenderScene::BuildSegmentationGeom(int32 GeomIndex, int32 ObjectId,
	const FRenderScene& CandidateScene, URenderAssetBuilder& CandidateBuilder,
	AActor& GeomActor, UMaterialInterface& SegmentationMaterial)
{
	UPrimitiveComponent* Component = BuildGeom(GeomIndex, CandidateScene, CandidateBuilder, GeomActor);
	if (!Component) return nullptr;
	Component->bVisibleInSceneCaptureOnly = true;
	Component->SetCastShadow(false);
	Component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Component->SetGenerateOverlapEvents(false);
	Component->bAffectDynamicIndirectLighting = false;
	Component->bAffectDistanceFieldLighting = false;
	Component->bVisibleInReflectionCaptures = false;
	Component->bVisibleInRealTimeSkyCaptures = false;
	Component->bVisibleInRayTracing = false;
	Component->bReceivesDecals = false;
	UMaterialInstanceDynamic* Material = UMaterialInstanceDynamic::Create(
		&SegmentationMaterial, Component);
	if (!Material) return nullptr;
	Material->SetVectorParameterValue(TEXT("SegId"), EncodeSegmentationId(ObjectId));
	const int32 Slots = FMath::Max(1, Component->GetNumMaterials());
	for (int32 Slot = 0; Slot < Slots; ++Slot) Component->SetMaterial(Slot, Material);
	// BuildGeom registers the primitive before this sibling-only state is known.
	// Recreate its proxy so the renderer observes scene-capture-only visibility,
	// disabled lighting participation, and the segmentation material together.
	Component->MarkRenderStateDirty();
	return Component;
}

UPrimitiveComponent* AURLabRenderScene::BuildGeom(int32 GeomIndex,
	const FRenderScene& CandidateScene, URenderAssetBuilder& CandidateBuilder, AActor& GeomActor)
{
	const FRenderGeomDesc& Geom = CandidateScene.Geoms[GeomIndex];
	auto ApplyMaterial = [&](UPrimitiveComponent* Component)
	{
		CandidateBuilder.ApplyGeomMaterial(Component, GeomIndex);
	};

	if (Geom.Kind != ERenderGeomKind::Mesh)
	{
		return BuildNeutralPrimitive(Geom, &GeomActor, ApplyMaterial);
	}
#if WITH_EDITOR
	UStaticMesh* Mesh = CandidateBuilder.GetOrBuildStaticMesh(Geom.MeshResourceIndex);
	if (!Mesh) return nullptr;
	UStaticMeshComponent* Component = NewObject<UStaticMeshComponent>(&GeomActor);
	if (!Component) return nullptr;
	Component->SetStaticMesh(Mesh);
	Component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	DisableDistanceFields(Component);
	Component->RegisterComponent();
	Component->AttachToComponent(GeomActor.GetRootComponent(),
		FAttachmentTransformRules::KeepRelativeTransform);
	ApplyMaterial(Component);
	return Component;
#else
	UProceduralMeshComponent* Component = CandidateBuilder.BuildMesh(
		Geom.MeshResourceIndex, &GeomActor);
	if (Component) ApplyMaterial(Component);
	return Component;
#endif
}

void AURLabRenderScene::ApplyFrameToBodies(const FRenderFrame& Frame,
	const TArray<TObjectPtr<AActor>>& Bodies)
{
	for (int32 BodyIndex = 0; BodyIndex < Bodies.Num(); ++BodyIndex)
	{
		if (AActor* Body = Bodies[BodyIndex])
		{
			Body->SetActorRelativeTransform(Frame.BodySceneTransforms[BodyIndex]);
		}
	}
}

void AURLabRenderScene::ApplyFrameToLights(const FRenderFrame& Frame,
	const TArray<TObjectPtr<ULightComponent>>& Lights)
{
	for (int32 LightIndex = 0; LightIndex < Lights.Num(); ++LightIndex)
		if (ULightComponent* Light = Lights[LightIndex])
			Light->SetRelativeTransform(Frame.LightSceneTransforms[LightIndex]);
}

void AURLabRenderScene::DestroyTree(FCandidateTree& Tree)
{
	if (Tree.CameraRig) Tree.CameraRig->Teardown();
	Tree.CameraRig = nullptr;
	for (ULightComponent* Light : Tree.Lights) if (Light) Light->DestroyComponent();
	if (Tree.SkyLight) Tree.SkyLight->DestroyComponent();
	if (Tree.SkyDome) Tree.SkyDome->DestroyComponent();
	for (UProceduralMeshComponent* Mesh : Tree.Deformables) if (Mesh) Mesh->DestroyComponent();
	for (UProceduralMeshComponent* Mesh : Tree.DebugMeshes) if (Mesh) Mesh->DestroyComponent();
	for (AActor* GeomActor : Tree.GeomActors) if (GeomActor) GeomActor->Destroy();
	for (AActor* Body : Tree.Bodies) if (Body) Body->Destroy();
	Tree.GeomComps.Reset();
	Tree.InstanceSegComps.Reset();
	Tree.SemanticSegComps.Reset();
	Tree.GeomActors.Reset();
	Tree.Bodies.Reset();
	Tree.Lights.Reset();
	Tree.SkyLight = nullptr;
	Tree.SkyDome = nullptr;
	Tree.Deformables.Reset();
	Tree.DebugMeshes.Reset();
}

void AURLabRenderScene::SetTreeHidden(const FCandidateTree& Tree, bool bHidden)
{
	auto SetActorTreeHidden = [bHidden](AActor* Actor)
	{
		if (!Actor) return;
		Actor->SetActorHiddenInGame(bHidden);
		TArray<USceneComponent*> Components;
		Actor->GetComponents(Components);
		for (USceneComponent* Component : Components)
			if (Component) Component->SetVisibility(!bHidden, true);
	};
	for (AActor* Body : Tree.Bodies) SetActorTreeHidden(Body);
	for (AActor* GeomActor : Tree.GeomActors) SetActorTreeHidden(GeomActor);
	if (Tree.CameraRig) Tree.CameraRig->SetVisible(!bHidden);
	for (ULightComponent* Light : Tree.Lights)
		if (Light) Light->SetVisibility(!bHidden && Light->IsActive());
	if (Tree.SkyLight) Tree.SkyLight->SetVisibility(!bHidden);
	if (Tree.SkyDome) Tree.SkyDome->SetVisibility(!bHidden);
	for (UProceduralMeshComponent* Mesh : Tree.Deformables)
		if (Mesh) Mesh->SetVisibility(!bHidden);
	for (UProceduralMeshComponent* Mesh : Tree.DebugMeshes)
		if (Mesh) Mesh->SetVisibility(!bHidden);
}

void AURLabRenderScene::TeardownScene()
{
	FCandidateTree Tree;
	Tree.Bodies = MoveTemp(BodyActors);
	Tree.GeomActors = MoveTemp(GeomActors);
	Tree.GeomComps = MoveTemp(GeomComps);
	Tree.InstanceSegComps = MoveTemp(InstanceSegComps);
	Tree.SemanticSegComps = MoveTemp(SemanticSegComps);
	Tree.Lights = MoveTemp(LightComponents);
	Tree.SkyLight = SkyLightComponent;
	Tree.SkyDome = SkyDomeComponent;
	SkyLightComponent = nullptr;
	SkyDomeComponent = nullptr;
	Tree.Deformables = MoveTemp(DeformableComponents);
	Tree.DebugMeshes = MoveTemp(DebugMeshComponents);
	Tree.CameraRig = CameraRig;
	CameraRig = nullptr;
	DestroyTree(Tree);
	if (AssetBuilder) AssetBuilder->Reset();
	AssetBuilder = nullptr;
	AppliedFrame = FRenderFrame();
	Scene = FRenderScene();
	ActiveGeomOverrideSerial = 0;
	bSceneVisible = false;
	ResourceLifetimeLease.Reset();
}

AActor* AURLabRenderScene::GetBodyActor(int32 BodyIndex) const
{
	return BodyActors.IsValidIndex(BodyIndex) ? BodyActors[BodyIndex].Get() : nullptr;
}

USceneComponent* AURLabRenderScene::GetBodyRootComponent(int32 BodyIndex) const
{
	AActor* Body = GetBodyActor(BodyIndex);
	return Body ? Body->GetRootComponent() : nullptr;
}

UPrimitiveComponent* AURLabRenderScene::GetGeomComponent(int32 GeomIndex) const
{
	return GeomComps.IsValidIndex(GeomIndex) ? GeomComps[GeomIndex].Get() : nullptr;
}

int32 AURLabRenderScene::NumBuiltGeoms() const
{
	int32 Count = 0;
	for (UPrimitiveComponent* Component : GeomComps) if (Component) ++Count;
	return Count;
}

bool AURLabRenderScene::IsGeomVisible(int32 GeomIndex) const
{
	if (!Scene.Geoms.IsValidIndex(GeomIndex)) return false;
	const int32 Group = Scene.Geoms[GeomIndex].Group;
	return Group >= 0 && Group <= 30 && (InstalledOptions.VisibleGroupMask & (1 << Group)) != 0;
}

bool AURLabRenderScene::AreRenderResourcesReadyForCapture() const
{
#if WITH_DEV_AUTOMATION_TESTS
	if (RenderResourcesReadyForTest.IsSet()) return RenderResourcesReadyForTest.GetValue();
#endif
	auto IsReady = [](const UPrimitiveComponent* Component)
	{
		if (!Component) return true;
		if (!Component->IsRegistered()) return false;
		// Visibility does not suppress render-state creation for these primitive
		// types: the hidden segmentation siblings still need proxies for show-only
		// captures. A drawable component declining a render state (for example while
		// its static mesh compiles) is therefore still pending, not safely skippable.
		if (!Component->ShouldCreateRenderState()) return false;
		// PSO completion schedules a render-state recreation on the game thread. Do
		// not issue a capture in the small interval after compilation completed but
		// before that deferred recreation has been processed.
		return !Component->IsPSOPrecaching()
			&& !Component->IsRenderStateDirty()
			&& !Component->IsRenderStateRecreating()
			&& Component->IsRenderStateCreated();
	};
	for (const UPrimitiveComponent* Component : GeomComps)
		if (!IsReady(Component)) return false;
	for (const UPrimitiveComponent* Component : InstanceSegComps)
		if (!IsReady(Component)) return false;
	for (const UPrimitiveComponent* Component : SemanticSegComps)
		if (!IsReady(Component)) return false;
	for (const UPrimitiveComponent* Component : DeformableComponents)
		if (!IsReady(Component)) return false;
	for (const UPrimitiveComponent* Component : DebugMeshComponents)
		if (!IsReady(Component)) return false;
	return true;
}

void AURLabRenderScene::SetGeomsVisible(bool bVisible)
{
	for (AActor* GeomActor : GeomActors) if (GeomActor) GeomActor->SetActorHiddenInGame(!bVisible);
}

void AURLabRenderScene::SetSceneVisible(bool bVisible)
{
	bSceneVisible = bVisible;
	FCandidateTree Tree;
	Tree.Bodies = BodyActors;
	Tree.GeomActors = GeomActors;
	Tree.GeomComps = GeomComps;
	Tree.Lights = LightComponents;
	Tree.SkyLight = SkyLightComponent;
	Tree.SkyDome = SkyDomeComponent;
	Tree.Deformables = DeformableComponents;
	Tree.DebugMeshes = DebugMeshComponents;
	Tree.CameraRig = CameraRig;
	SetTreeHidden(Tree, !bVisible);
	if (bVisible && SkyLightComponent) SkyLightComponent->RecaptureSky();
}
