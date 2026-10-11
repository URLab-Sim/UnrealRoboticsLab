// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Render/RenderFrame.h"
#include "Render/RenderScene.h"
#include "URLabRenderScene.generated.h"

class UPrimitiveComponent;
class UMaterialInterface;
class URenderAssetBuilder;
class URenderCameraComponent;
class URenderCameraRig;
class ULightComponent;
class USkyLightComponent;
class UStaticMeshComponent;
class UProceduralMeshComponent;

/** Physics-independent production owner of URLab's rigid Unreal scene graph. */
UCLASS()
class URLAB_API AURLabRenderScene : public AActor
{
	GENERATED_BODY()

public:
	AURLabRenderScene();
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Destroyed() override;

	/** Validate and stage a complete rigid scene before atomically replacing the live tree. */
	bool InstallScene(FRenderSceneInstallPayload&& Payload,
		const FRenderSceneInstallOptions& Options,
		FString& OutError);
	/** Copy the currently installed neutral inputs and resource lease for an atomic
	 *  scene configuration replacement. No MuJoCo data is exposed here. */
	bool CopyInstalledPayload(FRenderSceneInstallPayload& OutPayload,
		FRenderSceneInstallOptions& OutOptions, FString& OutError) const;

	/** Validate the complete frame before applying any body transform. */
	bool ApplyFrame(const FRenderFrame& Candidate, FString& OutError);
	bool PlanGeomPoseOverrides(const TArray<FRenderGeomPoseOverride>& Overrides,
		TArray<FRenderGeomPoseOverride>& OutCanonical, FString& OutConfigurationId,
		FString& OutError) const;
	bool BeginGeomPoseOverrides(const TArray<FRenderGeomPoseOverride>& CanonicalOverrides,
		FRenderGeomPoseOverrideToken& OutToken, FString& OutError);
	bool RestoreGeomPoseOverrides(const FRenderGeomPoseOverrideToken& Token, FString& OutError);

	void TeardownScene();

	const FRenderScene& GetScene() const { return Scene; }
	const FRenderFrame& GetAppliedFrame() const { return AppliedFrame; }
	URenderAssetBuilder* GetAssetBuilder() const { return AssetBuilder; }
	AActor* GetBodyActor(int32 BodyIndex) const;
	USceneComponent* GetBodyRootComponent(int32 BodyIndex) const;
	UPrimitiveComponent* GetGeomComponent(int32 GeomIndex) const;
	UProceduralMeshComponent* GetDeformableComponent(int32 Index) const
	{ return DeformableComponents.IsValidIndex(Index) ? DeformableComponents[Index].Get() : nullptr; }
	int32 NumDebugMeshComponents() const { return DebugMeshComponents.Num(); }
	URenderCameraRig* GetCameraRig() const { return CameraRig; }
	int32 NumBodyActors() const { return BodyActors.Num(); }
	int32 NumGeoms() const { return GeomComps.Num(); }
	int32 NumBuiltGeoms() const;
	bool IsGeomVisible(int32 GeomIndex) const;
	/** True once every renderable primitive owned by this scene can be captured. */
	bool AreRenderResourcesReadyForCapture() const;
	void SetSceneVisible(bool bVisible);
	void SetGeomsVisible(bool bVisible);

#if WITH_DEV_AUTOMATION_TESTS
	/** Deterministic scheduler gate; unset restores the production component predicate. */
	void SetRenderResourcesReadyForTest(TOptional<bool> bReady)
	{
		RenderResourcesReadyForTest = bReady;
	}
#endif

private:
	struct FCandidateTree
	{
		TArray<TObjectPtr<AActor>> Bodies;
		TArray<TObjectPtr<AActor>> GeomActors;
		TArray<TObjectPtr<UPrimitiveComponent>> GeomComps;
		TArray<TObjectPtr<UPrimitiveComponent>> InstanceSegComps;
		TArray<TObjectPtr<UPrimitiveComponent>> SemanticSegComps;
		TArray<TObjectPtr<ULightComponent>> Lights;
		TObjectPtr<USkyLightComponent> SkyLight = nullptr;
		TObjectPtr<UStaticMeshComponent> SkyDome = nullptr;
		TArray<TObjectPtr<UProceduralMeshComponent>> Deformables;
		TArray<TObjectPtr<UProceduralMeshComponent>> DebugMeshes;
		TObjectPtr<URenderCameraRig> CameraRig = nullptr;
	};

	bool BuildCandidate(const FRenderScene& CandidateScene,
		const FRenderSceneResources& CandidateResources, URenderAssetBuilder& CandidateBuilder,
		const FRenderSceneInstallOptions& Options, FCandidateTree& OutTree, FString& OutError);
	UPrimitiveComponent* BuildGeom(int32 GeomIndex, const FRenderScene& CandidateScene,
		URenderAssetBuilder& CandidateBuilder, AActor& GeomActor);
	UPrimitiveComponent* BuildSegmentationGeom(int32 GeomIndex, int32 ObjectId,
		const FRenderScene& CandidateScene, URenderAssetBuilder& CandidateBuilder,
		AActor& GeomActor, UMaterialInterface& SegmentationMaterial);
	static void ApplyFrameToBodies(const FRenderFrame& Frame,
		const TArray<TObjectPtr<AActor>>& Bodies);
	static void ApplyFrameToLights(const FRenderFrame& Frame,
		const TArray<TObjectPtr<ULightComponent>>& Lights);
	bool BuildDynamicMeshes(const FRenderScene& CandidateScene, const FRenderFrame& InitialFrame,
		URenderAssetBuilder& CandidateBuilder, FCandidateTree& OutTree, FString& OutError);
	bool BuildEnvironment(const FRenderScene& CandidateScene,
		const FRenderSceneResources& Resources, FCandidateTree& OutTree, FString& OutError);
	bool ApplyDynamicMeshes(const FRenderScene& CandidateScene, const FRenderFrame& Frame,
		TArray<TObjectPtr<UProceduralMeshComponent>>& Deformables,
		TArray<TObjectPtr<UProceduralMeshComponent>>& DebugMeshes, FString& OutError);
	static void DestroyTree(FCandidateTree& Tree);
	static void SetTreeHidden(const FCandidateTree& Tree, bool bHidden);

	FRenderSceneResourceLifetimeLeasePtr ResourceLifetimeLease;
	FRenderScene Scene;
	FRenderFrame AppliedFrame;
	FRenderSceneInstallOptions InstalledOptions;

	UPROPERTY(Transient)
	TObjectPtr<URenderAssetBuilder> AssetBuilder = nullptr;
	UPROPERTY(Transient)
	TArray<TObjectPtr<AActor>> BodyActors;
	UPROPERTY(Transient)
	TArray<TObjectPtr<AActor>> GeomActors;
	UPROPERTY(Transient)
	TArray<TObjectPtr<UPrimitiveComponent>> GeomComps;
	UPROPERTY(Transient)
	TArray<TObjectPtr<UPrimitiveComponent>> InstanceSegComps;
	UPROPERTY(Transient)
	TArray<TObjectPtr<UPrimitiveComponent>> SemanticSegComps;
	UPROPERTY(Transient)
	TArray<TObjectPtr<ULightComponent>> LightComponents;
	UPROPERTY(Transient)
	TObjectPtr<USkyLightComponent> SkyLightComponent = nullptr;
	UPROPERTY(Transient)
	TObjectPtr<UStaticMeshComponent> SkyDomeComponent = nullptr;
	UPROPERTY(Transient)
	TArray<TObjectPtr<UProceduralMeshComponent>> DeformableComponents;
	UPROPERTY(Transient)
	TArray<TObjectPtr<UProceduralMeshComponent>> DebugMeshComponents;
	UPROPERTY(Transient)
	TObjectPtr<URenderCameraRig> CameraRig = nullptr;
	uint64 NextGeomOverrideSerial = 1;
	uint64 ActiveGeomOverrideSerial = 0;
	bool bSceneVisible = true;
#if WITH_DEV_AUTOMATION_TESTS
	TOptional<bool> RenderResourcesReadyForTest;
#endif
};
