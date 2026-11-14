#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Generated/MJSpecStructs.h"
#include "MJSpecHandle.h"
#include "MJModelComponent.generated.h"

class UMJAssetLibrary;
class USceneComponent;
class UMJBodyComponent;

USTRUCT(BlueprintType)
struct URLAB_API FMJDefaultClassEntry
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Default")
	FString ClassName = TEXT("main");

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Default")
	FString ParentClassName;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Default")
	FMJGeomSpec Geom;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Default")
	FMJJointSpec Joint;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Default")
	FMJSiteSpec Site;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Default")
	FMJActuatorSpec Actuator;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Default")
	FMJCameraSpec Camera;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Default")
	FMJLightSpec Light;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Default")
	FMJTendonSpec Tendon;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Default")
	FMJMeshSpec Mesh;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Default")
	FMJMaterialSpec Material;
};

/**
 * Owns mjSpec*, mjModel*, mjData*, and mjVFS for an Actor's MuJoCo component hierarchy.
 */
UCLASS(ClassGroup = (MuJoCo), meta = (BlueprintSpawnableComponent))
class URLAB_API UMJModelComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UMJModelComponent();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Model")
	FString ModelName = TEXT("MuJoCoModel");

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Model")
	bool bCompilerDegree = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Model")
	TObjectPtr<UMJAssetLibrary> AssetLibrary;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Model")
	TArray<FMJDefaultClassEntry> DefaultClasses;

	/** Rebuilds mjSpec from the owning Actor's MuJoCo components. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Model")
	bool BuildSpecFromActor(FString& OutError);

	/** Compiles Spec -> Model + Data (via mj_compile). */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Model")
	bool CompileModel(FString& OutError);

	/** Rebuilds Spec and calls mj_recompile in-place, preserving qpos/qvel. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Model")
	bool RecompileModel(FString& OutError);

	/** Steps simulation by Dt seconds and syncs body transforms. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Model")
	void StepSimulation(double Dt);

	/** Syncs UMJBodyComponent transforms from current mjData. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Model")
	void SyncTransformsFromData();

	/** Exports current Spec to MJCF XML string. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Model")
	bool ExportXML(
		FString& OutXml,
		FString& OutError,
		bool bSaveCompiled = false,
		bool bSaveCanonical = false,
		bool bSaveInertial = false);

	mjSpec* GetSpec() const { return SpecHandle.Get(); }
	mjModel* GetModel() const { return ModelHandle.Get(); }
	mjData* GetData() const { return DataHandle.Get(); }
	mjVFS* GetVFS() const { return VfsHandle.Get(); }

	/** Adopts an externally parsed mjSpec (e.g. from XML import). */
	void AdoptSpec(mjSpec* InSpec) { SpecHandle.Reset(InSpec); }

	void BuildDefaultsInSpec(mjSpec* S) const;
	void BuildNonSpatialComponents(mjSpec* S, const FString& Prefix = FString()) const;

private:
	void BuildSceneSubtree(mjSpec* S, USceneComponent* Comp, mjsBody* ParentBody, mjsFrame* ParentFrame) const;
	void RefreshRuntimeIds();

	FMJSpecHandle SpecHandle;
	FMJModelHandle ModelHandle;
	FMJDataHandle DataHandle;
	FMJVFSHandle VfsHandle;

	TArray<TWeakObjectPtr<UMJBodyComponent>> CachedBodies;
};
