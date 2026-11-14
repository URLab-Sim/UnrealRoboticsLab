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
	FMJTextureSpec Texture;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Default")
	FMJMaterialSpec Material;

	UPROPERTY()
	bool bOverride_GeomPos = false;
	UPROPERTY()
	FVector GeomPos = FVector::ZeroVector;
	UPROPERTY()
	bool bOverride_GeomQuat = false;
	UPROPERTY()
	FQuat GeomQuat = FQuat::Identity;

	UPROPERTY()
	bool bOverride_SitePos = false;
	UPROPERTY()
	FVector SitePos = FVector::ZeroVector;
	UPROPERTY()
	bool bOverride_SiteQuat = false;
	UPROPERTY()
	FQuat SiteQuat = FQuat::Identity;
};

USTRUCT(BlueprintType)
struct URLAB_API FMJMeshAssetEntry
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Asset")
	FString Name;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Asset")
	FString DefaultClass;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Asset")
	FString File;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Asset")
	FString ContentType;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Asset")
	FMJMeshSpec Spec;

	UPROPERTY()
	TArray<float> Userverts;
	UPROPERTY()
	TArray<float> Usernormals;
	UPROPERTY()
	TArray<float> Usertexcoords;
	UPROPERTY()
	TArray<int32> Userfaces;
	UPROPERTY()
	TArray<int32> Userfacenormals;
	UPROPERTY()
	TArray<int32> Userfacetexcoords;
	UPROPERTY()
	TArray<int32> Usertets;
};

USTRUCT(BlueprintType)
struct URLAB_API FMJHFieldAssetEntry
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Asset")
	FString Name;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Asset")
	FString File;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Asset")
	FString ContentType;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Asset")
	TArray<double> Size;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Asset")
	int32 Nrow = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Asset")
	int32 Ncol = 0;

	UPROPERTY()
	TArray<float> Userdata;
};

USTRUCT(BlueprintType)
struct URLAB_API FMJTextureAssetEntry
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Asset")
	FString Name;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Asset")
	FString DefaultClass;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Asset")
	FString File;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Asset")
	FString ContentType;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Asset")
	TArray<FString> CubeFiles;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Asset")
	FMJTextureSpec Spec;
};

USTRUCT(BlueprintType)
struct URLAB_API FMJMaterialAssetEntry
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Asset")
	FString Name;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Asset")
	FString DefaultClass;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Asset")
	TArray<FString> Textures;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Asset")
	FMJMaterialSpec Spec;
};

USTRUCT(BlueprintType)
struct URLAB_API FMJEqualityEntry
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Equality")
	FString Name;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Equality")
	FString DefaultClass;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Equality")
	int32 Type = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Equality")
	int32 ObjType = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Equality")
	FString Name1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Equality")
	FString Name2;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Equality")
	bool bActive = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Equality")
	TArray<double> Data;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Equality")
	TArray<double> Solref;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Equality")
	TArray<double> Solimp;
};

USTRUCT(BlueprintType)
struct URLAB_API FMJExcludeEntry
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Contact")
	FString Name;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Contact")
	FString BodyName1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Contact")
	FString BodyName2;
};

USTRUCT(BlueprintType)
struct URLAB_API FMJPairEntry
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Contact")
	FString Name;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Contact")
	FString DefaultClass;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Contact")
	FString GeomName1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Contact")
	FString GeomName2;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Contact")
	int32 Condim = 3;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Contact")
	double Margin = 0.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Contact")
	double Gap = 0.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Contact")
	double Adhesion = 0.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Contact")
	TArray<double> Solref;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Contact")
	TArray<double> SolrefFriction;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Contact")
	TArray<double> Solimp;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Contact")
	TArray<double> Friction;
};

USTRUCT(BlueprintType)
struct URLAB_API FMJFlexEntry
{
	GENERATED_BODY()

	UPROPERTY()
	FString Name;

	UPROPERTY()
	int32 Contype = 1;
	UPROPERTY()
	int32 Conaffinity = 1;
	UPROPERTY()
	int32 Condim = 3;
	UPROPERTY()
	int32 Priority = 0;
	UPROPERTY()
	TArray<double> Friction;
	UPROPERTY()
	double Solmix = 1.0;
	UPROPERTY()
	TArray<double> Solref;
	UPROPERTY()
	TArray<double> Solimp;
	UPROPERTY()
	double Margin = 0.0;
	UPROPERTY()
	double Gap = 0.0;

	UPROPERTY()
	int32 Dim = 2;
	UPROPERTY()
	double Radius = 0.005;
	UPROPERTY()
	TArray<double> Size;
	UPROPERTY()
	bool bFlatskin = false;
	UPROPERTY()
	int32 Selfcollide = 0;
	UPROPERTY()
	int32 Passive = 0;
	UPROPERTY()
	int32 Activelayers = 1;
	UPROPERTY()
	int32 Group = 0;
	UPROPERTY()
	double Edgestiffness = 0.0;
	UPROPERTY()
	double Edgedamping = 0.0;
	UPROPERTY()
	FLinearColor Rgba = FLinearColor(0.5f, 0.5f, 0.5f, 1.0f);
	UPROPERTY()
	FString Material;
	UPROPERTY()
	double Young = 0.0;
	UPROPERTY()
	double Poisson = 0.25;
	UPROPERTY()
	double Shear = 0.0;
	UPROPERTY()
	double Damping = 0.0;
	UPROPERTY()
	double Thickness = 0.0;
	UPROPERTY()
	TArray<double> Stressyield;
	UPROPERTY()
	int32 Elastic2d = 0;
	UPROPERTY()
	int32 Elastic3d = 0;
	UPROPERTY()
	int32 Section = 0;
	UPROPERTY()
	TArray<double> Sectionsize;
	UPROPERTY()
	TArray<int32> Cellcount;
	UPROPERTY()
	int32 Order = 0;

	UPROPERTY()
	TArray<FString> Nodebody;
	UPROPERTY()
	TArray<FString> Vertbody;
	UPROPERTY()
	TArray<double> Node;
	UPROPERTY()
	TArray<double> Vert;
	UPROPERTY()
	TArray<int32> Elem;
	UPROPERTY()
	TArray<float> Texcoord;
	UPROPERTY()
	TArray<int32> Elemtexcoord;
};

USTRUCT(BlueprintType)
struct URLAB_API FMJTupleEntry
{
	GENERATED_BODY()

	UPROPERTY()
	FString Name;
	UPROPERTY()
	TArray<int32> Objtype;
	UPROPERTY()
	TArray<FString> Objname;
	UPROPERTY()
	TArray<double> Objprm;
};

USTRUCT(BlueprintType)
struct URLAB_API FMJCompilerSettings
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Compiler")
	bool bEnabled = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Compiler")
	bool bAutolimits = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Compiler")
	double BoundMass = 0.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Compiler")
	double BoundInertia = 0.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Compiler")
	double SetTotalMass = -1.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Compiler")
	bool bBalanceInertia = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Compiler")
	bool bFitAabb = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Compiler")
	FString EulerSeq = TEXT("xyz");

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Compiler")
	bool bDiscardVisual = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Compiler")
	bool bFuseStatic = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Compiler")
	int32 InertiaFromGeom = 2;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Compiler")
	int32 InertiaGroupRangeMin = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Compiler")
	int32 InertiaGroupRangeMax = 5;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Compiler")
	bool bAlignFree = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Compiler")
	FString MeshDir;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Compiler")
	FString TextureDir;
};

USTRUCT(BlueprintType)
struct URLAB_API FMJOptionSettings
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Option")
	bool bEnabled = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Option")
	double Timestep = 0.002;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Option")
	double ImpRatio = 1.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Option")
	double Tolerance = 1e-8;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Option")
	double LsTolerance = 0.01;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Option")
	double NoslipTolerance = 1e-6;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Option")
	double CcdTolerance = 1e-6;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Option")
	double SleepTolerance = 1e-4;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Option")
	FVector Gravity = FVector(0.0, 0.0, -9.81);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Option")
	FVector Wind = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Option")
	FVector Magnetic = FVector(0.0, -0.5, 0.0);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Option")
	double Density = 0.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Option")
	double Viscosity = 0.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Option")
	int32 Integrator = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Option")
	int32 Cone = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Option")
	int32 Jacobian = 2;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Option")
	int32 Solver = 2;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Option")
	int32 Iterations = 100;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Option")
	int32 LsIterations = 50;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Option")
	int32 NoslipIterations = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Option")
	int32 CcdIterations = 50;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Option")
	int32 DisableFlags = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Option")
	int32 EnableFlags = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Option")
	int32 DisableActuator = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Option")
	int32 SdfInitPoints = 40;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Option")
	int32 SdfIterations = 10;
};

USTRUCT(BlueprintType)
struct URLAB_API FMJSizeSettings
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Size")
	bool bEnabled = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Size")
	int64 Memory = -1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Size")
	int32 Nemax = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Size")
	int32 Nuserdata = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Size")
	int32 NuserBody = -1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Size")
	int32 NuserJnt = -1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Size")
	int32 NuserGeom = -1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Size")
	int32 NuserSite = -1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Size")
	int32 NuserCam = -1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Size")
	int32 NuserTendon = -1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Size")
	int32 NuserActuator = -1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Size")
	int32 NuserSensor = -1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Size")
	int32 Nkey = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Size")
	int32 Njmax = -1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Size")
	int32 Nconmax = -1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Size")
	int64 Nstack = -1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Size")
	bool bStripPath = false;
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
	FMJCompilerSettings CompilerSettings;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Model")
	FMJOptionSettings OptionSettings;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Model")
	FMJSizeSettings SizeSettings;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Model")
	TObjectPtr<UMJAssetLibrary> AssetLibrary;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Model")
	TArray<FMJDefaultClassEntry> DefaultClasses;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Asset")
	TArray<FMJTextureAssetEntry> Textures;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Asset")
	TArray<FMJMaterialAssetEntry> Materials;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Asset")
	TArray<FMJMeshAssetEntry> Meshes;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Asset")
	TArray<FMJHFieldAssetEntry> HFields;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Equality")
	TArray<FMJEqualityEntry> Equalities;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Contact")
	TArray<FMJExcludeEntry> Excludes;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Contact")
	TArray<FMJPairEntry> Pairs;

	UPROPERTY()
	TArray<FMJFlexEntry> Flexes;

	UPROPERTY()
	TArray<FMJTupleEntry> Tuples;

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
	void BuildAssetsInSpec(mjSpec* S) const;
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
