#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "Generated/MJSpecStructs.h"
#include "MJElementRef.h"
#include "MJSceneComponents.generated.h"

/** Base spatial component for MuJoCo elements attached to a USceneComponent hierarchy. */
UCLASS(Abstract, ClassGroup = (MuJoCo))
class URLAB_API UMJSceneComponent : public USceneComponent
{
	GENERATED_BODY()

public:
	/** Optional explicit MuJoCo element name; if empty, uses GetName(). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
	FString ElementName;

	/** Optional <default class="..."> name for inheritance. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
	FString DefaultClass;

	FString GetMJName() const { return ElementName.IsEmpty() ? GetName() : ElementName; }
};

UCLASS(ClassGroup = (MuJoCo), meta = (BlueprintSpawnableComponent))
class URLAB_API UMJBodyComponent : public UMJSceneComponent
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Body")
	FMJBodySpec Spec;

	UPROPERTY(Transient, BlueprintReadOnly, Category = "MuJoCo|Runtime")
	int32 BodyId = -1;
};

UCLASS(ClassGroup = (MuJoCo), meta = (BlueprintSpawnableComponent))
class URLAB_API UMJFrameComponent : public UMJSceneComponent
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Frame")
	FMJFrameSpec Spec;
};

UCLASS(ClassGroup = (MuJoCo), meta = (BlueprintSpawnableComponent))
class URLAB_API UMJGeomComponent : public UMJSceneComponent
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Geom")
	FMJGeomSpec Spec;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Geom")
	FMJElementRef MaterialRef;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Geom")
	FMJElementRef MeshRef;

	UPROPERTY(Transient, BlueprintReadOnly, Category = "MuJoCo|Runtime")
	int32 GeomId = -1;
};

UCLASS(ClassGroup = (MuJoCo), meta = (BlueprintSpawnableComponent))
class URLAB_API UMJJointComponent : public UMJSceneComponent
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Joint")
	FMJJointSpec Spec;

	UPROPERTY(Transient, BlueprintReadOnly, Category = "MuJoCo|Runtime")
	int32 JointId = -1;
};

UCLASS(ClassGroup = (MuJoCo), meta = (BlueprintSpawnableComponent))
class URLAB_API UMJSiteComponent : public UMJSceneComponent
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Site")
	FMJSiteSpec Spec;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Site")
	FMJElementRef MaterialRef;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Site")
	FMJElementRef MeshRef;

	UPROPERTY(Transient, BlueprintReadOnly, Category = "MuJoCo|Runtime")
	int32 SiteId = -1;
};

UCLASS(ClassGroup = (MuJoCo), meta = (BlueprintSpawnableComponent))
class URLAB_API UMJCameraComponent : public UMJSceneComponent
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera")
	FMJCameraSpec Spec;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera")
	FMJElementRef TargetBodyRef;

	UPROPERTY(Transient, BlueprintReadOnly, Category = "MuJoCo|Runtime")
	int32 CamId = -1;
};

UCLASS(ClassGroup = (MuJoCo), meta = (BlueprintSpawnableComponent))
class URLAB_API UMJLightComponent : public UMJSceneComponent
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Light")
	FMJLightSpec Spec;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Light")
	FMJElementRef TargetBodyRef;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Light")
	FMJElementRef TextureRef;

	UPROPERTY(Transient, BlueprintReadOnly, Category = "MuJoCo|Runtime")
	int32 LightId = -1;
};

/** Attaches a sub-model XML via MuJoCo 4 mjs_attach(). */
UCLASS(ClassGroup = (MuJoCo), meta = (BlueprintSpawnableComponent))
class URLAB_API UMJAttachComponent : public UMJSceneComponent
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Attach")
	FString ChildXml;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Attach")
	FString Prefix;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Attach")
	FString Suffix;
};
