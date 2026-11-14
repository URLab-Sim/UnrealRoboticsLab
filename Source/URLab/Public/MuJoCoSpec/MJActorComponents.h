#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Generated/MJSpecStructs.h"
#include "MJElementRef.h"
#include "MJActorComponents.generated.h"

UCLASS(Abstract, ClassGroup = (MuJoCo))
class URLAB_API UMJActorComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
	FString ElementName;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
	FString DefaultClass;

	FString GetMJName() const { return ElementName.IsEmpty() ? GetName() : ElementName; }
};

UCLASS(ClassGroup = (MuJoCo), meta = (BlueprintSpawnableComponent))
class URLAB_API UMJActuatorComponent : public UMJActorComponent
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator")
	EMJActuatorShortcut Shortcut = EMJActuatorShortcut::General;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator")
	FMJActuatorShortcutParams ShortcutParams;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator")
	FMJActuatorSpec Spec;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator")
	FMJElementRef TargetRef;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator")
	double Ctrl = 0.0;

	UPROPERTY(Transient, BlueprintReadOnly, Category = "MuJoCo|Runtime")
	int32 ActuatorId = -1;
};

UCLASS(ClassGroup = (MuJoCo), meta = (BlueprintSpawnableComponent))
class URLAB_API UMJSensorComponent : public UMJActorComponent
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Sensor")
	FMJSensorSpec Spec;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Sensor")
	FMJElementRef ObjRef;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Sensor")
	FMJElementRef RefObjRef;

	UPROPERTY(Transient, BlueprintReadOnly, Category = "MuJoCo|Runtime")
	int32 SensorId = -1;
};

USTRUCT(BlueprintType)
struct URLAB_API FMJTendonWrapEntry
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Tendon")
	EMJWrap WrapType = EMJWrap::Site;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Tendon")
	FMJElementRef TargetRef;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Tendon")
	FMJElementRef SideSiteRef;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Tendon")
	double CoefOrDivisor = 1.0;
};

UCLASS(ClassGroup = (MuJoCo), meta = (BlueprintSpawnableComponent))
class URLAB_API UMJTendonComponent : public UMJActorComponent
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Tendon")
	FMJTendonSpec Spec;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Tendon")
	FMJElementRef MaterialRef;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Tendon")
	TArray<FMJTendonWrapEntry> Wraps;

	UPROPERTY(Transient, BlueprintReadOnly, Category = "MuJoCo|Runtime")
	int32 TendonId = -1;
};
