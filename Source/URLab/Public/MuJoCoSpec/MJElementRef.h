#pragma once

#include "CoreMinimal.h"
#include "Engine/EngineTypes.h"
#include "MJElementRef.generated.h"

/**
 * Rename-safe, Blueprint-inheritance-safe reference to a MuJoCo element.
 * Prefers FComponentReference (for scene/actor components) or soft asset ref,
 * falling back to explicit string name when referencing XML-defined assets.
 */
USTRUCT(BlueprintType)
struct URLAB_API FMJElementRef
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Ref")
	FComponentReference ComponentRef;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Ref")
	TSoftObjectPtr<UObject> AssetRef;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Ref")
	FString FallbackName;

	FString ResolveName(const AActor* OwnerActor) const;

	bool IsEmpty() const
	{
		return ComponentRef.ComponentProperty == NAME_None
			&& ComponentRef.PathToComponent.IsEmpty()
			&& AssetRef.IsNull()
			&& FallbackName.IsEmpty();
	}
};
