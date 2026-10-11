// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#pragma once

#include "CoreMinimal.h"
#include "MuJoCo/Gen/Elements/Assets/MjModelAsset.gen.h"
#include "MuJoCo/Gen/MjDispatch.gen.h"
#include "MuJoCo/Spec/MjGenHooks.h"

#include "MjModelAsset.generated.h"

class AMjArticulation;

/** A native `<model file>` whose authoring source is another editable articulation Blueprint. */
UCLASS(ClassGroup = (MuJoCo), meta = (BlueprintSpawnableComponent))
class URLAB_API UMjModelAsset : public UMjModelAssetBase
{
	GENERATED_BODY()

public:
	/** Child authoring source. Compilation exports its current component tree to native MJCF. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Model")
	TSoftClassPtr<AMjArticulation> ChildModelClass;

#if WITH_EDITOR
	/** Keep the schema file input read-only; it is derived from ChildModelClass. */
	virtual bool CanEditChange(const FProperty* InProperty) const override;
#endif
};

#if defined(URLAB_MJ_GEN) && URLAB_MJ_GEN
namespace ps::ue
{
template <>
struct TMjElementType<UMjModelAsset> : TMjElementType<UMjModelAssetBase>
{
};
}
#endif
