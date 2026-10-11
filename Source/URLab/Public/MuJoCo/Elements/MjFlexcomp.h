// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include "CoreMinimal.h"
#include "MuJoCo/Gen/Elements/Deformable/MjFlexcomp.gen.h"
#include "MjFlexcomp.generated.h"

class UStaticMesh;

/** Authored <flexcomp> element. The MuJoCo adapter owns runtime deformation. */
UCLASS(ClassGroup = (MuJoCo), meta = (BlueprintSpawnableComponent))
class URLAB_API UMjFlexcomp : public UMjFlexcompBase
{
	GENERATED_BODY()

public:
	/** UE-authored surface mesh backing a file-based type=mesh flexcomp. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Flexcomp")
	TObjectPtr<UStaticMesh> MeshAsset;
};
