// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"
#include "MuJoCo/Entity/MjGeomAssetResolver.h"
#include "MuJoCo/Entity/MjBakedAssetResolver.h"
#include "MuJoCo/Spec/MjSpecRef.h"

struct mjModel_;
class UMjbAssetBaker;
class UPrimitiveComponent;

/**
 * The authored path's resolver: a geom's component built from the UE assets a
 * model import produced. A mesh geom resolves its `<mesh>` element by name to the
 * imported UStaticMesh (and its textures through the same spec); a primitive geom
 * takes a scaled engine primitive the way the editor preview does. Anything the
 * importer never produced -- an inline mesh that lives only in the MJB -- falls
 * back to the baked resolver, so a mixed scene still draws in full.
 */
class URLAB_API FMjImportedAssetResolver : public IMjGeomAssetResolver
{
public:
	FMjImportedAssetResolver(mjModel_* InModel, const FSpecRef& InSpec, UMjbAssetBaker* InBaker);

	virtual UPrimitiveComponent* MakeGeomComponent(int32 GeomId, AActor* Body) const override;

private:
	// Apply the geom's authored material onto Comp: resolve the `<material>` it
	// names through the spec and route it, base colour and planar size included,
	// through the shared MjApplyMaterialParameters + master material.
	void ApplyImportedMaterial(UPrimitiveComponent* Comp, int32 GeomId) const;

	// Borrowed; the owning scene holds the mjModel and baker lifetimes.
	mjModel_* Model = nullptr;
	FSpecRef Spec;

	// Draws the inline meshes the import never produced.
	FMjBakedAssetResolver BakedFallback;
};
