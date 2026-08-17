// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
// trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"

#include "MuJoCo/Spec/MjSceneContributor.h"

#include "MjQuickConvertComponent.generated.h"

class AActor;
class UMjBodyBase;
class UMjModel;
class UMjNodeComponent;

/**
 * The settings and the runtime coupling for an actor whose physics comes from
 * its Unreal geometry rather than from authored MJCF.
 *
 * The properties here are what a conversion of that geometry has to be told:
 * how accurate a convex decomposition of the actor's mesh needs to be, whether
 * the resulting body moves at all, whether Unreal drives it as a mocap body,
 * and the contact parameters its collision geoms carry. Alongside them sits the
 * one per-frame coupling that survives: when Unreal drives the body, the actor's
 * mocap pose is pushed into the engine. The reverse -- writing the simulated
 * pose back onto the actor -- is gone: the prop renders through the one entity
 * render view like every other body, from the compiled model, so the actor's
 * own mesh is hidden at play to keep the prop drawn exactly once.
 *
 * The conversion itself authors a spec on the owning actor -- one body, its
 * free joint, a geom per hull, and a `<mesh>` per exported hull under `<asset>`
 * -- which the scene attaches as an ordinary participant. The hulls have no
 * source file, so one is written: an OBJ under the project's Saved directory,
 * content-hashed so an unchanged mesh is exported once and re-read on every
 * later compile, exactly as an imported robot's meshes are. The visual geom's
 * `<mesh>` also records the actor's own StaticMesh, so the render view draws the
 * source asset at full fidelity while collision runs on the exported hulls.
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class URLAB_API UMjQuickConvertComponent : public UActorComponent
	, public IMjSceneContributor
{
	GENERATED_BODY()

public:
	UMjQuickConvertComponent();

	// --- Conversion settings ----------------------------------------------- //

	/** Run CoACD on the mesh instead of using its convex hull directly. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MuJoCo|Mesh",
		meta = (ToolTip = "When true, runs CoACD convex decomposition on the mesh to produce accurate collision geometry. Slower but required for non-convex shapes."))
	bool ComplexMeshRequired = false;

	/** CoACD concavity threshold: lower is more accurate and more hulls. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MuJoCo|Mesh",
		meta = (EditCondition = "ComplexMeshRequired", EditConditionHides, ClampMin = "0.01", ClampMax = "1.0",
			ToolTip = "CoACD concavity threshold. Lower values produce more accurate (but more) convex hulls. Default 0.05."))
	float CoACDThreshold = 0.05f;

	/** Draw the collision geoms of the converted body in the viewport. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MuJoCo|Debug",
		meta = (ToolTip = "Draw debug wireframes for all MuJoCo collision geoms created by this component."))
	bool m_debug_meshes = false;

	/** A static body carries no free joint and cannot move under physics. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MuJoCo|Physics",
		meta = (ToolTip = "Static bodies have no free joint and cannot move under physics forces. Use for fixed obstacles."))
	bool Static = false;

	/** One-way coupling: Unreal owns the transform, MuJoCo follows it. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MuJoCo|Physics",
		meta = (ToolTip = "When true, writes the actor's world transform to MuJoCo as a mocap body every tick. The physics simulation does not feed back into Unreal."))
	bool bDrivenByUnreal = false;

	/** Sliding, torsional and rolling friction, for every geom converted here. */
	UPROPERTY(EditAnywhere, Category = "MuJoCo|Physics")
	FVector3d friction = {1.0, 1, 1};

	/** Constraint solver reference (timeconst, dampratio), for every geom. */
	UPROPERTY(EditAnywhere, Category = "MuJoCo|Physics")
	FVector3d solref = {0.02, 1.0, 0.0};

	/** Constraint solver impedance (dmin, dmax, width), for every geom. */
	UPROPERTY(EditAnywhere, Category = "MuJoCo|Physics")
	FVector3d solimp = {0.9, 0.95, 0.001};

	/** Geom name to compiled-model id, for inspecting a converted actor. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|Debug",
		meta = (ToolTip = "Read-only map from geom name to MuJoCo geom ID. Populated after the model is compiled. Useful for debugging."))
	TMap<FString, int> m_geomName2ID;

	// --- Runtime ------------------------------------------------------------ //

	/** The MJCF name of the body this component converted its actor into. */
	FString GetBodyName();

	/** The compiled-model body id, or -1 when there is no bound body. */
	int32 GetMjBodyId() const;

	// --- Conversion, as a value ---------------------------------------------- //

	/** The conversion settings a body is authored from, as a plain value. */
	struct FConvertSettings
	{
		bool Static = false;
		bool ComplexMeshRequired = false;
		bool bDrivenByUnreal = false;
		float CoACDThreshold = 0.05f;
		FVector3d Friction{1.0, 1.0, 1.0};
		FVector3d Solref{0.02, 1.0, 0.0};
		FVector3d Solimp{0.9, 0.95, 0.001};
	};

	/** This component's settings, so a promotion can carry them onto an art. */
	FConvertSettings GetConvertSettings() const;

	/**
	 * Author one converted body under `Root` from `SourceActor`'s static meshes.
	 *
	 * A worldbody, the body, its free joint (unless the settings pin it static or
	 * hand it to Unreal as a mocap body) and a geom per exported hull -- the same
	 * `<mesh>`-per-hull assets, the same contact parameters. The nodes attach to
	 * whichever actor the ambient FMjInstanceScope names, so the caller opens the
	 * scope on the actor that is to own the spec: the converted actor itself for a
	 * quick convert, or a freshly spawned articulation for a promotion. Returns
	 * the body element, or null when the actor had nothing convertible on it.
	 */
	static UMjBodyBase* AuthorConvertedBody(
		AActor& SourceActor,
		UMjModel& Root,
		const FConvertSettings& Settings,
		TArray<TObjectPtr<UMjNodeComponent>>& OutGeoms);

	// --- Scene contribution -------------------------------------------------- //

	/**
	 * Convert the owning actor's static meshes into a MuJoCo spec.
	 *
	 * Idempotent: the previous conversion's elements are destroyed first, so a
	 * recompile leaves one body rather than a stack of them. The OBJ export
	 * underneath is content-hashed, so re-authoring an unchanged actor costs a
	 * hash and no geometry work.
	 */
	virtual void AuthorSceneSpec() override;
	virtual FSpecRef GetSceneSpec() const override;
	virtual FString GetScenePrefix() const override;
	virtual FTransform GetScenePlacement() const override;
	virtual void OnSceneBound() override;

	/** The converted body element, once one has been authored. */
	UMjNodeComponent* GetBodyElement() const;

	/** The collision and visual geoms the conversion produced, in spec order. */
	const TArray<TObjectPtr<UMjNodeComponent>>& GetGeomElements() const { return m_GeomElements; }

	/** Pushes the actor's transform into the engine when Unreal drives it. */
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/**
	 * Hide (or show) the actor's own source meshes in the game world.
	 *
	 * The compiled render view draws the converted prop from the model, so the
	 * actor's authored StaticMeshes must not draw beside it at play. The spec's
	 * preview parts are left alone -- the view never draws them.
	 */
	void SetSourceMeshesHiddenInGame(bool bHidden);

protected:
	virtual void BeginPlay() override;

	/**
	 * Take the converted spec with the component.
	 *
	 * The conversion's elements are components of the owning actor, not of this
	 * one, so removing quick convert would otherwise leave a converted body on an
	 * actor that no longer converts. Covers every removal path -- the editor's
	 * delete, the remove RPC, and the actor going away -- because it is the one
	 * point they all pass through.
	 */
	virtual void OnComponentDestroyed(bool bDestroyingHierarchy) override;

	void DrawDebugCollision();

private:
	/** Create the spec root on the owning actor if it does not have one. */
	UMjModel* EnsureSpecRoot();

	/**
	 * The spec root, held on the owning actor.
	 *
	 * Attached under whatever the actor's root already is rather than becoming
	 * it: quick convert is added to somebody else's actor and must not rearrange
	 * its component hierarchy.
	 */
	UPROPERTY()
	TObjectPtr<UMjModel> Spec;

	/** The body this component converted its actor into, once one exists. */
	UPROPERTY(Transient)
	TObjectPtr<UMjBodyBase> m_CreatedBody;

	/** The geoms of that body, in the order they were authored. */
	UPROPERTY(Transient)
	TArray<TObjectPtr<UMjNodeComponent>> m_GeomElements;

	FString m_BodyName;
};
