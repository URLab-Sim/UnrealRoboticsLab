// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// What the rest of a MuJoCo model looks like while it is being edited.
//
// A geom previews as a real mesh because a geom has a shape Unreal can build.
// A joint, a site, a light and a camera have no mesh: what they are is an axis,
// a range, a direction, a field of view. The simulation already draws those
// (`AMjArticulation::DrawDebugJoints`, `DrawDebugSites`) but it reads the
// compiled model, and there is no compiled model in the Blueprint editor -- so
// the editor showed nothing at all for four of the five element families a user
// spends their time placing.
//
// This is that drawing, ported: same shapes, same colours, driven from the
// components and from EFFECTIVE values rather than from `mjModel`. A geom whose
// `size` comes from a `<default class="visual">` previews at the inherited size,
// and a site or a joint has to do the same or the preview lies about what the
// compiler will build.
//
// One `FComponentVisualizer`, registered against `UMjNodeComponent`, serves
// every element: UnrealEd resolves a visualizer by walking up the component's
// class chain, so one registration covers all 145 generated classes and the
// hand subclasses over them. Nothing here creates a component, which is
// deliberate -- an editor-only preview component on a Blueprint template is one
// serialisation mistake away from shipping inside the asset, and a visualizer
// cannot make that mistake because it owns no objects at all.

#include "CoreMinimal.h"

#include "ComponentVisualizer.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN
#include "MuJoCo/Spec/MjEffective.h"
#endif

class UMjModel;
class UMjNodeComponent;

/**
 * Editor drawing for every MuJoCo element that has no mesh to preview with.
 *
 * Joints draw their axis and their range arc, sites their wireframe shape,
 * lights their direction and cone, cameras their frustum, and anything else a
 * small marker so a selected element is at least locatable.
 *
 * The whole selection is drawn element by element, once per frame, and every
 * element reads EFFECTIVE values -- which means resolving the default-class
 * chain, which means indexing the entire spec. Once per element per frame is
 * the quadratic this project has already paid for once, and at editor frame
 * rates it does not read as slowness: the gizmo runs ahead of the component and
 * the drag appears to fight back. So the sweep holds ONE index open: the first
 * element of a frame builds it and every element after joins it.
 */
class FMjElementVisualizer : public FComponentVisualizer
{
public:
	FMjElementVisualizer();
	virtual ~FMjElementVisualizer() override;

	// FComponentVisualizer
	virtual void DrawVisualization(const UActorComponent* Component, const FSceneView* View,
		FPrimitiveDrawInterface* PDI) override;

	/** Register (and unregister) against UMjNodeComponent. */
	static void RegisterAll();
	static void UnregisterAll();

private:
#if URLAB_MJ_GEN
	/**
	 * Open the frame's index over `Element`'s spec, or join the one already open.
	 *
	 * Keyed on the frame number and the spec root, so a new frame and a
	 * selection in a different model each rebuild, and everything else adopts.
	 * Nothing is held across a frame boundary: the index is a snapshot of a tree
	 * that the very next edit may restructure, so it is dropped at end of frame
	 * -- after every viewport has painted and before anything can edit again.
	 */
	void BeginSweep(const UMjNodeComponent& Element);

	/** Drop the frame's index. Bound to end-of-frame, and run by the destructor. */
	void EndSweep();

	/** The index the frame's elements share, when this instance owns it. */
	TUniquePtr<urlab::spec::FMjEffectiveScope> SweepScope;

	/** The frame the current index describes. */
	uint32 SweepFrame = 0;

	/** The spec root it describes, so a second model in the frame rebuilds. */
	TWeakObjectPtr<const UMjModel> SweepRoot;

	/**
	 * Radians per authored angle unit for `SweepRoot`, resolved once per frame.
	 *
	 * `<compiler angle>` is a property of the document, and every joint drawn
	 * was walking the whole spec to find it -- a second whole-graph walk per
	 * hinge per frame, on top of the index.
	 */
	TOptional<double> SweepRadiansPerAngle;

	FDelegateHandle EndFrameHandle;
#endif  // URLAB_MJ_GEN

	/**
	 * Set while the registration is waiting for the editor engine.
	 *
	 * The registry belongs to `GUnrealEd`, and this module can load before that
	 * exists; registering then is not an error, it is a no-op, so the whole
	 * feature would be off with nothing said.
	 */
	static FDelegateHandle PostEngineInitHandle;
};

/**
 * The add-time legality warning.
 *
 * MuJoCo's schema says which children an element admits, and a component
 * dropped somewhere it is not admitted does not fail when it is dropped: it
 * fails at compile, in play, as a message about an element the user placed
 * minutes earlier. This watches the Blueprint's construction script and says so
 * at the moment of the drop instead.
 *
 * It deliberately does NOT filter the Add Component picker. A filter that is
 * wrong hides a legal child with no way to find out why, which is a worse
 * failure than accepting the drop and explaining it.
 */
class FMjAddTimeLegality
{
public:
	static void RegisterAll();
	static void UnregisterAll();
};
