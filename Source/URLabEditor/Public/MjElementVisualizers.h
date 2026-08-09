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

/**
 * Editor drawing for every MuJoCo element that has no mesh to preview with.
 *
 * Joints draw their axis and their range arc, sites their wireframe shape,
 * lights their direction and cone, cameras their frustum, and anything else a
 * small marker so a selected element is at least locatable.
 */
class FMjElementVisualizer : public FComponentVisualizer
{
public:
	// FComponentVisualizer
	virtual void DrawVisualization(const UActorComponent* Component, const FSceneView* View,
		FPrimitiveDrawInterface* PDI) override;

	/** Register (and unregister) against UMjNodeComponent. */
	static void RegisterAll();
	static void UnregisterAll();
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
