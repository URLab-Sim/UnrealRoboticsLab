// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Spec/MjSceneContributor.h"

#include "MuJoCo/Spec/MjNodeComponent.h"

namespace
{
/**
 * Everything under `Parent`, elements and their previews alike.
 *
 * Depth first, and not restricted to MuJoCo nodes: `DestroyComponent` detaches
 * a component's children rather than taking them with it, so a geom destroyed on
 * its own leaves the static mesh components it previews itself with loose on the
 * actor. One set of those per compile is a leak, and the next pass converting
 * them back into geometry is worse than a leak.
 */
void DestroySubtree(USceneComponent& Parent)
{
	// A copy: DestroyComponent detaches, which mutates the list being walked.
	TArray<USceneComponent*> Children = Parent.GetAttachChildren();
	for (USceneComponent* Child : Children)
	{
		if (Child != nullptr)
		{
			DestroySubtree(*Child);
			Child->DestroyComponent();
		}
	}
}
} // namespace

void MjDestroySpecChildren(UMjNodeComponent& Root)
{
	DestroySubtree(Root);
}
