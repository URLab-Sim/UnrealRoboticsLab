// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MjSpecNodes.h"

#if URLAB_MJ_GEN

#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjSpecProfile.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Spec/MjTreeAdapters.h"

namespace urlab::spec
{
namespace
{
template <class Adapter>
void CollectNodes(UMjNodeComponent& Node, bool bTopLevel, bool bInDefault, FMjSpecNodes& Out)
{
	Out.Nodes.Add(&Node);
	if (bTopLevel || bInDefault)
	{
		Out.Unnamable.Add(&Node);
	}

	psm::ElementType Type{};
	const bool bDefaultBelow =
		bInDefault || (gen::ElementTypeOfNode(Node, Type) && Type == psm::ElementType::Default);

	for (const FMjOrderedChild& Child : Adapter::OrderedChildren(Node))
	{
		if (Child.Node != nullptr)
		{
			CollectNodes<Adapter>(*Child.Node, false, bDefaultBelow, Out);
		}
	}
}
}  // namespace

FMjSpecNodes MjSpecNodesOf(const FSpecRef& Spec)
{
	FMjSpecNodes Out;
	UMjNodeComponent* Root = Spec.GetRoot();
	if (Root == nullptr)
	{
		return Out;
	}
#if WITH_EDITOR
	if (Spec.GetGraph() == EMjSpecGraph::Scs)
	{
		UBlueprint* Blueprint = Spec.GetBlueprint();
		if (Blueprint == nullptr)
		{
			return Out;
		}
		FMjScsScope Scope(*Blueprint);
		Out.Nodes.Add(Root);
		Out.Unnamable.Add(Root);
		for (const FMjOrderedChild& Child : FMjScsAdapter::OrderedChildren(*Root))
		{
			if (Child.Node != nullptr)
			{
				CollectNodes<FMjScsAdapter>(*Child.Node, true, false, Out);
			}
		}
		return Out;
	}
#endif
	Out.Nodes.Add(Root);
	Out.Unnamable.Add(Root);
	for (const FMjOrderedChild& Child : FMjInstanceAdapter::OrderedChildren(*Root))
	{
		if (Child.Node != nullptr)
		{
			CollectNodes<FMjInstanceAdapter>(*Child.Node, true, false, Out);
		}
	}
	return Out;
}

}  // namespace urlab::spec

#endif  // URLAB_MJ_GEN
