// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Spec/MjNodeRefOptions.h"

#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjSpecRef.h"

#if URLAB_MJ_GEN
#include <vector>
THIRD_PARTY_INCLUDES_START
#include "protospec/core.h"
#include "protospec/detail.h"
#include "protospec/model_core.h"
#include "protospec/profile.h"
THIRD_PARTY_INCLUDES_END
#include "MuJoCo/Gen/MjDispatch.gen.h"
#endif

namespace urlab::spec
{
TArray<FString> MjNodeRefNameOptions(const UMjNodeComponent& Node, int32 FieldId)
{
	TArray<FString> Out;
#if URLAB_MJ_GEN
	ps::mjcf::ElementType Self;
	if (!urlab::spec::gen::ElementTypeOfNode(Node, Self))
	{
		return Out;
	}
	const std::vector<ps::mjcf::ElementType> Targets = ps::sdk::detail::RefTargetsAt(Self, FieldId);
	if (Targets.empty())
	{
		return Out;
	}

	// The dispatch-level RefNameOptions walks the attach hierarchy, which SCS
	// templates never have, so an editing Blueprint always sees empty
	// dropdowns. FSpecRef's graph-aware walk serves both the live-actor attach
	// tree and the SCS construction-script tree, so it is used here instead.
	const FSpecRef SpecRef = FSpecRef::OverOwner(&Node);
	for (ps::mjcf::ElementType Target : Targets)
	{
		if (const UClass* TargetClass = urlab::spec::gen::ClassForElement(Target))
		{
			for (const FString& Name : SpecRef.NamesOfType(TargetClass))
			{
				Out.AddUnique(Name);
			}
		}
	}
	Out.Sort();
#else
	(void)Node;
	(void)FieldId;
#endif
	return Out;
}
}
