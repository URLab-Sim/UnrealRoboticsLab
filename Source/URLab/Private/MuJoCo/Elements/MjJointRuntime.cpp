// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Elements/MjJointRuntime.h"

#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN
#include "MuJoCo/Gen/MjDispatch.gen.h"
#endif


#if URLAB_MJ_GEN

namespace
{
using urlab::spec::psm::ElementType;

bool IsElementOf(const UMjNodeComponent* Node, std::initializer_list<ElementType> Kinds)
{
	if (Node == nullptr)
	{
		return false;
	}
	ElementType Type;
	if (!urlab::spec::gen::ElementTypeOfNode(*Node, Type))
	{
		return false;
	}
	for (const ElementType Kind : Kinds)
	{
		if (Kind == Type)
		{
			return true;
		}
	}
	return false;
}


/** A bound joint id, for the reads that index by joint rather than by slot. */


/** A bound joint, with its qpos and dof addresses already resolved. */

} // namespace

bool UMjJointRuntime::IsJoint(const UMjNodeComponent* Node)
{
	return IsElementOf(Node, {ElementType::Joint, ElementType::FreeJoint});
}


bool UMjTendonRuntime::IsTendon(const UMjNodeComponent* Node)
{
	return IsElementOf(Node, {ElementType::Spatial, ElementType::Fixed});
}


#else // URLAB_MJ_GEN

bool UMjJointRuntime::IsJoint(const UMjNodeComponent*)
{
	return false;
}


bool UMjTendonRuntime::IsTendon(const UMjNodeComponent*)
{
	return false;
}


#endif // URLAB_MJ_GEN
