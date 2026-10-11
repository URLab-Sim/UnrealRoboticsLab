// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Elements/MjActuatorRuntime.h"

#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN
#include "MuJoCo/Gen/MjDispatch.gen.h"
#include "MuJoCo/Gen/MjElements.gen.h"
#endif


#if URLAB_MJ_GEN

namespace
{
using urlab::spec::psm::ElementType;

/**
 * The element types the schema lets `<actuator>` hold.
 *
 * Taken from the grammar rather than listed by hand, so an actuator kind
 * MuJoCo gains is one the library accepts the day the schema does. Built once;
 * the set is fixed for the life of the process.
 */
const TSet<ElementType>& ActuatorKinds()
{
	static const TSet<ElementType> Kinds = [] {
		TSet<ElementType> Out;
		urlab::spec::gen::ChildSlots(static_cast<const UMjActuator*>(nullptr),
			[&Out](int32, auto Tag) {
				using Leaf = typename decltype(Tag)::type;
				Out.Add(urlab::spec::gen::TMjElementType<Leaf>::Value);
			});
		return Out;
	}();
	return Kinds;
}

/** The compiled id `Node` bound to, if it is an actuator that bound at all. */


/** Route a setpoint into the engine's one control buffer, addressed by the owning entity's name. */

} // namespace

bool UMjActuatorRuntime::IsActuator(const UMjNodeComponent* Node)
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
	return ActuatorKinds().Contains(Type);
}


#else // URLAB_MJ_GEN

bool UMjActuatorRuntime::IsActuator(const UMjNodeComponent*)
{
	return false;
}


#endif // URLAB_MJ_GEN
