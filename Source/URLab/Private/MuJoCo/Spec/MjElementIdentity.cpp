// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Spec/MjElementIdentity.h"

#if URLAB_MJ_GEN

#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Gen/MjDispatch.gen.h"

namespace urlab::spec
{

bool MjElementTypeOfNode(const UMjNodeComponent& Node, psm::ElementType& Out)
{
	return gen::ElementTypeOfNode(Node, Out);
}

bool MjElementTypeOfClass(const UClass* Class, psm::ElementType& Out)
{
	return gen::ElementTypeOfClass(Class, Out);
}

UClass* MjGeneratedClassOf(psm::ElementType Type)
{
	return gen::ClassForElement(Type);
}

const TCHAR* MjTagOf(psm::ElementType Type)
{
	return gen::TagForElement(Type);
}

}  // namespace urlab::spec

#endif  // URLAB_MJ_GEN
