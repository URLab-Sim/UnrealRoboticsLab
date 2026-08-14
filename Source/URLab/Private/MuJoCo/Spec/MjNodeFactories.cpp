// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Spec/MjNodeFactories.h"

#if URLAB_MJ_GEN

#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Gen/MjDispatch.gen.h"

namespace urlab::spec
{

namespace
{
thread_local FMjInstanceScope* GCurrentInstanceScope = nullptr;

/**
 * Element type -> the class to build for it.
 *
 * Keyed on the underlying integer rather than the enum, so the table needs no
 * hash for a schema type and no count of how many there are. Sparse by design:
 * a handful of elements carry presentation state and the rest never appear.
 *
 * The values are native UClass objects, which are permanently rooted, so this
 * holds them raw and is not a garbage-collection reference.
 */
TMap<int32, UClass*>& ElementClassOverrides()
{
	static TMap<int32, UClass*> Table;
	return Table;
}
} // namespace

void MjSetElementClass(psm::ElementType Type, UClass* Class)
{
	checkf(Class != nullptr, TEXT("MjSetElementClass needs a class"));

	// A registration that is not a subclass of the generated class would be a
	// silent corruption of every read and write of that element: the reader
	// would hand its fields to an object that does not have them. The check is
	// here rather than at the construction site because this is where the
	// mistake is made.
	UClass* Generated = gen::ClassForElement(Type);
	checkf(Generated != nullptr, TEXT("no generated class for element type %d"), static_cast<int32>(Type));
	checkf(Class->IsChildOf(Generated),
		TEXT("%s cannot stand in for %s: a registered class must derive from the generated one"),
		*Class->GetName(), *Generated->GetName());

	ElementClassOverrides().Add(static_cast<int32>(Type), Class);
}

UClass* MjElementClass(psm::ElementType Type, UClass* Fallback)
{
	if (UClass* const* Found = ElementClassOverrides().Find(static_cast<int32>(Type)))
	{
		return *Found;
	}
	return Fallback;
}

void MjResetElementClasses()
{
	ElementClassOverrides().Reset();
}

TMap<int32, UClass*> MjSnapshotElementClasses()
{
	return ElementClassOverrides();
}

void MjRestoreElementClasses(TMap<int32, UClass*> Snapshot)
{
	ElementClassOverrides() = MoveTemp(Snapshot);
}

FMjInstanceScope::FMjInstanceScope(AActor& InOwner)
	: Owner(&InOwner)
	, Previous(GCurrentInstanceScope)
{
	GCurrentInstanceScope = this;
}

FMjInstanceScope::~FMjInstanceScope()
{
	GCurrentInstanceScope = Previous;
}

FMjInstanceScope* FMjInstanceScope::Current()
{
	return GCurrentInstanceScope;
}

FName MjMakeNodeName(psm::ElementType Type, UObject& Outer)
{
	// The schema's own XML tag, which is what a reader of the components panel
	// expects to see next to a line of the MJCF they imported.
	return MakeUniqueObjectName(&Outer, UObject::StaticClass(), FName(gen::TagForElement(Type)));
}

} // namespace urlab::spec

#endif // URLAB_MJ_GEN
