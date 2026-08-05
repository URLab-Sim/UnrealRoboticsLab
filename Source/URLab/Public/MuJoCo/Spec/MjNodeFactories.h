// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// The reader's construction seam, twice.
//
// ProtoSpec's reader creates the spec root and then creates each child under
// a parent AT A POSITION -- construction and linkage in one call, because a
// component profile has no meaningful detached node and two reader sites insert
// other than at the end. The default factory builds through the profile's Ident
// and Tree policies, which is right for owned values and wrong for UObjects:
// where a component is constructed decides its outer, its flags, whether it is a
// Blueprint template or a live instance, and whether it registers with a world.
//
// So each Unreal object graph supplies its own factory. They differ in exactly
// one operation -- Construct -- and share positional linkage with everything
// else through their tree adapter.

#include "CoreMinimal.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN

#include "GameFramework/Actor.h"

#include "MuJoCo/Spec/MjSpecProfile.h"
#include "MuJoCo/Spec/MjTreeAdapters.h"

namespace urlab::spec
{

/**
 * The concrete Unreal class an element type is constructed as.
 *
 * Generated element classes carry schema attributes and nothing else, which is
 * right until an element needs per-instance state the reflection system has to
 * see: a render target, a mesh component reference held against garbage
 * collection, a pivot cache. That state cannot live in a function library and
 * it cannot be written into a generated file, so a hand subclass of the
 * generated class declares it and registers itself here; both factories then
 * build the subclass wherever they would have built the base.
 *
 * It is still the same element. `ElementTypeOfClass` resolves a class to the
 * element its nearest generated base names, so reading, writing, dispatch and
 * binding do not know the difference, and an element with no registration is
 * built as the generated class exactly as before.
 *
 * Registration is process-wide, and every registration must happen before the
 * first spec is read.
 */
URLAB_API void MjSetElementClass(psm::ElementType Type, UClass* Class);

/** The class registered for `Type`, or `Fallback` when nothing is registered. */
URLAB_API UClass* MjElementClass(psm::ElementType Type, UClass* Fallback);

/** Forget every registration. Test seam; not for runtime use. */
URLAB_API void MjResetElementClasses();

/**
 * The whole table, and the way to put it back.
 *
 * A test that registers a stand-in has to leave the process as it found it, and
 * it cannot do that by naming the elements it touched: the registrations it must
 * preserve are the ones module startup made, which is a list that grows without
 * the test knowing. So the seam is snapshot-and-restore rather than
 * save-these-two, and a test cannot go stale against it.
 */
URLAB_API TMap<int32, UClass*> MjSnapshotElementClasses();
URLAB_API void MjRestoreElementClasses(TMap<int32, UClass*> Snapshot);

/** The class to build for `E`: its registration if it has one, else `E` itself. */
template <class E>
UClass* MjClassToConstruct()
{
	return MjElementClass(gen::TMjElementType<E>::Value, E::StaticClass());
}

/** Linkage and root creation, shared; `Derived::Construct<E>` is the difference. */
template <class Profile, class Derived>
struct TMjNodeFactory
{
	using doc_type = pssdk::DocOf<Profile>;

	static doc_type* CreateRoot() { return Derived::template Construct<doc_type>(nullptr); }

	template <class E, class Parent>
	static E& Create(Parent& ParentElement, std::size_t Index = pssdk::kAppend)
	{
		E* Node = Derived::template Construct<E>(&ParentElement);
		return Profile::Tree::template Adopt<E>(ParentElement, Index, Node);
	}
};

#if WITH_EDITOR

/**
 * Elements as Blueprint construction-script templates.
 *
 * The Blueprint comes from the ambient FMjScsScope, for the same reason the SCS
 * tree adapter takes it that way: the reader's factory contract is static, and a
 * component knows its template graph only by being found in one.
 */
struct URLAB_API FScsNodeFactory : TMjNodeFactory<FMjScsProfile, FScsNodeFactory>
{
	/**
	 * Create a template component and its USCS_Node.
	 *
	 * `USimpleConstructionScript::CreateNode` is the only correct way in: it
	 * outers the template to the Blueprint, applies the archetype flags, and
	 * gives the node a unique variable name -- three things a bare NewObject gets
	 * wrong in ways that only surface when the asset is reloaded.
	 */
	template <class E>
	static E* Construct(UMjNodeComponent* ParentElement);
};

#endif  // WITH_EDITOR

/**
 * Elements as live components on a spawned actor.
 *
 * Game thread only: component registration touches the world's scene proxies.
 */
struct URLAB_API FInstanceNodeFactory : TMjNodeFactory<FMjInstanceProfile, FInstanceNodeFactory>
{
	template <class E>
	static E* Construct(UMjNodeComponent* ParentElement);
};

/** The owner every FInstanceNodeFactory construction attaches to. */
struct URLAB_API FMjInstanceScope
{
	explicit FMjInstanceScope(AActor& Owner);
	~FMjInstanceScope();

	FMjInstanceScope(const FMjInstanceScope&) = delete;
	FMjInstanceScope& operator=(const FMjInstanceScope&) = delete;

	static FMjInstanceScope* Current();

	AActor& GetOwner() const { return *Owner; }

private:
	AActor* Owner = nullptr;
	FMjInstanceScope* Previous = nullptr;
};

/**
 * The name a fresh element's Unreal object takes.
 *
 * Not the MJCF name: that is an authored attribute the reader sets afterwards,
 * and most elements have none. This is only the Blueprint variable / component
 * name, so it has to be unique and readable and nothing more. Derived from the
 * element's schema tag, which the generated tables supply.
 */
URLAB_API FName MjMakeNodeName(psm::ElementType Type, UObject& Outer);

// --- Definitions ------------------------------------------------------------ //

#if WITH_EDITOR

template <class E>
E* FScsNodeFactory::Construct(UMjNodeComponent* ParentElement)
{
	FMjScsScope* Scope = FMjScsScope::Current();
	checkf(Scope != nullptr, TEXT("FScsNodeFactory needs an open FMjScsScope"));

	USimpleConstructionScript& Scs = Scope->GetScs();
	USCS_Node* Node = Scs.CreateNode(MjClassToConstruct<E>(), MjMakeNodeName(gen::TMjElementType<E>::Value, Scs));
	checkf(Node != nullptr, TEXT("USimpleConstructionScript::CreateNode returned null"));

	E* Template = CastChecked<E>(Node->ComponentTemplate);
	Template->EnsureSerial();
	Scope->InvalidateNodeMap();

	if (ParentElement == nullptr)
	{
		// The root is the Blueprint's scene root: no parent to adopt it.
		Scs.AddNode(Node);
	}
	return Template;
}

#endif  // WITH_EDITOR

template <class E>
E* FInstanceNodeFactory::Construct(UMjNodeComponent* ParentElement)
{
	FMjInstanceScope* Scope = FMjInstanceScope::Current();
	checkf(Scope != nullptr, TEXT("FInstanceNodeFactory needs an open FMjInstanceScope"));
	checkf(IsInGameThread(), TEXT("FInstanceNodeFactory constructs components, which is game-thread only"));

	AActor& Owner = Scope->GetOwner();
	E* Node = NewObject<E>(&Owner, MjClassToConstruct<E>(),
		MjMakeNodeName(gen::TMjElementType<E>::Value, Owner), RF_Transactional);
	Node->EnsureSerial();

	if (ParentElement == nullptr)
	{
		Owner.SetRootComponent(Node);
	}
	Node->RegisterComponent();
	Owner.AddInstanceComponent(Node);
	return Node;
}

}  // namespace urlab::spec

#endif  // URLAB_MJ_GEN
