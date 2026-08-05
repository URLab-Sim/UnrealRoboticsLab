// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// The Tree policy, twice: over a live actor's attachment hierarchy, and over a
// Blueprint's USCS_Node template graph.
//
// These are the same contract over two object graphs, so the contract is written
// once. TMjTreeAdapter carries every operation ProtoSpec's SDK and io ask of a
// tree -- ordered iteration, slot enumeration, typed queries, positional adopt,
// remove, duplicate, prune, reparent -- in terms of four primitives its derived
// adapter supplies:
//
//   RawChildren(parent)      the parent's children, in whatever order storage has
//   ParentOf(child)          the child's parent, or null
//   Attach(parent, child)    link in the host graph
//   Detach(child)            unlink from the host graph
//
// Everything else -- spec order, slot grouping, sibling renumbering, the
// cycle check, the recursive search -- is graph-independent and lives in the
// base. The two derived adapters are the four primitives and nothing more.

#include "CoreMinimal.h"

#include "MuJoCo/Spec/MjGenHooks.h"
#include "MuJoCo/Spec/MjNodeComponent.h"

#if URLAB_MJ_GEN

#include "Components/SceneComponent.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"

THIRD_PARTY_INCLUDES_START
#include "protospec/profile.h"
#include "types.h"
THIRD_PARTY_INCLUDES_END

struct FSpecRef;

namespace urlab::spec
{

/**
 * One child of a parent, resolved to the storage slot it occupies and the
 * position it holds within that slot.
 *
 * The slot is the schema's child-list index, which is what lets the writer
 * recover the contextual XML tag of a child that several tags can spell.
 */
struct FMjOrderedChild
{
	UMjNodeComponent* Node = nullptr;
	int32 Slot = -1;
};

/**
 * Spec order for a set of siblings.
 *
 * Sorted on (slot, SiblingIndex, Serial). The serial tie-break is not defensive
 * padding: SiblingIndex is authored data and can be duplicated by a hand edit, a
 * merge, or a partially-migrated asset, and a sort that left ties to storage
 * order would be nondeterministic in exactly the case Unreal gives no ordering
 * guarantee for -- a level instance, whose AttachChildren is Transient and
 * rebuilt from registration order. Serials are minted in spec order by the
 * reader, so the tie-break resolves to the original spec order.
 */
URLAB_API void MjSortSpecOrder(TArray<FMjOrderedChild>& Children);

/**
 * `Parent`'s children in whichever graph `Spec` lives in.
 *
 * A Blueprint's templates are linked only by USCS_Node::ChildNodes -- they are
 * never attached to one another -- so reading the attachment tree over an SCS
 * spec walks an empty list and the caller silently finds nothing. Which
 * adapter answers is the spec's business, not the caller's.
 */
URLAB_API TArray<FMjOrderedChild> MjOrderedChildrenOf(const FSpecRef& Spec, UMjNodeComponent& Parent);

/**
 * The contract, once. `Derived` supplies the four host-graph primitives.
 */
template <class Derived>
struct TMjTreeAdapter
{
	/** UObject lifetime is the garbage collector's; the owner handle is a pointer. */
	template <class T>
	using owner = T*;

	// --- Ordered iteration ------------------------------------------------- //

	template <class E, class Fn>
	static void ForEachChild(E& Parent, Fn&& Function)
	{
		for (const FMjOrderedChild& Child : OrderedChildren(Parent))
		{
			DispatchChild<E>(*Child.Node, [&](auto& Concrete) { Function(Concrete); });
		}
	}

	template <class E, class Fn>
	static void ForEachChildAt(E& Parent, Fn&& Function)
	{
		for (const FMjOrderedChild& Child : OrderedChildren(Parent))
		{
			const int Slot = Child.Slot;
			DispatchChild<E>(*Child.Node, [&](auto& Concrete) { Function(Slot, Concrete); });
		}
	}

	/** The (slot, admissible child type) pairs this element's schema declares. */
	template <class E, class Fn>
	static void ForEachChildSlot(E&, Fn&& Function)
	{
		gen::ChildSlots(static_cast<const std::remove_const_t<E>*>(nullptr), Function);
	}

	template <class T, class E, class Fn>
	static void ForEachChildOfType(E& Parent, Fn&& Function)
	{
		for (const FMjOrderedChild& Child : OrderedChildren(Parent))
		{
			T* Typed = Cast<T>(Child.Node);
			if (Typed == nullptr)
			{
				continue;
			}
			if constexpr (std::is_const_v<E>)
			{
				Function(static_cast<const T&>(*Typed));
			}
			else
			{
				Function(*Typed);
			}
		}
	}

	template <class T, class E>
	static auto FirstChildOfType(E& Parent) -> std::conditional_t<std::is_const_v<E>, const T*, T*>
	{
		for (const FMjOrderedChild& Child : OrderedChildren(Parent))
		{
			if (T* Typed = Cast<T>(Child.Node))
			{
				return Typed;
			}
		}
		return nullptr;
	}

	template <class T, class E>
	static void ClearChildrenOfType(E& Parent)
	{
		for (const FMjOrderedChild& Child : OrderedChildren(Parent))
		{
			if (Cast<T>(Child.Node) != nullptr)
			{
				Derived::Detach(*Child.Node);
				Release(*Child.Node);
			}
		}
		Renumber(Parent);
	}

	// --- Structure --------------------------------------------------------- //

	/**
	 * Link an owned child into `Parent` at `Index` among the siblings sharing its
	 * storage slot; kAppend means last.
	 *
	 * The index is storage-relative by contract, not global. That is the only
	 * coordinate in which "insert immediately after this child" means the same
	 * thing here -- where one attachment list holds every child type a parent
	 * admits -- as it does in a profile with one list per type.
	 */
	template <class T, class E>
	static T& Adopt(E& Parent, std::size_t Index, owner<T> Child)
	{
		const int32 Slot = FindSlot(Parent, *Child);

		TArray<UMjNodeComponent*> Peers;
		for (const FMjOrderedChild& Existing : OrderedChildren(Parent))
		{
			if (Existing.Slot == Slot)
			{
				Peers.Add(Existing.Node);
			}
		}

		const int32 At = Index >= static_cast<std::size_t>(Peers.Num()) ? Peers.Num() : static_cast<int32>(Index);
		Peers.Insert(Child, At);

		Derived::Attach(Parent, *Child);
		for (int32 Position = 0; Position < Peers.Num(); ++Position)
		{
			Peers[Position]->SiblingIndex = Position;
		}
		return *Child;
	}

	/**
	 * Detach the element at `Target` from wherever under `Root` holds it.
	 *
	 * The contract promises detachment plus release at an unspecified later
	 * point, so a caller must not dereference `Target` afterwards. Here that
	 * latitude is load-bearing rather than incidental: a UObject is released by
	 * the garbage collector, and an editor undo can bring it back.
	 */
	template <class Root>
	static bool Remove(Root& RootNode, const void* Target)
	{
		UMjNodeComponent* Found = FindNode(RootNode, Target);
		if (Found == nullptr)
		{
			return false;
		}
		UMjNodeComponent* Parent = Derived::ParentOf(*Found);
		Derived::Detach(*Found);
		Release(*Found);
		if (Parent != nullptr)
		{
			Renumber(*Parent);
		}
		return true;
	}

	template <class Root>
	static void* CloneAsNextSibling(Root& RootNode, const void* Target)
	{
		UMjNodeComponent* Found = FindNode(RootNode, Target);
		if (Found == nullptr)
		{
			return nullptr;
		}
		UMjNodeComponent* Parent = Derived::ParentOf(*Found);
		if (Parent == nullptr)
		{
			return nullptr;
		}

		UMjNodeComponent* Clone = nullptr;
		gen::DispatchByType(*Found, [&](auto& Concrete) {
			using T = std::decay_t<decltype(Concrete)>;
			T* Copy = DuplicateObject<T>(&Concrete, Concrete.GetOuter());
			Copy->MintSerial();
			// One past the source, in the source's own slot.
			std::size_t At = pssdk::kAppend;
			int32 Position = 0;
			for (const FMjOrderedChild& Sibling : OrderedChildren(*Parent))
			{
				if (Sibling.Slot != FindSlot(*Parent, *Found))
				{
					continue;
				}
				if (Sibling.Node == Found)
				{
					At = static_cast<std::size_t>(Position + 1);
					break;
				}
				++Position;
			}
			gen::DispatchByType(*Parent, [&](auto& TypedParent) { Adopt<T>(TypedParent, At, Copy); });
			Clone = Copy;
		});
		return Clone;
	}

	template <class Root, class Pred>
	static void PruneIf(Root& RootNode, Pred&& Predicate)
	{
		TArray<UMjNodeComponent*> Doomed;
		Walk(RootNode, [&](UMjNodeComponent& Node) {
			gen::DispatchByType(Node, [&](auto& Concrete) {
				if (Predicate(Concrete))
				{
					Doomed.Add(&Node);
				}
			});
		});
		for (UMjNodeComponent* Node : Doomed)
		{
			UMjNodeComponent* Parent = Derived::ParentOf(*Node);
			Derived::Detach(*Node);
			Release(*Node);
			if (Parent != nullptr)
			{
				Renumber(*Parent);
			}
		}
	}

	/**
	 * Move a body-context child under a new parent, keeping its authored local
	 * pose. `NewParent` null means the world body.
	 */
	template <class Root>
	static pssdk::MoveStatus Reparent(Root& RootNode, const void* Element, const void* NewParent)
	{
		UMjNodeComponent* Node = FindNode(RootNode, Element);
		if (Node == nullptr || Derived::ParentOf(*Node) == nullptr)
		{
			return pssdk::MoveStatus::NotMovable;
		}

		UMjNodeComponent* Target = nullptr;
		if (NewParent == nullptr)
		{
			Target = FirstWorldBody(RootNode);
			if (Target == nullptr)
			{
				return pssdk::MoveStatus::BadTarget;
			}
		}
		else
		{
			Target = FindNode(RootNode, NewParent);
			if (Target == nullptr)
			{
				return pssdk::MoveStatus::BadTarget;
			}
		}

		if (FindSlot(*Target, *Node) < 0)
		{
			return pssdk::MoveStatus::BadTarget;
		}

		bool bCycle = false;
		Walk(*Node, [&](UMjNodeComponent& Descendant) {
			if (&Descendant == Target)
			{
				bCycle = true;
			}
		});
		if (bCycle)
		{
			return pssdk::MoveStatus::Cycle;
		}

		UMjNodeComponent* OldParent = Derived::ParentOf(*Node);
		Derived::Detach(*Node);
		if (OldParent != nullptr)
		{
			Renumber(*OldParent);
		}
		gen::DispatchByType(*Node, [&](auto& Moved) {
			using T = std::decay_t<decltype(Moved)>;
			gen::DispatchByType(
				*Target, [&](auto& TypedTarget) { Adopt<T>(TypedTarget, pssdk::kAppend, &Moved); });
		});
		return pssdk::MoveStatus::Ok;
	}

	// --- Shared mechanics -------------------------------------------------- //

	/** `Parent`'s children in spec order, each tagged with its storage slot. */
	static TArray<FMjOrderedChild> OrderedChildren(const UMjNodeComponent& Parent)
	{
		psm::ElementType ParentType{};
		TArray<FMjOrderedChild> Out;
		if (!gen::ElementTypeOfNode(Parent, ParentType))
		{
			return Out;
		}
		for (UMjNodeComponent* Child : Derived::RawChildren(Parent))
		{
			psm::ElementType ChildType{};
			if (Child == nullptr || !gen::ElementTypeOfNode(*Child, ChildType))
			{
				continue;
			}
			Out.Add(FMjOrderedChild{Child, gen::SlotFor(ParentType, ChildType)});
		}
		MjSortSpecOrder(Out);
		return Out;
	}

	/** Restate spec order as a dense 0..n-1 within each slot. */
	static void Renumber(const UMjNodeComponent& Parent)
	{
		int32 Slot = MIN_int32;
		int32 Position = 0;
		for (const FMjOrderedChild& Child : OrderedChildren(Parent))
		{
			if (Child.Slot != Slot)
			{
				Slot = Child.Slot;
				Position = 0;
			}
			Child.Node->SiblingIndex = Position++;
		}
	}

private:
	template <class E, class Fn>
	static void DispatchChild(UMjNodeComponent& Child, Fn&& Function)
	{
		if constexpr (std::is_const_v<E>)
		{
			gen::DispatchByType(static_cast<const UMjNodeComponent&>(Child), Function);
		}
		else
		{
			gen::DispatchByType(Child, Function);
		}
	}

	static int32 FindSlot(const UMjNodeComponent& Parent, const UMjNodeComponent& Child)
	{
		psm::ElementType ParentType{};
		psm::ElementType ChildType{};
		if (!gen::ElementTypeOfNode(Parent, ParentType) || !gen::ElementTypeOfNode(Child, ChildType))
		{
			return -1;
		}
		return gen::SlotFor(ParentType, ChildType);
	}

	template <class E, class Fn>
	static void Walk(E& Node, Fn&& Function)
	{
		UMjNodeComponent& AsNode = const_cast<UMjNodeComponent&>(static_cast<const UMjNodeComponent&>(Node));
		Function(AsNode);
		for (const FMjOrderedChild& Child : OrderedChildren(AsNode))
		{
			Walk(*Child.Node, Function);
		}
	}

	template <class Root>
	static UMjNodeComponent* FindNode(Root& RootNode, const void* Target)
	{
		UMjNodeComponent* Found = nullptr;
		Walk(RootNode, [&](UMjNodeComponent& Node) {
			if (Found == nullptr && static_cast<const void*>(&Node) == Target)
			{
				Found = &Node;
			}
		});
		return Found;
	}

	template <class Root>
	static UMjNodeComponent* FirstWorldBody(Root& RootNode)
	{
		using BodyType = typename gen::TMjElementOf<psm::ElementType::Body>::Type;
		UMjNodeComponent* Found = nullptr;
		Walk(RootNode, [&](UMjNodeComponent& Node) {
			if (Found == nullptr && Cast<BodyType>(&Node) != nullptr)
			{
				Found = &Node;
			}
		});
		return Found;
	}

	/**
	 * Release a detached node.
	 *
	 * Detachment and release are separate by contract, and Unreal is the reason
	 * the contract is worded that way: marking an object as garbage does not free
	 * it, an open transaction can restore it, and the collector decides when.
	 */
	static void Release(UMjNodeComponent& Node)
	{
		Node.Modify();
		Node.MarkAsGarbage();
	}
};

/**
 * The live half: a spawned actor's attachment hierarchy.
 *
 * `USceneComponent::AttachChildren` is Transient and rebuilt from registration
 * order at load, so it supplies membership but never order. Order comes from the
 * persisted SiblingIndex the base sorts on.
 */
struct URLAB_API FMjInstanceAdapter : TMjTreeAdapter<FMjInstanceAdapter>
{
	static TArray<UMjNodeComponent*> RawChildren(const UMjNodeComponent& Parent);
	static UMjNodeComponent* ParentOf(const UMjNodeComponent& Child);
	static void Attach(UMjNodeComponent& Parent, UMjNodeComponent& Child);
	static void Detach(UMjNodeComponent& Child);
};

#if WITH_EDITOR

/**
 * A Blueprint's construction-script template graph, the ambient one from
 * FMjScsScope.
 *
 * `USCS_Node::ChildNodes` is a serialized ordered array, so this graph does
 * carry an order of its own -- but the two adapters must agree, and only one of
 * them can, so both defer to SiblingIndex and this one keeps ChildNodes in step
 * as a courtesy to the components panel.
 *
 * The scope is ambient because ProtoSpec's Tree policy is a static contract: the
 * SDK calls `P::Tree::Adopt<T>(parent, index, child)` with no adapter instance to
 * carry a Blueprint on. A component knows its template graph only by being found
 * in one, so the graph is supplied for the duration of a read or a write.
 */
struct URLAB_API FMjScsScope
{
	explicit FMjScsScope(UBlueprint& Blueprint);
	~FMjScsScope();

	FMjScsScope(const FMjScsScope&) = delete;
	FMjScsScope& operator=(const FMjScsScope&) = delete;

	/** The innermost open scope, or null. */
	static FMjScsScope* Current();

	UBlueprint& GetBlueprint() const { return *Owner; }
	USimpleConstructionScript& GetScs() const { return *Scs; }

	/** The node holding `Template`, creating one if the template is loose. */
	USCS_Node* NodeFor(UMjNodeComponent& Template);

	/** The node holding `Template`, or null. */
	USCS_Node* FindNode(const UMjNodeComponent& Template) const;

	/** Forget the cached maps; the next lookup rebuilds them. */
	void InvalidateNodeMap();

	/**
	 * The node holding `Child`'s parent, or null at a root.
	 *
	 * From a map rather than a search. `USCS_Node` records its children and not
	 * its parent, so the only way to ask directly is to scan every node in the
	 * Blueprint and test whether `Child` is among its children -- and the parent
	 * of every element is exactly what building an effective context asks for,
	 * so that search made one context cost the square of the spec.
	 */
	USCS_Node* ParentNodeOf(const UMjNodeComponent& Child) const;

private:
	void EnsureNodeMap() const;

	UBlueprint* Owner = nullptr;
	USimpleConstructionScript* Scs = nullptr;
	FMjScsScope* Previous = nullptr;
	mutable TMap<const UMjNodeComponent*, USCS_Node*> NodeMap;
	mutable TMap<const USCS_Node*, USCS_Node*> ParentMap;
	mutable bool bNodeMapValid = false;
};

struct URLAB_API FMjScsAdapter : TMjTreeAdapter<FMjScsAdapter>
{
	static TArray<UMjNodeComponent*> RawChildren(const UMjNodeComponent& Parent);
	static UMjNodeComponent* ParentOf(const UMjNodeComponent& Child);
	static void Attach(UMjNodeComponent& Parent, UMjNodeComponent& Child);
	static void Detach(UMjNodeComponent& Child);
};

#endif  // WITH_EDITOR

}  // namespace urlab::spec

#endif  // URLAB_MJ_GEN
