// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MjSectionFold.h"

#if URLAB_MJ_GEN

#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#include "UObject/Package.h"

#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjNodeFactories.h"
#include "MuJoCo/Spec/MjRefView.h"
#include "MuJoCo/Spec/MjTreeAdapters.h"
#include "MuJoCo/Gen/Elements/MjModel.gen.h"
#include "MuJoCo/Gen/MjDispatch.gen.h"
#include "MuJoCo/Gen/MjStorage.gen.h"
#include "MuJoCo/Gen/MjVisit.gen.h"

#include <type_traits>

namespace urlab::spec
{
namespace
{

/** Retire an emptied section from a live actor's component hierarchy. */
void DropSection(FMjInstanceAdapter, UMjNodeComponent& Node)
{
	// Detaches, unregisters, and drops the actor's instance-component reference.
	Node.DestroyComponent();
}

#if WITH_EDITOR
/**
 * Retire an emptied section from a Blueprint's construction script.
 *
 * `RemoveNode` unlinks the node and takes it out of the script's flat list, so
 * it is called instead of a detach rather than after one. The template holds its
 * object name until the collector takes it, and the naming pass declines any
 * name something else still holds, so it is moved to the transient package too.
 */
void DropSection(FMjScsAdapter, UMjNodeComponent& Node)
{
	FMjScsScope* Scope = FMjScsScope::Current();
	if (Scope == nullptr)
	{
		return;
	}
	USCS_Node* ScsNode = Scope->FindNode(Node);
	if (ScsNode == nullptr)
	{
		return;
	}
	Scope->GetScs().RemoveNode(ScsNode);
	Scope->InvalidateNodeMap();
	Node.Rename(nullptr, GetTransientPackage(), REN_DontCreateRedirectors | REN_DoNotDirty | REN_NonTransactional);
}
#endif // WITH_EDITOR

/**
 * Where each of an element's attributes is stored, indexed by schema field id.
 *
 * A reference arrives from Visit wrapped in a view that dies with the traversal
 * frame, so what is recorded is the storage the view aliases rather than the
 * view.
 */
struct FAttributeSlots
{
	TArray<void*> Slots;

	template <class S>
	void field(int Id, const char*, S& Slot)
	{
		if (Slots.Num() <= Id)
		{
			Slots.SetNumZeroed(Id + 1);
		}
		if constexpr (ref_view_detail::TIsRefView<std::decay_t<S>>::value)
		{
			Slots[Id] = Slot.SlotPtr();
		}
		else
		{
			Slots[Id] = &Slot;
		}
	}
};

/**
 * Write each authored attribute into the slot the same field holds elsewhere.
 *
 * Correct only between two elements of one type, which is what the caller
 * guarantees: field ids and storage types then agree by construction, so the
 * recorded address is of exactly the type being copied out of.
 *
 * Authorship is the profile's question rather than the storage's: an optional
 * slot is authored when it holds a value, a reference when it names something,
 * and a required attribute always -- which is the right answer for a merge, in
 * that a required attribute of the later occurrence is a value it stated.
 */
struct FCopyAuthored
{
	const TArray<void*>* Slots = nullptr;

	template <class S>
	void field(int Id, const char*, S& Slot)
	{
		void* const Target = Slots->IsValidIndex(Id) ? (*Slots)[Id] : nullptr;
		if (Target == nullptr || !gen::FMjShape::IsSet(Slot))
		{
			return;
		}
		if constexpr (ref_view_detail::TIsRefView<std::decay_t<S>>::value)
		{
			*static_cast<typename std::decay_t<S>::SlotStorage*>(Target) = *Slot.SlotPtr();
		}
		else
		{
			*static_cast<std::decay_t<S>*>(Target) = Slot;
		}
	}
};

/**
 * Copy `Source`'s authored attributes onto `Target`, leaving the rest alone.
 *
 * Unauthored is not a value to copy: MuJoCo reads each occurrence of a section
 * into one struct, so an attribute the later occurrence is silent about keeps
 * whatever the earlier one said.
 */
void CopyAuthoredAttributes(UMjNodeComponent& Target, UMjNodeComponent& Source)
{
	gen::DispatchByType(Target, [&](auto& TypedTarget) {
		using E = std::decay_t<decltype(TypedTarget)>;
		E* const TypedSource = Cast<E>(&Source);
		if (TypedSource == nullptr)
		{
			return;
		}
		FAttributeSlots Slots;
		gen::Visit(TypedTarget, Slots);
		FCopyAuthored Copy{&Slots.Slots};
		gen::Visit(*TypedSource, Copy);
	});
}

/** True once any attribute of the visited element is authored. */
struct FAnyAuthored
{
	bool bAny = false;

	template <class S>
	void field(int, const char*, S& Slot)
	{
		bAny = bAny || gen::FMjShape::IsSet(Slot);
	}
};

/** Merge every element of `Group` after the first into `Group[0]`, then drop them. */
template <class Adapter>
void FoldGroup(TArray<UMjNodeComponent*>& Group)
{
	UMjNodeComponent& Survivor = *Group[0];
	for (int32 Index = 1; Index < Group.Num(); ++Index)
	{
		UMjNodeComponent& Duplicate = *Group[Index];

		// Later wins, which is what reading each occurrence into one struct in
		// document order amounts to.
		CopyAuthoredAttributes(Survivor, Duplicate);

		// Adopting at the end of the child's own storage slot: the first
		// occurrence's children keep the lower ids, and the later occurrence's
		// follow in the order it declared them.
		for (const FMjOrderedChild& Child : Adapter::OrderedChildren(Duplicate))
		{
			Adapter::Detach(*Child.Node);
			Adapter::template Adopt<UMjNodeComponent>(Survivor, pssdk::kAppend, Child.Node);
		}
		DropSection(Adapter{}, Duplicate);
	}
}

/** `Parent`'s children of exactly one storage slot's type, in document order. */
template <class Adapter, class T>
TArray<UMjNodeComponent*> ChildrenOfType(UMjNodeComponent& Parent)
{
	TArray<UMjNodeComponent*> Out;
	for (const FMjOrderedChild& Child : Adapter::OrderedChildren(Parent))
	{
		if (Cast<T>(Child.Node) != nullptr)
		{
			Out.Add(Child.Node);
		}
	}
	return Out;
}

/**
 * Fold every repeated child of `Node`, and of everything beneath it.
 *
 * Only ever called inside a settings section, whose subtree is blocks the schema
 * admits at most once each: merging two `<option>` sections leaves the survivor
 * holding two `<flag>` children, and one `<flag>` is all MuJoCo has. Nothing
 * else may be walked this way -- two `<geom>` under a body are two geoms.
 */
template <class Adapter>
void FoldRepeatedBlocks(UMjNodeComponent& Node)
{
	gen::DispatchByType(Node, [&](auto& Element) {
		Adapter::ForEachChildSlot(Element, [&](int, auto Tag) {
			using T = typename decltype(Tag)::type;
			TArray<UMjNodeComponent*> Group = ChildrenOfType<Adapter, T>(Node);
			if (Group.Num() == 0)
			{
				return;
			}
			if (Group.Num() > 1)
			{
				FoldGroup<Adapter>(Group);
			}
			FoldRepeatedBlocks<Adapter>(*Group[0]);
		});
	});
	Adapter::Renumber(Node);
}

/**
 * The sections that carry attributes rather than a population.
 *
 * Their subtrees are singleton blocks -- `<option>` has one `<flag>`,
 * `<compiler>` one `<lengthrange>`, `<visual>` one of each of its six -- so a
 * merge can leave a repeat one level down, and folding it is meaning-preserving.
 * The container sections hold populations, where a repeat is the population.
 */
bool IsSettingsSection(psm::ElementType Type)
{
	return Type == psm::ElementType::Compiler || Type == psm::ElementType::Option
		|| Type == psm::ElementType::Size || Type == psm::ElementType::Statistic
		|| Type == psm::ElementType::Visual;
}

} // namespace

bool MjAuthorsAnything(const FSpecRef& Spec, UMjNodeComponent& Node)
{
	if (MjOrderedChildrenOf(Spec, Node).Num() > 0)
	{
		return true;
	}
	FAnyAuthored Authored;
	gen::DispatchByType(static_cast<const UMjNodeComponent&>(Node),
		[&Authored](const auto& Element) { gen::Visit(Element, Authored); });
	return Authored.bAny;
}

template <class Adapter, class Factory>
void NormalizeModelSections(UMjNodeComponent& Root)
{
	UMjModel* const Model = Cast<UMjModel>(&Root);
	if (Model == nullptr)
	{
		return;
	}

	// Driven by the schema's own child slots, so a section added to `<mujoco>`
	// upstream is folded and created without this pass being told about it.
	Adapter::ForEachChildSlot(*Model, [&](int, auto Tag) {
		using T = typename decltype(Tag)::type;
		TArray<UMjNodeComponent*> Group = ChildrenOfType<Adapter, T>(*Model);
		if (Group.Num() == 0)
		{
			Factory::template Create<T>(*Model);
			return;
		}
		if (Group.Num() > 1)
		{
			FoldGroup<Adapter>(Group);
		}
		psm::ElementType Type{};
		if (gen::ElementTypeOfNode(*Group[0], Type) && IsSettingsSection(Type))
		{
			FoldRepeatedBlocks<Adapter>(*Group[0]);
		}
	});
	Adapter::Renumber(*Model);
}

template void NormalizeModelSections<FMjInstanceAdapter, FInstanceNodeFactory>(UMjNodeComponent&);
#if WITH_EDITOR
template void NormalizeModelSections<FMjScsAdapter, FScsNodeFactory>(UMjNodeComponent&);
#endif

} // namespace urlab::spec

#endif // URLAB_MJ_GEN
