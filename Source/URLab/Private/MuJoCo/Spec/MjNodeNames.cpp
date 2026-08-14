// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MjNodeNames.h"

#if URLAB_MJ_GEN

#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#include "Misc/Guid.h"
#include "UObject/UObjectGlobals.h"

#include "MuJoCo/Spec/MjSpecProfile.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjTreeAdapters.h"
#include "MuJoCo/Gen/Elements/MjModel.gen.h"
#include "MuJoCo/Gen/MjDispatch.gen.h"

namespace urlab::spec
{
namespace
{

/**
 * One node, the name it should end up with, and the name to try if some object
 * outside this plan is already holding that one.
 *
 * The fallback is empty unless the planned name is the element's own MJCF name
 * unqualified, which is the only case where a type suffix reads as an answer
 * rather than as noise.
 */
struct FNamePlan
{
	UMjNodeComponent* Node = nullptr;
	FString Name;
	FString Fallback;
};

/**
 * `Name` reduced to what Unreal accepts as a component variable name.
 *
 * MJCF is looser: a menagerie model namespaces its elements with `/`, and a
 * name may begin with a digit. Neither survives as an identifier, so both are
 * folded rather than allowed to produce a component nobody can reference.
 */
FString SanitizeName(const FString& Name)
{
	FString Out;
	Out.Reserve(Name.Len() + 1);
	for (const TCHAR Character : Name)
	{
		const bool bAllowed = (Character >= TEXT('A') && Character <= TEXT('Z')) || (Character >= TEXT('a') && Character <= TEXT('z')) || (Character >= TEXT('0') && Character <= TEXT('9')) || Character == TEXT('_');
		Out.AppendChar(bAllowed ? Character : TEXT('_'));
	}
	if (Out.IsEmpty())
	{
		return FString();
	}
	if (Out[0] >= TEXT('0') && Out[0] <= TEXT('9'))
	{
		Out.InsertAt(0, TEXT('_'));
	}
	return Out;
}

/**
 * What the spec root should be called.
 *
 * The root has no `name` attribute -- it is the spec, not an element, and
 * nothing can refer to it -- but it is not anonymous: `<mujoco model="Humanoid">`
 * is the model's own label, and the profile already reads it as the spec's
 * name. Empty when the MJCF omitted it, which leaves the caller on the tag.
 */
FString RootLabel(const UMjNodeComponent& Node)
{
	if (const UMjModel* Model = Cast<UMjModel>(&Node))
	{
		if (const std::optional<FStringView> Name = FMjInstanceProfile::Doc::Name(*Model))
		{
			return FString(*Name);
		}
	}
	return FString();
}

/** The tag the writer would spell this node with, in the parent it sits under. */
FString ContextualTag(const UMjNodeComponent& Node, const UMjNodeComponent* Parent)
{
	psm::ElementType Type;
	if (!gen::ElementTypeOfNode(Node, Type))
	{
		return FString();
	}
	psm::ElementType ParentType;
	if (Parent != nullptr && gen::ElementTypeOfNode(*Parent, ParentType))
	{
		return gen::ChildTagFor(ParentType, Type);
	}
	return gen::TagForElement(Type);
}

// --- The two graphs ------------------------------------------------------- //
// Each supplies the rename and the question "is the object name this rename
// would claim free". They differ because an SCS rename claims the variable name
// AND the template's object name, which carries Unreal's own suffix.
//
// Both ask before renaming rather than renaming and coping, because Unreal
// treats a rename onto a live object as a fatal error, not a failure: there is
// no version of this that recovers afterwards. The outer is shared with objects
// this spec does not own -- an earlier import into the same Blueprint
// leaves its templates there until they are collected -- so "free among the
// nodes I am renaming" is not the same question as "free".

/** The object name the rename will claim, which is not always the given one. */
FString ClaimedName(FMjInstanceAdapter, const FString& Name)
{
	return Name;
}

void RenameNode(FMjInstanceAdapter, UMjNodeComponent& Node, const FString& Name)
{
	Node.Rename(*Name, nullptr, REN_DontCreateRedirectors | REN_DoNotDirty | REN_NonTransactional);
}

#if WITH_EDITOR
FString ClaimedName(FMjScsAdapter, const FString& Name)
{
	return Name + USimpleConstructionScript::ComponentTemplateNameSuffix;
}

void RenameNode(FMjScsAdapter, UMjNodeComponent& Node, const FString& Name)
{
	FMjScsScope* Scope = FMjScsScope::Current();
	if (Scope == nullptr)
	{
		return;
	}
	if (USCS_Node* ScsNode = Scope->FindNode(Node))
	{
		ScsNode->SetVariableName(FName(*Name), /*bRenameTemplate=*/true);
	}
}
#endif

/** Rename only if the name is usable and unclaimed; false leaves the node alone. */
template <class Adapter>
bool TryRename(UMjNodeComponent& Node, const FString& Name)
{
	// An empty name is NAME_None, and every node that asked for one would land
	// on the same object.
	if (Name.IsEmpty() || FName(*Name).IsNone())
	{
		return false;
	}
	if (StaticFindObjectFast(nullptr, Node.GetOuter(), FName(*ClaimedName(Adapter{}, Name))) != nullptr)
	{
		return false;
	}
	RenameNode(Adapter{}, Node, Name);
	return true;
}

/**
 * Take `Preferred`, or `Fallback`, or the first free `Preferred_N`, or leave the
 * node as it is.
 *
 * `Fallback` is the type-suffixed spelling, tried before any ordinal for the
 * same reason the plan prefers it: `torso_geom` says what the element is,
 * `torso_1` says only that something else got there first.
 *
 * Giving up is a real outcome: a node keeping the ordinal it was constructed
 * with is a worse name, not a broken asset.
 */
template <class Adapter>
void RenameToBest(UMjNodeComponent& Node, const FString& Preferred, const FString& Fallback)
{
	if (TryRename<Adapter>(Node, Preferred))
	{
		return;
	}
	if (!Fallback.IsEmpty() && TryRename<Adapter>(Node, Fallback))
	{
		return;
	}
	for (int32 Ordinal = 1; Ordinal <= 64; ++Ordinal)
	{
		if (TryRename<Adapter>(Node, FString::Printf(TEXT("%s_%d"), *Preferred, Ordinal)))
		{
			return;
		}
	}
}

/**
 * Walk in spec order, recording the name each node should take.
 *
 * Spec order is what makes the result stable and lets the FIRST claimant of
 * a shared name keep it unsuffixed -- the body `torso` before the geom `torso`,
 * which is the order the MJCF reads in.
 *
 * The SECOND claimant is disambiguated by what it is rather than by how many
 * came before it. MuJoCo names elements uniquely within a type and not across
 * types, so two NAMED elements can only ever collide across types -- and the
 * type is therefore always the true distinction between them. `torso_geom` and
 * `torso_joint` say which is which; `torso_1` and `torso_2` say only what order
 * the file happened to be in. Ordinals remain for unnamed elements, which have
 * nothing but their tag to be told apart by, and as the last resort for an
 * authored name that collides with a type-suffixed one.
 */
template <class Adapter>
void PlanSubtree(UMjNodeComponent& Node, const UMjNodeComponent* Parent, TSet<FString>& Taken,
	TArray<FNamePlan>& Plan)
{
	// The root carries no MJCF `name` -- nothing can refer to it -- so its label
	// is the spec's own, `<mujoco model=...>`. It is planned like everything
	// else rather than skipped: left alone it keeps the ordinal
	// `MakeUniqueObjectName` appended when it was constructed, and `mujoco_0` is
	// the one name in the tree that names nothing at all.
	FString Desired;
	if (Parent == nullptr)
	{
		Desired = SanitizeName(RootLabel(Node));
	}
	else if (Node.MjName.IsSet())
	{
		Desired = SanitizeName(Node.MjName.GetValue());
	}
	const bool bNamed = !Desired.IsEmpty();
	if (Desired.IsEmpty())
	{
		Desired = SanitizeName(ContextualTag(Node, Parent));
	}
	if (!Desired.IsEmpty())
	{
		const FString Tag = bNamed ? SanitizeName(ContextualTag(Node, Parent)) : FString();
		const FString Tagged = Tag.IsEmpty() ? FString() : Desired + TEXT("_") + Tag;

		FString Unique = Desired;
		if (Taken.Contains(Unique) && !Tagged.IsEmpty() && !Taken.Contains(Tagged))
		{
			Unique = Tagged;
		}
		for (int32 Ordinal = 1; Taken.Contains(Unique); ++Ordinal)
		{
			Unique = FString::Printf(TEXT("%s_%d"), *Desired, Ordinal);
		}
		Taken.Add(Unique);
		const FString Fallback = Unique == Desired ? Tagged : FString();
		Plan.Add(FNamePlan{&Node, MoveTemp(Unique), Fallback});
	}

	for (const FMjOrderedChild& Child : Adapter::OrderedChildren(Node))
	{
		PlanSubtree<Adapter>(*Child.Node, &Node, Taken, Plan);
	}
}

} // namespace

template <class Adapter>
void NameTreeFromSpec(UMjNodeComponent& Root)
{
	TArray<FNamePlan> Plan;
	TSet<FString> Taken;
	PlanSubtree<Adapter>(Root, nullptr, Taken, Plan);

	// Park every node on a name nothing can hold before handing out the real
	// ones. A single pass walks into names it has not reassigned yet -- renaming
	// the texture that happens to be `texture_1` onto `body_1` while the body
	// still holds `body_1` -- and Unreal treats a rename onto a live object as
	// fatal rather than as a conflict to resolve.
	const FString Batch = FGuid::NewGuid().ToString(EGuidFormats::Digits);
	for (int32 Index = 0; Index < Plan.Num(); ++Index)
	{
		TryRename<Adapter>(*Plan[Index].Node, FString::Printf(TEXT("MjNaming_%s_%d"), *Batch, Index));
	}

	for (const FNamePlan& Entry : Plan)
	{
		RenameToBest<Adapter>(*Entry.Node, Entry.Name, Entry.Fallback);
	}

#if WITH_EDITOR
	if (FMjScsScope* Scope = FMjScsScope::Current())
	{
		Scope->InvalidateNodeMap();
	}
#endif
}

template void NameTreeFromSpec<FMjInstanceAdapter>(UMjNodeComponent&);
#if WITH_EDITOR
template void NameTreeFromSpec<FMjScsAdapter>(UMjNodeComponent&);
#endif

} // namespace urlab::spec

#endif // URLAB_MJ_GEN
