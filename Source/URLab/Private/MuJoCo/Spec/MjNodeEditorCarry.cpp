// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Spec/MjNodeEditorCarry.h"

#if WITH_EDITOR

#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjElementIdentity.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "Logging/MessageLog.h"
#include "Utils/URLabLogging.h"

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

#if URLAB_MJ_GEN
namespace
{
namespace psm = ps::mjcf;

/**
 * Elements already reported, by identity rather than by object.
 *
 * A component is not the same object from one Blueprint reconstruct to the
 * next, and a rule broken once should be said once -- not on every recompile for
 * the rest of the session. `Serial` survives a reconstruct and a genuinely new
 * element mints a fresh one, so it separates "this again" from "another one".
 *
 * Cleared for an element whose placement becomes legal, because putting the same
 * element somewhere illegal a second time is a new mistake.
 */
TSet<uint64> GReportedIllegalPlacement;
} // namespace
#endif // URLAB_MJ_GEN

namespace urlab::spec
{
void MjNodeCheckPlacementLegality(UMjNodeComponent& Node)
{
#if URLAB_MJ_GEN
	Node.PlacementProblems.Reset();

	// A class default object is not placed anywhere, and a template is not
	// registered at all; both reach here only through some other path.
	if (Node.HasAnyFlags(RF_ClassDefaultObject))
	{
		return;
	}

	const UMjNodeComponent* const Parent = Cast<UMjNodeComponent>(Node.GetAttachParent());
	if (Parent == nullptr)
	{
		// An organisational folder or the actor root: not an element, so the
		// schema has nothing to say about the pair. The import pass owns where
		// those go.
		return;
	}

	psm::ElementType ChildType{};
	psm::ElementType ParentType{};
	if (!urlab::spec::MjElementTypeOfNode(Node, ChildType)
		|| !urlab::spec::MjElementTypeOfNode(*Parent, ParentType))
	{
		return;
	}

	if (urlab::spec::gen::SlotFor(ParentType, ChildType) >= 0)
	{
		GReportedIllegalPlacement.Remove(Node.Serial);
		return;
	}

	const FString Message = FString::Printf(
		TEXT("<%s> is not a legal child of <%s>: it will be dropped when the model compiles"),
		urlab::spec::gen::TagForElement(ChildType), urlab::spec::gen::TagForElement(ParentType));
	Node.PlacementProblems.Add(Message);

	// The row above is the persistent surface and is rewritten every time. The
	// two logs are a transition: said when the element becomes illegally placed,
	// not once per registration for the rest of the session.
	if (GReportedIllegalPlacement.Contains(Node.Serial))
	{
		return;
	}
	GReportedIllegalPlacement.Add(Node.Serial);

	const FString Line = FString::Printf(TEXT("%s: %s"), *Node.MjName.Get(Node.GetName()), *Message);
	UE_LOG(LogURLab, Warning, TEXT("%s (parent '%s')"), *Line, *Parent->MjName.Get(Parent->GetName()));
	FMessageLog(TEXT("URLab")).Warning(FText::FromString(Line));
#endif // URLAB_MJ_GEN
}

void MjNodeCarryEditToInstances(UMjNodeComponent& Node, const FSpecRef& Doc, const FPropertyChangedEvent& Event)
{
#if URLAB_MJ_GEN
	FProperty* const Property = Node.PropertyBeforeEdit;
	const FString Before = Node.PropertyTextBeforeEdit;
	Node.PropertyBeforeEdit = nullptr;
	Node.PropertyTextBeforeEdit.Reset();

	if (Property == nullptr || Node.GetOwner() != nullptr)
	{
		return;
	}

	// The snapshot has to belong to the edit that just landed. `PreEditChange`
	// and `PostEditChangeProperty` are not always a matched pair -- a cancelled
	// edit fires only the first -- so a stale snapshot must not be spent on the
	// next property that happens along.
	const FName Name = Property->GetFName();
	if (Event.Property != Property && Event.MemberProperty != Property && Event.GetPropertyName() != Name && Event.GetMemberPropertyName() != Name)
	{
		return;
	}

	FString After;
	Property->ExportTextItem_Direct(After, Property->ContainerPtrToValuePtr<void>(&Node), nullptr, &Node, PPF_None);
	if (After == Before)
	{
		return;
	}

	MjNodeForEachInstanceOfTemplate(Doc, Node, [Property, &Before, &After](UMjNodeComponent& Instance) {
		if (!Instance.IsA(Property->GetOwnerClass()))
		{
			return;
		}
		void* const Slot = Property->ContainerPtrToValuePtr<void>(&Instance);
		FString Held;
		Property->ExportTextItem_Direct(Held, Slot, nullptr, &Instance, PPF_None);
		if (Held != Before)
		{
			// This instance authored its own value: the template is no longer
			// what decides it, and taking that back would discard the user's edit.
			return;
		}

		Instance.Modify();
		Property->ImportText_Direct(*After, Slot, &Instance, PPF_None);

		FPropertyChangedEvent Carried(Property, EPropertyChangeType::ValueSet);
		Instance.PostEditChangeProperty(Carried);
	});
#else
	// Without the generated profile there is no spec to walk and no template
	// graph to find instances in.
	(void)Node;
	(void)Doc;
	(void)Event;
#endif // URLAB_MJ_GEN
}
} // namespace urlab::spec

#endif // WITH_EDITOR
