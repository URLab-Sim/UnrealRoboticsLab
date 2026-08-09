// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Spec/MjSpecRef.h"

#include "Components/ActorComponent.h"
#include "GameFramework/Actor.h"
#include "MuJoCo/Spec/MjNodeComponent.h"

#if URLAB_MJ_GEN
#include "Engine/Blueprint.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"

#include "MjFromtoFold.h"
#include "MjMjcfIoInternal.h"
#include "MjNodeNames.h"
#include "MuJoCo/Spec/MjSpecProfile.h"
#include "MuJoCo/Spec/MjNodeFactories.h"
#include "MuJoCo/Spec/MjTreeAdapters.h"
#endif

FString FMjSpecDiagnostic::ToString() const
{
	if (File.IsEmpty())
	{
		return Message;
	}
	return FString::Printf(TEXT("%s(%d): %s"), *File, Line, *Message);
}

bool FMjSpecParseResult::IsUnsupportedOnly() const
{
	if (Errors.Num() == 0)
	{
		return false;
	}
	for (const FMjSpecDiagnostic& Diagnostic : Errors)
	{
		if (!Diagnostic.bUnsupportedElement)
		{
			return false;
		}
	}
	return true;
}

#if URLAB_MJ_GEN

namespace
{
using namespace urlab::spec;

/**
 * The spec root under an actor: the parentless MuJoCo node with content.
 *
 * An actor can carry more than one candidate. `AMjArticulation` constructs an
 * empty spec as a default subobject so a freshly placed actor has one to
 * author into, and an imported Blueprint's construction script carries the one
 * the reader built -- both are parentless, and taking whichever the component
 * array happens to yield first would compile an empty spec for every
 * imported robot. So the one with children wins, and the empty subobject is the
 * answer only when it is the only answer.
 */
UMjNodeComponent* RootOfActor(AActor& Actor)
{
	if (UMjNodeComponent* Node = Cast<UMjNodeComponent>(Actor.GetRootComponent()))
	{
		return Node;
	}
	TArray<UMjNodeComponent*> Nodes;
	Actor.GetComponents(Nodes);

	UMjNodeComponent* Empty = nullptr;
	for (UMjNodeComponent* Node : Nodes)
	{
		if (Node == nullptr || FMjInstanceAdapter::ParentOf(*Node) != nullptr)
		{
			continue;
		}
		if (FMjInstanceAdapter::RawChildren(*Node).Num() > 0)
		{
			return Node;
		}
		if (Empty == nullptr)
		{
			Empty = Node;
		}
	}
	return Empty;
}

/**
 * Drive every node of a freshly read tree from the spec it came from.
 *
 * Inside the parse rather than at each caller, so an import through any entry
 * point -- the asset action, a test, a scripted op -- spawns posed. For the
 * Blueprint graph this runs before the caller compiles, so the SCS templates
 * carry `RelativeLocation` into instance construction.
 */
template <class Adapter>
void SyncPreviewTree(UMjNodeComponent& Node)
{
	Node.SyncPreviewFromSpec();
	for (const FMjOrderedChild& Child : Adapter::OrderedChildren(Node))
	{
		SyncPreviewTree<Adapter>(*Child.Node);
	}
}

/** The spec root of a Blueprint: the SCS node with no MuJoCo parent. */
UMjNodeComponent* RootOfBlueprint(UBlueprint& Blueprint)
{
	USimpleConstructionScript* Scs = Blueprint.SimpleConstructionScript;
	if (Scs == nullptr)
	{
		return nullptr;
	}
	FMjScsScope Scope(Blueprint);
	for (USCS_Node* Node : Scs->GetRootNodes())
	{
		if (UMjNodeComponent* Template = Cast<UMjNodeComponent>(Node->ComponentTemplate))
		{
			return Template;
		}
	}
	return nullptr;
}
}  // namespace

#endif  // URLAB_MJ_GEN

FSpecRef FSpecRef::OverActor(AActor& InActor)
{
	FSpecRef Out;
#if URLAB_MJ_GEN
	Out.Actor = &InActor;
	Out.Graph = EMjSpecGraph::Instance;
	Out.Root = RootOfActor(InActor);
#else
	(void)InActor;
#endif
	return Out;
}

FSpecRef FSpecRef::OverBlueprint(UBlueprint& InBlueprint)
{
	FSpecRef Out;
#if URLAB_MJ_GEN && WITH_EDITOR
	Out.Blueprint = &InBlueprint;
	Out.Graph = EMjSpecGraph::Scs;
	Out.Root = RootOfBlueprint(InBlueprint);
#else
	(void)InBlueprint;
#endif
	return Out;
}

FSpecRef FSpecRef::OverOwner(const UActorComponent* Component)
{
	FSpecRef Out;
#if URLAB_MJ_GEN
	if (Component == nullptr)
	{
		return Out;
	}
	if (AActor* Owner = Component->GetOwner())
	{
		return OverActor(*Owner);
	}
#if WITH_EDITOR
	// A Blueprint template has no owning actor, and its outer chain reaches the
	// Blueprint by one of two routes: directly, or through the generated class
	// that construction-script templates are outered to. Both are checked,
	// because missing the second one silently degrades every query that needs the
	// rest of the spec -- the reference dropdowns and the effective-value
	// preview both answer "no spec" rather than failing.
	for (UObject* Outer = Component->GetOuter(); Outer != nullptr; Outer = Outer->GetOuter())
	{
		if (UBlueprint* Blueprint = Cast<UBlueprint>(Outer))
		{
			return OverBlueprint(*Blueprint);
		}
		if (const UClass* Generated = Cast<UClass>(Outer))
		{
			if (UBlueprint* Blueprint = Cast<UBlueprint>(Generated->ClassGeneratedBy))
			{
				return OverBlueprint(*Blueprint);
			}
		}
	}
#endif
#else
	(void)Component;
#endif
	return Out;
}

FString FSpecRef::WriteMjcf(TArray<FMjSpecDiagnostic>* OutErrors) const
{
#if URLAB_MJ_GEN
	if (Root == nullptr)
	{
		return FString();
	}
#if WITH_EDITOR
	if (Graph == EMjSpecGraph::Scs)
	{
		if (Blueprint == nullptr)
		{
			return FString();
		}
		urlab::spec::FMjScsScope Scope(*Blueprint);
		return urlab::spec::io::WriteFromScs(*Root, OutErrors);
	}
#endif
	return urlab::spec::io::WriteFromInstance(*Root, OutErrors);
#else
	(void)OutErrors;
	return FString();
#endif
}

FString FSpecRef::WriteMjcfElement(const UMjNodeComponent& Node, TArray<FMjSpecDiagnostic>* OutErrors) const
{
#if URLAB_MJ_GEN
#if WITH_EDITOR
	if (Graph == EMjSpecGraph::Scs)
	{
		if (Blueprint == nullptr)
		{
			return FString();
		}
		urlab::spec::FMjScsScope Scope(*Blueprint);
		return urlab::spec::io::WriteElementFromScs(Node, OutErrors);
	}
#endif
	return urlab::spec::io::WriteElementFromInstance(Node, OutErrors);
#else
	(void)Node;
	(void)OutErrors;
	return FString();
#endif
}

TArray<FString> FSpecRef::NamesOfType(const UClass* Type) const
{
	TArray<FString> Out;
#if URLAB_MJ_GEN
	if (Root == nullptr || Type == nullptr)
	{
		return Out;
	}

	// Walked with the graph's own adapter, so spec order is the order the
	// dropdown offers -- the order the user sees in the components panel.
	auto Gather = [&](UMjNodeComponent& Node, auto& Self) -> void {
		if (Node.IsA(Type) && Node.MjName.IsSet() && !Node.MjName->IsEmpty())
		{
			Out.Add(*Node.MjName);
		}
#if WITH_EDITOR
		if (Graph == EMjSpecGraph::Scs)
		{
			for (const urlab::spec::FMjOrderedChild& Child : urlab::spec::FMjScsAdapter::OrderedChildren(Node))
			{
				Self(*Child.Node, Self);
			}
			return;
		}
#endif
		for (const urlab::spec::FMjOrderedChild& Child : urlab::spec::FMjInstanceAdapter::OrderedChildren(Node))
		{
			Self(*Child.Node, Self);
		}
	};

#if WITH_EDITOR
	if (Graph == EMjSpecGraph::Scs && Blueprint != nullptr)
	{
		urlab::spec::FMjScsScope Scope(*Blueprint);
		Gather(*Root, Gather);
		return Out;
	}
#endif
	Gather(*Root, Gather);
#else
	(void)Type;
#endif
	return Out;
}

#if WITH_EDITOR
FMjSpecParseResult MjParseIntoBlueprint(UBlueprint& Blueprint, const FString& Xml, const FString& Filename,
	const FMjDocParseOptions& Options)
{
	FMjSpecParseResult Out;
#if URLAB_MJ_GEN
	urlab::spec::FMjScsScope Scope(Blueprint);
	Out = urlab::spec::io::ParseIntoScs(Xml, Filename, Options);
	if (Out.Root != nullptr)
	{
		// Before the preview, so a fromto-authored capsule previews at the pose
		// the fold just gave it rather than at the origin.
		urlab::spec::FoldFromtoTree<urlab::spec::FMjScsAdapter>(*Out.Root);
		// After the read, because a node is created before the `name` it should
		// carry has been read into it.
		urlab::spec::NameTreeFromSpec<urlab::spec::FMjScsAdapter>(*Out.Root);
		SyncPreviewTree<urlab::spec::FMjScsAdapter>(*Out.Root);
	}
#else
	(void)Blueprint;
	(void)Xml;
	(void)Filename;
	(void)Options;
	FMjSpecDiagnostic Diagnostic;
	Diagnostic.Message = TEXT("the generated MuJoCo spec profile is not compiled into this build");
	Out.Errors.Add(MoveTemp(Diagnostic));
#endif
	return Out;
}
#endif

FMjSpecParseResult MjParseIntoActor(AActor& Actor, const FString& Xml, const FString& Filename,
	const FMjDocParseOptions& Options)
{
	FMjSpecParseResult Out;
#if URLAB_MJ_GEN
	checkf(IsInGameThread(), TEXT("MjParseIntoActor constructs components, which is game-thread only"));
	urlab::spec::FMjInstanceScope Scope(Actor);
	Out = urlab::spec::io::ParseIntoInstance(Xml, Filename, Options);
	if (Out.Root != nullptr)
	{
		urlab::spec::FoldFromtoTree<urlab::spec::FMjInstanceAdapter>(*Out.Root);
		urlab::spec::NameTreeFromSpec<urlab::spec::FMjInstanceAdapter>(*Out.Root);
		SyncPreviewTree<urlab::spec::FMjInstanceAdapter>(*Out.Root);
	}
#else
	(void)Actor;
	(void)Xml;
	(void)Filename;
	(void)Options;
	FMjSpecDiagnostic Diagnostic;
	Diagnostic.Message = TEXT("the generated MuJoCo spec profile is not compiled into this build");
	Out.Errors.Add(MoveTemp(Diagnostic));
#endif
	return Out;
}
