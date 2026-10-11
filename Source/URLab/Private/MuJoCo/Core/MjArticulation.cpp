// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Core/MjArticulation.h"

#include "Components/SceneComponent.h"
#include "Engine/Blueprint.h"

#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Spec/MjEffective.h"
#include "MuJoCo/Spec/MjElementIdentity.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Elements/MjBody.h"
#include "MuJoCo/Elements/MjGeom.h"
#include "MuJoCo/Gen/Elements/Joints/MjJoint.gen.h"
#include "MuJoCo/Gen/Elements/Keyframes/MjKey.gen.h"
#include "MuJoCo/Gen/Elements/MjModel.gen.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace
{
/**
 * Every spec element on `Actor`, in component order.
 *
 * Whole-actor rather than tree-walked: an element that is not attached under
 * the spec root is still an element the reader made, and the failure mode
 * of missing one is silent.
 */
TArray<UMjNodeComponent*> SpecNodes(const AActor& Actor)
{
	TArray<UMjNodeComponent*> Nodes;
	Actor.GetComponents(Nodes);
	return Nodes;
}

#if URLAB_MJ_GEN

using urlab::spec::psm::ElementType;

/**
 * The elements of `Actor` that are `Wanted`, class partials excluded.
 *
 * This is where the debug draws and the visibility toggles meet the authoring
 * tree rather than the compiled model, and a `<default>` partial is in that tree
 * without being an element. The compiler emits none for it, so a walk over bound
 * ids cannot reach one and neither may this -- or the joint axes and site
 * crosses of the model's classes get drawn at the origin beside its own.
 */
TArray<UMjNodeComponent*> NodesOfType(const AActor& Actor, ElementType Wanted)
{
	TArray<UMjNodeComponent*> Out;
	for (UMjNodeComponent* Node : SpecNodes(Actor))
	{
		ElementType Type;
		if (Node != nullptr && urlab::spec::MjElementTypeOfNode(*Node, Type) && Type == Wanted && !Node->IsClassPartial())
		{
			Out.Add(Node);
		}
	}
	return Out;
}

#endif // URLAB_MJ_GEN

/** The names an element answers to: its authored MJCF name and its own. */
void EachNameOf(const UMjNodeComponent& Node, TFunctionRef<void(const FString&)> Visit)
{
	const FString ComponentName = Node.GetName();
	if (!ComponentName.IsEmpty())
	{
		Visit(ComponentName);
	}
	if (Node.MjName.IsSet() && !Node.MjName.GetValue().IsEmpty() && Node.MjName.GetValue() != ComponentName)
	{
		Visit(Node.MjName.GetValue());
	}
}

} // namespace

AMjArticulation::AMjArticulation()
{

	DefaultSceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("ArticulationRoot"));
	RootComponent = DefaultSceneRoot;

	// The articulation is one MuJoCo spec and this is its root. It starts
	// empty: an <option> here would be a section of every articulation's
	// spec that nobody authored, and the scene's option is the manager's.
	Spec = CreateDefaultSubobject<UMjModel>(TEXT("Spec"));
	Spec->SetupAttachment(DefaultSceneRoot);
}

FSpecRef AMjArticulation::GetSpec() const
{
	return FSpecRef::OverActor(const_cast<AMjArticulation&>(*this));
}

FString AMjArticulation::GetCompiledPrefix() const
{
	return GetName() + TEXT("_");
}


// --- Lifecycle -------------------------------------------------------------- //

void AMjArticulation::BeginPlay()
{
	Super::BeginPlay();
	UpdateGroup3Visibility();
}

// --- Element index ---------------------------------------------------------- //

void AMjArticulation::IndexBoundElement(UMjNodeComponent& Node, int32 ObjType, int32 Id)
{
	FMjElementFamily& Family = ElementIndex.FindOrAdd(ObjType);
	Family.ById.Add(Id, &Node);
	EachNameOf(Node, [&Family, &Node](const FString& Name) { Family.ByName.Add(Name, &Node); });
}

void AMjArticulation::ClearElementIndex()
{
	ElementIndex.Reset();
}

UMjNodeComponent* AMjArticulation::GetComponentByMjId(int32 ObjType, int32 Id) const
{
	if (const FMjElementFamily* Family = ElementIndex.Find(ObjType))
	{
		if (const TObjectPtr<UMjNodeComponent>* Found = Family->ById.Find(Id))
		{
			return *Found;
		}
	}
	return nullptr;
}

UMjNodeComponent* AMjArticulation::GetComponentByName(int32 ObjType, const FString& Name) const
{
	if (const FMjElementFamily* Family = ElementIndex.Find(ObjType))
	{
		if (const TObjectPtr<UMjNodeComponent>* Found = Family->ByName.Find(Name))
		{
			return *Found;
		}
	}
	return nullptr;
}

TArray<UMjNodeComponent*> AMjArticulation::GetComponentsOfFamily(int32 ObjType) const
{
	TArray<UMjNodeComponent*> Out;
	if (const FMjElementFamily* Family = ElementIndex.Find(ObjType))
	{
		Out.Reserve(Family->ById.Num());
		for (const TPair<int32, TObjectPtr<UMjNodeComponent>>& Entry : Family->ById)
		{
			if (Entry.Value != nullptr)
			{
				Out.Add(Entry.Value);
			}
		}
	}
	return Out;
}

// --- Runtime discovery ------------------------------------------------------ //

namespace
{
/** The MJCF names of an index family, or the component names where unnamed. */
TArray<FString> FamilyNames(const TArray<UMjNodeComponent*>& Nodes)
{
	TArray<FString> Names;
	Names.Reserve(Nodes.Num());
	for (const UMjNodeComponent* Node : Nodes)
	{
		if (Node == nullptr)
		{
			continue;
		}
		Names.Add(Node->MjName.IsSet() && !Node->MjName.GetValue().IsEmpty()
					  ? Node->MjName.GetValue()
					  : Node->GetName());
	}
	return Names;
}
} // namespace

TArray<UMjNodeComponent*> AMjArticulation::GetActuators() const
{
	return GetComponentsOfFamily(mjOBJ_ACTUATOR);
}

TArray<UMjNodeComponent*> AMjArticulation::GetJoints() const
{
	return GetComponentsOfFamily(mjOBJ_JOINT);
}

TArray<UMjNodeComponent*> AMjArticulation::GetSensors() const
{
	return GetComponentsOfFamily(mjOBJ_SENSOR);
}

TArray<UMjNodeComponent*> AMjArticulation::GetTendons() const
{
	return GetComponentsOfFamily(mjOBJ_TENDON);
}

TArray<UMjBody*> AMjArticulation::GetBodies() const
{
	TArray<UMjBody*> Out;
	for (UMjNodeComponent* Node : GetComponentsOfFamily(mjOBJ_BODY))
	{
		if (UMjBody* Body = Cast<UMjBody>(Node))
		{
			Out.Add(Body);
		}
	}
	return Out;
}

TArray<UMjGeom*> AMjArticulation::GetGeoms() const
{
	TArray<UMjGeom*> Out;
	for (UMjNodeComponent* Node : GetComponentsOfFamily(mjOBJ_GEOM))
	{
		if (UMjGeom* Geom = Cast<UMjGeom>(Node))
		{
			Out.Add(Geom);
		}
	}
	return Out;
}

TArray<FString> AMjArticulation::GetActuatorNames() const
{
	return FamilyNames(GetActuators());
}

TArray<FString> AMjArticulation::GetJointNames() const
{
	return FamilyNames(GetJoints());
}

TArray<FString> AMjArticulation::GetSensorNames() const
{
	return FamilyNames(GetSensors());
}

TArray<FString> AMjArticulation::GetTendonNames() const
{
	return FamilyNames(GetTendons());
}

TArray<FString> AMjArticulation::GetBodyNames() const
{
	return FamilyNames(GetComponentsOfFamily(mjOBJ_BODY));
}

UMjNodeComponent* AMjArticulation::GetActuator(const FString& Name) const
{
	return GetComponentByName(mjOBJ_ACTUATOR, Name);
}

UMjNodeComponent* AMjArticulation::GetJoint(const FString& Name) const
{
	return GetComponentByName(mjOBJ_JOINT, Name);
}

UMjNodeComponent* AMjArticulation::GetSensor(const FString& Name) const
{
	return GetComponentByName(mjOBJ_SENSOR, Name);
}

UMjNodeComponent* AMjArticulation::GetTendon(const FString& Name) const
{
	return GetComponentByName(mjOBJ_TENDON, Name);
}

UMjBody* AMjArticulation::GetBody(const FString& Name) const
{
	return Cast<UMjBody>(GetComponentByName(mjOBJ_BODY, Name));
}

UMjBody* AMjArticulation::GetBodyByMjId(int32 Id) const
{
	return Cast<UMjBody>(GetComponentByMjId(mjOBJ_BODY, Id));
}

UMjGeom* AMjArticulation::GetGeomByMjId(int32 Id) const
{
	return Cast<UMjGeom>(GetComponentByMjId(mjOBJ_GEOM, Id));
}

TArray<UMjNodeComponent*> AMjArticulation::GetElementsByTag(const FString& TagName) const
{
	TArray<UMjNodeComponent*> Out;
#if URLAB_MJ_GEN
	for (UMjNodeComponent* Node : SpecNodes(*this))
	{
		ElementType Type;
		if (Node == nullptr || !urlab::spec::MjElementTypeOfNode(*Node, Type))
		{
			continue;
		}
		if (TagName.Equals(urlab::spec::MjTagOf(Type), ESearchCase::IgnoreCase))
		{
			Out.Add(Node);
		}
	}
#endif
	return Out;
}





// --- Keyframes -------------------------------------------------------------- //

TArray<UMjNodeComponent*> AMjArticulation::GetKeyframes() const
{
	TArray<UMjNodeComponent*> Out;
#if URLAB_MJ_GEN
	Out = NodesOfType(*this, ElementType::Key);
#endif
	return Out;
}

TArray<FString> AMjArticulation::GetKeyframeNames() const
{
	return FamilyNames(GetKeyframes());
}


// --- Convenience one-liners ------------------------------------------------- //






// --- Presentation ----------------------------------------------------------- //






void AMjArticulation::ToggleGroup3Visibility()
{
	bShowGroup3 = !bShowGroup3;
	UpdateGroup3Visibility();
}

void AMjArticulation::UpdateGroup3Visibility()
{
#if URLAB_MJ_GEN
	const TArray<UMjNodeComponent*> Nodes = NodesOfType(*this, ElementType::Geom);
	if (Nodes.Num() == 0)
	{
		return;
	}

	// The effective group, not the authored one. Models put `group` on a
	// <default class="collision"> and never on the geoms themselves -- both
	// cards.xml and Spot do -- so reading only what the geom authored finds
	// nothing to hide and the toggle does nothing at all.
	//
	// One context for the whole pass. It indexes every element and every class
	// to answer a single query, so building one per geom makes hiding a robot's
	// collision shapes quadratic in the size of the robot.
	const bool bResolved = urlab::spec::WithEffectiveDoc(*Nodes[0], [this, &Nodes](auto& Effective) {
		for (UMjNodeComponent* Node : Nodes)
		{
			UMjGeom* Geom = Cast<UMjGeom>(Node);
			if (Geom == nullptr)
			{
				continue;
			}
			int32 Group = Geom->Group.Get(0);
			Effective.ForEachLayer(static_cast<const UMjGeomBase&>(*Geom), [&Group](const auto& Layer) {
				if (!Layer.Group.IsSet())
				{
					return false;
				}
				Group = Layer.Group.GetValue();
				return true;
			});
			if (Group == 3)
			{
				Geom->SetGeomVisibility(bShowGroup3);
			}
		}
	});
	if (bResolved)
	{
		return;
	}

	// No spec to resolve against -- a detached actor, or a tree that has not
	// been built. The authored group is all there is to read.
	for (UMjNodeComponent* Node : Nodes)
	{
		UMjGeom* Geom = Cast<UMjGeom>(Node);
		if (Geom != nullptr && Geom->Group.Get(0) == 3)
		{
			Geom->SetGeomVisibility(bShowGroup3);
		}
	}
#endif
}

#if WITH_EDITOR

void AMjArticulation::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

	const FName PropertyName = PropertyChangedEvent.Property != nullptr
								 ? PropertyChangedEvent.Property->GetFName()
								 : NAME_None;
	if (PropertyName == GET_MEMBER_NAME_CHECKED(AMjArticulation, bShowGroup3))
	{
		UpdateGroup3Visibility();
	}
}

void AMjArticulation::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);
	UpdateGroup3Visibility();
}

void AMjArticulation::PostEditMove(bool bFinished)
{
	// Unreal rebuilds every construction-script component on every delta of a
	// drag, and a robot's spec is hundreds of them, each of which resolves
	// its pose and its shape through the default-class chain as it registers.
	// Nothing this actor's construction script does reads the transform that
	// moved, so the rebuild cannot say anything new until the drag ends -- where
	// it still runs, and where the spec picks the move up as it always did.
	//
	// The flag is put back before returning: the Blueprint editor branches on it
	// too, and it means something different there.
	UBlueprint* Blueprint = Cast<UBlueprint>(GetClass()->ClassGeneratedBy);
	const bool bDefer = !bFinished && Blueprint != nullptr && Blueprint->bRunConstructionScriptOnDrag;
	if (bDefer)
	{
		Blueprint->bRunConstructionScriptOnDrag = false;
	}

	Super::PostEditMove(bFinished);

	if (bDefer)
	{
		Blueprint->bRunConstructionScriptOnDrag = true;
	}
}

#endif // WITH_EDITOR
