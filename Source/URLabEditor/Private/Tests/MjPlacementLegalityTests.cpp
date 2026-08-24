// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// Adding a component somewhere the schema does not admit it.
//
// MJCF says which elements may contain which. A component parented against that
// is not refused: the element is simply absent from the compiled model, and the
// model still compiles, so the first sign of it is a robot missing a part.
//
// The check runs on `OnRegister`, which both ways a user adds a component --
// the Blueprint construction script and the level editor's Add Component on a
// placed actor -- reach, and what it produces is a row on the component rather
// than a toast that is gone before the next add.
//
// This drives the level-editor path -- create, attach, register -- because that
// is the one that had no coverage at all.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "IMessageLogListing.h"
#include "MessageLogModule.h"
#include "Misc/Guid.h"
#include "Modules/ModuleManager.h"

#include "MuJoCo/Elements/MjBody.h"
#include "MuJoCo/Elements/MjGeom.h"
#include "MuJoCo/Gen/Elements/Joints/MjJoint.gen.h"
#include "MuJoCo/Spec/MjNodeComponent.h"

namespace MjPlacementLegalityTests
{

/** A world and an actor to hang components on, torn down with the test. */
struct FScratchActor
{
	UWorld* World = nullptr;
	AActor* Actor = nullptr;

	bool Open(FAutomationTestBase& Test)
	{
		const FString Stem = FString::Printf(TEXT("MjPlace_%s"), *FGuid::NewGuid().ToString(EGuidFormats::Digits));
		World = UWorld::CreateWorld(EWorldType::Editor, /*bInformEngineOfWorld=*/false, FName(*Stem));
		if (World == nullptr)
		{
			Test.AddError(TEXT("could not create a scratch world"));
			return false;
		}
		Actor = World->SpawnActor<AActor>();
		if (Actor == nullptr)
		{
			Test.AddError(TEXT("could not spawn a scratch actor"));
			return false;
		}
		return true;
	}

	~FScratchActor()
	{
		if (World != nullptr)
		{
			World->DestroyWorld(false);
		}
	}
};

/**
 * Add one element under `Parent`, the way the components panel adds one.
 *
 * Create, attach, register: `RegisterComponent` is the moment the level editor
 * reaches, and the moment the legality check now runs.
 */
template <class T>
T* AddUnder(AActor& Actor, USceneComponent* Parent)
{
	T* const Component = NewObject<T>(&Actor);
	if (Component == nullptr)
	{
		return nullptr;
	}
	if (Parent != nullptr)
	{
		Component->SetupAttachment(Parent);
	}
	Component->RegisterComponent();
	return Component;
}

/** The actor's root element, which has no element parent to be judged against. */
UMjBody* AddRoot(AActor& Actor)
{
	UMjBody* const Body = NewObject<UMjBody>(&Actor);
	if (Body == nullptr)
	{
		return nullptr;
	}
	Actor.SetRootComponent(Body);
	Body->RegisterComponent();
	return Body;
}

/** The "URLab" listing, emptied so a readback is about this test. */
TSharedPtr<IMessageLogListing> ClearedListing()
{
	FMessageLogModule& Module = FModuleManager::LoadModuleChecked<FMessageLogModule>(TEXT("MessageLog"));
	if (!Module.IsRegisteredLogListing(TEXT("URLab")))
	{
		return nullptr;
	}
	const TSharedRef<IMessageLogListing> Listing = Module.GetLogListing(TEXT("URLab"));
	Listing->ClearMessages();
	return Listing;
}

} // namespace MjPlacementLegalityTests

// ============================================================================
// URLab.Editor.AnIllegalChildIsReportedOnTheComponent
//   The level editor's own add path: a geom dropped under a joint. MuJoCo's
//   schema gives `<joint>` no children at all, so the geom would vanish at
//   compile.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjIllegalChildReportedOnComponent,
	"URLab.Editor.AnIllegalChildIsReportedOnTheComponent",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjIllegalChildReportedOnComponent::RunTest(const FString& Parameters)
{
	using namespace MjPlacementLegalityTests;

	FScratchActor Scratch;
	if (!Scratch.Open(*this))
	{
		return false;
	}

	const TSharedPtr<IMessageLogListing> Listing = ClearedListing();

	UMjBody* const Body = AddRoot(*Scratch.Actor);
	if (!TestNotNull(TEXT("a body to build under"), Body))
	{
		return false;
	}
	UMjJoint* const Joint = AddUnder<UMjJoint>(*Scratch.Actor, Body);
	if (!TestNotNull(TEXT("a joint under the body"), Joint))
	{
		return false;
	}

	// The control first, so a check that reported everything would be caught
	// here rather than passing on the illegal case alone.
	UMjGeom* const Legal = AddUnder<UMjGeom>(*Scratch.Actor, Body);
	if (!TestNotNull(TEXT("a geom under the body"), Legal))
	{
		return false;
	}
	TestEqual(TEXT("a geom under a body is where a geom belongs"), Legal->PlacementProblems.Num(), 0);
	TestEqual(TEXT("and so is a joint"), Joint->PlacementProblems.Num(), 0);

	UMjGeom* const Illegal = AddUnder<UMjGeom>(*Scratch.Actor, Joint);
	if (!TestNotNull(TEXT("a geom under the joint"), Illegal))
	{
		return false;
	}

	if (!TestEqual(TEXT("a geom under a joint is reported on the component itself"),
			Illegal->PlacementProblems.Num(), 1))
	{
		return false;
	}
	const FString Reported = Illegal->PlacementProblems[0];
	TestTrue(FString::Printf(TEXT("the report names what was added, got '%s'"), *Reported),
		Reported.Contains(TEXT("<geom>")));
	TestTrue(FString::Printf(TEXT("and what it was added to, got '%s'"), *Reported),
		Reported.Contains(TEXT("<joint>")));

	// The other audience. A row is only read by someone who has already selected
	// the component, and the user who just made the mistake has not.
	if (TestTrue(TEXT("the \"URLab\" listing is registered"), Listing.IsValid()))
	{
		const FString Listed = Listing->GetAllMessagesAsString();
		TestTrue(FString::Printf(TEXT("the illegal add reached the message log, got '%s'"), *Listed),
			Listed.Contains(TEXT("<geom>")) && Listed.Contains(TEXT("<joint>")));
	}

	return !HasAnyErrors();
}

// ============================================================================
// URLab.Editor.AMovedChildStopsBeingReported
//   The row describes where the element is NOW. Moving it somewhere legal and
//   re-registering clears it, or the diagnostic is a stain rather than a state.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjMovedChildStopsBeingReported, "URLab.Editor.AMovedChildStopsBeingReported",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjMovedChildStopsBeingReported::RunTest(const FString& Parameters)
{
	using namespace MjPlacementLegalityTests;

	FScratchActor Scratch;
	if (!Scratch.Open(*this))
	{
		return false;
	}

	UMjBody* const Body = AddRoot(*Scratch.Actor);
	UMjJoint* const Joint = Body != nullptr ? AddUnder<UMjJoint>(*Scratch.Actor, Body) : nullptr;
	UMjGeom* const Geom = Joint != nullptr ? AddUnder<UMjGeom>(*Scratch.Actor, Joint) : nullptr;
	if (!TestNotNull(TEXT("the misplaced geom"), Geom))
	{
		return false;
	}
	if (!TestEqual(TEXT("it starts out reported"), Geom->PlacementProblems.Num(), 1))
	{
		return false;
	}

	Geom->UnregisterComponent();
	Geom->AttachToComponent(Body, FAttachmentTransformRules::KeepRelativeTransform);
	Geom->RegisterComponent();

	TestEqual(TEXT("moving it under the body clears the report"), Geom->PlacementProblems.Num(), 0);

	return !HasAnyErrors();
}

#endif // URLAB_MJ_GEN && WITH_EDITOR
