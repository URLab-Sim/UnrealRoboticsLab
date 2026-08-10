// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// Dragging a geom has to reach the spec.
//
// Preview-sync and write-back are the two halves of one cycle, and the guard
// that keeps them from chasing each other is a cached baseline: the transform
// the preview last applied. Write-back authors only what differs from it, which
// is what stops a body drag -- delivered to every descendant by Unreal --
// authoring a pose on children that did not move.
//
// The cache is transient, and it does NOT survive everything that can hand a
// user a component to drag. When it was cold, the write-back synced from the
// spec and returned: the widget moved, the spec did not, and the geom
// snapped back. The cache is a cache OF the spec, so a cold cache is
// answerable from the spec rather than a reason to discard the move.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Editor.h"
#include "Engine/Blueprint.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "Misc/Guid.h"
#include "Misc/ScopeExit.h"
#include "UObject/Package.h"

#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Elements/MjGeom.h"

namespace MjDragTests
{

const TCHAR* const Model = TEXT(R"(<mujoco model="drag">
  <compiler angle="radian"/>
  <worldbody>
    <body name="b" pos="0 0 0">
      <geom name="g" type="sphere" size="0.05" pos="0.1 0.2 0.3"/>
      <geom name="still" type="sphere" size="0.05"/>
    </body>
  </worldbody>
</mujoco>
)");

UMjGeom* ParseAndFind(FAutomationTestBase& Test, const TCHAR* MjName, UBlueprint*& OutBlueprint)
{
	const FString Name = FString::Printf(TEXT("MjDrag_%s"), *FGuid::NewGuid().ToString(EGuidFormats::Digits));
	UPackage* Package = CreatePackage(*(TEXT("/Temp/") + Name));
	OutBlueprint = FKismetEditorUtilities::CreateBlueprint(AActor::StaticClass(), Package, FName(*Name), BPTYPE_Normal,
		UBlueprint::StaticClass(), UBlueprintGeneratedClass::StaticClass());
	if (OutBlueprint == nullptr)
	{
		Test.AddError(TEXT("could not create a scratch Blueprint"));
		return nullptr;
	}
	const FMjSpecParseResult Parsed = MjParseIntoBlueprint(*OutBlueprint, Model, TEXT("<inline>"));
	if (!Parsed.IsOk())
	{
		Test.AddError(TEXT("parse failed"));
		return nullptr;
	}
	for (USCS_Node* Node : OutBlueprint->SimpleConstructionScript->GetAllNodes())
	{
		UMjGeom* Geom = Node != nullptr ? Cast<UMjGeom>(Node->ComponentTemplate) : nullptr;
		if (Geom != nullptr && Geom->MjName.IsSet() && Geom->MjName.GetValue() == MjName)
		{
			return Geom;
		}
	}
	Test.AddError(FString::Printf(TEXT("geom '%s' not found"), MjName));
	return nullptr;
}

/** The template of the geom named `MjName`, re-found after a recompile moved it. */
UMjGeom* TemplateNamed(UBlueprint& Blueprint, const TCHAR* MjName)
{
	for (USCS_Node* Node : Blueprint.SimpleConstructionScript->GetAllNodes())
	{
		UMjGeom* Geom = Node != nullptr ? Cast<UMjGeom>(Node->ComponentTemplate) : nullptr;
		if (Geom != nullptr && Geom->MjName.IsSet() && Geom->MjName.GetValue() == MjName)
		{
			return Geom;
		}
	}
	return nullptr;
}

/** The geom named `MjName` among a spawned actor's components. */
UMjGeom* InstanceNamed(AActor& Actor, const TCHAR* MjName)
{
	TArray<UMjGeom*> Geoms;
	Actor.GetComponents(Geoms);
	for (UMjGeom* Geom : Geoms)
	{
		if (Geom != nullptr && Geom->MjName.IsSet() && Geom->MjName.GetValue() == MjName)
		{
			return Geom;
		}
	}
	return nullptr;
}

/**
 * A translation-only gizmo delta, through the engine's own apply.
 *
 * Not `SetRelativeLocation` plus a hand-called hook: what the two editor
 * viewports actually run is `UEditorEngine::ApplyDeltaToComponent`, and the hook
 * it fires -- and the object it fires it on -- is half of what is under test.
 */
void DragBy(USceneComponent& Component, const FVector& Delta)
{
	FVector Drag = Delta;
	FRotator NoRotation = FRotator::ZeroRotator;
	FVector NoScale = FVector::ZeroVector;
	GEditor->ApplyDeltaToComponent(&Component, /*bDelta=*/true, &Drag, &NoRotation, &NoScale,
		Component.GetRelativeLocation());
}

UWorld* ScratchWorld()
{
	return UWorld::CreateWorld(EWorldType::Editor, /*bInformEngineOfWorld=*/false,
		FName(*FString::Printf(TEXT("MjDragWorld_%s"), *FGuid::NewGuid().ToString(EGuidFormats::Digits))));
}

/** MJCF metres from the geom's authored `pos`, or (0,0,0) when it authors none. */
FVector AuthoredPos(const UMjGeom& Geom)
{
	const FMjPosition3 Pos = Geom.GetPos();
	return FVector(Pos.X, Pos.Y, Pos.Z);
}

} // namespace MjDragTests

// ============================================================================
// URLab.Doc.DragReachesTheSpecWithAColdBaseline
//   The reported failure, in the one state that produces it. The baseline is
//   not a UPROPERTY, so a duplicated component has none -- which is the same
//   position a component handed to the user by a path that never synced is in.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjDragColdBaselineTest, "URLab.Doc.DragReachesTheSpecWithAColdBaseline",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjDragColdBaselineTest::RunTest(const FString& Parameters)
{
	using namespace MjDragTests;

	UBlueprint* Blueprint = nullptr;
	UMjGeom* Source = ParseAndFind(*this, TEXT("g"), Blueprint);
	if (Source == nullptr)
	{
		return false;
	}

	// A duplicate carries every reflected field and none of the transient ones,
	// so its baseline is cold while its spec values are intact.
	UMjGeom* Geom = DuplicateObject<UMjGeom>(Source, GetTransientPackage());
	if (!TestNotNull(TEXT("duplicated geom"), Geom))
	{
		return false;
	}
	TestTrue(TEXT("the duplicate kept the spec pose"), Geom->HasPos());

	// Drag it. MJCF (0.1, 0.2, 0.3) is Unreal (10, -20, 30); move it to Unreal
	// (50, -60, 70), which is MJCF (0.5, 0.6, 0.7).
	Geom->SetRelativeLocation(FVector(50.0, -60.0, 70.0));
	Geom->PostEditComponentMove(true);

	const FMjPosition3 Pos = Geom->GetPos();
	TestNearlyEqual(TEXT("the drag reached pos X"), (float)Pos.X, 0.5f, 1e-4f);
	TestNearlyEqual(TEXT("the drag reached pos Y"), (float)Pos.Y, 0.6f, 1e-4f);
	TestNearlyEqual(TEXT("the drag reached pos Z"), (float)Pos.Z, 0.7f, 1e-4f);

	// And the component stayed where it was put rather than being snapped back.
	TestTrue(TEXT("the component was not snapped back to the spec"),
		Geom->GetRelativeLocation().Equals(FVector(50.0, -60.0, 70.0), 1e-3));

	return true;
}

// ============================================================================
// URLab.Doc.UnmovedNodeStillAuthorsNothing
//   The other half, which the change must not cost. Unreal delivers the move
//   hook to every descendant of what was dragged, so a node that did not move
//   receives it too -- and `pos` is defaultable, so authoring one on a guess
//   silently severs that element from its default class.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjDragUnmovedTest, "URLab.Doc.UnmovedNodeStillAuthorsNothing",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjDragUnmovedTest::RunTest(const FString& Parameters)
{
	using namespace MjDragTests;

	UBlueprint* Blueprint = nullptr;
	UMjGeom* Source = ParseAndFind(*this, TEXT("still"), Blueprint);
	if (Source == nullptr)
	{
		return false;
	}

	UMjGeom* Geom = DuplicateObject<UMjGeom>(Source, GetTransientPackage());
	if (!TestNotNull(TEXT("duplicated geom"), Geom))
	{
		return false;
	}
	TestFalse(TEXT("the geom authors no pos to begin with"), Geom->HasPos());

	// The hook, with a cold baseline and no move: the spec must not acquire
	// a pose it never had.
	Geom->PostEditComponentMove(true);
	TestFalse(TEXT("a hook without a move authors no pos"), Geom->HasPos());

	return true;
}

// ============================================================================
// URLab.Doc.DragInTheBlueprintEditorReachesTheSpec
//   The Blueprint editor's viewport does not drag the component the user can
//   see. It applies the delta to the construction-script TEMPLATE and lets the
//   preview actor be rebuilt from it -- so the write-back has to fire on an
//   unregistered template, in a Blueprint that has been compiled, with the rest
//   of the spec reachable only through the outer chain.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjDragTemplateTest, "URLab.Doc.DragInTheBlueprintEditorReachesTheSpec",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjDragTemplateTest::RunTest(const FString& Parameters)
{
	using namespace MjDragTests;

	UBlueprint* Blueprint = nullptr;
	if (ParseAndFind(*this, TEXT("g"), Blueprint) == nullptr)
	{
		return false;
	}
	FKismetEditorUtilities::CompileBlueprint(Blueprint);

	UMjGeom* Geom = TemplateNamed(*Blueprint, TEXT("g"));
	if (!TestNotNull(TEXT("geom template after compile"), Geom))
	{
		return false;
	}

	// MJCF (0.1, 0.2, 0.3) is Unreal (10, -20, 30). This delta lands it on Unreal
	// (50, -60, 70), which is MJCF (0.5, 0.6, 0.7).
	DragBy(*Geom, FVector(40.0, -40.0, 40.0));

	TestTrue(TEXT("the drag reached the template's pos"),
		AuthoredPos(*Geom).Equals(FVector(0.5, 0.6, 0.7), 1e-4));
	TestTrue(TEXT("the template stayed where the drag put it"),
		Geom->GetRelativeLocation().Equals(FVector(50.0, -60.0, 70.0), 1e-3));

	return true;
}

// ============================================================================
// URLab.Doc.DragOnALevelInstanceSurvivesReconstruction
//   Dragging a component of a placed Blueprint actor ends with
//   AActor::PostEditMove(true), which re-runs the construction scripts: every
//   construction-script component is destroyed and rebuilt from its template,
//   and only what the instance-data cache carried across comes back. If the
//   authored `pos` is not among it, the rebuilt geom reads the template's old
//   pose and the drag is gone -- the widget moved and the geom did not.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjDragInstanceTest, "URLab.Doc.DragOnALevelInstanceSurvivesReconstruction",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjDragInstanceTest::RunTest(const FString& Parameters)
{
	using namespace MjDragTests;

	UBlueprint* Blueprint = nullptr;
	if (ParseAndFind(*this, TEXT("g"), Blueprint) == nullptr)
	{
		return false;
	}
	FKismetEditorUtilities::CompileBlueprint(Blueprint);

	UWorld* World = ScratchWorld();
	if (!TestNotNull(TEXT("scratch world"), World))
	{
		return false;
	}
	ON_SCOPE_EXIT
	{
		World->DestroyWorld(false);
	};

	AActor* Actor = World->SpawnActor<AActor>(Blueprint->GeneratedClass);
	if (!TestNotNull(TEXT("spawned instance"), Actor))
	{
		return false;
	}

	UMjGeom* Geom = InstanceNamed(*Actor, TEXT("g"));
	if (!TestNotNull(TEXT("geom on the instance"), Geom))
	{
		return false;
	}
	const UMjGeom* Dragged = Geom;

	DragBy(*Geom, FVector(40.0, -40.0, 40.0));
	TestTrue(TEXT("the drag reached the instance's pos"),
		AuthoredPos(*Geom).Equals(FVector(0.5, 0.6, 0.7), 1e-4));

	// What the level viewport does when the drag ends.
	Actor->PostEditMove(/*bFinished=*/true);

	UMjGeom* Rebuilt = InstanceNamed(*Actor, TEXT("g"));
	if (!TestNotNull(TEXT("geom after reconstruction"), Rebuilt))
	{
		return false;
	}
	TestTrue(TEXT("the construction scripts really did rebuild the component"), Rebuilt != Dragged);

	TestTrue(TEXT("the spec kept the drag across reconstruction"),
		AuthoredPos(*Rebuilt).Equals(FVector(0.5, 0.6, 0.7), 1e-4));
	TestTrue(TEXT("the rebuilt component is where the spec says"),
		Rebuilt->GetRelativeLocation().Equals(FVector(50.0, -60.0, 70.0), 1e-3));

	return true;
}

// ============================================================================
// URLab.Doc.ABlueprintDragCarriesToTheInstancesFollowingIt
//   The reported failure, and it is not the write-back.
//
//   The Blueprint editor's viewport drags the TEMPLATE and then re-runs the
//   preview actor's construction scripts, on every delta. Between those two
//   things the instance holds the pre-drag `pos` and the template holds the new
//   one -- and that difference is exactly what the instance-data cache records
//   as an instance OVERRIDE. It caches it, rebuilds the component from the
//   template, and puts the pre-drag `pos` back on top. From then on the geom is
//   pinned to where it started while the widget goes on moving.
//
//   `RerunConstructionScripts` here is not a stand-in for the editor: it is
//   literally what FSCSEditorViewportClient::InputWidgetDelta calls.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjDragTemplateToInstanceTest, "URLab.Doc.ABlueprintDragCarriesToTheInstancesFollowingIt",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjDragTemplateToInstanceTest::RunTest(const FString& Parameters)
{
	using namespace MjDragTests;

	UBlueprint* Blueprint = nullptr;
	if (ParseAndFind(*this, TEXT("g"), Blueprint) == nullptr)
	{
		return false;
	}
	FKismetEditorUtilities::CompileBlueprint(Blueprint);

	UWorld* World = ScratchWorld();
	if (!TestNotNull(TEXT("scratch world"), World))
	{
		return false;
	}
	ON_SCOPE_EXIT
	{
		World->DestroyWorld(false);
	};

	AActor* Actor = World->SpawnActor<AActor>(Blueprint->GeneratedClass);
	UMjGeom* Template = TemplateNamed(*Blueprint, TEXT("g"));
	if (!TestNotNull(TEXT("spawned instance"), Actor) || !TestNotNull(TEXT("geom template"), Template))
	{
		return false;
	}
	if (!TestNotNull(TEXT("geom on the instance"), InstanceNamed(*Actor, TEXT("g"))))
	{
		return false;
	}

	// MJCF (0.1, 0.2, 0.3) is Unreal (10, -20, 30); this lands on MJCF (0.5, 0.6, 0.7).
	DragBy(*Template, FVector(40.0, -40.0, 40.0));
	TestTrue(TEXT("the drag reached the template's pos"),
		AuthoredPos(*Template).Equals(FVector(0.5, 0.6, 0.7), 1e-4));

	// What the Blueprint editor's viewport does with every delta.
	Actor->RerunConstructionScripts();

	UMjGeom* Rebuilt = InstanceNamed(*Actor, TEXT("g"));
	if (!TestNotNull(TEXT("geom after reconstruction"), Rebuilt))
	{
		return false;
	}
	TestTrue(TEXT("the instance's spec followed the template it was following"),
		AuthoredPos(*Rebuilt).Equals(FVector(0.5, 0.6, 0.7), 1e-4));
	TestTrue(TEXT("and the preview is where the drag put it, not where it started"),
		Rebuilt->GetRelativeLocation().Equals(FVector(50.0, -60.0, 70.0), 1e-3));

	return true;
}

// ============================================================================
// URLab.Doc.ABlueprintDragLeavesAnOverriddenInstanceAlone
//   The other half, which carrying a template edit onto instances must not
//   cost. An instance the user has moved itself has really overridden the
//   template, and a later drag of the template must not take that back.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjDragTemplateKeepsOverrideTest,
	"URLab.Doc.ABlueprintDragLeavesAnOverriddenInstanceAlone",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjDragTemplateKeepsOverrideTest::RunTest(const FString& Parameters)
{
	using namespace MjDragTests;

	UBlueprint* Blueprint = nullptr;
	if (ParseAndFind(*this, TEXT("g"), Blueprint) == nullptr)
	{
		return false;
	}
	FKismetEditorUtilities::CompileBlueprint(Blueprint);

	UWorld* World = ScratchWorld();
	if (!TestNotNull(TEXT("scratch world"), World))
	{
		return false;
	}
	ON_SCOPE_EXIT
	{
		World->DestroyWorld(false);
	};

	AActor* Actor = World->SpawnActor<AActor>(Blueprint->GeneratedClass);
	UMjGeom* Template = TemplateNamed(*Blueprint, TEXT("g"));
	UMjGeom* Instance = Actor != nullptr ? InstanceNamed(*Actor, TEXT("g")) : nullptr;
	if (!TestNotNull(TEXT("geom template"), Template) || !TestNotNull(TEXT("geom on the instance"), Instance))
	{
		return false;
	}

	// The user moves this one placed geom: Unreal (10, -20, 30) -> (10, -20, 90),
	// which is MJCF (0.1, 0.2, 0.9).
	DragBy(*Instance, FVector(0.0, 0.0, 60.0));
	TestTrue(TEXT("the instance authored its own pos"),
		AuthoredPos(*Instance).Equals(FVector(0.1, 0.2, 0.9), 1e-4));

	// Then the template moves. The instance is no longer following it.
	DragBy(*Template, FVector(40.0, -40.0, 40.0));

	TestTrue(TEXT("the template moved"), AuthoredPos(*Template).Equals(FVector(0.5, 0.6, 0.7), 1e-4));
	TestTrue(TEXT("the instance kept the pose it authored"),
		AuthoredPos(*Instance).Equals(FVector(0.1, 0.2, 0.9), 1e-4));

	return true;
}

#endif // URLAB_MJ_GEN && WITH_EDITOR
