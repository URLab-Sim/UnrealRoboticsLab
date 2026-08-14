// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// What is drawn, and when it is redrawn.
//
// Two failures with one shape. A `<default>` partial was drawn as though it were
// an element, so a class declaring `type="capsule"` and no size put a
// person-sized capsule at the origin that belonged to nothing and answered to no
// toggle. And an element's picture was pushed once, at registration, from values
// that live somewhere else -- a material's `rgba`, a class's `pos` -- so editing
// where the value lives changed nothing at all.
//
// The first is answered by asking what an element IS: a partial is an
// inheritance template, and templates are not scene content. The second by
// asking who has to re-read: an edit to a shared node makes every element of
// that spec re-derive its own picture, because a dependent list is a reverse
// index that goes stale and this cannot.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Components/StaticMeshComponent.h"
#include "Engine/Blueprint.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Misc/Guid.h"
#include "Misc/ScopeExit.h"
#include "UObject/Package.h"

#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Elements/MjGeom.h"
#include "MuJoCo/Gen/Elements/Defaults/MjDefault.gen.h"
#include "MuJoCo/Gen/Elements/Assets/MjMaterial.gen.h"

namespace MjPresentationTests
{

// A class that declares a shape and no size, exactly as MuJoCo's own humanoid
// does, with the size arriving from a nested class. The partial is therefore
// unresolvable on its own -- which is the whole reason it must not be drawn.
const TCHAR* const Model = TEXT(R"(<mujoco model="present">
  <asset>
    <material name="skin" rgba="0.2 0.4 0.6 1"/>
  </asset>
  <default>
    <default class="body">
      <geom type="capsule" material="skin"/>
      <default class="limb">
        <geom size="0.05 0.1"/>
      </default>
    </default>
  </default>
  <worldbody>
    <body name="b">
      <geom name="shown" class="limb" pos="0 0 0"/>
    </body>
  </worldbody>
</mujoco>
)");

/** A class that inherits `pos` and nothing else, for the propagation case. */
const TCHAR* const InheritedPoseModel = TEXT(R"(<mujoco model="inherit">
  <default>
    <default class="high">
      <geom type="sphere" size="0.05" pos="0 0 1"/>
    </default>
  </default>
  <worldbody>
    <body name="b">
      <geom name="follower" class="high"/>
    </body>
  </worldbody>
</mujoco>
)");

/** Two plain spheres, for the attribute a user changes and immediately looks at. */
const TCHAR* const ShapeModel = TEXT(R"(<mujoco model="shape">
  <worldbody>
    <body name="b">
      <geom name="ball" type="sphere" size="0.05"/>
      <geom name="own" type="sphere" size="0.05"/>
    </body>
  </worldbody>
</mujoco>
)");

UBlueprint* ParseScratch(FAutomationTestBase& Test, const TCHAR* Xml)
{
	const FString Name = FString::Printf(TEXT("MjPresent_%s"), *FGuid::NewGuid().ToString(EGuidFormats::Digits));
	UPackage* Package = CreatePackage(*(TEXT("/Temp/") + Name));
	UBlueprint* Blueprint = FKismetEditorUtilities::CreateBlueprint(AActor::StaticClass(), Package, FName(*Name),
		BPTYPE_Normal, UBlueprint::StaticClass(), UBlueprintGeneratedClass::StaticClass());
	if (Blueprint == nullptr)
	{
		Test.AddError(TEXT("could not create a scratch Blueprint"));
		return nullptr;
	}
	const FMjSpecParseResult Parsed = MjParseIntoBlueprint(*Blueprint, Xml, TEXT("<inline>"));
	if (!Parsed.IsOk())
	{
		Test.AddError(TEXT("parse failed"));
		return nullptr;
	}
	FKismetEditorUtilities::CompileBlueprint(Blueprint);
	return Blueprint;
}

UWorld* ScratchWorld()
{
	return UWorld::CreateWorld(EWorldType::Editor, /*bInformEngineOfWorld=*/false,
		FName(*FString::Printf(TEXT("MjPresentWorld_%s"), *FGuid::NewGuid().ToString(EGuidFormats::Digits))));
}

/** The construction-script template of the element named `MjName`. */
template <class T>
T* TemplateNamed(UBlueprint& Blueprint, const TCHAR* MjName)
{
	if (Blueprint.SimpleConstructionScript == nullptr)
	{
		return nullptr;
	}
	for (USCS_Node* Node : Blueprint.SimpleConstructionScript->GetAllNodes())
	{
		T* Element = Node != nullptr ? Cast<T>(Node->ComponentTemplate) : nullptr;
		if (Element != nullptr && Element->MjName.IsSet() && Element->MjName.GetValue() == MjName)
		{
			return Element;
		}
	}
	return nullptr;
}

template <class T>
T* ComponentNamed(AActor& Actor, const TCHAR* MjName)
{
	TArray<T*> Found;
	Actor.GetComponents(Found);
	for (T* Element : Found)
	{
		if (Element != nullptr && Element->MjName.IsSet() && Element->MjName.GetValue() == MjName)
		{
			return Element;
		}
	}
	return nullptr;
}

/**
 * The geoms that are class partials, found by where they sit.
 *
 * Through `<default>`'s own children rather than through `IsClassPartial`, so
 * the fixture does not answer the question the test is asking.
 */
TArray<UMjGeom*> PartialGeoms(AActor& Actor)
{
	TArray<UMjGeom*> Out;
	TArray<UMjDefault*> Classes;
	Actor.GetComponents(Classes);
	for (UMjDefault* Class : Classes)
	{
		if (Class == nullptr)
		{
			continue;
		}
		for (USceneComponent* Child : Class->GetAttachChildren())
		{
			if (UMjGeom* Geom = Cast<UMjGeom>(Child))
			{
				Out.Add(Geom);
			}
		}
	}
	return Out;
}

/** The colour a geom's preview is actually tinted with, off its material instance. */
bool DrawnColor(const UMjGeom& Geom, FLinearColor& Out)
{
	UStaticMeshComponent* Mesh = Geom.GetVisualizerMesh();
	if (Mesh == nullptr)
	{
		return false;
	}
	UMaterialInstanceDynamic* Instance = Cast<UMaterialInstanceDynamic>(Mesh->GetMaterial(0));
	if (Instance == nullptr)
	{
		return false;
	}
	Out = Instance->K2_GetVectorParameterValue(TEXT("BaseColor"));
	return true;
}

/** Announce an edit of `Property` on `Node` the way the details panel does. */
void NotifyEdited(UMjNodeComponent& Node, const TCHAR* PropertyName)
{
	FProperty* Property = Node.GetClass()->FindPropertyByName(FName(PropertyName));
	FPropertyChangedEvent Event(Property);
	Node.PostEditChangeProperty(Event);
}

/**
 * The whole edit, in the order the property editor performs it.
 *
 * The announcement BEFORE the write is not a formality here: a template records
 * at that moment what the instances following it are still holding, and after
 * the write there is nothing left to compare them against. `NotifyEdited` above
 * announces only the second half, which is enough for the tests that write both
 * sides themselves and not enough for one about the carry.
 */
void EditProperty(UMjNodeComponent& Node, const TCHAR* PropertyName, TFunctionRef<void()> Write)
{
	FProperty* Property = Node.GetClass()->FindPropertyByName(FName(PropertyName));
	Node.PreEditChange(Property);
	Write();
	FPropertyChangedEvent Event(Property, EPropertyChangeType::ValueSet);
	Node.PostEditChangeProperty(Event);
}

/** The engine primitive a geom's preview is currently built from. */
FString PreviewMeshName(const UMjGeom& Geom)
{
	const UStaticMeshComponent* const Part = Geom.GetVisualizerMesh();
	const UStaticMesh* const Asset = Part != nullptr ? Part->GetStaticMesh() : nullptr;
	return Asset != nullptr ? Asset->GetName() : FString(TEXT("<none>"));
}

} // namespace MjPresentationTests

// ============================================================================
// URLab.Preview.ClassPartialsAreNotDrawn
//   A `<default>` partial is an inheritance template. MuJoCo never places it,
//   never compiles it and never draws it, and its attributes need not describe
//   anything that could stand alone -- this one declares a capsule and no size,
//   which is a metre-radius capsule at the origin if anyone previews it. Real
//   elements in the same spec must keep their previews, or the guard is
//   just "draw nothing".
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjPartialsNotDrawnTest, "URLab.Preview.ClassPartialsAreNotDrawn",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjPartialsNotDrawnTest::RunTest(const FString& Parameters)
{
	using namespace MjPresentationTests;

	UBlueprint* Blueprint = ParseScratch(*this, Model);
	if (Blueprint == nullptr)
	{
		return false;
	}

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

	const TArray<UMjGeom*> Partials = PartialGeoms(*Actor);
	TestEqual(TEXT("the fixture has both class partials"), Partials.Num(), 2);
	for (UMjGeom* Partial : Partials)
	{
		TestTrue(TEXT("a geom under <default> knows it is a class partial"), Partial->IsClassPartial());
		TestNull(TEXT("a class partial builds no preview mesh"), Partial->GetVisualizerMesh());
	}

	UMjGeom* Shown = ComponentNamed<UMjGeom>(*Actor, TEXT("shown"));
	if (!TestNotNull(TEXT("the model's own geom"), Shown))
	{
		return false;
	}
	TestFalse(TEXT("an element in the worldbody is not a class partial"), Shown->IsClassPartial());
	TestNotNull(TEXT("an element in the worldbody still previews"), Shown->GetVisualizerMesh());

	return true;
}

// ============================================================================
// URLab.Preview.EditingAMaterialRetintsWhatUsesIt
//   A geom's colour is resolved once and baked into a dynamic material instance
//   when its preview is built. The material is two hops away -- the geom names
//   no material at all, its class does -- and nothing re-reads it, so the
//   material node was inert decoration.
//
//   Edited on the spawned actor, which is the level-instance case and stays
//   inside one spec. Note what that cannot reach:
//   `ForEachInstanceOfTemplate` returns immediately for a component that has an
//   owner, so no template-to-instances walk opens here and none ever could.
//   This test read as though it covered that leg and structurally did not,
//   which is why a fatal fault on it survived a green suite and turned up in a
//   user's editor instead. `EditingAMaterialTemplateRetintsItsInstances` is the
//   one that opens the walk.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjMaterialPropagatesTest, "URLab.Preview.EditingAMaterialRetintsWhatUsesIt",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjMaterialPropagatesTest::RunTest(const FString& Parameters)
{
	using namespace MjPresentationTests;

	UBlueprint* Blueprint = ParseScratch(*this, Model);
	if (Blueprint == nullptr)
	{
		return false;
	}

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

	UMjGeom* Shown = ComponentNamed<UMjGeom>(*Actor, TEXT("shown"));
	UMjMaterial* Material = ComponentNamed<UMjMaterial>(*Actor, TEXT("skin"));
	if (!TestNotNull(TEXT("the geom"), Shown) || !TestNotNull(TEXT("the material"), Material))
	{
		return false;
	}

	FLinearColor Drawn;
	if (!TestTrue(TEXT("the geom is tinted at all"), DrawnColor(*Shown, Drawn)))
	{
		return false;
	}
	TestTrue(TEXT("the import tint is the material's colour"),
		Drawn.Equals(FLinearColor(0.2f, 0.4f, 0.6f, 1.0f), 1e-3f));

	const FLinearColor Edited(0.9f, 0.1f, 0.1f, 1.0f);
	Material->Rgba = Edited;
	NotifyEdited(*Material, TEXT("Rgba"));

	TestTrue(TEXT("the geom is still tinted"), DrawnColor(*Shown, Drawn));
	TestTrue(TEXT("editing the material retinted the geom that resolves through it"),
		Drawn.Equals(Edited, 1e-3f));

	return true;
}

// ============================================================================
// URLab.Preview.EditingAMaterialTemplateRetintsItsInstances
//   The same edit one level up, on the Blueprint's template rather than on a
//   placed actor -- which is where a user actually edits an imported model, and
//   which nothing exercised.
//
//   The two halves of this pass meet here for the first time. Finding a
//   template's instances walks the object hash, and refreshing an instance's
//   presentation builds preview components and a dynamic material instance for
//   each. Creating a UObject while that walk is open is not slow, it is fatal:
//   "Trying to modify UObject map (FindOrAdd) that is currently being
//   iterated". Each half was right on its own and the pair took the editor
//   down, so the traversal has to finish before any of the work starts.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjMaterialTemplatePropagatesTest,
	"URLab.Preview.EditingAMaterialTemplateRetintsItsInstances",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjMaterialTemplatePropagatesTest::RunTest(const FString& Parameters)
{
	using namespace MjPresentationTests;

	UBlueprint* Blueprint = ParseScratch(*this, Model);
	if (Blueprint == nullptr)
	{
		return false;
	}

	UWorld* World = ScratchWorld();
	if (!TestNotNull(TEXT("scratch world"), World))
	{
		return false;
	}
	ON_SCOPE_EXIT
	{
		World->DestroyWorld(false);
	};

	// At least one instance built from the template, which is what makes the
	// object-hash walk find anything and so what makes the hazard reachable.
	AActor* Actor = World->SpawnActor<AActor>(Blueprint->GeneratedClass);
	if (!TestNotNull(TEXT("spawned instance"), Actor))
	{
		return false;
	}

	UMjMaterial* Template = TemplateNamed<UMjMaterial>(*Blueprint, TEXT("skin"));
	UMjMaterial* Instance = ComponentNamed<UMjMaterial>(*Actor, TEXT("skin"));
	UMjGeom* Shown = ComponentNamed<UMjGeom>(*Actor, TEXT("shown"));
	if (!TestNotNull(TEXT("the material template"), Template) || !TestNotNull(TEXT("the material on the instance"), Instance) || !TestNotNull(TEXT("the geom on the instance"), Shown))
	{
		return false;
	}

	const FLinearColor Edited(0.05f, 0.7f, 0.35f, 1.0f);
	Template->Rgba = Edited;

	// Unreal's own property system carries a template's new value onto the
	// instances that had not overridden it, before the change hook runs. That is
	// not what is under test here, so the test stands in for it; what is under
	// test is whether the refresh that follows reaches the instance at all, and
	// whether it survives being reached from inside the walk that found it.
	Instance->Rgba = Edited;

	NotifyEdited(*Template, TEXT("Rgba"));

	FLinearColor Drawn;
	TestTrue(TEXT("the geom is still tinted"), DrawnColor(*Shown, Drawn));
	TestTrue(TEXT("editing the material template retinted the instance's geom"), Drawn.Equals(Edited, 1e-3f));

	return true;
}

// ============================================================================
// URLab.Preview.EditingAClassMovesWhatInheritsFromIt
//   The same mechanism, on the other shared node. An element that authors no
//   `pos` previews at the one its class declares; moving the class has to move
//   the element, and the element never names the attribute that moved.
//
//   Edited on the spawned actor, so like the instance-side material case this
//   covers propagation within one spec and opens no template walk. The
//   template leg is the test below.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjClassPropagatesTest, "URLab.Preview.EditingAClassMovesWhatInheritsFromIt",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjClassPropagatesTest::RunTest(const FString& Parameters)
{
	using namespace MjPresentationTests;

	UBlueprint* Blueprint = ParseScratch(*this, InheritedPoseModel);
	if (Blueprint == nullptr)
	{
		return false;
	}

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

	UMjGeom* Follower = ComponentNamed<UMjGeom>(*Actor, TEXT("follower"));
	const TArray<UMjGeom*> Partials = PartialGeoms(*Actor);
	if (!TestNotNull(TEXT("the inheriting geom"), Follower) || !TestEqual(TEXT("the fixture has one class partial"), Partials.Num(), 1))
	{
		return false;
	}

	// MJCF (0, 0, 1) is Unreal (0, 0, 100).
	TestFalse(TEXT("the element authors no pos of its own"), Follower->HasPos());
	TestTrue(TEXT("it previews where its class puts it"),
		Follower->GetRelativeLocation().Equals(FVector(0.0, 0.0, 100.0), 1e-3));

	Partials[0]->SetPos(FMjPosition3(0.0, 0.0, 2.0));
	NotifyEdited(*Partials[0], TEXT("Pos"));

	TestTrue(TEXT("moving the class moved what inherits from it"),
		Follower->GetRelativeLocation().Equals(FVector(0.0, 0.0, 200.0), 1e-3));
	TestFalse(TEXT("and the element still authors no pos of its own"), Follower->HasPos());

	return true;
}

// ============================================================================
// URLab.Preview.EditingAClassTemplateMovesTheInstancesThatInheritIt
//   The class edited on the Blueprint's template rather than on a placed
//   actor -- the identical arrangement to the material template case, and so
//   the identical hazard: a shared node edited on the template, instances that
//   have to follow, and a walk of the object hash between the two. What a
//   class moves is a transform rather than a colour, but the refresh it runs
//   is the same one, and it rebuilds visualisers just as readily.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjClassTemplatePropagatesTest,
	"URLab.Preview.EditingAClassTemplateMovesTheInstancesThatInheritIt",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjClassTemplatePropagatesTest::RunTest(const FString& Parameters)
{
	using namespace MjPresentationTests;

	UBlueprint* Blueprint = ParseScratch(*this, InheritedPoseModel);
	if (Blueprint == nullptr)
	{
		return false;
	}

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

	UMjGeom* Follower = ComponentNamed<UMjGeom>(*Actor, TEXT("follower"));
	const TArray<UMjGeom*> InstancePartials = PartialGeoms(*Actor);
	if (!TestNotNull(TEXT("the inheriting geom on the instance"), Follower) || !TestEqual(TEXT("the instance has one class partial"), InstancePartials.Num(), 1))
	{
		return false;
	}

	// The class partial's template. A partial carries no MJCF name, so it is
	// found by where it sits -- through the construction script's own node tree,
	// which is where a template's parentage lives, and not through
	// `IsClassPartial`, which is part of what this exercises.
	UMjGeom* TemplatePartial = nullptr;
	for (USCS_Node* Node : Blueprint->SimpleConstructionScript->GetAllNodes())
	{
		if (Node == nullptr || Cast<UMjDefault>(Node->ComponentTemplate) == nullptr)
		{
			continue;
		}
		for (USCS_Node* Child : Node->GetChildNodes())
		{
			if (UMjGeom* Geom = Child != nullptr ? Cast<UMjGeom>(Child->ComponentTemplate) : nullptr)
			{
				TemplatePartial = Geom;
				break;
			}
		}
		if (TemplatePartial != nullptr)
		{
			break;
		}
	}
	if (!TestNotNull(TEXT("the class partial's template"), TemplatePartial))
	{
		return false;
	}

	TestTrue(TEXT("the instance starts where its class puts it"),
		Follower->GetRelativeLocation().Equals(FVector(0.0, 0.0, 100.0), 1e-3));

	// MJCF (0, 0, 2) is Unreal (0, 0, 200). As with the material template, the
	// value's journey from template to instance is Unreal's property system and
	// not what is under test; the refresh that has to follow it is.
	TemplatePartial->SetPos(FMjPosition3(0.0, 0.0, 2.0));
	InstancePartials[0]->SetPos(FMjPosition3(0.0, 0.0, 2.0));

	NotifyEdited(*TemplatePartial, TEXT("Pos"));

	TestTrue(TEXT("moving the class template moved the instance that inherits it"),
		Follower->GetRelativeLocation().Equals(FVector(0.0, 0.0, 200.0), 1e-3));
	TestFalse(TEXT("and that instance still authors no pos of its own"), Follower->HasPos());

	return true;
}

// ============================================================================
// URLab.Preview.EditingATemplateAttributeReachesThePreview
//   Author `type="box"` on a geom and the picture stays a sphere.
//
//   Not the rebuild gate, which names `Type` and fires. The Blueprint editor
//   re-runs the preview actor's construction scripts on a template edit, and
//   re-running them reads any attribute an instance holds differently from its
//   template as an instance OVERRIDE: it caches the instance's pre-edit value and
//   puts it back on the rebuilt component. So the moment the template's `type`
//   moves, the instance's unchanged `type` becomes an override of it, and the
//   preview is a sphere for the rest of the session while the panel says box.
//
//   The pose has carried across since the drag work. Every other attribute had
//   nothing, so this asserts the carry on `type` -- the one a user changes and
//   immediately looks at -- and on an attribute that decides nothing visual,
//   which must carry just as faithfully and must NOT cost a rebuild.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjTemplateAttributeReachesThePreview,
	"URLab.Preview.EditingATemplateAttributeReachesThePreview",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjTemplateAttributeReachesThePreview::RunTest(const FString& Parameters)
{
	using namespace MjPresentationTests;

	UBlueprint* Blueprint = ParseScratch(*this, ShapeModel);
	if (Blueprint == nullptr)
	{
		return false;
	}

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
	UMjGeom* Template = TemplateNamed<UMjGeom>(*Blueprint, TEXT("ball"));
	UMjGeom* Instance = Actor != nullptr ? ComponentNamed<UMjGeom>(*Actor, TEXT("ball")) : nullptr;
	if (Actor == nullptr || !TestNotNull(TEXT("the geom's template"), Template) || !TestNotNull(TEXT("the geom on the preview"), Instance))
	{
		return false;
	}

	TestEqual(TEXT("the preview starts as the sphere the document authored"), PreviewMeshName(*Instance),
		FString(TEXT("Sphere")));

	// The edit, on the template alone -- which is what the Blueprint editor's
	// details panel changes when the user picks a different type.
	EditProperty(*Template, TEXT("Type"), [Template]() { Template->Type = EMjGeomType::box; });

	TestTrue(TEXT("the instance following the template took the new type"),
		Instance->Type.IsSet() && Instance->Type.GetValue() == EMjGeomType::box);
	TestEqual(TEXT("so its picture is the box"), PreviewMeshName(*Instance), FString(TEXT("Cube")));

	// And it survives the reconstruction that used to undo it. This is literally
	// what FSCSEditorViewportClient runs after a template edit.
	Actor->RerunConstructionScripts();
	UMjGeom* Rebuilt = ComponentNamed<UMjGeom>(*Actor, TEXT("ball"));
	if (!TestNotNull(TEXT("the geom after reconstruction"), Rebuilt))
	{
		return false;
	}
	TestTrue(TEXT("the rebuilt component still carries the type the template authored"),
		Rebuilt->Type.IsSet() && Rebuilt->Type.GetValue() == EMjGeomType::box);
	TestEqual(TEXT("and draws it"), PreviewMeshName(*Rebuilt), FString(TEXT("Cube")));

	// The other arm. An instance the user has authored on has really overridden
	// the template, and a later template edit must not take that back -- the same
	// rule the pose carry follows, and without this the assertions above would
	// pass for a carry that simply overwrote everything.
	UMjGeom* OwnTemplate = TemplateNamed<UMjGeom>(*Blueprint, TEXT("own"));
	UMjGeom* Own = ComponentNamed<UMjGeom>(*Actor, TEXT("own"));
	if (TestNotNull(TEXT("the second geom's template"), OwnTemplate) && TestNotNull(TEXT("the second geom on the preview"), Own))
	{
		EditProperty(*Own, TEXT("Type"), [Own]() { Own->Type = EMjGeomType::capsule; });
		EditProperty(*OwnTemplate, TEXT("Type"), [OwnTemplate]() { OwnTemplate->Type = EMjGeomType::box; });

		TestTrue(TEXT("an instance that authored its own type keeps it"),
			Own->Type.IsSet() && Own->Type.GetValue() == EMjGeomType::capsule);
	}

	// The cost. An attribute that decides nothing visual has to reach the
	// instance just as faithfully, and must not rebuild a picture on the way:
	// that rebuild is what the lag work removed, and a carry that reinstated it
	// would put it back on every keystroke of every geom attribute.
	UMjGeom* Cheap = ComponentNamed<UMjGeom>(*Actor, TEXT("ball"));
	UMjGeom* CheapTemplate = TemplateNamed<UMjGeom>(*Blueprint, TEXT("ball"));
	if (TestNotNull(TEXT("the geom for the cost arm"), Cheap) && TestNotNull(TEXT("its template"), CheapTemplate))
	{
		const UStaticMeshComponent* const Drawn = Cheap->GetVisualizerMesh();
		EditProperty(*CheapTemplate, TEXT("Contype"), [CheapTemplate]() { CheapTemplate->Contype = 5; });

		TestTrue(TEXT("a non-visual attribute reaches the instance too"),
			Cheap->Contype.IsSet() && Cheap->Contype.GetValue() == 5);
		TestTrue(TEXT("and the picture was not torn down and rebuilt for it"),
			Cheap->GetVisualizerMesh() == Drawn);
	}

	return !HasAnyErrors();
}

#endif // URLAB_MJ_GEN && WITH_EDITOR
