// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// A geom that resolves to a shape with nothing to draw.
//
// Adding a geom to a menagerie robot by hand is not the exotic case; it is the
// first thing anyone does with an imported model. The body carries
// `childclass="visual"`, the class says `type="mesh"`, and the geom the user
// just added inherits that type while naming no mesh. Nothing appears in the
// viewport, and the NEXT compile fails, at which point the model is a robot with
// one invisible element and no line number pointing at it.
//
// Two arms rather than one, because the fixture has to be able to fail: the arm
// with no class on it adds the same geom and builds a preview, so a run where
// nothing draws at all cannot pass this by drawing nothing everywhere. And a
// third geom names a mesh whose asset was never prepared -- the case the import
// pass already reports, with the file name this code does not have -- which is
// the one that must stay silent here.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Components/StaticMeshComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Misc/Guid.h"

#include "MuJoCo/Elements/MjGeom.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjSpecRef.h"

namespace MjUndrawableGeomTests
{

/**
 * One arm whose class supplies `type="mesh"`, one with no class at all.
 *
 * The `<mesh>` asset is declared and never prepared, which is what the third
 * geom below points at.
 */
const TCHAR* const Model = TEXT(R"(<mujoco model="two_arms">
  <default>
    <default class="visual">
      <geom type="mesh" rgba="0.8 0.8 0.8 1"/>
    </default>
  </default>
  <asset>
    <mesh name="link_0" file="link_0.stl"/>
  </asset>
  <worldbody>
    <body name="mesh_arm" childclass="visual" pos="0 0 0"/>
    <body name="plain_arm" pos="1 0 0"/>
  </worldbody>
</mujoco>
)");

/** A live actor holding the parsed spec, so hand-added components really register. */
struct FScratchDoc
{
	UWorld* World = nullptr;
	AActor* Actor = nullptr;

	~FScratchDoc()
	{
		if (World != nullptr)
		{
			World->DestroyWorld(false);
		}
	}
};

bool Parse(FAutomationTestBase& Test, FScratchDoc& Doc)
{
	const FString Stem = FString::Printf(TEXT("MjUndrawable_%s"), *FGuid::NewGuid().ToString(EGuidFormats::Digits));
	Doc.World = UWorld::CreateWorld(EWorldType::Editor, /*bInformEngineOfWorld=*/false, FName(*Stem));
	if (Doc.World == nullptr)
	{
		Test.AddError(TEXT("could not create a scratch world"));
		return false;
	}
	Doc.Actor = Doc.World->SpawnActor<AActor>();
	if (Doc.Actor == nullptr)
	{
		Test.AddError(TEXT("could not spawn a scratch actor"));
		return false;
	}
	if (!MjParseIntoActor(*Doc.Actor, Model, Stem + TEXT(".xml")).IsOk())
	{
		Test.AddError(TEXT("parse failed"));
		return false;
	}
	return true;
}

/** The element named `MjName` among the actor's components. */
template <class T>
T* Named(AActor& Actor, const TCHAR* MjName)
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
 * A geom added under `Parent` exactly as the components panel adds one.
 *
 * Attached before it registers, because the class chain is reached through the
 * parent: a geom registered first and attached afterwards would resolve against
 * nothing and draw the schema's own sphere, which is not what the user sees.
 */
UMjGeom* AddGeomUnder(AActor& Actor, USceneComponent& Parent, const TCHAR* Label)
{
	UMjGeom* const Geom = NewObject<UMjGeom>(&Actor, UMjGeom::StaticClass(), FName(Label));
	if (Geom == nullptr)
	{
		return nullptr;
	}
	Geom->CreationMethod = EComponentCreationMethod::Instance;
	Actor.AddInstanceComponent(Geom);
	Geom->SetupAttachment(&Parent);
	Geom->RegisterComponent();
	return Geom;
}

/** The element's undrawable-shape row, or empty when it has none. */
FString ProblemText(const UMjNodeComponent& Element)
{
	return Element.PreviewProblems.Num() > 0 ? Element.PreviewProblems[0] : FString();
}

}  // namespace MjUndrawableGeomTests

// ============================================================================
// URLab.Preview.InheritedMeshTypeWithNoMeshIsReported
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjUndrawableGeomIsReported, "URLab.Preview.InheritedMeshTypeWithNoMeshIsReported",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjUndrawableGeomIsReported::RunTest(const FString& Parameters)
{
	using namespace MjUndrawableGeomTests;

	FScratchDoc Doc;
	if (!Parse(*this, Doc))
	{
		return false;
	}

	UMjNodeComponent* const MeshArm = Named<UMjNodeComponent>(*Doc.Actor, TEXT("mesh_arm"));
	UMjNodeComponent* const PlainArm = Named<UMjNodeComponent>(*Doc.Actor, TEXT("plain_arm"));
	if (MeshArm == nullptr || PlainArm == nullptr)
	{
		AddError(TEXT("the fixture's bodies were not imported"));
		return false;
	}

	UMjGeom* const Inherited = AddGeomUnder(*Doc.Actor, *MeshArm, TEXT("HandAddedUnderMeshArm"));
	UMjGeom* const Control = AddGeomUnder(*Doc.Actor, *PlainArm, TEXT("HandAddedUnderPlainArm"));
	if (Inherited == nullptr || Control == nullptr)
	{
		AddError(TEXT("could not add a geom by hand"));
		return false;
	}

	// The control first, because it is what makes the rest of this test mean
	// anything: the same gesture on a body with no class on it produces a geom
	// that draws, so an absent preview below is the inherited type and not a
	// harness that draws nothing.
	TestNotNull(TEXT("a hand-added geom under a class-free body builds a preview"), Control->GetVisualizerMesh());
	TestTrue(TEXT("and it is a sphere, whose size a scale handle edits"), Control->HasScaleMapping());
	TestEqual(TEXT("so it has nothing to report"), Control->PreviewProblems.Num(), 0);

	// The reported arm. Its type came from three levels away and it names no
	// mesh, so there is no picture to build and no geom for the compiler.
	TestNull(TEXT("a hand-added geom under a mesh class builds no preview"), Inherited->GetVisualizerMesh());
	TestTrue(TEXT("its effective mesh name is empty"), Inherited->EffectiveMeshName().IsEmpty());
	TestFalse(TEXT("a mesh shape has no scale mapping"), Inherited->HasScaleMapping());

	if (TestEqual(TEXT("the undrawable geom carries exactly one diagnostic"), Inherited->PreviewProblems.Num(), 1))
	{
		const FString Reported = ProblemText(*Inherited);
		TestTrue(FString::Printf(TEXT("it names the class the type came from, got '%s'"), *Reported),
			Reported.Contains(TEXT("'visual'")));
		TestTrue(FString::Printf(TEXT("it says the type is mesh, got '%s'"), *Reported),
			Reported.Contains(TEXT("type=\"mesh\"")));
		TestTrue(FString::Printf(TEXT("it says no mesh is named, got '%s'"), *Reported),
			Reported.Contains(TEXT("no <mesh> is named")));
		TestTrue(FString::Printf(TEXT("it says the model will not compile, got '%s'"), *Reported),
			Reported.Contains(TEXT("will not compile")));
	}

	// Naming a mesh is what clears it, and the diagnostic is withdrawn the moment
	// it stops being true rather than lingering for the session.
	Inherited->Mesh = FString(TEXT("link_0"));
	Inherited->RefreshPresentation();
	TestEqual(TEXT("naming a mesh withdraws the diagnostic"), Inherited->PreviewProblems.Num(), 0);

	// And it stays withdrawn although the asset itself is absent: that geom has a
	// name, the name failed to resolve to a prepared asset, and reporting it here
	// would fire on every import whose meshes are prepared in a later step.
	TestNull(TEXT("an unprepared mesh asset still builds no preview"), Inherited->GetVisualizerMesh());
	TestEqual(TEXT("but an unresolved asset is not this diagnostic's business"),
		Inherited->PreviewProblems.Num(), 0);

	return !HasAnyErrors();
}

#endif  // URLAB_MJ_GEN && WITH_EDITOR
