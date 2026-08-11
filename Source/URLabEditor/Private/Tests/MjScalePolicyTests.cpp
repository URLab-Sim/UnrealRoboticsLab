// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// What a scale handle is allowed to do to an element.
//
// MJCF has no scale, so a component's scale is an editor of the element's
// `size` or it is a lie about what will be simulated. Three things are asserted
// here, and each of them failed at some point in exactly one way:
//
//   the lock runs at all      A sphere is one radius, and the per-type lock that
//                             says so was correct and was being SKIPPED: a
//                             freshly added geom has no cached baseline and no
//                             authored size, the baseline was seeded from the
//                             component being dragged, the change detector said
//                             nothing had moved, and a three-radii sphere stood.
//
//   sites are elements too    `<site>` carries the same `type` enum and the same
//                             `size` as `<geom>`, and nothing but geom had any
//                             scale policy at all. A sized element that is not a
//                             geom is the case a hand-written list of "things
//                             with a size" gets wrong, so it has its own tests.
//
//   nothing is left over      The set of elements that carry both a transform
//                             and a size is read off the schema, and every
//                             member of it is classified. A MuJoCo release that
//                             adds one fails here by name rather than silently
//                             falling through to the refusal.
//
//   a refusal is visible      A mesh geom has no scale mapping at all, and its
//                             handle used to move and do nothing whatever: no
//                             edit, no snap, no explanation. It snaps back like
//                             a body and says which element owns the size.
//
//   zero is never authored    A size of zero is not a small geom, it is a model
//                             that will not compile. The level viewport's scale
//                             grid steps in 0.25, which is larger than most of a
//                             robot, so the first drag on a sub-grid geom used to
//                             author exactly that.
//
//   the actor is an element   Every refusal above rides on `PostEditComponentMove`,
//                             and the ordinary level gesture -- select the placed
//                             model, drag the scale handle -- never reached it.
//                             `AActor::PostEditMove` calls that hook on the root
//                             only when the construction script did NOT create it
//                             (`ActorEditor.cpp:323-327`), and a model's root
//                             always is one. So a scaled actor drew every geom
//                             under it stretched while MuJoCo went on simulating
//                             the sphere.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Engine/Blueprint.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "Misc/FileHelper.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"
#include "Misc/ScopeExit.h"

#include "MjParitySupport.h"

#include "MuJoCo/Elements/MjGeom.h"
#include "MuJoCo/Gen/Elements/Geometry/MjSite.gen.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjScalePolicy.h"
#include "MuJoCo/Spec/MjSpecRef.h"

THIRD_PARTY_INCLUDES_START
#include "reflect.h"
THIRD_PARTY_INCLUDES_END

#include <string>

namespace MjScalePolicyTests
{
using namespace urlab::spec;

/** A schema name, which reflection hands out as a view rather than a string. */
FString NameOfType(psm::ElementType Type)
{
	const std::string Name(psm::reflect::Describe(Type).name);
	return FString(UTF8_TO_TCHAR(Name.c_str()));
}

/** One body, one named sphere geom, one named site. */
const TCHAR* const Model = TEXT(R"(<mujoco model="scale_policy">
  <worldbody>
    <body name="link" pos="0.1 0.2 0.3">
      <geom name="ball" type="sphere" size="0.05"/>
      <site name="mount" type="capsule" size="0.01 0.05"/>
    </body>
  </worldbody>
</mujoco>
)");

/** The template whose MJCF name is `MjName`, of any element type. */
UMjNodeComponent* Named(UBlueprint& Blueprint, const TCHAR* MjName)
{
	for (USCS_Node* Node : Blueprint.SimpleConstructionScript->GetAllNodes())
	{
		UMjNodeComponent* const Element = Node != nullptr ? Cast<UMjNodeComponent>(Node->ComponentTemplate) : nullptr;
		if (Element != nullptr && Element->MjName.IsSet() && Element->MjName.GetValue() == MjName)
		{
			return Element;
		}
	}
	return nullptr;
}

/** The SCS node holding `Element`. */
USCS_Node* NodeOf(UBlueprint& Blueprint, const UMjNodeComponent& Element)
{
	for (USCS_Node* Node : Blueprint.SimpleConstructionScript->GetAllNodes())
	{
		if (Node != nullptr && Node->ComponentTemplate == &Element)
		{
			return Node;
		}
	}
	return nullptr;
}

/**
 * A brand new element under `link`, exactly as the components panel adds one.
 *
 * Never registered and never synced, so it has no cached baseline -- which is
 * the state the lock used to be skipped in, and the only state in which the
 * defect reproduced.
 */
UMjNodeComponent* AddCold(FAutomationTestBase& Test, UBlueprint& Blueprint, UClass* Class, const TCHAR* Label)
{
	UMjNodeComponent* const Parent = Named(Blueprint, TEXT("link"));
	USCS_Node* const ParentNode = Parent != nullptr ? NodeOf(Blueprint, *Parent) : nullptr;
	if (ParentNode == nullptr)
	{
		Test.AddError(TEXT("the fixture has no body to add under"));
		return nullptr;
	}
	USCS_Node* const Added = Blueprint.SimpleConstructionScript->CreateNode(Class, FName(Label));
	if (Added == nullptr)
	{
		Test.AddError(FString::Printf(TEXT("could not create a %s node"), *Class->GetName()));
		return nullptr;
	}
	ParentNode->AddChildNode(Added);
	return Cast<UMjNodeComponent>(Added->ComponentTemplate);
}

/** Drag the scale handle, through the same hook both editor viewports call. */
void DragScale(UMjNodeComponent& Element, const FVector& Scale)
{
	Element.SetRelativeScale3D(Scale);
	Element.PostEditComponentMove(/*bFinished=*/true);
}

UWorld* ScratchWorld()
{
	return UWorld::CreateWorld(EWorldType::Editor, /*bInformEngineOfWorld=*/false,
		FName(*FString::Printf(TEXT("MjScaleWorld_%s"), *FGuid::NewGuid().ToString(EGuidFormats::Digits))));
}

/** The component of type `T` a spawned actor carries under the MJCF name `MjName`. */
template <class T>
T* ComponentNamed(AActor& Actor, const TCHAR* MjName)
{
	TArray<T*> Found;
	Actor.GetComponents(Found);
	for (T* const One : Found)
	{
		if (One != nullptr && One->MjName.IsSet() && One->MjName.GetValue() == MjName)
		{
			return One;
		}
	}
	return nullptr;
}

/** The element's authored `size`, or an empty array when it authored none. */
TArray<double> AuthoredSize(const UMjNodeComponent& Element)
{
	if (const UMjGeomBase* const Geom = Cast<UMjGeomBase>(&Element))
	{
		return Geom->Size.Get(TArray<double>());
	}
	if (const UMjSite* const Site = Cast<UMjSite>(&Element))
	{
		return Site->Size.Get(TArray<double>());
	}
	return TArray<double>();
}
} // namespace MjScalePolicyTests

// ---------------------------------------------------------------------------
// The cold baseline, which is where the sphere got away
// ---------------------------------------------------------------------------

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjColdBaselineGeomSnapsUniform,
	"URLab.Preview.ColdBaselineGeomSnapsUniform",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjColdBaselineGeomSnapsUniform::RunTest(const FString& Parameters)
{
	using namespace MjScalePolicyTests;

	UBlueprint* const Blueprint =
		MjParitySupport::ParseFixture(*this, TEXT("MjScaleCold"), TEXT("cold geom"), Model, TEXT("<inline>"));
	if (Blueprint == nullptr)
	{
		return false;
	}

	UMjNodeComponent* const Fresh = AddCold(*this, *Blueprint, UMjGeom::StaticClass(), TEXT("FreshGeom"));
	if (Fresh == nullptr)
	{
		return false;
	}

	// Nothing authored: no size, no type, no baseline. A geom in exactly this
	// state compiles as MuJoCo's default sphere, so a non-uniform scale on it
	// is a preview of a shape that will not exist.
	TestEqual(TEXT("the fresh geom authors no size to begin with"), AuthoredSize(*Fresh).Num(), 0);

	DragScale(*Fresh, FVector(2.0, 1.0, 1.0));

	const FVector Scale = Fresh->GetRelativeScale3D();
	TestEqual(TEXT("a sphere's Y follows its X"), Scale.Y, Scale.X);
	TestEqual(TEXT("a sphere's Z follows its X"), Scale.Z, Scale.X);

	const TArray<double> Size = AuthoredSize(*Fresh);
	TestEqual(TEXT("the drag authored a sphere's one radius"), Size.Num(), 1);
	if (Size.Num() == 1)
	{
		// Half of the component scale, which is the table's own factor: an
		// engine primitive is a metre across, so a scale of 2 is a one-metre
		// radius.
		TestEqual(TEXT("the authored radius is the dragged scale"), Size[0], 1.0);
	}

	return !HasAnyErrors();
}

// ---------------------------------------------------------------------------
// A sized element that is not a geom
// ---------------------------------------------------------------------------

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSizedSiteHonoursTypeLock,
	"URLab.Preview.SizedSiteHonoursTypeLock",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSizedSiteHonoursTypeLock::RunTest(const FString& Parameters)
{
	using namespace MjScalePolicyTests;

	UBlueprint* const Blueprint =
		MjParitySupport::ParseFixture(*this, TEXT("MjScaleSite"), TEXT("sized site"), Model, TEXT("<inline>"));
	if (Blueprint == nullptr)
	{
		return false;
	}

	// An authored capsule site: two size values, X and Y locked together.
	UMjNodeComponent* const Site = Named(*Blueprint, TEXT("mount"));
	if (Site == nullptr)
	{
		AddError(TEXT("the fixture's site was not imported"));
		return false;
	}
	TestTrue(TEXT("a site's size is edited by the scale handle"), Site->HasScaleMapping());

	FVector FromSpec = FVector::ZeroVector;
	if (TestTrue(TEXT("a site previews at the scale its size implies"), Site->TryPreviewScaleFromSpec(FromSpec)))
	{
		TestEqual(TEXT("the capsule's radius drives X"), FromSpec.X, 0.02);
		TestEqual(TEXT("the capsule's half-length drives Z"), FromSpec.Z, 0.1);
	}

	DragScale(*Site, FVector(0.04, 0.005, 0.1));

	const FVector Scale = Site->GetRelativeScale3D();
	TestEqual(TEXT("a capsule site's Y follows its X"), Scale.Y, Scale.X);

	const TArray<double> Size = AuthoredSize(*Site);
	TestEqual(TEXT("a capsule authors the two values its type reads"), Size.Num(), 2);
	if (Size.Num() == 2)
	{
		TestEqual(TEXT("the authored radius is the dragged X"), Size[0], 0.02);
		TestEqual(TEXT("the authored half-length is the dragged Z"), Size[1], 0.05);
	}

	// And the same cold case the geom has: a site added by hand, never synced,
	// with no size of its own. This is the case a policy written around geoms
	// would have missed entirely.
	UMjNodeComponent* const Fresh = AddCold(*this, *Blueprint, UMjSite::StaticClass(), TEXT("FreshSite"));
	if (Fresh == nullptr)
	{
		return false;
	}
	DragScale(*Fresh, FVector(3.0, 1.0, 1.0));

	const FVector FreshScale = Fresh->GetRelativeScale3D();
	TestEqual(TEXT("a cold site's Y follows its X"), FreshScale.Y, FreshScale.X);
	TestEqual(TEXT("a cold site's Z follows its X"), FreshScale.Z, FreshScale.X);

	const TArray<double> FreshSize = AuthoredSize(*Fresh);
	TestEqual(TEXT("a cold sphere site authors one radius"), FreshSize.Num(), 1);

	return !HasAnyErrors();
}

// ---------------------------------------------------------------------------
// An element with no size at all
// ---------------------------------------------------------------------------

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjUnsizedElementSnapsToUnit,
	"URLab.Preview.UnsizedElementSnapsToUnit",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjUnsizedElementSnapsToUnit::RunTest(const FString& Parameters)
{
	using namespace MjScalePolicyTests;

	UBlueprint* const Blueprint =
		MjParitySupport::ParseFixture(*this, TEXT("MjScaleBody"), TEXT("unsized body"), Model, TEXT("<inline>"));
	if (Blueprint == nullptr)
	{
		return false;
	}

	UMjNodeComponent* const Body = Named(*Blueprint, TEXT("link"));
	if (Body == nullptr)
	{
		AddError(TEXT("the fixture's body was not imported"));
		return false;
	}
	TestFalse(TEXT("a body's scale edits nothing"), Body->HasScaleMapping());

	const FVector Before = Body->GetRelativeLocation();
	DragScale(*Body, FVector(2.0, 3.0, 4.0));

	// The refusal: a scaled body distorts every descendant's picture while the
	// simulation goes on using the numbers the body did not change, so the
	// scale goes back rather than standing as a lie.
	TestEqual(TEXT("an unsized element refuses a scale"), Body->GetRelativeScale3D(), FVector::OneVector);

	// And the refusal is a scale answer only: the position it was dragged to on
	// some other day is not re-authored by it.
	TestEqual(TEXT("refusing a scale does not move the element"), Body->GetRelativeLocation(), Before);

	return !HasAnyErrors();
}

// ---------------------------------------------------------------------------
// A shape whose size is not a scale
// ---------------------------------------------------------------------------

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjMeshGeomRefusesAScale,
	"URLab.Preview.MeshGeomRefusesAScale",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjMeshGeomRefusesAScale::RunTest(const FString& Parameters)
{
	using namespace MjScalePolicyTests;

	// A mesh geom and a sphere in one document, so the sphere is the control:
	// the same gesture on it is an ordinary size edit, which is what makes the
	// mesh geom's refusal a property of the shape rather than of the harness.
	const TCHAR* const MeshModel = TEXT(R"(<mujoco model="mesh_scale">
  <asset>
    <mesh name="shell" file="shell.stl"/>
  </asset>
  <worldbody>
    <body name="link">
      <geom name="hull" type="mesh" mesh="shell"/>
      <geom name="ball" type="sphere" size="0.05"/>
    </body>
  </worldbody>
</mujoco>
)");

	UBlueprint* const Blueprint =
		MjParitySupport::ParseFixture(*this, TEXT("MjScaleMesh"), TEXT("mesh geom"), MeshModel, TEXT("<inline>"));
	if (Blueprint == nullptr)
	{
		return false;
	}

	UMjNodeComponent* const Hull = Named(*Blueprint, TEXT("hull"));
	UMjNodeComponent* const Ball = Named(*Blueprint, TEXT("ball"));
	if (Hull == nullptr || Ball == nullptr)
	{
		AddError(TEXT("the fixture's geoms were not imported"));
		return false;
	}
	TestFalse(TEXT("a mesh geom's size is not the scale handle's to edit"), Hull->HasScaleMapping());

	DragScale(*Hull, FVector(3.0, 1.0, 0.5));

	// The refusal, which used to be a handle that moved and did nothing at all.
	// A mesh geom's picture rides on the mesh asset's own scale, so the scale the
	// spec implies for the geom component is one.
	TestEqual(TEXT("a mesh geom's scale goes back to what the spec implies"),
		Hull->GetRelativeScale3D(), FVector::OneVector);
	TestEqual(TEXT("and the drag authored no size"), AuthoredSize(*Hull).Num(), 0);

	if (TestEqual(TEXT("the refusal is explained on the element"), Hull->PreviewProblems.Num(), 1))
	{
		const FString Reported = Hull->PreviewProblems[0];
		TestTrue(FString::Printf(TEXT("it names the element that does own the size, got '%s'"), *Reported),
			Reported.Contains(TEXT("<mesh>")));
	}

	// The control. Without it, a run in which every drag was inert would pass the
	// assertions above without the refusal existing at all.
	DragScale(*Ball, FVector(0.4, 0.4, 0.4));
	const TArray<double> Size = AuthoredSize(*Ball);
	if (TestEqual(TEXT("a sphere in the same document still authors its radius"), Size.Num(), 1))
	{
		TestEqual(TEXT("and the radius is half the dragged scale"), Size[0], 0.2);
	}
	TestEqual(TEXT("a shape whose size a scale expresses has nothing to explain"), Ball->PreviewProblems.Num(), 0);

	return !HasAnyErrors();
}

// ---------------------------------------------------------------------------
// The size a drag must never author
// ---------------------------------------------------------------------------

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjNonPositiveSizeIsRefused,
	"URLab.Preview.NonPositiveSizeIsRefused",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjNonPositiveSizeIsRefused::RunTest(const FString& Parameters)
{
	using namespace MjScalePolicyTests;

	UBlueprint* const Blueprint =
		MjParitySupport::ParseFixture(*this, TEXT("MjScaleZero"), TEXT("zero size"), Model, TEXT("<inline>"));
	if (Blueprint == nullptr)
	{
		return false;
	}

	UMjNodeComponent* const Ball = Named(*Blueprint, TEXT("ball"));
	if (Ball == nullptr)
	{
		AddError(TEXT("the fixture's geom was not imported"));
		return false;
	}

	// The authored radius, and the scale it previews at: 0.05 m against a 0.5 m
	// engine primitive is a scale of 0.1, which is well under the level
	// viewport's default 0.25 scale grid. One grid step down and the drag arrives
	// here asking for a radius of zero.
	const TArray<double> Before = AuthoredSize(*Ball);
	if (!TestEqual(TEXT("the geom starts with the radius the document authored"), Before.Num(), 1))
	{
		return false;
	}
	TestEqual(TEXT("which is 0.05"), Before[0], 0.05);

	DragScale(*Ball, FVector::ZeroVector);

	const TArray<double> After = AuthoredSize(*Ball);
	if (TestEqual(TEXT("the collapsing drag authored no new size"), After.Num(), 1))
	{
		TestEqual(TEXT("the radius is the one the document authored, untouched"), After[0], 0.05);
	}
	TestEqual(TEXT("and the component goes back to the scale the spec implies"),
		Ball->GetRelativeScale3D(), FVector(0.1, 0.1, 0.1));

	if (TestEqual(TEXT("the refusal is explained on the element"), Ball->PreviewProblems.Num(), 1))
	{
		const FString Reported = Ball->PreviewProblems[0];
		TestTrue(FString::Printf(TEXT("it says the size is unchanged, got '%s'"), *Reported),
			Reported.Contains(TEXT("size is unchanged")));
		TestTrue(FString::Printf(TEXT("it names the scale grid, got '%s'"), *Reported),
			Reported.Contains(TEXT("scale grid")));
	}

	// A negative scale asks for a negative size, which MuJoCo refuses for the
	// same reason: mirroring a geom is not an edit of its radius.
	DragScale(*Ball, FVector(-0.5, -0.5, -0.5));
	const TArray<double> Mirrored = AuthoredSize(*Ball);
	if (TestEqual(TEXT("a mirrored drag leaves the one authored radius"), Mirrored.Num(), 1))
	{
		TestEqual(TEXT("and does not author a negative one"), Mirrored[0], 0.05);
	}

	// And an ordinary drag still authors, so the refusal is the sign of the size
	// rather than the write-back having been switched off.
	DragScale(*Ball, FVector(0.4, 0.4, 0.4));
	const TArray<double> Grown = AuthoredSize(*Ball);
	if (TestEqual(TEXT("a positive drag still authors one radius"), Grown.Num(), 1))
	{
		TestEqual(TEXT("and it is half the dragged scale"), Grown[0], 0.2);
	}
	TestEqual(TEXT("which withdraws the refusal"), Ball->PreviewProblems.Num(), 0);

	return !HasAnyErrors();
}

// ---------------------------------------------------------------------------
// The gesture that reached no hook at all
// ---------------------------------------------------------------------------

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjActorScaleDoesNotDistortTheModel,
	"URLab.Preview.AnActorScaleDoesNotDistortTheModel",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjActorScaleDoesNotDistortTheModel::RunTest(const FString& Parameters)
{
	using namespace MjScalePolicyTests;

	UBlueprint* const Blueprint =
		MjParitySupport::ParseFixture(*this, TEXT("MjScaleActor"), TEXT("actor scale"), Model, TEXT("<inline>"));
	if (Blueprint == nullptr)
	{
		return false;
	}
	FKismetEditorUtilities::CompileBlueprint(Blueprint);

	UWorld* const World = ScratchWorld();
	if (!TestNotNull(TEXT("scratch world"), World))
	{
		return false;
	}
	ON_SCOPE_EXIT
	{
		World->DestroyWorld(false);
	};

	AActor* const Actor = World->SpawnActor<AActor>(Blueprint->GeneratedClass);
	if (!TestNotNull(TEXT("the placed model"), Actor))
	{
		return false;
	}
	USceneComponent* const Root = Actor->GetRootComponent();
	UMjNodeComponent* const Ball = ComponentNamed<UMjNodeComponent>(*Actor, TEXT("ball"));
	if (!TestNotNull(TEXT("the model's root"), Root) || !TestNotNull(TEXT("the placed geom"), Ball))
	{
		return false;
	}

	// The whole reason the hook never arrived, asserted rather than assumed: the
	// engine skips a root the construction script made, and this is one.
	TestTrue(TEXT("the model's root is a construction-script component"), Root->IsCreatedByConstructionScript());
	TestNotNull(TEXT("and it is an element, so the refusal applies to it"), Cast<UMjNodeComponent>(Root));

	const TArray<double> Before = AuthoredSize(*Ball);
	const FVector Uniform = Ball->GetComponentScale();
	if (!TestEqual(TEXT("the geom starts at the scale its radius implies"), Uniform, FVector(0.1, 0.1, 0.1)))
	{
		return false;
	}

	// The gesture: scale the placed actor, which lands on its root component and
	// multiplies every element under it.
	Root->SetRelativeScale3D(FVector(3.0, 1.0, 1.0));
	TestEqual(TEXT("the drag really did reach the root"), Root->GetRelativeScale3D(), FVector(3.0, 1.0, 1.0));
	TestFalse(TEXT("and the geom really was drawn distorted before the move finished"),
		Ball->GetComponentScale().Equals(Uniform, 1e-4));

	// What the level viewport does when the drag ends.
	Actor->PostEditMove(/*bFinished=*/true);

	UMjNodeComponent* const Settled = ComponentNamed<UMjNodeComponent>(*Actor, TEXT("ball"));
	if (!TestNotNull(TEXT("the geom after the move"), Settled))
	{
		return false;
	}
	TestEqual(TEXT("a scale MJCF cannot express goes back to one"),
		Actor->GetRootComponent()->GetRelativeScale3D(), FVector::OneVector);
	TestTrue(FString::Printf(TEXT("so the sphere is drawn as a sphere again, got %s"),
				 *Settled->GetComponentScale().ToString()),
		Settled->GetComponentScale().Equals(Uniform, 1e-4));

	// And the refusal is a refusal, not a write: the drag must not have authored
	// the distortion into the model either.
	const TArray<double> After = AuthoredSize(*Settled);
	if (TestEqual(TEXT("the geom still authors its one radius"), After.Num(), Before.Num()))
	{
		for (int32 Index = 0; Index < After.Num(); ++Index)
		{
			TestEqual(TEXT("unchanged by the actor gesture"), After[Index], Before[Index]);
		}
	}

	// The control. Moving the actor is legal and must still be free: an actor
	// translation is not a lie about anything, and if the hook above refused
	// everything the assertions would pass for the wrong reason.
	Actor->SetActorLocation(FVector(100.0, 200.0, 300.0));
	Actor->PostEditMove(/*bFinished=*/true);
	TestTrue(TEXT("moving a placed model is left alone"),
		Actor->GetActorLocation().Equals(FVector(100.0, 200.0, 300.0), 1e-3));

	return !HasAnyErrors();
}

// ---------------------------------------------------------------------------
// Arity, spec-side
// ---------------------------------------------------------------------------

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSizeArityReported,
	"URLab.Spec.SizeArityReported",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSizeArityReported::RunTest(const FString& Parameters)
{
	using namespace MjScalePolicyTests;

	// Three radii on a sphere. MuJoCo takes the first, carries the other two into
	// the compiled model where they decide nothing, and says nothing about it --
	// which is how a hand-edited file, or the array widget, produces a shape the
	// author did not mean and never finds out.
	const TCHAR* const OverLong = TEXT(R"(<mujoco model="arity">
  <worldbody>
    <geom name="ball" type="sphere" size="0.1 0.2 0.3"/>
    <geom name="brick" type="box" size="0.1 0.2 0.3"/>
    <site name="dot" type="capsule" size="0.01 0.05 0.09"/>
  </worldbody>
</mujoco>
)");

	UBlueprint* const Blueprint =
		MjParitySupport::ParseFixture(*this, TEXT("MjArity"), TEXT("arity"), OverLong, TEXT("<inline>"));
	if (Blueprint == nullptr)
	{
		return false;
	}

	const FSpecRef Spec = FSpecRef::OverBlueprint(*Blueprint);
	TArray<FMjSpecDiagnostic> Diagnostics;
	urlab::spec::FMjBuiltSpec Built = urlab::spec::BuildSpec(Spec, Diagnostics);
	if (!TestNotNull(TEXT("the over-long document still builds"), Built.Spec))
	{
		return false;
	}

	// The box is the control: three values is exactly what a box reads, so a
	// check that reported it too would be reporting length rather than arity.
	// The site is there because arity is a property of the TYPE, not of geoms.
	int32 Reported = 0;
	bool bNamedTheSphere = false;
	bool bNamedTheSite = false;
	for (const FMjSpecDiagnostic& Diagnostic : Diagnostics)
	{
		if (!Diagnostic.Message.Contains(TEXT("size values")))
		{
			continue;
		}
		++Reported;
		bNamedTheSphere = bNamedTheSphere || Diagnostic.Message.Contains(TEXT("a sphere reads 1"));
		bNamedTheSite = bNamedTheSite || Diagnostic.Message.Contains(TEXT("a capsule reads 2"));
	}
	TestEqual(TEXT("two over-long sizes are reported"), Reported, 2);
	TestTrue(TEXT("the sphere is reported against its own arity"), bNamedTheSphere);
	TestTrue(TEXT("the site is reported against its own arity"), bNamedTheSite);

	// And the compile receives a legal size: the value MuJoCo itself would
	// receive from the same document, which is the one the shape is read out of.
	// The extra entries are reported and LEFT: `checksize` bounds its loop by the
	// arity (user_objects.cc:163) and `CopyObjects` copies all three slots
	// whatever the type reads (user_model.cc:3055), so dropping them would
	// compile a different model from stock MuJoCo's out of a document stock
	// MuJoCo accepts -- and the two models being the same one is what this
	// project's correctness rests on.
	mjVFS Vfs;
	mj_defaultVFS(&Vfs);
	mjModel* const Compiled = mj_compile(Built.Spec, &Vfs);
	mj_deleteVFS(&Vfs);
	if (!TestNotNull(TEXT("the reported document still compiles"), Compiled))
	{
		return false;
	}

	const int32 Ball = mj_name2id(Compiled, mjOBJ_GEOM, "ball");
	if (TestTrue(TEXT("the reported geom compiled"), Ball >= 0))
	{
		TestEqual(TEXT("the radius the compiler uses is the first value"), Compiled->geom_size[3 * Ball], 0.1);
		TestEqual(TEXT("the slots a sphere never reads are carried as MuJoCo carries them"),
			Compiled->geom_size[3 * Ball + 1], 0.2);
	}

	const int32 Brick = mj_name2id(Compiled, mjOBJ_GEOM, "brick");
	if (TestTrue(TEXT("the control geom compiled"), Brick >= 0))
	{
		// A box reads all three, so nothing about it was ever in question.
		TestEqual(TEXT("a box keeps its second half-extent"), Compiled->geom_size[3 * Brick + 1], 0.2);
		TestEqual(TEXT("a box keeps its third half-extent"), Compiled->geom_size[3 * Brick + 2], 0.3);
	}

	const int32 Dot = mj_name2id(Compiled, mjOBJ_SITE, "dot");
	if (TestTrue(TEXT("the site compiled"), Dot >= 0))
	{
		TestEqual(TEXT("a capsule site's radius is its first value"), Compiled->site_size[3 * Dot], 0.01);
		TestEqual(TEXT("a capsule site's half-length is its second"), Compiled->site_size[3 * Dot + 1], 0.05);
	}
	mj_deleteModel(Compiled);

	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSizeArityResolvesAnInheritedType,
	"URLab.Spec.SizeArityResolvesAnInheritedType",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSizeArityResolvesAnInheritedType::RunTest(const FString& Parameters)
{
	using namespace MjScalePolicyTests;

	// A nested `<default>` tree where the inner classes author a `size` and no
	// `type`: they inherit `type="box"`, so three values is exactly right for
	// every one of them. This is the shape a menagerie model is written in, and
	// it is the shape a checker that reads a partial's OWN type gets wrong --
	// an unauthored `type` reads as the schema's `sphere`, whose arity is one,
	// and every class partial in the document is accused of authoring two size
	// values too many.
	const TCHAR* const Nested = TEXT(R"(<mujoco model="nested">
  <default>
    <default class="top">
      <geom type="box" size="0.1 0.1 0.1"/>
      <default class="mid">
        <geom size="0.05 0.05 0.05"/>
        <default class="leaf">
          <geom size="0.025 0.025 0.025"/>
        </default>
      </default>
    </default>
  </default>
  <worldbody>
    <body name="link" childclass="top">
      <geom name="outer"/>
      <geom name="inner" class="leaf"/>
    </body>
  </worldbody>
</mujoco>
)");

	UBlueprint* const Blueprint =
		MjParitySupport::ParseFixture(*this, TEXT("MjArityNested"), TEXT("nested"), Nested, TEXT("<inline>"));
	if (Blueprint == nullptr)
	{
		return false;
	}

	const FSpecRef Spec = FSpecRef::OverBlueprint(*Blueprint);
	TArray<FMjSpecDiagnostic> Diagnostics;
	urlab::spec::FMjBuiltSpec Built = urlab::spec::BuildSpec(Spec, Diagnostics);
	if (!TestNotNull(TEXT("the nested-defaults document builds"), Built.Spec))
	{
		return false;
	}

	TArray<FString> Accused;
	for (const FMjSpecDiagnostic& Diagnostic : Diagnostics)
	{
		if (Diagnostic.Message.Contains(TEXT("size values")))
		{
			Accused.Add(Diagnostic.Message);
		}
	}
	TestEqual(FString::Printf(TEXT("nothing in a legal nested-defaults document is reported, got: %s"),
				  *FString::Join(Accused, TEXT(" / "))),
		Accused.Num(), 0);

	// And the document really did compile the boxes it says it did, so the
	// silence above is the checker agreeing rather than the fixture being empty.
	mjVFS Vfs;
	mj_defaultVFS(&Vfs);
	mjModel* const Compiled = mj_compile(Built.Spec, &Vfs);
	mj_deleteVFS(&Vfs);
	if (!TestNotNull(TEXT("the nested-defaults document compiles"), Compiled))
	{
		return false;
	}
	const int32 Inner = mj_name2id(Compiled, mjOBJ_GEOM, "inner");
	if (TestTrue(TEXT("the leaf-class geom compiled"), Inner >= 0))
	{
		TestEqual(TEXT("it inherited its type from three classes up"),
			static_cast<int32>(Compiled->geom_type[Inner]), static_cast<int32>(mjGEOM_BOX));
		TestEqual(TEXT("and reads all three of its half-extents"), Compiled->geom_size[3 * Inner + 2], 0.025);
	}
	mj_deleteModel(Compiled);

	// The fixture the false positive was found on, by name. The inline document
	// above states the shape; this one is the file a reader would go and open.
	const FString Fixture = FPaths::Combine(MjParitySupport::ParityDir(), TEXT("defaults_nested.xml"));
	FString Authored;
	if (TestTrue(FString::Printf(TEXT("the nested-defaults fixture is on disk at '%s'"), *Fixture),
			FFileHelper::LoadFileToString(Authored, *Fixture)))
	{
		UBlueprint* const FromFile =
			MjParitySupport::ParseFixture(*this, TEXT("MjArityFixture"), TEXT("defaults_nested"), Authored, Fixture);
		if (FromFile != nullptr)
		{
			TArray<FMjSpecDiagnostic> FixtureDiagnostics;
			urlab::spec::FMjBuiltSpec FixtureBuilt =
				urlab::spec::BuildSpec(FSpecRef::OverBlueprint(*FromFile), FixtureDiagnostics);
			TestNotNull(TEXT("the fixture builds"), FixtureBuilt.Spec);

			TArray<FString> FixtureAccused;
			for (const FMjSpecDiagnostic& Diagnostic : FixtureDiagnostics)
			{
				if (Diagnostic.Message.Contains(TEXT("size values")))
				{
					FixtureAccused.Add(Diagnostic.Message);
				}
			}
			TestEqual(FString::Printf(TEXT("defaults_nested.xml is not accused of anything, got: %s"),
						  *FString::Join(FixtureAccused, TEXT(" / "))),
				FixtureAccused.Num(), 0);
		}
	}

	return !HasAnyErrors();
}

// ---------------------------------------------------------------------------
// The derivation itself
// ---------------------------------------------------------------------------

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSizedElementsAreClassified,
	"URLab.Spec.SizedElementsAreClassified",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSizedElementsAreClassified::RunTest(const FString& Parameters)
{
	using namespace MjScalePolicyTests;

	// Read off the schema, not listed here. The list below is what the schema
	// says TODAY, and it is spelled out so that a MuJoCo release which adds a
	// sized, placeable element fails this test by name -- the failure being the
	// point, because the alternative is a new element silently inheriting the
	// refusal and nobody deciding whether that is right for it.
	const TArray<psm::ElementType> Sized = MjSizedTransformElements();

	TArray<FString> Names;
	for (const psm::ElementType Type : Sized)
	{
		Names.Add(NameOfType(Type));
	}
	AddInfo(FString::Printf(TEXT("sized transform-bearing elements: %s"), *FString::Join(Names, TEXT(", "))));

	TestEqual(TEXT("three schema elements carry both a transform and a size"), Sized.Num(), 3);
	TestTrue(TEXT("<geom> is one of them"), Sized.Contains(psm::ElementType::Geom));
	TestTrue(TEXT("<site> is one of them"), Sized.Contains(psm::ElementType::Site));
	TestTrue(TEXT("<composite> is one of them"), Sized.Contains(psm::ElementType::Composite));

	// Every one of them is classified, and none falls through to "no size".
	for (const psm::ElementType Type : Sized)
	{
		TestTrue(FString::Printf(TEXT("%s is not treated as unsized"), *NameOfType(Type)),
			MjScalePolicyFor(Type) != EMjScalePolicy::Unsized);
	}

	// A geom and a site are the same question with the same answer: their
	// `size` is read through MuJoCo's GeomType.
	TestTrue(TEXT("<geom> is shaped by its type"), MjScalePolicyFor(psm::ElementType::Geom) == EMjScalePolicy::GeomShaped);
	TestTrue(TEXT("<site> is shaped by its type"), MjScalePolicyFor(psm::ElementType::Site) == EMjScalePolicy::GeomShaped);

	// A composite's `size` sizes a lattice, not a shape: its `type` is
	// CompositeType, and no scale of the component expresses it.
	TestTrue(TEXT("<composite>'s size is not a scale"),
		MjScalePolicyFor(psm::ElementType::Composite) == EMjScalePolicy::SizeIsNotAScale);

	// A body is the unsized case, and the reason the refusal exists.
	TestTrue(TEXT("<body> has no size"), MjScalePolicyFor(psm::ElementType::Body) == EMjScalePolicy::Unsized);

	// The arity table is MuJoCo's own, spot-checked at both ends.
	TestEqual(TEXT("a sphere reads one size value"), MjSizeArityFor(EMjGeomType::sphere), 1);
	TestEqual(TEXT("a capsule reads two"), MjSizeArityFor(EMjGeomType::capsule), 2);
	TestEqual(TEXT("a box reads three"), MjSizeArityFor(EMjGeomType::box), 3);
	TestEqual(TEXT("a plane reads three"), MjSizeArityFor(EMjGeomType::plane), 3);
	TestEqual(TEXT("a mesh reads none"), MjSizeArityFor(EMjGeomType::mesh), 0);

	return !HasAnyErrors();
}

#endif // URLAB_MJ_GEN && WITH_EDITOR
