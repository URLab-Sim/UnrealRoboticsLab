// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// Does the editor draw what the compiler will build?
//
// Two claims are worth a test rather than a screenshot. The first is that the
// drawing reads EFFECTIVE values: a site whose `size` lives on its default
// class has to preview at the inherited size, or the preview is a picture of a
// model nobody asked for. The second is that an authored angle is read in the
// unit the document declares -- MJCF's default is degrees, the compiled model
// is always radians, and the ported drawing is the only place where that
// difference exists at all. A 57x error there looks like a plausible arc.
//
// The drawing is captured through a counting FPrimitiveDrawInterface rather
// than a viewport, so what is asserted is the geometry that was emitted.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Editor/UnrealEdEngine.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"
#include "PrimitiveDrawInterface.h"
#include "UnrealEdGlobals.h"

#include "MjElementVisualizers.h"
#include "MuJoCo/Gen/Elements/Cameras/MjCamera.gen.h"
#include "MuJoCo/Gen/Elements/Cameras/MjLight.gen.h"
#include "MuJoCo/Gen/Elements/Geometry/MjSite.gen.h"
#include "MuJoCo/Gen/Elements/Joints/MjJoint.gen.h"
#include "MuJoCo/Spec/MjSpecRef.h"

namespace MjVisualizerTests
{
using namespace urlab::spec;

/**
 * Every line the drawing emitted, as endpoints.
 *
 * Both line calls are recorded because the engine's own helpers pick between
 * them for reasons of their own -- `DrawCircle` and `DrawArc` are translucent,
 * a bare `DrawLine` is not -- and a test that watched only one would silently
 * assert over an empty array.
 */
class FCountingPDI final : public FPrimitiveDrawInterface
{
public:
	FCountingPDI() : FPrimitiveDrawInterface(nullptr) {}

	virtual bool IsHitTesting() override { return false; }
	virtual void SetHitProxy(HHitProxy*) override {}
	virtual void RegisterDynamicResource(FDynamicPrimitiveResource*) override {}
	virtual void AddReserveLines(uint8, int32, bool, bool) override {}

	virtual void DrawSprite(const FVector&, float, float, const FTexture*, const FLinearColor&, uint8,
		float, float, float, float, uint8, float) override
	{
	}

	virtual void DrawLine(const FVector& Start, const FVector& End, const FLinearColor&, uint8, float,
		float, bool) override
	{
		Points.Add(Start);
		Points.Add(End);
	}

	virtual void DrawTranslucentLine(const FVector& Start, const FVector& End, const FLinearColor&, uint8,
		float, float, bool) override
	{
		Points.Add(Start);
		Points.Add(End);
		Curved.Add(Start);
		Curved.Add(End);
	}

	virtual void DrawPoint(const FVector& Position, const FLinearColor&, float, uint8) override
	{
		Points.Add(Position);
	}

	virtual int32 DrawMesh(const FMeshBatch&) override { return 0; }

	/** Every endpoint, whichever call produced it. */
	TArray<FVector> Points;

	/** Only the endpoints of the circles and arcs. */
	TArray<FVector> Curved;
};

/** A spec over a live actor, which is the graph the Blueprint viewport draws. */
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

bool Parse(FAutomationTestBase& Test, FScratchDoc& Doc, const TCHAR* Xml)
{
	const FString Stem = FString::Printf(TEXT("MjVis_%s"), *FGuid::NewGuid().ToString(EGuidFormats::Digits));
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
	const FString Path =
		FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / TEXT("URLab/TestModels") / Stem);
	if (!MjParseIntoActor(*Doc.Actor, Xml, Path / (Stem + TEXT(".xml"))).IsOk())
	{
		Test.AddError(TEXT("parse failed"));
		return false;
	}
	return true;
}

/** The furthest any drawn endpoint got from `Centre`. */
double FurthestFrom(const TArray<FVector>& Points, const FVector& Centre)
{
	double Furthest = 0.0;
	for (const FVector& Point : Points)
	{
		Furthest = FMath::Max(Furthest, FVector::Dist(Point, Centre));
	}
	return Furthest;
}

}  // namespace MjVisualizerTests

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjElementVisualizerIsRegisteredForEveryElement,
	"URLab.Editor.ElementVisualizerIsRegisteredForEveryElement",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjElementVisualizerIsRegisteredForEveryElement::RunTest(const FString& Parameters)
{
	if (GUnrealEd == nullptr)
	{
		AddError(TEXT("no editor engine, so no visualizer registry to ask"));
		return false;
	}

	// One registration on the base has to answer for every element class, which
	// is the whole reason there is one visualizer instead of five. A joint, a
	// site, a light and a camera are four different generated classes and one
	// of them has a hand subclass over it.
	const TSharedPtr<FComponentVisualizer> ForJoint = GUnrealEd->FindComponentVisualizer(UMjJoint::StaticClass());
	TestTrue(TEXT("a joint has a visualizer"), ForJoint.IsValid());

	for (UClass* Class : {UMjSite::StaticClass(), UMjLight::StaticClass(), UMjCameraBase::StaticClass()})
	{
		const TSharedPtr<FComponentVisualizer> Found = GUnrealEd->FindComponentVisualizer(Class);
		TestTrue(FString::Printf(TEXT("%s has a visualizer"), *Class->GetName()), Found.IsValid());
		TestTrue(FString::Printf(TEXT("%s shares the one instance"), *Class->GetName()), Found == ForJoint);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjElementVisualizerDrawsInheritedSize,
	"URLab.Editor.ElementVisualizerDrawsInheritedSize",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjElementVisualizerDrawsInheritedSize::RunTest(const FString& Parameters)
{
	using namespace MjVisualizerTests;

	// The site authors no size at all: every number the preview needs comes off
	// the class it names.
	const TCHAR* const Xml = TEXT(R"(<mujoco model="inherited">
  <default>
    <default class="wide">
      <site size="0.3"/>
    </default>
  </default>
  <worldbody>
    <body name="base">
      <site name="marker" class="wide"/>
    </body>
  </worldbody>
</mujoco>
)");

	FScratchDoc Doc;
	if (!Parse(*this, Doc, Xml))
	{
		return false;
	}

	UMjSite* const Site = Doc.Actor->FindComponentByClass<UMjSite>();
	if (!TestNotNull(TEXT("the site component"), Site))
	{
		return false;
	}
	TestFalse(TEXT("the site authors no size of its own"), Site->Size.IsSet());

	FCountingPDI PDI;
	FMjElementVisualizer Visualizer;
	Visualizer.DrawVisualization(Site, nullptr, &PDI);

	TestTrue(TEXT("the site drew something"), PDI.Points.Num() > 0);

	// 0.3 m is 30 cm. An authored-only read would have fallen back to MuJoCo's
	// 0.005 m default and drawn a half-centimetre dot, so the number is the
	// assertion: this fails loudly if the class chain stops being consulted.
	const double Radius = FurthestFrom(PDI.Points, Site->GetComponentLocation());
	TestTrue(FString::Printf(TEXT("drawn at the inherited radius, got %.2f cm"), Radius),
		Radius > 25.0 && Radius < 35.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjElementVisualizerReadsRangeInAuthoredAngleUnit,
	"URLab.Editor.ElementVisualizerReadsRangeInAuthoredAngleUnit",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjElementVisualizerReadsRangeInAuthoredAngleUnit::RunTest(const FString& Parameters)
{
	using namespace MjVisualizerTests;

	// No <compiler angle=...>, so MJCF's default applies and "90" means ninety
	// degrees. Read as radians it is more than fourteen full turns, and the arc
	// would close on itself.
	const TCHAR* const Xml = TEXT(R"(<mujoco model="degrees">
  <worldbody>
    <body name="base">
      <joint name="hinge" type="hinge" axis="0 0 1" limited="true" range="0 90"/>
      <geom name="mass" type="sphere" size="0.05"/>
    </body>
  </worldbody>
</mujoco>
)");

	FScratchDoc Doc;
	if (!Parse(*this, Doc, Xml))
	{
		return false;
	}

	UMjJoint* const Joint = Doc.Actor->FindComponentByClass<UMjJoint>();
	if (!TestNotNull(TEXT("the joint component"), Joint))
	{
		return false;
	}

	FCountingPDI PDI;
	FMjElementVisualizer Visualizer;
	Visualizer.DrawVisualization(Joint, nullptr, &PDI);

	if (!TestTrue(TEXT("the joint drew a range arc"), PDI.Curved.Num() > 0))
	{
		return false;
	}

	// Every arc point measured against the first one. A ninety-degree sector
	// cannot exceed ninety degrees of spread; ninety radians covers the circle
	// and would reach a hundred and eighty.
	const FVector Anchor = Joint->GetComponentLocation();
	const FVector First = (PDI.Curved[0] - Anchor).GetSafeNormal();
	double WidestDegrees = 0.0;
	for (const FVector& Point : PDI.Curved)
	{
		const FVector Direction = (Point - Anchor).GetSafeNormal();
		const double Dot = FMath::Clamp(FVector::DotProduct(First, Direction), -1.0, 1.0);
		WidestDegrees = FMath::Max(WidestDegrees, FMath::RadiansToDegrees(FMath::Acos(Dot)));
	}
	TestTrue(FString::Printf(TEXT("the arc spans the authored ninety degrees, got %.1f"), WidestDegrees),
		WidestDegrees > 80.0 && WidestDegrees < 100.0);
	return true;
}

#endif  // URLAB_MJ_GEN && WITH_EDITOR
