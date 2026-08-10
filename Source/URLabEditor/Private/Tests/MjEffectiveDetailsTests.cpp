// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// The inherited value the panel puts on an unset row, and what it costs.
//
// Two things are worth pinning. The string, because "0.3 (from visual)" is the
// whole feature and a wrong class name is a confident lie about where a number
// came from. And the number of effective contexts one refresh builds, because
// the natural way to write this row builder -- ask the spec once per attribute
// -- is quadratic in the model, and it is the exact shape of lag this project
// has already paid for once.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"
#include "UObject/PropertyOptional.h"
#include "UObject/UnrealType.h"

#include "MjArrayCustomizations.h"
#include "MjEffectiveDetails.h"
#include "MuJoCo/Gen/Elements/Geometry/MjGeom.gen.h"
#include "MuJoCo/Spec/MjEffective.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjSpecRef.h"

namespace MjEffectiveDetailsTests
{
using namespace urlab::spec;

/**
 * A geom that authors nothing: every number it uses comes off its class.
 *
 * Except the ones no class mentions, which is most of them -- `condim` and
 * `margin` are two -- and which resolve through MuJoCo's own initialised values.
 * `mass` is the third case: MuJoCo computes it, so nothing supplies it.
 */
const TCHAR* const Corpus = TEXT(R"(<mujoco model="inherited">
  <default>
    <default class="visual">
      <geom type="sphere" size="0.3" rgba="1 0 0 1" contype="0"/>
    </default>
  </default>
  <worldbody>
    <body name="base">
      <geom name="shell" class="visual"/>
    </body>
  </worldbody>
</mujoco>
)");

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
	const FString Stem = FString::Printf(TEXT("MjEff_%s"), *FGuid::NewGuid().ToString(EGuidFormats::Digits));
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
	if (!MjParseIntoActor(*Doc.Actor, Corpus, Path / (Stem + TEXT(".xml"))).IsOk())
	{
		Test.AddError(TEXT("parse failed"));
		return false;
	}
	return true;
}

/**
 * The element of type `T` this document names `MjName`.
 *
 * Asking the actor for the first component of a class finds the `<default>`
 * class partial just as readily as the geom in the worldbody -- a partial is a
 * component of the same class -- and the partial is the one that authors the
 * value, so the test would assert the opposite of what it means to.
 */
template <class T>
T* ElementNamed(const AActor& Actor, const TCHAR* MjName)
{
	for (UActorComponent* Component : Actor.GetComponents())
	{
		T* const Typed = Cast<T>(Component);
		if (Typed != nullptr && Typed->MjName.IsSet() && Typed->MjName.GetValue() == MjName)
		{
			return Typed;
		}
	}
	return nullptr;
}

/** The optional property named `Name` on `Node`'s class, or null. */
const FOptionalProperty* OptionalNamed(const UMjNodeComponent& Node, const TCHAR* Name)
{
	return CastField<FOptionalProperty>(Node.GetClass()->FindPropertyByName(FName(Name)));
}

}  // namespace MjEffectiveDetailsTests

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjEffectiveDetailsNamesTheClassThatSuppliedTheValue,
	"URLab.Editor.EffectiveDetailsNamesTheClassThatSuppliedTheValue",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjEffectiveDetailsNamesTheClassThatSuppliedTheValue::RunTest(const FString& Parameters)
{
	using namespace MjEffectiveDetailsTests;

	FScratchDoc Doc;
	if (!Parse(*this, Doc))
	{
		return false;
	}

	UMjGeomBase* const Geom = ElementNamed<UMjGeomBase>(*Doc.Actor, TEXT("shell"));
	if (!TestNotNull(TEXT("the geom component"), Geom))
	{
		return false;
	}
	TestFalse(TEXT("the geom authors no size"), Geom->Size.IsSet());

	const FOptionalProperty* const Size = OptionalNamed(*Geom, TEXT("Size"));
	if (!TestNotNull(TEXT("the Size property is optional"), Size))
	{
		return false;
	}

	FMjEffectiveValue Resolved;
	TestTrue(TEXT("size resolves through the class chain"),
		FMjEffectiveDetails::ResolveInherited(*Geom, *Size, Resolved));
	TestTrue(TEXT("the class is what supplied it"), Resolved.Source == EMjValueSource::Class);
	TestEqual(TEXT("named the class that authored it"), Resolved.ClassName, FString(TEXT("visual")));
	TestTrue(FString::Printf(TEXT("the value reads as the authored one, got '%s'"), *Resolved.Text),
		Resolved.Text.Contains(TEXT("0.3")));
	TestEqual(TEXT("and the row names the class beside the value"),
		FMjEffectiveDetails::DescribeValue(Resolved).ToString(), Resolved.Text + TEXT("  (from visual)"));

	// An attribute no class mentions still has a value: MuJoCo's own. That layer
	// is the whole of this item -- without it the panel showed a bare Set button
	// on nearly every row, because a document's `<default>` classes speak for a
	// handful of attributes and the schema speaks for the rest. `condim` is one
	// nothing in this document mentions.
	if (const FOptionalProperty* const Condim = OptionalNamed(*Geom, TEXT("Condim")))
	{
		FMjEffectiveValue Schema;
		TestTrue(TEXT("an attribute no class mentions falls to MuJoCo's own value"),
			FMjEffectiveDetails::ResolveInherited(*Geom, *Condim, Schema));
		TestTrue(TEXT("and says so: the schema supplied it, not a class"),
			Schema.Source == EMjValueSource::Schema);
		TestEqual(TEXT("the value is MuJoCo's own condim"), Schema.Text, FString(TEXT("3")));
		TestEqual(TEXT("the row reads it as a default rather than as a class"),
			FMjEffectiveDetails::DescribeValue(Schema).ToString(), FString(TEXT("3  (default)")));
	}

	// `margin` is the case that used to have no answer anywhere: `mjcf.schema`
	// states no `=` default for it, so the row was blank and the user was left to
	// read MuJoCo's manual. It is not undefaulted -- `mjs_defaultGeom` initialises
	// it, and that is the value the compiler merges against -- and the schema
	// layer now carries it.
	if (const FOptionalProperty* const Margin = OptionalNamed(*Geom, TEXT("Margin")))
	{
		FMjEffectiveValue Resolved;
		TestTrue(TEXT("an attribute the schema states no default for still resolves"),
			FMjEffectiveDetails::ResolveInherited(*Geom, *Margin, Resolved));
		TestTrue(TEXT("and it is MuJoCo's own layer that supplied it"), Resolved.Source == EMjValueSource::Schema);
		TestEqual(TEXT("MuJoCo's margin is zero"), FCString::Atod(*Resolved.Text), 0.0);
		TestTrue(FString::Printf(TEXT("the row marks it a default, got '%s'"),
					 *FMjEffectiveDetails::DescribeValue(Resolved).ToString()),
			FMjEffectiveDetails::DescribeValue(Resolved).ToString().EndsWith(TEXT("(default)")));
	}

	// What is left unresolved is an attribute MuJoCo COMPUTES rather than
	// defaults: a geom's `mass` comes from its density and its volume, and
	// `mjs_defaultGeom` marks it mjNAN to say so. That is not a value, so none is
	// invented -- and the row says "(no default)" rather than going blank, which
	// is the difference between an answer and a gap.
	if (const FOptionalProperty* const Mass = OptionalNamed(*Geom, TEXT("Mass")))
	{
		FMjEffectiveValue Unresolved;
		TestFalse(TEXT("a computed attribute reports no value"),
			FMjEffectiveDetails::ResolveInherited(*Geom, *Mass, Unresolved));
		TestEqual(TEXT("and the row says so in words"),
			FMjEffectiveDetails::DescribeValue(Unresolved).ToString(), FString(TEXT("(no default)")));
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjEffectiveDetailsBuildsOneContextPerRefresh,
	"URLab.Editor.EffectiveDetailsBuildsOneContextPerRefresh",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjEffectiveDetailsBuildsOneContextPerRefresh::RunTest(const FString& Parameters)
{
	using namespace MjEffectiveDetailsTests;

	FScratchDoc Doc;
	if (!Parse(*this, Doc))
	{
		return false;
	}

	UMjGeomBase* const Geom = ElementNamed<UMjGeomBase>(*Doc.Actor, TEXT("shell"));
	if (!TestNotNull(TEXT("the geom component"), Geom))
	{
		return false;
	}

	TArray<const FOptionalProperty*> Asked;
	for (const TCHAR* Name : {TEXT("Size"), TEXT("Rgba"), TEXT("Type"), TEXT("Contype"), TEXT("Margin")})
	{
		if (const FOptionalProperty* const Property = OptionalNamed(*Geom, Name))
		{
			Asked.Add(Property);
		}
	}
	if (!TestTrue(TEXT("several attributes to ask about"), Asked.Num() >= 4))
	{
		return false;
	}

	// One scope around the batch, exactly as the customization opens one around
	// a refresh. Without it this is one whole-spec walk per attribute.
	const int64 Before = MjEffectiveContextBuilds();
	{
		FMjEffectiveScope Scope(*Geom);
		for (const FOptionalProperty* const Property : Asked)
		{
			FMjEffectiveValue Resolved;
			FMjEffectiveDetails::ResolveInherited(*Geom, *Property, Resolved);
		}
	}
	const int64 Built = MjEffectiveContextBuilds() - Before;

	TestEqual(TEXT("one context for the whole refresh"), Built, static_cast<int64>(1));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjEulerRowRoundTripsThroughTheAuthoredQuaternion,
	"URLab.Editor.EulerRowRoundTripsThroughTheAuthoredQuaternion",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjEulerRowRoundTripsThroughTheAuthoredQuaternion::RunTest(const FString& Parameters)
{
	// A quarter turn about x is the case that catches a swapped component
	// order and a wrong handedness at once: MJCF writes it [w x y z] with the
	// scalar first, and Unreal's own quaternion writes the same rotation with
	// the scalar last and a negated x.
	{
		const double Degrees[3] = {90.0, 0.0, 0.0};
		double Quat[4];
		FMjArrayCustomizations::EulerDegreesToQuat(Degrees, Quat);

		const double Root = FMath::Sqrt(0.5);
		TestTrue(TEXT("w is cos 45"), FMath::IsNearlyEqual(Quat[0], Root, 1e-9));
		TestTrue(TEXT("x is sin 45"), FMath::IsNearlyEqual(Quat[1], Root, 1e-9));
		TestTrue(TEXT("y is zero"), FMath::IsNearlyEqual(Quat[2], 0.0, 1e-9));
		TestTrue(TEXT("z is zero"), FMath::IsNearlyEqual(Quat[3], 0.0, 1e-9));
	}

	// Every triple away from the gimbal pole comes back as itself. A wrong
	// extraction order survives the identity and fails here.
	const double Cases[4][3] = {
		{0.0, 0.0, 0.0},
		{30.0, -20.0, 45.0},
		{-115.0, 40.0, 10.0},
		{5.0, 89.0, -170.0},
	};
	for (const double(&Degrees)[3] : Cases)
	{
		double Quat[4];
		FMjArrayCustomizations::EulerDegreesToQuat(Degrees, Quat);
		double Back[3];
		FMjArrayCustomizations::QuatToEulerDegrees(Quat, Back);

		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			TestTrue(FString::Printf(TEXT("axis %d round trips: %.3f became %.3f"), Axis, Degrees[Axis],
						 Back[Axis]),
				FMath::IsNearlyEqual(Degrees[Axis], Back[Axis], 1e-6));
		}
	}
	return true;
}

#endif  // URLAB_MJ_GEN && WITH_EDITOR
