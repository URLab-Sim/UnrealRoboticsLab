// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// The legality the hand-written hooks enforce, held against MuJoCo's reader.
//
// Two elections in the spec write are not field copies: an actuator's
// transmission and an equality's operands both decide a type and a target
// together, from whichever spelling the author used. MuJoCo's reader rejects
// the half-authored combinations at the point of reading; ours used to carry
// them into the compiler, where they surface as a reference to nothing, naming
// neither the element nor the line it was authored on. These are the
// combinations, one test per edge, asserting that the diagnostic arrives here
// and says which rule was broken.
//
// The other direction is asserted in the same tests: every legal spelling still
// builds. A legality check that also rejects something MuJoCo accepts is a
// worse defect than the one it fixes, and nothing about these inputs is exotic.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_DEV_AUTOMATION_TESTS

#include "Engine/Engine.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"

#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Elements/MjBody.h"
#include "MuJoCo/Gen/MjElements.gen.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjNodeFactories.h"
#include "MuJoCo/Spec/MjSpecBuild.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "Tests/MjTestHelpers.h"

namespace MjHookLegalityTests
{

/** A component tree with no manager, no compile and no world tick. */
struct FHookFixture
{
	UWorld* World = nullptr;
	AMjArticulation* Robot = nullptr;
	UMjBodyBase* WorldBody = nullptr;

	bool Init()
	{
		World = UWorld::CreateWorld(EWorldType::Game, false);
		if (World == nullptr)
		{
			return false;
		}
		FWorldContext& Context = GEngine->CreateNewWorldContext(EWorldType::Game);
		Context.SetCurrentWorld(World);

		FActorSpawnParameters Params;
		Robot = World->SpawnActor<AMjArticulation>(Params);
		if (Robot == nullptr)
		{
			return false;
		}
		// MJCF forbids <worldbody> from carrying attributes, so the first body
		// under the model is anonymous and everything else hangs off it.
		WorldBody = Add<UMjBodyBase>(Robot->Spec);
		return WorldBody != nullptr;
	}

	~FHookFixture()
	{
		if (World != nullptr)
		{
			GEngine->DestroyWorldContext(World);
			World->DestroyWorld(false);
		}
	}

	template <class E>
	E* Add(UMjNodeComponent* Parent, const TCHAR* Name = nullptr)
	{
		if (Robot == nullptr || Parent == nullptr)
		{
			return nullptr;
		}
		urlab::spec::FMjInstanceScope Scope(*Robot);
		// Through the factory rather than NewObject: identity and sibling order
		// are what a node needs to be found by the walk at all, and only the
		// factory stamps them.
		UMjNodeComponent& Node =
			urlab::spec::FInstanceNodeFactory::Create<typename TMjGeneratedOf<E>::Type>(*Parent);
		if (Name != nullptr)
		{
			Node.MjName = Name;
		}
		return Cast<E>(&Node);
	}

	/** The `<actuator>` section, created on first ask. */
	UMjActuator* ActuatorSection()
	{
		if (Actuators == nullptr)
		{
			Actuators = Add<UMjActuator>(Robot->Spec);
		}
		return Actuators;
	}

	/** The `<equality>` section, created on first ask. */
	UMjEquality* EqualitySection()
	{
		if (Equalities == nullptr)
		{
			Equalities = Add<UMjEquality>(Robot->Spec);
		}
		return Equalities;
	}

	FSpecRef Spec() const { return FSpecRef::OverActor(*Robot); }

private:
	UMjActuator* Actuators = nullptr;
	UMjEquality* Equalities = nullptr;
};

/** What one build came to: whether it produced a spec, and what it said. */
struct FBuildOutcome
{
	bool bBuilt = false;
	FString Diagnostics;
};

FBuildOutcome BuildOnce(const FSpecRef& Root)
{
	TArray<FMjSpecDiagnostic> Reported;
	const urlab::spec::FMjBuiltSpec Built = urlab::spec::BuildSpec(Root, Reported);

	FBuildOutcome Outcome;
	Outcome.bBuilt = Built.Spec != nullptr;
	TArray<FString> Lines;
	for (const FMjSpecDiagnostic& Diagnostic : Reported)
	{
		Lines.Add(Diagnostic.ToString());
	}
	Outcome.Diagnostics = FString::Join(Lines, TEXT("; "));
	return Outcome;
}

/** A one-element document, authored by the caller and built once. */
template <class TAuthor>
FBuildOutcome Author(FAutomationTestBase& Test, const TCHAR* What, TAuthor&& Authoring)
{
	FHookFixture Fixture;
	if (!Fixture.Init())
	{
		Test.AddError(FString::Printf(TEXT("%s: could not build the component tree"), What));
		return FBuildOutcome();
	}
	Authoring(Fixture);
	return BuildOnce(Fixture.Spec());
}

/** The build refused it, and said which rule it broke. */
template <class TAuthor>
void ExpectRefused(FAutomationTestBase& Test, const TCHAR* What, const TCHAR* MustSay, TAuthor&& Authoring)
{
	const FBuildOutcome Outcome = Author(Test, What, Forward<TAuthor>(Authoring));
	if (Outcome.bBuilt)
	{
		Test.AddError(FString::Printf(TEXT("%s: the spec built; the illegal spelling was carried into the "
										   "compiler instead of being reported here"),
			What));
		return;
	}
	if (!Outcome.Diagnostics.Contains(MustSay))
	{
		Test.AddError(FString::Printf(TEXT("%s: refused, but the diagnostic does not name the rule ('%s' "
										   "expected); it said: %s"),
			What, MustSay, *Outcome.Diagnostics));
	}
}

/** The build accepted it, which is what MuJoCo's reader does with it too. */
template <class TAuthor>
void ExpectAccepted(FAutomationTestBase& Test, const TCHAR* What, TAuthor&& Authoring)
{
	const FBuildOutcome Outcome = Author(Test, What, Forward<TAuthor>(Authoring));
	if (!Outcome.bBuilt)
	{
		Test.AddError(FString::Printf(
			TEXT("%s: MuJoCo's reader accepts this and the build refused it: %s"), What, *Outcome.Diagnostics));
	}
}

FMjPosition3 Origin()
{
	FMjPosition3 Anchor;
	Anchor.X = 0.0;
	Anchor.Y = 0.0;
	Anchor.Z = 0.0;
	return Anchor;
}

} // namespace MjHookLegalityTests

// ============================================================================
// URLab.MuJoCo.HookLegality.Transmission
//   An actuator's transmission is elected by whichever operand names a target.
//   The reader rejects the operands that contradict the election, and an
//   actuator that elects nothing at all is caught here rather than in the
//   compiler, which names neither the actuator nor its line.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjHookLegalityTransmissionTest, "URLab.MuJoCo.HookLegality.Transmission",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjHookLegalityTransmissionTest::RunTest(const FString& Parameters)
{
	using namespace MjHookLegalityTests;

	ExpectRefused(*this, TEXT("a motor naming nothing to drive"), TEXT("elects no transmission"),
		[](FHookFixture& Fixture) { Fixture.Add<UMjMotor>(Fixture.ActuatorSection(), TEXT("act")); });

	ExpectRefused(*this, TEXT("cranklength on a joint transmission"), TEXT("slidercrank"),
		[](FHookFixture& Fixture)
		{
			UMjMotor* const Motor = Fixture.Add<UMjMotor>(Fixture.ActuatorSection(), TEXT("act"));
			Motor->Joint = FString(TEXT("hinge"));
			Motor->Cranklength = 0.5;
		});

	ExpectRefused(*this, TEXT("slidersite on a joint transmission"), TEXT("slidercrank"),
		[](FHookFixture& Fixture)
		{
			UMjMotor* const Motor = Fixture.Add<UMjMotor>(Fixture.ActuatorSection(), TEXT("act"));
			Motor->Joint = FString(TEXT("hinge"));
			Motor->Slidersite = FString(TEXT("base"));
		});

	ExpectRefused(*this, TEXT("refsite on a joint transmission"), TEXT("site transmission"),
		[](FHookFixture& Fixture)
		{
			UMjPosition* const Position = Fixture.Add<UMjPosition>(Fixture.ActuatorSection(), TEXT("act"));
			Position->Joint = FString(TEXT("hinge"));
			Position->Refsite = FString(TEXT("ref"));
		});

	ExpectAccepted(*this, TEXT("a motor on a joint"),
		[](FHookFixture& Fixture)
		{
			UMjMotor* const Motor = Fixture.Add<UMjMotor>(Fixture.ActuatorSection(), TEXT("act"));
			Motor->Joint = FString(TEXT("hinge"));
		});

	ExpectAccepted(*this, TEXT("refsite on a site transmission"),
		[](FHookFixture& Fixture)
		{
			UMjPosition* const Position = Fixture.Add<UMjPosition>(Fixture.ActuatorSection(), TEXT("act"));
			Position->Site = FString(TEXT("tip"));
			Position->Refsite = FString(TEXT("ref"));
		});

	ExpectAccepted(*this, TEXT("slidersite on a slidercrank transmission"),
		[](FHookFixture& Fixture)
		{
			UMjMotor* const Motor = Fixture.Add<UMjMotor>(Fixture.ActuatorSection(), TEXT("act"));
			Motor->Cranksite = FString(TEXT("crank"));
			Motor->Slidersite = FString(TEXT("base"));
		});

	return !HasAnyErrors();
}

// ============================================================================
// URLab.MuJoCo.HookLegality.EqualityOperands
//   An equality constraint's operands elect the object kind it relates. The
//   reader takes the body spelling only when the body half is complete, and
//   reads both site names in every other case, so a half-authored pair is
//   rejected there rather than resolved to nothing later.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjHookLegalityEqualityTest, "URLab.MuJoCo.HookLegality.EqualityOperands",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjHookLegalityEqualityTest::RunTest(const FString& Parameters)
{
	using namespace MjHookLegalityTests;

	ExpectRefused(*this, TEXT("a connect with body1 and no anchor"), TEXT("connect constraint"),
		[](FHookFixture& Fixture)
		{
			UMjConnect* const Connect = Fixture.Add<UMjConnect>(Fixture.EqualitySection(), TEXT("eq"));
			Connect->Body1 = FString(TEXT("a"));
			Connect->Body2 = FString(TEXT("b"));
		});

	ExpectRefused(*this, TEXT("a connect naming one site"), TEXT("connect constraint"),
		[](FHookFixture& Fixture)
		{
			UMjConnect* const Connect = Fixture.Add<UMjConnect>(Fixture.EqualitySection(), TEXT("eq"));
			Connect->Site1 = FString(TEXT("s1"));
		});

	ExpectRefused(*this, TEXT("a weld naming one site"), TEXT("weld constraint"),
		[](FHookFixture& Fixture)
		{
			UMjWeld* const Weld = Fixture.Add<UMjWeld>(Fixture.EqualitySection(), TEXT("eq"));
			Weld->Site1 = FString(TEXT("s1"));
		});

	ExpectRefused(*this, TEXT("a joint equality naming no joint1"), TEXT("name joint1"),
		[](FHookFixture& Fixture)
		{
			UMjEqualityJoint* const Joint = Fixture.Add<UMjEqualityJoint>(Fixture.EqualitySection(), TEXT("eq"));
			Joint->Joint2 = FString(TEXT("j2"));
		});

	ExpectRefused(*this, TEXT("a tendon equality naming no tendon1"), TEXT("name tendon1"),
		[](FHookFixture& Fixture)
		{
			UMjEqualityTendon* const Tendon = Fixture.Add<UMjEqualityTendon>(Fixture.EqualitySection(), TEXT("eq"));
			Tendon->Tendon2 = FString(TEXT("t2"));
		});

	ExpectRefused(*this, TEXT("a flex equality naming no flex"), TEXT("name flex"),
		[](FHookFixture& Fixture) { Fixture.Add<UMjEqualityFlex>(Fixture.EqualitySection(), TEXT("eq")); });

	ExpectAccepted(*this, TEXT("a connect with body1 and an anchor"),
		[](FHookFixture& Fixture)
		{
			UMjConnect* const Connect = Fixture.Add<UMjConnect>(Fixture.EqualitySection(), TEXT("eq"));
			Connect->Body1 = FString(TEXT("a"));
			Connect->Body2 = FString(TEXT("b"));
			Connect->Anchor = Origin();
		});

	ExpectAccepted(*this, TEXT("a connect naming two sites"),
		[](FHookFixture& Fixture)
		{
			UMjConnect* const Connect = Fixture.Add<UMjConnect>(Fixture.EqualitySection(), TEXT("eq"));
			Connect->Site1 = FString(TEXT("s1"));
			Connect->Site2 = FString(TEXT("s2"));
		});

	// Weld elects the body spelling on body1 alone; the anchor is optional
	// there and defaults to the body origin.
	ExpectAccepted(*this, TEXT("a weld with body1 and no anchor"),
		[](FHookFixture& Fixture)
		{
			UMjWeld* const Weld = Fixture.Add<UMjWeld>(Fixture.EqualitySection(), TEXT("eq"));
			Weld->Body1 = FString(TEXT("a"));
		});

	ExpectAccepted(*this, TEXT("a joint equality naming joint1"),
		[](FHookFixture& Fixture)
		{
			UMjEqualityJoint* const Joint = Fixture.Add<UMjEqualityJoint>(Fixture.EqualitySection(), TEXT("eq"));
			Joint->Joint1 = FString(TEXT("j1"));
		});

	return !HasAnyErrors();
}

// ============================================================================
// URLab.MuJoCo.HookLegality.ClassTemplatesAreExempt
//   An actuator spelling under a <default> is that class's template, and a
//   template has no transmission of its own: the element resolving through it
//   supplies one. Requiring a transmission there would reject every classed
//   actuator in the corpus, which is the failure mode a legality check has.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjHookLegalityClassTemplateTest, "URLab.MuJoCo.HookLegality.ClassTemplatesAreExempt",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjHookLegalityClassTemplateTest::RunTest(const FString& Parameters)
{
	using namespace MjHookLegalityTests;

	ExpectAccepted(*this, TEXT("a class template carrying a transmissionless actuator"),
		[](FHookFixture& Fixture)
		{
			UMjDefault* const Class = Fixture.Add<UMjDefault>(Fixture.Robot->Spec, TEXT("driven"));
			UMjPosition* const Position = Fixture.Add<UMjPosition>(Class);
			Position->Kp = 25.0;
		});

	// An actuator resolving through that class still elects its own
	// transmission, so the check has not been disarmed for the elements the
	// class serves.
	ExpectRefused(*this, TEXT("a classed actuator naming nothing to drive"), TEXT("elects no transmission"),
		[](FHookFixture& Fixture)
		{
			UMjDefault* const Class = Fixture.Add<UMjDefault>(Fixture.Robot->Spec, TEXT("driven"));
			UMjPosition* const Template = Fixture.Add<UMjPosition>(Class);
			Template->Kp = 25.0;

			UMjPosition* const Position = Fixture.Add<UMjPosition>(Fixture.ActuatorSection(), TEXT("act"));
			Position->Dclass = FString(TEXT("driven"));
		});

	return !HasAnyErrors();
}

// ============================================================================
// URLab.MuJoCo.HookLegality.DiagnosticNamesTheElement
//   Which element broke the rule, not just which rule broke. Two actuators of
//   the same type, one legal and one not: a diagnostic that named neither, or
//   named the type, would read identically in both fixtures and send the user
//   looking through every actuator in the model. The source file and line stay
//   where they were, because a model that came from a file still has them and
//   one authored in the editor never did.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjHookLegalityNamedElementTest, "URLab.MuJoCo.HookLegality.DiagnosticNamesTheElement",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjHookLegalityNamedElementTest::RunTest(const FString& Parameters)
{
	using namespace MjHookLegalityTests;

	FHookFixture Fixture;
	if (!Fixture.Init())
	{
		AddError(TEXT("could not build the component tree"));
		return false;
	}

	UMjMotor* const Driven = Fixture.Add<UMjMotor>(Fixture.ActuatorSection(), TEXT("elbow_drive"));
	UMjMotor* const Adrift = Fixture.Add<UMjMotor>(Fixture.ActuatorSection(), TEXT("wrist_drive"));
	if (Driven == nullptr || Adrift == nullptr)
	{
		AddError(TEXT("could not author the two actuators"));
		return false;
	}
	Driven->Joint = FString(TEXT("elbow"));
	// Adrift names nothing to drive, which is the rule under test.
	Adrift->SourceFile = FString(TEXT("robot.xml"));
	Adrift->SourceLine = 42;

	const FBuildOutcome Outcome = BuildOnce(Fixture.Spec());
	if (Outcome.bBuilt)
	{
		AddError(TEXT("the spec built; the transmissionless actuator was carried into the compiler"));
		return false;
	}

	TestTrue(TEXT("the diagnostic names the offending element by its MJCF name"),
		Outcome.Diagnostics.Contains(TEXT("wrist_drive")));
	TestFalse(TEXT("and not its legal sibling of the same type"),
		Outcome.Diagnostics.Contains(TEXT("elbow_drive")));
	TestTrue(TEXT("it names the component the user selects, too"),
		Outcome.Diagnostics.Contains(Adrift->GetName()));
	TestTrue(TEXT("alongside the source location, which is not displaced by the names"),
		Outcome.Diagnostics.Contains(TEXT("robot.xml(42)")));
	TestTrue(TEXT("and still says which rule was broken"),
		Outcome.Diagnostics.Contains(TEXT("elects no transmission")));

	return !HasAnyErrors();
}

#endif // URLAB_MJ_GEN && WITH_DEV_AUTOMATION_TESTS
