// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_DEV_AUTOMATION_TESTS

#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Elements/MjBody.h"
#include "MuJoCo/Elements/MjGeom.h"
#include "MuJoCo/Gen/MjElements.gen.h"
#include "MuJoCo/Gen/MjSpecWrite.gen.h"
#include "MuJoCo/Spec/MjNodeFactories.h"
#include "MuJoCo/Spec/MjSpecBuild.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "Tests/MjTestHelpers.h"

THIRD_PARTY_INCLUDES_START
#include <mujoco/mujoco.h>
THIRD_PARTY_INCLUDES_END

// The spec write, asserted where it actually lands: on the mjs structs.
//
// Every one of these builds its component tree in the test rather than reading
// a fixture, because what is under test is the crossing from components to an
// mjSpec and a file would put the reader in the middle of it. It also keeps the
// tests out of the shared fixture directory, which other suites sweep whole.
//
// The shapes are covered one element each rather than exhaustively: the writes
// are generated from one rule per storage shape, so a shape that works once
// works everywhere, and a shape that has no test here has no coverage at all.

namespace
{

/** A component tree with no manager, no compile and no world tick. */
struct FSpecWriteFixture
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

	~FSpecWriteFixture()
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
		UMjNodeComponent& Node = urlab::spec::FInstanceNodeFactory::Create<
			typename TMjGeneratedOf<E>::Type>(*Parent);
		if (Name != nullptr)
		{
			Node.MjName = Name;
		}
		return Cast<E>(&Node);
	}

	FSpecRef Spec() const { return FSpecRef::OverActor(*Robot); }
};

/** Build, and report the diagnostics rather than swallowing them. */
urlab::spec::FMjBuiltSpec Build(FAutomationTestBase& Test, const FSpecRef& Root)
{
	TArray<FMjSpecDiagnostic> Diagnostics;
	urlab::spec::FMjBuiltSpec Built = urlab::spec::BuildSpec(Root, Diagnostics);
	if (Built.Spec == nullptr)
	{
		for (const FMjSpecDiagnostic& Diagnostic : Diagnostics)
		{
			Test.AddError(Diagnostic.ToString());
		}
	}
	return Built;
}

/**
 * The struct behind a named element.
 *
 * By name rather than through ElementFor, so that what is asserted is what a
 * compile would see: an mjsElement is a handle and the struct is a separate
 * object, which is why mjs_as* exists and a cast does not.
 */
mjsGeom* FindGeom(mjSpec* Spec, const TCHAR* Name)
{
	const FTCHARToUTF8 Utf8(Name);
	return mjs_asGeom(mjs_findElement(Spec, mjOBJ_GEOM, Utf8.Get()));
}

mjsSensor* FindSensor(mjSpec* Spec, const TCHAR* Name)
{
	const FTCHARToUTF8 Utf8(Name);
	return mjs_asSensor(mjs_findElement(Spec, mjOBJ_SENSOR, Utf8.Get()));
}

}  // namespace

// --- Storage shapes -------------------------------------------------------- //

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSpecWriteShapesTest,
	"URLab.MuJoCo.SpecWrite.Shapes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSpecWriteShapesTest::RunTest(const FString& Parameters)
{
	FSpecWriteFixture Fixture;
	if (!Fixture.Init())
	{
		AddError(TEXT("could not build the component tree"));
		return false;
	}

	UMjBody* const Body = Fixture.Add<UMjBody>(Fixture.WorldBody, TEXT("Root"));
	UMjGeom* const Geom = Fixture.Add<UMjGeom>(Body, TEXT("Shape"));
	if (Body == nullptr || Geom == nullptr)
	{
		AddError(TEXT("could not author the body and geom"));
		return false;
	}

	// One authored value per storage shape the emitter produces.
	Geom->Type = EMjGeomType::box;                             // enum
	Geom->Condim = 4;                                          // int scalar
	Geom->Mass = 2.5;                                          // double scalar
	Geom->Shellinertia = true;                                 // bool onto an enum field
	Geom->Size = TArray<double>({ 0.1, 0.2 });                 // range into a fixed array
	Geom->Pos = FMjPosition3(1.0, 2.0, 3.0);                   // fixed arity, kind-typed
	Geom->Rgba = FLinearColor(0.25f, 0.5f, 0.75f, 1.0f);       // fixed arity, float target
	Geom->User = TArray<double>({ 7.0, 8.0 });                 // unbounded onto a vector
	Geom->Material = FString(TEXT("Paint"));                   // reference onto a string

	urlab::spec::FMjBuiltSpec Built = Build(*this, Fixture.Spec());
	if (Built.Spec == nullptr)
	{
		return false;
	}

	mjsElement* const Element = Built.ElementFor.FindRef(Geom);
	if (!TestNotNull(TEXT("the geom reached the spec"), Element))
	{
		return false;
	}
	const mjsGeom* const Out = FindGeom(Built.Spec, TEXT("Shape"));
	if (!TestNotNull(TEXT("the geom is findable by name"), Out))
	{
		return false;
	}

	TestEqual(TEXT("enum keyword became the C constant"), static_cast<int>(Out->type),
		static_cast<int>(mjGEOM_BOX));
	TestEqual(TEXT("int scalar"), Out->condim, 4);
	TestEqual(TEXT("double scalar"), Out->mass, 2.5);
	TestEqual(TEXT("bool onto an enum field"), static_cast<int>(Out->typeinertia), 1);
	TestEqual(TEXT("range arity writes the authored count"), Out->size[0], 0.1);
	TestEqual(TEXT("range arity writes the authored count"), Out->size[1], 0.2);
	TestEqual(TEXT("fixed arity, in MJCF component order"), Out->pos[0], 1.0);
	TestEqual(TEXT("fixed arity, in MJCF component order"), Out->pos[1], 2.0);
	TestEqual(TEXT("fixed arity, in MJCF component order"), Out->pos[2], 3.0);
	TestEqual(TEXT("float target"), Out->rgba[0], 0.25f);
	TestEqual(TEXT("float target"), Out->rgba[3], 1.0f);
	TestEqual(TEXT("string reference"),
		FString(UTF8_TO_TCHAR(mjs_getString(Out->material))), FString(TEXT("Paint")));

	return !HasAnyErrors();
}

// --- Unauthored fields ----------------------------------------------------- //

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSpecWriteUnauthoredTest,
	"URLab.MuJoCo.SpecWrite.Unauthored",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSpecWriteUnauthoredTest::RunTest(const FString& Parameters)
{
	// The whole reason default classes resolve the way MuJoCo's reader makes
	// them: an unauthored attribute must be left exactly as mjs_addGeom left it,
	// because writing a zero would look like an authored zero at compile.
	FSpecWriteFixture Fixture;
	if (!Fixture.Init())
	{
		AddError(TEXT("could not build the component tree"));
		return false;
	}

	UMjBody* const Body = Fixture.Add<UMjBody>(Fixture.WorldBody, TEXT("Root"));
	UMjGeom* const Geom = Fixture.Add<UMjGeom>(Body, TEXT("Bare"));
	if (Geom == nullptr)
	{
		AddError(TEXT("could not author the geom"));
		return false;
	}
	Geom->Type = EMjGeomType::sphere;
	Geom->Size = TArray<double>({ 0.1 });

	mjsGeom Fresh;
	mjs_defaultGeom(&Fresh);

	urlab::spec::FMjBuiltSpec Built = Build(*this, Fixture.Spec());
	if (Built.Spec == nullptr)
	{
		return false;
	}
	const mjsGeom* const Out = FindGeom(Built.Spec, TEXT("Bare"));
	if (!TestNotNull(TEXT("the geom is findable by name"), Out))
	{
		return false;
	}

	TestEqual(TEXT("an unauthored double keeps the mjs default"), Out->solmix, Fresh.solmix);
	TestEqual(TEXT("an unauthored int keeps the mjs default"), Out->contype, Fresh.contype);
	TestEqual(TEXT("an unauthored margin keeps the mjs default"), Out->margin, Fresh.margin);
	TestEqual(TEXT("an unauthored fixed array keeps the mjs default"),
		Out->friction[0], Fresh.friction[0]);

	return !HasAnyErrors();
}

// --- Nested default classes ------------------------------------------------ //

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSpecWriteDefaultsTest,
	"URLab.MuJoCo.SpecWrite.NestedDefaults",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSpecWriteDefaultsTest::RunTest(const FString& Parameters)
{
	// Class resolution has to reach mj_compile, not just the spec: a geom that
	// names a nested class inherits through the whole chain, and the only proof
	// is a compiled model carrying the inherited value.
	FSpecWriteFixture Fixture;
	if (!Fixture.Init())
	{
		AddError(TEXT("could not build the component tree"));
		return false;
	}

	UMjDefault* const Outer = Fixture.Add<UMjDefault>(Fixture.Robot->Spec, TEXT("outer"));
	UMjGeomBase* const OuterGeom = Fixture.Add<UMjGeomBase>(Outer);
	UMjDefault* const Inner = Fixture.Add<UMjDefault>(Outer, TEXT("inner"));
	UMjGeomBase* const InnerGeom = Fixture.Add<UMjGeomBase>(Inner);
	if (Outer == nullptr || OuterGeom == nullptr || Inner == nullptr || InnerGeom == nullptr)
	{
		AddError(TEXT("could not author the default classes"));
		return false;
	}
	// The outer class sets the contact dimension; the inner one leaves it alone
	// and sets only the group, so an element on the inner class must show both.
	OuterGeom->Condim = 6;
	InnerGeom->Group = 3;

	UMjBody* const Body = Fixture.Add<UMjBody>(Fixture.WorldBody, TEXT("Root"));
	UMjGeom* const Geom = Fixture.Add<UMjGeom>(Body, TEXT("Inherited"));
	// A sibling on the OUTER class, so that a nesting which quietly collapsed
	// the two into one still fails here: it would hand this geom the inner
	// class's group as well, and the chain would look right from "Inherited"
	// alone.
	UMjGeom* const Shallow = Fixture.Add<UMjGeom>(Body, TEXT("Shallow"));
	if (Geom == nullptr || Shallow == nullptr)
	{
		AddError(TEXT("could not author the geoms"));
		return false;
	}
	Geom->Dclass = FString(TEXT("inner"));
	Geom->Type = EMjGeomType::sphere;
	Geom->Size = TArray<double>({ 0.1 });
	Shallow->Dclass = FString(TEXT("outer"));
	Shallow->Type = EMjGeomType::sphere;
	Shallow->Size = TArray<double>({ 0.1 });

	urlab::spec::FMjBuiltSpec Built = Build(*this, Fixture.Spec());
	if (Built.Spec == nullptr)
	{
		return false;
	}

	mjModel* const Model = mj_compile(Built.Spec, nullptr);
	if (Model == nullptr)
	{
		AddError(FString::Printf(TEXT("mj_compile failed: %s"),
			UTF8_TO_TCHAR(mjs_getError(Built.Spec))));
		return false;
	}

	const int Id = mj_name2id(Model, mjOBJ_GEOM, "Inherited");
	if (TestTrue(TEXT("the geom compiled into the model"), Id >= 0))
	{
		TestEqual(TEXT("condim came from the outer class"), Model->geom_condim[Id], 6);
		TestEqual(TEXT("group came from the inner class"), Model->geom_group[Id], 3);
	}

	// The outer class alone carries the contact dimension and NOT the group, so
	// the two classes have to be distinct objects for both of these to hold.
	const int ShallowId = mj_name2id(Model, mjOBJ_GEOM, "Shallow");
	if (TestTrue(TEXT("the outer-class geom compiled into the model"), ShallowId >= 0))
	{
		TestEqual(TEXT("the outer class carries the contact dimension"),
			Model->geom_condim[ShallowId], 6);
		TestNotEqual(TEXT("the inner class's group did not leak into the outer one"),
			Model->geom_group[ShallowId], 3);
	}
	mj_deleteModel(Model);

	return !HasAnyErrors();
}

// --- Childclass over a nested class chain ---------------------------------- //

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSpecWriteChildclassTest,
	"URLab.MuJoCo.SpecWrite.NestedChildclass",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSpecWriteChildclassTest::RunTest(const FString& Parameters)
{
	// Childclass is a second route to a class and not the same one: it names
	// the class the SUBTREE resolves through, where Dclass names the one the
	// element itself resolves through. A nested chain reached that way has to
	// inherit the whole chain, keep applying down the subtree, and give way to
	// an element that names its own class.
	FSpecWriteFixture Fixture;
	if (!Fixture.Init())
	{
		AddError(TEXT("could not build the component tree"));
		return false;
	}

	UMjDefault* const Outer = Fixture.Add<UMjDefault>(Fixture.Robot->Spec, TEXT("outer"));
	UMjGeomBase* const OuterGeom = Fixture.Add<UMjGeomBase>(Outer);
	UMjDefault* const Inner = Fixture.Add<UMjDefault>(Outer, TEXT("inner"));
	UMjGeomBase* const InnerGeom = Fixture.Add<UMjGeomBase>(Inner);
	if (OuterGeom == nullptr || InnerGeom == nullptr)
	{
		AddError(TEXT("could not author the default classes"));
		return false;
	}
	OuterGeom->Condim = 6;
	InnerGeom->Group = 3;

	const auto Sphere = [](UMjGeom* Geom) {
		Geom->Type = EMjGeomType::sphere;
		Geom->Size = TArray<double>({ 0.1 });
	};

	UMjBody* const Host = Fixture.Add<UMjBody>(Fixture.WorldBody, TEXT("Host"));
	UMjBody* const Nested = Fixture.Add<UMjBody>(Host, TEXT("Nested"));
	UMjGeom* const Direct = Fixture.Add<UMjGeom>(Host, TEXT("Direct"));
	UMjGeom* const Deep = Fixture.Add<UMjGeom>(Nested, TEXT("Deep"));
	UMjGeom* const Overriding = Fixture.Add<UMjGeom>(Host, TEXT("Overriding"));
	UMjBody* const Shallow = Fixture.Add<UMjBody>(Fixture.WorldBody, TEXT("Shallow"));
	UMjGeom* const ShallowGeom = Fixture.Add<UMjGeom>(Shallow, TEXT("ShallowGeom"));
	if (Host == nullptr || Nested == nullptr || Direct == nullptr || Deep == nullptr ||
		Overriding == nullptr || Shallow == nullptr || ShallowGeom == nullptr)
	{
		AddError(TEXT("could not author the bodies"));
		return false;
	}
	Host->Childclass = FString(TEXT("inner"));
	Shallow->Childclass = FString(TEXT("outer"));
	Overriding->Dclass = FString(TEXT("outer"));
	Sphere(Direct);
	Sphere(Deep);
	Sphere(Overriding);
	Sphere(ShallowGeom);

	urlab::spec::FMjBuiltSpec Built = Build(*this, Fixture.Spec());
	if (Built.Spec == nullptr)
	{
		return false;
	}

	mjModel* const Model = mj_compile(Built.Spec, nullptr);
	if (Model == nullptr)
	{
		AddError(FString::Printf(TEXT("mj_compile failed: %s"),
			UTF8_TO_TCHAR(mjs_getError(Built.Spec))));
		return false;
	}

	const auto Check = [this, Model](const char* Name, int ExpectedCondim, int ExpectedGroup) {
		const int Id = mj_name2id(Model, mjOBJ_GEOM, Name);
		if (!TestTrue(FString::Printf(TEXT("%s compiled"), UTF8_TO_TCHAR(Name)), Id >= 0))
		{
			return;
		}
		TestEqual(FString::Printf(TEXT("%s condim"), UTF8_TO_TCHAR(Name)),
			Model->geom_condim[Id], ExpectedCondim);
		TestEqual(FString::Printf(TEXT("%s group"), UTF8_TO_TCHAR(Name)),
			Model->geom_group[Id], ExpectedGroup);
	};

	// The whole chain, reached through childclass rather than by naming it.
	Check("Direct", 6, 3);
	// And it keeps applying below the body that opened it.
	Check("Deep", 6, 3);
	// An element naming its own class is not overridden by the subtree's.
	Check("Overriding", 6, 0);
	// The outer class alone, so a chain that collapsed into one class fails.
	Check("ShallowGeom", 6, 0);
	mj_deleteModel(Model);

	return !HasAnyErrors();
}

// --- The walk -------------------------------------------------------------- //

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSpecWriteWalkTest,
	"URLab.MuJoCo.SpecWrite.Walk",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSpecWriteWalkTest::RunTest(const FString& Parameters)
{
	FSpecWriteFixture Fixture;
	if (!Fixture.Init())
	{
		AddError(TEXT("could not build the component tree"));
		return false;
	}

	// A tree wide enough to reach every creating category the walk carries:
	// a nested body, its geom and joint, a spec-scoped asset, and a spec-scoped
	// sensor that names the site above it.
	UMjBody* const Body = Fixture.Add<UMjBody>(Fixture.WorldBody, TEXT("Root"));
	UMjBody* const Child = Fixture.Add<UMjBody>(Body, TEXT("Child"));
	UMjGeom* const Geom = Fixture.Add<UMjGeom>(Child, TEXT("Shape"));
	UMjJoint* const Joint = Fixture.Add<UMjJoint>(Child, TEXT("Hinge"));
	UMjSite* const Site = Fixture.Add<UMjSite>(Child, TEXT("Probe"));
	if (Geom == nullptr || Joint == nullptr || Site == nullptr)
	{
		AddError(TEXT("could not author the body subtree"));
		return false;
	}
	Geom->Type = EMjGeomType::sphere;
	Geom->Size = TArray<double>({ 0.1 });

	UMjSensor* const Sensors = Fixture.Add<UMjSensor>(Fixture.Robot->Spec);
	UMjGyro* const Gyro = Fixture.Add<UMjGyro>(Sensors, TEXT("Rates"));
	if (Gyro == nullptr)
	{
		AddError(TEXT("could not author the sensor"));
		return false;
	}
	Gyro->Site = FString(TEXT("Probe"));

	urlab::spec::FMjBuiltSpec Built = Build(*this, Fixture.Spec());
	if (Built.Spec == nullptr)
	{
		return false;
	}

	// Every component that named an element has a handle, and it is the handle
	// the walk created rather than one recovered by matching names afterwards.
	const UMjNodeComponent* const Expected[] = {
		Fixture.WorldBody, Body, Child, Geom, Joint, Site, Gyro };
	for (const UMjNodeComponent* Node : Expected)
	{
		if (!TestTrue(FString::Printf(TEXT("%s is in ElementFor"), *Node->GetName()),
			Built.ElementFor.Contains(Node)))
		{
			continue;
		}
		TestNotNull(TEXT("the recorded element is real"), Built.ElementFor.FindRef(Node));
	}

	// A sensor spelling is a schema constant rather than an attribute, so the
	// generated write is the only thing that could have set it.
	const mjsSensor* const Sensor = FindSensor(Built.Spec, TEXT("Rates"));
	if (TestNotNull(TEXT("the gyro reached the spec"), Sensor))
	{
		TestEqual(TEXT("the sensor spelling set its type"),
			static_cast<int>(Sensor->type), static_cast<int>(mjSENS_GYRO));
		TestEqual(TEXT("the sensor spelling set its object kind"),
			static_cast<int>(Sensor->objtype), static_cast<int>(mjOBJ_SITE));
	}

	mjModel* const Model = mj_compile(Built.Spec, nullptr);
	if (Model == nullptr)
	{
		AddError(FString::Printf(TEXT("mj_compile failed: %s"),
			UTF8_TO_TCHAR(mjs_getError(Built.Spec))));
		return false;
	}
	// The model's counts are mjtSize, not int, so the comparison says which
	// type it is in rather than leaving the overload set to guess.
	TestEqual(TEXT("both authored bodies compiled, plus the world"),
		static_cast<int32>(Model->nbody), 3);
	mj_deleteModel(Model);

	return !HasAnyErrors();
}

#endif  // URLAB_MJ_GEN && WITH_DEV_AUTOMATION_TESTS
