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
#include "HAL/FileManager.h"
#include "Misc/AutomationTest.h"
#include "Misc/FileHelper.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"

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

/** Compile, reporting the spec's own account of a failure. */
mjModel* Compile(FAutomationTestBase& Test, mjSpec* Spec)
{
	mjModel* const Model = mj_compile(Spec, nullptr);
	if (Model == nullptr)
	{
		Test.AddError(FString::Printf(TEXT("mj_compile failed: %s"), UTF8_TO_TCHAR(mjs_getError(Spec))));
	}
	return Model;
}

/** A scratch directory of this run's own, removed with everything under it. */
struct FSpecWriteScratch
{
	FString Path;

	FSpecWriteScratch()
		: Path(FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("URLabTests"),
			  FGuid::NewGuid().ToString(EGuidFormats::Digits)))
	{
		IFileManager::Get().MakeDirectory(*Path, /*Tree=*/true);
	}

	~FSpecWriteScratch() { IFileManager::Get().DeleteDirectory(*Path, /*RequireExists=*/false, /*Tree=*/true); }
};

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

// --- Size arity ------------------------------------------------------------ //

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSpecWriteSizeArityTest,
	"URLab.MuJoCo.SpecWrite.SizeArity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSpecWriteSizeArityTest::RunTest(const FString& Parameters)
{
	// Authored, not dragged. A gizmo cannot produce this and the shape lock
	// stops it trying; a hand-edited MJCF file, a script and the array widget
	// all can, and every one of them lands in the same build.
	//
	// The contract has two halves and the second is the load-bearing one. The
	// build SAYS the extra values decide nothing, and it CHANGES NOTHING: stock
	// MuJoCo accepts this document and carries all three slots into geom_size
	// (`checksize` bounds its loop by the arity, user_objects.cc:163;
	// `mjCModel::CopyObjects` copies three, user_model.cc:3055), so a build that
	// normalised the size here would compile a different model from the one
	// stock compiles out of the same text. The last assertion is ours against
	// stock over exactly that document, which is what makes the divergence
	// impossible to reintroduce quietly.
	FSpecWriteFixture Fixture;
	if (!Fixture.Init())
	{
		AddError(TEXT("could not build the component tree"));
		return false;
	}

	UMjBody* const Body = Fixture.Add<UMjBody>(Fixture.WorldBody, TEXT("Root"));
	UMjGeom* const Ball = Fixture.Add<UMjGeom>(Body, TEXT("ball"));
	UMjGeom* const Brick = Fixture.Add<UMjGeom>(Body, TEXT("brick"));
	if (Ball == nullptr || Brick == nullptr)
	{
		AddError(TEXT("could not author the two geoms"));
		return false;
	}
	Ball->Type = EMjGeomType::sphere;
	Ball->Size = TArray<double>({ 0.1, 0.2, 0.3 });
	// The control: three values is exactly what a box reads, so a pass that
	// reported both would be reporting length rather than arity.
	Brick->Type = EMjGeomType::box;
	Brick->Size = TArray<double>({ 0.1, 0.2, 0.3 });

	TArray<FMjSpecDiagnostic> Diagnostics;
	urlab::spec::FMjBuiltSpec Built = urlab::spec::BuildSpec(Fixture.Spec(), Diagnostics);
	if (Built.Spec == nullptr)
	{
		for (const FMjSpecDiagnostic& Diagnostic : Diagnostics)
		{
			AddError(Diagnostic.ToString());
		}
		return false;
	}

	TArray<FString> Lines;
	for (const FMjSpecDiagnostic& Diagnostic : Diagnostics)
	{
		Lines.Add(Diagnostic.ToString());
	}
	const FString Reported = FString::Join(Lines, TEXT("; "));
	if (TestEqual(TEXT("one over-long size is reported and one only"), Diagnostics.Num(), 1))
	{
		TestTrue(TEXT("the report says what was authored and what the shape reads"),
			Reported.Contains(TEXT("authors 3 size values where a sphere reads 1")));
		TestTrue(TEXT("and names the element, not just the rule"), Reported.Contains(TEXT("ball")));
		TestFalse(TEXT("the box, which reads all three, is not accused"), Reported.Contains(TEXT("brick")));
		TestFalse(TEXT("a remark does not read as a failure"), MjAnyError(Diagnostics));
	}

	// The compiled model is where this has to be true: the defaults path fails
	// by producing a plausible model rather than an error, so the spec agreeing
	// is not the same as the compile agreeing.
	mjModel* const Model = Compile(*this, Built.Spec);
	if (Model == nullptr)
	{
		return false;
	}
	const int BallId = mj_name2id(Model, mjOBJ_GEOM, "ball");
	if (TestTrue(TEXT("the sphere compiled"), BallId >= 0))
	{
		TestEqual(TEXT("the radius the shape reads is the first value"), Model->geom_size[3 * BallId], 0.1);
		TestEqual(TEXT("and the values it does not read are carried, not dropped"),
			Model->geom_size[3 * BallId + 1], 0.2);
		TestEqual(TEXT("both of them"), Model->geom_size[3 * BallId + 2], 0.3);
	}
	const int BrickId = mj_name2id(Model, mjOBJ_GEOM, "brick");
	if (TestTrue(TEXT("the box compiled"), BrickId >= 0))
	{
		TestEqual(TEXT("a box keeps all three half-extents"),
			Model->geom_size[3 * BrickId + 1], 0.2);
		TestEqual(TEXT("including the last"), Model->geom_size[3 * BrickId + 2], 0.3);
	}

	// Ours against stock, over the document our own writer emits for this tree,
	// so the two compiles are reading the same text and any divergence is the
	// spec path's. Nothing else in the corpus authors an over-long size, so this
	// is the only place the two could drift here without a test noticing.
	FSpecWriteScratch Scratch;
	TArray<FMjSpecDiagnostic> WriteErrors;
	const FString Mjcf = Fixture.Spec().WriteMjcf(&WriteErrors);
	const FString MjcfPath = FPaths::Combine(Scratch.Path, TEXT("arity.xml"));
	if (Mjcf.IsEmpty() || !FFileHelper::SaveStringToFile(Mjcf, *MjcfPath))
	{
		AddError(FString::Printf(TEXT("could not write the document out: %s"),
			WriteErrors.Num() > 0 ? *WriteErrors[0].ToString() : TEXT("empty text")));
		mj_deleteModel(Model);
		return false;
	}

	char Error[1024] = {0};
	mjModel* const Stock = mj_loadXML(TCHAR_TO_UTF8(*MjcfPath), nullptr, Error, sizeof(Error));
	if (Stock == nullptr)
	{
		AddError(FString::Printf(TEXT("stock MuJoCo refused the document our writer emitted: %s"),
			UTF8_TO_TCHAR(Error)));
		mj_deleteModel(Model);
		return false;
	}
	const int StockBallId = mj_name2id(Stock, mjOBJ_GEOM, "ball");
	if (TestTrue(TEXT("stock compiled the sphere too"), StockBallId >= 0 && BallId >= 0))
	{
		for (int32 Slot = 0; Slot < 3; ++Slot)
		{
			TestEqual(FString::Printf(TEXT("size[%d] is what stock MuJoCo compiles from the same text"), Slot),
				Model->geom_size[3 * BallId + Slot], Stock->geom_size[3 * StockBallId + Slot]);
		}
	}
	mj_deleteModel(Stock);
	mj_deleteModel(Model);

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

// --- The unnamed top-level class ------------------------------------------- //

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSpecWriteRootDefaultTest,
	"URLab.MuJoCo.SpecWrite.RootDefault",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSpecWriteRootDefaultTest::RunTest(const FString& Parameters)
{
	// The `<default>` every MJCF document opens its class tree with names no
	// class, because it IS the spec's root class. A spec always has that class,
	// so trying to add it fails -- and the whole class tree under it goes with
	// it, which reaches the model as elements silently carrying engine defaults.
	FSpecWriteFixture Fixture;
	if (!Fixture.Init())
	{
		AddError(TEXT("could not build the component tree"));
		return false;
	}

	UMjDefault* const Root = Fixture.Add<UMjDefault>(Fixture.Robot->Spec);
	UMjGeomBase* const RootGeom = Fixture.Add<UMjGeomBase>(Root);
	UMjDefault* const Named = Fixture.Add<UMjDefault>(Root, TEXT("named"));
	UMjGeomBase* const NamedGeom = Fixture.Add<UMjGeomBase>(Named);
	if (Root == nullptr || RootGeom == nullptr || Named == nullptr || NamedGeom == nullptr)
	{
		AddError(TEXT("could not author the class tree"));
		return false;
	}
	RootGeom->Condim = 6;
	NamedGeom->Group = 3;

	UMjBody* const Body = Fixture.Add<UMjBody>(Fixture.WorldBody, TEXT("Root"));
	UMjGeom* const Plain = Fixture.Add<UMjGeom>(Body, TEXT("Plain"));
	UMjGeom* const Classed = Fixture.Add<UMjGeom>(Body, TEXT("Classed"));
	if (Plain == nullptr || Classed == nullptr)
	{
		AddError(TEXT("could not author the geoms"));
		return false;
	}
	Plain->Type = EMjGeomType::sphere;
	Plain->Size = TArray<double>({ 0.1 });
	Classed->Type = EMjGeomType::sphere;
	Classed->Size = TArray<double>({ 0.1 });
	Classed->Dclass = FString(TEXT("named"));

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

	// A geom naming no class resolves through the root class, so the template
	// written onto it is the only thing that could have set this.
	const int PlainId = mj_name2id(Model, mjOBJ_GEOM, "Plain");
	if (TestTrue(TEXT("the unclassed geom compiled"), PlainId >= 0))
	{
		TestEqual(TEXT("condim came from the top-level class"), Model->geom_condim[PlainId], 6);
	}

	// And a nested class still inherits from it, which is what fails if the
	// top-level class is a second object rather than the spec's own.
	const int ClassedId = mj_name2id(Model, mjOBJ_GEOM, "Classed");
	if (TestTrue(TEXT("the classed geom compiled"), ClassedId >= 0))
	{
		TestEqual(TEXT("condim was inherited through the nested class"),
			Model->geom_condim[ClassedId], 6);
		TestEqual(TEXT("group came from the nested class"), Model->geom_group[ClassedId], 3);
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

// --- The model name -------------------------------------------------------- //

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSpecWriteModelNameTest,
	"URLab.MuJoCo.SpecWrite.ModelName",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSpecWriteModelNameTest::RunTest(const FString& Parameters)
{
	// The model name is written only when the root authored one, which is the
	// condition MuJoCo's own reader writes it under. An unauthored root keeps
	// what mj_makeSpec left, so an untitled document compiles to the same name
	// buffer whether it arrived through a file or through this path.
	const auto NameOfCompiled = [this](bool bAuthored, const TCHAR* Authored) {
		FSpecWriteFixture Fixture;
		if (!Fixture.Init())
		{
			AddError(TEXT("could not build the component tree"));
			return FString();
		}
		if (bAuthored)
		{
			Fixture.Robot->Spec->Model = FString(Authored);
		}
		UMjGeom* const Geom = Fixture.Add<UMjGeom>(Fixture.WorldBody, TEXT("Ball"));
		if (Geom == nullptr)
		{
			AddError(TEXT("could not author the geom"));
			return FString();
		}
		Geom->Type = EMjGeomType::sphere;
		Geom->Size = TArray<double>({ 0.1 });

		urlab::spec::FMjBuiltSpec Built = Build(*this, Fixture.Spec());
		if (Built.Spec == nullptr)
		{
			return FString();
		}
		mjModel* const Model = Compile(*this, Built.Spec);
		if (Model == nullptr)
		{
			return FString();
		}
		// The model name is the first entry of the compiled name buffer.
		const FString Out = FString(UTF8_TO_TCHAR(Model->names));
		mj_deleteModel(Model);
		return Out;
	};

	TestEqual(TEXT("an unauthored root compiles under MuJoCo's own default name"),
		NameOfCompiled(false, nullptr), FString(TEXT("MuJoCo Model")));
	TestEqual(TEXT("an authored name is what compiles"),
		NameOfCompiled(true, TEXT("authored_model")), FString(TEXT("authored_model")));

	return !HasAnyErrors();
}

// --- Nested models --------------------------------------------------------- //

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSpecWriteNestedModelTest,
	"URLab.MuJoCo.SpecWrite.NestedModel",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSpecWriteNestedModelTest::RunTest(const FString& Parameters)
{
	// A `<model>` asset is a whole second document, and `<attach>` splices it
	// in. The child carries its own default class, so what is asserted is the
	// value that class supplies on the compiled model: an attach that brought
	// the bodies across without their classes produces a model that compiles
	// and is quietly wrong.
	FSpecWriteScratch Scratch;
	const FString ChildXml = TEXT(R"(<mujoco model="child_model">
  <default>
    <default class="chunky">
      <geom condim="6" group="3"/>
    </default>
  </default>
  <worldbody>
    <body name="link">
      <geom name="shape" class="chunky" type="sphere" size="0.1"/>
    </body>
  </worldbody>
</mujoco>)");
	const FString ChildPath = FPaths::Combine(Scratch.Path, TEXT("child.xml"));
	if (!FFileHelper::SaveStringToFile(ChildXml, *ChildPath))
	{
		AddError(FString::Printf(TEXT("could not write '%s'"), *ChildPath));
		return false;
	}

	FSpecWriteFixture Fixture;
	if (!Fixture.Init())
	{
		AddError(TEXT("could not build the component tree"));
		return false;
	}

	UMjAsset* const Section = Fixture.Add<UMjAsset>(Fixture.Robot->Spec);
	UMjModelAsset* const Asset = Fixture.Add<UMjModelAsset>(Section, TEXT("child"));
	UMjAttach* const Attach = Fixture.Add<UMjAttach>(Fixture.WorldBody);
	// Twice, because one asset spliced in more than once is the reason a nested
	// model exists at all, and it only works if each attach copies: the second
	// one finds nothing if the first moved the child's body out of it.
	UMjAttach* const Again = Fixture.Add<UMjAttach>(Fixture.WorldBody);
	if (Asset == nullptr || Attach == nullptr || Again == nullptr)
	{
		AddError(TEXT("could not author the model asset and its attaches"));
		return false;
	}
	Asset->File = FString(TEXT("child.xml"));
	// The file resolves against the element's own source directory, so a tree
	// nobody parsed has to say where it would have come from.
	Asset->SourceFile = FPaths::Combine(Scratch.Path, TEXT("parent.xml"));
	// The asset's own name renames the child spec, and the renamed child is
	// what the attach looks up.
	Attach->Model = FString(TEXT("child"));
	Attach->Body = FString(TEXT("link"));
	Attach->Prefix = FString(TEXT("c_"));
	Again->Model = FString(TEXT("child"));
	Again->Body = FString(TEXT("link"));
	Again->Prefix = FString(TEXT("d_"));

	urlab::spec::FMjBuiltSpec Built = Build(*this, Fixture.Spec());
	if (Built.Spec == nullptr)
	{
		return false;
	}
	mjModel* const Model = Compile(*this, Built.Spec);
	if (Model == nullptr)
	{
		return false;
	}

	for (const char* const Prefix : {"c_", "d_"})
	{
		const FString Label = UTF8_TO_TCHAR(Prefix);
		TestTrue(FString::Printf(TEXT("the child's body came across under '%s'"), *Label),
			mj_name2id(Model, mjOBJ_BODY, TCHAR_TO_UTF8(*(Label + TEXT("link")))) >= 0);
		const int GeomId = mj_name2id(Model, mjOBJ_GEOM, TCHAR_TO_UTF8(*(Label + TEXT("shape"))));
		if (TestTrue(FString::Printf(TEXT("the child's geom came across under '%s'"), *Label), GeomId >= 0))
		{
			TestEqual(FString::Printf(TEXT("%s: condim came from the child's own class"), *Label),
				Model->geom_condim[GeomId], 6);
			TestEqual(FString::Printf(TEXT("%s: group came from the child's own class"), *Label),
				Model->geom_group[GeomId], 3);
		}
	}
	mj_deleteModel(Model);

	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSpecWriteSelfAttachTest,
	"URLab.MuJoCo.SpecWrite.SelfAttach",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSpecWriteSelfAttachTest::RunTest(const FString& Parameters)
{
	// An `<attach>` naming no model duplicates a body of the current spec into
	// another body of it. The duplicate keeps the original's default class
	// rather than a prefixed one, because MuJoCo renames classes only across
	// specs -- so the copy carrying the class's value is the assertion that says
	// the duplication went through the resolver rather than past it.
	FSpecWriteFixture Fixture;
	if (!Fixture.Init())
	{
		AddError(TEXT("could not build the component tree"));
		return false;
	}

	UMjDefault* const Class = Fixture.Add<UMjDefault>(Fixture.Robot->Spec, TEXT("chunky"));
	UMjGeomBase* const ClassGeom = Fixture.Add<UMjGeomBase>(Class);
	UMjBody* const Link = Fixture.Add<UMjBody>(Fixture.WorldBody, TEXT("link"));
	UMjGeom* const Shape = Fixture.Add<UMjGeom>(Link, TEXT("shape"));
	UMjBody* const Host = Fixture.Add<UMjBody>(Fixture.WorldBody, TEXT("host"));
	UMjAttach* const Attach = Fixture.Add<UMjAttach>(Host);
	if (ClassGeom == nullptr || Shape == nullptr || Host == nullptr || Attach == nullptr)
	{
		AddError(TEXT("could not author the body and its duplicate"));
		return false;
	}
	ClassGeom->Condim = 6;
	Shape->Dclass = FString(TEXT("chunky"));
	Shape->Type = EMjGeomType::sphere;
	Shape->Size = TArray<double>({ 0.1 });
	Attach->Body = FString(TEXT("link"));
	Attach->Prefix = FString(TEXT("dup_"));

	urlab::spec::FMjBuiltSpec Built = Build(*this, Fixture.Spec());
	if (Built.Spec == nullptr)
	{
		return false;
	}
	mjModel* const Model = Compile(*this, Built.Spec);
	if (Model == nullptr)
	{
		return false;
	}

	TestTrue(TEXT("the original body is still there"), mj_name2id(Model, mjOBJ_BODY, "link") >= 0);
	const int HostId = mj_name2id(Model, mjOBJ_BODY, "host");
	const int CopyBodyId = mj_name2id(Model, mjOBJ_BODY, "dup_link");
	if (TestTrue(TEXT("the duplicate is there under its prefix"), HostId >= 0 && CopyBodyId >= 0))
	{
		// Under the body the attach was authored in, which is what says the
		// frame it hangs from was added where the walk was rather than at the
		// top of the tree.
		TestEqual(TEXT("the duplicate hangs off the attaching body"), Model->body_parentid[CopyBodyId], HostId);
	}
	const int OriginalId = mj_name2id(Model, mjOBJ_GEOM, "shape");
	const int CopyId = mj_name2id(Model, mjOBJ_GEOM, "dup_shape");
	if (TestTrue(TEXT("both geoms compiled"), OriginalId >= 0 && CopyId >= 0))
	{
		TestEqual(TEXT("the original resolved its class"), Model->geom_condim[OriginalId], 6);
		TestEqual(TEXT("the duplicate resolved the same class"), Model->geom_condim[CopyId], 6);
	}
	mj_deleteModel(Model);

	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSpecWriteWholeModelAttachTest,
	"URLab.MuJoCo.SpecWrite.WholeModelAttach",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSpecWriteWholeModelAttachTest::RunTest(const FString& Parameters)
{
	// An `<attach>` that names a model but neither a body nor a frame takes the
	// child spec's own element as the source, and MuJoCo then splices the whole
	// of the child's world body: every top-level body of it at once, and the
	// content hanging off the world body directly. The child here has both, and
	// two top-level bodies rather than one, because either is enough on its own
	// to tell this apart from a source resolved to some single named body -- and
	// the world body itself is not reproduced, which is what the body count says.
	FSpecWriteScratch Scratch;
	const FString ChildXml = TEXT(R"(<mujoco model="child_model">
  <default>
    <default class="chunky">
      <geom condim="6"/>
    </default>
  </default>
  <worldbody>
    <geom name="pad" class="chunky" type="box" size="0.2 0.2 0.02"/>
    <body name="alpha">
      <geom name="alpha_shape" type="sphere" size="0.1"/>
    </body>
    <body name="beta">
      <geom name="beta_shape" type="sphere" size="0.1"/>
    </body>
  </worldbody>
</mujoco>)");
	const FString ChildPath = FPaths::Combine(Scratch.Path, TEXT("child.xml"));
	if (!FFileHelper::SaveStringToFile(ChildXml, *ChildPath))
	{
		AddError(FString::Printf(TEXT("could not write '%s'"), *ChildPath));
		return false;
	}

	FSpecWriteFixture Fixture;
	if (!Fixture.Init())
	{
		AddError(TEXT("could not build the component tree"));
		return false;
	}

	UMjAsset* const Section = Fixture.Add<UMjAsset>(Fixture.Robot->Spec);
	UMjModelAsset* const Asset = Fixture.Add<UMjModelAsset>(Section, TEXT("child"));
	UMjBody* const Host = Fixture.Add<UMjBody>(Fixture.WorldBody, TEXT("host"));
	UMjAttach* const Attach = Fixture.Add<UMjAttach>(Host);
	if (Asset == nullptr || Host == nullptr || Attach == nullptr)
	{
		AddError(TEXT("could not author the model asset and its attach"));
		return false;
	}
	Asset->File = FString(TEXT("child.xml"));
	Asset->SourceFile = FPaths::Combine(Scratch.Path, TEXT("parent.xml"));
	Attach->Model = FString(TEXT("child"));
	Attach->Prefix = FString(TEXT("kid_"));

	urlab::spec::FMjBuiltSpec Built = Build(*this, Fixture.Spec());
	if (Built.Spec == nullptr)
	{
		return false;
	}
	mjModel* const Model = Compile(*this, Built.Spec);
	if (Model == nullptr)
	{
		return false;
	}

	const int HostId = mj_name2id(Model, mjOBJ_BODY, "host");
	const int AlphaId = mj_name2id(Model, mjOBJ_BODY, "kid_alpha");
	const int BetaId = mj_name2id(Model, mjOBJ_BODY, "kid_beta");
	if (TestTrue(TEXT("both of the child's top-level bodies came across under the prefix"),
			HostId >= 0 && AlphaId >= 0 && BetaId >= 0))
	{
		TestEqual(TEXT("the first hangs off the attaching body"), Model->body_parentid[AlphaId], HostId);
		TestEqual(TEXT("the second hangs off it too, as a sibling"), Model->body_parentid[BetaId], HostId);
	}
	// The world body, the host, and the child's two: a source resolved to one
	// named body of the child would be one short, and a world body spliced in as
	// a body of its own rather than unwrapped would be one over.
	TestEqual(TEXT("the child's world body was unwrapped rather than reproduced"),
		static_cast<int>(Model->nbody), 4);

	const int PadId = mj_name2id(Model, mjOBJ_GEOM, "kid_pad");
	if (TestTrue(TEXT("the geom on the child's world body came across"), PadId >= 0))
	{
		TestEqual(TEXT("it landed on the attaching body"), Model->geom_bodyid[PadId], HostId);
		TestEqual(TEXT("it kept the value of the child's own class"), Model->geom_condim[PadId], 6);
	}
	mj_deleteModel(Model);

	return !HasAnyErrors();
}

#endif  // URLAB_MJ_GEN && WITH_DEV_AUTOMATION_TESTS
