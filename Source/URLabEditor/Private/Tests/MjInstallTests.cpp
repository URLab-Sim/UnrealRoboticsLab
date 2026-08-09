// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_DEV_AUTOMATION_TESTS

#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Elements/MjGeom.h"
#include "MuJoCo/Gen/Elements/Options/MjFlag.gen.h"
#include "MuJoCo/Gen/Elements/Options/MjOption.gen.h"
#include "MuJoCo/Spec/MjSceneSpec.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "Tests/MjTestHelpers.h"

THIRD_PARTY_INCLUDES_START
#include <mujoco/mujoco.h>
THIRD_PARTY_INCLUDES_END

// Installing a compiled scene into a running engine.
//
// Two properties are asserted here and neither is visible from the compile on
// its own: that a compile which fails costs the session nothing, and that the
// simulation options a model runs under come from the spec and from nowhere
// else. Both are about what the ENGINE does with a compile, so both are driven
// through the install rather than through the builder.

namespace
{

// Qualified rather than imported: the engine has a compiled-scene type of its
// own under the same name, and the two are different types.
namespace mjspec = urlab::spec;

// Nothing's defaults, so an assertion cannot pass on a model that ignored the
// authored section entirely. At namespace scope because the session's config
// callback is a non-capturing lambda.
constexpr double AuthoredTimestep = 0.004;
constexpr double AuthoredGravityZ = -3.25;
constexpr double AuthoredImpratio = 3.5;

/**
 * Compile the session's level a second time, without the engine.
 *
 * The same scene root and the same participant, through the same builder, with
 * nothing installed and no engine to post-process the result. It is the
 * reference for what the spec alone says the model should be.
 */
mjspec::FMjCompiledScene CompileBeside(const FMjUESession& Session)
{
	mjspec::FMjSceneSpecBuilder Builder;
	Builder.SetSceneRoot(Session.Manager->GetSceneSpec());

	mjspec::FMjSceneSpecParticipant Placed;
	Placed.Spec = Session.Robot->GetSpec();
	Placed.Prefix = Session.Robot->GetCompiledPrefix();
	Builder.AddParticipant(Placed);

	return Builder.Compile();
}

}  // namespace

// --- A failed compile costs the session nothing ----------------------------- //

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjInstallFailedCompileKeepsSessionTest,
	"URLab.MuJoCo.Install.FailedCompileKeepsTheSession",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjInstallFailedCompileKeepsSessionTest::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(S.LastError);
		return false;
	}

	UMjPhysicsEngine* const Engine = S.Manager->PhysicsEngine;
	const mjModel* const RunningModel = Engine->m_model;
	const mjData* const RunningData = Engine->m_data;
	const int32 GeomIdBefore = S.Geom->GetBoundId().Get(-1);
	if (!TestTrue(TEXT("the geom bound before the edit"), GeomIdBefore >= 0))
	{
		return false;
	}

	S.Step(5);
	const double TimeBefore = Engine->m_data->time;
	if (!TestTrue(TEXT("the session was stepping"), TimeBefore > 0.0))
	{
		return false;
	}

	// A sphere cannot have a negative radius. The spec API has no opinion about
	// it, so this fails in the compiler, which is the failure the install has to
	// survive.
	S.Geom->Type = EMjGeomType::sphere;
	S.Geom->Size = TArray<double>({-1.0});

	// Driven directly rather than through Compile(), which reports a failure by
	// opening a modal dialog that no automation run can answer.
	FString Error;
	TestFalse(TEXT("the scene did not compile"), Engine->InstallCompiledSpec(Error));
	TestFalse(TEXT("and the failure says why"), Error.IsEmpty());

	TestTrue(TEXT("the model is still the one that was running"), Engine->m_model == RunningModel);
	TestTrue(TEXT("and so is its data"), Engine->m_data == RunningData);
	TestEqual(TEXT("the session did not lose its place"), Engine->m_data->time, TimeBefore, 1e-12);
	TestEqual(TEXT("the geom still answers with the id it had"), S.Geom->GetBoundId().Get(-1), GeomIdBefore);

	// The proof the rest of it is for: it still steps, against the model it was
	// stepping before anybody asked for a new one.
	S.Step(5);
	TestTrue(TEXT("and it is still stepping"), Engine->m_data->time > TimeBefore);

	S.Cleanup();
	return true;
}

// --- Options reach the model through the spec and nowhere else -------------- //

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjInstallOptionAuthorityTest, "URLab.MuJoCo.Install.OptionAuthorityIsTheSpec",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjInstallOptionAuthorityTest::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init([](FMjUESession& Sess) {
			Sess.Manager->SceneOption->Timestep = AuthoredTimestep;
			Sess.Manager->SceneOption->Gravity = FMjDirection3(0.0, 0.0, AuthoredGravityZ);
			Sess.Manager->SceneOption->Impratio = AuthoredImpratio;
			Sess.Manager->SceneOption->Integrator = EMjIntegrator::RK4;
			Sess.Manager->SceneFlags->Sleep = EMjEnable::enable;
		}))
	{
		AddError(S.LastError);
		return false;
	}

	const mjModel* const Installed = S.Model();
	if (!TestNotNull(TEXT("the installed model"), Installed))
	{
		return false;
	}

	TestEqual(TEXT("the authored timestep reached the model"), Installed->opt.timestep, AuthoredTimestep, 1e-12);
	TestEqual(TEXT("the authored gravity reached the model"), Installed->opt.gravity[2], AuthoredGravityZ, 1e-12);
	TestEqual(TEXT("the authored impratio reached the model"), Installed->opt.impratio, AuthoredImpratio, 1e-12);
	TestEqual(TEXT("the authored integrator reached the model"), static_cast<int32>(Installed->opt.integrator),
		static_cast<int32>(mjINT_RK4));
	TestTrue(TEXT("the authored sleep flag reached the model"), (Installed->opt.enableflags & mjENBL_SLEEP) != 0);

	// The same specs compiled with no engine in the way, so nothing could have
	// written over the result afterwards. Every field the engine's retired
	// post-compile stamp used to touch is compared against it: a stamp that ever
	// disagreed with the compiler used to win silently, and this is the
	// assertion that would not let it.
	const mjspec::FMjCompiledScene Beside = CompileBeside(S);
	if (!TestTrue(TEXT("the reference compile succeeded"), Beside.IsValid()))
	{
		for (const FMjSpecDiagnostic& Diagnostic : Beside.Errors)
		{
			AddError(Diagnostic.ToString());
		}
		S.Cleanup();
		return false;
	}

	const mjOption& Spec = Beside.Model->opt;
	const mjOption& Live = Installed->opt;
	TestEqual(TEXT("timestep is the compiler's"), Live.timestep, Spec.timestep, 1e-12);
	TestEqual(TEXT("impratio is the compiler's"), Live.impratio, Spec.impratio, 1e-12);
	TestEqual(TEXT("tolerance is the compiler's"), Live.tolerance, Spec.tolerance, 1e-12);
	TestEqual(TEXT("sleep_tolerance is the compiler's"), Live.sleep_tolerance, Spec.sleep_tolerance, 1e-12);
	TestEqual(TEXT("density is the compiler's"), Live.density, Spec.density, 1e-12);
	TestEqual(TEXT("viscosity is the compiler's"), Live.viscosity, Spec.viscosity, 1e-12);
	for (int32 Axis = 0; Axis < 3; ++Axis)
	{
		TestEqual(TEXT("gravity is the compiler's"), Live.gravity[Axis], Spec.gravity[Axis], 1e-12);
		TestEqual(TEXT("wind is the compiler's"), Live.wind[Axis], Spec.wind[Axis], 1e-12);
		TestEqual(TEXT("magnetic is the compiler's"), Live.magnetic[Axis], Spec.magnetic[Axis], 1e-12);
	}
	TestEqual(TEXT("integrator is the compiler's"), static_cast<int32>(Live.integrator),
		static_cast<int32>(Spec.integrator));
	TestEqual(TEXT("cone is the compiler's"), static_cast<int32>(Live.cone), static_cast<int32>(Spec.cone));
	TestEqual(TEXT("jacobian is the compiler's"), static_cast<int32>(Live.jacobian),
		static_cast<int32>(Spec.jacobian));
	TestEqual(TEXT("solver is the compiler's"), static_cast<int32>(Live.solver), static_cast<int32>(Spec.solver));
	TestEqual(TEXT("iterations is the compiler's"), static_cast<int32>(Live.iterations),
		static_cast<int32>(Spec.iterations));
	TestEqual(TEXT("enableflags is the compiler's"), static_cast<int32>(Live.enableflags),
		static_cast<int32>(Spec.enableflags));
	TestEqual(TEXT("disableflags is the compiler's"), static_cast<int32>(Live.disableflags),
		static_cast<int32>(Spec.disableflags));
	TestEqual(TEXT("disableactuator is the compiler's"), static_cast<int32>(Live.disableactuator),
		static_cast<int32>(Spec.disableactuator));

	S.Cleanup();
	return true;
}

#endif  // URLAB_MJ_GEN && WITH_DEV_AUTOMATION_TESTS
