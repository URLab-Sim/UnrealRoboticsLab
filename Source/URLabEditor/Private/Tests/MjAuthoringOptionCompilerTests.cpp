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
#include "MuJoCo/Elements/MjGeom.h"
#include "MuJoCo/Gen/Elements/Options/MjFlag.gen.h"
#include "MuJoCo/Gen/Elements/Options/MjOption.gen.h"
#include "MuJoCo/Spec/MjSceneSpec.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "Tests/MjTestHelpers.h"

THIRD_PARTY_INCLUDES_START
#include <mujoco/mujoco.h>
THIRD_PARTY_INCLUDES_END

// Authoring compiler acceptance: authored MuJoCo options survive the compiler
// and match an independent compile of the same level spec.

namespace
{

// Qualified explicitly to distinguish the generated scene compiler API.
namespace mjspec = urlab::spec;

// Nothing's defaults, so an assertion cannot pass on a model that ignored the
// authored section entirely. At namespace scope because the session's config
// callback is a non-capturing lambda.
constexpr double AuthoredTimestep = 0.004;
constexpr double AuthoredGravityZ = -3.25;
constexpr double AuthoredImpratio = 3.5;

/**
 * Compile the session's level a second time through the spec builder.
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

} // namespace

// --- Authored options reach the compiled model ------------------------------- //

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjAuthoringOptionAuthorityTest, "URLab.MuJoCo.Authoring.OptionsComeFromTheSpec",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjAuthoringOptionAuthorityTest::RunTest(const FString& Parameters)
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

	const mjModel* const CompiledModel = S.Model();
	if (!TestNotNull(TEXT("the compiled model"), CompiledModel))
	{
		return false;
	}

	TestEqual(TEXT("the authored timestep reached the model"), CompiledModel->opt.timestep, AuthoredTimestep, 1e-12);
	TestEqual(TEXT("the authored gravity reached the model"), CompiledModel->opt.gravity[2], AuthoredGravityZ, 1e-12);
	TestEqual(TEXT("the authored impratio reached the model"), CompiledModel->opt.impratio, AuthoredImpratio, 1e-12);
	TestEqual(TEXT("the authored integrator reached the model"), static_cast<int32>(CompiledModel->opt.integrator),
		static_cast<int32>(mjINT_RK4));
	TestTrue(TEXT("the authored sleep flag reached the model"), (CompiledModel->opt.enableflags & mjENBL_SLEEP) != 0);

	// Compare with an independent spec-builder compile so the acceptance covers
	// both the authored handoff and the resulting MuJoCo option values.
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
	const mjOption& Compiled = CompiledModel->opt;
	TestEqual(TEXT("timestep matches an independent compile"), Compiled.timestep, Spec.timestep, 1e-12);
	TestEqual(TEXT("impratio matches an independent compile"), Compiled.impratio, Spec.impratio, 1e-12);
	TestEqual(TEXT("tolerance matches an independent compile"), Compiled.tolerance, Spec.tolerance, 1e-12);
	TestEqual(TEXT("sleep_tolerance matches an independent compile"), Compiled.sleep_tolerance, Spec.sleep_tolerance, 1e-12);
	TestEqual(TEXT("density matches an independent compile"), Compiled.density, Spec.density, 1e-12);
	TestEqual(TEXT("viscosity matches an independent compile"), Compiled.viscosity, Spec.viscosity, 1e-12);
	for (int32 Axis = 0; Axis < 3; ++Axis)
	{
		TestEqual(TEXT("gravity matches an independent compile"), Compiled.gravity[Axis], Spec.gravity[Axis], 1e-12);
		TestEqual(TEXT("wind matches an independent compile"), Compiled.wind[Axis], Spec.wind[Axis], 1e-12);
		TestEqual(TEXT("magnetic matches an independent compile"), Compiled.magnetic[Axis], Spec.magnetic[Axis], 1e-12);
	}
	TestEqual(TEXT("integrator matches an independent compile"), static_cast<int32>(Compiled.integrator),
		static_cast<int32>(Spec.integrator));
	TestEqual(TEXT("cone matches an independent compile"), static_cast<int32>(Compiled.cone), static_cast<int32>(Spec.cone));
	TestEqual(TEXT("jacobian matches an independent compile"), static_cast<int32>(Compiled.jacobian),
		static_cast<int32>(Spec.jacobian));
	TestEqual(TEXT("solver matches an independent compile"), static_cast<int32>(Compiled.solver), static_cast<int32>(Spec.solver));
	TestEqual(TEXT("iterations matches an independent compile"), static_cast<int32>(Compiled.iterations),
		static_cast<int32>(Spec.iterations));
	TestEqual(TEXT("enableflags matches an independent compile"), static_cast<int32>(Compiled.enableflags),
		static_cast<int32>(Spec.enableflags));
	TestEqual(TEXT("disableflags matches an independent compile"), static_cast<int32>(Compiled.disableflags),
		static_cast<int32>(Spec.disableflags));
	TestEqual(TEXT("disableactuator matches an independent compile"), static_cast<int32>(Compiled.disableactuator),
		static_cast<int32>(Spec.disableactuator));

	S.Cleanup();
	return true;
}

#endif // URLAB_MJ_GEN && WITH_DEV_AUTOMATION_TESTS
