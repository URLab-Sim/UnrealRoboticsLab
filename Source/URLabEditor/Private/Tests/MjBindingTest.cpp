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
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
// trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Tests/MjTestHelpers.h"

#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Elements/MjBody.h"
#include "MuJoCo/Elements/MjGeom.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

// A spec authored element by element, rather than read from MJCF, taken
// through the same compile every other path uses. The two things it proves are
// the two the reader cannot: that a tree built through the factories binds, and
// that a runtime write reaches the model the elements were bound into.

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjBindingIntegrationTest, "URLab.MuJoCo.Binding.Integration",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjBindingIntegrationTest::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(S.LastError);
		return false;
	}

	TestTrue(TEXT("Manager should be initialized"), S.Manager->IsInitialized());

	// The id an element carries has to be the id its name resolves to in the
	// compiled model; the model is asked by name and nothing else.
	const int32 GeomId = S.MjId(mjOBJ_GEOM, TEXT("TestGeom"));
	const int32 JointId = S.MjId(mjOBJ_JOINT, TEXT("TestJoint"));
	TestTrue(TEXT("Geom survived the compile"), GeomId >= 0);
	TestTrue(TEXT("Joint survived the compile"), JointId >= 0);

	if (TestTrue(TEXT("Geom is bound"), S.Geom->GetBoundId().IsSet()))
	{
		TestEqual(TEXT("Geom bound to the id its name resolves to"), S.Geom->GetBoundId().GetValue(), GeomId);
	}
	if (TestTrue(TEXT("Joint is bound"), S.Joint->GetBoundId().IsSet()))
	{
		TestEqual(TEXT("Joint bound to the id its name resolves to"), S.Joint->GetBoundId().GetValue(), JointId);
	}

	// A runtime friction write is a model edit rather than a spec one, so
	// it is spelled against the engine and read back out of the model.
	if (GeomId >= 0)
	{
		const double NewFriction = 0.5;
		S.Manager->PhysicsEngine->ApplyGeomFriction(GeomId, NewFriction);
		TestEqual(TEXT("Geom friction should be updated"),
			S.Model()->geom_friction[GeomId * 3], NewFriction);
	}

	S.Cleanup();
	return true;
}
