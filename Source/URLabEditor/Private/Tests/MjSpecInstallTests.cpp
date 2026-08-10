// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// From MJCF to a runtime read, with nothing in between but the spec.
//
// Each half of this chain already had a gate. The reader and writer are held
// to a fixpoint and the compiler to a byte-identical model; the runtime
// libraries are held to classifying and correcting. What none of them covers
// is the join: that the id an element is told after a compile is the id that
// indexes the model it was compiled into. That is a single integer and it is
// the only thing holding the two halves together, so it is asserted against a
// simulation rather than against a table.
//
// The reads go through the libraries deliberately. Reading d->qpos here and
// calling it proven would test the test; going through UMjJointRuntime is what
// a caller does, and it is the path that resolves the engine, the binding and
// the address separately, any one of which could be wrong on its own.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Engine/Engine.h"
#include "Engine/World.h"

#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Elements/MjActuatorRuntime.h"
#include "MuJoCo/Elements/MjJointRuntime.h"
#include "MuJoCo/Elements/MjSensorRuntime.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace MjSpecInstallTests
{

/** A hinge with something to drive it and something to watch it. */
const TCHAR* const Corpus = TEXT(R"(<mujoco model="install">
  <compiler angle="radian"/>
  <worldbody>
    <body name="base" pos="0 0 0.5">
      <joint name="hinge" type="hinge" axis="0 1 0" range="-1.5 1.5"/>
      <geom name="link" type="capsule" size="0.02 0.15"/>
      <site name="tip" pos="0 0 0.3"/>
    </body>
  </worldbody>
  <actuator>
    <position name="drive" joint="hinge" kp="120"/>
  </actuator>
  <sensor>
    <jointpos name="hingepos" joint="hinge"/>
  </sensor>
</mujoco>
)");

/** A world holding one manager and one articulation, torn down on scope exit. */
struct FScene
{
	UWorld* World = nullptr;
	AAMjManager* Manager = nullptr;
	AMjArticulation* Articulation = nullptr;
	FString LastError;

	bool Open()
	{
		World = UWorld::CreateWorld(EWorldType::Game, false);
		if (World == nullptr)
		{
			LastError = TEXT("CreateWorld failed");
			return false;
		}
		FWorldContext& Context = GEngine->CreateNewWorldContext(EWorldType::Game);
		Context.SetCurrentWorld(World);

		FActorSpawnParameters Params;
		Articulation = World->SpawnActor<AMjArticulation>(Params);
		Manager = World->SpawnActor<AAMjManager>(Params);
		if (Articulation == nullptr || Manager == nullptr)
		{
			LastError = TEXT("SpawnActor failed");
			return false;
		}

		const FMjSpecParseResult Parsed = MjParseIntoActor(*Articulation, Corpus, TEXT("<inline>"));
		if (!Parsed.IsOk())
		{
			LastError = TEXT("the corpus did not parse into the articulation");
			return false;
		}
		return true;
	}

	bool Install()
	{
		return Manager->PhysicsEngine->InstallCompiledSpec(LastError);
	}

	mjModel* Model() const { return Manager != nullptr ? Manager->PhysicsEngine->GetModel() : nullptr; }
	mjData* Data() const { return Manager != nullptr ? Manager->PhysicsEngine->GetData() : nullptr; }

	/** The element the articulation authored under `MjName`, or null. */
	UMjNodeComponent* Node(const TCHAR* MjName) const
	{
		if (Articulation == nullptr)
		{
			return nullptr;
		}
		TArray<UMjNodeComponent*> Nodes;
		Articulation->GetComponents(Nodes);
		for (UMjNodeComponent* Candidate : Nodes)
		{
			if (Candidate != nullptr && Candidate->MjName.IsSet() && Candidate->MjName.GetValue() == MjName)
			{
				return Candidate;
			}
		}
		return nullptr;
	}

	/** The id the compiled model gave `MjName`, found by name and nothing else. */
	int32 CompiledId(mjtObj Type, const TCHAR* MjName) const
	{
		mjModel* M = Model();
		if (M == nullptr || Articulation == nullptr)
		{
			return -1;
		}
		const FString Full = Articulation->GetName() + TEXT("_") + MjName;
		return mj_name2id(M, Type, TCHAR_TO_UTF8(*Full));
	}

	~FScene()
	{
		if (Manager != nullptr)
		{
			Manager->PhysicsEngine->bShouldStopTask = true;
		}
		if (World != nullptr)
		{
			World->DestroyWorld(false);
			GEngine->DestroyWorldContext(World);
		}
	}
};

}  // namespace MjSpecInstallTests

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSpecInstallBindsElements,
	"URLab.Doc.InstalledSpecBindsItsElements",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSpecInstallBindsElements::RunTest(const FString& Parameters)
{
	MjSpecInstallTests::FScene Scene;
	if (!Scene.Open())
	{
		AddError(Scene.LastError);
		return false;
	}
	if (!Scene.Install())
	{
		AddError(FString::Printf(TEXT("install failed: %s"), *Scene.LastError));
		return false;
	}

	mjModel* Model = Scene.Model();
	if (!TestNotNull(TEXT("a model was installed"), Model))
	{
		return false;
	}
	TestNotNull(TEXT("data was made for it"), Scene.Data());
	TestEqual(TEXT("the corpus compiled one actuator"), static_cast<int32>(Model->nu), 1);
	TestEqual(TEXT("and one sensor"), static_cast<int32>(Model->nsensor), 1);

	// The id an element carries has to be the id its name resolves to in the
	// model. Anything else and every read below is reading someone else.
	struct FCase
	{
		const TCHAR* Name;
		mjtObj Type;
	};
	const FCase Cases[] = {
		{TEXT("hinge"), mjOBJ_JOINT},
		{TEXT("drive"), mjOBJ_ACTUATOR},
		{TEXT("hingepos"), mjOBJ_SENSOR},
		{TEXT("base"), mjOBJ_BODY},
		{TEXT("link"), mjOBJ_GEOM},
	};
	for (const FCase& Case : Cases)
	{
		UMjNodeComponent* Element = Scene.Node(Case.Name);
		if (!TestNotNull(FString::Printf(TEXT("the spec holds %s"), Case.Name), Element))
		{
			continue;
		}
		const int32 Expected = Scene.CompiledId(Case.Type, Case.Name);
		TestTrue(FString::Printf(TEXT("%s survived the compile"), Case.Name), Expected >= 0);
		if (!TestTrue(FString::Printf(TEXT("%s is bound"), Case.Name), Element->GetBoundId().IsSet()))
		{
			continue;
		}
		TestEqual(FString::Printf(TEXT("%s bound to the id its name resolves to"), Case.Name),
			Element->GetBoundId().GetValue(), Expected);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSpecInstallReadsThroughLibraries,
	"URLab.Doc.InstalledSpecReadsThroughLibraries",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSpecInstallReadsThroughLibraries::RunTest(const FString& Parameters)
{
	MjSpecInstallTests::FScene Scene;
	if (!Scene.Open())
	{
		AddError(Scene.LastError);
		return false;
	}
	if (!Scene.Install())
	{
		AddError(FString::Printf(TEXT("install failed: %s"), *Scene.LastError));
		return false;
	}

	UMjNodeComponent* Joint = Scene.Node(TEXT("hinge"));
	UMjNodeComponent* Sensor = Scene.Node(TEXT("hingepos"));
	UMjNodeComponent* Actuator = Scene.Node(TEXT("drive"));
	if (!TestNotNull(TEXT("joint"), Joint) || !TestNotNull(TEXT("sensor"), Sensor)
		|| !TestNotNull(TEXT("actuator"), Actuator))
	{
		return false;
	}

	TestTrue(TEXT("the joint classifies as one"), UMjJointRuntime::IsJoint(Joint));
	TestTrue(TEXT("the sensor classifies as one"), UMjSensorRuntime::IsSensor(Sensor));
	TestTrue(TEXT("the actuator classifies as one"), UMjActuatorRuntime::IsActuator(Actuator));

	// A jointpos sensor reads one value, and only the compiled model knows
	// that -- the schema does not carry a width for every kind.
	TestEqual(TEXT("the sensor reports its compiled width"), UMjSensorRuntime::GetDimension(Sensor), 1);

	// Write through the library, read back through the model, and then read
	// back through the library: the address is only proven if both agree.
	UMjJointRuntime::SetPosition(Joint, 0.4f);
	mjModel* Model = Scene.Model();
	mjData* Data = Scene.Data();
	const int32 JointId = Joint->GetBoundId().GetValue();
	const int32 QposAdr = Model->jnt_qposadr[JointId];
	TestEqual(TEXT("the write landed on the joint's own qpos slot"),
		static_cast<float>(Data->qpos[QposAdr]), 0.4f);
	TestEqual(TEXT("and reads back through the library"), UMjJointRuntime::GetPosition(Joint), 0.4f);

	// The sensor is downstream of that qpos, so a forward pass has to move it.
	Scene.Manager->PhysicsEngine->ForwardSync();
	const TArray<float> Reading = UMjSensorRuntime::GetReading(Sensor);
	if (TestEqual(TEXT("the sensor read one value"), Reading.Num(), 1))
	{
		TestTrue(TEXT("and it followed the joint"), FMath::Abs(Reading[0] - 0.4f) < 1e-3f);
	}

	// Range comes from the model, not from the authored attribute, which is
	// the whole reason a compiled read exists beside the spec.
	const FVector2D Range = UMjJointRuntime::GetJointRange(Joint);
	TestTrue(TEXT("the joint range is the compiled one"),
		FMath::Abs(Range.X + 1.5f) < 1e-3f && FMath::Abs(Range.Y - 1.5f) < 1e-3f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSpecInstallSizesControlSlots,
	"URLab.Doc.InstalledSpecSizesControlSlots",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSpecInstallSizesControlSlots::RunTest(const FString& Parameters)
{
	MjSpecInstallTests::FScene Scene;
	if (!Scene.Open())
	{
		AddError(Scene.LastError);
		return false;
	}
	if (!Scene.Install())
	{
		AddError(FString::Printf(TEXT("install failed: %s"), *Scene.LastError));
		return false;
	}

	UMjNodeComponent* Actuator = Scene.Node(TEXT("drive"));
	if (!TestNotNull(TEXT("actuator"), Actuator))
	{
		return false;
	}
	const int32 ActuatorId = Actuator->GetBoundId().Get(-1);

	AMjArticulation* Art = Scene.Articulation;
	TestEqual(TEXT("the slots are sized to the compiled scene"), Art->GetControlSlotCount(),
		static_cast<int32>(Scene.Model()->nu));
	TestTrue(TEXT("the articulation owns its own actuator"), Art->GetOwnedActuatorIds().Contains(ActuatorId));

	// Staging goes through the element and lands on the articulation's slot
	// for that element's compiled id: the one thing the two halves have to
	// agree on for the step loop to write the right ctrl entry.
	Art->ControlSource = 0;
	UMjActuatorRuntime::SetNetworkControl(Actuator, 0.6f);
	TestEqual(TEXT("staged on the element, resolved on the articulation"),
		Art->ResolveDesiredControl(ActuatorId), 0.6f);
	TestEqual(TEXT("and read back through the element"), UMjActuatorRuntime::GetControl(Actuator), 0.6f);

	Art->ControlSource = 1;
	TestEqual(TEXT("the UI source does not see the network slot"),
		UMjActuatorRuntime::GetControl(Actuator), 0.0f);
	UMjActuatorRuntime::SetControl(Actuator, -0.3f);
	TestEqual(TEXT("but does see its own"), UMjActuatorRuntime::GetControl(Actuator), -0.3f);

	// A compiled read is a different question from a staged one: nothing has
	// stepped, so d->ctrl is still zero while both slots hold values.
	TestEqual(TEXT("staging is not applying"), UMjActuatorRuntime::GetAppliedControl(Actuator), 0.0f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSpecInstallUnbindsOnRecompile,
	"URLab.Doc.InstalledSpecUnbindsOnRecompile",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSpecInstallUnbindsOnRecompile::RunTest(const FString& Parameters)
{
	MjSpecInstallTests::FScene Scene;
	if (!Scene.Open())
	{
		AddError(Scene.LastError);
		return false;
	}

	// Before the first install there is no model, so an element is unbound and
	// every library read is the empty answer rather than an index into nothing.
	UMjNodeComponent* Joint = Scene.Node(TEXT("hinge"));
	if (!TestNotNull(TEXT("joint"), Joint))
	{
		return false;
	}
	TestFalse(TEXT("an element is unbound before any compile"), Joint->GetBoundId().IsSet());
	TestEqual(TEXT("and reads zero"), UMjJointRuntime::GetPosition(Joint), 0.0f);

	if (!Scene.Install())
	{
		AddError(FString::Printf(TEXT("install failed: %s"), *Scene.LastError));
		return false;
	}
	TestTrue(TEXT("the install bound it"), Joint->GetBoundId().IsSet());

	UMjJointRuntime::SetPosition(Joint, 0.25f);
	TestEqual(TEXT("staged position"), UMjJointRuntime::GetPosition(Joint), 0.25f);

	// A second install is a second model, and the ids are re-issued rather than
	// carried: an element still holding the previous compile's id would be
	// indexing freed memory. The state is not re-issued with them -- it is
	// migrated onto the new addresses by element identity -- so the joint reads
	// back the pose it was left at, against a model that never saw it set.
	// URLab.Doc.RecompileMigratesStateByElement is where that rule is asserted.
	if (!Scene.Install())
	{
		AddError(FString::Printf(TEXT("re-install failed: %s"), *Scene.LastError));
		return false;
	}
	TestTrue(TEXT("the element is bound again"), Joint->GetBoundId().IsSet());
	TestEqual(TEXT("and its pose migrated onto the fresh model"),
		UMjJointRuntime::GetPosition(Joint), 0.25f);
	return true;
}

#endif  // URLAB_MJ_GEN && WITH_EDITOR
