// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// The element runtime libraries: what they classify and what they correct.
//
// Neither half needs a running simulation. Classification is a property of the
// element's class, and the MuJoCo -> Unreal correction is a pure function of a
// value and its kind, so both are testable without an engine -- which is the
// point of having put them in libraries rather than on components.
//
// The totality test is the one that earns its keep. The table it guards
// replaces a hand-maintained one that had drifted from the sensor list it was
// supposed to cover; keying it on the schema means a sensor kind the grammar
// gains shows up here as a failure rather than as a sensor that silently reads
// as an untransformed scalar.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Elements/MjJointRuntime.h"
#include "MuJoCo/Elements/MjSensorRuntime.h"
#include "MuJoCo/Gen/MjDispatch.gen.h"
#include "MuJoCo/Gen/MjElements.gen.h"

namespace MjElementRuntimeTests
{
using namespace urlab::spec;

/** Every class the schema lets `<sensor>` hold, as its class-default object. */
TArray<const UMjNodeComponent*> SensorLeafDefaults()
{
	TArray<const UMjNodeComponent*> Out;
	gen::ChildSlots(static_cast<const UMjSensor*>(nullptr),
		[&Out](int32, auto Tag) {
			using Leaf = typename decltype(Tag)::type;
			Out.Add(GetDefault<Leaf>());
		});
	return Out;
}

} // namespace MjElementRuntimeTests

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSensorDescriptorTotalityTest,
	"URLab.Elements.SensorDescriptorCoversEverySensorKind",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSensorDescriptorTotalityTest::RunTest(const FString& Parameters)
{
	const TArray<const UMjNodeComponent*> Leaves = MjElementRuntimeTests::SensorLeafDefaults();
	TestTrue(TEXT("the schema declares sensor kinds at all"), Leaves.Num() > 0);

	for (const UMjNodeComponent* Leaf : Leaves)
	{
		if (!TestNotNull(TEXT("sensor leaf CDO"), Leaf))
		{
			continue;
		}
		TestTrue(FString::Printf(TEXT("%s is classified as a sensor"), *Leaf->GetClass()->GetName()),
			UMjSensorRuntime::IsSensor(Leaf));
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjElementClassificationFailsClosedTest,
	"URLab.Elements.RuntimeClassificationFailsClosed",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjElementClassificationFailsClosedTest::RunTest(const FString& Parameters)
{
	// The libraries take the shared base, so the type system cannot stop a
	// caller passing a body to the sensor library. Every entry point has to
	// refuse it rather than index sensordata with a body id.
	const UMjNodeComponent* Body = GetDefault<UMjBodyBase>();
	const UMjNodeComponent* Geom = GetDefault<UMjGeomBase>();
	const UMjNodeComponent* Joint = GetDefault<UMjJoint>();
	const UMjNodeComponent* FreeJoint = GetDefault<UMjFreeJoint>();
	const UMjNodeComponent* Gyro = GetDefault<UMjGyro>();
	const UMjNodeComponent* Spatial = GetDefault<UMjSpatial>();
	const UMjNodeComponent* Fixed = GetDefault<UMjFixed>();

	TestFalse(TEXT("a body is not a sensor"), UMjSensorRuntime::IsSensor(Body));
	TestFalse(TEXT("a geom is not a sensor"), UMjSensorRuntime::IsSensor(Geom));
	TestFalse(TEXT("a joint is not a sensor"), UMjSensorRuntime::IsSensor(Joint));
	TestTrue(TEXT("a gyro is a sensor"), UMjSensorRuntime::IsSensor(Gyro));

	TestTrue(TEXT("a joint is a joint"), UMjJointRuntime::IsJoint(Joint));
	TestTrue(TEXT("a freejoint is a joint"), UMjJointRuntime::IsJoint(FreeJoint));
	TestFalse(TEXT("a gyro is not a joint"), UMjJointRuntime::IsJoint(Gyro));
	TestFalse(TEXT("a body is not a joint"), UMjJointRuntime::IsJoint(Body));

	TestTrue(TEXT("a spatial tendon is a tendon"), UMjTendonRuntime::IsTendon(Spatial));
	TestTrue(TEXT("a fixed tendon is a tendon"), UMjTendonRuntime::IsTendon(Fixed));
	TestFalse(TEXT("a joint is not a tendon"), UMjTendonRuntime::IsTendon(Joint));

	// Null is the case a Blueprint reaches with an unset pin.
	TestFalse(TEXT("null is not a sensor"), UMjSensorRuntime::IsSensor(nullptr));
	TestFalse(TEXT("null is not a joint"), UMjJointRuntime::IsJoint(nullptr));
	TestFalse(TEXT("null is not a tendon"), UMjTendonRuntime::IsTendon(nullptr));

	// Unbound and engineless, every read is the empty answer rather than a crash.
	TestEqual(TEXT("unbound sensor has no dimension"), UMjSensorRuntime::GetDimension(Gyro), 0);
	TestEqual(TEXT("unbound sensor reads empty"), UMjSensorRuntime::GetReading(Gyro).Num(), 0);
	TestEqual(TEXT("unbound joint reads zero"), UMjJointRuntime::GetPosition(Joint), 0.0f);
	TestEqual(TEXT("unbound tendon reads zero"), UMjTendonRuntime::GetLength(Spatial), 0.0f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSensorReadingTransformTest,
	"URLab.Elements.SensorReadingTransform",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSensorReadingTransformTest::RunTest(const FString& Parameters)
{
	// A position is metres in MuJoCo's right-handed frame and centimetres in
	// Unreal's left-handed one, so it scales and flips Y.
	{
		TArray<float> Values = {1.0f, 2.0f, 3.0f};
		MjTransformSensorReading(Values, EMjSensorValueKind::Position);
		TestEqual(TEXT("position X scales"), Values[0], 100.0f);
		TestEqual(TEXT("position Y scales and flips"), Values[1], -200.0f);
		TestEqual(TEXT("position Z scales"), Values[2], 300.0f);
	}

	// A direction is already unit length, so it flips without scaling --
	// scaling it would be the bug this case exists to rule out.
	{
		TArray<float> Values = {0.0f, 1.0f, 0.0f};
		MjTransformSensorReading(Values, EMjSensorValueKind::Direction);
		TestEqual(TEXT("direction X unchanged"), Values[0], 0.0f);
		TestEqual(TEXT("direction Y flips only"), Values[1], -1.0f);
		TestEqual(TEXT("direction Z unchanged"), Values[2], 0.0f);
	}

	{
		TArray<float> Values = {1.0f, 2.0f, 3.0f};
		MjTransformSensorReading(Values, EMjSensorValueKind::Vector3);
		TestEqual(TEXT("vector X unchanged"), Values[0], 1.0f);
		TestEqual(TEXT("vector Y flips"), Values[1], -2.0f);
		TestEqual(TEXT("vector Z unchanged"), Values[2], 3.0f);
	}

	// MuJoCo writes (w, x, y, z); FQuat is constructed from (X, Y, Z, W). The
	// reorder alone compiles and is wrong, so the signs are asserted too.
	{
		TArray<float> Values = {0.1f, 0.2f, 0.3f, 0.4f};
		MjTransformSensorReading(Values, EMjSensorValueKind::Quaternion);
		TestEqual(TEXT("quat X is -mjX"), Values[0], -0.2f);
		TestEqual(TEXT("quat Y is mjY"), Values[1], 0.3f);
		TestEqual(TEXT("quat Z is -mjZ"), Values[2], -0.4f);
		TestEqual(TEXT("quat W is mjW"), Values[3], 0.1f);
	}

	// Two positions end to end, each corrected in its own right.
	{
		TArray<float> Values = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
		MjTransformSensorReading(Values, EMjSensorValueKind::GeomFromTo);
		TestEqual(TEXT("fromto from X"), Values[0], 100.0f);
		TestEqual(TEXT("fromto from Y"), Values[1], -200.0f);
		TestEqual(TEXT("fromto from Z"), Values[2], 300.0f);
		TestEqual(TEXT("fromto to X"), Values[3], 400.0f);
		TestEqual(TEXT("fromto to Y"), Values[4], -500.0f);
		TestEqual(TEXT("fromto to Z"), Values[5], 600.0f);
	}

	{
		TArray<float> Values = {42.0f};
		MjTransformSensorReading(Values, EMjSensorValueKind::Scalar);
		TestEqual(TEXT("scalar is untouched"), Values[0], 42.0f);
	}

	// A reading shorter than its kind expects is what a variable-width sensor
	// produces before its model exists; it must pass through, not index past.
	{
		TArray<float> Short = {1.0f};
		MjTransformSensorReading(Short, EMjSensorValueKind::Quaternion);
		TestEqual(TEXT("short reading survives"), Short.Num(), 1);
		TestEqual(TEXT("short reading unchanged"), Short[0], 1.0f);

		TArray<float> Empty;
		MjTransformSensorReading(Empty, EMjSensorValueKind::Position);
		TestEqual(TEXT("empty reading survives"), Empty.Num(), 0);
	}
	return true;
}

#endif // URLAB_MJ_GEN && WITH_EDITOR
