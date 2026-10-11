// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "MuJoCo/Core/MjModelRuntime.h"
#include "mujoco/mujoco.h"
#include <limits>

namespace
{
constexpr const char* ModelXml = R"(
<mujoco model="runtime">
  <worldbody>
    <body name="base"><freejoint name="root"/><geom type="sphere" size=".1"/>
      <body name="arm"><joint name="hinge" type="hinge" range="-1 1"/><geom type="capsule" size=".03 .2"/>
        <site name="tip" pos="0 0 .2"/><camera name="eye"/>
      </body>
    </body>
  </worldbody>
  <actuator><motor name="drive" joint="hinge" ctrllimited="true" ctrlrange="-2 2"/></actuator>
  <sensor><framepos name="tip_pos" objtype="site" objname="tip"/></sensor>
</mujoco>)";

constexpr const char* NoControlXml = R"(
<mujoco model="empty-control"><worldbody><body><joint name="j"/><geom type="sphere" size=".1"/></body></worldbody></mujoco>)";

constexpr const char* MultiInputActuatorXml = R"(
<mujoco><compiler autolimits="true"/>
<default>
  <default class="exp"><position ctrlrange="-2 2" ctrllimited="true"/></default>
  <default class="quat"><position ctrlrange="-1 1" ctrllimited="true"/></default>
</default>
<worldbody>
  <body><joint name="exp" type="ball"/><geom type="sphere" size=".1"/></body>
  <body><joint name="quat" type="ball"/><geom type="sphere" size=".1"/></body>
</worldbody>
<actuator>
  <orientation name="exp" joint="exp" kp="1" class="exp"/>
  <orientation name="quat" joint="quat" input="quat" kp="1" class="quat"/>
</actuator></mujoco>)";

mjModel* MakeModelFrom(FAutomationTestBase& Test, const char* Xml)
{
	char Error[1024]{};
	mjSpec* Spec = mj_parseXMLString(Xml, nullptr, Error, sizeof(Error));
	if (!Test.TestNotNull(TEXT("fixture parses"), Spec))
		return nullptr;
	mjModel* Model = mj_compile(Spec, nullptr);
	mj_deleteSpec(Spec);
	Test.TestNotNull(TEXT("fixture compiles"), Model);
	return Model;
}

mjModel* MakeModel(FAutomationTestBase& Test)
{
	return MakeModelFrom(Test, ModelXml);
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjModelRuntimeMultiInputActuatorTest,
	"URLab.Physics.ModelRuntime.MultiInputActuatorControlSpan",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FMjModelRuntimeMultiInputActuatorTest::RunTest(const FString&)
{
	mjModel* Model = MakeModelFrom(*this, MultiInputActuatorXml);
	if (!Model) return false;
	TestEqual(TEXT("two native actuators"), Model->nactuator, static_cast<mjtSize>(2));
	TestEqual(TEXT("three plus four scalar controls"), Model->nu, static_cast<mjtSize>(7));
	FMjModelDescriptor Descriptor;
	FString Error;
	if (!TestTrue(TEXT("multi-input descriptor builds"), MjModelRuntime::BuildDescriptor(Model, 9, Descriptor, Error)))
	{
		AddError(Error); mj_deleteModel(Model); return false;
	}
	if (!TestEqual(TEXT("two actuator descriptors"), Descriptor.Actuators.Num(), 2))
	{
		mj_deleteModel(Model); return false;
	}
	int32 CoveredControls = 0;
	for (int32 ActuatorId = 0; ActuatorId < Descriptor.Actuators.Num(); ++ActuatorId)
	{
		const FMjActuatorDescriptor& Actuator = Descriptor.Actuators[ActuatorId];
		TestEqual(FString::Printf(TEXT("native address[%d]"), ActuatorId), Actuator.ControlAddress,
			Model->actuator_ctrladr[ActuatorId]);
		TestEqual(FString::Printf(TEXT("native width[%d]"), ActuatorId), Actuator.ControlWidth,
			Model->actuator_ctrlnum[ActuatorId]);
		TestEqual(FString::Printf(TEXT("slot count[%d]"), ActuatorId), Actuator.Controls.Num(),
			Actuator.ControlWidth);
		for (int32 Offset = 0; Offset < Actuator.Controls.Num(); ++Offset)
		{
			const int32 ControlIndex = Actuator.ControlAddress + Offset;
			TestEqual(FString::Printf(TEXT("limited[%d,%d]"), ActuatorId, Offset), Actuator.Controls[Offset].bLimited,
				Model->actuator_ctrllimited[ControlIndex] != 0);
			TestEqual(FString::Printf(TEXT("min[%d,%d]"), ActuatorId, Offset), Actuator.Controls[Offset].RangeMin,
				Model->actuator_ctrlrange[2 * ControlIndex]);
			TestEqual(FString::Printf(TEXT("max[%d,%d]"), ActuatorId, Offset), Actuator.Controls[Offset].RangeMax,
				Model->actuator_ctrlrange[2 * ControlIndex + 1]);
			++CoveredControls;
		}
	}
	TestEqual(TEXT("all scalar controls covered"), CoveredControls, static_cast<int32>(Model->nu));
	TestTrue(TEXT("later actuator id differs from scalar address"),
		Descriptor.Actuators[1].Id != Descriptor.Actuators[1].ControlAddress);
	TestNotEqual(TEXT("actuator limit groups differ"), Descriptor.Actuators[0].Controls[0].RangeMin,
		Descriptor.Actuators[1].Controls[0].RangeMin);
	mj_deleteModel(Model);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjModelRuntimeDescriptorTest,
	"URLab.Physics.ModelRuntime.DescriptorAndObservation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjModelRuntimeDescriptorTest::RunTest(const FString& Parameters)
{
	mjModel* Model = MakeModel(*this);
	if (!Model)
		return false;
	mjData* Data = mj_makeData(Model);
	TestNotNull(TEXT("data created"), Data);
	if (!Data)
	{
		mj_deleteModel(Model);
		return false;
	}

	FMjModelDescriptor Descriptor;
	FString Error;
	TestTrue(TEXT("descriptor builds"), MjModelRuntime::BuildDescriptor(Model, 7, Descriptor, Error));
	TestEqual(TEXT("generation retained"), Descriptor.Identity.Generation, static_cast<uint64>(7));
	TestEqual(TEXT("model hash is SHA-256 hex"), Descriptor.Identity.ContentSha256.Len(), 64);
	TestEqual(TEXT("free and hinge joints described"), Descriptor.Joints.Num(), 2);
	TestEqual(TEXT("free qpos width"), Descriptor.Joints[0].QposWidth, 7);
	TestEqual(TEXT("free dof width"), Descriptor.Joints[0].DofWidth, 6);
	TestEqual(TEXT("actuator name"), Descriptor.Actuators[0].Name, FString(TEXT("drive")));
	TestEqual(TEXT("sensor dimension"), Descriptor.Sensors[0].Dimension, 3);
	TestEqual(TEXT("camera name"), Descriptor.Cameras[0].Name, FString(TEXT("eye")));

	Data->qpos[7] = 0.25;
	mj_forward(Model, Data);
	FMjModelObservation Observation;
	TestTrue(TEXT("observation captured"), MjModelRuntime::CaptureObservation(Model, Data,
		Descriptor.Identity, 3, 11, Observation));
	TestEqual(TEXT("observation epoch"), Observation.ClockEpoch, static_cast<uint64>(3));
	TestEqual(TEXT("observation step"), Observation.StepSequence, static_cast<uint64>(11));
	TestEqual(TEXT("qpos exact dimension"), Observation.Qpos.Num(), Model->nq);
	TestEqual(TEXT("sensor exact dimension"), Observation.SensorData.Num(), Model->nsensordata);

	mj_deleteData(Data);
	mj_deleteModel(Model);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjModelRuntimeStateTest,
	"URLab.Physics.ModelRuntime.ValidatedState",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjModelRuntimeStateTest::RunTest(const FString& Parameters)
{
	mjModel* Model = MakeModel(*this);
	if (!Model)
		return false;
	mjData* Data = mj_makeData(Model);
	FMjModelDescriptor Descriptor;
	FString Error;
	MjModelRuntime::BuildDescriptor(Model, 1, Descriptor, Error);
	Data->time = 1.25;
	Data->qpos[7] = 0.4;

	FMjModelState Checkpoint;
	TestTrue(TEXT("checkpoint captured"), MjModelRuntime::CaptureState(Model, Data,
		Descriptor.Identity, 2, mjSTATE_INTEGRATION, Checkpoint) == EMjModelStateError::None);
	TestTrue(TEXT("only integration mask is complete"), Checkpoint.bCompleteCheckpoint);
	Data->time = 9.0;
	TestTrue(TEXT("checkpoint applies"), MjModelRuntime::ApplyState(Model, Data,
		Descriptor.Identity, Checkpoint) == EMjModelStateError::None);
	TestEqual(TEXT("apply restored time without forward/step"), Data->time, 1.25);
	Data->xpos[3] = 123.0;
	TestTrue(TEXT("second checkpoint apply succeeds"), MjModelRuntime::ApplyState(Model, Data,
		Descriptor.Identity, Checkpoint) == EMjModelStateError::None);
	TestEqual(TEXT("apply does not secretly refresh derived Cartesian state"),
		static_cast<double>(Data->xpos[3]), 123.0);

	FMjModelState Bad = Checkpoint;
	Bad.Identity.Generation++;
	const double TimeBefore = Data->time;
	TestTrue(TEXT("stale generation rejected"), MjModelRuntime::ApplyState(Model, Data,
		Descriptor.Identity, Bad) == EMjModelStateError::WrongModel);
	TestEqual(TEXT("rejection leaves data untouched"), Data->time, TimeBefore);
	Bad = Checkpoint;
	Bad.Values.Pop();
	TestTrue(TEXT("wrong length rejected"), MjModelRuntime::ValidateState(Model,
		Descriptor.Identity, Bad) == EMjModelStateError::WrongLength);
	Bad = Checkpoint;
	Bad.Values[0] = std::numeric_limits<double>::quiet_NaN();
	TestTrue(TEXT("non-finite rejected"), MjModelRuntime::ValidateState(Model,
		Descriptor.Identity, Bad) == EMjModelStateError::NonFinite);
	FMjModelObservation UnchangedObservation;
	UnchangedObservation.ClockEpoch = 77;
	Data->qpos[0] = std::numeric_limits<double>::quiet_NaN();
	TestFalse(TEXT("non-finite observation rejected"), MjModelRuntime::CaptureObservation(
		Model, Data, Descriptor.Identity, 2, 0, UnchangedObservation));
	TestEqual(TEXT("failed observation leaves output untouched"),
		UnchangedObservation.ClockEpoch, static_cast<uint64>(77));
	Data->qpos[0] = 1.0;
	FMjModelState Untouched;
	Untouched.ClockEpoch = 99;
	TestTrue(TEXT("invalid mask rejected"), MjModelRuntime::CaptureState(Model, Data,
		Descriptor.Identity, 2, 0, Untouched) == EMjModelStateError::InvalidMask);
	TestEqual(TEXT("failed capture leaves output untouched"), Untouched.ClockEpoch,
		static_cast<uint64>(99));
	FMjModelIdentity WrongDimensions = Descriptor.Identity;
	--WrongDimensions.Nq;
	TestTrue(TEXT("identity dimensions must match model"), MjModelRuntime::CaptureState(
		Model, Data, WrongDimensions, 2, mjSTATE_TIME, Untouched)
		== EMjModelStateError::WrongModel);

	mjModel* NoControl = MakeModelFrom(*this, NoControlXml);
	mjData* NoControlData = NoControl ? mj_makeData(NoControl) : nullptr;
	if (NoControl && NoControlData)
	{
		FMjModelDescriptor NoControlDescriptor;
		MjModelRuntime::BuildDescriptor(NoControl, 2, NoControlDescriptor, Error);
		FMjModelState EmptyControl;
		TestTrue(TEXT("zero-width ctrl state is valid"), MjModelRuntime::CaptureState(
			NoControl, NoControlData, NoControlDescriptor.Identity, 0, mjSTATE_CTRL,
			EmptyControl) == EMjModelStateError::None);
		TestEqual(TEXT("zero-width state owns an empty vector"), EmptyControl.Values.Num(), 0);
		TestTrue(TEXT("empty state applies"), MjModelRuntime::ApplyState(NoControl,
			NoControlData, NoControlDescriptor.Identity, EmptyControl) == EMjModelStateError::None);

		TestTrue(TEXT("mismatched model/data signatures reject capture"),
			MjModelRuntime::CaptureState(Model, NoControlData, Descriptor.Identity, 0,
				mjSTATE_TIME, Untouched) == EMjModelStateError::WrongData);
	}
	else
	{
		AddError(TEXT("zero-control fixture failed"));
	}
	if (NoControlData)
		mj_deleteData(NoControlData);
	if (NoControl)
		mj_deleteModel(NoControl);

	mj_deleteData(Data);
	mj_deleteModel(Model);
	return true;
}
