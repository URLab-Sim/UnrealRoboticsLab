// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#include "Misc/AutomationTest.h"

#include "MuJoCo/Runtime/MjSimulationRuntime.h"

#include <limits>

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
const char* ClockSnapshotDynamicXml = R"(
<mujoco model="clock_snapshot_dynamic">
  <option timestep="0.01" gravity="0 0 -10"/>
  <worldbody>
    <body name="falling" pos="0 0 1">
      <freejoint name="root"/>
      <geom type="sphere" size="0.05" mass="1"/>
    </body>
  </worldbody>
</mujoco>)";

const char* ClockSnapshotSleepingCameraXml = R"(
<mujoco model="clock_snapshot_sleep">
  <option timestep="0.01" gravity="0 0 0"><flag sleep="enable"/></option>
  <worldbody>
    <body name="mover" pos="0 0 1" sleep="init">
      <freejoint name="root"/>
      <geom type="sphere" size="0.05" mass="1"/>
      <camera name="tracking" mode="trackcom" pos="0 -2 1"/>
    </body>
  </worldbody>
</mujoco>)";

const char* ClockSnapshotSlowClockXml = R"(
<mujoco model="clock_snapshot_slow">
  <option timestep="1" gravity="0 0 0"/>
  <worldbody><body><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody>
</mujoco>)";

TArray<uint8> ClockSnapshotBytesOf(const char* Text)
{
	TArray<uint8> Out;
	Out.Append(reinterpret_cast<const uint8*>(Text), FCStringAnsi::Strlen(Text));
	return Out;
}

bool ClockSnapshotPrepare(const char* Xml, FMjOwnedModelCandidate& Out, FString& OutError)
{
	return FMjOwnedModelCandidate::FromBytes(
		EMjModelSourceKind::XmlWithAssets, ClockSnapshotBytesOf(Xml), {}, Out,
		OutError, TEXT("clock snapshot fixture"));
}

bool ClockSnapshotSameObservation(const FMjModelObservation& A, const FMjModelObservation& B)
{
	return A.Identity.Matches(B.Identity) && A.ClockEpoch == B.ClockEpoch
		&& A.StepSequence == B.StepSequence && A.Time == B.Time
		&& A.Qpos == B.Qpos && A.Qvel == B.Qvel && A.Ctrl == B.Ctrl
		&& A.SensorData == B.SensorData;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSimulationClockDeterminismTest,
	"URLab.Runtime.Clock.DeterministicModesDebtAndAuthority",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSimulationClockDeterminismTest::RunTest(const FString&)
{
	double Now = 5.0;
	const auto Clock = FMjSimulationControlClock::ForTest([&Now]() { return Now; });
	FMjSimulationRuntime Runtime(Clock);
	FString Error;
	FMjOwnedModelCandidate Candidate;
	if (!TestTrue(TEXT("clock fixture prepares"),
		ClockSnapshotPrepare(ClockSnapshotDynamicXml, Candidate, Error)))
	{
		AddError(Error);
		return false;
	}
	const FMjSimulationInstallResult Install = Runtime.Install(MoveTemp(Candidate));
	if (!TestTrue(TEXT("clock fixture installs"), Install.bCommitted)) return false;
	const uint64 InitialRevision = Runtime.GetStateRevision();
	const FMjSimulationClockStatus Initial = Runtime.GetClockStatus();
	TestEqual(TEXT("runtime boots inert in stepped mode"), Initial.Mode,
		EMjSimulationClockMode::Stepped);
	TestEqual(TEXT("clock publishes installed generation"), Initial.ModelGeneration,
		Install.Generation);
	TestEqual(TEXT("clock publishes model timestep"), Initial.TimeStepSeconds, 0.01);
	const FMjSimulationAdvanceResult Inert = Runtime.AdvanceFreeRun(1.0);
	TestEqual(TEXT("free-run advance rejects stepped mode"), Inert.Error,
		EMjSimulationClockError::WrongMode);
	TestEqual(TEXT("rejected advance leaves sequence unchanged"), Runtime.GetStepSequence(),
		static_cast<uint64>(0));

	const FMjSimulationClockChangeResult WrongGeneration = Runtime.ConfigureLocalClockMode(
		EMjSimulationClockMode::FreeRun, Install.Generation + 1);
	TestEqual(TEXT("local clock selection requires exact generation"), WrongGeneration.Error,
		EMjSimulationClockError::WrongModelGeneration);
	const FMjSimulationClockChangeResult FreeRun = Runtime.ConfigureLocalClockMode(
		EMjSimulationClockMode::FreeRun, Install.Generation);
	TestTrue(TEXT("local owner explicitly selects free run"), FreeRun.bCommitted);
	const FMjSimulationClockStatus BeforeInvalid = Runtime.GetClockStatus();
	TestEqual(TEXT("negative elapsed is rejected"), Runtime.AdvanceFreeRun(-0.1).Error,
		EMjSimulationClockError::InvalidElapsed);
	TestEqual(TEXT("nonfinite elapsed is rejected"),
		Runtime.AdvanceFreeRun(std::numeric_limits<double>::quiet_NaN()).Error,
		EMjSimulationClockError::InvalidElapsed);
	TestEqual(TEXT("failed elapsed writes preserve debt"),
		Runtime.GetClockStatus().AccumulatedDebtSeconds,
		BeforeInvalid.AccumulatedDebtSeconds);

	const FMjSimulationAdvanceResult First = Runtime.AdvanceFreeRun(0.025);
	TestTrue(TEXT("elapsed time advances free run"), First.IsOk());
	TestEqual(TEXT("whole model timesteps integrate"), First.IntegratedStepCount,
		static_cast<uint32>(2));
	TestTrue(TEXT("fractional time is retained"),
		FMath::IsNearlyEqual(First.Status.AccumulatedDebtSeconds, 0.005, 1.0e-12));
	TestEqual(TEXT("one bounded advance is one state revision"), Runtime.GetStateRevision(),
		InitialRevision + 1);
	const FMjSimulationAdvanceResult Bounded = Runtime.AdvanceFreeRun(1.0);
	TestEqual(TEXT("catch-up work is bounded"), Bounded.IntegratedStepCount,
		FMjSimulationRuntime::MaxFreeRunStepsPerAdvance);
	TestTrue(TEXT("retained debt is bounded"),
		Bounded.Status.AccumulatedDebtSeconds <=
			FMjSimulationRuntime::MaxFreeRunRetainedDebtSeconds);
	TestTrue(TEXT("excess catch-up time is reported"), Bounded.DroppedTimeSeconds > 0.0);
	TestTrue(TEXT("dropped native steps are reported"), Bounded.DroppedStepCount > 0);
	TestEqual(TEXT("inner steps still consume one revision"), Runtime.GetStateRevision(),
		InitialRevision + 2);
	TestFalse(TEXT("native explicit step cannot race free run"), Runtime.Step(1, Error));

	FMjSimulationControlHandle Forged;
	Forged.SessionId = TEXT("forged");
	Forged.ClaimToken = TEXT("forged");
	Forged.ModelGeneration = Install.Generation;
	const FMjSimulationClockChangeResult Unauthorized = Runtime.AuthorizedSetClockMode(
		Forged, EMjSimulationClockMode::Stepped);
	TestFalse(TEXT("remote clock mutation requires authority"), Unauthorized.IsOk());
	TestEqual(TEXT("failed authority check preserves mode"), Runtime.GetClockStatus().Mode,
		EMjSimulationClockMode::FreeRun);
	auto Session = FMjSimulationControlSessionLifetime::CreateRemote(
		TEXT("clock-owner"), Clock, Error);
	FMjSimulationControlClaimRequest ClaimRequest;
	ClaimRequest.Session = Session;
	ClaimRequest.ExpectedModelGeneration = Install.Generation;
	const FMjSimulationControlClaimResult Claim = Runtime.ClaimControl(ClaimRequest);
	if (!TestTrue(TEXT("remote source claims"), Claim.IsOk())) return false;
	TestEqual(TEXT("claim does not select a clock"), Runtime.GetClockStatus().Mode,
		EMjSimulationClockMode::FreeRun);
	TestEqual(TEXT("authorized explicit step reports the clock conflict"),
		Runtime.AuthorizedStep(Claim.Handle, 1).Error,
		EMjSimulationControlError::WrongClockMode);
	const FMjSimulationClockChangeResult Stepped = Runtime.AuthorizedSetClockMode(
		Claim.Handle, EMjSimulationClockMode::Stepped);
	TestTrue(TEXT("authority can select stepped mode"), Stepped.bCommitted);
	TestEqual(TEXT("mode transition clears debt"), Stepped.Status.AccumulatedDebtSeconds, 0.0);
	TestTrue(TEXT("release succeeds"), Runtime.ReleaseControl(Claim.Handle).bChanged);
	TestEqual(TEXT("release does not select a clock"), Runtime.GetClockStatus().Mode,
		EMjSimulationClockMode::Stepped);

	Runtime.ConfigureLocalClockMode(EMjSimulationClockMode::FreeRun, Install.Generation);
	Runtime.AdvanceFreeRun(0.005);
	TestTrue(TEXT("reset debt fixture"), Runtime.GetClockStatus().AccumulatedDebtSeconds > 0.0);
	TestTrue(TEXT("native reset succeeds in free run"), Runtime.Reset(Error));
	TestEqual(TEXT("reset clears debt"), Runtime.GetClockStatus().AccumulatedDebtSeconds, 0.0);
	Runtime.AdvanceFreeRun(0.005);
	FMjModelState QposState;
	TestEqual(TEXT("capture state for debt reset"), Runtime.CaptureState(mjSTATE_QPOS, QposState),
		EMjModelStateError::None);
	TestEqual(TEXT("state apply succeeds in free run"), Runtime.ApplyState(QposState),
		EMjModelStateError::None);
	TestEqual(TEXT("state apply clears debt"), Runtime.GetClockStatus().AccumulatedDebtSeconds, 0.0);
	Runtime.AdvanceFreeRun(0.005);
	FMjOwnedModelCandidate Replacement;
	if (!TestTrue(TEXT("replacement fixture prepares"),
		ClockSnapshotPrepare(ClockSnapshotDynamicXml, Replacement, Error)))
		return false;
	const FMjSimulationInstallResult Reload = Runtime.Install(MoveTemp(Replacement));
	TestTrue(TEXT("replacement installs"), Reload.bCommitted);
	TestEqual(TEXT("replacement clears debt"), Runtime.GetClockStatus().AccumulatedDebtSeconds, 0.0);
	TestEqual(TEXT("replacement preserves explicit clock selection"), Runtime.GetClockStatus().Mode,
		EMjSimulationClockMode::FreeRun);

	FMjSimulationRuntime SlowRuntime;
	FMjOwnedModelCandidate SlowCandidate;
	if (!TestTrue(TEXT("slow-timestep fixture prepares"),
		ClockSnapshotPrepare(ClockSnapshotSlowClockXml, SlowCandidate, Error)))
		return false;
	const FMjSimulationInstallResult SlowInstall = SlowRuntime.Install(MoveTemp(SlowCandidate));
	if (!TestTrue(TEXT("slow-timestep fixture installs"), SlowInstall.bCommitted)) return false;
	SlowRuntime.ConfigureLocalClockMode(EMjSimulationClockMode::FreeRun, SlowInstall.Generation);
	TestEqual(TEXT("effective debt cap retains one partial timestep"),
		SlowRuntime.GetClockStatus().MaxRetainedDebtSeconds, 1.0);
	for (int32 Index = 0; Index < 8; ++Index)
	{
		SlowRuntime.AdvanceFreeRun(0.125);
	}
	TestEqual(TEXT("sub-timestep elapsed eventually integrates a slow model"),
		SlowRuntime.GetStepSequence(), static_cast<uint64>(1));
	const FMjSimulationAdvanceResult FirstFractionalDrop = SlowRuntime.AdvanceFreeRun(65.5);
	TestEqual(TEXT("first fractional drop is below one cumulative step"),
		FirstFractionalDrop.DroppedStepCount, static_cast<uint64>(0));
	const FMjSimulationAdvanceResult SecondFractionalDrop = SlowRuntime.AdvanceFreeRun(64.5);
	TestEqual(TEXT("fractional drops accumulate into a reported step"),
		SecondFractionalDrop.DroppedStepCount, static_cast<uint64>(1));
	TestEqual(TEXT("cumulative dropped count derives from total dropped time"),
		SecondFractionalDrop.Status.DroppedStepCount, static_cast<uint64>(1));
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSimulationCartesianSnapshotTest,
	"URLab.Runtime.Snapshot.PostStepSleepingCameraAndLiveIsolation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSimulationCartesianSnapshotTest::RunTest(const FString&)
{
	FString Error;
	FMjSimulationRuntime Dynamic;
	FMjOwnedModelCandidate DynamicCandidate;
	if (!TestTrue(TEXT("dynamic fixture prepares"),
		ClockSnapshotPrepare(ClockSnapshotDynamicXml, DynamicCandidate, Error))) return false;
	const FMjSimulationInstallResult DynamicInstall = Dynamic.Install(MoveTemp(DynamicCandidate));
	if (!TestTrue(TEXT("dynamic fixture installs"), DynamicInstall.bCommitted)) return false;
	TestTrue(TEXT("dynamic fixture steps"), Dynamic.Step(1, Error));
	FMjModelObservation PostStep;
	Dynamic.GetObservation(PostStep);
	FMjModelObservation BeforeSnapshot = PostStep;
	FMjSimulationCartesianSnapshot DynamicSnapshot;
	TestEqual(TEXT("post-step Cartesian snapshot succeeds"),
		Dynamic.CaptureCartesianSnapshot(DynamicSnapshot), EMjSimulationSnapshotError::None);
	TestEqual(TEXT("snapshot owns runtime generation"),
		DynamicSnapshot.Identity.Model.Generation, DynamicInstall.Generation);
	TestEqual(TEXT("snapshot owns step sequence"), DynamicSnapshot.Identity.StepSequence,
		static_cast<uint64>(1));
	TestEqual(TEXT("snapshot owns state revision"), DynamicSnapshot.Identity.StateRevision,
		Dynamic.GetStateRevision());
	TestTrue(TEXT("scratch kinematics matches post-integration qpos"),
		DynamicSnapshot.BodyPositions.Num() >= 6 && PostStep.Qpos.Num() >= 3
		&& FMath::IsNearlyEqual(DynamicSnapshot.BodyPositions[5], PostStep.Qpos[2], 1.0e-12));
	FMjModelObservation AfterSnapshot;
	Dynamic.GetObservation(AfterSnapshot);
	TestTrue(TEXT("snapshot extraction leaves live state unchanged"),
		ClockSnapshotSameObservation(BeforeSnapshot, AfterSnapshot));

	FMjSimulationRuntime Sleeping;
	FMjOwnedModelCandidate SleepingCandidate;
	if (!TestTrue(TEXT("sleeping camera fixture prepares"),
		ClockSnapshotPrepare(ClockSnapshotSleepingCameraXml, SleepingCandidate, Error)))
	{
		AddError(Error);
		return false;
	}
	const FMjSimulationInstallResult SleepingInstall = Sleeping.Install(MoveTemp(SleepingCandidate));
	if (!TestTrue(TEXT("sleeping camera fixture installs"), SleepingInstall.bCommitted)) return false;
	FMjSimulationCartesianSnapshot Initial;
	if (!TestEqual(TEXT("initial sleeping snapshot"), Sleeping.CaptureCartesianSnapshot(Initial),
		EMjSimulationSnapshotError::None)) return false;
	FMjModelState Qpos;
	if (!TestEqual(TEXT("capture qpos for sleeping restore"),
		Sleeping.CaptureState(mjSTATE_QPOS, Qpos), EMjModelStateError::None)) return false;
	Qpos.Values[0] += 2.0;
	const uint64 BeforeApplyRevision = Sleeping.GetStateRevision();
	TestEqual(TEXT("apply qpos without forwarding"), Sleeping.ApplyState(Qpos),
		EMjModelStateError::None);
	FMjModelObservation AppliedObservation;
	Sleeping.GetObservation(AppliedObservation);
	FMjSimulationCartesianSnapshot Moved;
	TestEqual(TEXT("scratch wakes moved sleeping kinematics"),
		Sleeping.CaptureCartesianSnapshot(Moved), EMjSimulationSnapshotError::None);
	TestEqual(TEXT("state restore advances revision"), Moved.Identity.StateRevision,
		BeforeApplyRevision + 1);
	TestTrue(TEXT("sleeping body snapshot follows supplied qpos"),
		Moved.BodyPositions.Num() >= 6
		&& FMath::IsNearlyEqual(Moved.BodyPositions[3] - Initial.BodyPositions[3], 2.0, 1.0e-12));
	TestTrue(TEXT("track-COM camera follows moved sleeping body"),
		Moved.CameraPositions.Num() >= 3
		&& FMath::IsNearlyEqual(Moved.CameraPositions[0] - Initial.CameraPositions[0], 2.0, 1.0e-12));
	FMjModelObservation AfterSleepingSnapshot;
	Sleeping.GetObservation(AfterSleepingSnapshot);
	TestTrue(TEXT("sleeping scratch computation leaves restored live state unchanged"),
		ClockSnapshotSameObservation(AppliedObservation, AfterSleepingSnapshot));
	return !HasAnyErrors();
}

#endif
