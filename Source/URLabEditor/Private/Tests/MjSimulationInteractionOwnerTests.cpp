#include "Misc/AutomationTest.h"
#include "MuJoCo/Core/MjOwnedModel.h"
#include "MuJoCo/Runtime/MjSimulationInteractionOwner.h"
#include "MuJoCo/Runtime/MjSimulationRuntime.h"
#include "Transport/PoseStreamProducer.h"

#if WITH_DEV_AUTOMATION_TESTS
namespace URLabSimulationInteractionTest
{
const char* Xml = R"(
<mujoco model="interaction_owner">
  <option timestep="0.01" gravity="0 0 0"/>
  <worldbody>
    <body name="slider"><joint name="slide" type="slide" axis="1 0 0"/>
      <geom type="sphere" size="0.05" mass="1"/>
    </body>
  </worldbody>
</mujoco>)";

struct FSink final : IURLabInteractionCompletionSink
{
	TMap<FString, FURLabInteractionResult> Results;
	virtual void CompleteInteraction(const FString& Ticket,
		const FURLabInteractionResult& Result) override
	{
		Results.Add(Ticket, Result);
	}
};

FURLabPoseStreamDescriptor Descriptor(const FString& Revision, uint64 Generation = 1)
{
	FURLabPoseStreamDescriptor Value;
	Value.ProducerBootId = TEXT("simulation-boot");
	Value.StreamId = TEXT("simulation-poses");
	Value.StreamGeneration = Generation;
	Value.SourceRevision = Revision;
	return Value;
}

FURLabInteractionWrenchRequest Request(const FURLabInteractionGrant& Grant,
	const FString& Ticket)
{
	FURLabInteractionWrenchRequest Value;
	Value.Ticket = Ticket;
	Value.GrantToken = Grant.Token;
	Value.Source = Grant.Source;
	Value.ViewerBootId = Grant.ViewerBootId;
	Value.ClockEpoch = Grant.ClockEpoch;
	Value.BodyIndex = 1;
	Value.ForceNewton = FVector3d(1.0, 0.0, 0.0);
	return Value;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSimulationInteractionOwnerTest,
	"URLab.Runtime.InteractionOwner.OneStepAuthorityRestorationAndReceipt",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSimulationInteractionOwnerTest::RunTest(const FString&)
{
	using namespace URLabSimulationInteractionTest;
	TArray<uint8> Bytes;
	Bytes.Append(reinterpret_cast<const uint8*>(Xml), FCStringAnsi::Strlen(Xml));
	FMjOwnedModelCandidate Candidate;
	FString Error;
	if (!TestTrue(TEXT("fixture prepares"), FMjOwnedModelCandidate::FromBytes(
		EMjModelSourceKind::XmlWithAssets, Bytes, {}, Candidate, Error,
		TEXT("interaction fixture"))))
	{
		AddError(Error);
		return false;
	}
	FMjSimulationRuntime Runtime;
	const FMjSimulationInstallResult Installed = Runtime.Install(MoveTemp(Candidate));
	if (!TestTrue(TEXT("fixture installs"), Installed.bCommitted)) return false;
	auto Owner = MakeShared<FMjSimulationInteractionOwner, ESPMode::ThreadSafe>();
	Runtime.SetInteractionOwner(Owner);
	TestTrue(TEXT("existing pose identity binds"),
		Owner->BindSource(Descriptor(TEXT("scene-r1")), Error));
	auto Session = FMjSimulationControlSessionLifetime::CreateLocal(
		TEXT("interaction-control"), Runtime.GetControlClock(), Error);
	FMjSimulationControlClaimRequest ClaimRequest;
	ClaimRequest.Session = Session;
	ClaimRequest.SourceKind = EMjSimulationControlSourceKind::Local;
	ClaimRequest.ExpectedModelGeneration = Installed.Generation;
	const FMjSimulationControlClaimResult Claim = Runtime.ClaimControl(ClaimRequest);
	if (!TestTrue(TEXT("explicit control claim succeeds"), Claim.IsOk())) return false;
	auto Sink = MakeShared<FSink, ESPMode::ThreadSafe>();
	FURLabInteractionGrant Grant;
	TestTrue(TEXT("current controller issues grant"), Owner->IssueGrant(Runtime,
		Claim.Handle, {TEXT("simulation-boot"), TEXT("simulation-poses"), 1,
			TEXT("scene-r1")}, TEXT("viewer-boot"), 5.0, Grant, Error));
	const auto Wrench = Request(Grant, TEXT("apply-once"));
	TestEqual(TEXT("wrench enqueues"), Owner->Enqueue(Wrench, Sink).Admission,
		EURLabInteractionAdmission::Accepted);
	TestTrue(TEXT("authorized step integrates"), Runtime.AuthorizedStep(Claim.Handle, 1).IsOk());
	TestEqual(TEXT("receipt is post-step applied"), Sink->Results[TEXT("apply-once")].Terminal,
		EURLabInteractionTerminal::Applied);
	TestEqual(TEXT("receipt has actual first frame"),
		Sink->Results[TEXT("apply-once")].AppliedFrame.FrameId, uint64(1));
	FMjModelObservation AfterForce;
	TestTrue(TEXT("post-force observation reads"), Runtime.GetObservation(AfterForce));
	if (!TestTrue(TEXT("slide velocity exists"), AfterForce.Qvel.Num() == 1)) return false;
	const double VelocityAfterForce = AfterForce.Qvel[0];
	TestTrue(TEXT("one-step force changed velocity"), VelocityAfterForce > 0.0);
	TestTrue(TEXT("second authorized step integrates"),
		Runtime.AuthorizedStep(Claim.Handle, 1).IsOk());
	FMjModelObservation AfterRestore;
	TestTrue(TEXT("post-restore observation reads"), Runtime.GetObservation(AfterRestore));
	TestTrue(TEXT("owned wrench was restored after one step"),
		FMath::IsNearlyEqual(AfterRestore.Qvel[0], VelocityAfterForce, 1e-9));

	const auto Stale = Request(Grant, TEXT("stale-after-reset"));
	TestEqual(TEXT("pre-reset request queues"), Owner->Enqueue(Stale, Sink).Admission,
		EURLabInteractionAdmission::Accepted);
	TestTrue(TEXT("authorized reset advances epoch"), Runtime.AuthorizedReset(Claim.Handle).IsOk());
	TestTrue(TEXT("step after reset integrates"), Runtime.AuthorizedStep(Claim.Handle, 1).IsOk());
	TestEqual(TEXT("epoch change rejects queued interaction"),
		Sink->Results[TEXT("stale-after-reset")].Terminal,
		EURLabInteractionTerminal::RejectedSource);

	FURLabInteractionGrant ViewerA;
	FURLabInteractionGrant ViewerB;
	TestTrue(TEXT("first viewer grant issues independently"), Owner->IssueGrant(Runtime,
		Claim.Handle, Grant.Source, TEXT("viewer-a"), 5.0, ViewerA, Error));
	TestTrue(TEXT("second viewer grant does not replace first"), Owner->IssueGrant(Runtime,
		Claim.Handle, Grant.Source, TEXT("viewer-b"), 5.0, ViewerB, Error));
	TestTrue(TEXT("exact first-viewer revoke succeeds"), Owner->RevokeGrant(Runtime,
		Claim.Handle, Grant.Source, ViewerA.Token, Error));
	TestEqual(TEXT("retired viewer grant is stale"),
		Owner->Enqueue(Request(ViewerA, TEXT("viewer-a-retired")), Sink).Admission,
		EURLabInteractionAdmission::StaleGrant);
	TestEqual(TEXT("other viewer remains admitted"),
		Owner->Enqueue(Request(ViewerB, TEXT("viewer-b-live")), Sink).Admission,
		EURLabInteractionAdmission::Accepted);
	TestTrue(TEXT("other viewer contribution steps"),
		Runtime.AuthorizedStep(Claim.Handle, 1).IsOk());
	TestEqual(TEXT("other viewer receives post-step receipt"),
		Sink->Results[TEXT("viewer-b-live")].Terminal,
		EURLabInteractionTerminal::Applied);

	FURLabInteractionGrant StaleSourceGrant;
	TestTrue(TEXT("source-staleness grant issues"), Owner->IssueGrant(Runtime,
		Claim.Handle, Grant.Source, TEXT("viewer-source"), 5.0,
		StaleSourceGrant, Error));
	TestEqual(TEXT("source-bound work queues"), Owner->Enqueue(Request(
		StaleSourceGrant, TEXT("stale-source")), Sink).Admission,
		EURLabInteractionAdmission::Accepted);
	TestTrue(TEXT("source replacement binds atomically"),
		Owner->BindSource(Descriptor(TEXT("scene-r2"), 2), Error));
	TestTrue(TEXT("step after source replacement integrates"),
		Runtime.AuthorizedStep(Claim.Handle, 1).IsOk());
	TestEqual(TEXT("source replacement rejects queued interaction"),
		Sink->Results[TEXT("stale-source")].Terminal,
		EURLabInteractionTerminal::RejectedStale);

	FURLabInteractionGrant StaleControlGrant;
	const FURLabInteractionSourceIdentity SourceR2{TEXT("simulation-boot"),
		TEXT("simulation-poses"), 2, TEXT("scene-r2")};
	TestTrue(TEXT("control-staleness grant issues"), Owner->IssueGrant(Runtime,
		Claim.Handle, SourceR2, TEXT("viewer-control"), 5.0,
		StaleControlGrant, Error));
	TestEqual(TEXT("control-bound work queues"), Owner->Enqueue(Request(
		StaleControlGrant, TEXT("stale-control")), Sink).Admission,
		EURLabInteractionAdmission::Accepted);
	TestTrue(TEXT("old control releases"), Runtime.ReleaseControl(Claim.Handle).bChanged);
	const FMjSimulationControlClaimResult ReplacementClaim = Runtime.ClaimControl(ClaimRequest);
	if (!TestTrue(TEXT("replacement control claims"), ReplacementClaim.IsOk())) return false;
	TestTrue(TEXT("replacement controller steps"),
		Runtime.AuthorizedStep(ReplacementClaim.Handle, 1).IsOk());
	TestEqual(TEXT("control replacement rejects queued interaction"),
		Sink->Results[TEXT("stale-control")].Terminal,
		EURLabInteractionTerminal::RejectedStale);

	Owner->RevokeSource();
	FURLabInteractionGrant Unsupported;
	TestFalse(TEXT("missing pose publisher source is unsupported"), Owner->IssueGrant(
		Runtime, ReplacementClaim.Handle, SourceR2, TEXT("viewer-boot"), 5.0,
		Unsupported, Error));
	Runtime.SetInteractionOwner(nullptr);
	return !HasAnyErrors();
}

namespace URLabSimulationInteractionTest
{
const char* FrameXml = R"(
<mujoco model="interaction_frames">
  <option timestep="0.01" gravity="0 0 0"/>
  <worldbody>
    <body name="slider" euler="0 0 90">
      <joint name="slide" type="slide" axis="1 0 0"/>
      <geom type="sphere" size="0.05" mass="1"/>
    </body>
    <body name="spinner" pos="0 0 1">
      <joint name="spin" type="hinge" axis="0 0 1"/>
      <geom type="sphere" size="0.05" mass="1"/>
    </body>
  </worldbody>
</mujoco>)";

struct FFixture
{
	FMjSimulationRuntime Runtime;
	TSharedPtr<FMjSimulationInteractionOwner, ESPMode::ThreadSafe> Owner;
	TSharedPtr<FSink, ESPMode::ThreadSafe> Sink;
	FMjSimulationControlHandle Control;
	FURLabInteractionGrant Grant;
	bool bReady = false;

	explicit FFixture(const char* ModelXml)
	{
		TArray<uint8> Bytes;
		Bytes.Append(reinterpret_cast<const uint8*>(ModelXml), FCStringAnsi::Strlen(ModelXml));
		FMjOwnedModelCandidate Candidate;
		FString Error;
		if (!FMjOwnedModelCandidate::FromBytes(EMjModelSourceKind::XmlWithAssets, Bytes, {},
			Candidate, Error, TEXT("interaction frame fixture")))
		{
			return;
		}
		const FMjSimulationInstallResult Installed = Runtime.Install(MoveTemp(Candidate));
		if (!Installed.bCommitted) return;
		Owner = MakeShared<FMjSimulationInteractionOwner, ESPMode::ThreadSafe>();
		Runtime.SetInteractionOwner(Owner);
		if (!Owner->BindSource(Descriptor(TEXT("scene-r1")), Error)) return;
		auto Session = FMjSimulationControlSessionLifetime::CreateLocal(
			TEXT("interaction-control"), Runtime.GetControlClock(), Error);
		FMjSimulationControlClaimRequest ClaimRequest;
		ClaimRequest.Session = Session;
		ClaimRequest.SourceKind = EMjSimulationControlSourceKind::Local;
		ClaimRequest.ExpectedModelGeneration = Installed.Generation;
		const FMjSimulationControlClaimResult Claim = Runtime.ClaimControl(ClaimRequest);
		if (!Claim.IsOk()) return;
		Control = Claim.Handle;
		Sink = MakeShared<FSink, ESPMode::ThreadSafe>();
		bReady = Owner->IssueGrant(Runtime, Control, {TEXT("simulation-boot"),
			TEXT("simulation-poses"), 1, TEXT("scene-r1")}, TEXT("viewer-boot"), 30.0, Grant, Error);
	}

	~FFixture()
	{
		Runtime.SetInteractionOwner(nullptr);
	}

	double Qvel(int32 Index)
	{
		FMjModelObservation Observation;
		return Runtime.GetObservation(Observation) && Observation.Qvel.IsValidIndex(Index)
			? Observation.Qvel[Index] : TNumericLimits<double>::Max();
	}
};
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSimulationInteractionFramesTest,
	"URLab.Runtime.InteractionOwner.BodyFrameAndPointOfApplication",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSimulationInteractionFramesTest::RunTest(const FString&)
{
	using namespace URLabSimulationInteractionTest;
	FFixture Fixture(FrameXml);
	if (!TestTrue(TEXT("frame fixture is ready"), Fixture.bReady)) return false;
	auto World = Request(Fixture.Grant, TEXT("world-x"));
	TestEqual(TEXT("world-frame wrench enqueues"), Fixture.Owner->Enqueue(World, Fixture.Sink).Admission,
		EURLabInteractionAdmission::Accepted);
	TestTrue(TEXT("world step"), Fixture.Runtime.AuthorizedStep(Fixture.Control, 1).IsOk());
	TestTrue(TEXT("world +X is orthogonal to the rotated slide axis"),
		FMath::Abs(Fixture.Qvel(0)) < 1e-12);
	auto Body = Request(Fixture.Grant, TEXT("body-x"));
	Body.bBodyFrame = true;
	Fixture.Owner->Enqueue(Body, Fixture.Sink);
	TestTrue(TEXT("body step"), Fixture.Runtime.AuthorizedStep(Fixture.Control, 1).IsOk());
	TestTrue(TEXT("body-frame +X follows the slide axis"),
		FMath::IsNearlyEqual(Fixture.Qvel(0), 0.01, 1e-9));
	auto Lever = Request(Fixture.Grant, TEXT("lever"));
	Lever.BodyIndex = 2;
	Lever.ForceNewton = FVector3d(0.0, 1.0, 0.0);
	Lever.bHasPoint = true;
	Lever.PointMetre = FVector3d(1.0, 0.0, 1.0);
	Fixture.Owner->Enqueue(Lever, Fixture.Sink);
	TestTrue(TEXT("lever step"), Fixture.Runtime.AuthorizedStep(Fixture.Control, 1).IsOk());
	TestTrue(TEXT("off-COM force produces (p - com) x F torque"), Fixture.Qvel(1) > 0.0);
	TestEqual(TEXT("lever applied"), Fixture.Sink->Results[TEXT("lever")].Terminal,
		EURLabInteractionTerminal::Applied);
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSimulationInteractionHoldTest,
	"URLab.Runtime.InteractionOwner.HeldWrenchImpulseRevokeAndInputPermission",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSimulationInteractionHoldTest::RunTest(const FString&)
{
	using namespace URLabSimulationInteractionTest;
	FFixture Fixture(Xml);
	if (!TestTrue(TEXT("hold fixture is ready"), Fixture.bReady)) return false;
	auto Held = Request(Fixture.Grant, TEXT("held"));
	Held.DurationSeconds = 0.1;
	TestEqual(TEXT("hold enqueues"), Fixture.Owner->Enqueue(Held, Fixture.Sink).Admission,
		EURLabInteractionAdmission::Accepted);
	TestTrue(TEXT("hold steps"), Fixture.Runtime.AuthorizedStep(Fixture.Control, 20).IsOk());
	TestTrue(TEXT("held impulse equals force * duration"),
		FMath::IsNearlyEqual(Fixture.Qvel(0), 0.1, 1e-9));
	TestEqual(TEXT("hold reports its first step"),
		Fixture.Sink->Results[TEXT("held")].AppliedFrame.FrameId, uint64(1));
	TestEqual(TEXT("elapsed hold is gone"), Fixture.Owner->NumHolds(), 0);

	auto Long = Request(Fixture.Grant, TEXT("long"));
	Long.DurationSeconds = 10.0;
	Fixture.Owner->Enqueue(Long, Fixture.Sink);
	TestTrue(TEXT("long hold starts"), Fixture.Runtime.AuthorizedStep(Fixture.Control, 5).IsOk());
	const double BeforeRevoke = Fixture.Qvel(0);
	FString Error;
	TestTrue(TEXT("revoke mid-hold"), Fixture.Owner->RevokeGrant(Fixture.Runtime, Fixture.Control,
		Fixture.Grant.Source, Fixture.Grant.Token, Error));
	TestTrue(TEXT("steps after revoke"), Fixture.Runtime.AuthorizedStep(Fixture.Control, 5).IsOk());
	TestTrue(TEXT("revoked hold stops before the next step"),
		FMath::IsNearlyEqual(Fixture.Qvel(0), BeforeRevoke, 1e-12));

	bool bInputAllowed = true;
	Fixture.Owner->SetInputPermission([&bInputAllowed]() { return bInputAllowed; });
	FURLabInteractionGrant Second;
	TestTrue(TEXT("grant issues while input is allowed"), Fixture.Owner->IssueGrant(Fixture.Runtime,
		Fixture.Control, Fixture.Grant.Source, TEXT("viewer-boot"), 30.0, Second, Error));
	auto Input = Request(Second, TEXT("input-hold"));
	Input.DurationSeconds = 10.0;
	Fixture.Owner->Enqueue(Input, Fixture.Sink);
	TestTrue(TEXT("input hold starts"), Fixture.Runtime.AuthorizedStep(Fixture.Control, 1).IsOk());
	bInputAllowed = false;
	const double BeforeWithdraw = Fixture.Qvel(0);
	TestTrue(TEXT("steps after withdrawal"), Fixture.Runtime.AuthorizedStep(Fixture.Control, 3).IsOk());
	TestTrue(TEXT("withdrawn accept_input ends the hold"),
		FMath::IsNearlyEqual(Fixture.Qvel(0), BeforeWithdraw, 1e-12));
	TestEqual(TEXT("withdrawn accept_input revokes grants"), Fixture.Owner->NumGrants(), 0);
	FURLabInteractionGrant Refused;
	TestFalse(TEXT("no grant issues without accept_input"), Fixture.Owner->IssueGrant(
		Fixture.Runtime, Fixture.Control, Fixture.Grant.Source, TEXT("viewer-boot"), 30.0,
		Refused, Error));
	TestTrue(TEXT("refusal names accept_input"), Error.Contains(TEXT("accept_input")));
	return !HasAnyErrors();
}
#endif
