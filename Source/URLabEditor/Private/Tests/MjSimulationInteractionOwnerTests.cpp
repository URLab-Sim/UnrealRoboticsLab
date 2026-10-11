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
		EURLabInteractionTerminal::RejectedStale);

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
#endif
