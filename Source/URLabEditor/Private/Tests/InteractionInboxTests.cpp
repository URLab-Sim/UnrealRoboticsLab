#include "Misc/AutomationTest.h"
#include "Workflows/InteractionInbox.h"

#include <limits>

#if WITH_DEV_AUTOMATION_TESTS
namespace URLabInteractionInboxTest
{
struct FHandler final : IURLabInteractionOwnerHandler
{
	int32 BodyCount = 2;
	int32 AppliedCount = 0;
	uint64 LastApplicationId = 0;
	TArray<double> AppliedForceX;
	virtual bool ValidateTarget(int32 BodyIndex, FString& OutError) override
	{
		if (BodyIndex >= 0 && BodyIndex < BodyCount) return true;
		OutError = TEXT("target is outside the owner body table");
		return false;
	}
	virtual bool ApplyNextStepContribution(const FURLabInteractionWrenchRequest& Request,
		uint64 ApplicationId, FString&) override
	{
		++AppliedCount;
		LastApplicationId = ApplicationId;
		AppliedForceX.Add(Request.ForceNewton.X);
		return true;
	}
};

struct FSink final : IURLabInteractionCompletionSink
{
	TMap<FString, FURLabInteractionResult> Results;
	virtual void CompleteInteraction(const FString& Ticket,
		const FURLabInteractionResult& Result) override
	{
		Results.Add(Ticket, Result);
	}
};

FURLabInteractionOwnerContext Context(uint64 Epoch = 4)
{
	FURLabInteractionOwnerContext Value;
	Value.Source.ProducerBootId = TEXT("producer-boot");
	Value.Source.StreamId = TEXT("poses");
	Value.Source.StreamGeneration = 3;
	Value.Source.SourceRevision = TEXT("scene-r1");
	Value.Control.AuthorityId = TEXT("simulation-control");
	Value.Control.Generation = 7;
	Value.Control.OpaqueProof = TEXT("opaque-proof");
	Value.ClockEpoch = Epoch;
	return Value;
}

FURLabInteractionWrenchRequest Request(const FURLabInteractionGrant& Grant,
	const FString& Ticket, int32 BodyIndex = 1)
{
	FURLabInteractionWrenchRequest Value;
	Value.Ticket = Ticket;
	Value.GrantToken = Grant.Token;
	Value.Source = Grant.Source;
	Value.ViewerBootId = Grant.ViewerBootId;
	Value.ClockEpoch = Grant.ClockEpoch;
	Value.BodyIndex = BodyIndex;
	Value.ForceNewton = FVector3d(1.0, 2.0, 3.0);
	Value.TorqueNewtonMetre = FVector3d(0.1, 0.2, 0.3);
	return Value;
}

FURLabInteractionOwnerContext ContextAt(double SourceTime)
{
	FURLabInteractionOwnerContext Value = Context();
	Value.SourceTime = SourceTime;
	return Value;
}

FURLabInteractionAppliedFrame Frame(uint64 FrameId, double SourceTime = 0.0)
{
	FURLabInteractionAppliedFrame Value;
	Value.FrameId = FrameId;
	Value.ClockEpoch = Context().ClockEpoch;
	Value.bHasSourceTime = true;
	Value.SourceTime = SourceTime;
	return Value;
}

/** One owner step: drain, then complete the newly drained application if any. */
void Step(FURLabInteractionInbox& Inbox, FHandler& Handler, double SourceTime, uint64 FrameId)
{
	const uint64 Before = Handler.LastApplicationId;
	Inbox.Drain(ContextAt(SourceTime));
	FString Error;
	if (Handler.LastApplicationId != Before)
		Inbox.CompleteApplied(Handler.LastApplicationId, ContextAt(SourceTime), Frame(FrameId,
			SourceTime), Error);
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabInteractionInboxLifecycleTest,
	"URLab.Workflows.InteractionInbox.GrantAuthorityExactlyOnceAndCompletion",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabInteractionInboxLifecycleTest::RunTest(const FString&)
{
	using namespace URLabInteractionInboxTest;
	double Now = 10.0;
	FURLabInteractionInbox Inbox(FURLabRenderOwnerClock::ForTest([&] { return Now; }));
	auto Handler = MakeShared<FHandler, ESPMode::ThreadSafe>();
	auto Sink = MakeShared<FSink, ESPMode::ThreadSafe>();
	FURLabInteractionGrant Grant;
	FString Error;
	TestFalse(TEXT("default-disabled inbox does not mint a handlerless grant"),
		Inbox.IssueGrant(Context(), TEXT("viewer-boot"), 5.0, nullptr,
			Grant, Error));
	if (!TestTrue(TEXT("explicit owner handler mints grant"), Inbox.IssueGrant(
		Context(), TEXT("viewer-boot"), 5.0, Handler, Grant, Error)))
	{
		AddError(Error);
		return false;
	}
	const FURLabInteractionWrenchRequest First = Request(Grant, TEXT("ticket-1"));
	TestEqual(TEXT("first ticket enqueues"), Inbox.Enqueue(First, Sink).Admission,
		EURLabInteractionAdmission::Accepted);
	TestEqual(TEXT("same ticket stays pending"), Inbox.Enqueue(First, Sink).Admission,
		EURLabInteractionAdmission::ExistingPending);
	auto Conflict = First;
	Conflict.ForceNewton.X = 9.0;
	TestEqual(TEXT("same ticket different wrench conflicts"), Inbox.Enqueue(Conflict, Sink).Admission,
		EURLabInteractionAdmission::Conflict);
	TestEqual(TEXT("owner drains one contribution"), Inbox.Drain(Context()), 1);
	TestEqual(TEXT("handler applied once"), Handler->AppliedCount, 1);
	TestTrue(TEXT("application id minted"), Handler->LastApplicationId != 0);
	TestFalse(TEXT("forwarding is not terminal before the authoritative step"),
		Sink->Results.Contains(TEXT("ticket-1")));
	FURLabInteractionAppliedFrame Applied;
	Applied.FrameId = 42;
	Applied.ClockEpoch = Context().ClockEpoch;
	Applied.bHasSourceTime = true;
	Applied.SourceTime = 1.25;
	TestTrue(TEXT("post-step completion succeeds"), Inbox.CompleteApplied(
		Handler->LastApplicationId, Context(), Applied, Error));
	TestEqual(TEXT("completion reports applied"), Sink->Results[TEXT("ticket-1")].Terminal,
		EURLabInteractionTerminal::Applied);
	TestEqual(TEXT("completion reports actual frame"),
		Sink->Results[TEXT("ticket-1")].AppliedFrame.FrameId, uint64(42));
	TestEqual(TEXT("terminal duplicate cannot apply twice"), Inbox.Enqueue(First, Sink).Admission,
		EURLabInteractionAdmission::ExistingTerminal);
	TestEqual(TEXT("handler still applied once"), Handler->AppliedCount, 1);
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabInteractionInboxStaleAndBoundsTest,
	"URLab.Workflows.InteractionInbox.StaleRevalidationValidationAndBounds",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabInteractionInboxStaleAndBoundsTest::RunTest(const FString&)
{
	using namespace URLabInteractionInboxTest;
	double Now = 20.0;
	FURLabInteractionInbox Inbox(FURLabRenderOwnerClock::ForTest([&] { return Now; }));
	auto Handler = MakeShared<FHandler, ESPMode::ThreadSafe>();
	auto Sink = MakeShared<FSink, ESPMode::ThreadSafe>();
	FURLabInteractionGrant Grant;
	FString Error;
	if (!Inbox.IssueGrant(Context(), TEXT("viewer-boot"), 5.0, Handler, Grant, Error))
		return false;
	auto Invalid = Request(Grant, TEXT("invalid"));
	Invalid.ForceNewton.X = std::numeric_limits<double>::quiet_NaN();
	TestEqual(TEXT("non-finite wrench rejects"), Inbox.Enqueue(Invalid, Sink).Admission,
		EURLabInteractionAdmission::Invalid);
	TestEqual(TEXT("stale target request enqueues for owner validation"),
		Inbox.Enqueue(Request(Grant, TEXT("bad-target"), 99), Sink).Admission,
		EURLabInteractionAdmission::Accepted);
	TestEqual(TEXT("owner drains rejected target"), Inbox.Drain(Context()), 1);
	TestEqual(TEXT("target rejection is terminal"),
		Sink->Results[TEXT("bad-target")].Terminal,
		EURLabInteractionTerminal::RejectedTarget);
	TestEqual(TEXT("epoch-bound request enqueues"),
		Inbox.Enqueue(Request(Grant, TEXT("stale-epoch")), Sink).Admission,
		EURLabInteractionAdmission::Accepted);
	TestEqual(TEXT("changed epoch drains stale work"), Inbox.Drain(Context(5)), 1);
	TestEqual(TEXT("epoch change rejects before handler"),
		Sink->Results[TEXT("stale-epoch")].Terminal,
		EURLabInteractionTerminal::RejectedSource);
	TestEqual(TEXT("no stale request reached handler"), Handler->AppliedCount, 0);
	for (int32 Index = 0; Index < FURLabInteractionInbox::MaxPending; ++Index)
	{
		TestEqual(TEXT("bounded request accepted"), Inbox.Enqueue(Request(Grant,
			FString::Printf(TEXT("queued-%d"), Index)), Sink).Admission,
			EURLabInteractionAdmission::Accepted);
	}
	TestEqual(TEXT("pending bound rejects overflow"), Inbox.Enqueue(Request(Grant,
		TEXT("overflow")), Sink).Admission, EURLabInteractionAdmission::Capacity);
	Inbox.RevokeGrant();
	TestEqual(TEXT("revocation makes new work stale"), Inbox.Enqueue(Request(Grant,
		TEXT("after-revoke")), Sink).Admission, EURLabInteractionAdmission::StaleGrant);
	TestEqual(TEXT("revoked queued work drains without applying"),
		Inbox.Drain(Context(), FURLabInteractionInbox::MaxPending),
		FURLabInteractionInbox::MaxPending);
	TestEqual(TEXT("revoked work remains unapplied"), Handler->AppliedCount, 0);
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabInteractionInboxRetentionTest,
	"URLab.Workflows.InteractionInbox.TerminalRecordsAreEvictedNotExhausted",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabInteractionInboxRetentionTest::RunTest(const FString&)
{
	using namespace URLabInteractionInboxTest;
	double Now = 10.0;
	FURLabInteractionInbox Inbox(FURLabRenderOwnerClock::ForTest([&] { return Now; }));
	auto Handler = MakeShared<FHandler, ESPMode::ThreadSafe>();
	auto Sink = MakeShared<FSink, ESPMode::ThreadSafe>();
	FURLabInteractionGrant Grant;
	FString Error;
	if (!Inbox.IssueGrant(Context(), TEXT("viewer-boot"), 30.0, Handler, Grant, Error))
		return false;
	constexpr int32 Total = 3 * FURLabInteractionInbox::MaxRetainedTickets;
	int32 Accepted = 0;
	for (int32 Index = 0; Index < Total; ++Index)
	{
		const FString Ticket = FString::Printf(TEXT("ticket-%d"), Index);
		if (Inbox.Enqueue(Request(Grant, Ticket), Sink).Admission
			== EURLabInteractionAdmission::Accepted)
		{
			++Accepted;
		}
		Step(Inbox, *Handler, Index * 0.01, Index + 1);
	}
	TestEqual(TEXT("every sequential wrench is admitted"), Accepted, Total);
	TestEqual(TEXT("every wrench applied exactly once"), Handler->AppliedCount, Total);
	TestEqual(TEXT("last wrench completed applied"),
		Sink->Results[FString::Printf(TEXT("ticket-%d"), Total - 1)].Terminal,
		EURLabInteractionTerminal::Applied);
	TestTrue(TEXT("retained records are bounded"),
		Inbox.NumRetained() <= FURLabInteractionInbox::MaxRetainedTickets);
	TestEqual(TEXT("recent terminal duplicate still replays"), Inbox.Enqueue(Request(Grant,
		FString::Printf(TEXT("ticket-%d"), Total - 1)), Sink).Admission,
		EURLabInteractionAdmission::ExistingTerminal);
	Inbox.RevokeGrant();
	TestEqual(TEXT("revoke-all drops terminal records"), Inbox.NumRetained(), 0);
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabInteractionInboxIdempotencyTest,
	"URLab.Workflows.InteractionInbox.InteractionIdNeverAppliesTwice",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabInteractionInboxIdempotencyTest::RunTest(const FString&)
{
	using namespace URLabInteractionInboxTest;
	double Now = 10.0;
	FURLabInteractionInbox Inbox(FURLabRenderOwnerClock::ForTest([&] { return Now; }));
	auto Handler = MakeShared<FHandler, ESPMode::ThreadSafe>();
	auto Sink = MakeShared<FSink, ESPMode::ThreadSafe>();
	FURLabInteractionGrant Grant;
	FString Error;
	if (!Inbox.IssueGrant(Context(), TEXT("viewer-boot"), 5.0, Handler, Grant, Error))
		return false;
	auto First = Request(Grant, TEXT("session:1"));
	First.InteractionId = TEXT("click-1");
	auto Retry = First;
	Retry.Ticket = TEXT("session:2");
	TestEqual(TEXT("first submission is accepted"), Inbox.Enqueue(First, Sink).Admission,
		EURLabInteractionAdmission::Accepted);
	TestEqual(TEXT("retry under a new ticket joins the pending interaction"),
		Inbox.Enqueue(Retry, Sink).Admission, EURLabInteractionAdmission::ExistingPending);
	Step(Inbox, *Handler, 0.0, 1);
	TestEqual(TEXT("one application for both tickets"), Handler->AppliedCount, 1);
	TestEqual(TEXT("original ticket applied"), Sink->Results[TEXT("session:1")].Terminal,
		EURLabInteractionTerminal::Applied);
	TestEqual(TEXT("retry ticket applied"), Sink->Results[TEXT("session:2")].Terminal,
		EURLabInteractionTerminal::Applied);
	auto Late = First;
	Late.Ticket = TEXT("session:3");
	const FURLabInteractionAdmissionResult Replay = Inbox.Enqueue(Late, Sink);
	TestEqual(TEXT("late retry replays the terminal result"), Replay.Admission,
		EURLabInteractionAdmission::ExistingTerminal);
	TestEqual(TEXT("replayed frame is the applied frame"), Replay.ExistingResult.AppliedFrame.FrameId,
		uint64(1));
	auto Changed = Late;
	Changed.ForceNewton.X = 5.0;
	TestEqual(TEXT("reused id with different wrench conflicts"), Inbox.Enqueue(Changed, Sink).Admission,
		EURLabInteractionAdmission::Conflict);
	Step(Inbox, *Handler, 0.01, 2);
	TestEqual(TEXT("still exactly one application"), Handler->AppliedCount, 1);
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabInteractionInboxGrantExpiryTest,
	"URLab.Workflows.InteractionInbox.ExpiredGrantsFreeCapacity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabInteractionInboxGrantExpiryTest::RunTest(const FString&)
{
	using namespace URLabInteractionInboxTest;
	double Now = 10.0;
	FURLabInteractionInbox Inbox(FURLabRenderOwnerClock::ForTest([&] { return Now; }));
	auto Handler = MakeShared<FHandler, ESPMode::ThreadSafe>();
	auto Sink = MakeShared<FSink, ESPMode::ThreadSafe>();
	FURLabInteractionGrant Grant;
	FString Error;
	for (int32 Index = 0; Index < FURLabInteractionInbox::MaxGrants; ++Index)
		TestTrue(TEXT("grant within capacity issues"), Inbox.IssueGrant(Context(),
			TEXT("viewer-boot"), 0.5, Handler, Grant, Error));
	TestFalse(TEXT("live grants exhaust capacity"), Inbox.IssueGrant(Context(),
		TEXT("viewer-boot"), 0.5, Handler, Grant, Error));
	TestTrue(TEXT("capacity error names capacity"), Error.Contains(TEXT("capacity")));
	TestEqual(TEXT("expired-grant work reports expiry"), Inbox.Enqueue(Request(Grant,
		TEXT("before-expiry")), Sink).Admission, EURLabInteractionAdmission::Accepted);
	Now += 1.0;
	TestEqual(TEXT("queued work on an expired grant drains"), Inbox.Drain(Context()), 1);
	TestEqual(TEXT("expiry is reported as expiry"), Sink->Results[TEXT("before-expiry")].Terminal,
		EURLabInteractionTerminal::RejectedExpired);
	TestEqual(TEXT("admission after expiry reports expiry"), Inbox.Enqueue(Request(Grant,
		TEXT("after-expiry")), Sink).Admission, EURLabInteractionAdmission::ExpiredGrant);
	TestTrue(TEXT("expired grants no longer hold capacity"), Inbox.IssueGrant(Context(),
		TEXT("viewer-boot"), 0.5, Handler, Grant, Error));
	TestEqual(TEXT("expired grants were pruned"), Inbox.NumGrants(), 1);

	FURLabInteractionGrant Short;
	TestTrue(TEXT("short grant issues"), Inbox.IssueGrant(Context(), TEXT("viewer-boot"), 0.1,
		Handler, Short, Error));
	TestEqual(TEXT("work queues before expiry"), Inbox.Enqueue(Request(Short, TEXT("in-step")),
		Sink).Admission, EURLabInteractionAdmission::Accepted);
	TestEqual(TEXT("work drains before expiry"), Inbox.Drain(Context()), 1);
	Now += 1.0;
	TestTrue(TEXT("completion after expiry succeeds"), Inbox.CompleteApplied(
		Handler->LastApplicationId, Context(), Frame(7), Error));
	TestEqual(TEXT("integrated wrench reports applied even if the grant lapsed in the step"),
		Sink->Results[TEXT("in-step")].Terminal, EURLabInteractionTerminal::Applied);
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabInteractionInboxHoldTest,
	"URLab.Workflows.InteractionInbox.HeldWrenchDurationChannelAndRevoke",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabInteractionInboxHoldTest::RunTest(const FString&)
{
	using namespace URLabInteractionInboxTest;
	double Now = 10.0;
	FURLabInteractionInbox Inbox(FURLabRenderOwnerClock::ForTest([&] { return Now; }));
	auto Handler = MakeShared<FHandler, ESPMode::ThreadSafe>();
	auto Sink = MakeShared<FSink, ESPMode::ThreadSafe>();
	FURLabInteractionGrant Grant;
	FString Error;
	if (!Inbox.IssueGrant(Context(), TEXT("viewer-boot"), 30.0, Handler, Grant, Error))
		return false;
	auto TooLong = Request(Grant, TEXT("too-long"));
	TooLong.DurationSeconds = 31.0;
	TestEqual(TEXT("duration beyond 30 s is invalid"), Inbox.Enqueue(TooLong, Sink).Admission,
		EURLabInteractionAdmission::Invalid);

	constexpr double Dt = 0.01;
	auto Held = Request(Grant, TEXT("held"));
	Held.DurationSeconds = 0.05;
	TestEqual(TEXT("hold enqueues"), Inbox.Enqueue(Held, Sink).Admission,
		EURLabInteractionAdmission::Accepted);
	for (int32 StepIndex = 0; StepIndex < 10; ++StepIndex)
		Step(Inbox, *Handler, 1.0 + StepIndex * Dt, StepIndex + 1);
	TestEqual(TEXT("hold applies on ceil(duration/dt) steps"), Handler->AppliedCount, 5);
	TestEqual(TEXT("hold completes once, after its first step"),
		Sink->Results[TEXT("held")].AppliedFrame.FrameId, uint64(1));
	TestEqual(TEXT("elapsed hold is dropped"), Inbox.NumHolds(), 0);

	Handler->AppliedForceX.Reset();
	auto DragA = Request(Grant, TEXT("drag-a"));
	DragA.ForceNewton.X = 10.0;
	DragA.DurationSeconds = 1.0;
	DragA.Channel = TEXT("drag");
	Inbox.Enqueue(DragA, Sink);
	Step(Inbox, *Handler, 2.0, 100);
	Step(Inbox, *Handler, 2.0 + Dt, 101);
	auto DragB = DragA;
	DragB.Ticket = TEXT("drag-b");
	DragB.ForceNewton.X = 20.0;
	Inbox.Enqueue(DragB, Sink);
	Step(Inbox, *Handler, 2.0 + 2 * Dt, 102);
	Step(Inbox, *Handler, 2.0 + 3 * Dt, 103);
	TestEqual(TEXT("channel update replaces the previous hold in the same step"),
		Handler->AppliedForceX, TArray<double>({10.0, 10.0, 20.0, 20.0}));
	TestEqual(TEXT("one hold per channel"), Inbox.NumHolds(), 1);
	auto Release = DragA;
	Release.Ticket = TEXT("release");
	Release.ForceNewton = FVector3d::ZeroVector;
	Release.DurationSeconds = 0.0;
	Inbox.Enqueue(Release, Sink);
	Step(Inbox, *Handler, 2.0 + 4 * Dt, 104);
	Step(Inbox, *Handler, 2.0 + 5 * Dt, 105);
	TestEqual(TEXT("zero-duration channel submission releases the hold"), Inbox.NumHolds(), 0);
	TestEqual(TEXT("release applies once and nothing after"), Handler->AppliedForceX,
		TArray<double>({10.0, 10.0, 20.0, 20.0, 0.0}));

	FURLabInteractionGrant Other;
	TestTrue(TEXT("second grant issues"), Inbox.IssueGrant(Context(), TEXT("viewer-boot"), 30.0,
		Handler, Other, Error));
	auto Long = Request(Grant, TEXT("long"));
	Long.DurationSeconds = 10.0;
	auto OtherLong = Request(Other, TEXT("other-long"));
	OtherLong.DurationSeconds = 10.0;
	Inbox.Enqueue(Long, Sink);
	Inbox.Enqueue(OtherLong, Sink);
	Step(Inbox, *Handler, 3.0, 200);
	TestEqual(TEXT("both holds are active"), Inbox.NumHolds(), 2);
	TestTrue(TEXT("exact revoke succeeds"), Inbox.RevokeGrant(Grant.Token));
	TestEqual(TEXT("revoke ends that grant's hold immediately"), Inbox.NumHolds(), 1);
	const int32 Applied = Handler->AppliedCount;
	Step(Inbox, *Handler, 3.0 + Dt, 201);
	TestEqual(TEXT("only the unrevoked hold applies"), Handler->AppliedCount, Applied + 1);
	Inbox.RevokeGrant();
	Step(Inbox, *Handler, 3.0 + 2 * Dt, 202);
	TestEqual(TEXT("revoke-all ends every hold"), Handler->AppliedCount, Applied + 1);
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabInteractionInboxAbortTest,
	"URLab.Workflows.InteractionInbox.AbortedStepFailsApplication",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabInteractionInboxAbortTest::RunTest(const FString&)
{
	using namespace URLabInteractionInboxTest;
	double Now = 10.0;
	FURLabInteractionInbox Inbox(FURLabRenderOwnerClock::ForTest([&] { return Now; }));
	auto Handler = MakeShared<FHandler, ESPMode::ThreadSafe>();
	auto Sink = MakeShared<FSink, ESPMode::ThreadSafe>();
	FURLabInteractionGrant Grant;
	FString Error;
	if (!Inbox.IssueGrant(Context(), TEXT("viewer-boot"), 5.0, Handler, Grant, Error))
		return false;
	Inbox.Enqueue(Request(Grant, TEXT("aborted")), Sink);
	Inbox.Drain(Context());
	TestTrue(TEXT("abort settles the drained application"),
		Inbox.AbortApplied(Handler->LastApplicationId, TEXT("step did not run")));
	TestEqual(TEXT("aborted application reports apply failure"),
		Sink->Results[TEXT("aborted")].Terminal, EURLabInteractionTerminal::ApplyFailed);
	TestFalse(TEXT("aborted application cannot complete later"), Inbox.CompleteApplied(
		Handler->LastApplicationId, Context(), Frame(3), Error));
	return !HasAnyErrors();
}
#endif
