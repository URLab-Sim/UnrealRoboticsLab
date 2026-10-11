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
	virtual bool ValidateTarget(int32 BodyIndex, FString& OutError) override
	{
		if (BodyIndex >= 0 && BodyIndex < BodyCount) return true;
		OutError = TEXT("target is outside the owner body table");
		return false;
	}
	virtual bool ApplyNextStepContribution(const FURLabInteractionWrenchRequest&,
		uint64 ApplicationId, FString&) override
	{
		++AppliedCount;
		LastApplicationId = ApplicationId;
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
		EURLabInteractionTerminal::RejectedStale);
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
#endif
