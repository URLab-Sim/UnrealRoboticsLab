// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Async/Async.h"
#include "Bridge/RequestLedger.h"

namespace
{
FURLabRequestDigest Digest(uint8 Seed)
{
	FURLabRequestDigest Out;
	for (int32 Index = 0; Index < 32; ++Index)
		Out.Bytes[Index] = Seed + Index;
	return Out;
}

TArray<uint8> Bytes(int32 Num, uint8 Value = 7)
{
	TArray<uint8> Out;
	Out.Init(Value, Num);
	return Out;
}
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRequestLedgerIdentity,
	"URLab.Bridge.RequestLedger.IdentityAndConcurrency",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRequestLedgerIdentity::RunTest(const FString& Parameters)
{
	FURLabRequestLedger Ledger;
	auto Session = Ledger.OpenSession(TEXT("identity"));
	const FURLabRequestDigest A = Digest(1);
	const FURLabRequestDigest B = Digest(2);
	TestEqual(TEXT("positive ID admitted"), Ledger.Admit(Session, 1, A).Status,
		FURLabRequestLedger::EAdmission::Accepted);
	TestEqual(TEXT("same ID and digest is pending duplicate"), Ledger.Admit(Session, 1, A).Status,
		FURLabRequestLedger::EAdmission::ExistingPending);
	TestEqual(TEXT("same ID different digest conflicts"), Ledger.Admit(Session, 1, B).Status,
		FURLabRequestLedger::EAdmission::Conflict);
	TestEqual(TEXT("zero ID rejected"), Ledger.Admit(Session, 0, A).Status,
		FURLabRequestLedger::EAdmission::InvalidId);
	FURLabRequestLedger ForeignLedger;
	TestEqual(TEXT("foreign ledger rejects session handle"),
		ForeignLedger.Admit(Session, 2, A).Status, FURLabRequestLedger::EAdmission::Closed);
	TestFalse(TEXT("foreign ledger cannot complete session work"),
		ForeignLedger.Complete(Session, 1, A, Bytes(1)));
	ForeignLedger.CloseSession(Session);
	TestEqual(TEXT("foreign close does not revoke owning ledger session"),
		Ledger.Admit(Session, 2, A).Status, FURLabRequestLedger::EAdmission::Accepted);
	TestTrue(TEXT("post-foreign-close request completes"),
		Ledger.Complete(Session, 2, A, Bytes(1)));
	TestTrue(TEXT("pending completes"), Ledger.Complete(Session, 1, A, Bytes(3, 9)));
	const auto StatsAfterFirstCompletion = Ledger.GetStats();
	TestFalse(TEXT("completion is first-terminal-wins"),
		Ledger.Complete(Session, 1, A, Bytes(2, 4)));
	TestEqual(TEXT("duplicate completion does not release pending twice"),
		Ledger.GetStats().Pending, StatsAfterFirstCompletion.Pending);
	TestEqual(TEXT("duplicate completion does not change retained accounting"),
		Ledger.GetStats().RetainedBytes, StatsAfterFirstCompletion.RetainedBytes);
	const auto Terminal = Ledger.Admit(Session, 1, A);
	TestEqual(TEXT("duplicate returns terminal"), Terminal.Status,
		FURLabRequestLedger::EAdmission::ExistingTerminal);
	TestEqual(TEXT("terminal bytes retained"), Terminal.ReplyBytes->Num(), 3);

	auto Concurrent = Ledger.OpenSession(TEXT("concurrent"));
	TArray<TFuture<FURLabRequestLedger::EAdmission>> Futures;
	for (int32 Index = 0; Index < 16; ++Index)
	{
		Futures.Add(Async(EAsyncExecution::Thread, [&Ledger, Concurrent, A]() {
			return Ledger.Admit(Concurrent, 7, A).Status;
		}));
	}
	int32 Accepted = 0;
	int32 PendingDuplicate = 0;
	for (auto& Future : Futures)
	{
		const auto Status = Future.Get();
		Accepted += Status == FURLabRequestLedger::EAdmission::Accepted;
		PendingDuplicate += Status == FURLabRequestLedger::EAdmission::ExistingPending;
	}
	TestEqual(TEXT("concurrent duplicate has one winner"), Accepted, 1);
	TestEqual(TEXT("other concurrent calls see pending"), PendingDuplicate, 15);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRequestLedgerWindow,
	"URLab.Bridge.RequestLedger.WindowAndAtomicAdmission",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRequestLedgerWindow::RunTest(const FString& Parameters)
{
	FURLabRequestLedger Ledger;
	auto Session = Ledger.OpenSession(TEXT("window"));
	const auto A = Digest(4);
	TestEqual(TEXT("first admitted"), Ledger.Admit(Session, 1, A).Status,
		FURLabRequestLedger::EAdmission::Accepted);
	TestTrue(TEXT("first completed"), Ledger.Complete(Session, 1, A, Bytes(1)));
	TestEqual(TEXT("large skipped-ID jump admitted"), Ledger.Admit(Session, 5000, A).Status,
		FURLabRequestLedger::EAdmission::Accepted);
	TestEqual(TEXT("floor advanced without iterating gap"), Ledger.GetRetiredWatermark(Session),
		static_cast<uint64>(5000 - FURLabRequestLedger::WindowWidth));
	TestEqual(TEXT("old ID permanently retired"), Ledger.Admit(Session, 1, A).Status,
		FURLabRequestLedger::EAdmission::Retired);

	auto Blocked = Ledger.OpenSession(TEXT("blocked"));
	TestEqual(TEXT("pending low ID admitted"), Ledger.Admit(Blocked, 1, A).Status,
		FURLabRequestLedger::EAdmission::Accepted);
	const uint64 HighBefore = Ledger.GetHighWatermark(Blocked);
	TestEqual(TEXT("jump crossing pending rejected"), Ledger.Admit(Blocked, 5000, A).Status,
		FURLabRequestLedger::EAdmission::PendingBlocksRetirement);
	TestEqual(TEXT("failed plan leaves high watermark unchanged"),
		Ledger.GetHighWatermark(Blocked), HighBefore);
	TestEqual(TEXT("failed plan leaves only original reservation"),
		Ledger.GetSessionStats(Blocked).RetainedBytes,
		FURLabRequestLedger::MaxInlineReplyBytes);

	auto Entries = Ledger.OpenSession(TEXT("entry-bound"));
	for (int32 Id = 1; Id <= FURLabRequestLedger::MaxEntriesPerSession; ++Id)
	{
		TestEqual(TEXT("terminal entry admitted"), Ledger.Admit(Entries, Id, A).Status,
			FURLabRequestLedger::EAdmission::Accepted);
		TestTrue(TEXT("terminal entry completed"), Ledger.Complete(Entries, Id, A, Bytes(0)));
	}
	TestEqual(TEXT("entry bound retires earliest terminal atomically"),
		Ledger.Admit(Entries, 257, A).Status, FURLabRequestLedger::EAdmission::Accepted);
	TestEqual(TEXT("entry-bound retirement advances watermark"),
		Ledger.GetRetiredWatermark(Entries), static_cast<uint64>(1));
	TestEqual(TEXT("retired terminal cannot reexecute"), Ledger.Admit(Entries, 1, A).Status,
		FURLabRequestLedger::EAdmission::Retired);

	auto IncomingWouldRetire = Ledger.OpenSession(TEXT("incoming-retirement"));
	for (int32 Id = 2; Id <= 257; ++Id)
	{
		TestEqual(TEXT("offset terminal admitted"),
			Ledger.Admit(IncomingWouldRetire, Id, A).Status,
			FURLabRequestLedger::EAdmission::Accepted);
		TestTrue(TEXT("offset terminal completed"),
			Ledger.Complete(IncomingWouldRetire, Id, A, Bytes(0)));
	}
	const uint64 RetiredBeforeRejectedIncoming = Ledger.GetRetiredWatermark(IncomingWouldRetire);
	const uint64 HighBeforeRejectedIncoming = Ledger.GetHighWatermark(IncomingWouldRetire);
	TestEqual(TEXT("capacity cannot retire the incoming skipped ID"),
		Ledger.Admit(IncomingWouldRetire, 1, A).Status,
		FURLabRequestLedger::EAdmission::SessionCapacity);
	TestEqual(TEXT("rejected incoming leaves retired watermark unchanged"),
		Ledger.GetRetiredWatermark(IncomingWouldRetire), RetiredBeforeRejectedIncoming);
	TestEqual(TEXT("rejected incoming leaves high watermark unchanged"),
		Ledger.GetHighWatermark(IncomingWouldRetire), HighBeforeRejectedIncoming);
	TestEqual(TEXT("rejected incoming leaves retained terminal intact"),
		Ledger.Admit(IncomingWouldRetire, 2, A).Status,
		FURLabRequestLedger::EAdmission::ExistingTerminal);

	auto BytePressure = Ledger.OpenSession(TEXT("byte-pressure"));
	for (int32 Id = 1; Id <= 32; ++Id)
	{
		TestEqual(TEXT("full reply admitted"), Ledger.Admit(BytePressure, Id, A).Status,
			FURLabRequestLedger::EAdmission::Accepted);
		TestTrue(TEXT("full reply completed"), Ledger.Complete(BytePressure, Id, A,
			Bytes(FURLabRequestLedger::MaxInlineReplyBytes)));
	}
	TestEqual(TEXT("byte pressure retires earliest terminal for new reservation"),
		Ledger.Admit(BytePressure, 33, A).Status,
		FURLabRequestLedger::EAdmission::Accepted);
	TestEqual(TEXT("byte-pressure retirement advances watermark"),
		Ledger.GetRetiredWatermark(BytePressure), static_cast<uint64>(1));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRequestLedgerBounds,
	"URLab.Bridge.RequestLedger.BoundsAndClosingLifetime",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRequestLedgerBounds::RunTest(const FString& Parameters)
{
	{
		FURLabRequestLedger DropLedger;
		const auto DropDigest = Digest(42);
		{
			auto Dropped = DropLedger.OpenSession(TEXT("raii-terminal"));
			TestEqual(TEXT("RAII terminal admitted"), DropLedger.Admit(Dropped, 1, DropDigest).Status,
				FURLabRequestLedger::EAdmission::Accepted);
			TestTrue(TEXT("RAII terminal completed"),
				DropLedger.Complete(Dropped, 1, DropDigest, Bytes(11)));
		}
		TestEqual(TEXT("last session ref releases retained accounting"),
			DropLedger.GetStats().RetainedBytes, static_cast<int64>(0));
		TestEqual(TEXT("last session ref leaves no pending accounting"),
			DropLedger.GetStats().Pending, 0);
		{
			auto DroppedPending = DropLedger.OpenSession(TEXT("raii-pending"));
			TestEqual(TEXT("RAII pending admitted"),
				DropLedger.Admit(DroppedPending, 1, DropDigest).Status,
				FURLabRequestLedger::EAdmission::Accepted);
		}
		TestEqual(TEXT("unretained execution handle releases pending credit"),
			DropLedger.GetStats().Pending, 0);
		TestEqual(TEXT("unretained execution handle releases byte reservation"),
			DropLedger.GetStats().RetainedBytes, static_cast<int64>(0));
	}

	FURLabRequestLedger Ledger;
	const auto A = Digest(8);
	auto Session = Ledger.OpenSession(TEXT("session-bound"));
	for (int32 Id = 1; Id <= FURLabRequestLedger::MaxPendingPerSession; ++Id)
	{
		TestEqual(TEXT("pending slot admitted"), Ledger.Admit(Session, Id, A).Status,
			FURLabRequestLedger::EAdmission::Accepted);
	}
	const uint64 SessionHighAtCapacity = Ledger.GetHighWatermark(Session);
	TestEqual(TEXT("session pending/reply reservation bound enforced"),
		Ledger.Admit(Session, 33, A).Status, FURLabRequestLedger::EAdmission::SessionCapacity);
	TestEqual(TEXT("session count rejection leaves watermark unchanged"),
		Ledger.GetHighWatermark(Session), SessionHighAtCapacity);
	TestEqual(TEXT("session reserves exactly 8 MiB"), Ledger.GetSessionStats(Session).RetainedBytes,
		FURLabRequestLedger::MaxRetainedBytesPerSession);

	TArray<FURLabRequestLedger::FSessionRef> Sessions;
	for (int32 SessionIndex = 0; SessionIndex < 7; ++SessionIndex)
	{
		auto More = Ledger.OpenSession(FString::Printf(TEXT("global-%d"), SessionIndex));
		Sessions.Add(More);
		for (int32 Id = 1; Id <= 32; ++Id)
			Ledger.Admit(More, Id, A);
	}
	auto Full = Ledger.OpenSession(TEXT("global-full"));
	TestEqual(TEXT("global pending/reply reservation bound enforced"),
		Ledger.Admit(Full, 1, A).Status, FURLabRequestLedger::EAdmission::GlobalCapacity);
	TestEqual(TEXT("global count rejection does not admit an ID"),
		Ledger.GetHighWatermark(Full), static_cast<uint64>(0));
	TestEqual(TEXT("global reserves exactly 64 MiB"), Ledger.GetStats().RetainedBytes,
		FURLabRequestLedger::MaxRetainedBytesGlobal);

	TestTrue(TEXT("completion releases unused reservation"),
		Ledger.Complete(Session, 1, A, Bytes(4)));
	TestEqual(TEXT("four retained bytes still prevent a full reservation"),
		Ledger.Admit(Full, 1, A).Status, FURLabRequestLedger::EAdmission::GlobalCapacity);
	Ledger.CloseSession(Session);
	TestEqual(TEXT("discarding terminal bytes releases exact aggregate capacity"),
		Ledger.Admit(Full, 1, A).Status, FURLabRequestLedger::EAdmission::Accepted);
	TestTrue(TEXT("newly admitted aggregate slot can complete"),
		Ledger.Complete(Full, 1, A, Bytes(1)));

	FURLabRequestLedger LifetimeLedger;
	auto Closing = LifetimeLedger.OpenSession(TEXT("closing"));
	TestEqual(TEXT("closing work accepted"), LifetimeLedger.Admit(Closing, 1, A).Status,
		FURLabRequestLedger::EAdmission::Accepted);
	const int32 PendingBeforeClose = LifetimeLedger.GetStats().Pending;
	LifetimeLedger.CloseSession(Closing);
	TestEqual(TEXT("closed session rejects admission"), LifetimeLedger.Admit(Closing, 2, A).Status,
		FURLabRequestLedger::EAdmission::Closed);
	TestEqual(TEXT("detached pending remains globally accounted"), LifetimeLedger.GetStats().Pending,
		PendingBeforeClose);
	TestTrue(TEXT("shared closing record completes safely"),
		LifetimeLedger.Complete(Closing, 1, A, Bytes(2)));
	TestFalse(TEXT("closing completion is also first-terminal-wins"),
		LifetimeLedger.Complete(Closing, 1, A, Bytes(2)));
	TestEqual(TEXT("closing completion releases pending account"), LifetimeLedger.GetStats().Pending,
		PendingBeforeClose - 1);

	auto Oversize = LifetimeLedger.OpenSession(TEXT("oversize"));
	TestEqual(TEXT("oversize work accepted"), LifetimeLedger.Admit(Oversize, 1, A).Status,
		FURLabRequestLedger::EAdmission::Accepted);
	TestTrue(TEXT("oversize completion accepted"), LifetimeLedger.Complete(Oversize, 1, A,
		Bytes(FURLabRequestLedger::MaxInlineReplyBytes + 1)));
	const auto Unavailable = LifetimeLedger.Admit(Oversize, 1, A);
	TestEqual(TEXT("oversize is bounded terminal error"), Unavailable.Terminal,
		FURLabRequestLedger::ETerminal::ResultUnavailable);
	TestFalse(TEXT("oversize bytes are discarded without a fake encoded reply"),
		Unavailable.ReplyBytes.IsValid());
	return true;
}
