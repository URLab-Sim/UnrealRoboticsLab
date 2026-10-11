// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "Misc/AutomationTest.h"

#include "Bridge/DataSession.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"

#if WITH_DEV_AUTOMATION_TESTS
namespace URLabDataSessionStateMachineTestPrivate
{
struct FFixture
{
	static constexpr const TCHAR* BootId = TEXT("boot-data-session");

	bool Create(FAutomationTestBase& Test, const TCHAR* SessionId)
	{
#if !PLATFORM_LINUX
		return false;
#else
		FURLabPayloadStoreLimits Limits;
		Limits.MaxBlobBytes = 64;
		Limits.MaxChunkBytes = 64;
		Limits.MaxChunksPerBlob = 8;
		Limits.MaxSessionBytes = 256;
		Limits.MaxGlobalBytes = 512;
		Limits.MaxBlobCount = 16;
		Limits.MaxSessionBlobCount = 8;
		Limits.MaxSessionPhysicalObjectCount = 16;
		Limits.MaxActiveSessions = 4;
		Store = MakeUnique<FURLabPayloadStore>(BootId, Limits);
		FURLabPayloadSessionPtr PayloadSession;
		const FURLabPayloadStoreStatus Open = Store->OpenSession(
			BootId, SessionId, PayloadSession);
		if (!Test.TestTrue(TEXT("payload owner opens"), Open.IsOk())) return false;
		Owner = FURLabPayloadOwner::FromSession(PayloadSession);
		BaseDirectory = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("URLabTests"),
			TEXT("DataSession-") + FGuid::NewGuid().ToString(EGuidFormats::Digits));
		const FURLabShmExactStatus Created = FURLabShmExactRegion::Create(
			BootId, BaseDirectory, Region);
		if (!Test.TestTrue(TEXT("exact region creates"), Created.IsOk())) return false;
		return true;
#endif
	}

	FURLabShmExactChallengePtr CreateChallenge(FAutomationTestBase& Test)
	{
		FURLabShmExactChallengePtr Challenge;
		const FURLabShmExactStatus Status = Region->CreateChallenge(
			*Store, Owner, Challenge);
		Test.TestTrue(TEXT("challenge creates"), Status.IsOk());
		return Challenge;
	}

	FURLabDataSessionPtr CreateProvisional(FAutomationTestBase& Test,
		EURLabDataPolicy Policy, double ExpiresAt,
		FURLabShmExactChallengePtr& OutChallenge)
	{
		OutChallenge = CreateChallenge(Test);
		FString Error;
		FURLabDataSessionPtr Session = FURLabDataSession::CreateProvisionalShm(
			TEXT("zmq"), Policy, Owner, Region, OutChallenge, ExpiresAt, Error);
		if (!Test.TestTrue(TEXT("provisional data session creates"), Session.IsValid()))
			Test.AddError(Error);
		return Session;
	}

	bool MakeProof(FAutomationTestBase& Test,
		const FURLabShmExactChallengePtr& Challenge,
		FURLabShmExactChallengeProof& OutProof) const
	{
		const FURLabShmExactChallengeOffer& Offer = Challenge->GetOffer();
		OutProof.RegionId = Offer.RegionId;
		OutProof.RegionGeneration = Offer.RegionGeneration;
		OutProof.ChallengeName = Offer.ChallengeName;
		const FString Path = FPaths::Combine(Region->GetDescriptor().DirectoryPath,
			Offer.ChallengeName);
		return Test.TestTrue(TEXT("challenge proof bytes load"),
			FFileHelper::LoadFileToArray(OutProof.Nonce, *Path));
	}

	~FFixture()
	{
		Region.Reset();
		Store.Reset();
		if (!BaseDirectory.IsEmpty())
			IFileManager::Get().DeleteDirectory(*BaseDirectory, false, true);
	}

	FString BaseDirectory;
	TUniquePtr<FURLabPayloadStore> Store;
	FURLabPayloadOwner Owner;
	FURLabShmExactRegionPtr Region;
};
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabDataSessionTwoPhaseSelectionTest,
	"URLab.DataSession.TwoPhaseSelectionAndExactCommit",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabDataSessionTwoPhaseSelectionTest::RunTest(const FString&)
{
#if !PLATFORM_LINUX
	return true;
#else
	using namespace URLabDataSessionStateMachineTestPrivate;
	FFixture Fixture;
	if (!Fixture.Create(*this, TEXT("two-phase"))) return false;
	FURLabShmExactChallengePtr Challenge;
	FURLabDataSessionPtr Session = Fixture.CreateProvisional(
		*this, EURLabDataPolicy::Auto, 10.0, Challenge);
	if (!Session) return false;

	FURLabDataSelectionCommit Commit;
	FURLabShmExactChallengePtr Detached;
	if (!TestTrue(TEXT("network choice prepares"),
		Session->PrepareSelectNetwork(1.0, Commit, Detached).IsOk())) return false;
	TestTrue(TEXT("prepare returns exact commit token"), Commit.IsValid());
	TestTrue(TEXT("prepare detaches challenge for external retirement"), Detached.IsValid());
	TestFalse(TEXT("backend remains unusable before retirement"), Session->IsSelected());
	TestEqual(TEXT("snapshot remains provisional before completion"),
		Session->Snapshot(1.0).Transfer, EURLabDataTransfer::Provisional);
	FURLabDataSelectionCommit RefusedCommit;
	FURLabShmExactChallengePtr RefusedChallenge;
	TestEqual(TEXT("another selection cannot overtake pending retirement"),
		Session->PrepareSelectNetwork(1.0, RefusedCommit, RefusedChallenge).Code,
		EURLabDataSelectionError::SelectionConflict);
	FURLabShmExactChallengePtr SiblingChallenge;
	FURLabDataSessionPtr Sibling = Fixture.CreateProvisional(
		*this, EURLabDataPolicy::Auto, 10.0, SiblingChallenge);
	if (!Sibling) return false;
	FURLabDataSelectionCommit SiblingCommit;
	FURLabShmExactChallengePtr SiblingDetached;
	FURLabShmExactChallengeProof SiblingProof;
	if (!Fixture.MakeProof(*this, SiblingChallenge, SiblingProof)) return false;
	if (!TestTrue(TEXT("verified SHM choice prepares independently"),
		Sibling->PrepareSelectShm(
			1.0, SiblingProof, SiblingCommit, SiblingDetached).IsOk()))
		return false;
	TestEqual(TEXT("opaque commit cannot finalize another data-session state machine"),
		Sibling->CompleteSelection(Commit, true, FString()).Code,
		EURLabDataSelectionError::SelectionConflict);
	if (!TestTrue(TEXT("challenge retires outside session lock"),
		Fixture.Region->RetireChallenge(Fixture.Owner, Detached).IsOk())) return false;
	if (!TestTrue(TEXT("successful retirement commits pending backend"),
		Session->CompleteSelection(Commit, true, FString()).IsOk())) return false;
	TestTrue(TEXT("completed backend becomes usable"), Session->IsSelected());
	TestEqual(TEXT("completed backend is immutable network choice"),
		Session->Snapshot(1.0).Transfer, EURLabDataTransfer::Network);
	TestEqual(TEXT("duplicate completion cannot change selected backend"),
		Session->CompleteSelection(Commit, true, FString()).Code,
		EURLabDataSelectionError::SelectionConflict);
	TestTrue(TEXT("sibling challenge retires"), Fixture.Region->RetireChallenge(
		Fixture.Owner, SiblingDetached).IsOk());
	TestTrue(TEXT("sibling exact commit still completes"),
		Sibling->CompleteSelection(SiblingCommit, true, FString()).IsOk());
	TestEqual(TEXT("verified sibling backend commits as SHM"),
		Sibling->Snapshot(1.0).Transfer, EURLabDataTransfer::Shm);
	return !HasAnyErrors();
#endif
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabDataSessionExpiryBoundaryTest,
	"URLab.DataSession.LazyExpiryAndPendingBoundary",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabDataSessionExpiryBoundaryTest::RunTest(const FString&)
{
#if !PLATFORM_LINUX
	return true;
#else
	using namespace URLabDataSessionStateMachineTestPrivate;
	FFixture Fixture;
	if (!Fixture.Create(*this, TEXT("expiry"))) return false;
	FURLabShmExactChallengePtr Challenge;
	FURLabDataSessionPtr Session = Fixture.CreateProvisional(
		*this, EURLabDataPolicy::Shm, 10.0, Challenge);
	if (!Session) return false;
	FURLabShmExactChallengePtr Detached;
	TestTrue(TEXT("predeadline expiry probe is a no-op"),
		Session->ExpireChallenge(9.0, Detached).IsOk());
	TestFalse(TEXT("predeadline probe retains challenge"), Detached.IsValid());
	TestEqual(TEXT("deadline lazily expires challenge"),
		Session->ExpireChallenge(10.0, Detached).Code,
		EURLabDataSelectionError::NegotiationExpired);
	if (!TestTrue(TEXT("expiry detaches cleanup handle"), Detached.IsValid())) return false;
	TestTrue(TEXT("expired challenge retires outside data lock"),
		Fixture.Region->RetireChallenge(Fixture.Owner, Detached).IsOk());
	FURLabDataSelectionCommit ExpiredCommit;
	FURLabShmExactChallengePtr None;
	TestEqual(TEXT("future selection preserves expired error"),
		Session->PrepareSelectNetwork(10.0, ExpiredCommit, None).Code,
		EURLabDataSelectionError::NegotiationExpired);

	FURLabShmExactChallengePtr PendingChallenge;
	FURLabDataSessionPtr Pending = Fixture.CreateProvisional(
		*this, EURLabDataPolicy::Auto, 20.0, PendingChallenge);
	if (!Pending) return false;
	FURLabDataSelectionCommit PendingCommit;
	FURLabShmExactChallengePtr PendingDetached;
	if (!TestTrue(TEXT("predeadline choice prepares"), Pending->PrepareSelectNetwork(
		19.0, PendingCommit, PendingDetached).IsOk())) return false;
	FURLabShmExactChallengePtr ExpiryCannotSteal;
	TestTrue(TEXT("lazy expiry does not interfere with selection in progress"),
		Pending->ExpireChallenge(21.0, ExpiryCannotSteal).IsOk());
	TestFalse(TEXT("pending cleanup remains with preparing caller"),
		ExpiryCannotSteal.IsValid());
	TestTrue(TEXT("pending challenge retires"), Fixture.Region->RetireChallenge(
		Fixture.Owner, PendingDetached).IsOk());
	TestTrue(TEXT("choice prepared before deadline can complete"),
		Pending->CompleteSelection(PendingCommit, true, FString()).IsOk());
	return !HasAnyErrors();
#endif
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabDataSessionFailureAndOwnerTest,
	"URLab.DataSession.FailedRetirementAndOwnerDeactivation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabDataSessionFailureAndOwnerTest::RunTest(const FString&)
{
#if !PLATFORM_LINUX
	return true;
#else
	using namespace URLabDataSessionStateMachineTestPrivate;
	FFixture Fixture;
	if (!Fixture.Create(*this, TEXT("failure"))) return false;
	FURLabShmExactChallengePtr Challenge;
	FURLabDataSessionPtr Session = Fixture.CreateProvisional(
		*this, EURLabDataPolicy::Auto, 10.0, Challenge);
	if (!Session) return false;
	FURLabDataSelectionCommit Commit;
	FURLabShmExactChallengePtr Detached;
	if (!TestTrue(TEXT("failure fixture prepares"),
		Session->PrepareSelectNetwork(1.0, Commit, Detached).IsOk())) return false;
	TestEqual(TEXT("failed retirement disables the pending selection"),
		Session->CompleteSelection(Commit, false, TEXT("forced retirement failure")).Code,
		EURLabDataSelectionError::BackingFailure);
	TestTrue(TEXT("selection failure keeps protocol data scope live for ledger recovery"),
		Session->IsActive());
	TestFalse(TEXT("selection failure never exposes pending backend"), Session->IsSelected());
	TestEqual(TEXT("failed selection remains provisional"),
		Session->Snapshot(2.0).Transfer, EURLabDataTransfer::Provisional);
	FURLabDataSelectionCommit RetryCommit;
	FURLabShmExactChallengePtr RetryChallenge;
	TestEqual(TEXT("selection failure is permanent"), Session->PrepareSelectNetwork(
		2.0, RetryCommit, RetryChallenge).Code, EURLabDataSelectionError::BackingFailure);
	TestTrue(TEXT("test cleanup retires detached failed challenge"),
		Fixture.Region->RetireChallenge(Fixture.Owner, Detached).IsOk());

	FString Error;
	FURLabDataSessionPtr OwnerSession = FURLabDataSession::CreateProvisionalShm(
		TEXT("zmq"), EURLabDataPolicy::Auto, Fixture.Owner, Fixture.Region,
		Fixture.CreateChallenge(*this), 10.0, Error);
	if (!TestTrue(TEXT("owner-deactivation fixture creates"), OwnerSession.IsValid())) return false;
	FURLabDataSelectionCommit OwnerCommit;
	FURLabShmExactChallengePtr OwnerDetached;
	if (!TestTrue(TEXT("owner-deactivation choice prepares"),
		OwnerSession->PrepareSelectNetwork(1.0, OwnerCommit, OwnerDetached).IsOk())) return false;
	Fixture.Owner.Session->Deactivate();
	TestTrue(TEXT("detached challenge can retire after owner deactivation"),
		Fixture.Region->RetireChallenge(Fixture.Owner, OwnerDetached).IsOk());
	TestEqual(TEXT("inactive exact owner prevents backend commit"),
		OwnerSession->CompleteSelection(OwnerCommit, true, FString()).Code,
		EURLabDataSelectionError::Inactive);
	TestFalse(TEXT("owner deactivation leaves no selected backend"), OwnerSession->IsSelected());
	return !HasAnyErrors();
#endif
}

#endif // WITH_DEV_AUTOMATION_TESTS
