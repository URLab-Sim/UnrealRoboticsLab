// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "Workflows/RenderOwnerAuthority.h"

#include <limits>

#if WITH_DEV_AUTOMATION_TESTS

struct FURLabRenderOwnerAuthorityTestAccess
{
	static void ExhaustAttachmentGeneration(FURLabRenderOwnerAuthority& Authority)
	{
		Authority.AttachmentGeneration = MAX_uint64;
	}
};

namespace URLabRenderOwnerAuthorityTestPrivate
{
FURLabRenderOwnerSessionScopePtr MakeScope(const TCHAR* Label)
{
	FString Error;
	return FURLabRenderOwnerSessionScope::Create(Label, Error);
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabRenderOwnerExclusiveLeaseTest,
	"URLab.Workflows.RenderOwnerAuthority.ExclusiveLeaseAndRetirementBarrier",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FURLabRenderOwnerExclusiveLeaseTest::RunTest(const FString&)
{
	using namespace URLabRenderOwnerAuthorityTestPrivate;
	double Now = 100.0;
	auto Clock = FURLabRenderOwnerClock::ForTest([&Now]() { return Now; });
	FURLabRenderOwnerAuthority Authority(Clock);
	auto Owner = MakeScope(TEXT("owner-a"));
	auto Other = MakeScope(TEXT("owner-b"));
	if (!TestTrue(TEXT("independent owner scopes mint"), Owner && Other)) return false;

	TestTrue(TEXT("fresh owner slot is available"), Authority.Status().Status.bAvailable);
	const FURLabRenderOwnerAttachResult Attached = Authority.Attach(Owner, 10.0);
	if (!TestTrue(TEXT("first exact owner attaches"), Attached.IsOk())) return false;
	TestTrue(TEXT("owner proof is valid"), Attached.Proof.IsValid());
	TestEqual(TEXT("first generation is one"), Attached.Proof.GetGeneration(), uint64(1));
	TestFalse(TEXT("opaque owner token is nonempty"), Attached.Proof.GetOpaqueToken().IsEmpty());
	TestEqual(TEXT("occupied slot rejects takeover"), Authority.Attach(Other).Error,
		EURLabRenderOwnerAuthorityError::Occupied);

	Now = 103.0;
	const FURLabRenderOwnerStatus FirstRead = Authority.Status().Status;
	TestTrue(TEXT("passive status reports active"), FirstRead.bActive);
	TestEqual(TEXT("passive status reports lease remaining"),
		FirstRead.SecondsRemaining, 7.0);
	Now = 104.0;
	TestEqual(TEXT("a later passive read does not renew the lease"),
		Authority.Status().Status.SecondsRemaining, 6.0);

	TestEqual(TEXT("wrong wire token cannot authorize"),
		Authority.Authorize(Owner, Attached.Proof.GetGeneration(), TEXT("wrong")).Error,
		EURLabRenderOwnerAuthorityError::InvalidProof);
	const FURLabRenderOwnerAuthorizationResult Authorized = Authority.Authorize(
		Owner, Attached.Proof.GetGeneration(), Attached.Proof.GetOpaqueToken());
	TestTrue(TEXT("exact wire credentials reconstruct an owned proof"), Authorized.IsOk());
	TestTrue(TEXT("reconstructed proof authorizes queued mutation"),
		Authority.Validate(Authorized.Proof).IsOk());

	Now = 105.0;
	const FURLabRenderOwnerLeaseResult Renewed = Authority.Renew(Attached.Proof, 20.0);
	TestTrue(TEXT("exact proof renews"), Renewed.IsOk() && Renewed.bChanged);
	TestEqual(TEXT("renewal uses requested TTL from current monotonic time"),
		Renewed.Status.SecondsRemaining, 20.0);
	Now = 125.0;
	const FURLabRenderOwnerStatus Boundary = Authority.Status().Status;
	TestTrue(TEXT("expiry boundary is stale"), Boundary.bStale);
	TestFalse(TEXT("expired slot remains occupied"), Boundary.bAvailable);
	TestEqual(TEXT("expired proof cannot mutate"), Authority.Validate(Attached.Proof).Error,
		EURLabRenderOwnerAuthorityError::Expired);

	const FURLabRenderOwnerRetirementResult Begun =
		Authority.BeginLifecycleRetirement(Owner);
	TestTrue(TEXT("expired exact lifecycle begins retirement"),
		Begun.IsOk() && Begun.bChanged && Begun.Token.IsValid());
	TestEqual(TEXT("retiring slot rejects attachment"), Authority.Attach(Other).Error,
		EURLabRenderOwnerAuthorityError::Retiring);
	const FURLabRenderOwnerRetirementResult Repeated =
		Authority.BeginLifecycleRetirement(Owner);
	TestTrue(TEXT("repeated begin returns retained barrier token"),
		Repeated.IsOk() && !Repeated.bChanged && Repeated.Token.IsValid());
	TestTrue(TEXT("retained retirement token completes barrier"),
		Authority.FinishRetirement(Repeated.Token).IsOk());

	Now = 126.0;
	const FURLabRenderOwnerAttachResult Successor = Authority.Attach(Other);
	if (!TestTrue(TEXT("successor attaches only after finish"), Successor.IsOk())) return false;
	TestEqual(TEXT("successor advances generation"), Successor.Proof.GetGeneration(), uint64(2));
	TestEqual(TEXT("old proof cannot affect successor"), Authority.Validate(Attached.Proof).Error,
		EURLabRenderOwnerAuthorityError::InvalidProof);
	TestEqual(TEXT("stale retirement cannot release successor"),
		Authority.FinishRetirement(Begun.Token).Error,
		EURLabRenderOwnerAuthorityError::InvalidRetirement);
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabRenderOwnerScopeIsolationTest,
	"URLab.Workflows.RenderOwnerAuthority.ScopeIsolationAndDeactivation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FURLabRenderOwnerScopeIsolationTest::RunTest(const FString&)
{
	using namespace URLabRenderOwnerAuthorityTestPrivate;
	double Now = 10.0;
	auto Clock = FURLabRenderOwnerClock::ForTest([&Now]() { return Now; });
	FURLabRenderOwnerAuthority FirstAuthority(Clock);
	auto OldScope = MakeScope(TEXT("same-session-label"));
	const FURLabRenderOwnerAttachResult OldAttach = FirstAuthority.Attach(OldScope);
	if (!TestTrue(TEXT("old exact scope attaches"), OldAttach.IsOk())) return false;

	OldScope->Deactivate();
	TestEqual(TEXT("queued proof observes one-way session deactivation"),
		FirstAuthority.Validate(OldAttach.Proof).Error,
		EURLabRenderOwnerAuthorityError::InactiveScope);
	TestTrue(TEXT("passive status reports deactivated owner stale"),
		FirstAuthority.Status().Status.bStale);
	Now = std::numeric_limits<double>::quiet_NaN();
	const FURLabRenderOwnerRetirementResult Retiring =
		FirstAuthority.BeginLifecycleRetirement(OldScope);
	TestTrue(TEXT("deactivated exact scope retires with invalid clock"), Retiring.IsOk());
	TestTrue(TEXT("retirement completion is clock independent"),
		FirstAuthority.FinishRetirement(Retiring.Token).IsOk());

	Now = 20.0;
	auto FreshSameLabel = MakeScope(TEXT("same-session-label"));
	const FURLabRenderOwnerAttachResult FreshAttach =
		FirstAuthority.Attach(FreshSameLabel);
	if (!TestTrue(TEXT("fresh same-label scope attaches"), FreshAttach.IsOk())) return false;
	TestEqual(TEXT("old same-label proof cannot authorize fresh scope"),
		FirstAuthority.Authorize(FreshSameLabel, OldAttach.Proof.GetGeneration(),
			OldAttach.Proof.GetOpaqueToken()).Error,
		EURLabRenderOwnerAuthorityError::InvalidProof);

	FURLabRenderOwnerAuthority SecondAuthority(Clock);
	const FURLabRenderOwnerAttachResult SecondAttach =
		SecondAuthority.Attach(FreshSameLabel);
	if (!TestTrue(TEXT("same session may own a distinct consumer"), SecondAttach.IsOk()))
		return false;
	TestEqual(TEXT("proof from another authority lifetime is foreign"),
		SecondAuthority.Validate(FreshAttach.Proof).Error,
		EURLabRenderOwnerAuthorityError::InvalidProof);
	TestEqual(TEXT("token from another authority cannot reconstruct proof"),
		SecondAuthority.Authorize(FreshSameLabel, FreshAttach.Proof.GetGeneration(),
			FreshAttach.Proof.GetOpaqueToken()).Error,
		EURLabRenderOwnerAuthorityError::InvalidProof);

	// The injected clock represents arbitrary caller code at the sampling
	// boundary. Every timed proof path must recheck the atomic scope afterward.
	for (int32 Operation = 0; Operation < 3; ++Operation)
	{
		double SampleNow = 40.0;
		bool bDeactivateDuringSample = false;
		auto SampleScope = MakeScope(*FString::Printf(TEXT("sample-owner-%d"), Operation));
		auto SampleClock = FURLabRenderOwnerClock::ForTest(
			[&SampleNow, &bDeactivateDuringSample, SampleScope]()
			{
				if (bDeactivateDuringSample) SampleScope->Deactivate();
				return SampleNow;
			});
		FURLabRenderOwnerAuthority SampleAuthority(SampleClock);
		const FURLabRenderOwnerAttachResult SampleAttach =
			SampleAuthority.Attach(SampleScope);
		if (!TestTrue(TEXT("clock-deactivation fixture attaches"), SampleAttach.IsOk()))
			return false;
		bDeactivateDuringSample = true;
		EURLabRenderOwnerAuthorityError Error = EURLabRenderOwnerAuthorityError::None;
		if (Operation == 0)
		{
			Error = SampleAuthority.Authorize(SampleScope,
				SampleAttach.Proof.GetGeneration(),
				SampleAttach.Proof.GetOpaqueToken()).Error;
		}
		else if (Operation == 1)
		{
			Error = SampleAuthority.Validate(SampleAttach.Proof).Error;
		}
		else
		{
			Error = SampleAuthority.Renew(SampleAttach.Proof).Error;
		}
		TestEqual(TEXT("scope deactivation during clock sample rejects timed proof"),
			Error, EURLabRenderOwnerAuthorityError::InactiveScope);
	}
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabRenderOwnerClockCleanupTest,
	"URLab.Workflows.RenderOwnerAuthority.ClockCleanupShutdownAndExhaustion",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FURLabRenderOwnerClockCleanupTest::RunTest(const FString&)
{
	using namespace URLabRenderOwnerAuthorityTestPrivate;
	double Now = 100.0;
	auto Clock = FURLabRenderOwnerClock::ForTest([&Now]() { return Now; });
	FURLabRenderOwnerAuthority Authority(Clock);
	auto Scope = MakeScope(TEXT("clock-owner"));
	const FURLabRenderOwnerAttachResult Attached = Authority.Attach(Scope, 30.0);
	if (!TestTrue(TEXT("clock fixture owner attaches"), Attached.IsOk())) return false;

	Now = 105.0;
	TestEqual(TEXT("oversized renewal is rejected"), Authority.Renew(Attached.Proof, 31.0).Error,
		EURLabRenderOwnerAuthorityError::InvalidTtl);
	Now = 104.0;
	TestEqual(TEXT("clock floor advances even on failed timed operation"),
		Authority.Validate(Attached.Proof).Error,
		EURLabRenderOwnerAuthorityError::ClockMovedBackward);
	Now = std::numeric_limits<double>::quiet_NaN();
	TestEqual(TEXT("nonfinite timed admission is rejected"),
		Authority.Validate(Attached.Proof).Error,
		EURLabRenderOwnerAuthorityError::InvalidClock);
	TestEqual(TEXT("normal wire authorization rejects an invalid clock"),
		Authority.Authorize(Scope, Attached.Proof.GetGeneration(),
			Attached.Proof.GetOpaqueToken()).Error,
		EURLabRenderOwnerAuthorityError::InvalidClock);
	const FURLabRenderOwnerAuthorizationResult InvalidClockRelease =
		Authority.AuthorizeForRelease(Scope, Attached.Proof.GetGeneration(),
			Attached.Proof.GetOpaqueToken());
	TestTrue(TEXT("exact wire credentials reconstruct cleanup proof with invalid clock"),
		InvalidClockRelease.IsOk());
	TestEqual(TEXT("wrong wire token cannot bypass invalid-clock cleanup identity"),
		Authority.AuthorizeForRelease(Scope, Attached.Proof.GetGeneration(),
			TEXT("wrong")).Error, EURLabRenderOwnerAuthorityError::InvalidProof);
	const FURLabRenderOwnerRetirementResult Released =
		Authority.BeginRelease(InvalidClockRelease.Proof);
	TestTrue(TEXT("explicit exact release ignores invalid clock"), Released.IsOk());
	TestTrue(TEXT("explicit retirement finish ignores invalid clock"),
		Authority.FinishRetirement(Released.Token).IsOk());

	Now = 110.0;
	auto ShutdownScope = MakeScope(TEXT("shutdown-owner"));
	const FURLabRenderOwnerAttachResult ShutdownAttach = Authority.Attach(ShutdownScope);
	if (!TestTrue(TEXT("shutdown fixture owner attaches"), ShutdownAttach.IsOk())) return false;
	Now = std::numeric_limits<double>::infinity();
	const FURLabRenderOwnerRetirementResult Shutdown =
		Authority.BeginShutdownRetirement();
	TestTrue(TEXT("workflow shutdown begins without a valid clock"), Shutdown.IsOk());
	const FURLabRenderOwnerRetirementResult RepeatedShutdown =
		Authority.BeginShutdownRetirement();
	TestTrue(TEXT("shutdown begin is idempotent while retiring"),
		RepeatedShutdown.IsOk() && !RepeatedShutdown.bChanged);
	TestTrue(TEXT("shutdown retirement completes"),
		Authority.FinishRetirement(RepeatedShutdown.Token).IsOk());
	TestFalse(TEXT("shutdown authority never becomes available"),
		Authority.Status().Status.bAvailable);
	TestEqual(TEXT("shutdown authority rejects later attachment"),
		Authority.Attach(MakeScope(TEXT("late-owner"))).Error,
		EURLabRenderOwnerAuthorityError::ShuttingDown);

	Now = 200.0;
	FURLabRenderOwnerAuthority Exhausted(Clock);
	FURLabRenderOwnerAuthorityTestAccess::ExhaustAttachmentGeneration(Exhausted);
	TestEqual(TEXT("attachment generation exhaustion is explicit"),
		Exhausted.Attach(MakeScope(TEXT("exhausted-owner"))).Error,
		EURLabRenderOwnerAuthorityError::CounterExhausted);

	double LargeNow = 1.0e16;
	auto LargeClock = FURLabRenderOwnerClock::ForTest([&LargeNow]() { return LargeNow; });
	FURLabRenderOwnerAuthority RoundedAttach(LargeClock);
	TestEqual(TEXT("positive TTL that cannot advance a finite clock is invalid clock"),
		RoundedAttach.Attach(MakeScope(TEXT("rounded-attach")),
			std::numeric_limits<double>::denorm_min()).Error,
		EURLabRenderOwnerAuthorityError::InvalidClock);
	FURLabRenderOwnerAuthority RoundedRenew(LargeClock);
	const FURLabRenderOwnerAttachResult LargeAttach =
		RoundedRenew.Attach(MakeScope(TEXT("rounded-renew")), 30.0);
	if (!TestTrue(TEXT("large finite clock supports representable lease"), LargeAttach.IsOk()))
		return false;
	const double OldRemaining = RoundedRenew.Status().Status.SecondsRemaining;
	TestEqual(TEXT("unrepresentable renewal deadline is invalid clock"),
		RoundedRenew.Renew(LargeAttach.Proof,
			std::numeric_limits<double>::denorm_min()).Error,
		EURLabRenderOwnerAuthorityError::InvalidClock);
	TestEqual(TEXT("failed deadline arithmetic preserves the old lease"),
		RoundedRenew.Status().Status.SecondsRemaining, OldRemaining);

	FURLabRenderOwnerAuthority ExpiredRelease(Clock);
	auto ExpiredScope = MakeScope(TEXT("expired-release"));
	const FURLabRenderOwnerAttachResult Expiring = ExpiredRelease.Attach(ExpiredScope, 1.0);
	Now = 201.0;
	TestEqual(TEXT("expired proof fails admission at exact boundary"),
		ExpiredRelease.Validate(Expiring.Proof).Error,
		EURLabRenderOwnerAuthorityError::Expired);
	TestEqual(TEXT("normal wire authorization rejects an expired lease"),
		ExpiredRelease.Authorize(ExpiredScope, Expiring.Proof.GetGeneration(),
			Expiring.Proof.GetOpaqueToken()).Error,
		EURLabRenderOwnerAuthorityError::Expired);
	const FURLabRenderOwnerAuthorizationResult ExpiredCredentials =
		ExpiredRelease.AuthorizeForRelease(ExpiredScope,
			Expiring.Proof.GetGeneration(), Expiring.Proof.GetOpaqueToken());
	TestTrue(TEXT("exact wire credentials reconstruct cleanup proof after expiry"),
		ExpiredCredentials.IsOk());
	TestEqual(TEXT("another scope cannot reconstruct expired cleanup authority"),
		ExpiredRelease.AuthorizeForRelease(MakeScope(TEXT("wrong-expired-scope")),
			Expiring.Proof.GetGeneration(), Expiring.Proof.GetOpaqueToken()).Error,
		EURLabRenderOwnerAuthorityError::InvalidProof);
	TestTrue(TEXT("wire-reconstructed exact release remains valid after expiry"),
		ExpiredRelease.BeginRelease(ExpiredCredentials.Proof).IsOk());
	return !HasAnyErrors();
}

#endif
