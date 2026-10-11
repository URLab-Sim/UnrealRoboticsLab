// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "Bridge/URLabPayloadStore.h"
#include "Utils/AssetCache.h"

#include <limits>

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
TArray<uint8> Bytes(std::initializer_list<uint8> Values)
{
	TArray<uint8> Out;
	Out.Append(Values.begin(), static_cast<int32>(Values.size()));
	return Out;
}

TArray<uint8> PinnedNetworkBytes(const FURLabPayloadPin& Pin)
{
	TConstArrayView<uint8> View;
	if (!Pin || !Pin->TryGetNetworkBytes(View)) return {};
	TArray<uint8> Out;
	Out.Append(View.GetData(), View.Num());
	return Out;
}

FURLabPayloadStoreLimits SmallLimits()
{
	FURLabPayloadStoreLimits Out;
	Out.MaxBlobBytes = 32;
	Out.MaxChunkBytes = 8;
	Out.MaxChunksPerBlob = 8;
	Out.MaxSessionBytes = 32;
	Out.MaxGlobalBytes = 64;
	Out.MaxBlobCount = 4;
	Out.MaxSessionBlobCount = 4;
	Out.MaxSessionPhysicalObjectCount = 4;
	Out.MaxActiveSessions = 2;
	return Out;
}

bool OpenOwner(FAutomationTestBase& Test, FURLabPayloadStore& Store,
	const TCHAR* BootId, const TCHAR* SessionId, FURLabPayloadOwner& OutOwner,
	FURLabPayloadSessionPtr* OutSession = nullptr)
{
	FURLabPayloadSessionPtr Session;
	const FURLabPayloadStoreStatus Status = Store.OpenSession(BootId, SessionId, Session);
	if (!Test.TestTrue(TEXT("payload session opens"), Status.IsOk()))
	{
		Test.AddError(Status.Message);
		return false;
	}
	OutOwner = FURLabPayloadOwner::FromSession(Session);
	if (OutSession) *OutSession = MoveTemp(Session);
	return true;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabPayloadStoreLifecycleTest,
	"URLab.PayloadStore.ImmutableLifecycleAndSessionIsolation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabPayloadStoreLifecycleTest::RunTest(const FString&)
{
	FURLabPayloadStore Store(TEXT("boot-a"), SmallLimits());
	FURLabPayloadSessionPtr InvalidSession;
	TestEqual(TEXT("another boot cannot open a session"),
		Store.OpenSession(TEXT("boot-b"), TEXT("session"), InvalidSession).Code,
		EURLabPayloadStoreError::InvalidIdentity);
	TestEqual(TEXT("empty session identity rejects"),
		Store.OpenSession(TEXT("boot-a"), TEXT(""), InvalidSession).Code,
		EURLabPayloadStoreError::InvalidIdentity);
	FURLabPayloadOwner A;
	FURLabPayloadOwner B;
	if (!OpenOwner(*this, Store, TEXT("boot-a"), TEXT("session-a"), A)
		|| !OpenOwner(*this, Store, TEXT("boot-a"), TEXT("session-b"), B)) return false;
	FURLabPayloadSessionPtr ExcessSession;
	TestEqual(TEXT("active session count is bounded"),
		Store.OpenSession(TEXT("boot-a"), TEXT("session-c"), ExcessSession).Code,
		EURLabPayloadStoreError::LimitExceeded);

	const TArray<uint8> Payload = Bytes({'a', 'b', 'c', 'd', 'e', 'f'});
	const FString Sha = FURLabAssetCache::Sha256Hex(Payload);
	FString BlobA;
	TestTrue(TEXT("uppercase digest is canonicalized"),
		Store.BeginWrite(A, Payload.Num(), Sha.ToUpper(), BlobA).IsOk());
	TestFalse(TEXT("blob ids are not content hashes"), BlobA == Sha);
	TestTrue(TEXT("tail chunk accepted"),
		Store.WriteChunk(A, BlobA, 3, TConstArrayView<uint8>(Payload.GetData() + 3, 3)).IsOk());
	TestEqual(TEXT("conflicting overlap is rejected"),
		Store.WriteChunk(A, BlobA, 4, TConstArrayView<uint8>(Payload.GetData(), 2)).Code,
		EURLabPayloadStoreError::Overlap);
	FURLabStoredPayloadDescriptor DescriptorA;
	FURLabPayloadPin PinA;
	TestEqual(TEXT("partial payload cannot commit"),
		Store.Commit(A, BlobA, DescriptorA, PinA).Code, EURLabPayloadStoreError::Incomplete);
	TestTrue(TEXT("head chunk completes coverage"),
		Store.WriteChunk(A, BlobA, 0, TConstArrayView<uint8>(Payload.GetData(), 3)).IsOk());
	TestTrue(TEXT("complete exact payload commits"),
		Store.Commit(A, BlobA, DescriptorA, PinA).IsOk());
	TestEqual(TEXT("committed digest is canonical lowercase"), DescriptorA.Sha256, Sha);
	TestEqual(TEXT("committed bytes are owned and exact"), PinnedNetworkBytes(PinA), Payload);

	FURLabPayloadPin OtherPin;
	TestEqual(TEXT("another session cannot pin the blob"), Store.Pin(B, BlobA, OtherPin).Code,
		EURLabPayloadStoreError::IdentityMismatch);
	FURLabPayloadOwner WrongBoot = A;
	WrongBoot.BootId = TEXT("boot-b");
	TestEqual(TEXT("read validates boot identity"), Store.Pin(WrongBoot, BlobA, OtherPin).Code,
		EURLabPayloadStoreError::InvalidIdentity);
	FURLabPayloadOwner StringsOnly;
	StringsOnly.BootId = A.BootId;
	StringsOnly.SessionId = A.SessionId;
	TestEqual(TEXT("identity strings without the minted handle have no authority"),
		Store.Pin(StringsOnly, BlobA, OtherPin).Code, EURLabPayloadStoreError::InvalidIdentity);

	FURLabPayloadCopyPin Range;
	TestTrue(TEXT("bounded copied read succeeds"), Store.CopyRange(A, BlobA, 1, 3, Range).IsOk());
	TestEqual(TEXT("copied read has requested bytes"), Range->GetBytes(), Bytes({'b', 'c', 'd'}));
	TestEqual(TEXT("out-of-bounds copied read rejects"),
		Store.CopyRange(A, BlobA, 5, 2, Range).Code, EURLabPayloadStoreError::OutOfBounds);

	FString BlobB;
	TestTrue(TEXT("independent session can stage its own blob"),
		Store.BeginWrite(B, Payload.Num(), Sha, BlobB).IsOk());
	TestFalse(TEXT("independent blobs receive distinct unpredictable ids"), BlobB == BlobA);
	TestTrue(TEXT("independent session writes"), Store.WriteChunk(B, BlobB, 0, Payload).IsOk());
	FURLabStoredPayloadDescriptor DescriptorB;
	FURLabPayloadPin PinB;
	TestTrue(TEXT("independent session commits"), Store.Commit(B, BlobB, DescriptorB, PinB).IsOk());
	FString StagingA;
	TestTrue(TEXT("session can also hold private staging"),
		Store.BeginWrite(A, Payload.Num(), Sha, StagingA).IsOk());
	int32 Removed = 0;
	TestTrue(TEXT("session revoke succeeds"), Store.RevokeSession(A, Removed).IsOk());
	TestEqual(TEXT("revoke removes sealed and staging entries"), Removed, 2);
	TestEqual(TEXT("revoke exposes neither former entry"), Store.GetSessionStats(A.SessionId).SealedCount, 0);
	TestEqual(TEXT("other session remains readable after revoke"), PinnedNetworkBytes(PinB), Payload);
	TestEqual(TEXT("sealed lookup through revoked handle rejects"),
		Store.Pin(A, BlobA, OtherPin).Code, EURLabPayloadStoreError::SessionRevoked);
	TestEqual(TEXT("accepted pin survives session revoke"), PinnedNetworkBytes(PinA), Payload);
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabPayloadStoreValidationTest,
	"URLab.PayloadStore.ChunkBoundsCoverageAndIntegrity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabPayloadStoreValidationTest::RunTest(const FString&)
{
	FURLabPayloadStoreLimits Limits = SmallLimits();
	Limits.MaxChunksPerBlob = 2;
	FURLabPayloadStore Store(TEXT("boot"), Limits);
	FURLabPayloadOwner Owner;
	if (!OpenOwner(*this, Store, TEXT("boot"), TEXT("session"), Owner)) return false;
	const TArray<uint8> Payload = Bytes({0, 1, 2, 3, 4, 5});
	const FString Sha = FURLabAssetCache::Sha256Hex(Payload);
	FString Blob;
	TestEqual(TEXT("non-64-hex expected digest rejects"),
		Store.BeginWrite(Owner, 6, TEXT("not-a-sha256"), Blob).Code,
		EURLabPayloadStoreError::InvalidDeclaration);
	TestTrue(TEXT("bounded staging begins"), Store.BeginWrite(Owner, 6, Sha, Blob).IsOk());
	const TArray<uint8> Oversized = Bytes({0, 1, 2, 3, 4, 5, 6, 7, 8});
	TestEqual(TEXT("oversized chunk rejects"), Store.WriteChunk(Owner, Blob, 0, Oversized).Code,
		EURLabPayloadStoreError::ChunkTooLarge);
	TestEqual(TEXT("negative chunk offset rejects"), Store.WriteChunk(Owner, Blob, -1, Payload).Code,
		EURLabPayloadStoreError::OutOfBounds);
	TestEqual(TEXT("range past declared length rejects"),
		Store.WriteChunk(Owner, Blob, 5, TConstArrayView<uint8>(Payload.GetData(), 2)).Code,
		EURLabPayloadStoreError::OutOfBounds);
	TestEqual(TEXT("empty chunk rejects"),
		Store.WriteChunk(Owner, Blob, 0, TConstArrayView<uint8>()).Code,
		EURLabPayloadStoreError::InvalidChunk);
	TestTrue(TEXT("first disjoint chunk accepted"),
		Store.WriteChunk(Owner, Blob, 0, TConstArrayView<uint8>(Payload.GetData(), 2)).IsOk());
	TestTrue(TEXT("second disjoint chunk accepted"),
		Store.WriteChunk(Owner, Blob, 3, TConstArrayView<uint8>(Payload.GetData() + 3, 1)).IsOk());
	TestEqual(TEXT("interval metadata cap rejects before another write"),
		Store.WriteChunk(Owner, Blob, 4, TConstArrayView<uint8>(Payload.GetData() + 4, 1)).Code,
		EURLabPayloadStoreError::ChunkLimitExceeded);
	FURLabStoredPayloadDescriptor Descriptor;
	FURLabPayloadPin Pin;
	TestEqual(TEXT("gapped coverage remains private"), Store.Commit(Owner, Blob, Descriptor, Pin).Code,
		EURLabPayloadStoreError::Incomplete);
	TestFalse(TEXT("incomplete commit returns no pin"), Pin.IsValid());

	TestTrue(TEXT("gapped staging releases"), Store.Release(Owner, Blob).IsOk());
	const TArray<uint8> Wrong = Bytes({9, 8, 7});
	TestTrue(TEXT("integrity fixture begins"), Store.BeginWrite(Owner, Wrong.Num(), Sha, Blob).IsOk());
	TestTrue(TEXT("integrity fixture fills"), Store.WriteChunk(Owner, Blob, 0, Wrong).IsOk());
	TestEqual(TEXT("hash mismatch rejects commit"), Store.Commit(Owner, Blob, Descriptor, Pin).Code,
		EURLabPayloadStoreError::IntegrityMismatch);
	TestFalse(TEXT("failed integrity returns no bytes"), Pin.IsValid());
	TestEqual(TEXT("failed integrity creates no sealed entry"), Store.GetStats().SealedCount, 0);
	TestEqual(TEXT("failed integrity retires staging"), Store.Pin(Owner, Blob, Pin).Code,
		EURLabPayloadStoreError::NotFound);

	FURLabPayloadStoreLimits ReadLimits = SmallLimits();
	ReadLimits.MaxBlobBytes = 4;
	ReadLimits.MaxChunkBytes = 2;
	ReadLimits.MaxChunksPerBlob = 2;
	ReadLimits.MaxSessionBytes = 6;
	ReadLimits.MaxGlobalBytes = 6;
	ReadLimits.MaxBlobCount = 2;
	ReadLimits.MaxSessionBlobCount = 1;
	ReadLimits.MaxSessionPhysicalObjectCount = 2;
	FURLabPayloadStore ReadStore(TEXT("read-boot"), ReadLimits);
	FURLabPayloadOwner ReadOwner;
	if (!OpenOwner(*this, ReadStore, TEXT("read-boot"), TEXT("read-session"), ReadOwner))
		return false;
	const TArray<uint8> ReadPayload = Bytes({1, 2, 3, 4});
	FString ReadBlob;
	ReadStore.BeginWrite(ReadOwner, 4, FURLabAssetCache::Sha256Hex(ReadPayload), ReadBlob);
	ReadStore.WriteChunk(ReadOwner, ReadBlob, 0,
		TConstArrayView<uint8>(ReadPayload.GetData(), 2));
	ReadStore.WriteChunk(ReadOwner, ReadBlob, 2,
		TConstArrayView<uint8>(ReadPayload.GetData() + 2, 2));
	FURLabStoredPayloadDescriptor ReadDescriptor;
	FURLabPayloadPin ReadPin;
	TestTrue(TEXT("read-limit fixture commits"),
		ReadStore.Commit(ReadOwner, ReadBlob, ReadDescriptor, ReadPin).IsOk());
	FURLabPayloadCopyPin ReadBytes;
	TestTrue(TEXT("read at chunk limit succeeds"),
		ReadStore.CopyRange(ReadOwner, ReadBlob, 0, 2, ReadBytes).IsOk());
	TestEqual(TEXT("read copy owns exact bytes"), ReadBytes->GetBytes(), Bytes({1, 2}));
	TestEqual(TEXT("read beyond chunk limit rejects before copying"),
		ReadStore.CopyRange(ReadOwner, ReadBlob, 0, 3, ReadBytes).Code,
		EURLabPayloadStoreError::ChunkTooLarge);
	TestFalse(TEXT("oversize read leaves output empty"), ReadBytes.IsValid());
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabPayloadStorePinAccountingTest,
	"URLab.PayloadStore.ReleaseKeepsPinnedPhysicalBudget",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabPayloadStorePinAccountingTest::RunTest(const FString&)
{
	FURLabPayloadStoreLimits Limits = SmallLimits();
	Limits.MaxBlobBytes = 8;
	Limits.MaxChunkBytes = 8;
	Limits.MaxSessionBytes = 8;
	Limits.MaxGlobalBytes = 8;
	Limits.MaxBlobCount = 1;
	Limits.MaxSessionBlobCount = 1;
	Limits.MaxSessionPhysicalObjectCount = 1;
	FURLabPayloadStore Store(TEXT("boot"), Limits);
	FURLabPayloadOwner Owner;
	if (!OpenOwner(*this, Store, TEXT("boot"), TEXT("session"), Owner)) return false;
	const TArray<uint8> Payload = Bytes({0, 1, 2, 3, 4, 5, 6, 7});
	const FString Sha = FURLabAssetCache::Sha256Hex(Payload);
	FString Blob;
	TestTrue(TEXT("capacity fixture begins"), Store.BeginWrite(Owner, 8, Sha, Blob).IsOk());
	TestTrue(TEXT("capacity fixture fills"), Store.WriteChunk(Owner, Blob, 0, Payload).IsOk());
	FURLabStoredPayloadDescriptor Descriptor;
	FURLabPayloadPin Pin;
	TestTrue(TEXT("capacity fixture commits"), Store.Commit(Owner, Blob, Descriptor, Pin).IsOk());
	FURLabPayloadPin AcceptedJobPin = Pin;
	Pin.Reset();
	TestTrue(TEXT("store reference releases"), Store.Release(Owner, Blob).IsOk());
	FURLabPayloadPin Probe;
	TestEqual(TEXT("released payload cannot be looked up"),
		Store.Pin(Owner, Blob, Probe).Code, EURLabPayloadStoreError::NotFound);
	TestEqual(TEXT("accepted-job pin remains physically charged"),
		Store.GetStats().PhysicalBytes, 8ll);
	TestEqual(TEXT("accepted-job pin retains its physical blob count"),
		Store.GetStats().PhysicalBlobCount, 1);
	FString BlockedBlob;
	TestEqual(TEXT("pinned released bytes cannot bypass the budget"),
		Store.BeginWrite(Owner, 8, Sha, BlockedBlob).Code,
		EURLabPayloadStoreError::LimitExceeded);
	TestEqual(TEXT("accepted-job pin remains readable"), PinnedNetworkBytes(AcceptedJobPin), Payload);
	AcceptedJobPin.Reset();
	TestEqual(TEXT("last accepted-job pin returns physical budget"),
		Store.GetStats().PhysicalBytes, 0ll);
	TestEqual(TEXT("last accepted-job pin returns blob-count budget"),
		Store.GetStats().PhysicalBlobCount, 0);
	TestTrue(TEXT("reservation succeeds only after the pin dies"),
		Store.BeginWrite(Owner, 8, Sha, BlockedBlob).IsOk());
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabPayloadStoreRevokeGenerationTest,
	"URLab.PayloadStore.RevokeRejectsStaleHandleAndPreservesPins",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabPayloadStoreRevokeGenerationTest::RunTest(const FString&)
{
	FURLabPayloadStoreLimits Limits = SmallLimits();
	Limits.MaxBlobBytes = 8;
	Limits.MaxChunkBytes = 8;
	Limits.MaxSessionBytes = 8;
	Limits.MaxGlobalBytes = 8;
	Limits.MaxBlobCount = 1;
	Limits.MaxSessionBlobCount = 1;
	Limits.MaxSessionPhysicalObjectCount = 1;
	Limits.MaxActiveSessions = 1;
	FURLabPayloadStore Store(TEXT("boot"), Limits);
	FURLabPayloadOwner OldOwner;
	FURLabPayloadSessionPtr OldSession;
	if (!OpenOwner(*this, Store, TEXT("boot"), TEXT("same-id"), OldOwner, &OldSession)) return false;
	const TArray<uint8> Payload = Bytes({0, 1, 2, 3, 4, 5, 6, 7});
	const FString Sha = FURLabAssetCache::Sha256Hex(Payload);
	FString OldBlob;
	Store.BeginWrite(OldOwner, 8, Sha, OldBlob);
	Store.WriteChunk(OldOwner, OldBlob, 0, Payload);
	FURLabStoredPayloadDescriptor OldDescriptor;
	FURLabPayloadPin OldPin;
	TestTrue(TEXT("old session commits"), Store.Commit(OldOwner, OldBlob, OldDescriptor, OldPin).IsOk());
	int32 Removed = 0;
	TestTrue(TEXT("session revoke succeeds"), Store.RevokeSession(OldOwner, Removed).IsOk());
	TestEqual(TEXT("revoke removed sealed entry"), Removed, 1);
	TestFalse(TEXT("revoked handle is inactive"), OldSession->IsActive());
	TestEqual(TEXT("old pinned bytes remain exact"), PinnedNetworkBytes(OldPin), Payload);
	TestEqual(TEXT("old pin remains physically charged"), Store.GetStats().PhysicalBytes, 8ll);

	FURLabPayloadOwner NewOwner;
	FURLabPayloadSessionPtr NewSession;
	TestTrue(TEXT("same textual session can explicitly reopen"),
		OpenOwner(*this, Store, TEXT("boot"), TEXT("same-id"), NewOwner, &NewSession));
	TestFalse(TEXT("reopen mints a distinct handle"), NewSession == OldSession);
	FString NewBlob;
	TestEqual(TEXT("old pin prevents budget reuse"), Store.BeginWrite(NewOwner, 8, Sha, NewBlob).Code,
		EURLabPayloadStoreError::LimitExceeded);
	TestEqual(TEXT("stale handle cannot revoke the new session"),
		Store.RevokeSession(OldOwner, Removed).Code, EURLabPayloadStoreError::SessionRevoked);
	TestTrue(TEXT("new handle remains active"), NewSession->IsActive());

	OldPin.Reset();
	TestEqual(TEXT("last pin releases physical bytes"), Store.GetStats().PhysicalBytes, 0ll);
	TestTrue(TEXT("new session can reserve after old pin dies"),
		Store.BeginWrite(NewOwner, 8, Sha, NewBlob).IsOk());
	TestEqual(TEXT("stale handle cannot write newly opened session blob"),
		Store.WriteChunk(OldOwner, NewBlob, 0, Payload).Code,
		EURLabPayloadStoreError::SessionRevoked);
	Store.WriteChunk(NewOwner, NewBlob, 0, Payload);
	FURLabStoredPayloadDescriptor NewDescriptor;
	FURLabPayloadPin NewPin;
	Store.Commit(NewOwner, NewBlob, NewDescriptor, NewPin);
	FURLabPayloadPin Probe;
	TestEqual(TEXT("stale handle cannot read newly opened session blob"),
		Store.Pin(OldOwner, NewBlob, Probe).Code, EURLabPayloadStoreError::SessionRevoked);

	FURLabPayloadSessionPtr LifetimeSession;
	FURLabPayloadOwner LifetimeOwner;
	FURLabPayloadPin LifetimePin;
	{
		TUniquePtr<FURLabPayloadStore> LifetimeStore =
			MakeUnique<FURLabPayloadStore>(TEXT("other-boot"), SmallLimits());
		OpenOwner(*this, *LifetimeStore, TEXT("other-boot"), TEXT("lifetime"),
			LifetimeOwner, &LifetimeSession);
		const TArray<uint8> Tiny = Bytes({7});
		FString TinyBlob;
		LifetimeStore->BeginWrite(LifetimeOwner, 1, FURLabAssetCache::Sha256Hex(Tiny), TinyBlob);
		LifetimeStore->WriteChunk(LifetimeOwner, TinyBlob, 0, Tiny);
		FURLabStoredPayloadDescriptor TinyDescriptor;
		LifetimeStore->Commit(LifetimeOwner, TinyBlob, TinyDescriptor, LifetimePin);
	}
	TestFalse(TEXT("store destruction deactivates its live handles"), LifetimeSession->IsActive());
	TestEqual(TEXT("pin survives store destruction"), PinnedNetworkBytes(LifetimePin), Bytes({7}));
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabPayloadStoreExpiryAndAckTest,
	"URLab.PayloadStore.ExpiryTombstonesAndAcknowledgement",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabPayloadStoreExpiryAndAckTest::RunTest(const FString&)
{
	double Now = 10.0;
	FURLabPayloadStoreLimits Limits = SmallLimits();
	Limits.StagingTtlSeconds = 5.0;
	Limits.SealedTtlSeconds = 10.0;
	Limits.ExpiredTombstoneTtlSeconds = 100.0;
	FURLabPayloadStore Store(TEXT("boot"), Limits, [&Now] { return Now; });
	FURLabPayloadOwner Owner;
	if (!OpenOwner(*this, Store, TEXT("boot"), TEXT("session"), Owner)) return false;
	const TArray<uint8> Payload = Bytes({1, 2, 3});
	const FString Sha = FURLabAssetCache::Sha256Hex(Payload);

	FString Staging;
	TestTrue(TEXT("staging fixture begins"),
		Store.BeginWrite(Owner, Payload.Num(), Sha, Staging).IsOk());
	Now = 15.0;
	TestEqual(TEXT("operation at staging deadline reports expired"),
		Store.WriteChunk(Owner, Staging, 0, Payload).Code, EURLabPayloadStoreError::Expired);
	TestEqual(TEXT("expired staging releases physical bytes"),
		Store.GetStats().PhysicalBytes, 0ll);

	Now = 20.0;
	FString Blob;
	Store.BeginWrite(Owner, Payload.Num(), Sha, Blob);
	Store.WriteChunk(Owner, Blob, 0, Payload);
	FURLabStoredPayloadDescriptor Descriptor;
	FURLabPayloadPin AcceptedPin;
	TestTrue(TEXT("sealed fixture commits"),
		Store.Commit(Owner, Blob, Descriptor, AcceptedPin).IsOk());
	TestEqual(TEXT("commit reports the full sealed TTL"), Descriptor.ExpiresInSeconds, 10.0);

	Now = 24.0;
	FURLabStoredPayloadDescriptor Remaining;
	TestTrue(TEXT("current descriptor remains available"),
		Store.GetDescriptor(Owner, Blob, Remaining).IsOk());
	TestEqual(TEXT("descriptor reports truthful remaining TTL"), Remaining.ExpiresInSeconds, 6.0);
	bool bNewlyAcknowledged = true;
	TestEqual(TEXT("wrong acknowledgement hash rejects"),
		Store.Acknowledge(Owner, Blob, FString::ChrN(64, TEXT('0')),
			bNewlyAcknowledged).Code,
		EURLabPayloadStoreError::IntegrityMismatch);
	TestFalse(TEXT("failed acknowledgement does not set marker"), bNewlyAcknowledged);
	TestTrue(TEXT("matching acknowledgement succeeds"),
		Store.Acknowledge(Owner, Blob, Sha.ToUpper(), bNewlyAcknowledged).IsOk());
	TestTrue(TEXT("first acknowledgement reports a change"), bNewlyAcknowledged);
	TestTrue(TEXT("repeated acknowledgement succeeds"),
		Store.Acknowledge(Owner, Blob, Sha, bNewlyAcknowledged).IsOk());
	TestFalse(TEXT("repeated acknowledgement is unchanged"), bNewlyAcknowledged);
	TestEqual(TEXT("acknowledgement marker is observable"),
		Store.GetStats().AcknowledgedCount, 1);

	Now = 29.0;
	Store.GetDescriptor(Owner, Blob, Remaining);
	TestEqual(TEXT("read and acknowledgement did not refresh TTL"),
		Remaining.ExpiresInSeconds, 1.0);
	Now = 30.0;
	FURLabPayloadPin Probe;
	TestEqual(TEXT("sealed deadline reports expired"), Store.Pin(Owner, Blob, Probe).Code,
		EURLabPayloadStoreError::Expired);
	TestEqual(TEXT("expiry removes sealed lookup"), Store.GetStats().SealedCount, 0);
	TestEqual(TEXT("bounded tombstone records recent expiry"),
		Store.GetStats().ExpiredTombstoneCount, 2);
	TestEqual(TEXT("accepted pin keeps expired bytes charged"),
		Store.GetStats().PhysicalBytes, static_cast<int64>(Payload.Num()));
	AcceptedPin.Reset();
	TestEqual(TEXT("last pin returns expired physical quota"),
		Store.GetStats().PhysicalBytes, 0ll);

	for (int32 Index = 0; Index < 3; ++Index)
	{
		Now = 31.0 + Index * 6.0;
		FString Extra;
		Store.BeginWrite(Owner, Payload.Num(), Sha, Extra);
		Now += 5.0;
		FURLabPayloadSweepResult ExtraSweep;
		Store.SweepExpired(ExtraSweep);
		TestEqual(TEXT("additional staging expiry is swept"), ExtraSweep.StagingRemoved, 1);
	}
	TestEqual(TEXT("tombstone count remains globally bounded"),
		Store.GetStats().ExpiredTombstoneCount, Limits.MaxBlobCount);
	TestEqual(TEXT("oldest evicted tombstone degrades to not-found"),
		Store.WriteChunk(Owner, Staging, 0, Payload).Code, EURLabPayloadStoreError::NotFound);

	Now = 200.0;
	FURLabPayloadSweepResult Sweep;
	TestTrue(TEXT("explicit sweep succeeds"), Store.SweepExpired(Sweep).IsOk());
	TestEqual(TEXT("aged tombstones are forgotten"), Sweep.TombstonesForgotten, 4);
	TestEqual(TEXT("forgotten expiry is bounded to not-found"),
		Store.Pin(Owner, Blob, Probe).Code, EURLabPayloadStoreError::NotFound);
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabPayloadStoreProducerAndCopyBudgetTest,
	"URLab.PayloadStore.DirectProducerAndPhysicalCopyBudget",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabPayloadStoreProducerAndCopyBudgetTest::RunTest(const FString&)
{
	double Now = 0.0;
	FURLabPayloadStoreLimits Limits = SmallLimits();
	Limits.MaxBlobBytes = 16;
	Limits.MaxChunkBytes = 8;
	Limits.MaxSessionBytes = 32;
	Limits.MaxGlobalBytes = 32;
	Limits.MaxBlobCount = 4;
	Limits.MaxSessionBlobCount = 2;
	Limits.MaxSessionPhysicalObjectCount = 4;
	Limits.StagingTtlSeconds = 5.0;
	FURLabPayloadStore Store(TEXT("boot"), Limits, [&Now] { return Now; });
	FURLabPayloadOwner Owner;
	if (!OpenOwner(*this, Store, TEXT("boot"), TEXT("session"), Owner)) return false;
	const TArray<uint8> Payload = Bytes({0, 1, 2, 3, 4, 5, 6, 7});

	FURLabPayloadProducerPtr Producer;
	TestTrue(TEXT("producer capacity is admitted before allocation"),
		Store.BeginProduce(Owner, Payload.Num(), Producer).IsOk());
	TestEqual(TEXT("producer admission charges declared bytes"),
		Store.GetStats().PhysicalBytes, static_cast<int64>(Payload.Num()));
	TArray<uint8> SlackPayload = Payload;
	SlackPayload.Reserve(1024);
	FURLabStoredPayloadDescriptor Descriptor;
	FURLabPayloadPin Pin;
	TestEqual(TEXT("oversized caller capacity rejects without consumption"),
		Store.SealProduced(Owner, Producer, MoveTemp(SlackPayload), Descriptor, Pin).Code,
		EURLabPayloadStoreError::InvalidDeclaration);
	TestEqual(TEXT("rejected producer buffer remains caller-owned"), SlackPayload, Payload);
	TestEqual(TEXT("rejected producer reservation remains charged"),
		Store.GetStats().PhysicalBytes, static_cast<int64>(Payload.Num()));

	Now = 5.0;
	FURLabPayloadSweepResult Sweep;
	Store.SweepExpired(Sweep);
	TestEqual(TEXT("producer staging expires"), Sweep.StagingRemoved, 1);
	TestEqual(TEXT("caller-held producer keeps expired reservation charged"),
		Store.GetStats().PhysicalBytes, static_cast<int64>(Payload.Num()));
	TArray<uint8> ExpiredBytes;
	ExpiredBytes.Reserve(Payload.Num());
	ExpiredBytes.Append(Payload);
	TestEqual(TEXT("expired producer reports exact expiry"),
		Store.SealProduced(Owner, Producer, MoveTemp(ExpiredBytes), Descriptor, Pin).Code,
		EURLabPayloadStoreError::Expired);
	TestEqual(TEXT("expired seal error leaves caller bytes intact"), ExpiredBytes, Payload);
	Producer.Reset();
	TestEqual(TEXT("caller release returns expired producer quota"),
		Store.GetStats().PhysicalBytes, 0ll);

	Now = 6.0;
	TestTrue(TEXT("second producer admission succeeds"),
		Store.BeginProduce(Owner, Payload.Num(), Producer).IsOk());
	TArray<uint8> Produced;
	Produced.Reserve(Payload.Num());
	Produced.Append(Payload);
	TestTrue(TEXT("exact producer bytes seal without copying"),
		Store.SealProduced(Owner, Producer, MoveTemp(Produced), Descriptor, Pin).IsOk());
	TestEqual(TEXT("successful seal moved caller bytes"), Produced.Num(), 0);
	TestEqual(TEXT("producer descriptor hashes exact bytes"),
		Descriptor.Sha256, FURLabAssetCache::Sha256Hex(Payload));

	FURLabPayloadCopyPin Copy;
	TestTrue(TEXT("copy reserves before allocating"),
		Store.CopyRange(Owner, Descriptor.BlobId, 2, 4, Copy).IsOk());
	TestEqual(TEXT("copy contains exact range"), Copy->GetBytes(), Bytes({2, 3, 4, 5}));
	TestEqual(TEXT("sealed bytes plus copy are both charged"),
		Store.GetStats().PhysicalBytes, 12ll);
	FURLabPayloadPhysicalReservationPtr FutureMapping;
	TestTrue(TEXT("future mapped copy uses the same physical admission boundary"),
		Store.ReservePhysicalCapacity(Owner, 4, FutureMapping).IsOk());
	TestEqual(TEXT("independent physical reservation is charged"),
		Store.GetStats().PhysicalBytes, 16ll);

	TestTrue(TEXT("release removes lookup only"),
		Store.Release(Owner, Descriptor.BlobId).IsOk());
	int32 Removed = -1;
	TestTrue(TEXT("session revoke is independent of retained physical owners"),
		Store.RevokeSession(Owner, Removed).IsOk());
	TestEqual(TEXT("no live lookup remained at revoke"), Removed, 0);
	TestEqual(TEXT("pins, copies and mapping reservations survive revoke"),
		Store.GetStats().PhysicalBytes, 16ll);
	FutureMapping.Reset();
	Copy.Reset();
	Pin.Reset();
	TestEqual(TEXT("all physical owners release the complete quota"),
		Store.GetStats().PhysicalBytes, 0ll);
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabPayloadStoreLimitsAndClockTest,
	"URLab.PayloadStore.SessionObjectLimitAndInvalidClock",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabPayloadStoreLimitsAndClockTest::RunTest(const FString&)
{
	double Now = 0.0;
	bool bInvalidClock = false;
	FURLabPayloadStoreLimits Limits = SmallLimits();
	Limits.MaxBlobBytes = 4;
	Limits.MaxChunkBytes = 4;
	Limits.MaxSessionBytes = 16;
	Limits.MaxGlobalBytes = 16;
	Limits.MaxBlobCount = 4;
	Limits.MaxSessionBlobCount = 2;
	Limits.MaxSessionPhysicalObjectCount = 4;
	FURLabPayloadStore Store(TEXT("boot"), Limits, [&Now, &bInvalidClock]
	{
		return bInvalidClock ? std::numeric_limits<double>::quiet_NaN() : Now;
	});
	FURLabPayloadOwner Owner;
	if (!OpenOwner(*this, Store, TEXT("boot"), TEXT("session"), Owner)) return false;
	const TArray<uint8> Tiny = Bytes({7});
	const FString Sha = FURLabAssetCache::Sha256Hex(Tiny);
	FString First;
	FString Second;
	FString Excess;
	TestTrue(TEXT("first logical object begins"), Store.BeginWrite(Owner, 1, Sha, First).IsOk());
	TestTrue(TEXT("second logical object begins"), Store.BeginWrite(Owner, 1, Sha, Second).IsOk());
	FURLabPayloadPhysicalReservationPtr ExtraPhysical;
	TestTrue(TEXT("physical-copy count is separate from live lookup count"),
		Store.ReservePhysicalCapacity(Owner, 1, ExtraPhysical).IsOk());
	TestEqual(TEXT("per-session logical object limit rejects third lookup"),
		Store.BeginWrite(Owner, 1, Sha, Excess).Code, EURLabPayloadStoreError::LimitExceeded);

	bInvalidClock = true;
	FURLabPayloadSweepResult Sweep;
	TestEqual(TEXT("non-finite clock rejects sweep"), Store.SweepExpired(Sweep).Code,
		EURLabPayloadStoreError::InvalidClock);
	TestEqual(TEXT("invalid clock rejects mutation"), Store.Release(Owner, First).Code,
		EURLabPayloadStoreError::InvalidClock);
	TestEqual(TEXT("invalid clock left both staging entries intact"),
		Store.GetStats().StagingCount, 2);
	bInvalidClock = false;
	TestTrue(TEXT("valid clock permits release"), Store.Release(Owner, First).IsOk());
	TestTrue(TEXT("second staging releases"), Store.Release(Owner, Second).IsOk());
	ExtraPhysical.Reset();
	TestEqual(TEXT("all test reservations release"), Store.GetStats().PhysicalBytes, 0ll);
	return !HasAnyErrors();
}

#endif
