// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "Render/RenderFrame.h"
#include "Render/RenderScene.h"
#include "RenderWorldSubsystem.generated.h"

class AURLabRenderScene;
class URenderCameraSubsystem;
class UURLabRenderWorldSubsystem;

/** Internal identity for one producer registered with one world coordinator lifetime. */
struct URLAB_API FURLabRenderProducerRegistration
{
public:
	bool IsValid() const;

private:
	friend class UURLabRenderWorldSubsystem;
	friend class URenderCameraSubsystem;
	TWeakObjectPtr<UURLabRenderWorldSubsystem> Coordinator;
	FGuid WorldLifetime;
	uint64 ProducerSerial = 0;
};

/** Opaque identity for one hidden scene fully staged by the coordinator. */
struct URLAB_API FURLabRenderSceneCandidate
{
public:
	bool IsValid() const;

private:
	friend class UURLabRenderWorldSubsystem;
	TWeakObjectPtr<UURLabRenderWorldSubsystem> Coordinator;
	FGuid WorldLifetime;
	uint64 ProducerSerial = 0;
	uint64 CandidateSerial = 0;
};

struct URLAB_API FURLabRenderWorldCommitResult
{
	bool bCommitted = false;
	uint64 SceneGeneration = 0;
	FString SceneRevision;
	FString Error;
};

/** Opaque ownership of one exact job presentation on the active scene. */
struct URLAB_API FURLabRenderPresentationHold
{
public:
	bool IsValid() const;

private:
	friend class UURLabRenderWorldSubsystem;
	TWeakObjectPtr<UURLabRenderWorldSubsystem> Coordinator;
	FGuid WorldLifetime;
	uint64 ProducerSerial = 0;
	uint64 SceneGeneration = 0;
	FString SceneRevision;
	uint64 HoldSerial = 0;
};

enum class EURLabRenderFramePublication : uint8
{
	Applied,
	Coalesced,
};

struct URLAB_API FURLabRenderFramePublicationResult
{
	bool bAccepted = false;
	EURLabRenderFramePublication Disposition = EURLabRenderFramePublication::Applied;
	FString Error;
};

/**
 * Physics- and transport-independent owner of one world's neutral render scene.
 * Registration is an internal producer lifetime guard, not network authority.
 */
UCLASS()
class URLAB_API UURLabRenderWorldSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()

public:
	/** Register the sole producer. Creates no actor or placeholder scene. */
	bool RegisterProducer(FURLabRenderProducerRegistration& OutRegistration,
		FString& OutError);
	bool IsRegistrationCurrent(const FURLabRenderProducerRegistration& Registration) const;

	/**
	 * Build a complete hidden actor owned by this subsystem. ExpectedCurrentRevision is
	 * empty only for an initial scene. Owned scene/resources are consumed on entry.
	 */
	bool StageScene(const FURLabRenderProducerRegistration& Registration,
		const FString& ExpectedCurrentRevision, FRenderSceneInstallPayload&& Payload,
		const FRenderSceneInstallOptions& Options, FURLabRenderSceneCandidate& OutCandidate,
		FString& OutError);

	/** Fallible precommit frame finalization on the still-hidden exact candidate. */
	bool ApplyCandidateFrame(const FURLabRenderProducerRegistration& Registration,
		const FURLabRenderSceneCandidate& Candidate, const FRenderFrame& Frame,
		FString& OutError);

	/**
	 * Validate the exact transaction, run a short synchronous related-state commit,
	 * then perform the infallible pointer/visibility swap. The callback must not yield
	 * or dispatch asynchronously, and must not destroy actors while related-state locks
	 * are held. It may call ApplyCandidateFrame for this candidate; all other coordinator
	 * mutations are rejected until it returns. Actor retirement occurs after it returns.
	 */
	FURLabRenderWorldCommitResult CommitCandidate(
		const FURLabRenderProducerRegistration& Registration,
		const FURLabRenderSceneCandidate& Candidate,
		TFunctionRef<bool(FString& OutError)> CommitRelatedState);
	/** Commit with no related state. */
	FURLabRenderWorldCommitResult CommitCandidate(
		const FURLabRenderProducerRegistration& Registration,
		const FURLabRenderSceneCandidate& Candidate);

	/** Apply now, or retain the latest authoritative frame while an exact job holds presentation. */
	FURLabRenderFramePublicationResult PublishAuthoritativeFrame(
		const FURLabRenderProducerRegistration& Registration,
		const FString& ExpectedSceneRevision, const FRenderFrame& Frame);

	/** Acquire the sole exact-job presentation hold and apply its owned frame. */
	bool AcquirePresentationHold(const FURLabRenderProducerRegistration& Registration,
		const FString& ExpectedSceneRevision, const FRenderFrame& JobFrame,
		FURLabRenderPresentationHold& OutHold, FString& OutError);
	/** Restore the newest coalesced authoritative frame (or the acquisition baseline). */
	bool ReleasePresentationHold(const FURLabRenderProducerRegistration& Registration,
		const FURLabRenderPresentationHold& Hold, FString& OutError);

	/** Compatibility surface for local callers; use PublishAuthoritativeFrame for disposition. */
	bool ApplyFrame(const FURLabRenderProducerRegistration& Registration,
		const FString& ExpectedSceneRevision, const FRenderFrame& Frame, FString& OutError);
	bool DiscardCandidate(const FURLabRenderProducerRegistration& Registration,
		const FURLabRenderSceneCandidate& Candidate, FString& OutError);

	/** Exact release. A stale copy cannot release a successor registration. */
	bool ReleaseProducer(const FURLabRenderProducerRegistration& Registration,
		FString& OutError);

	AURLabRenderScene* ResolveActiveScene(
		const FURLabRenderProducerRegistration& Registration) const;
	FString ResolveActiveSceneRevision(
		const FURLabRenderProducerRegistration& Registration) const;
	uint64 ResolveActiveSceneGeneration(
		const FURLabRenderProducerRegistration& Registration) const;

	virtual bool DoesSupportWorldType(EWorldType::Type WorldType) const override;
	virtual void Deinitialize() override;

private:
	friend class URenderCameraSubsystem;
	enum class EMutationPhase : uint8
	{
		None,
		Registering,
		Staging,
		CandidateFrame,
		PrecommitCancellation,
		RelatedCommit,
		CommitRetirement,
		ActiveFrame,
		PresentationHold,
		Discarding,
		Releasing,
		Shutdown,
	};
	static constexpr int32 MaxStagedCandidates = 2;
	class FMutationScope
	{
	public:
		FMutationScope(UURLabRenderWorldSubsystem& InOwner, EMutationPhase Phase);
		~FMutationScope();
	private:
		UURLabRenderWorldSubsystem& Owner;
		EMutationPhase PreviousPhase;
	};

	struct FStagedScene
	{
		TWeakObjectPtr<AURLabRenderScene> Actor;
		FString ExpectedCurrentRevision;
		FString SceneRevision;
		uint64 ProposedGeneration = 0;
		bool bInitiallyVisible = true;
	};

	bool ValidateRegistration(const FURLabRenderProducerRegistration& Registration,
		FString& OutError) const;
	bool ValidateActiveFrame(const FString& ExpectedSceneRevision,
		const FRenderFrame& Frame, FString& OutError) const;
	bool ValidatePresentationHold(const FURLabRenderProducerRegistration& Registration,
		const FURLabRenderPresentationHold& Hold, FString& OutError) const;
	bool CancelSceneCaptureJobs(bool bRestorePresentation, FString& OutError);
	void CancelPresentationHoldForRetirement();
	FStagedScene* FindCandidate(const FURLabRenderProducerRegistration& Registration,
		const FURLabRenderSceneCandidate& Candidate, FString& OutError);
	void DestroyCandidate(uint64 CandidateSerial);
	void DestroyAllCandidates();
	void ShutdownInternal();

	FGuid WorldLifetime;
	uint64 CurrentProducerSerial = 0;
	uint64 NextProducerSerial = 1;
	uint64 NextCandidateSerial = 1;
	uint64 NextHoldSerial = 1;
	uint64 ActiveSceneGeneration = 0;
	uint64 TransactionCandidateSerial = 0;
	EMutationPhase MutationPhase = EMutationPhase::None;
	bool bShutdownRequested = false;
	bool bDeinitialized = false;
	bool bSuperDeinitialized = false;
	TMap<uint64, FStagedScene> Candidates;

	struct FActivePresentationHold
	{
		uint64 HoldSerial = 0;
		uint64 ProducerSerial = 0;
		uint64 SceneGeneration = 0;
		FString SceneRevision;
		FRenderFrame SavedAuthoritativeFrame;
		TOptional<FRenderFrame> CoalescedAuthoritativeFrame;
	};
	TOptional<FActivePresentationHold> PresentationHold;

	UPROPERTY(Transient)
	TObjectPtr<AURLabRenderScene> ActiveScene = nullptr;
	UPROPERTY(Transient)
	TArray<TObjectPtr<AURLabRenderScene>> StagedSceneActors;
};
