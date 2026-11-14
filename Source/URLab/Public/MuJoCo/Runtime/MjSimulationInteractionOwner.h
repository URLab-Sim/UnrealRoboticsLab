#pragma once

#include "CoreMinimal.h"
#include "Workflows/InteractionInbox.h"

class FMjSimulationRuntime;
struct FMjSimulationControlHandle;
struct FURLabPoseStreamDescriptor;
struct mjModel_;
struct mjData_;

/** Optional simulation-owner implementation of the generic interaction inbox. */
class URLAB_API FMjSimulationInteractionOwner
	: public IURLabInteractionOwnerHandler
	, public TSharedFromThis<FMjSimulationInteractionOwner, ESPMode::ThreadSafe>
{
public:
	FMjSimulationInteractionOwner();
	~FMjSimulationInteractionOwner();

	/** Owner-thread source lifecycle. The identity is copied from the pose producer. */
	bool BindSource(const FURLabPoseStreamDescriptor& Descriptor, FString& OutError);
	void RevokeSource();
	bool IssueGrant(FMjSimulationRuntime& Runtime,
		const FMjSimulationControlHandle& Control,
		const FURLabInteractionSourceIdentity& ExpectedSource,
		const FString& ViewerBootId,
		double TtlSeconds,
		FURLabInteractionGrant& OutGrant, FString& OutError);
	void RevokeGrant();
	bool RevokeGrant(FMjSimulationRuntime& Runtime,
		const FMjSimulationControlHandle& Control,
		const FURLabInteractionSourceIdentity& ExpectedSource,
		const FString& ExactToken, FString& OutError);
	FURLabInteractionAdmissionResult Enqueue(const FURLabInteractionWrenchRequest& Request,
		TSharedPtr<IURLabInteractionCompletionSink, ESPMode::ThreadSafe> CompletionSink);
	int32 NumHolds() const;
	int32 NumGrants() const;
	/**
	 * Optional live input permission. While it returns false no grant can be
	 * issued, and every grant (with its held wrenches) is revoked at the next step.
	 */
	void SetInputPermission(TFunction<bool()> Predicate);

	virtual bool ValidateTarget(int32 BodyIndex, FString& OutError) override;
	virtual bool ApplyNextStepContribution(const FURLabInteractionWrenchRequest& Request,
		uint64 ApplicationId, FString& OutError) override;

private:
	friend class FMjSimulationRuntime;
	struct FImpl;
	TUniquePtr<FImpl> Impl;

	void BeforeStepLocked(const FURLabInteractionOwnerContext& Context,
		mjModel_* Model, mjData_* Data);
	void AfterStepLocked(const FURLabInteractionOwnerContext& Context,
		uint64 FrameId, double SourceTime);
	void AbortStepLocked();
	void OverlayLastStepWrenchesLocked(const mjModel_* Model, mjData_* Scratch) const;
};
