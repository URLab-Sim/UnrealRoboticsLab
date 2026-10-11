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
};
