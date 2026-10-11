#pragma once

#include "CoreMinimal.h"
#include "Bridge/RenderCapturePayloadAdapter.h"
#include "Containers/Ticker.h"
#include "Workflows/RenderOwnerAuthority.h"

#include <atomic>

class UURLabRenderServerWorldSubsystem;
struct FURLabRenderServerCaptureHandle;

/** Shared exact dispatcher binding. UObject access is game-thread-only. */
enum class EURLabRenderMaterializationState : uint8
{
	Pending,
	Succeeded,
	Failed,
};

struct URLAB_API FURLabRenderServiceCaptureView
{
	FURLabRenderCaptureJobSnapshot Native;
	FRenderFrameIdentity SubmittedFrame;
	EURLabRenderMaterializationState MaterializationState =
		EURLabRenderMaterializationState::Pending;
	FURLabCapturePayloadMaterialization Materialization;
};

class URLAB_API FURLabRenderServiceBinding
	: public TSharedFromThis<FURLabRenderServiceBinding, ESPMode::ThreadSafe>
{
public:
	~FURLabRenderServiceBinding();
	static TSharedPtr<FURLabRenderServiceBinding, ESPMode::ThreadSafe> Create(
		UURLabRenderServerWorldSubsystem& Workflow, FString& OutError);

	bool IsActive() const { return bActive.load(std::memory_order_acquire); }
	void Deactivate() { bActive.store(false, std::memory_order_release); }
	UURLabRenderServerWorldSubsystem* ResolveWorkflow() const;
	UURLabRenderServerWorldSubsystem* ResolveWorkflowForRetirement() const;
	bool CanAcceptCapture(const FURLabRenderOwnerSessionScopePtr& Scope,
		FString& OutError) const;
	bool GetRetentionContext(const FURLabRenderOwnerSessionScopePtr& Scope,
		const FURLabRenderOwnerProof& Proof, FGuid& OutContext, FString& OutError);
	void ResetRetentionContext(const FURLabRenderOwnerSessionScopePtr& Scope,
		const FURLabRenderOwnerProof& Proof);

	bool AddCapture(const FURLabRenderOwnerSessionScopePtr& Scope,
		const FURLabRenderOwnerProof& Proof,
		FURLabRenderServerCaptureHandle&& Handle,
		TSharedPtr<FURLabCapturePayloadAdmission, ESPMode::ThreadSafe>&& Admission,
		FString& OutJobId, FString& OutError);
	bool PollCapture(const FURLabRenderOwnerSessionScopePtr& Scope,
		const FURLabRenderOwnerProof& Proof, const FString& JobId,
		FURLabRenderServiceCaptureView& OutView, FString& OutError);
	bool CancelCapture(const FURLabRenderOwnerSessionScopePtr& Scope,
		const FURLabRenderOwnerProof& Proof, const FString& JobId,
		bool& OutCancelled, FString& OutError);
	bool ReleaseCapture(const FURLabRenderOwnerSessionScopePtr& Scope,
		const FURLabRenderOwnerProof& Proof, const FString& JobId, FString& OutError);
	FURLabRenderOwnerLeaseResult RetireScope(
		const FURLabRenderOwnerSessionScopePtr& Scope);
	void RetireAttachment(const FURLabRenderOwnerSessionScopePtr& Scope,
		uint64 Generation, const FString& Token);
#if WITH_DEV_AUTOMATION_TESTS
	int32 GetRetainedCaptureCountForTest() const
	{
		check(IsInGameThread());
		return Captures.Num();
	}
#endif

private:
	explicit FURLabRenderServiceBinding(UURLabRenderServerWorldSubsystem& InWorkflow);
	struct FCaptureRecord;
	struct FRetentionContext
	{
		FURLabRenderOwnerSessionScopePtr Scope;
		uint64 Generation = 0;
		FString Token;
		FGuid Id;
	};
	bool ValidateRecord(const FURLabRenderOwnerSessionScopePtr& Scope,
		const FURLabRenderOwnerProof& Proof, const FString& JobId,
		TSharedPtr<FCaptureRecord, ESPMode::ThreadSafe>& OutRecord, FString& OutError);
	bool Pump(float DeltaSeconds);
	bool PollRecord(const FString& JobId,
		const TSharedPtr<FCaptureRecord, ESPMode::ThreadSafe>& Record,
		FURLabRenderCaptureJobSnapshot& OutSnapshot,
		FString& OutError);
	void EnsurePump();
	void StopPump();
	TWeakObjectPtr<UURLabRenderServerWorldSubsystem> Workflow;
	TMap<FString, TSharedPtr<FCaptureRecord, ESPMode::ThreadSafe>> Captures;
	TArray<FRetentionContext> RetentionContexts;
	FTSTicker::FDelegateHandle PumpHandle;
	std::atomic<bool> bActive{true};
};
