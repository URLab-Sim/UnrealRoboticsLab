#pragma once

#include "CoreMinimal.h"
#include "Workflows/RenderOwnerAuthority.h"
#include <atomic>

class UURLabViewerWorldSubsystem;

/** Thread-safe admission handle; UObject access remains game-thread-only. */
class URLAB_API FURLabViewerServiceBinding
	: public TSharedFromThis<FURLabViewerServiceBinding, ESPMode::ThreadSafe>
{
public:
	static TSharedPtr<FURLabViewerServiceBinding, ESPMode::ThreadSafe> Create(
		UURLabViewerWorldSubsystem& Workflow, FString& OutError);
	bool IsActive() const { return bActive.load(std::memory_order_acquire); }
	void Deactivate() { bActive.store(false,std::memory_order_release); }
	UURLabViewerWorldSubsystem* ResolveWorkflow() const;
	UURLabViewerWorldSubsystem* ResolveWorkflowForRetirement() const;
	FURLabRenderOwnerLeaseResult RetireScope(const FURLabRenderOwnerSessionScopePtr& Scope);

private:
	explicit FURLabViewerServiceBinding(UURLabViewerWorldSubsystem& InWorkflow);
	TWeakObjectPtr<UURLabViewerWorldSubsystem> Workflow;
	std::atomic<bool> bActive{true};
};
