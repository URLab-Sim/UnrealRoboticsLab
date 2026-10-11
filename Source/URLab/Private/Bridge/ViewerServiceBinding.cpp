#include "Bridge/ViewerServiceBinding.h"
#include "Workflows/ViewerWorldSubsystem.h"

FURLabViewerServiceBinding::FURLabViewerServiceBinding(UURLabViewerWorldSubsystem& InWorkflow)
	: Workflow(&InWorkflow) {}

TSharedPtr<FURLabViewerServiceBinding,ESPMode::ThreadSafe> FURLabViewerServiceBinding::Create(
	UURLabViewerWorldSubsystem& Workflow,FString& OutError)
{
	check(IsInGameThread());
	if(!Workflow.IsActive()){OutError=TEXT("viewer workflow must be active before service binding");return nullptr;}
	OutError.Reset(); return MakeShareable(new FURLabViewerServiceBinding(Workflow));
}

UURLabViewerWorldSubsystem* FURLabViewerServiceBinding::ResolveWorkflow() const
{ check(IsInGameThread()); return IsActive()?Workflow.Get():nullptr; }

UURLabViewerWorldSubsystem* FURLabViewerServiceBinding::ResolveWorkflowForRetirement() const
{ check(IsInGameThread()); return Workflow.Get(); }

FURLabRenderOwnerLeaseResult FURLabViewerServiceBinding::RetireScope(
	const FURLabRenderOwnerSessionScopePtr& Scope)
{
	check(IsInGameThread());
	if(UURLabViewerWorldSubsystem* Owner=ResolveWorkflowForRetirement())
		return Owner->RetireOwnerScope(Scope);
	FURLabRenderOwnerLeaseResult Result;
	Result.Error=EURLabRenderOwnerAuthorityError::Unavailable;
	Result.Message=TEXT("viewer workflow is unavailable");
	return Result;
}
