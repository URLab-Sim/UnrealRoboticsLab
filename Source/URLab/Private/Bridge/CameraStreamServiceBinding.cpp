#include "Bridge/CameraStreamServiceBinding.h"

#include "MuJoCo/Runtime/MjSimulationWorldSubsystem.h"
#include "Workflows/ViewerWorldSubsystem.h"

FURLabCameraStreamServiceBinding::FURLabCameraStreamServiceBinding(
	TUniquePtr<FURLabCameraStreamController>&& InController)
	: Controller(MoveTemp(InController)) {}

TSharedPtr<FURLabCameraStreamServiceBinding, ESPMode::ThreadSafe>
FURLabCameraStreamServiceBinding::Create(UMjSimulationWorldSubsystem& Workflow,
	const FURLabCameraStreamProducerConfig& Config, FString& OutError)
{
	check(IsInGameThread());
	auto Controller = FURLabCameraStreamController::Create(Config, OutError);
	if (!Controller) return nullptr;
	TSharedPtr<FURLabCameraStreamServiceBinding, ESPMode::ThreadSafe> Result =
		MakeShareable(new FURLabCameraStreamServiceBinding(MoveTemp(Controller)));
	Result->Simulation = &Workflow;
	return Result;
}

TSharedPtr<FURLabCameraStreamServiceBinding, ESPMode::ThreadSafe>
FURLabCameraStreamServiceBinding::Create(UURLabViewerWorldSubsystem& Workflow,
	const FURLabCameraStreamProducerConfig& Config, FString& OutError)
{
	check(IsInGameThread());
	auto Controller = FURLabCameraStreamController::Create(Config, OutError);
	if (!Controller) return nullptr;
	TSharedPtr<FURLabCameraStreamServiceBinding, ESPMode::ThreadSafe> Result =
		MakeShareable(new FURLabCameraStreamServiceBinding(MoveTemp(Controller)));
	Result->Viewer = &Workflow;
	return Result;
}

void FURLabCameraStreamServiceBinding::Deactivate()
{
	check(IsInGameThread());
	if (!bActive.exchange(false, std::memory_order_acq_rel)) return;
	if (Controller) Controller->Disable();
}

void FURLabCameraStreamServiceBinding::DisableStream()
{
	check(IsInGameThread());
	if (IsActive() && Controller) Controller->Disable();
}

bool FURLabCameraStreamServiceBinding::ConfigureSimulation(
	const FMjSimulationControlHandle& Control, const FString& CameraName,
	bool bEnabled, TOptional<double> RateHz,
	FURLabCameraStreamDescriptor& OutDescriptor, FString& OutError)
{
	check(IsInGameThread());
	auto* Workflow = IsActive() ? Simulation.Get() : nullptr;
	if (!Workflow || !Controller)
	{
		OutError = TEXT("simulation camera stream service is unavailable"); return false;
	}
	return Workflow->ConfigureCameraStream(Control, *Controller, CameraName,
		bEnabled, RateHz, OutDescriptor, OutError);
}

bool FURLabCameraStreamServiceBinding::ConfigureViewer(
	const FURLabRenderOwnerProof& Proof, const FString& ExpectedRevision,
	const FString& CameraName, bool bEnabled, TOptional<double> RateHz,
	FURLabCameraStreamDescriptor& OutDescriptor, FString& OutError)
{
	check(IsInGameThread());
	auto* Workflow = IsActive() ? Viewer.Get() : nullptr;
	if (!Workflow || !Controller)
	{
		OutError = TEXT("viewer camera stream service is unavailable"); return false;
	}
	return Workflow->ConfigureCameraStream(Proof, ExpectedRevision, *Controller,
		CameraName, bEnabled, RateHz, OutDescriptor, OutError);
}

bool FURLabCameraStreamServiceBinding::Describe(const FString& CameraName,
	bool bShmEligible, FURLabCameraStreamDescriptor& OutDescriptor,
	FURLabCameraStreamDelivery& OutDelivery, FString& OutError) const
{
	check(IsInGameThread());
	if (!IsActive() || !Controller)
	{
		OutError = TEXT("camera stream service is unavailable"); return false;
	}
	if (!Controller->Describe(CameraName, bShmEligible, OutDescriptor, OutDelivery, OutError))
		return false;
	const FString ActiveRevision = Simulation.IsValid()
		? Simulation->GetActiveSceneRevision()
		: Viewer.IsValid() ? Viewer->Status().SceneRevision : FString();
	if (ActiveRevision.IsEmpty() || OutDescriptor.SourceRevision != ActiveRevision)
	{
		OutError = TEXT("camera stream source revision is no longer active"); return false;
	}
	return true;
}
