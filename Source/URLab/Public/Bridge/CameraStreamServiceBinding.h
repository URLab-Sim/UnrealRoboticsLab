#pragma once

#include "CoreMinimal.h"
#include "MuJoCo/Runtime/MjSimulationRuntime.h"
#include "Render/CameraStreamController.h"
#include "Workflows/RenderOwnerAuthority.h"
#include <atomic>

class UMjSimulationWorldSubsystem;
class UURLabViewerWorldSubsystem;

/** One workflow-scoped live-camera controller and its exact authority target. */
class URLAB_API FURLabCameraStreamServiceBinding
	: public TSharedFromThis<FURLabCameraStreamServiceBinding, ESPMode::ThreadSafe>
{
public:
	static TSharedPtr<FURLabCameraStreamServiceBinding, ESPMode::ThreadSafe> Create(
		UMjSimulationWorldSubsystem& Workflow,
		const FURLabCameraStreamProducerConfig& Config, FString& OutError);
	static TSharedPtr<FURLabCameraStreamServiceBinding, ESPMode::ThreadSafe> Create(
		UURLabViewerWorldSubsystem& Workflow,
		const FURLabCameraStreamProducerConfig& Config, FString& OutError);

	bool IsActive() const { return bActive.load(std::memory_order_acquire); }
	void Deactivate();
	/** Stop this binding's current camera while keeping the scoped service available. */
	void DisableStream();
	bool ConfigureSimulation(const FMjSimulationControlHandle& Control,
		const FString& CameraName, bool bEnabled, TOptional<double> RateHz,
		FURLabCameraStreamDescriptor& OutDescriptor, FString& OutError);
	bool ConfigureViewer(const FURLabRenderOwnerProof& Proof,
		const FString& ExpectedRevision, const FString& CameraName, bool bEnabled,
		TOptional<double> RateHz, FURLabCameraStreamDescriptor& OutDescriptor,
		FString& OutError);
	bool Describe(const FString& CameraName, bool bShmEligible,
		FURLabCameraStreamDescriptor& OutDescriptor,
		FURLabCameraStreamDelivery& OutDelivery, FString& OutError) const;

private:
	explicit FURLabCameraStreamServiceBinding(
		TUniquePtr<FURLabCameraStreamController>&& InController);
	TWeakObjectPtr<UMjSimulationWorldSubsystem> Simulation;
	TWeakObjectPtr<UURLabViewerWorldSubsystem> Viewer;
	TUniquePtr<FURLabCameraStreamController> Controller;
	std::atomic<bool> bActive{true};
};
