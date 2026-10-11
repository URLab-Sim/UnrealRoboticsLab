#include "Render/CameraStreamController.h"

#include "Render/RenderCameraComponent.h"
#include "Render/RenderCameraRig.h"
#include "Render/URLabRenderScene.h"

namespace
{
bool StreamConfiguration(const FString& SourceRevision,
	const FRenderCameraEffectiveConfiguration& Camera,
	FURLabCameraStreamConfiguration& Out, FString& OutError)
{
	Out = {};
	Out.SourceRevision = SourceRevision;
	Out.CameraName = Camera.CanonicalName;
	Out.CameraConfigurationRevision = Camera.ConfigurationRevision;
	Out.Width = Camera.Resolution.X;
	Out.Height = Camera.Resolution.Y;
	Out.DepthNearCm = Camera.DepthNearCm;
	Out.DepthFarCm = Camera.DepthFarCm;
	switch (Camera.Mode)
	{
		case ERenderCameraMode::Real: Out.Encoding = TEXT("rgba8"); Out.StrideBytes = Out.Width * 4; break;
		case ERenderCameraMode::Depth: Out.Encoding = TEXT("depth_f32le"); Out.StrideBytes = Out.Width * 4; break;
		case ERenderCameraMode::SemanticSegmentation:
		case ERenderCameraMode::InstanceSegmentation:
			Out.Encoding = TEXT("seg_i32x2le"); Out.StrideBytes = Out.Width * 8; break;
		default: OutError = TEXT("camera stream baseline mode is unsupported"); return false;
	}
	return true;
}
}

TUniquePtr<FURLabCameraStreamController> FURLabCameraStreamController::Create(
	const FURLabCameraStreamProducerConfig& Config, FString& OutError)
{
	TUniquePtr<FURLabCameraStreamProducer> Producer =
		FURLabCameraStreamProducer::Create(Config, OutError);
	return Producer ? TUniquePtr<FURLabCameraStreamController>(
		new FURLabCameraStreamController(MoveTemp(Producer))) : nullptr;
}

FURLabCameraStreamController::FURLabCameraStreamController(
	TUniquePtr<FURLabCameraStreamProducer>&& InProducer)
	: Producer(MoveTemp(InProducer))
{
}

FURLabCameraStreamController::~FURLabCameraStreamController()
{
	Disable();
}

bool FURLabCameraStreamController::Configure(AURLabRenderScene& Scene,
	const FString& InSourceRevision, const FString& CameraName, bool bEnabled,
	TOptional<double> RateHz, FURLabCameraStreamDescriptor& OutDescriptor,
	FString& OutError)
{
	check(IsInGameThread());
	OutError.Reset();
	if (!bEnabled)
	{
		Disable();
		return true;
	}
	if (!Producer || InSourceRevision.IsEmpty() || CameraName.IsEmpty()
		|| (RateHz.IsSet() && (!FMath::IsFinite(RateHz.GetValue())
			|| RateHz.GetValue() <= 0.0
			|| RateHz.GetValue() > static_cast<double>(TNumericLimits<float>::Max()))))
	{
		OutError = TEXT("camera stream source/camera/rate is invalid");
		return false;
	}
	URenderCameraRig* Rig = Scene.GetCameraRig();
	URenderCameraComponent* Selected = Rig ? Rig->FindByCanonicalName(CameraName) : nullptr;
	if (!Selected)
	{
		OutError = TEXT("camera stream camera is not installed");
		return false;
	}

	const bool bSame = Camera.Get() == Selected && SourceRevision == InSourceRevision;
	if (!bSame)
	{
		Disable();
		bWasStreaming = Selected->IsStreamingActive();
		bWasContinuous = Selected->bContinuousCapture;
		bWasManualCaptureOnly = Selected->bManualCaptureOnly;
		bBaselineOnStateChange = Selected->GetDescriptor().Capture.bCaptureOnStateChange;
		BaselineMaxFps = Selected->GetDescriptor().Capture.CaptureMaxFps;
	}
	const bool bPreviousContinuous = Selected->bContinuousCapture;
	const bool bPreviousManualCaptureOnly = Selected->bManualCaptureOnly;
	const bool bPreviousOnStateChange = Selected->GetDescriptor().Capture.bCaptureOnStateChange;
	const float PreviousMaxFps = Selected->GetDescriptor().Capture.CaptureMaxFps;
	const bool bPreviousStreaming = Selected->IsStreamingActive();
	auto RollBackSelected = [&]
	{
		Selected->bContinuousCapture = bPreviousContinuous;
		Selected->bManualCaptureOnly = bPreviousManualCaptureOnly;
		if (Selected->GetDescriptor().Capture.bCaptureOnStateChange != bPreviousOnStateChange
			|| Selected->GetDescriptor().Capture.CaptureMaxFps != PreviousMaxFps)
		{
			Selected->SetCaptureRate(bPreviousOnStateChange, PreviousMaxFps);
		}
		if (!bPreviousStreaming && Selected->IsStreamingActive())
			Selected->SetStreamingEnabled(false);
	};
	if (RateHz.IsSet())
	{
		Selected->SetCaptureRate(false, static_cast<float>(RateHz.GetValue()));
		bRateChanged = true;
	}
	Selected->bContinuousCapture = true;
	Selected->bManualCaptureOnly = false;
	// SetStreamingEnabled immediately issues CaptureScene. A freshly installed
	// scene may still be finishing component PSO precache after its materials were
	// assigned, so leave activation to the shared scheduler while that is pending.
	// The stream configuration is valid and bounded independently of its first frame.
	if (Scene.AreRenderResourcesReadyForCapture())
	{
		Selected->SetStreamingEnabled(true);
		if (!Selected->IsStreamingActive())
		{
			RollBackSelected();
			OutError = TEXT("camera stream could not activate the installed camera");
			return false;
		}
	}

	FRenderCameraEffectiveConfiguration Effective;
	if (!Selected->ResolveCaptureConfiguration({}, Effective, OutError))
	{
		RollBackSelected();
		return false;
	}
	FURLabCameraStreamConfiguration Configuration;
	if (!StreamConfiguration(InSourceRevision, Effective, Configuration, OutError)
		|| !Producer->CommitCamera(Configuration, OutDescriptor, OutError))
	{
		RollBackSelected();
		return false;
	}
	if (!bSame)
	{
		Camera = Selected;
		FrameDelegate = Selected->OnSharedFrameCompleted.AddRaw(
			this, &FURLabCameraStreamController::HandleFrame);
	}
	ActiveCameraName = CameraName;
	SourceRevision = InSourceRevision;
	ConfigurationRevision = Effective.ConfigurationRevision;
	return true;
}

bool FURLabCameraStreamController::Describe(const FString& CameraName,
	bool bShmEligible, FURLabCameraStreamDescriptor& OutDescriptor,
	FURLabCameraStreamDelivery& OutDelivery, FString& OutError) const
{
	if (!Producer || !Camera.IsValid() || CameraName != ActiveCameraName)
	{
		OutError = TEXT("camera stream is not enabled for the requested camera");
		return false;
	}
	return Producer->Describe(bShmEligible, OutDescriptor, OutDelivery, OutError);
}

void FURLabCameraStreamController::Disable()
{
	check(IsInGameThread());
	if (URenderCameraComponent* Existing = Camera.Get())
	{
		if (FrameDelegate.IsValid()) Existing->OnSharedFrameCompleted.Remove(FrameDelegate);
		Existing->bContinuousCapture = bWasContinuous;
		Existing->bManualCaptureOnly = bWasManualCaptureOnly;
		if (bRateChanged) Existing->SetCaptureRate(bBaselineOnStateChange, BaselineMaxFps);
		if (!bWasStreaming) Existing->SetStreamingEnabled(false);
	}
	Camera.Reset(); FrameDelegate.Reset(); ActiveCameraName.Reset(); SourceRevision.Reset();
	ConfigurationRevision = 0; bRateChanged = false; LastPublishError.Reset();
}

void FURLabCameraStreamController::HandleFrame(const FRenderCameraFramePtr& Frame)
{
	check(IsInGameThread());
	if (!Producer || !Camera.IsValid() || !Frame
		|| Frame->CameraConfigurationRevision != ConfigurationRevision
		|| Frame->Identity.SceneRevision != SourceRevision) return;
	FString Error;
	if (Producer->SubmitCompleted(Frame, Error)) LastPublishError.Reset();
	else LastPublishError = MoveTemp(Error);
}
