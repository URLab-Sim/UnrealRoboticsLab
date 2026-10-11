// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "Templates/Function.h"
#include "Render/RenderCameraTypes.h"
#include "Render/RenderWorldSubsystem.h"
#include "RenderCameraSubsystem.generated.h"

class URenderCameraComponent;
struct FURLabRenderCaptureJobSharedState;

/** Exact ordered camera configuration and neutral capacity bounds for one job. */
struct URLAB_API FURLabRenderExactCapturePlan
{
	FURLabRenderProducerRegistration Producer;
	FGuid RetentionContext;
	FString SceneRevision;
	uint64 SceneGeneration = 0;
	TArray<FRenderCameraEffectiveConfiguration> Cameras;
	TArray<FRenderGeomPoseOverride> GeomPoseOverrides;
	FString GeometryConfigurationId;
	int64 AggregateLogicalPixelBytes = 0;
	int64 AggregatePhysicalArrayBytes = 0;
};

URLAB_API FString RenderCameraConfigurationId(
	const FRenderCameraEffectiveConfiguration& Configuration);

struct URLAB_API FURLabRenderHistoryCameraQuery
{
	FString Name;
	FString ConfigurationId;
};

struct URLAB_API FURLabRenderHistoryQuery
{
	FGuid RetentionContext;
	FString ExpectedSceneRevision;
	uint64 ClockEpoch = 0;
	double TargetSourceTime = 0.0;
	TArray<FURLabRenderHistoryCameraQuery> Cameras;
	FString GeometryConfigurationId;
};

enum class EURLabRenderCaptureJobState : uint8
{
	Queued,
	Running,
	Succeeded,
	Failed,
	Cancelled,
};

/** Retained view of one exact all-camera job. Frame access is game-thread-only. */
struct URLAB_API FURLabRenderCaptureJobSnapshot
{
	EURLabRenderCaptureJobState State = EURLabRenderCaptureJobState::Queued;
	FString SceneRevision;
	FString GeometryConfigurationId;
	TArray<FString> CameraNames;
	/** Populated in request order only when every selected camera succeeds. */
	TArray<FRenderCameraFramePtr> Frames;
	FString AffectedCamera;
	FString Message;
	bool bGpuWritersDrained = true;

	bool IsTerminal() const
	{
		return State == EURLabRenderCaptureJobState::Succeeded
			|| State == EURLabRenderCaptureJobState::Failed
			|| State == EURLabRenderCaptureJobState::Cancelled;
	}
};

/** Thread-safe terminal/identity view which never touches non-thread-safe frame pointers. */
struct URLAB_API FURLabRenderCaptureJobStatus
{
	EURLabRenderCaptureJobState State = EURLabRenderCaptureJobState::Queued;
	FString SceneRevision;
	FString GeometryConfigurationId;
	TArray<FString> CameraNames;
	FString AffectedCamera;
	FString Message;
	bool bGpuWritersDrained = true;
	bool IsTerminal() const
	{
		return State == EURLabRenderCaptureJobState::Succeeded
			|| State == EURLabRenderCaptureJobState::Failed
			|| State == EURLabRenderCaptureJobState::Cancelled;
	}
};

struct URLAB_API FURLabRenderCaptureJob
{
public:
	bool IsValid() const;
	FURLabRenderCaptureJobStatus PollStatus() const;
	/** Game-thread-only because retained camera frames use Unreal's default shared-pointer mode. */
	FURLabRenderCaptureJobSnapshot Poll() const;

private:
	friend class URenderCameraSubsystem;
	TWeakObjectPtr<URenderCameraSubsystem> Scheduler;
	FGuid SchedulerLifetime;
	uint64 JobSerial = 0;
	TSharedPtr<FURLabRenderCaptureJobSharedState, ESPMode::ThreadSafe> State;
};

/** Move-only exact presentation/camera request. Timeout 0 selects the 30-second default. */
struct URLAB_API FURLabRenderCaptureJobRequest
{
	FURLabRenderProducerRegistration Producer;
	FString ExpectedSceneRevision;
	FRenderFrame PresentationFrame;
	FURLabRenderExactCapturePlan Plan;
	double TimeoutSeconds = 0.0;
	FRenderCameraCaptureLifetimeLeasePtr LifetimeLease;
	/** Invoked on the game thread after scheduler/coordinator mutation has ended. */
	TUniqueFunction<void()> OnTerminal;
};

struct URLAB_API FURLabRenderCaptureJobAdmission
{
	bool bAccepted = false;
	FURLabRenderCaptureJob Job;
	FString Error;
};

/** Physics-independent world tick for registered render cameras. */
UCLASS()
class URLAB_API URenderCameraSubsystem : public UTickableWorldSubsystem
{
	GENERATED_BODY()
public:
	URenderCameraSubsystem();
	virtual ~URenderCameraSubsystem() override;
	static constexpr int32 MaxQueuedExactCaptureJobs = 16;
	static constexpr int32 MaxRetainedExactCaptureJobs = 17;
	static constexpr int32 MaxCamerasPerExactCaptureJob = 8;
	static constexpr double DefaultExactCaptureDeadlineSeconds = 30.0;
	static constexpr double MaxExactCaptureDeadlineSeconds = 30.0;

	void RegisterCamera(URenderCameraComponent* Camera);
	void UnregisterCamera(URenderCameraComponent* Camera);
	bool PlanExactCapture(const FURLabRenderProducerRegistration& Producer,
		const FString& ExpectedSceneRevision, const TArray<FString>& CameraNames,
		FURLabRenderExactCapturePlan& OutPlan, FString& OutError) const;
	bool PlanExactCapture(const FURLabRenderProducerRegistration& Producer,
		const FString& ExpectedSceneRevision,
		const TArray<FRenderCameraCaptureSelection>& Cameras,
		FURLabRenderExactCapturePlan& OutPlan, FString& OutError) const;
	FURLabRenderCaptureJobAdmission SubmitExactCapture(
		FURLabRenderCaptureJobRequest&& Request);
	FURLabRenderCaptureJobAdmission QueryExactCaptureHistory(
		const FURLabRenderHistoryQuery& Query);
	bool ResolveExactCaptureHistory(const FURLabRenderHistoryQuery& Query,
		FURLabRenderExactCapturePlan& OutPlan, FRenderFrameIdentity& OutIdentity,
		FString& OutError) const;
	void RetireExactCaptureHistory(const FGuid& RetentionContext);
	bool CancelExactCapture(const FURLabRenderCaptureJob& Job, FString& OutError);
	virtual void Tick(float DeltaTime) override;
	virtual TStatId GetStatId() const override;
	virtual bool IsTickableInEditor() const override { return true; }
	virtual bool DoesSupportWorldType(EWorldType::Type WorldType) const override;
	virtual void Deinitialize() override;

#if WITH_DEV_AUTOMATION_TESTS
	void SetMonotonicSecondsForTest(double InSeconds) { TestMonotonicSeconds = InSeconds; }
	bool SeedExactCaptureHistoryForTest(const FURLabRenderExactCapturePlan& Plan,
		const FRenderFrameIdentity& Identity,
		const TArray<FRenderCameraFramePtr>& Frames);
	int32 GetExactCaptureHistoryGroupCountForTest() const { return HistoryGroups.Num(); }
	int64 GetExactCaptureHistoryPhysicalBytesForTest() const { return HistoryPhysicalBytes; }
#endif
private:
	friend class UURLabRenderWorldSubsystem;
	struct FQueuedJob;
	struct FHistoryGroup
	{
		FGuid RetentionContext;
		FString SceneRevision;
		FURLabRenderProducerRegistration Producer;
		uint64 SceneGeneration = 0;
		FRenderFrameIdentity Identity;
		FString GeometryConfigurationId;
		TArray<FString> Names;
		TArray<FString> ConfigurationIds;
		TArray<FRenderCameraEffectiveConfiguration> Configurations;
		TArray<FRenderCameraFramePtr> Frames;
		int64 PhysicalBytes = 0;
	};
	struct FQueuedJobDeleter
	{
		void operator()(FQueuedJob* Job) const;
	};
	using FQueuedJobPtr = TUniquePtr<FQueuedJob, FQueuedJobDeleter>;
	void CancelJobsForSceneMutation(UURLabRenderWorldSubsystem* Coordinator,
		uint64 ProducerSerial, uint64 SceneGeneration, const FString& SceneRevision,
		bool bRestorePresentation);
	bool ValidateAdmission(const FURLabRenderCaptureJobRequest& Request,
		UURLabRenderWorldSubsystem*& OutCoordinator, uint64& OutSceneGeneration,
		FString& OutError) const;
	void StartNextJob(double NowSeconds);
	void PumpActiveJob(double NowSeconds);
	void PumpDrainingJobs();
	void FinishActive(EURLabRenderCaptureJobState State, const FString& AffectedCamera,
		const FString& Message, bool bRestorePresentation);
	void MarkQueuedTerminal(FQueuedJob& Job, EURLabRenderCaptureJobState State,
		const FString& Message);
	void QueueNotification(FQueuedJob& Job);
	void DrainNotifications();
	double ReadMonotonicSeconds() const;
	bool JobMatchesScene(const FQueuedJob& Job, UURLabRenderWorldSubsystem* Coordinator,
		uint64 ProducerSerial, uint64 SceneGeneration, const FString& SceneRevision) const;
	bool RetainExactCaptureHistory(const FURLabRenderExactCapturePlan& Plan,
		const FRenderFrameIdentity& Identity,
		const TArray<FRenderCameraFramePtr>& Frames);

	TArray<TWeakObjectPtr<URenderCameraComponent>> Cameras;
	TArray<FQueuedJobPtr> QueuedJobs;
	FQueuedJobPtr ActiveJob;
	TArray<FQueuedJobPtr> DrainingJobs;
	TArray<TUniqueFunction<void()>> PendingNotifications;
	TArray<FHistoryGroup> HistoryGroups;
	int64 HistoryPhysicalBytes = 0;
	static constexpr int32 MaxHistoryGroups = 64;
	static constexpr int64 MaxHistoryPhysicalBytes = 128ll * 1024ll * 1024ll;
	FGuid SchedulerLifetime;
	uint64 NextJobSerial = 1;
	bool bTickingScheduler = false;
	bool bShuttingDown = false;
#if WITH_DEV_AUTOMATION_TESTS
	TOptional<double> TestMonotonicSeconds;
#endif
};
