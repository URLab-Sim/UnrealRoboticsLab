#pragma once

#include "CoreMinimal.h"
#include "Render/RenderCameraSubsystem.h"
#include "Render/RenderWorldSubsystem.h"
#include "Subsystems/WorldSubsystem.h"
#include "Workflows/RenderOwnerAuthority.h"
#include "Workflows/RenderScenePatch.h"
#include "RenderServerWorldSubsystem.generated.h"

class FMjRenderKinematics;
class FMjRenderKinematicsCandidate;
class FURLabCameraStreamController;
class UURLabRenderServerWorldSubsystem;
struct FURLabRenderWorldServiceBinding;

struct URLAB_API FURLabRenderServerCaptureHandle
{
public:
	bool IsValid() const;
	const FRenderFrameIdentity& GetSubmittedFrameIdentity() const { return SubmittedFrame; }

private:
	friend class UURLabRenderServerWorldSubsystem;
	TWeakObjectPtr<UURLabRenderServerWorldSubsystem> Workflow;
	FGuid WorkflowLifetime;
	uint64 JobSerial = 0;
	FRenderFrameIdentity SubmittedFrame;
};

struct URLAB_API FURLabRenderServerCaptureAdmission
{
	bool bAccepted = false;
	FURLabRenderServerCaptureHandle Handle;
	FString Error;
};

struct URLAB_API FURLabRenderServerInstallResult
{
	bool bCommitted = false;
	uint64 SceneGeneration = 0;
	FString SceneRevision;
	FString Error;
};

/** Exact owner-bound description of the currently installed render source. */
struct URLAB_API FURLabRenderSourceDescription
{
	FString SceneRevision;
	uint64 SceneGeneration = 0;
	TArray<FString> BodyNames;
	int32 GeomCount = 0;
	TArray<FRenderDeformableDesc> Deformables;
	TArray<FString> LightNames;
	TArray<FRenderCameraEffectiveConfiguration> ModelCameras;
	TOptional<FRenderCameraEffectiveConfiguration> UserCamera;
	bool bSupportsQpos = false;
	int32 QposWidth = 0;
};

/** Explicit, world-scoped render-request owner/source workflow. */
UCLASS()
class URLAB_API UURLabRenderServerWorldSubsystem : public UTickableWorldSubsystem
{
	GENERATED_BODY()

public:
	static constexpr int32 MaxRetainedCaptureJobs = 32;
	UURLabRenderServerWorldSubsystem();
	virtual ~UURLabRenderServerWorldSubsystem() override;

	bool Activate(FString& OutError);
	bool AttachBridge(FString& OutError);
	void DetachBridge();
	bool IsActive() const;
	bool IsBridgeAttached() const;

	FURLabRenderOwnerStatusResult OwnerStatus();
	FURLabRenderOwnerAttachResult AttachOwner(
		const FURLabRenderOwnerSessionScopePtr& Scope,
		double TtlSeconds = FURLabRenderOwnerAuthority::DefaultLeaseSeconds);
	FURLabRenderOwnerAuthorizationResult AuthorizeOwner(
		const FURLabRenderOwnerSessionScopePtr& Scope, uint64 Generation,
		const FString& Token);
	FURLabRenderOwnerLeaseResult RenewOwner(const FURLabRenderOwnerProof& Proof,
		double TtlSeconds = FURLabRenderOwnerAuthority::DefaultLeaseSeconds);
	FURLabRenderOwnerLeaseResult ReleaseOwner(const FURLabRenderOwnerProof& Proof);
	FURLabRenderOwnerLeaseResult ReleaseOwnerCredentials(
		const FURLabRenderOwnerSessionScopePtr& Scope, uint64 Generation,
		const FString& Token);
	FURLabRenderOwnerLeaseResult RetireOwnerScope(
		const FURLabRenderOwnerSessionScopePtr& Scope);

	FURLabRenderServerInstallResult InstallNeutralScene(
		const FURLabRenderOwnerProof& Proof, const FString& ExpectedRevision,
		FRenderSceneInstallPayload&& Payload, const FRenderSceneInstallOptions& Options);
	FURLabRenderServerInstallResult InstallKinematicsCandidate(
		const FURLabRenderOwnerProof& Proof, const FString& ExpectedRevision,
		FMjRenderKinematicsCandidate&& Candidate,
		const FRenderSceneInstallOptions& Options);
	FURLabRenderServerInstallResult PatchScene(const FURLabRenderOwnerProof& Proof,
		const FString& ExpectedRevision, const FURLabRenderScenePatch& Patch);
	bool DescribeSource(const FURLabRenderOwnerProof& Proof,
		FURLabRenderSourceDescription& OutDescription, FString& OutError);
	bool ConfigureCameraStream(const FURLabRenderOwnerProof& Proof,
		const FString& ExpectedRevision, FURLabCameraStreamController& Controller,
		const FString& CameraName, bool bEnabled, TOptional<double> RateHz,
		struct FURLabCameraStreamDescriptor& OutDescriptor, FString& OutError);

	FURLabRenderFramePublicationResult PublishCartesian(
		const FURLabRenderOwnerProof& Proof, const FString& ExpectedRevision,
		const FRenderFrame& Frame);
	FURLabRenderFramePublicationResult PublishQpos(
		const FURLabRenderOwnerProof& Proof, const FString& ExpectedRevision,
		TConstArrayView<double> Qpos, uint64 FrameId, double SourceTime,
		bool bHasSourceTime, uint64 ClockEpoch);
	bool PlanCapture(const FURLabRenderOwnerProof& Proof,
		const FString& ExpectedRevision, const TArray<FString>& CameraNames,
		FURLabRenderExactCapturePlan& OutPlan, FString& OutError);
	bool PlanCapture(const FURLabRenderOwnerProof& Proof,
		const FString& ExpectedRevision,
		const TArray<FRenderCameraCaptureSelection>& Cameras,
		FURLabRenderExactCapturePlan& OutPlan, FString& OutError);
	bool PlanCapture(const FURLabRenderOwnerProof& Proof,
		const FString& ExpectedRevision,
		const TArray<FRenderCameraCaptureSelection>& Cameras,
		const TArray<FRenderGeomPoseOverride>& GeomPoseOverrides,
		FURLabRenderExactCapturePlan& OutPlan, FString& OutError);
	FURLabRenderServerCaptureAdmission SubmitCartesianCapture(
		const FURLabRenderOwnerProof& Proof, const FString& ExpectedRevision,
		FRenderFrame&& Frame, FURLabRenderExactCapturePlan Plan,
		FRenderCameraCaptureLifetimeLeasePtr PixelLease,
		FRenderCameraCaptureLifetimeLeasePtr ResultLease,
		double TimeoutSeconds = 0.0);
	FURLabRenderServerCaptureAdmission SubmitQposCapture(
		const FURLabRenderOwnerProof& Proof, const FString& ExpectedRevision,
		TConstArrayView<double> Qpos, uint64 FrameId, double SourceTime,
		bool bHasSourceTime, uint64 ClockEpoch, FURLabRenderExactCapturePlan Plan,
		FRenderCameraCaptureLifetimeLeasePtr PixelLease,
		FRenderCameraCaptureLifetimeLeasePtr ResultLease,
		double TimeoutSeconds = 0.0);
	FURLabRenderServerCaptureAdmission QueryCaptureHistory(
		const FURLabRenderOwnerProof& Proof, const FURLabRenderHistoryQuery& Query,
		FRenderCameraCaptureLifetimeLeasePtr ResultLease);
	bool ResolveCaptureHistory(const FURLabRenderOwnerProof& Proof,
		const FURLabRenderHistoryQuery& Query, FURLabRenderExactCapturePlan& OutPlan,
		FRenderFrameIdentity& OutIdentity, FString& OutError);
	void RetireCaptureHistory(const FGuid& RetentionContext);
	bool PollCapture(const FURLabRenderOwnerProof& Proof,
		const FURLabRenderServerCaptureHandle& Handle,
		FURLabRenderCaptureJobSnapshot& OutSnapshot, FString& OutError);
	bool CancelCapture(const FURLabRenderOwnerProof& Proof,
		const FURLabRenderServerCaptureHandle& Handle, FString& OutError);
	bool ReleaseCapture(const FURLabRenderOwnerProof& Proof,
		const FURLabRenderServerCaptureHandle& Handle, FString& OutError);

	void ShutdownWorkflow();
	virtual void Tick(float DeltaTime) override;
	virtual TStatId GetStatId() const override;
	virtual bool DoesSupportWorldType(EWorldType::Type WorldType) const override;
	virtual void Deinitialize() override;

private:
	struct FKinematicsDeleter { void operator()(FMjRenderKinematics* Value) const; };
	using FKinematicsPtr = TUniquePtr<FMjRenderKinematics, FKinematicsDeleter>;
	struct FCaptureJobRecord;
	struct FCaptureJobRecordDeleter { void operator()(FCaptureJobRecord* Value) const; };
	using FCaptureJobRecordPtr = TUniquePtr<FCaptureJobRecord, FCaptureJobRecordDeleter>;

	struct FSourceBinding
	{
		FURLabRenderOwnerProof Owner;
		FString SceneRevision;
		FKinematicsPtr Kinematics;
	};

	FURLabRenderServerInstallResult InstallPrepared(
		const FURLabRenderOwnerProof& Proof, const FString& ExpectedRevision,
		FRenderSceneInstallPayload&& Payload, const FRenderSceneInstallOptions& Options,
		FKinematicsPtr&& Kinematics);
	FURLabRenderServerCaptureAdmission SubmitPreparedCapture(
		const FURLabRenderOwnerProof& Proof, const FString& ExpectedRevision,
		FRenderFrame&& Frame, FURLabRenderExactCapturePlan&& Plan,
		FRenderCameraCaptureLifetimeLeasePtr&& PixelLease,
		FRenderCameraCaptureLifetimeLeasePtr&& ResultLease, double TimeoutSeconds);
	bool ValidateMutation(const FURLabRenderOwnerProof& Proof,
		const FString& ExpectedRevision, FString& OutError);
	bool IsBoundOwner(const FURLabRenderOwnerProof& Proof) const;
	bool ValidateJobOwner(const FURLabRenderOwnerProof& Proof,
		const FURLabRenderServerCaptureHandle& Handle,
		FCaptureJobRecord*& OutRecord, FString& OutError);
	void RetireCaptureJobs(const FURLabRenderOwnerProof* ExactOwner);
	FURLabRenderOwnerLeaseResult CompleteRetirement(
		FURLabRenderOwnerRetirementResult&& Begun);
	void RevokeSource();
	void FinishMutation();
	void ShutdownNow();

	FURLabRenderOwnerAuthority Authority;
	TSharedPtr<FURLabRenderWorldServiceBinding, ESPMode::ThreadSafe> ServiceBinding;
	FURLabRenderOwnerSessionScopePtr OwnerScope;
	TWeakObjectPtr<UURLabRenderWorldSubsystem> Coordinator;
	FURLabRenderProducerRegistration Producer;
	TOptional<FSourceBinding> Source;
	TMap<uint64, FCaptureJobRecordPtr> CaptureJobs;
	TArray<FCaptureJobRecordPtr> PendingRetiredCaptureJobs;
	FGuid WorkflowLifetime;
	uint64 NextCaptureJobSerial = 1;
	bool bActive = false;
	bool bMutationInProgress = false;
	bool bShutdownRequested = false;
	bool bShutdownInProgress = false;
	bool bShutdownPermanent = false;
};
