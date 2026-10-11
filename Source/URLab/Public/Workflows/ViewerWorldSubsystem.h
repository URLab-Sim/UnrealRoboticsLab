#pragma once

#include "CoreMinimal.h"
#include "Render/RenderWorldSubsystem.h"
#include "Subsystems/WorldSubsystem.h"
#include "Workflows/RenderOwnerAuthority.h"
#include "Workflows/RenderScenePatch.h"
#include "Bridge/ViewerStreamConnection.h"
#include "Bridge/URLabPayloadStore.h"
#include "Workflows/InteractionInbox.h"
#include "ViewerWorldSubsystem.generated.h"

class FMjRenderKinematics;
class FMjRenderKinematicsCandidate;
class FURLabCameraStreamController;
struct FURLabViewerWorldServiceBinding;

struct URLAB_API FURLabViewerStreamDescriptor
{
	FString ProducerBootId;
	FString StreamId;
	uint64 StreamGeneration = 0;
	FString SourceRevision;
	uint32 SchemaVersion = 2;
	FString PayloadFormat = TEXT("dense_rh_m_wxyz_f64le_dynamic_mesh_v1");

	bool IsValid(FString& OutError) const;
};

struct URLAB_API FURLabViewerStreamFrameHeader
{
	FString ProducerBootId;
	FString StreamId;
	uint64 StreamGeneration = 0;
	FString SourceRevision;
	uint64 Sequence = 0;
	uint64 FrameId = 0;
	double SourceTime = 0.0;
	bool bHasSourceTime = false;
	uint64 ClockEpoch = 0;
};

struct URLAB_API FURLabViewerInstallResult
{
	bool bCommitted = false;
	uint64 SceneGeneration = 0;
	FString SceneRevision;
	FString Error;
};

struct URLAB_API FURLabViewerStatus
{
	FURLabRenderOwnerStatus Owner;
	bool bHasInstalledSource = false;
	bool bStreamBound = false;
	bool bDisplayStale = true;
	FString SceneRevision;
};

/** Manager-free, transport-independent continuous viewer workflow. */
UCLASS()
class URLAB_API UURLabViewerWorldSubsystem : public UTickableWorldSubsystem
{
	GENERATED_BODY()

public:
	UURLabViewerWorldSubsystem();
	virtual ~UURLabViewerWorldSubsystem() override;

	bool Activate(FString& OutError);
	bool AttachBridge(FString& OutError);
	void DetachBridge();
	bool IsBridgeAttached() const;
	bool IsActive() const;
	FURLabViewerStatus Status();

	FURLabRenderOwnerAttachResult AttachOwner(const FURLabRenderOwnerSessionScopePtr& Scope,
		double TtlSeconds = FURLabRenderOwnerAuthority::DefaultLeaseSeconds);
	FURLabRenderOwnerAuthorizationResult AuthorizeOwner(
		const FURLabRenderOwnerSessionScopePtr& Scope, uint64 Generation, const FString& Token);
	FURLabRenderOwnerLeaseResult RenewOwner(const FURLabRenderOwnerProof& Proof,
		double TtlSeconds = FURLabRenderOwnerAuthority::DefaultLeaseSeconds);
	FURLabRenderOwnerLeaseResult ReleaseOwnerCredentials(
		const FURLabRenderOwnerSessionScopePtr& Scope, uint64 Generation, const FString& Token);
	FURLabRenderOwnerLeaseResult RetireOwnerScope(const FURLabRenderOwnerSessionScopePtr& Scope);

	FURLabViewerInstallResult InstallNeutralScene(const FURLabRenderOwnerProof& Proof,
		const FString& ExpectedRevision, FRenderSceneInstallPayload&& Payload,
		const FRenderSceneInstallOptions& Options);
	FURLabViewerInstallResult InstallKinematicsCandidate(const FURLabRenderOwnerProof& Proof,
		const FString& ExpectedRevision, FMjRenderKinematicsCandidate&& Candidate,
		const FRenderSceneInstallOptions& Options);
	FURLabViewerInstallResult PatchScene(const FURLabRenderOwnerProof& Proof,
		const FString& ExpectedRevision, const FURLabRenderScenePatch& Patch);
	bool BindStream(const FURLabRenderOwnerProof& Proof, const FString& ExpectedRevision,
		const FURLabViewerStreamDescriptor& Descriptor, FString& OutError);
	bool BindPreparedStream(const FURLabRenderOwnerProof& Proof,const FString& ExpectedRevision,
		const FURLabViewerStreamDescriptor& Descriptor,FURLabViewerStreamConnectionPtr Connection,
		TSharedPtr<FURLabPayloadStore,ESPMode::ThreadSafe> Store,const FURLabPayloadOwner& PayloadOwner,
		FString& OutError);
	bool ConfigureCameraStream(const FURLabRenderOwnerProof& Proof,
		const FString& ExpectedRevision, FURLabCameraStreamController& Controller,
		const FString& CameraName, bool bEnabled, TOptional<double> RateHz,
		struct FURLabCameraStreamDescriptor& OutDescriptor, FString& OutError);
	bool ForwardInteraction(const FURLabRenderOwnerProof& Proof,
		const FString& ExpectedRevision,
		const FURLabInteractionSourceIdentity& ProducerSource,
		FURLabWireValue Body, int32 TimeoutMs,
		FURLabViewerStreamConnection::FInteractionCallback&& Completion,
		FString& OutError);
	FURLabRenderFramePublicationResult ApplyStreamFrame(const FURLabRenderOwnerProof& Proof,
		const FURLabViewerStreamFrameHeader& Header, const FRenderFrame& Frame);

	void ShutdownWorkflow();
	virtual void Tick(float DeltaTime) override;
	virtual TStatId GetStatId() const override;
	virtual bool DoesSupportWorldType(EWorldType::Type WorldType) const override;
	virtual void Deinitialize() override;

private:
	struct FKinematicsDeleter { void operator()(FMjRenderKinematics* Value) const; };
	using FKinematicsPtr = TUniquePtr<FMjRenderKinematics, FKinematicsDeleter>;
	struct FSourceState { FString SceneRevision; FKinematicsPtr Kinematics; int32 BodyCount=0; int32 ModelCameraCount=0; int32 LightCount=0; bool bHasUserCamera=false; TArray<FRenderDeformableDesc> Deformables; };
	struct FStreamBinding
	{
		FURLabRenderOwnerProof Owner;
		FURLabViewerStreamDescriptor Descriptor;
		FString LocalSceneRevision;
		uint64 LastSequence = 0;
		FURLabViewerStreamConnectionPtr Connection;
		TSharedPtr<FURLabPayloadStore,ESPMode::ThreadSafe> Store;
		FURLabPayloadOwner PayloadOwner;
	};

	FURLabViewerInstallResult InstallPrepared(const FURLabRenderOwnerProof& Proof,
		const FString& ExpectedRevision, FRenderSceneInstallPayload&& Payload,
		const FRenderSceneInstallOptions& Options, FKinematicsPtr&& Kinematics);
	bool ValidateMutation(const FURLabRenderOwnerProof& Proof,
		const FString& ExpectedRevision, FString& OutError);
	FURLabRenderOwnerLeaseResult CompleteRetirement(FURLabRenderOwnerRetirementResult&& Begun);
	void ClearStreamBinding();
	void FinishMutation();
	void ShutdownNow();

	FURLabRenderOwnerAuthority Authority;
	TSharedPtr<FURLabViewerWorldServiceBinding,ESPMode::ThreadSafe> ServiceBinding;
	FRenderCameraCaptureLifetimeLeasePtr VisibleFrameLease;
	FURLabRenderOwnerSessionScopePtr OwnerScope;
	TWeakObjectPtr<UURLabRenderWorldSubsystem> Coordinator;
	FURLabRenderProducerRegistration Producer;
	TOptional<FSourceState> Source;
	TOptional<FStreamBinding> Stream;
	TArray<FURLabViewerStreamConnectionPtr> RetiringConnections;
	bool bDisplayStale = true;
	bool bActive = false;
	bool bMutationInProgress = false;
	bool bShutdownRequested = false;
	bool bShutdownInProgress = false;
	bool bShutdownPermanent = false;
};
