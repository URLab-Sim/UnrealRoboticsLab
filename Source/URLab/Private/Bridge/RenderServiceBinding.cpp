#include "Bridge/RenderServiceBinding.h"

#include "Workflows/RenderServerWorldSubsystem.h"

struct FURLabRenderServiceBinding::FCaptureRecord
{
	FURLabRenderOwnerSessionScopePtr Scope;
	FURLabRenderOwnerProof Proof;
	FURLabRenderServerCaptureHandle Handle;
	TSharedPtr<FURLabCapturePayloadAdmission, ESPMode::ThreadSafe> Admission;
	EURLabRenderMaterializationState MaterializationState =
		EURLabRenderMaterializationState::Pending;
	FURLabCapturePayloadMaterialization Materialization;
};

FURLabRenderServiceBinding::FURLabRenderServiceBinding(
	UURLabRenderServerWorldSubsystem& InWorkflow)
	: Workflow(&InWorkflow)
{
}

FURLabRenderServiceBinding::~FURLabRenderServiceBinding()
{
	check(Captures.IsEmpty());
	check(!PumpHandle.IsValid());
}

bool FURLabRenderServiceBinding::GetRetentionContext(
	const FURLabRenderOwnerSessionScopePtr& Scope, const FURLabRenderOwnerProof& Proof,
	FGuid& OutContext, FString& OutError)
{
	check(IsInGameThread()); OutContext.Invalidate();
	if (!Scope || !CanAcceptCapture(Scope, OutError)) return false;
	for (const FRetentionContext& Context : RetentionContexts)
		if (Context.Scope == Scope && Context.Generation == Proof.GetGeneration()
			&& Context.Token == Proof.GetOpaqueToken())
		{
			OutContext = Context.Id; OutError.Reset(); return true;
		}
	FRetentionContext& Added = RetentionContexts.AddDefaulted_GetRef();
	Added.Scope = Scope; Added.Generation = Proof.GetGeneration();
	Added.Token = Proof.GetOpaqueToken(); Added.Id = FGuid::NewGuid();
	OutContext = Added.Id; OutError.Reset(); return true;
}

void FURLabRenderServiceBinding::ResetRetentionContext(
	const FURLabRenderOwnerSessionScopePtr& Scope, const FURLabRenderOwnerProof& Proof)
{
	check(IsInGameThread());
	UURLabRenderServerWorldSubsystem* Owner = ResolveWorkflowForRetirement();
	for (int32 Index = RetentionContexts.Num() - 1; Index >= 0; --Index)
	{
		const FRetentionContext& Context = RetentionContexts[Index];
		if (Context.Scope != Scope || Context.Generation != Proof.GetGeneration()
			|| Context.Token != Proof.GetOpaqueToken()) continue;
		if (Owner) Owner->RetireCaptureHistory(Context.Id);
		RetentionContexts.RemoveAt(Index);
	}
}

UURLabRenderServerWorldSubsystem* FURLabRenderServiceBinding::ResolveWorkflowForRetirement() const
{
	check(IsInGameThread());
	return Workflow.Get();
}

TSharedPtr<FURLabRenderServiceBinding, ESPMode::ThreadSafe>
FURLabRenderServiceBinding::Create(UURLabRenderServerWorldSubsystem& Workflow,
	FString& OutError)
{
	check(IsInGameThread());
	if (!Workflow.IsActive())
	{
		OutError = TEXT("render workflow must be active before service binding");
		return nullptr;
	}
	OutError.Reset();
	return MakeShareable(new FURLabRenderServiceBinding(Workflow));
}

UURLabRenderServerWorldSubsystem* FURLabRenderServiceBinding::ResolveWorkflow() const
{
	check(IsInGameThread());
	return IsActive() ? Workflow.Get() : nullptr;
}

bool FURLabRenderServiceBinding::ValidateRecord(
	const FURLabRenderOwnerSessionScopePtr& Scope,
	const FURLabRenderOwnerProof& Proof, const FString& JobId,
	TSharedPtr<FCaptureRecord, ESPMode::ThreadSafe>& OutRecord, FString& OutError)
{
	check(IsInGameThread());
	OutRecord.Reset();
	TSharedPtr<FCaptureRecord, ESPMode::ThreadSafe>* Found = Captures.Find(JobId);
	if (!Found || !Scope || (*Found)->Scope != Scope
		|| (*Found)->Proof.GetGeneration() != Proof.GetGeneration()
		|| (*Found)->Proof.GetOpaqueToken() != Proof.GetOpaqueToken())
	{
		OutError = TEXT("capture job is not owned by this render owner");
		return false;
	}
	OutRecord = *Found;
	OutError.Reset();
	return true;
}

bool FURLabRenderServiceBinding::PollRecord(const FString& JobId,
	const TSharedPtr<FCaptureRecord, ESPMode::ThreadSafe>& Record,
	FURLabRenderCaptureJobSnapshot& OutSnapshot, FString& OutError)
{
	UURLabRenderServerWorldSubsystem* Owner = ResolveWorkflowForRetirement();
	if (!Owner || !Owner->PollCapture(Record->Proof, Record->Handle, OutSnapshot, OutError))
		return false;
	if (Captures.FindRef(JobId) != Record)
	{
		OutError = TEXT("capture job retired during native polling");
		return false;
	}
	if (!OutSnapshot.IsTerminal()) return true;
	if (OutSnapshot.State == EURLabRenderCaptureJobState::Succeeded)
	{
		Record->Materialization = Record->Admission->Materialize(
			Record->Handle.GetSubmittedFrameIdentity(), OutSnapshot);
		Record->MaterializationState = Record->Materialization.bSucceeded
			? EURLabRenderMaterializationState::Succeeded
			: EURLabRenderMaterializationState::Failed;
	}
	else
	{
		Record->Admission->Abort();
		Record->MaterializationState = EURLabRenderMaterializationState::Failed;
		Record->Materialization.Error = OutSnapshot.Message.IsEmpty()
			? TEXT("capture did not succeed") : OutSnapshot.Message;
	}
	return true;
}

bool FURLabRenderServiceBinding::Pump(float)
{
	check(IsInGameThread());
	if (!IsActive()) { PumpHandle.Reset(); return false; }
	TArray<FString> JobIds;
	Captures.GetKeys(JobIds);
	for (const FString& JobId : JobIds)
	{
		const TSharedPtr<FCaptureRecord, ESPMode::ThreadSafe>* Found = Captures.Find(JobId);
		if (!Found) continue;
		const TSharedPtr<FCaptureRecord, ESPMode::ThreadSafe> Record = *Found;
		FURLabRenderCaptureJobSnapshot Snapshot; FString Error;
		if (!PollRecord(JobId, Record, Snapshot, Error))
		{
			UURLabRenderServerWorldSubsystem* Owner = ResolveWorkflowForRetirement();
			if (!Owner)
			{
				Record->Admission->Abort();
				if (Captures.FindRef(JobId) == Record) Captures.Remove(JobId);
				continue;
			}
			const FURLabRenderOwnerAuthorizationResult Authorization = Owner->AuthorizeOwner(
				Record->Scope, Record->Proof.GetGeneration(), Record->Proof.GetOpaqueToken());
			if (Authorization.Error == EURLabRenderOwnerAuthorityError::Unavailable
				|| Authorization.Error == EURLabRenderOwnerAuthorityError::InactiveScope)
				continue;
			if (!Authorization.IsOk())
			{
				const FURLabRenderOwnerLeaseResult Released = Owner->ReleaseOwnerCredentials(
					Record->Scope, Record->Proof.GetGeneration(), Record->Proof.GetOpaqueToken());
				if (Released.Error == EURLabRenderOwnerAuthorityError::Unavailable) continue;
			}
			if (Captures.FindRef(JobId) == Record)
			{
				Record->Admission->Abort();
				Captures.Remove(JobId);
			}
		}
	}
	const bool bKeep = !Captures.IsEmpty();
	if (!bKeep) PumpHandle.Reset();
	return bKeep;
}

void FURLabRenderServiceBinding::EnsurePump()
{
	check(IsInGameThread());
	if (PumpHandle.IsValid()) return;
	const TWeakPtr<FURLabRenderServiceBinding, ESPMode::ThreadSafe> Weak = AsShared();
	PumpHandle = FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateLambda(
		[Weak](float Delta)
		{
			const TSharedPtr<FURLabRenderServiceBinding, ESPMode::ThreadSafe> Self = Weak.Pin();
			return Self ? Self->Pump(Delta) : false;
		}));
}

void FURLabRenderServiceBinding::StopPump()
{
	check(IsInGameThread());
	if (!PumpHandle.IsValid()) return;
	FTSTicker::GetCoreTicker().RemoveTicker(PumpHandle);
	PumpHandle.Reset();
}

bool FURLabRenderServiceBinding::AddCapture(
	const FURLabRenderOwnerSessionScopePtr& Scope, const FURLabRenderOwnerProof& Proof,
	FURLabRenderServerCaptureHandle&& Handle,
	TSharedPtr<FURLabCapturePayloadAdmission, ESPMode::ThreadSafe>&& Admission,
	FString& OutJobId, FString& OutError)
{
	check(IsInGameThread());
	if (!IsActive() || !Scope || !Scope->IsActive() || !Admission)
	{
		OutError = TEXT("render service or capture admission is unavailable"); return false;
	}
	if (Captures.Num() >= 32)
	{
		OutError = TEXT("render service capture table is full"); return false;
	}
	OutJobId = FGuid::NewGuid().ToString(EGuidFormats::DigitsWithHyphens).ToLower();
	TSharedPtr<FCaptureRecord, ESPMode::ThreadSafe> Record =
		MakeShared<FCaptureRecord, ESPMode::ThreadSafe>();
	Record->Scope = Scope; Record->Proof = Proof; Record->Handle = MoveTemp(Handle);
	Record->Admission = MoveTemp(Admission);
	Captures.Add(OutJobId, MoveTemp(Record));
	EnsurePump(); OutError.Reset(); return true;
}

bool FURLabRenderServiceBinding::CanAcceptCapture(
	const FURLabRenderOwnerSessionScopePtr& Scope, FString& OutError) const
{
	check(IsInGameThread());
	if (!IsActive() || !Scope || !Scope->IsActive())
	{
		OutError = TEXT("render service is unavailable"); return false;
	}
	if (Captures.Num() >= 32)
	{
		OutError = TEXT("render service capture table is full"); return false;
	}
	OutError.Reset(); return true;
}

bool FURLabRenderServiceBinding::PollCapture(
	const FURLabRenderOwnerSessionScopePtr& Scope, const FURLabRenderOwnerProof& Proof,
	const FString& JobId, FURLabRenderServiceCaptureView& OutView, FString& OutError)
{
	check(IsInGameThread());
	TSharedPtr<FCaptureRecord, ESPMode::ThreadSafe> Record;
	if (!ValidateRecord(Scope, Proof, JobId, Record, OutError)) return false;
	FURLabRenderCaptureJobSnapshot Snapshot;
	if (!PollRecord(JobId, Record, Snapshot, OutError)) return false;
	OutView.Native = MoveTemp(Snapshot); OutView.MaterializationState = Record->MaterializationState;
	OutView.SubmittedFrame = Record->Handle.GetSubmittedFrameIdentity();
	OutView.Materialization = Record->Materialization; return true;
}

bool FURLabRenderServiceBinding::CancelCapture(
	const FURLabRenderOwnerSessionScopePtr& Scope, const FURLabRenderOwnerProof& Proof,
	const FString& JobId, bool& OutCancelled, FString& OutError)
{
	check(IsInGameThread()); OutCancelled = false;
	TSharedPtr<FCaptureRecord, ESPMode::ThreadSafe> Record;
	if (!ValidateRecord(Scope, Proof, JobId, Record, OutError)) return false;
	UURLabRenderServerWorldSubsystem* Owner = ResolveWorkflow();
	if (!Owner) { OutError = TEXT("render service is unavailable"); return false; }
	FURLabRenderCaptureJobSnapshot Snapshot;
	if (!Owner->PollCapture(Proof, Record->Handle, Snapshot, OutError)) return false;
	if (Snapshot.IsTerminal()) return true;
	if (!Owner->CancelCapture(Proof, Record->Handle, OutError)) return false;
	OutCancelled = true; EnsurePump(); return true;
}

bool FURLabRenderServiceBinding::ReleaseCapture(
	const FURLabRenderOwnerSessionScopePtr& Scope, const FURLabRenderOwnerProof& Proof,
	const FString& JobId, FString& OutError)
{
	check(IsInGameThread()); TSharedPtr<FCaptureRecord, ESPMode::ThreadSafe> Record;
	if (!ValidateRecord(Scope, Proof, JobId, Record, OutError)) return false;
	UURLabRenderServerWorldSubsystem* Owner = ResolveWorkflow();
	if (!Owner || !Owner->ReleaseCapture(Proof, Record->Handle, OutError)) return false;
	Record->Admission->Abort(); Captures.Remove(JobId);
	if (Captures.IsEmpty()) StopPump(); return true;
}

FURLabRenderOwnerLeaseResult FURLabRenderServiceBinding::RetireScope(
	const FURLabRenderOwnerSessionScopePtr& Scope)
{
	check(IsInGameThread());
	for (int32 Index = RetentionContexts.Num() - 1; Index >= 0; --Index)
	{
		if (RetentionContexts[Index].Scope != Scope) continue;
		if (UURLabRenderServerWorldSubsystem* Owner = ResolveWorkflowForRetirement())
			Owner->RetireCaptureHistory(RetentionContexts[Index].Id);
		RetentionContexts.RemoveAt(Index);
	}
	FURLabRenderOwnerLeaseResult Result;
	if (UURLabRenderServerWorldSubsystem* Owner = ResolveWorkflowForRetirement())
		Result = Owner->RetireOwnerScope(Scope);
	if (Result.Error == EURLabRenderOwnerAuthorityError::Unavailable) return Result;
	TArray<FString> JobIds;
	Captures.GetKeys(JobIds);
	for (const FString& JobId : JobIds)
	{
		const TSharedPtr<FCaptureRecord, ESPMode::ThreadSafe> Record = Captures.FindRef(JobId);
		if (Record && Record->Scope == Scope)
		{
			Record->Admission->Abort();
			if (Captures.FindRef(JobId) == Record) Captures.Remove(JobId);
		}
	}
	if (Captures.IsEmpty()) StopPump(); return Result;
}

void FURLabRenderServiceBinding::RetireAttachment(
	const FURLabRenderOwnerSessionScopePtr& Scope, uint64 Generation, const FString& Token)
{
	check(IsInGameThread());
	for (int32 Index = RetentionContexts.Num() - 1; Index >= 0; --Index)
	{
		const FRetentionContext& Context = RetentionContexts[Index];
		if (Context.Scope != Scope || Context.Generation != Generation || Context.Token != Token)
			continue;
		if (UURLabRenderServerWorldSubsystem* Owner = ResolveWorkflowForRetirement())
			Owner->RetireCaptureHistory(Context.Id);
		RetentionContexts.RemoveAt(Index);
	}
	TArray<FString> JobIds;
	Captures.GetKeys(JobIds);
	for (const FString& JobId : JobIds)
	{
		const TSharedPtr<FCaptureRecord, ESPMode::ThreadSafe> Record = Captures.FindRef(JobId);
		if (Record && Record->Scope == Scope && Record->Proof.GetGeneration() == Generation
			&& Record->Proof.GetOpaqueToken() == Token)
		{
			Record->Admission->Abort();
			if (Captures.FindRef(JobId) == Record) Captures.Remove(JobId);
		}
	}
	if (Captures.IsEmpty()) StopPump();
}
