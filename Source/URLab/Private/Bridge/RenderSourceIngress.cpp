// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "Bridge/RenderSourceIngress.h"

#include "Misc/ScopeExit.h"
#include "Utils/AssetCache.h"

namespace
{
struct FPinnedInput
{
	FString Name;
	FURLabPayloadPin Pin;
	FURLabStoredPayloadDescriptor Descriptor;
};

bool Fail(FString& OutError, const FString& Message)
{
	OutError = Message;
	return false;
}

bool CheckedAdd(int64 Value, int64& Total)
{
	if (Value <= 0 || Total > MAX_int64 - Value) return false;
	Total += Value;
	return true;
}

bool PinExact(FURLabPayloadStore& Store, const FURLabPayloadOwner& Owner,
	const FURLabRenderSourceBlobRef& Ref, const FString& Name,
	FPinnedInput& Out, FString& OutError)
{
	if (Ref.BlobId.IsEmpty() || !FURLabAssetCache::IsValidSha256Hex(Ref.Sha256))
		return Fail(OutError, FString::Printf(TEXT("%s requires a blob id and canonical sha256"), *Name));
	FURLabPayloadPin Pin;
	FURLabPayloadStoreStatus Status = Store.Pin(Owner, Ref.BlobId, Pin);
	if (!Status) return Fail(OutError, Status.Message);
	FURLabStoredPayloadDescriptor Descriptor;
	Status = Store.GetDescriptor(Owner, Ref.BlobId, Descriptor);
	if (!Status) return Fail(OutError, Status.Message);
	if (!Pin || Pin->Num() <= 0 || Pin->GetSha256() != Ref.Sha256
		|| Descriptor.BlobId != Ref.BlobId || Descriptor.Sha256 != Ref.Sha256
		|| Descriptor.ByteLength != Pin->Num()
		|| Descriptor.BootId != Owner.BootId || Descriptor.SessionId != Owner.SessionId)
	{
		return Fail(OutError, FString::Printf(TEXT("%s payload identity or hash mismatch"), *Name));
	}
	Out.Name = Name;
	Out.Pin = MoveTemp(Pin);
	Out.Descriptor = MoveTemp(Descriptor);
	return true;
}

bool CopyPinned(FURLabPayloadStore& Store, const FPinnedInput& Input,
	TArray<uint8>& OutBytes, FString& OutError)
{
	const int32 Length = static_cast<int32>(Input.Pin->Num());
	OutBytes.SetNumUninitialized(Length, EAllowShrinking::No);
	TConstArrayView<uint8> Network;
	if (Input.Pin->TryGetNetworkBytes(Network))
	{
		if (Network.Num() != Length)
			return Fail(OutError, TEXT("network payload pin length changed"));
		FMemory::Memcpy(OutBytes.GetData(), Network.GetData(), Length);
		return true;
	}

	const int32 ChunkLimit = Store.GetLimits().MaxChunkBytes;
	for (int64 Offset = 0; Offset < Length;)
	{
		const int64 ChunkLength = FMath::Min<int64>(ChunkLimit, Length - Offset);
		FURLabPayloadCopyPin Chunk;
		const FURLabPayloadStoreStatus Status = Store.CopyPinnedRange(
			Input.Pin, Offset, ChunkLength, Chunk);
		if (!Status) return Fail(OutError, Status.Message);
		if (!Chunk || Chunk->Num() != ChunkLength)
			return Fail(OutError, TEXT("SHM copied range length changed"));
		FMemory::Memcpy(OutBytes.GetData() + Offset, Chunk->GetBytes().GetData(), ChunkLength);
		Offset += ChunkLength;
	}
	return true;
}
}

bool FURLabRenderSourceIngress::Prepare(
	const TSharedPtr<FURLabPayloadStore, ESPMode::ThreadSafe>& Store,
	const FURLabPayloadOwner& Owner, const FURLabRenderSourceRequest& Request,
	const FString& SceneRevision, const FMjRenderKinematicsOptions& Options,
	FMjRenderKinematicsCandidate& OutCandidate, FString& OutError)
{
	OutError.Reset();
	if (!Store || Owner.BootId.IsEmpty() || Owner.SessionId.IsEmpty()
		|| !Owner.Session || SceneRevision.IsEmpty())
		return Fail(OutError, TEXT("source ingress requires store, owner, and scene revision"));
	bool bPrepared = false;
	ON_SCOPE_EXIT
	{
		if (!bPrepared) ReleaseReferencedPayloads(*Store, Owner, Request);
	};
	if (Request.Label.Len() > MaxLabelChars)
		return Fail(OutError, TEXT("source label exceeds its bound"));
	if (Request.Assets.Num() > MaxAssets)
		return Fail(OutError, FString::Printf(TEXT("source has %d assets; at most %d are accepted"),
			Request.Assets.Num(), MaxAssets));
	if (Request.Kind != EMjModelSourceKind::Mjb
		&& Request.Kind != EMjModelSourceKind::Mjz
		&& Request.Kind != EMjModelSourceKind::XmlWithAssets)
		return Fail(OutError, TEXT("source ingress kind is unsupported"));
	if (Request.Kind != EMjModelSourceKind::XmlWithAssets && !Request.Assets.IsEmpty())
		return Fail(OutError, TEXT("only XML sources accept sidecar assets"));

	TSet<FString> Names;
	for (const FURLabRenderSourceAssetRef& Asset : Request.Assets)
	{
		if (Asset.Name.IsEmpty() || Asset.Name.Len() > MaxAssetNameChars
			|| Names.Contains(Asset.Name))
			return Fail(OutError, TEXT("source asset name is empty, duplicate, or too long"));
		Names.Add(Asset.Name);
	}

	TArray<FPinnedInput> Inputs;
	Inputs.Reserve(1 + Request.Assets.Num());
	FPinnedInput Primary;
	if (!PinExact(*Store, Owner, Request.Primary, TEXT("primary"), Primary, OutError))
		return false;
	Inputs.Add(MoveTemp(Primary));
	for (const FURLabRenderSourceAssetRef& Asset : Request.Assets)
	{
		FPinnedInput Input;
		if (!PinExact(*Store, Owner, Asset.Payload,
			FString::Printf(TEXT("asset '%s'"), *Asset.Name), Input, OutError)) return false;
		Input.Name = Asset.Name;
		Inputs.Add(MoveTemp(Input));
	}

	int64 AggregateBytes = 0;
	for (const FPinnedInput& Input : Inputs)
		if (!CheckedAdd(Input.Pin->Num(), AggregateBytes))
			return Fail(OutError, TEXT("source aggregate byte count overflows"));
	const FURLabPayloadStoreLimits& Limits = Store->GetLimits();
	if (AggregateBytes > Limits.MaxSessionBytes || Inputs.Num() > Limits.MaxSessionBlobCount)
		return Fail(OutError, TEXT("source aggregate exceeds payload-store session bounds"));

	FMjModelSourcePayload Source;
	Source.Kind = Request.Kind;
	Source.Label = Request.Label;
	if (!CopyPinned(*Store, Inputs[0], Source.PrimaryBytes, OutError)) return false;
	for (int32 Index = 1; Index < Inputs.Num(); ++Index)
	{
		TArray<uint8> Bytes;
		if (!CopyPinned(*Store, Inputs[Index], Bytes, OutError)) return false;
		Source.Assets.Add(Inputs[Index].Name, MoveTemp(Bytes));
	}

	FMjRenderKinematicsCandidate Prepared;
	if (!FMjRenderKinematicsCandidate::Prepare(MoveTemp(Source), SceneRevision,
		Options, Prepared, OutError)) return false;
	OutCandidate = MoveTemp(Prepared);
	bPrepared = true;
	return true;
}

void FURLabRenderSourceIngress::ReleaseReferencedPayloads(FURLabPayloadStore& Store,
	const FURLabPayloadOwner& Owner, const FURLabRenderSourceRequest& Request)
{
	TSet<FString> Released;
	const auto ReleaseOnce = [&Store, &Owner, &Released](const FString& BlobId)
	{
		if (BlobId.IsEmpty() || Released.Contains(BlobId)) return;
		Released.Add(BlobId);
		(void)Store.Release(Owner, BlobId);
	};
	ReleaseOnce(Request.Primary.BlobId);
	for (const FURLabRenderSourceAssetRef& Asset : Request.Assets)
	{
		ReleaseOnce(Asset.Payload.BlobId);
	}
}
