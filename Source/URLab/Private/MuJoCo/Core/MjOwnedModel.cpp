// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#include "MuJoCo/Core/MjOwnedModel.h"

#include "MuJoCo/Core/MjModelRuntime.h"
#include "MuJoCo/Entity/MjModelSource.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace
{
bool AddAssetSizes(const TMap<FString, TArray<uint8>>& Assets,
	FMjModelSourceInfo& Info, FString& OutError)
{
	Info.AssetCount = Assets.Num();
	Info.AssetByteCount = 0;
	for (const TPair<FString, TArray<uint8>>& Asset : Assets)
	{
		if (Asset.Key.IsEmpty())
		{
			OutError = TEXT("model source contains an empty asset name");
			return false;
		}
		if (Asset.Value.Num() == 0)
		{
			OutError = FString::Printf(TEXT("model source asset '%s' is empty"), *Asset.Key);
			return false;
		}
		if (Info.AssetByteCount > MAX_int64 - Asset.Value.Num())
		{
			OutError = TEXT("model source asset byte count overflowed");
			return false;
		}
		Info.AssetByteCount += Asset.Value.Num();
	}
	return true;
}

}

FMjOwnedModelCandidate::FMjOwnedModelCandidate() = default;

FMjOwnedModelCandidate::~FMjOwnedModelCandidate()
{
	Reset();
}

FMjOwnedModelCandidate::FMjOwnedModelCandidate(FMjOwnedModelCandidate&& Other) noexcept
	: Model(Other.Model)
	, Data(Other.Data)
	, Descriptor(MoveTemp(Other.Descriptor))
	, SourceInfo(MoveTemp(Other.SourceInfo))
{
	Other.Model = nullptr;
	Other.Data = nullptr;
}

FMjOwnedModelCandidate& FMjOwnedModelCandidate::operator=(FMjOwnedModelCandidate&& Other) noexcept
{
	if (this != &Other)
	{
		Reset();
		Model = Other.Model;
		Data = Other.Data;
		Descriptor = MoveTemp(Other.Descriptor);
		SourceInfo = MoveTemp(Other.SourceInfo);
		Other.Model = nullptr;
		Other.Data = nullptr;
	}
	return *this;
}

void FMjOwnedModelCandidate::Reset()
{
	if (Data)
	{
		mj_deleteData(Data);
		Data = nullptr;
	}
	if (Model)
	{
		mj_deleteModel(Model);
		Model = nullptr;
	}
	Descriptor = FMjModelDescriptor();
	SourceInfo = FMjModelSourceInfo();
}

bool FMjOwnedModelCandidate::IsValid() const
{
	return Model != nullptr && Data != nullptr && Model->signature == Data->signature
		&& Descriptor.Identity.Generation > 0;
}

bool FMjOwnedModelCandidate::VisitPrepared(
	TFunctionRef<bool(const mjModel*, const mjData*)> Visitor) const
{
	return IsValid() && Visitor(Model, Data);
}

bool FMjOwnedModelCandidate::PrepareOwned(mjModel* OwnedModel,
	FMjModelSourceInfo&& InSourceInfo,
	FMjOwnedModelCandidate& OutCandidate, FString& OutError)
{
	if (!OwnedModel)
	{
		if (OutError.IsEmpty()) OutError = TEXT("model source produced no model");
		return false;
	}
	mjData* OwnedData = mj_makeData(OwnedModel);
	if (!OwnedData)
	{
		mj_deleteModel(OwnedModel);
		OutError = TEXT("mj_makeData failed for the candidate model");
		return false;
	}

	// Establish the render/sensor-ready initial state. This is FK/derived refresh,
	// not integration: candidate construction never calls mj_step.
	mj_forward(OwnedModel, OwnedData);
	FMjModelDescriptor NewDescriptor;
	if (!MjModelRuntime::BuildDescriptor(OwnedModel, /*temporary generation=*/1,
		NewDescriptor, OutError))
	{
		mj_deleteData(OwnedData);
		mj_deleteModel(OwnedModel);
		return false;
	}
	FMjOwnedModelCandidate NewCandidate;
	NewCandidate.Model = OwnedModel;
	NewCandidate.Data = OwnedData;
	NewCandidate.Descriptor = MoveTemp(NewDescriptor);
	NewCandidate.SourceInfo = MoveTemp(InSourceInfo);
	OutCandidate = MoveTemp(NewCandidate);
	OutError.Reset();
	return true;
}

bool FMjOwnedModelCandidate::FromBytes(EMjModelSourceKind Kind,
	const TArray<uint8>& Bytes, const TMap<FString, TArray<uint8>>& Assets,
	FMjOwnedModelCandidate& OutCandidate, FString& OutError, const FString& SourceLabel)
{
	const TCHAR* Format = nullptr;
	switch (Kind)
	{
	case EMjModelSourceKind::Mjb: Format = TEXT("mjb"); break;
	case EMjModelSourceKind::Mjz: Format = TEXT("mjz"); break;
	case EMjModelSourceKind::XmlWithAssets: Format = TEXT("xml"); break;
	case EMjModelSourceKind::CompiledModel:
		OutError = TEXT("compiled input must use CopyCompiled");
		return false;
	default:
		OutError = TEXT("unknown model source kind");
		return false;
	}
	if (Kind == EMjModelSourceKind::Mjb && Assets.Num() > 0)
	{
		OutError = TEXT("MJB input does not accept sidecar assets");
		return false;
	}

	FMjModelSourceInfo Info;
	Info.Kind = Kind;
	Info.Label = SourceLabel;
	Info.PrimaryByteCount = Bytes.Num();
	if (!AddAssetSizes(Assets, Info, OutError)) return false;
	mjModel* Model = MjModelSource::FromBytes(Bytes, Format, Assets, OutError);
	return FMjOwnedModelCandidate::PrepareOwned(Model, MoveTemp(Info), OutCandidate, OutError);
}

bool FMjOwnedModelCandidate::CopyCompiled(const mjModel* Source,
	FMjOwnedModelCandidate& OutCandidate, FString& OutError, const FString& SourceLabel)
{
	OutError.Reset();
	if (!Source)
	{
		OutError = TEXT("cannot copy a null compiled model");
		return false;
	}
	mjModel* Copy = mj_copyModel(nullptr, Source);
	if (!Copy)
	{
		OutError = TEXT("mj_copyModel failed");
		return false;
	}
	FMjModelSourceInfo Info;
	Info.Kind = EMjModelSourceKind::CompiledModel;
	Info.Label = SourceLabel;
	return FMjOwnedModelCandidate::PrepareOwned(Copy, MoveTemp(Info), OutCandidate, OutError);
}
