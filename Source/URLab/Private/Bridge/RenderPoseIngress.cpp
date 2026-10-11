// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "Bridge/RenderPoseIngress.h"
#include "Bridge/NeutralRenderSourceIngress.h"

#include "Containers/ContainerAllocationPolicies.h"
#include "Misc/ByteSwap.h"
#include "Utils/AssetCache.h"
#include "Utils/CoordinateConversion.h"

namespace
{
class FPresentationLease final : public IRenderCameraCaptureLifetimeLease
{
public:
	explicit FPresentationLease(FURLabPayloadPhysicalReservationPtr&& InReservation)
		: Reservation(MoveTemp(InReservation)) {}
private:
	FURLabPayloadPhysicalReservationPtr Reservation;
};

class FCompositeLease final : public IRenderCameraCaptureLifetimeLease
{
public:
	FCompositeLease(FRenderCameraCaptureLifetimeLeasePtr&& InFirst,
		FRenderCameraCaptureLifetimeLeasePtr&& InSecond)
		: First(MoveTemp(InFirst)), Second(MoveTemp(InSecond)) {}
private:
	FRenderCameraCaptureLifetimeLeasePtr First;
	FRenderCameraCaptureLifetimeLeasePtr Second;
};

bool Fail(FString& OutError, const FString& Message)
{
	OutError = Message;
	return false;
}

bool CheckedAdd(int64 Value, int64& Total)
{
	if (Value < 0 || Total > MAX_int64 - Value) return false;
	Total += Value;
	return true;
}

bool CheckedMultiply(int64 A, int64 B, int64& Out)
{
	if (A < 0 || B < 0 || (A != 0 && B > MAX_int64 / A)) return false;
	Out = A * B;
	return true;
}

template <typename T>
bool ArrayPhysicalBytes(int32 Count, int64& Out)
{
	if (Count < 0) return false;
	const int32 ReserveCapacity = DefaultCalculateSlackReserve<int32>(
		Count, sizeof(T), true, alignof(T));
	const int32 GrowCapacity = Count == 0 ? 0 : DefaultCalculateSlackGrow<int32>(
		Count, 0, sizeof(T), true, alignof(T));
	const int32 Capacity = FMath::Max(ReserveCapacity, GrowCapacity);
	return CheckedMultiply(Capacity, static_cast<int64>(sizeof(T)), Out);
}

bool PinExact(FURLabPayloadStore& Store, const FURLabPayloadOwner& Owner,
	const FURLabRenderSourceBlobRef& Ref, int64 ExpectedBytes,
	FURLabPayloadPin& OutPin, FString& OutError)
{
	if (Ref.BlobId.IsEmpty() || !FURLabAssetCache::IsValidSha256Hex(Ref.Sha256))
		return Fail(OutError, TEXT("pose payload requires a blob id and canonical sha256"));
	FURLabPayloadStoreStatus Status = Store.Pin(Owner, Ref.BlobId, OutPin);
	if (!Status) return Fail(OutError, Status.Message);
	FURLabStoredPayloadDescriptor Descriptor;
	Status = Store.GetDescriptor(Owner, Ref.BlobId, Descriptor);
	if (!Status) return Fail(OutError, Status.Message);
	if (!OutPin || OutPin->Num() != ExpectedBytes || OutPin->GetSha256() != Ref.Sha256
		|| Descriptor.BlobId != Ref.BlobId || Descriptor.Sha256 != Ref.Sha256
		|| Descriptor.ByteLength != ExpectedBytes || Descriptor.BootId != Owner.BootId
		|| Descriptor.SessionId != Owner.SessionId)
		return Fail(OutError, TEXT("pose payload identity, hash, owner, or length mismatch"));
	return true;
}

class FDoubleReader
{
public:
	FDoubleReader(FURLabPayloadStore& InStore, FURLabPayloadPin InPin)
		: Store(InStore), Pin(MoveTemp(InPin))
	{
		Pin->TryGetNetworkBytes(Network);
	}

	bool Read(int32 Index, double& Out, FString& OutError)
	{
		const int64 Offset = static_cast<int64>(Index) * sizeof(double);
		const uint8* Bytes = nullptr;
		if (!Network.IsEmpty()) Bytes = Network.GetData() + Offset;
		else
		{
			const int32 RawLimit = Store.GetLimits().MaxChunkBytes;
			const int64 AlignedLimit = (static_cast<int64>(RawLimit) / sizeof(double))
				* sizeof(double);
			if (AlignedLimit < static_cast<int64>(sizeof(double)))
				return Fail(OutError, TEXT("payload-store chunk limit is smaller than one float64"));
			if (!Chunk || Offset < ChunkOffset || Offset + sizeof(double) > ChunkEnd)
			{
				const int64 Length = FMath::Min<int64>(AlignedLimit, Pin->Num() - Offset);
				const FURLabPayloadStoreStatus Status = Store.CopyPinnedRange(
					Pin, Offset, Length, Chunk);
				if (!Status) return Fail(OutError, Status.Message);
				if (!Chunk || Chunk->Num() != Length)
					return Fail(OutError, TEXT("pose SHM range length changed"));
				ChunkOffset = Offset;
				ChunkEnd = Offset + Length;
			}
			Bytes = Chunk->GetBytes().GetData() + (Offset - ChunkOffset);
		}
		uint64 Bits = 0;
		FMemory::Memcpy(&Bits, Bytes, sizeof(Bits));
		Bits = INTEL_ORDER64(Bits);
		FMemory::Memcpy(&Out, &Bits, sizeof(Out));
		return true;
	}
private:
	FURLabPayloadStore& Store;
	FURLabPayloadPin Pin;
	TConstArrayView<uint8> Network;
	FURLabPayloadCopyPin Chunk;
	int64 ChunkOffset = 0;
	int64 ChunkEnd = 0;
};
}

FURLabDecodedRenderPose::FURLabDecodedRenderPose(
	FURLabDecodedRenderPose&& Other) noexcept
{
	*this = MoveTemp(Other);
}

FURLabDecodedRenderPose& FURLabDecodedRenderPose::operator=(
	FURLabDecodedRenderPose&& Other) noexcept
{
	if (this == &Other) return *this;
	Reset();
	PresentationLease = MoveTemp(Other.PresentationLease);
	Kind = Other.Kind;
	Qpos = MoveTemp(Other.Qpos);
	Cartesian = MoveTemp(Other.Cartesian);
	return *this;
}

void FURLabDecodedRenderPose::Reset()
{
	Qpos.Empty();
	Cartesian.BodySceneTransforms.Empty();
	Cartesian.CameraSceneTransforms.Empty();
	Cartesian.UserCameraSceneTransform.Reset();
	Cartesian.LightSceneTransforms.Empty();
	Cartesian.Deformables.Empty();
	Cartesian.DebugMeshes.Empty();
	Cartesian.Identity = FRenderFrameIdentity();
	PresentationLease.Reset();
	Kind = EURLabRenderPoseKind::Qpos;
}

FRenderCameraCaptureLifetimeLeasePtr FURLabRenderPoseIngress::ComposeLifetimeLeases(
	FRenderCameraCaptureLifetimeLeasePtr First,
	FRenderCameraCaptureLifetimeLeasePtr Second)
{
	if (!First) return Second;
	if (!Second) return First;
	return MakeShared<FCompositeLease, ESPMode::ThreadSafe>(MoveTemp(First), MoveTemp(Second));
}

bool FURLabRenderPoseIngress::ReserveCartesianPresentation(
	const TSharedPtr<FURLabPayloadStore, ESPMode::ThreadSafe>& Store,
	const FURLabPayloadOwner& Owner, int32 BodyCount, int32 CameraCount,
	bool bHasUserCamera, int32 LightCount,
	FRenderCameraCaptureLifetimeLeasePtr& OutLease,
	FString& OutError)
{
	OutError.Reset();
	if (!Store || Owner.BootId.IsEmpty() || Owner.SessionId.IsEmpty() || !Owner.Session)
		return Fail(OutError, TEXT("Cartesian presentation requires an exact store owner"));
	int64 BodyBytes=0, CameraBytes=0, LightBytes=0, FrameBytes=0, ReservedBytes=0;
	if (!ArrayPhysicalBytes<FTransform>(BodyCount, BodyBytes)
		|| !ArrayPhysicalBytes<FTransform>(CameraCount, CameraBytes)
		|| !ArrayPhysicalBytes<FTransform>(LightCount, LightBytes)
		|| !CheckedAdd(BodyBytes, FrameBytes) || !CheckedAdd(CameraBytes, FrameBytes)
		|| !CheckedAdd(LightBytes, FrameBytes)
		|| (bHasUserCamera && !CheckedAdd(static_cast<int64>(sizeof(FTransform)), FrameBytes))
		|| !CheckedMultiply(FrameBytes, 4, ReservedBytes))
		return Fail(OutError, TEXT("Cartesian presentation allocation bound overflows"));
	FURLabPayloadPhysicalReservationPtr Reservation;
	if (ReservedBytes > 0)
	{
		const FURLabPayloadStoreStatus Status=Store->ReservePhysicalCapacity(Owner,ReservedBytes,Reservation);
		if (!Status) return Fail(OutError,Status.Message);
	}
	OutLease=MakeShared<FPresentationLease,ESPMode::ThreadSafe>(MoveTemp(Reservation));
	return true;
}

bool FURLabRenderPoseIngress::ReserveGeomPoseOverrides(
	const TSharedPtr<FURLabPayloadStore, ESPMode::ThreadSafe>& Store,
	const FURLabPayloadOwner& Owner, int32 Count,
	FRenderCameraCaptureLifetimeLeasePtr& OutLease, FString& OutError)
{
	OutLease.Reset(); OutError.Reset();
	if (!Store || Owner.BootId.IsEmpty() || Owner.SessionId.IsEmpty() || !Owner.Session)
		return Fail(OutError,
		TEXT("geometry overrides require an exact store owner"));
	int64 Bytes = 0;
	if (!ArrayPhysicalBytes<FRenderGeomPoseOverride>(Count, Bytes))
		return Fail(OutError, TEXT("geometry override allocation bound overflows"));
	FURLabPayloadPhysicalReservationPtr Reservation;
	if (Bytes > 0)
	{
		const FURLabPayloadStoreStatus Status = Store->ReservePhysicalCapacity(
			Owner, Bytes, Reservation);
		if (!Status) return Fail(OutError, Status.Message);
	}
	OutLease = MakeShared<FPresentationLease, ESPMode::ThreadSafe>(MoveTemp(Reservation));
	return true;
}

bool FURLabRenderPoseIngress::Decode(
	const TSharedPtr<FURLabPayloadStore, ESPMode::ThreadSafe>& Store,
	const FURLabPayloadOwner& Owner, const FURLabRenderSourceDescription& Description,
	const FURLabRenderPoseRequest& Request, FURLabDecodedRenderPose& OutPose,
	FString& OutError)
{
	OutError.Reset();
	if (!Store || Owner.BootId.IsEmpty() || Owner.SessionId.IsEmpty() || !Owner.Session
		|| Description.SceneRevision.IsEmpty() || Description.SceneGeneration == 0
		|| Description.QposWidth < 0 || Request.Identity.SceneRevision != Description.SceneRevision
		|| (Request.Identity.bHasSourceTime && !FMath::IsFinite(Request.Identity.SourceTime)))
		return Fail(OutError, TEXT("pose ingress requires an exact store owner and source description"));
	if (Request.Kind != EURLabRenderPoseKind::Qpos
		&& Request.Kind != EURLabRenderPoseKind::Cartesian)
		return Fail(OutError, TEXT("pose kind is unsupported"));
	if (Request.Kind == EURLabRenderPoseKind::Qpos && !Description.bSupportsQpos)
		return Fail(OutError, TEXT("installed source does not support qpos"));
	if (Request.Kind != EURLabRenderPoseKind::Cartesian && Request.Visualization.IsSet())
		return Fail(OutError, TEXT("dynamic visualization requires Cartesian pose"));

	const int64 BodyCount = Description.BodyNames.Num();
	const int64 CameraCount = Description.ModelCameras.Num();
	const int64 LightCount = Description.LightNames.Num();
	int64 TransformCount = 0;
	if (!CheckedAdd(BodyCount, TransformCount) || !CheckedAdd(CameraCount, TransformCount)
		|| (Description.UserCamera.IsSet() && !CheckedAdd(1, TransformCount))
		|| !CheckedAdd(LightCount, TransformCount))
		return Fail(OutError, TEXT("Cartesian transform count overflows"));
	int64 ScalarCount = Request.Kind == EURLabRenderPoseKind::Qpos
		? Description.QposWidth : 0;
	if (Request.Kind == EURLabRenderPoseKind::Cartesian
		&& !CheckedMultiply(TransformCount, 7, ScalarCount))
		return Fail(OutError, TEXT("Cartesian scalar count overflows"));
	int64 ExpectedBytes = 0;
	if (!CheckedMultiply(ScalarCount, sizeof(double), ExpectedBytes)
		|| ExpectedBytes > MAX_int32)
		return Fail(OutError, TEXT("pose payload length is not representable"));

	if (ExpectedBytes == 0)
	{
		if (Request.Payload.IsSet())
			return Fail(OutError, TEXT("zero-width pose must omit its payload"));
	}
	else if (!Request.Payload.IsSet())
		return Fail(OutError, TEXT("pose payload is required"));

	int64 QposBytes = 0, BodyBytes = 0, CameraBytes = 0, LightBytes = 0, FrameBytes = 0;
	if (!ArrayPhysicalBytes<double>(Request.Kind == EURLabRenderPoseKind::Qpos
		? Description.QposWidth : 0, QposBytes)
		|| !ArrayPhysicalBytes<FTransform>(Description.BodyNames.Num(), BodyBytes)
		|| !ArrayPhysicalBytes<FTransform>(Description.ModelCameras.Num(), CameraBytes)
		|| !ArrayPhysicalBytes<FTransform>(Description.LightNames.Num(), LightBytes)
		|| !CheckedAdd(BodyBytes, FrameBytes) || !CheckedAdd(CameraBytes, FrameBytes)
		|| !CheckedAdd(LightBytes, FrameBytes)
		|| (Description.UserCamera.IsSet()
			&& !CheckedAdd(static_cast<int64>(sizeof(FTransform)), FrameBytes)))
		return Fail(OutError, TEXT("pose allocation bound overflows"));
	int64 FourFrames = 0, ReservedBytes = QposBytes;
	if (!CheckedMultiply(FrameBytes, 4, FourFrames) || !CheckedAdd(FourFrames, ReservedBytes))
		return Fail(OutError, TEXT("pose retained allocation bound overflows"));

	FURLabPayloadPhysicalReservationPtr Reservation;
	if (ReservedBytes > 0)
	{
		const FURLabPayloadStoreStatus Status = Store->ReservePhysicalCapacity(
			Owner, ReservedBytes, Reservation);
		if (!Status) return Fail(OutError, Status.Message);
	}
	FURLabDecodedRenderPose Candidate;
	Candidate.PresentationLease = MakeShared<FPresentationLease, ESPMode::ThreadSafe>(
		MoveTemp(Reservation));
	Candidate.Kind = Request.Kind;
	Candidate.Cartesian.Identity = Request.Identity;
	if (Request.Kind == EURLabRenderPoseKind::Cartesian && Request.Visualization.IsSet())
	{
		FRenderCameraCaptureLifetimeLeasePtr DynamicLease;
		if (!FURLabNeutralRenderSourceIngress::DecodeFrameVisualization(
			Store, Owner, Request.Visualization.GetValue(), Description.Deformables,
			Candidate.Cartesian, DynamicLease, OutError)) return false;
		Candidate.PresentationLease = ComposeLifetimeLeases(
			MoveTemp(Candidate.PresentationLease), MoveTemp(DynamicLease));
	}
	else if (Request.Kind == EURLabRenderPoseKind::Cartesian && !Description.Deformables.IsEmpty())
		return Fail(OutError, TEXT("Cartesian frame must include installed deformables"));
	if (ExpectedBytes == 0)
	{
		OutPose = MoveTemp(Candidate);
		return true;
	}

	FURLabPayloadPin Pin;
	if (!PinExact(*Store, Owner, Request.Payload.GetValue(), ExpectedBytes, Pin, OutError))
		return false;
	FDoubleReader Reader(*Store, MoveTemp(Pin));
	if (Request.Kind == EURLabRenderPoseKind::Qpos)
	{
		Candidate.Qpos.Reserve(Description.QposWidth);
		Candidate.Qpos.SetNumUninitialized(Description.QposWidth, EAllowShrinking::No);
		for (int32 Index = 0; Index < Candidate.Qpos.Num(); ++Index)
			if (!Reader.Read(Index, Candidate.Qpos[Index], OutError)
				|| !FMath::IsFinite(Candidate.Qpos[Index]))
				return Fail(OutError, OutError.IsEmpty()
					? TEXT("qpos contains a non-finite value") : OutError);
	}
	else
	{
		Candidate.Cartesian.BodySceneTransforms.Reserve(Description.BodyNames.Num());
		Candidate.Cartesian.CameraSceneTransforms.Reserve(Description.ModelCameras.Num());
		Candidate.Cartesian.LightSceneTransforms.Reserve(Description.LightNames.Num());
		int32 Scalar = 0;
		auto DecodeTransform = [&](bool bOpticalCamera, FTransform& Out) -> bool
		{
			double Values[7];
			for (double& Value : Values)
				if (!Reader.Read(Scalar++, Value, OutError) || !FMath::IsFinite(Value))
					return Fail(OutError, OutError.IsEmpty()
						? TEXT("Cartesian pose contains a non-finite value") : OutError);
			const double NormSquared = Values[3] * Values[3] + Values[4] * Values[4]
				+ Values[5] * Values[5] + Values[6] * Values[6];
			const double Norm = FMath::Sqrt(NormSquared);
			if (!FMath::IsFinite(Norm) || FMath::Abs(Norm - 1.0) > QuaternionNormTolerance)
				return Fail(OutError, TEXT("Cartesian quaternion is not near unit length"));
			for (int32 Index = 3; Index < 7; ++Index) Values[Index] /= Norm;
			Out = bOpticalCamera
				? URLabCoordinateConversion::RightHandedOpticalPoseToUnreal(Values, Values + 3)
				: FTransform(
					URLabCoordinateConversion::RightHandedWxyzQuaternionToUnreal(Values + 3),
					URLabCoordinateConversion::RightHandedMetresPositionToUnrealCentimetres(Values),
					FVector::OneVector);
			const FVector Translation = Out.GetTranslation();
			const FQuat Rotation = Out.GetRotation();
			if (!FMath::IsFinite(Translation.X) || !FMath::IsFinite(Translation.Y)
				|| !FMath::IsFinite(Translation.Z) || !FMath::IsFinite(Rotation.X)
				|| !FMath::IsFinite(Rotation.Y) || !FMath::IsFinite(Rotation.Z)
				|| !FMath::IsFinite(Rotation.W) || !Rotation.IsNormalized())
				return Fail(OutError, TEXT("Cartesian transform overflows Unreal representation"));
			return true;
		};
		for (int32 Index = 0; Index < Description.BodyNames.Num(); ++Index)
		{
			FTransform Transform;
			if (!DecodeTransform(false, Transform)) return false;
			Candidate.Cartesian.BodySceneTransforms.Add(MoveTemp(Transform));
		}
		for (int32 Index = 0; Index < Description.ModelCameras.Num(); ++Index)
		{
			FTransform Transform;
			if (!DecodeTransform(true, Transform)) return false;
			Candidate.Cartesian.CameraSceneTransforms.Add(MoveTemp(Transform));
		}
		if (Description.UserCamera.IsSet())
		{
			FTransform Transform;
			if (!DecodeTransform(true, Transform)) return false;
			Candidate.Cartesian.UserCameraSceneTransform = MoveTemp(Transform);
		}
		for (int32 Index = 0; Index < Description.LightNames.Num(); ++Index)
		{
			FTransform Transform;
			if (!DecodeTransform(false, Transform)) return false;
			Candidate.Cartesian.LightSceneTransforms.Add(MoveTemp(Transform));
		}
	}
	OutPose = MoveTemp(Candidate);
	return true;
}
