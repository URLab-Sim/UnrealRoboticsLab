// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
// trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

#include "UserChannels/MjUserChannelComponent.h"
#include "State/MjCanonicalName.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Utils/URLabAxisConv.h"
#include "Async/Async.h"

namespace
{
/** Map the Blueprint-facing input kind onto the IR kind (1:1). */
EMjUserChannelKind ToChannelKind(EMjUserInputKind Kind)
{
	switch (Kind)
	{
		case EMjUserInputKind::Bool:
			return EMjUserChannelKind::Bool;
		case EMjUserInputKind::Int:
			return EMjUserChannelKind::Int;
		case EMjUserInputKind::Scalar:
			return EMjUserChannelKind::Scalar;
		case EMjUserInputKind::Vec3:
			return EMjUserChannelKind::Vec3;
		case EMjUserInputKind::Quat:
			return EMjUserChannelKind::Quat;
		case EMjUserInputKind::Transform:
			return EMjUserChannelKind::Transform;
		case EMjUserInputKind::Array:
			return EMjUserChannelKind::Array;
		case EMjUserInputKind::String:
			return EMjUserChannelKind::String;
	}
	return EMjUserChannelKind::Scalar;
}

/** String and Struct are the text family; everything else is numeric. An input's
 *  provided kind must share a family with its declared kind to be accepted. */
bool IsTextKind(EMjUserChannelKind Kind)
{
	return Kind == EMjUserChannelKind::String || Kind == EMjUserChannelKind::Struct;
}
} // namespace

UMjUserChannelComponent::UMjUserChannelComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
}

FName UMjUserChannelComponent::MakeChannelName(FName Raw)
{
	return FName(*FMjCanonicalName::Sanitize(Raw.ToString()));
}

void UMjUserChannelComponent::StoreChannel(FMjUserChannel&& Channel)
{
	const FName Key = Channel.Name;
	bool bStructureChanged = false;
	{
		FScopeLock Lock(&MailboxMutex);
		if (const FMjUserChannel* Existing = Mailbox.Find(Key))
			bStructureChanged = (Existing->Kind != Channel.Kind);
		else
			bStructureChanged = true;
		Mailbox.Add(Key, MoveTemp(Channel));
	}

	// First publish of a name, or a kind change, is a structure change: bump the
	// collector's StructureVersion so schema-caching consumers rebuild. Steady
	// state publishes touch nothing but the mailbox.
	if (bStructureChanged)
	{
		if (AAMjManager* Mgr = ResolveManager())
			Mgr->GetStateCollector().MarkProducerCacheDirty();
	}
}

void UMjUserChannelComponent::PublishBool(FName Channel, bool bValue)
{
	FMjUserChannel C;
	C.Name = MakeChannelName(Channel);
	C.Kind = EMjUserChannelKind::Bool;
	C.Values = {bValue ? 1.0 : 0.0};
	StoreChannel(MoveTemp(C));
}

void UMjUserChannelComponent::PublishInt(FName Channel, int64 Value)
{
	FMjUserChannel C;
	C.Name = MakeChannelName(Channel);
	C.Kind = EMjUserChannelKind::Int;
	C.Values = {static_cast<double>(Value)};
	StoreChannel(MoveTemp(C));
}

void UMjUserChannelComponent::PublishFloat(FName Channel, double Value)
{
	FMjUserChannel C;
	C.Name = MakeChannelName(Channel);
	C.Kind = EMjUserChannelKind::Scalar;
	C.Values = {Value};
	StoreChannel(MoveTemp(C));
}

void UMjUserChannelComponent::PublishVector(FName Channel, FVector Value, bool bConvertFromUESpace)
{
	FMjUserChannel C;
	C.Name = MakeChannelName(Channel);
	C.Kind = EMjUserChannelKind::Vec3;
	C.Values.SetNumUninitialized(3);
	if (bConvertFromUESpace)
	{
		double Out[3];
		URLabAxisConv::UePositionToMj(Value, Out);
		C.Values[0] = Out[0];
		C.Values[1] = Out[1];
		C.Values[2] = Out[2];
	}
	else
	{
		C.Values[0] = Value.X;
		C.Values[1] = Value.Y;
		C.Values[2] = Value.Z;
	}
	StoreChannel(MoveTemp(C));
}

void UMjUserChannelComponent::PublishQuat(FName Channel, FQuat Value, bool bConvertFromUESpace)
{
	FMjUserChannel C;
	C.Name = MakeChannelName(Channel);
	C.Kind = EMjUserChannelKind::Quat;
	C.Values.SetNumUninitialized(4);
	if (bConvertFromUESpace)
	{
		double Out[4];
		URLabAxisConv::UeQuatToMj(Value, Out);
		C.Values[0] = Out[0];
		C.Values[1] = Out[1];
		C.Values[2] = Out[2];
		C.Values[3] = Out[3];
	}
	else
	{
		// Already MuJoCo wxyz.
		C.Values[0] = Value.W;
		C.Values[1] = Value.X;
		C.Values[2] = Value.Y;
		C.Values[3] = Value.Z;
	}
	StoreChannel(MoveTemp(C));
}

void UMjUserChannelComponent::PublishTransform(FName Channel, FTransform Value, bool bConvertFromUESpace)
{
	FMjUserChannel C;
	C.Name = MakeChannelName(Channel);
	C.Kind = EMjUserChannelKind::Transform;
	C.Values.SetNumUninitialized(7);
	const FVector Pos = Value.GetLocation();
	const FQuat Rot = Value.GetRotation();
	if (bConvertFromUESpace)
	{
		double P[3];
		double Q[4];
		URLabAxisConv::UePositionToMj(Pos, P);
		URLabAxisConv::UeQuatToMj(Rot, Q);
		C.Values[0] = P[0];
		C.Values[1] = P[1];
		C.Values[2] = P[2];
		C.Values[3] = Q[0];
		C.Values[4] = Q[1];
		C.Values[5] = Q[2];
		C.Values[6] = Q[3];
	}
	else
	{
		C.Values[0] = Pos.X;
		C.Values[1] = Pos.Y;
		C.Values[2] = Pos.Z;
		C.Values[3] = Rot.W;
		C.Values[4] = Rot.X;
		C.Values[5] = Rot.Y;
		C.Values[6] = Rot.Z;
	}
	StoreChannel(MoveTemp(C));
}

void UMjUserChannelComponent::PublishFloatArray(FName Channel, const TArray<double>& Values)
{
	FMjUserChannel C;
	C.Name = MakeChannelName(Channel);
	C.Kind = EMjUserChannelKind::Array;
	C.Values = Values;
	StoreChannel(MoveTemp(C));
}

void UMjUserChannelComponent::PublishString(FName Channel, const FString& Value)
{
	FMjUserChannel C;
	C.Name = MakeChannelName(Channel);
	C.Kind = EMjUserChannelKind::String;
	C.Text = Value;
	StoreChannel(MoveTemp(C));
}

void UMjUserChannelComponent::PublishStructBytes(FName Channel, const TArray<uint8>& PackedMsgpackMap)
{
	FMjUserChannel C;
	C.Name = MakeChannelName(Channel);
	C.Kind = EMjUserChannelKind::Struct;
	C.Packed = PackedMsgpackMap;
	StoreChannel(MoveTemp(C));
}

void UMjUserChannelComponent::DeclareInputChannel(FName Channel, EMjUserInputKind Kind)
{
	const FName Key = MakeChannelName(Channel);
	bool bChanged = false;
	{
		FScopeLock Lock(&InputMutex);
		const EMjUserChannelKind NewKind = ToChannelKind(Kind);
		if (const EMjUserChannelKind* Existing = InputDecls.Find(Key))
			bChanged = (*Existing != NewKind);
		else
			bChanged = true;
		InputDecls.Add(Key, NewKind);
	}
	// A new declared input is a structure change: ROS rebuilds its per-channel
	// subscriptions on the StructureVersion bump.
	if (bChanged)
	{
		if (AAMjManager* Mgr = ResolveManager())
			Mgr->GetStateCollector().MarkProducerCacheDirty();
	}
}

bool UMjUserChannelComponent::GetDeclaredInputKind(FName Channel, EMjUserChannelKind& OutKind) const
{
	const FName Key = MakeChannelName(Channel);
	FScopeLock Lock(&InputMutex);
	if (const EMjUserChannelKind* Found = InputDecls.Find(Key))
	{
		OutKind = *Found;
		return true;
	}
	return false;
}

void UMjUserChannelComponent::GetDeclaredInputChannels(
	TArray<TPair<FName, EMjUserChannelKind>>& Out) const
{
	FScopeLock Lock(&InputMutex);
	Out.Reserve(Out.Num() + InputDecls.Num());
	for (const TPair<FName, EMjUserChannelKind>& Pair : InputDecls)
		Out.Add(Pair);
}

bool UMjUserChannelComponent::ApplyInput(FName Channel, const FMjUserChannel& Value)
{
	const FName Key = MakeChannelName(Channel);
	EMjUserChannelKind Declared;
	{
		FScopeLock Lock(&InputMutex);
		const EMjUserChannelKind* Found = InputDecls.Find(Key);
		if (!Found)
			return false; // undeclared: not on the allowlist
		Declared = *Found;
	}

	// Kind-family check: a text value cannot fill a numeric channel or vice versa.
	if (IsTextKind(Declared) != IsTextKind(Value.Kind))
		return false;

	FMjUserChannel Stored;
	Stored.Name = Key;
	Stored.Kind = Declared;
	if (IsTextKind(Declared))
		Stored.Text = Value.Text;
	else
		Stored.Values = Value.Values;

	{
		FScopeLock Lock(&InputMutex);
		InputMailbox.Add(Key, MoveTemp(Stored));
	}

	// OnUserInput is a Blueprint delegate; broadcast on the game thread so graphs
	// never run on a transport thread.
	TWeakObjectPtr<UMjUserChannelComponent> WeakThis(this);
	AsyncTask(ENamedThreads::GameThread, [WeakThis, Key]() {
		if (UMjUserChannelComponent* Self = WeakThis.Get())
			Self->OnUserInput.Broadcast(Key);
	});
	return true;
}

bool UMjUserChannelComponent::GetInputBool(FName Channel, bool bDefault) const
{
	FScopeLock Lock(&InputMutex);
	if (const FMjUserChannel* C = InputMailbox.Find(MakeChannelName(Channel)))
		return C->Values.Num() > 0 && C->Values[0] != 0.0;
	return bDefault;
}

double UMjUserChannelComponent::GetInputFloat(FName Channel, double Default) const
{
	FScopeLock Lock(&InputMutex);
	if (const FMjUserChannel* C = InputMailbox.Find(MakeChannelName(Channel)))
		return C->Values.Num() > 0 ? C->Values[0] : Default;
	return Default;
}

FVector UMjUserChannelComponent::GetInputVector(FName Channel, bool bConvertToUESpace) const
{
	FScopeLock Lock(&InputMutex);
	const FMjUserChannel* C = InputMailbox.Find(MakeChannelName(Channel));
	if (!C || C->Values.Num() < 3)
		return FVector::ZeroVector;
	const double V[3] = {C->Values[0], C->Values[1], C->Values[2]};
	return bConvertToUESpace ? URLabAxisConv::MjPositionToUe(V)
							 : FVector(V[0], V[1], V[2]);
}

FTransform UMjUserChannelComponent::GetInputTransform(FName Channel, bool bConvertToUESpace) const
{
	FScopeLock Lock(&InputMutex);
	const FMjUserChannel* C = InputMailbox.Find(MakeChannelName(Channel));
	if (!C || C->Values.Num() < 7)
		return FTransform::Identity;
	const double P[3] = {C->Values[0], C->Values[1], C->Values[2]};
	const double Q[4] = {C->Values[3], C->Values[4], C->Values[5], C->Values[6]}; // wxyz
	if (bConvertToUESpace)
		return FTransform(URLabAxisConv::MjQuatToUe(Q), URLabAxisConv::MjPositionToUe(P));
	return FTransform(FQuat(Q[1], Q[2], Q[3], Q[0]), FVector(P[0], P[1], P[2]));
}

TArray<double> UMjUserChannelComponent::GetInputFloatArray(FName Channel) const
{
	FScopeLock Lock(&InputMutex);
	if (const FMjUserChannel* C = InputMailbox.Find(MakeChannelName(Channel)))
		return C->Values;
	return TArray<double>();
}

FString UMjUserChannelComponent::GetInputString(FName Channel) const
{
	FScopeLock Lock(&InputMutex);
	if (const FMjUserChannel* C = InputMailbox.Find(MakeChannelName(Channel)))
		return C->Text;
	return FString();
}

void UMjUserChannelComponent::DescribeState(FMjArticulationState& Out) const
{
	CopyMailboxInto(Out.UserChannels);
}

void UMjUserChannelComponent::DescribeSceneState(FMjStateSnapshot& Out) const
{
	CopyMailboxInto(Out.UserChannels);
}

void UMjUserChannelComponent::CopyMailboxInto(TArray<FMjUserChannel>& OutChannels) const
{
	FScopeLock Lock(&MailboxMutex);
	OutChannels.Reserve(OutChannels.Num() + Mailbox.Num());
	for (const TPair<FName, FMjUserChannel>& Pair : Mailbox)
		OutChannels.Add(Pair.Value);
}

AAMjManager* UMjUserChannelComponent::ResolveManager()
{
	if (AAMjManager* M = CachedManager.Get())
		return M;
	AAMjManager* M = AAMjManager::GetManager();
	CachedManager = M;
	return M;
}

void UMjUserChannelComponent::BeginPlay()
{
	Super::BeginPlay();
	if (AAMjManager* Mgr = ResolveManager())
		Mgr->RegisterStateProducer(this);
}

void UMjUserChannelComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (AAMjManager* Mgr = CachedManager.Get())
		Mgr->UnregisterStateProducer(this);
	Super::EndPlay(EndPlayReason);
}
