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

void UMjUserChannelComponent::DescribeState(FMjArticulationState& Out) const
{
	FScopeLock Lock(&MailboxMutex);
	Out.UserChannels.Reserve(Out.UserChannels.Num() + Mailbox.Num());
	for (const TPair<FName, FMjUserChannel>& Pair : Mailbox)
		Out.UserChannels.Add(Pair.Value);
}

void UMjUserChannelComponent::DescribeSceneState(FMjStateSnapshot& Out) const
{
	FScopeLock Lock(&MailboxMutex);
	Out.UserChannels.Reserve(Out.UserChannels.Num() + Mailbox.Num());
	for (const TPair<FName, FMjUserChannel>& Pair : Mailbox)
		Out.UserChannels.Add(Pair.Value);
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
