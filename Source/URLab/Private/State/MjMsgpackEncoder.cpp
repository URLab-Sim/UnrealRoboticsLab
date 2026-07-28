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

#include "State/MjMsgpackEncoder.h"
#include "State/MjStateTypes.h"
#include "Bridge/MsgpackHelpers.h"

namespace
{
TSharedPtr<FJsonValue> NumArray(const TArray<double>& Values)
{
	TArray<TSharedPtr<FJsonValue>> Out;
	Out.Reserve(Values.Num());
	for (double V : Values)
		Out.Add(MakeShared<FJsonValueNumber>(V));
	return MakeShared<FJsonValueArray>(Out);
}

TSharedPtr<FJsonValue> NumArrayN(const double* Values, int32 Count)
{
	TArray<TSharedPtr<FJsonValue>> Out;
	Out.Reserve(Count);
	for (int32 i = 0; i < Count; ++i)
		Out.Add(MakeShared<FJsonValueNumber>(Values[i]));
	return MakeShared<FJsonValueArray>(Out);
}

/** Encode one user channel into its self-describing msgpack value. The shape is
 *  keyed on Kind so a Python client reads typed values with no schema. */
TSharedPtr<FJsonValue> EncodeUserValue(const FMjUserChannel& C)
{
	switch (C.Kind)
	{
		case EMjUserChannelKind::Bool:
			return MakeShared<FJsonValueBoolean>(C.Values.Num() > 0 && C.Values[0] != 0.0);
		case EMjUserChannelKind::Int:
		case EMjUserChannelKind::Scalar:
			return MakeShared<FJsonValueNumber>(C.Values.Num() > 0 ? C.Values[0] : 0.0);
		case EMjUserChannelKind::Vec3:
		case EMjUserChannelKind::Quat:
		case EMjUserChannelKind::Array:
			return NumArray(C.Values);
		case EMjUserChannelKind::Transform:
		{
			TSharedPtr<FJsonObject> Obj = MakeShared<FJsonObject>();
			const int32 N = C.Values.Num();
			Obj->SetField(TEXT("pos"), NumArrayN(C.Values.GetData(), FMath::Min(3, N)));
			Obj->SetField(TEXT("quat"), N > 3 ? NumArrayN(C.Values.GetData() + 3, FMath::Min(4, N - 3))
											  : NumArrayN(nullptr, 0));
			return MakeShared<FJsonValueObject>(Obj);
		}
		case EMjUserChannelKind::String:
			return MakeShared<FJsonValueString>(C.Text);
		case EMjUserChannelKind::Struct:
		{
			// Packed carries a pre-built msgpack map; re-inflate it so it splices
			// into the snapshot as a nested object rather than an opaque blob.
			TSharedPtr<FJsonObject> Parsed;
			if (C.Packed.Num() > 0
				&& FURLabMsgpackUtil::UnpackToJsonObject(C.Packed.GetData(), C.Packed.Num(), Parsed)
				&& Parsed.IsValid())
			{
				return MakeShared<FJsonValueObject>(Parsed);
			}
			return MakeShared<FJsonValueObject>(MakeShared<FJsonObject>());
		}
	}
	return MakeShared<FJsonValueNull>();
}

/** Emit a `user` map keyed by channel name, if any channels exist. */
void EncodeUserChannels(const TArray<FMjUserChannel>& Channels, const TSharedPtr<FJsonObject>& Out)
{
	if (Channels.Num() == 0)
		return;
	TSharedPtr<FJsonObject> User = MakeShared<FJsonObject>();
	for (const FMjUserChannel& C : Channels)
		User->SetField(C.Name.ToString(), EncodeUserValue(C));
	Out->SetObjectField(TEXT("user"), User);
}

/** Encode one articulation into the per-art block for the requested level. */
TSharedPtr<FJsonObject> EncodeArticulation(const FMjArticulationState& Art, EObservationLevel Level)
{
	const bool bStandard = (Level == EObservationLevel::Standard) || (Level == EObservationLevel::Full);
	const bool bFull = (Level == EObservationLevel::Full);

	TSharedPtr<FJsonObject> Obj = MakeShared<FJsonObject>();

	// qpos / qvel -- present at every level, concatenated in joint order.
	{
		TArray<TSharedPtr<FJsonValue>> QPos;
		TArray<TSharedPtr<FJsonValue>> QVel;
		for (const FMjJointState& J : Art.Joints)
		{
			for (double V : J.QPos)
				QPos.Add(MakeShared<FJsonValueNumber>(V));
			for (double V : J.QVel)
				QVel.Add(MakeShared<FJsonValueNumber>(V));
		}
		Obj->SetArrayField(TEXT("qpos"), QPos);
		Obj->SetArrayField(TEXT("qvel"), QVel);
	}

	if (bStandard)
	{
		TArray<TSharedPtr<FJsonValue>> Ctrl;
		TArray<TSharedPtr<FJsonValue>> Act;
		for (const FMjActuatorState& A : Art.Actuators)
		{
			Ctrl.Add(MakeShared<FJsonValueNumber>(A.Ctrl));
			Act.Add(MakeShared<FJsonValueNumber>(A.Act));
		}
		Obj->SetArrayField(TEXT("ctrl"), Ctrl);
		Obj->SetArrayField(TEXT("act"), Act);

		TSharedPtr<FJsonObject> Sensors = MakeShared<FJsonObject>();
		for (const FMjSensorState& Sen : Art.Sensors)
			Sensors->SetField(Sen.Name.ToString(), NumArray(Sen.Values));
		Obj->SetObjectField(TEXT("sensors"), Sensors);

		// User channels emit at Standard and Full, beside sensors.
		EncodeUserChannels(Art.UserChannels, Obj);
	}

	if (bFull)
	{
		TSharedPtr<FJsonObject> Bodies = MakeShared<FJsonObject>();
		for (const FMjBodyState& B : Art.Bodies)
		{
			TSharedPtr<FJsonObject> Bo = MakeShared<FJsonObject>();
			Bo->SetField(TEXT("xpos"), NumArrayN(B.Xpos, 3));
			Bo->SetField(TEXT("xquat"), NumArrayN(B.Xquat, 4));
			Bodies->SetObjectField(B.Name.ToString(), Bo);
		}
		Obj->SetObjectField(TEXT("bodies"), Bodies);

		TArray<TSharedPtr<FJsonValue>> Force;
		for (const FMjActuatorState& A : Art.Actuators)
			Force.Add(MakeShared<FJsonValueNumber>(A.Force));
		Obj->SetArrayField(TEXT("actuator_force"), Force);
	}

	// Twist -- emitted whenever a twist controller is attached, regardless of
	// level. geometry_msgs/Twist layout.
	if (Art.Twist.IsSet())
	{
		const FMjTwistState& T = Art.Twist.GetValue();
		TSharedPtr<FJsonObject> TwistObj = MakeShared<FJsonObject>();
		TwistObj->SetField(TEXT("linear"), NumArrayN(T.Linear, 3));
		TwistObj->SetField(TEXT("angular"), NumArrayN(T.Angular, 3));
		Obj->SetObjectField(TEXT("twist"), TwistObj);
		Obj->SetNumberField(TEXT("actions"), static_cast<double>(T.Actions));
	}

	return Obj;
}

/** sim_time / wall_time blocks, matching FURLabRpcDispatcher::AppendClockFields. */
void EncodeClock(const FMjClock& Clock, const TSharedPtr<FJsonObject>& Out)
{
	TSharedPtr<FJsonObject> Sim = MakeShared<FJsonObject>();
	Sim->SetNumberField(TEXT("sec"), Clock.SimSec);
	Sim->SetNumberField(TEXT("nsec"), Clock.SimNsec);
	Out->SetObjectField(TEXT("sim_time"), Sim);

	TSharedPtr<FJsonObject> Wall = MakeShared<FJsonObject>();
	Wall->SetNumberField(TEXT("sec"), static_cast<double>(Clock.WallSec));
	Wall->SetNumberField(TEXT("nsec"), static_cast<double>(Clock.WallNsec));
	Out->SetObjectField(TEXT("wall_time"), Wall);
}
} // namespace

TSharedPtr<FJsonObject> FMjMsgpackEncoder::EncodeArts(const FMjStateSnapshot& S,
	EObservationLevel Level)
{
	TSharedPtr<FJsonObject> Arts = MakeShared<FJsonObject>();
	for (const FMjArticulationState& Art : S.Articulations)
		Arts->SetObjectField(Art.Name.ToString(), EncodeArticulation(Art, Level));
	return Arts;
}

TSharedPtr<FJsonObject> FMjMsgpackEncoder::EncodeScene(const FMjStateSnapshot& S)
{
	TSharedPtr<FJsonObject> Scene = MakeShared<FJsonObject>();
	for (const FMjEntityState& E : S.Entities)
	{
		TSharedPtr<FJsonObject> Obj = MakeShared<FJsonObject>();
		Obj->SetField(TEXT("xpos"), NumArrayN(E.Xpos, 3));
		Obj->SetField(TEXT("xquat"), NumArrayN(E.Xquat, 4));
		if (E.bFreeBase && E.QPos.Num() == 7 && E.QVel.Num() == 6)
		{
			Obj->SetField(TEXT("qpos"), NumArray(E.QPos));
			Obj->SetField(TEXT("qvel"), NumArray(E.QVel));
		}
		Scene->SetObjectField(E.Name.ToString(), Obj);
	}
	return Scene;
}

TSharedPtr<FJsonObject> FMjMsgpackEncoder::EncodeSnapshot(const FMjStateSnapshot& S,
	EObservationLevel Level)
{
	TSharedPtr<FJsonObject> Snap = MakeShared<FJsonObject>();
	Snap->SetStringField(TEXT("op"), TEXT("state_full"));
	Snap->SetNumberField(TEXT("time"), S.Time);
	Snap->SetNumberField(TEXT("step"), static_cast<double>(S.Step));
	EncodeClock(S.Clock, Snap);
	Snap->SetObjectField(TEXT("arts"), EncodeArts(S, Level));
	Snap->SetObjectField(TEXT("scene"), EncodeScene(S));
	// Scene-scoped user channels (producers not owned by any articulation).
	EncodeUserChannels(S.UserChannels, Snap);
	return Snap;
}

TArray<uint8> FMjMsgpackEncoder::EncodeSnapshotBytes(const FMjStateSnapshot& S,
	EObservationLevel Level)
{
	TArray<uint8> Buf;
	FURLabMsgpackUtil::PackJsonObject(EncodeSnapshot(S, Level), Buf);
	return Buf;
}
