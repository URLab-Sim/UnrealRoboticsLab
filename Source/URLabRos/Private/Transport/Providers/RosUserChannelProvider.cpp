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

#include "Transport/RosOutputProvider.h"
#include "State/MjStateTypes.h"
#include "Bridge/MsgpackHelpers.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonSerializer.h"

// User-channel -> ROS routing, typed per kind. Every FMjUserChannel on every
// articulation (topic /<art>/user/<name>) and every scene-scoped channel (topic
// /urlab/user/<name>) reaches ROS as a typed message keyed on its kind:
//   Bool                 -> std_msgs/Bool
//   Int / Scalar         -> std_msgs/Float64
//   Vec3                 -> geometry_msgs/Vector3
//   Quat / Transform     -> geometry_msgs/PoseStamped (wxyz reordered to xyzw)
//   Array                -> std_msgs/Float64MultiArray
//   String               -> std_msgs/String
//   Struct               -> std_msgs/String (the packed msgpack map as JSON text)
// The declared channel set is structure, so the publisher set rebuilds only on a
// StructureVersion change, like every other provider.
class FMjRosUserChannelProvider : public IMjRosOutputProvider
{
public:
	virtual FName GetProviderName() const override { return TEXT("user_channels"); }

	virtual void Build(FMjRosPublisherFactory& Factory, const FMjStateSnapshot& Snapshot) override
	{
		Entries.Reset();
		for (int32 i = 0; i < Snapshot.Articulations.Num(); ++i)
		{
			const FMjArticulationState& Art = Snapshot.Articulations[i];
			const FString ArtName = Art.Name.ToString();
			for (const FMjUserChannel& Channel : Art.UserChannels)
			{
				const FString Topic = FString::Printf(TEXT("/%s/user/%s"),
					*ArtName, *Channel.Name.ToString());
				BuildEntry(Factory, i, Channel, Topic, ArtName);
			}
		}
		for (const FMjUserChannel& Channel : Snapshot.UserChannels)
		{
			const FString Topic = FString::Printf(TEXT("/urlab/user/%s"),
				*Channel.Name.ToString());
			BuildEntry(Factory, /*SceneScope*/ -1, Channel, Topic, TEXT("world"));
		}
	}

	virtual void Publish(const FMjStateSnapshot& Snapshot, int64 SimTimeNs) override
	{
		for (FEntry& Entry : Entries)
		{
			const TArray<FMjUserChannel>* Channels = nullptr;
			if (Entry.ArtIndex < 0)
			{
				Channels = &Snapshot.UserChannels;
			}
			else if (Snapshot.Articulations.IsValidIndex(Entry.ArtIndex))
			{
				Channels = &Snapshot.Articulations[Entry.ArtIndex].UserChannels;
			}
			if (!Channels)
			{
				continue;
			}
			const FMjUserChannel* Channel = FindChannel(*Channels, Entry.Name);
			if (!Channel)
			{
				continue;
			}
			PublishEntry(Entry, *Channel, SimTimeNs);
		}
	}

	virtual int32 GetPublisherCountForTest() const override { return Entries.Num(); }

private:
	struct FEntry
	{
		int32 ArtIndex = -1;  // -1 = scene scope
		FName Name;
		EMjUserChannelKind Kind = EMjUserChannelKind::Scalar;
		FMjRosPub Pub;
	};

	static const FMjUserChannel* FindChannel(const TArray<FMjUserChannel>& Channels, FName Name)
	{
		for (const FMjUserChannel& C : Channels)
		{
			if (C.Name == Name)
			{
				return &C;
			}
		}
		return nullptr;
	}

	void BuildEntry(FMjRosPublisherFactory& Factory, int32 ArtIndex,
		const FMjUserChannel& Channel, const FString& Topic, const FString& FrameId)
	{
		FMjRosPub Pub;
		switch (Channel.Kind)
		{
		case EMjUserChannelKind::Bool:
			Pub = Factory.CreateBool(Topic);
			break;
		case EMjUserChannelKind::Int:
		case EMjUserChannelKind::Scalar:
			Pub = Factory.CreateFloat64(Topic);
			break;
		case EMjUserChannelKind::Vec3:
			Pub = Factory.CreateVector3(Topic);
			break;
		case EMjUserChannelKind::Quat:
		case EMjUserChannelKind::Transform:
			Pub = Factory.CreatePoseStamped(Topic, FrameId);
			break;
		case EMjUserChannelKind::Array:
			Pub = Factory.CreateFloat64MultiArray(Topic);
			break;
		case EMjUserChannelKind::String:
		case EMjUserChannelKind::Struct:
			Pub = Factory.CreateString(Topic);
			break;
		}
		if (Pub.IsValid())
		{
			FEntry Entry;
			Entry.ArtIndex = ArtIndex;
			Entry.Name = Channel.Name;
			Entry.Kind = Channel.Kind;
			Entry.Pub = MoveTemp(Pub);
			Entries.Add(MoveTemp(Entry));
		}
	}

	static void PublishEntry(FEntry& Entry, const FMjUserChannel& Channel, int64 SimTimeNs)
	{
		switch (Entry.Kind)
		{
		case EMjUserChannelKind::Bool:
			Entry.Pub.PublishBool(Channel.Values.Num() > 0 && Channel.Values[0] != 0.0);
			break;
		case EMjUserChannelKind::Int:
		case EMjUserChannelKind::Scalar:
			Entry.Pub.PublishFloat64(Channel.Values.Num() > 0 ? Channel.Values[0] : 0.0);
			break;
		case EMjUserChannelKind::Vec3:
		{
			double Xyz[3] = {0.0, 0.0, 0.0};
			for (int32 i = 0; i < 3 && i < Channel.Values.Num(); ++i)
			{
				Xyz[i] = Channel.Values[i];
			}
			Entry.Pub.PublishVector3(Xyz);
			break;
		}
		case EMjUserChannelKind::Quat:
		{
			// Values are wxyz; PoseStamped orientation is xyzw, position zero.
			const double Pos[3] = {0.0, 0.0, 0.0};
			double Quat[4] = {0.0, 0.0, 0.0, 1.0};
			if (Channel.Values.Num() >= 4)
			{
				Quat[0] = Channel.Values[1];
				Quat[1] = Channel.Values[2];
				Quat[2] = Channel.Values[3];
				Quat[3] = Channel.Values[0];
			}
			Entry.Pub.PublishPoseStamped(Pos, Quat, SimTimeNs);
			break;
		}
		case EMjUserChannelKind::Transform:
		{
			// Values: [0..2] pos, [3..6] quat wxyz -> xyzw.
			double Pos[3] = {0.0, 0.0, 0.0};
			double Quat[4] = {0.0, 0.0, 0.0, 1.0};
			for (int32 i = 0; i < 3 && i < Channel.Values.Num(); ++i)
			{
				Pos[i] = Channel.Values[i];
			}
			if (Channel.Values.Num() >= 7)
			{
				Quat[0] = Channel.Values[4];
				Quat[1] = Channel.Values[5];
				Quat[2] = Channel.Values[6];
				Quat[3] = Channel.Values[3];
			}
			Entry.Pub.PublishPoseStamped(Pos, Quat, SimTimeNs);
			break;
		}
		case EMjUserChannelKind::Array:
			Entry.Pub.PublishFloat64MultiArray(Channel.Values.GetData(), Channel.Values.Num());
			break;
		case EMjUserChannelKind::String:
			Entry.Pub.PublishString(Channel.Text);
			break;
		case EMjUserChannelKind::Struct:
			Entry.Pub.PublishString(StructToJson(Channel.Packed));
			break;
		}
	}

	/** Inflate the packed msgpack map and re-serialise it as JSON text so a Struct
	 *  channel is human-readable on its std_msgs/String topic. */
	static FString StructToJson(const TArray<uint8>& Packed)
	{
		TSharedPtr<FJsonObject> Parsed;
		if (Packed.Num() > 0
			&& FURLabMsgpackUtil::UnpackToJsonObject(Packed.GetData(), Packed.Num(), Parsed)
			&& Parsed.IsValid())
		{
			FString Out;
			TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&Out);
			FJsonSerializer::Serialize(Parsed.ToSharedRef(), Writer);
			return Out;
		}
		return TEXT("{}");
	}

	TArray<FEntry> Entries;
};

REGISTER_MJ_ROS_OUTPUT_PROVIDER("user_channels", FMjRosUserChannelProvider);
