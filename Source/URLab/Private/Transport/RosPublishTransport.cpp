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

#include "Transport/RosPublishTransport.h"
#include "State/MjStateTypes.h"

#if defined(URLAB_WITH_ROS2) && URLAB_WITH_ROS2
#include "Transport/RosContext.h"
#include "Ros/UrlabRclCore.h"
#include "Utils/URLabLogging.h"
#endif

// FillJointState is a pure IR -> arrays transform with no rcl dependency, so it
// compiles in every configuration (and is exercised by the fill-correctness
// test whether or not ROS is linked).
void UURLabRosPublishTransport::FillJointState(const FMjArticulationState& Art,
	TArray<FString>& OutNames, TArray<double>& OutPositions,
	TArray<double>& OutVelocities)
{
	OutNames.Reset();
	OutPositions.Reset();
	OutVelocities.Reset();
	OutNames.Reserve(Art.Joints.Num());
	for (const FMjJointState& Joint : Art.Joints)
	{
		OutNames.Add(Joint.Name.ToString());
		OutPositions.Append(Joint.QPos);
		OutVelocities.Append(Joint.QVel);
	}
}

#if defined(URLAB_WITH_ROS2) && URLAB_WITH_ROS2

bool UURLabRosPublishTransport::TransportInit()
{
	return FURLabRosContext::Get().Initialize();
}

void UURLabRosPublishTransport::TransportShutdown()
{
	ReleasePublishers();
}

void UURLabRosPublishTransport::ReleasePublishers()
{
	for (int32 i = Publishers.Num() - 1; i >= 0; --i)
	{
		UrlabRcl_DestroyJointStatePub(Publishers[i].JointStatePub);
	}
	Publishers.Reset();
	bPublishersBuilt = false;
}

void UURLabRosPublishTransport::RebuildPublishers(const FMjStateSnapshot& Snapshot)
{
	ReleasePublishers();

	UrlabRclContext* Ctx = FURLabRosContext::Get().GetHandle();
	if (Ctx == nullptr)
	{
		return;
	}

	Publishers.Reserve(Snapshot.Articulations.Num());
	for (const FMjArticulationState& Art : Snapshot.Articulations)
	{
		TArray<FString> Names;
		TArray<double> Positions;
		TArray<double> Velocities;
		FillJointState(Art, Names, Positions, Velocities);

		// The core copies the name strings at create time; keep one UTF-8 buffer
		// per name alive across the call and hand it a stable pointer array. The
		// outer array is reserved so element pointers do not move.
		TArray<TArray<ANSICHAR>> NameBytes;
		NameBytes.Reserve(Names.Num());
		TArray<const char*> NamePtrs;
		NamePtrs.Reserve(Names.Num());
		for (const FString& Name : Names)
		{
			FTCHARToUTF8 Conv(*Name);
			TArray<ANSICHAR>& Bytes = NameBytes.AddDefaulted_GetRef();
			Bytes.Append(reinterpret_cast<const ANSICHAR*>(Conv.Get()), Conv.Length());
			Bytes.Add('\0');
			NamePtrs.Add(Bytes.GetData());
		}

		const FString Topic = FString::Printf(TEXT("/%s/joint_states"), *Art.Name.ToString());
		FArtPublishers Entry;
		Entry.ArtSegment = Art.Name;
		Entry.JointStatePub = UrlabRcl_CreateJointStatePub(Ctx,
			TCHAR_TO_UTF8(*Topic), NamePtrs.GetData(), NamePtrs.Num());
		if (Entry.JointStatePub == nullptr)
		{
			UE_LOG(LogURLab, Warning,
				TEXT("ROS: JointState publisher create failed for %s (%hs)"),
				*Topic, UrlabRcl_LastError());
			continue;
		}
		Publishers.Add(Entry);
	}

	CachedStructureVersion = Snapshot.StructureVersion;
	bPublishersBuilt = true;
}

void UURLabRosPublishTransport::PublishState(const FMjStateSnapshot& Snapshot)
{
	if (!FURLabRosContext::Get().IsAvailable())
	{
		return;
	}
	if (!bPublishersBuilt || Snapshot.StructureVersion != CachedStructureVersion)
	{
		RebuildPublishers(Snapshot);
	}

	const int64 SimTimeNs = static_cast<int64>(Snapshot.Clock.SimSec) * 1000000000LL
		+ static_cast<int64>(Snapshot.Clock.SimNsec);

	// The publisher set and the snapshot's articulations share a StructureVersion
	// and RebuildPublishers preserves order, so index i lines up in both.
	const int32 N = FMath::Min(Publishers.Num(), Snapshot.Articulations.Num());
	for (int32 i = 0; i < N; ++i)
	{
		const FMjArticulationState& Art = Snapshot.Articulations[i];
		TArray<FString> Names;
		TArray<double> Positions;
		TArray<double> Velocities;
		FillJointState(Art, Names, Positions, Velocities);

		UrlabRcl_PublishJointState(Publishers[i].JointStatePub,
			Positions.GetData(), Velocities.GetData(), nullptr, Names.Num(), SimTimeNs);
	}
}

#else  // URLAB_WITH_ROS2

// Absent-ROS stubs so the class links; every real caller is fenced off too.
bool UURLabRosPublishTransport::TransportInit() { return false; }
void UURLabRosPublishTransport::TransportShutdown() {}
void UURLabRosPublishTransport::PublishState(const FMjStateSnapshot& /*Snapshot*/) {}
void UURLabRosPublishTransport::RebuildPublishers(const FMjStateSnapshot& /*Snapshot*/) {}
void UURLabRosPublishTransport::ReleasePublishers() {}

#endif  // URLAB_WITH_ROS2
