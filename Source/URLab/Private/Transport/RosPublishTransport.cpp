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
#include "State/MjCanonicalName.h"

#if defined(URLAB_WITH_ROS2) && URLAB_WITH_ROS2
#include "Transport/RosContext.h"
#include "Ros/UrlabRclCore.h"
#include "Utils/URLabLogging.h"
#endif

// The Fill* functions are pure IR -> arrays transforms with no rcl dependency, so
// they compile in every configuration (and are exercised by the fill-correctness
// tests whether or not ROS is linked).

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

bool UURLabRosPublishTransport::FillImu(const FMjArticulationState& Art,
	double OutAngularVel[3], bool& bOutHasAngularVel,
	double OutLinearAccel[3], bool& bOutHasLinearAccel)
{
	bOutHasAngularVel = false;
	bOutHasLinearAccel = false;
	for (int32 i = 0; i < 3; ++i)
	{
		OutAngularVel[i] = 0.0;
		OutLinearAccel[i] = 0.0;
	}

	// Pair the first gyro with the first accel found on the art. A gyro without an
	// accel still yields an Imu with angular velocity only, and vice versa.
	for (const FMjSensorState& Sensor : Art.Sensors)
	{
		if (!bOutHasAngularVel && Sensor.Semantic == EMjSensorSemantic::Gyro
			&& Sensor.Values.Num() >= 3)
		{
			OutAngularVel[0] = Sensor.Values[0];
			OutAngularVel[1] = Sensor.Values[1];
			OutAngularVel[2] = Sensor.Values[2];
			bOutHasAngularVel = true;
		}
		else if (!bOutHasLinearAccel && Sensor.Semantic == EMjSensorSemantic::Accel
			&& Sensor.Values.Num() >= 3)
		{
			OutLinearAccel[0] = Sensor.Values[0];
			OutLinearAccel[1] = Sensor.Values[1];
			OutLinearAccel[2] = Sensor.Values[2];
			bOutHasLinearAccel = true;
		}
	}

	return bOutHasAngularVel || bOutHasLinearAccel;
}

bool UURLabRosPublishTransport::FillTwistStamped(const FMjArticulationState& Art,
	double OutLinear[3], double OutAngular[3])
{
	if (!Art.Twist.IsSet())
	{
		return false;
	}
	const FMjTwistState& Twist = Art.Twist.GetValue();
	for (int32 i = 0; i < 3; ++i)
	{
		OutLinear[i] = Twist.Linear[i];
		OutAngular[i] = Twist.Angular[i];
	}
	return true;
}

void UURLabRosPublishTransport::FillTf(const FMjStateSnapshot& Snapshot,
	TArray<FString>& OutParents, TArray<FString>& OutChildren,
	TArray<double>& OutTranslations, TArray<double>& OutRotationsXyzw)
{
	OutParents.Reset();
	OutChildren.Reset();
	OutTranslations.Reset();
	OutRotationsXyzw.Reset();

	for (const FMjArticulationState& Art : Snapshot.Articulations)
	{
		for (const FMjBodyState& Body : Art.Bodies)
		{
			OutParents.Add(TEXT("world"));
			OutChildren.Add(FMjCanonicalName::Full(Art.Name, Body.Name));
			OutTranslations.Add(Body.Xpos[0]);
			OutTranslations.Add(Body.Xpos[1]);
			OutTranslations.Add(Body.Xpos[2]);
			// MuJoCo stores quaternions wxyz; the core (and ROS) expect xyzw.
			OutRotationsXyzw.Add(Body.Xquat[1]);
			OutRotationsXyzw.Add(Body.Xquat[2]);
			OutRotationsXyzw.Add(Body.Xquat[3]);
			OutRotationsXyzw.Add(Body.Xquat[0]);
		}
	}
}

int64 UURLabRosPublishTransport::FillClock(const FMjClock& Clock)
{
	return static_cast<int64>(Clock.SimSec) * 1000000000LL
		+ static_cast<int64>(Clock.SimNsec);
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
		UrlabRcl_DestroyTwistStampedPub(Publishers[i].TwistPub);
		UrlabRcl_DestroyImuPub(Publishers[i].ImuPub);
		UrlabRcl_DestroyJointStatePub(Publishers[i].JointStatePub);
	}
	Publishers.Reset();
	UrlabRcl_DestroyClockPub(ClockPub);
	ClockPub = nullptr;
	UrlabRcl_DestroyTfPub(TfPub);
	TfPub = nullptr;
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
		const FString ArtName = Art.Name.ToString();

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

		const FString JointTopic = FString::Printf(TEXT("/%s/joint_states"), *ArtName);
		FArtPublishers Entry;
		Entry.ArtSegment = Art.Name;
		Entry.JointStatePub = UrlabRcl_CreateJointStatePub(Ctx,
			TCHAR_TO_UTF8(*JointTopic), NamePtrs.GetData(), NamePtrs.Num());
		if (Entry.JointStatePub == nullptr)
		{
			UE_LOG(LogURLab, Warning,
				TEXT("ROS: JointState publisher create failed for %s (%hs)"),
				*JointTopic, UrlabRcl_LastError());
			continue;
		}

		// One Imu per art, created only when the art carries a gyro and/or accel.
		double Ang[3];
		double Acc[3];
		bool bHasAng = false;
		bool bHasAcc = false;
		if (FillImu(Art, Ang, bHasAng, Acc, bHasAcc))
		{
			const FString ImuTopic = FString::Printf(TEXT("/%s/imu"), *ArtName);
			Entry.ImuPub = UrlabRcl_CreateImuPub(Ctx, TCHAR_TO_UTF8(*ImuTopic),
				TCHAR_TO_UTF8(*ArtName));
			if (Entry.ImuPub == nullptr)
			{
				UE_LOG(LogURLab, Warning,
					TEXT("ROS: Imu publisher create failed for %s (%hs)"),
					*ImuTopic, UrlabRcl_LastError());
			}
		}

		// One TwistStamped per art, created only when the art has a twist command.
		if (Art.Twist.IsSet())
		{
			const FString TwistTopic = FString::Printf(TEXT("/%s/cmd_twist"), *ArtName);
			Entry.TwistPub = UrlabRcl_CreateTwistStampedPub(Ctx,
				TCHAR_TO_UTF8(*TwistTopic), TCHAR_TO_UTF8(*ArtName));
			if (Entry.TwistPub == nullptr)
			{
				UE_LOG(LogURLab, Warning,
					TEXT("ROS: TwistStamped publisher create failed for %s (%hs)"),
					*TwistTopic, UrlabRcl_LastError());
			}
		}

		Publishers.Add(Entry);
	}

	// Process-wide publishers: the whole-scene tf tree and the sim clock.
	TfPub = UrlabRcl_CreateTfPub(Ctx, /*bStatic=*/0);
	if (TfPub == nullptr)
	{
		UE_LOG(LogURLab, Warning, TEXT("ROS: /tf publisher create failed (%hs)"),
			UrlabRcl_LastError());
	}
	ClockPub = UrlabRcl_CreateClockPub(Ctx);
	if (ClockPub == nullptr)
	{
		UE_LOG(LogURLab, Warning, TEXT("ROS: /clock publisher create failed (%hs)"),
			UrlabRcl_LastError());
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
	++PublishStateCount;

	const int64 SimTimeNs = FillClock(Snapshot.Clock);

	// The publisher set and the snapshot's articulations share a StructureVersion
	// and RebuildPublishers preserves order, so index i lines up in both.
	const int32 N = FMath::Min(Publishers.Num(), Snapshot.Articulations.Num());
	for (int32 i = 0; i < N; ++i)
	{
		const FMjArticulationState& Art = Snapshot.Articulations[i];
		const FArtPublishers& Pubs = Publishers[i];

		TArray<FString> Names;
		TArray<double> Positions;
		TArray<double> Velocities;
		FillJointState(Art, Names, Positions, Velocities);
		UrlabRcl_PublishJointState(Pubs.JointStatePub,
			Positions.GetData(), Velocities.GetData(), nullptr, Names.Num(), SimTimeNs);

		if (Pubs.ImuPub)
		{
			double Ang[3];
			double Acc[3];
			bool bHasAng = false;
			bool bHasAcc = false;
			FillImu(Art, Ang, bHasAng, Acc, bHasAcc);
			UrlabRcl_PublishImu(Pubs.ImuPub, bHasAng ? Ang : nullptr,
				bHasAcc ? Acc : nullptr, nullptr, SimTimeNs);
		}

		if (Pubs.TwistPub)
		{
			double Lin[3];
			double Ang[3];
			if (FillTwistStamped(Art, Lin, Ang))
			{
				UrlabRcl_PublishTwistStamped(Pubs.TwistPub, Lin, Ang, SimTimeNs);
			}
		}
	}

	if (TfPub)
	{
		TArray<FString> Parents;
		TArray<FString> Children;
		TArray<double> Translations;
		TArray<double> Rotations;
		FillTf(Snapshot, Parents, Children, Translations, Rotations);
		if (Parents.Num() > 0)
		{
			// The core copies the frame strings; hold stable UTF-8 buffers and
			// pointer arrays alive across the call, as with the joint names above.
			TArray<TArray<ANSICHAR>> ParentBytes;
			TArray<TArray<ANSICHAR>> ChildBytes;
			ParentBytes.Reserve(Parents.Num());
			ChildBytes.Reserve(Children.Num());
			TArray<const char*> ParentPtrs;
			TArray<const char*> ChildPtrs;
			ParentPtrs.Reserve(Parents.Num());
			ChildPtrs.Reserve(Children.Num());
			auto AppendUtf8 = [](TArray<TArray<ANSICHAR>>& Store, TArray<const char*>& Ptrs,
				const FString& Value) {
				FTCHARToUTF8 Conv(*Value);
				TArray<ANSICHAR>& Bytes = Store.AddDefaulted_GetRef();
				Bytes.Append(reinterpret_cast<const ANSICHAR*>(Conv.Get()), Conv.Length());
				Bytes.Add('\0');
				Ptrs.Add(Bytes.GetData());
			};
			for (const FString& P : Parents)
				AppendUtf8(ParentBytes, ParentPtrs, P);
			for (const FString& C : Children)
				AppendUtf8(ChildBytes, ChildPtrs, C);

			UrlabRcl_PublishTf(TfPub, ParentPtrs.GetData(), ChildPtrs.GetData(),
				Translations.GetData(), Rotations.GetData(), ParentPtrs.Num(), SimTimeNs);
		}
	}

	if (ClockPub)
	{
		UrlabRcl_PublishClock(ClockPub, SimTimeNs);
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
