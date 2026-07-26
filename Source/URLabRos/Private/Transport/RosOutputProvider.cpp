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
#include "URLabRosLog.h"

#if defined(URLAB_WITH_ROS2) && URLAB_WITH_ROS2
#include "Ros/UrlabRclCore.h"
#endif

// --- Registry (ROS-agnostic; populated in every configuration) --------------

FMjRosOutputRegistry& FMjRosOutputRegistry::Get()
{
	static FMjRosOutputRegistry Instance;
	return Instance;
}

void FMjRosOutputRegistry::Register(FName Name, FMjRosOutputProviderFactoryFn Factory)
{
	for (TPair<FName, FMjRosOutputProviderFactoryFn>& Entry : Entries)
	{
		if (Entry.Key == Name)
		{
			UE_LOG(LogURLabRos, Warning,
				TEXT("[URLabRos] output provider '%s' re-registered; the later one wins."),
				*Name.ToString());
			Entry.Value = MoveTemp(Factory);
			return;
		}
	}
	Entries.Emplace(Name, MoveTemp(Factory));
}

void FMjRosOutputRegistry::InstantiateAll(TArray<TUniquePtr<IMjRosOutputProvider>>& Out) const
{
	Out.Reset();
	Out.Reserve(Entries.Num());
	for (const TPair<FName, FMjRosOutputProviderFactoryFn>& Entry : Entries)
	{
		if (Entry.Value)
		{
			if (TUniquePtr<IMjRosOutputProvider> Provider = Entry.Value())
			{
				Out.Add(MoveTemp(Provider));
			}
		}
	}
}

TArray<FName> FMjRosOutputRegistry::GetRegisteredNames() const
{
	TArray<FName> Names;
	Names.Reserve(Entries.Num());
	for (const TPair<FName, FMjRosOutputProviderFactoryFn>& Entry : Entries)
	{
		Names.Add(Entry.Key);
	}
	return Names;
}

FMjRosOutputProviderRegistrar::FMjRosOutputProviderRegistrar(FName Name,
	FMjRosOutputProviderFactoryFn Factory)
{
	FMjRosOutputRegistry::Get().Register(Name, MoveTemp(Factory));
}

#if defined(URLAB_WITH_ROS2) && URLAB_WITH_ROS2

// --- FMjRosPub (owning handle over one rcl publisher) -----------------------

void FMjRosPub::Reset()
{
	if (Handle == nullptr)
	{
		return;
	}
	switch (Kind)
	{
	case EKind::JointState:
		UrlabRcl_DestroyJointStatePub(static_cast<UrlabRclJointStatePub*>(Handle));
		break;
	case EKind::Imu:
		UrlabRcl_DestroyImuPub(static_cast<UrlabRclImuPub*>(Handle));
		break;
	case EKind::Tf:
		UrlabRcl_DestroyTfPub(static_cast<UrlabRclTfPub*>(Handle));
		break;
	case EKind::TwistStamped:
		UrlabRcl_DestroyTwistStampedPub(static_cast<UrlabRclTwistStampedPub*>(Handle));
		break;
	case EKind::Clock:
		UrlabRcl_DestroyClockPub(static_cast<UrlabRclClockPub*>(Handle));
		break;
	case EKind::String:
		UrlabRcl_DestroyStringPub(static_cast<UrlabRclStringPub*>(Handle));
		break;
	case EKind::Wrench:
		UrlabRcl_DestroyWrenchStampedPub(static_cast<UrlabRclWrenchStampedPub*>(Handle));
		break;
	case EKind::Range:
		UrlabRcl_DestroyRangePub(static_cast<UrlabRclRangePub*>(Handle));
		break;
	case EKind::MagneticField:
		UrlabRcl_DestroyMagneticFieldPub(static_cast<UrlabRclMagneticFieldPub*>(Handle));
		break;
	case EKind::MultiArray:
		UrlabRcl_DestroyFloat64MultiArrayPub(static_cast<UrlabRclFloat64MultiArrayPub*>(Handle));
		break;
	case EKind::None:
		break;
	}
	Handle = nullptr;
	Kind = EKind::None;
}

void FMjRosPub::PublishJointState(const double* Positions, const double* Velocities,
	const double* Efforts, int32 Count, int64 SimTimeNs)
{
	if (Kind == EKind::JointState && Handle)
	{
		UrlabRcl_PublishJointState(static_cast<UrlabRclJointStatePub*>(Handle),
			Positions, Velocities, Efforts, Count, SimTimeNs);
	}
}

void FMjRosPub::PublishImu(const double* AngularVel3, const double* LinearAccel3,
	const double* OrientationXyzw4, int64 SimTimeNs)
{
	if (Kind == EKind::Imu && Handle)
	{
		UrlabRcl_PublishImu(static_cast<UrlabRclImuPub*>(Handle),
			AngularVel3, LinearAccel3, OrientationXyzw4, SimTimeNs);
	}
}

void FMjRosPub::PublishTf(const TArray<FString>& Parents, const TArray<FString>& Children,
	const TArray<double>& TranslationsXyz, const TArray<double>& RotationsXyzw, int64 SimTimeNs)
{
	if (Kind != EKind::Tf || !Handle || Parents.Num() == 0)
	{
		return;
	}
	// The core copies the frame strings at publish time; keep stable UTF-8 buffers
	// and pointer arrays alive across the call.
	TArray<TArray<ANSICHAR>> ParentBytes;
	TArray<TArray<ANSICHAR>> ChildBytes;
	ParentBytes.Reserve(Parents.Num());
	ChildBytes.Reserve(Children.Num());
	TArray<const char*> ParentPtrs;
	TArray<const char*> ChildPtrs;
	ParentPtrs.Reserve(Parents.Num());
	ChildPtrs.Reserve(Children.Num());
	auto AppendUtf8 = [](TArray<TArray<ANSICHAR>>& Store, TArray<const char*>& Ptrs,
		const FString& Value)
	{
		FTCHARToUTF8 Conv(*Value);
		TArray<ANSICHAR>& Bytes = Store.AddDefaulted_GetRef();
		Bytes.Append(reinterpret_cast<const ANSICHAR*>(Conv.Get()), Conv.Length());
		Bytes.Add('\0');
		Ptrs.Add(Bytes.GetData());
	};
	for (const FString& P : Parents)
	{
		AppendUtf8(ParentBytes, ParentPtrs, P);
	}
	for (const FString& C : Children)
	{
		AppendUtf8(ChildBytes, ChildPtrs, C);
	}

	UrlabRcl_PublishTf(static_cast<UrlabRclTfPub*>(Handle), ParentPtrs.GetData(),
		ChildPtrs.GetData(), TranslationsXyz.GetData(), RotationsXyzw.GetData(),
		ParentPtrs.Num(), SimTimeNs);
}

void FMjRosPub::PublishTwistStamped(const double Linear3[3], const double Angular3[3],
	int64 SimTimeNs)
{
	if (Kind == EKind::TwistStamped && Handle)
	{
		UrlabRcl_PublishTwistStamped(static_cast<UrlabRclTwistStampedPub*>(Handle),
			Linear3, Angular3, SimTimeNs);
	}
}

void FMjRosPub::PublishClock(int64 SimTimeNs)
{
	if (Kind == EKind::Clock && Handle)
	{
		UrlabRcl_PublishClock(static_cast<UrlabRclClockPub*>(Handle), SimTimeNs);
	}
}

void FMjRosPub::PublishString(const FString& Text)
{
	if (Kind == EKind::String && Handle)
	{
		UrlabRcl_PublishString(static_cast<UrlabRclStringPub*>(Handle), TCHAR_TO_UTF8(*Text));
	}
}

void FMjRosPub::PublishWrench(const double Force3[3], const double Torque3[3], int64 SimTimeNs)
{
	if (Kind == EKind::Wrench && Handle)
	{
		UrlabRcl_PublishWrenchStamped(static_cast<UrlabRclWrenchStampedPub*>(Handle),
			Force3, Torque3, SimTimeNs);
	}
}

void FMjRosPub::PublishRange(double Range, int64 SimTimeNs)
{
	if (Kind == EKind::Range && Handle)
	{
		UrlabRcl_PublishRange(static_cast<UrlabRclRangePub*>(Handle),
			static_cast<float>(Range), SimTimeNs);
	}
}

void FMjRosPub::PublishMagneticField(const double Field3[3], int64 SimTimeNs)
{
	if (Kind == EKind::MagneticField && Handle)
	{
		UrlabRcl_PublishMagneticField(static_cast<UrlabRclMagneticFieldPub*>(Handle),
			Field3, SimTimeNs);
	}
}

void FMjRosPub::PublishFloat64MultiArray(const double* Values, int32 Count)
{
	if (Kind == EKind::MultiArray && Handle)
	{
		UrlabRcl_PublishFloat64MultiArray(
			static_cast<UrlabRclFloat64MultiArrayPub*>(Handle), Values, Count);
	}
}

// --- FMjRosPublisherFactory -------------------------------------------------

FMjRosPub FMjRosPublisherFactory::CreateJointState(const FString& Topic,
	const TArray<FString>& JointNames)
{
	if (!Context)
	{
		return FMjRosPub();
	}
	// The core copies the name strings at create time; hold one UTF-8 buffer per
	// name and hand it a stable pointer array (the outer array is reserved so the
	// element pointers do not move).
	TArray<TArray<ANSICHAR>> NameBytes;
	NameBytes.Reserve(JointNames.Num());
	TArray<const char*> NamePtrs;
	NamePtrs.Reserve(JointNames.Num());
	for (const FString& Name : JointNames)
	{
		FTCHARToUTF8 Conv(*Name);
		TArray<ANSICHAR>& Bytes = NameBytes.AddDefaulted_GetRef();
		Bytes.Append(reinterpret_cast<const ANSICHAR*>(Conv.Get()), Conv.Length());
		Bytes.Add('\0');
		NamePtrs.Add(Bytes.GetData());
	}

	UrlabRclJointStatePub* Pub = UrlabRcl_CreateJointStatePub(Context,
		TCHAR_TO_UTF8(*Topic), NamePtrs.GetData(), NamePtrs.Num());
	if (!Pub)
	{
		UE_LOG(LogURLabRos, Warning, TEXT("ROS: JointState publisher create failed for %s (%hs)"),
			*Topic, UrlabRcl_LastError());
		return FMjRosPub();
	}
	return FMjRosPub(Pub, FMjRosPub::EKind::JointState);
}

FMjRosPub FMjRosPublisherFactory::CreateImu(const FString& Topic, const FString& FrameId)
{
	if (!Context)
	{
		return FMjRosPub();
	}
	UrlabRclImuPub* Pub = UrlabRcl_CreateImuPub(Context, TCHAR_TO_UTF8(*Topic),
		TCHAR_TO_UTF8(*FrameId));
	if (!Pub)
	{
		UE_LOG(LogURLabRos, Warning, TEXT("ROS: Imu publisher create failed for %s (%hs)"),
			*Topic, UrlabRcl_LastError());
		return FMjRosPub();
	}
	return FMjRosPub(Pub, FMjRosPub::EKind::Imu);
}

FMjRosPub FMjRosPublisherFactory::CreateTf(bool bStatic)
{
	if (!Context)
	{
		return FMjRosPub();
	}
	UrlabRclTfPub* Pub = UrlabRcl_CreateTfPub(Context, bStatic ? 1 : 0);
	if (!Pub)
	{
		UE_LOG(LogURLabRos, Warning, TEXT("ROS: /tf publisher create failed (%hs)"),
			UrlabRcl_LastError());
		return FMjRosPub();
	}
	return FMjRosPub(Pub, FMjRosPub::EKind::Tf);
}

FMjRosPub FMjRosPublisherFactory::CreateTwistStamped(const FString& Topic, const FString& FrameId)
{
	if (!Context)
	{
		return FMjRosPub();
	}
	UrlabRclTwistStampedPub* Pub = UrlabRcl_CreateTwistStampedPub(Context,
		TCHAR_TO_UTF8(*Topic), TCHAR_TO_UTF8(*FrameId));
	if (!Pub)
	{
		UE_LOG(LogURLabRos, Warning, TEXT("ROS: TwistStamped publisher create failed for %s (%hs)"),
			*Topic, UrlabRcl_LastError());
		return FMjRosPub();
	}
	return FMjRosPub(Pub, FMjRosPub::EKind::TwistStamped);
}

FMjRosPub FMjRosPublisherFactory::CreateClock()
{
	if (!Context)
	{
		return FMjRosPub();
	}
	UrlabRclClockPub* Pub = UrlabRcl_CreateClockPub(Context);
	if (!Pub)
	{
		UE_LOG(LogURLabRos, Warning, TEXT("ROS: /clock publisher create failed (%hs)"),
			UrlabRcl_LastError());
		return FMjRosPub();
	}
	return FMjRosPub(Pub, FMjRosPub::EKind::Clock);
}

FMjRosPub FMjRosPublisherFactory::CreateString(const FString& Topic)
{
	if (!Context)
	{
		return FMjRosPub();
	}
	UrlabRclStringPub* Pub = UrlabRcl_CreateStringPub(Context, TCHAR_TO_UTF8(*Topic));
	if (!Pub)
	{
		UE_LOG(LogURLabRos, Warning, TEXT("ROS: String publisher create failed for %s (%hs)"),
			*Topic, UrlabRcl_LastError());
		return FMjRosPub();
	}
	return FMjRosPub(Pub, FMjRosPub::EKind::String);
}

FMjRosPub FMjRosPublisherFactory::CreateWrench(const FString& Topic, const FString& FrameId)
{
	if (!Context)
	{
		return FMjRosPub();
	}
	UrlabRclWrenchStampedPub* Pub = UrlabRcl_CreateWrenchStampedPub(Context,
		TCHAR_TO_UTF8(*Topic), TCHAR_TO_UTF8(*FrameId));
	if (!Pub)
	{
		UE_LOG(LogURLabRos, Warning, TEXT("ROS: WrenchStamped publisher create failed for %s (%hs)"),
			*Topic, UrlabRcl_LastError());
		return FMjRosPub();
	}
	return FMjRosPub(Pub, FMjRosPub::EKind::Wrench);
}

FMjRosPub FMjRosPublisherFactory::CreateRange(const FString& Topic, const FString& FrameId,
	uint8 RadiationType, float FieldOfView, float MinRange, float MaxRange)
{
	if (!Context)
	{
		return FMjRosPub();
	}
	UrlabRclRangePub* Pub = UrlabRcl_CreateRangePub(Context, TCHAR_TO_UTF8(*Topic),
		TCHAR_TO_UTF8(*FrameId), RadiationType, FieldOfView, MinRange, MaxRange);
	if (!Pub)
	{
		UE_LOG(LogURLabRos, Warning, TEXT("ROS: Range publisher create failed for %s (%hs)"),
			*Topic, UrlabRcl_LastError());
		return FMjRosPub();
	}
	return FMjRosPub(Pub, FMjRosPub::EKind::Range);
}

FMjRosPub FMjRosPublisherFactory::CreateMagneticField(const FString& Topic, const FString& FrameId)
{
	if (!Context)
	{
		return FMjRosPub();
	}
	UrlabRclMagneticFieldPub* Pub = UrlabRcl_CreateMagneticFieldPub(Context,
		TCHAR_TO_UTF8(*Topic), TCHAR_TO_UTF8(*FrameId));
	if (!Pub)
	{
		UE_LOG(LogURLabRos, Warning, TEXT("ROS: MagneticField publisher create failed for %s (%hs)"),
			*Topic, UrlabRcl_LastError());
		return FMjRosPub();
	}
	return FMjRosPub(Pub, FMjRosPub::EKind::MagneticField);
}

FMjRosPub FMjRosPublisherFactory::CreateFloat64MultiArray(const FString& Topic)
{
	if (!Context)
	{
		return FMjRosPub();
	}
	UrlabRclFloat64MultiArrayPub* Pub = UrlabRcl_CreateFloat64MultiArrayPub(Context,
		TCHAR_TO_UTF8(*Topic));
	if (!Pub)
	{
		UE_LOG(LogURLabRos, Warning, TEXT("ROS: Float64MultiArray publisher create failed for %s (%hs)"),
			*Topic, UrlabRcl_LastError());
		return FMjRosPub();
	}
	return FMjRosPub(Pub, FMjRosPub::EKind::MultiArray);
}

#else  // URLAB_WITH_ROS2

// Absent-ROS stubs: handles are never created (Create* return an invalid handle),
// so publishing and release are no-ops. Providers compile and register in every
// configuration; the transport simply never drives them without a live context.

void FMjRosPub::Reset() {}
void FMjRosPub::PublishJointState(const double*, const double*, const double*, int32, int64) {}
void FMjRosPub::PublishImu(const double*, const double*, const double*, int64) {}
void FMjRosPub::PublishTf(const TArray<FString>&, const TArray<FString>&,
	const TArray<double>&, const TArray<double>&, int64) {}
void FMjRosPub::PublishTwistStamped(const double[3], const double[3], int64) {}
void FMjRosPub::PublishClock(int64) {}
void FMjRosPub::PublishString(const FString&) {}
void FMjRosPub::PublishWrench(const double[3], const double[3], int64) {}
void FMjRosPub::PublishRange(double, int64) {}
void FMjRosPub::PublishMagneticField(const double[3], int64) {}
void FMjRosPub::PublishFloat64MultiArray(const double*, int32) {}

FMjRosPub FMjRosPublisherFactory::CreateJointState(const FString&, const TArray<FString>&) { return FMjRosPub(); }
FMjRosPub FMjRosPublisherFactory::CreateImu(const FString&, const FString&) { return FMjRosPub(); }
FMjRosPub FMjRosPublisherFactory::CreateTf(bool) { return FMjRosPub(); }
FMjRosPub FMjRosPublisherFactory::CreateTwistStamped(const FString&, const FString&) { return FMjRosPub(); }
FMjRosPub FMjRosPublisherFactory::CreateClock() { return FMjRosPub(); }
FMjRosPub FMjRosPublisherFactory::CreateString(const FString&) { return FMjRosPub(); }
FMjRosPub FMjRosPublisherFactory::CreateWrench(const FString&, const FString&) { return FMjRosPub(); }
FMjRosPub FMjRosPublisherFactory::CreateRange(const FString&, const FString&, uint8, float, float, float) { return FMjRosPub(); }
FMjRosPub FMjRosPublisherFactory::CreateMagneticField(const FString&, const FString&) { return FMjRosPub(); }
FMjRosPub FMjRosPublisherFactory::CreateFloat64MultiArray(const FString&) { return FMjRosPub(); }

#endif  // URLAB_WITH_ROS2
