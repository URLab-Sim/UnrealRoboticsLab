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

#pragma once

#include "CoreMinimal.h"

struct FMjStateSnapshot;
struct UrlabRclContext;
class AAMjManager;

/**
 * @class FMjRosPub
 * @brief An owning handle to one rcl publisher of a fixed message family.
 *
 * Created only by FMjRosPublisherFactory, which hides the C-ABI seam entirely, so
 * a provider (built-in or user, in any module) never touches rcl. Move-only; the
 * destructor releases the underlying publisher, so a provider that holds FMjRosPub
 * members cannot leak one. Each handle carries a message-family tag: the Publish*
 * method matching the create call fills and sends, and any mismatched Publish* is
 * a safe no-op. Every method is a no-op on an invalid handle and when ROS is not
 * linked, so provider code is written once and runs in every configuration.
 */
class URLABROS_API FMjRosPub
{
public:
	FMjRosPub() = default;
	~FMjRosPub() { Reset(); }

	FMjRosPub(FMjRosPub&& Other) noexcept
		: Handle(Other.Handle), Kind(Other.Kind)
	{
		Other.Handle = nullptr;
		Other.Kind = EKind::None;
	}
	FMjRosPub& operator=(FMjRosPub&& Other) noexcept
	{
		if (this != &Other)
		{
			Reset();
			Handle = Other.Handle;
			Kind = Other.Kind;
			Other.Handle = nullptr;
			Other.Kind = EKind::None;
		}
		return *this;
	}
	FMjRosPub(const FMjRosPub&) = delete;
	FMjRosPub& operator=(const FMjRosPub&) = delete;

	bool IsValid() const { return Handle != nullptr; }

	/** Release the underlying rcl publisher, if any. */
	void Reset();

	// Typed publish operations. Exactly one matches this handle's family; the rest
	// are no-ops. SimTimeNs is sim time in nanoseconds.
	void PublishJointState(const double* Positions, const double* Velocities,
		const double* Efforts, int32 Count, int64 SimTimeNs);
	void PublishImu(const double* AngularVel3, const double* LinearAccel3,
		const double* OrientationXyzw4, int64 SimTimeNs);
	void PublishTf(const TArray<FString>& Parents, const TArray<FString>& Children,
		const TArray<double>& TranslationsXyz, const TArray<double>& RotationsXyzw,
		int64 SimTimeNs);
	void PublishTwistStamped(const double Linear3[3], const double Angular3[3], int64 SimTimeNs);
	void PublishClock(int64 SimTimeNs);
	void PublishString(const FString& Text);
	void PublishWrench(const double Force3[3], const double Torque3[3], int64 SimTimeNs);
	void PublishRange(double Range, int64 SimTimeNs);
	void PublishMagneticField(const double Field3[3], int64 SimTimeNs);
	void PublishFloat64MultiArray(const double* Values, int32 Count);

private:
	enum class EKind : uint8
	{
		None, JointState, Imu, Tf, TwistStamped, Clock, String,
		Wrench, Range, MagneticField, MultiArray
	};

	FMjRosPub(void* InHandle, EKind InKind) : Handle(InHandle), Kind(InKind) {}

	void* Handle = nullptr;
	EKind Kind = EKind::None;

	friend class FMjRosPublisherFactory;
};

/**
 * @class FMjRosPublisherFactory
 * @brief The single place providers turn a topic into an owning FMjRosPub.
 *
 * One is built per publisher rebuild by the transport and handed to every
 * provider's Build. Each Create* wraps one UrlabRclCore triple; providers hold the
 * returned handles and publish through them, never seeing rcl. Also exposes the
 * owning manager for providers that read model-structure state (e.g. the exported
 * URDF for /<art>/robot_description).
 */
class URLABROS_API FMjRosPublisherFactory
{
public:
	FMjRosPublisherFactory(UrlabRclContext* InContext, const AAMjManager* InManager)
		: Context(InContext), Manager(InManager)
	{
	}

	bool IsValid() const { return Context != nullptr; }
	const AAMjManager* GetManager() const { return Manager; }

	FMjRosPub CreateJointState(const FString& Topic, const TArray<FString>& JointNames);
	FMjRosPub CreateImu(const FString& Topic, const FString& FrameId);
	FMjRosPub CreateTf(bool bStatic);
	FMjRosPub CreateTwistStamped(const FString& Topic, const FString& FrameId);
	FMjRosPub CreateClock();
	FMjRosPub CreateString(const FString& Topic);
	FMjRosPub CreateWrench(const FString& Topic, const FString& FrameId);
	FMjRosPub CreateRange(const FString& Topic, const FString& FrameId,
		uint8 RadiationType, float FieldOfView, float MinRange, float MaxRange);
	FMjRosPub CreateMagneticField(const FString& Topic, const FString& FrameId);
	FMjRosPub CreateFloat64MultiArray(const FString& Topic);

private:
	UrlabRclContext* Context = nullptr;
	const AAMjManager* Manager = nullptr;
};

/**
 * @class IMjRosOutputProvider
 * @brief One ROS output (a message type or a per-articulation family of them),
 *        self-registered into FMjRosOutputRegistry.
 *
 * The built-in outputs (JointState, Imu, tf2, TwistStamped, Clock, sensor
 * routing, robot_description) are each just an instance of this interface,
 * registered through the same registry a user's out-of-plugin provider uses.
 * Adding an output is dropping a self-registering file; there is no central switch
 * to edit. The transport instantiates one provider per registered entry on a
 * structure change, calls Build to create publishers for the current model, and
 * calls Publish each step. Destroying the provider releases its publishers.
 *
 * Build and Publish run on the physics thread; Publish must be fast and
 * non-blocking.
 */
class URLABROS_API IMjRosOutputProvider
{
public:
	virtual ~IMjRosOutputProvider() = default;

	/** A stable, unique name (also the key used to register). */
	virtual FName GetProviderName() const = 0;

	/** Create publishers for the current model shape. Called on every structure
	 *  change (a fresh provider instance each time). */
	virtual void Build(FMjRosPublisherFactory& Factory, const FMjStateSnapshot& Snapshot) = 0;

	/** Fill and publish this output from the per-step snapshot. */
	virtual void Publish(const FMjStateSnapshot& Snapshot, int64 SimTimeNs) = 0;

	/** Number of publishers this provider currently owns; a test seam, default 0. */
	virtual int32 GetPublisherCountForTest() const { return 0; }
};

/** Factory function that produces a fresh provider instance. Each transport owns
 *  its own provider set, so providers are instantiated per transport, not shared. */
using FMjRosOutputProviderFactoryFn = TFunction<TUniquePtr<IMjRosOutputProvider>()>;

/**
 * @class FMjRosOutputRegistry
 * @brief The process-wide table of registered output providers.
 *
 * Populated by self-registering statics at module load (see
 * REGISTER_MJ_ROS_OUTPUT_PROVIDER), independent of whether ROS is linked. The
 * transport asks it to instantiate the full set on each publisher rebuild.
 */
class URLABROS_API FMjRosOutputRegistry
{
public:
	static FMjRosOutputRegistry& Get();

	/** Register a named provider factory. A later duplicate name replaces the
	 *  earlier entry (with a warning), so a user can override a built-in by name. */
	void Register(FName Name, FMjRosOutputProviderFactoryFn Factory);

	/** Instantiate one provider per registered entry, in registration order. */
	void InstantiateAll(TArray<TUniquePtr<IMjRosOutputProvider>>& Out) const;

	/** The registered names, in registration order. Test / introspection seam. */
	TArray<FName> GetRegisteredNames() const;

	int32 Num() const { return Entries.Num(); }

private:
	TArray<TPair<FName, FMjRosOutputProviderFactoryFn>> Entries;
};

/** Self-registration helper: a file-scope instance registers its factory at
 *  module load. Use REGISTER_MJ_ROS_OUTPUT_PROVIDER rather than constructing
 *  directly. */
struct URLABROS_API FMjRosOutputProviderRegistrar
{
	FMjRosOutputProviderRegistrar(FName Name, FMjRosOutputProviderFactoryFn Factory);
};

/**
 * Register a provider type under a topic-family name. Drop this at file scope in a
 * provider .cpp; no other file needs to change. Example:
 *   REGISTER_MJ_ROS_OUTPUT_PROVIDER("joint_state", FMjRosJointStateProvider);
 */
#define REGISTER_MJ_ROS_OUTPUT_PROVIDER(NameLiteral, Type)                      \
	static const FMjRosOutputProviderRegistrar GMjRosProviderRegistrar_##Type(  \
		FName(TEXT(NameLiteral)),                                               \
		[]() -> TUniquePtr<IMjRosOutputProvider> { return MakeUnique<Type>(); })
