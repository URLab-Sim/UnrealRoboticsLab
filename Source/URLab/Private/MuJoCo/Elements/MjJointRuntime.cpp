// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Elements/MjJointRuntime.h"

#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Utils/URLabAxisConv.h"

#if URLAB_MJ_GEN
#include "MuJoCo/Spec/MjGenHooks.h"
#include "MuJoCo/Gen/MjDispatch.gen.h"
#endif

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

#if URLAB_MJ_GEN

namespace
{
using urlab::spec::psm::ElementType;

bool IsElementOf(const UMjNodeComponent* Node, std::initializer_list<ElementType> Kinds)
{
	if (Node == nullptr)
	{
		return false;
	}
	ElementType Type;
	if (!urlab::spec::gen::ElementTypeOfNode(*Node, Type))
	{
		return false;
	}
	for (const ElementType Kind : Kinds)
	{
		if (Kind == Type)
		{
			return true;
		}
	}
	return false;
}

/**
 * The engine, its model, and the compiled id `Node` bound to.
 *
 * The model is the compiled shape -- addresses, counts, ranges -- and is what
 * an accessor indexes WITH. The values themselves come from the engine's
 * published snapshot rather than from live mjData, so two questions asked
 * during one step cannot be answered out of two different steps.
 *
 * Says nothing about whether the id is in range: the count to check against is
 * per family and lives on the caller, which knows which one it wants.
 */
bool ResolveBound(const UMjNodeComponent* Node, const UMjPhysicsEngine*& OutEngine,
	const mjModel*& OutModel, int32& OutId)
{
	if (Node == nullptr)
	{
		return false;
	}
	const TOptional<int32>& Id = Node->GetBoundId();
	if (!Id.IsSet())
	{
		return false;
	}
	const UMjPhysicsEngine* Engine = AAMjManager::ResolveEngine(Node);
	if (Engine == nullptr)
	{
		return false;
	}
	const mjModel* Model = Engine->GetModel();
	if (Model == nullptr || Id.GetValue() < 0)
	{
		return false;
	}
	OutEngine = Engine;
	OutModel = Model;
	OutId = Id.GetValue();
	return true;
}

/** A bound joint id, for the reads that index by joint rather than by slot. */
bool ResolveJointId(const UMjNodeComponent* Node, const UMjPhysicsEngine*& OutEngine,
	const mjModel*& OutModel, int32& OutId)
{
	if (!IsElementOf(Node, {ElementType::Joint, ElementType::FreeJoint}))
	{
		return false;
	}
	return ResolveBound(Node, OutEngine, OutModel, OutId) && OutId < static_cast<int32>(OutModel->njnt);
}

bool ResolveTendon(const UMjNodeComponent* Node, const UMjPhysicsEngine*& OutEngine,
	const mjModel*& OutModel, int32& OutId)
{
	if (!IsElementOf(Node, {ElementType::Spatial, ElementType::Fixed}))
	{
		return false;
	}
	return ResolveBound(Node, OutEngine, OutModel, OutId) && OutId < static_cast<int32>(OutModel->ntendon);
}

/** A bound joint, with its qpos and dof addresses already resolved. */
bool ResolveJoint(const UMjNodeComponent* Node, const UMjPhysicsEngine*& OutEngine,
	const mjModel*& OutModel, int32& OutQposAdr, int32& OutDofAdr)
{
	int32 Id = 0;
	if (!ResolveJointId(Node, OutEngine, OutModel, Id))
	{
		return false;
	}
	OutQposAdr = OutModel->jnt_qposadr[Id];
	OutDofAdr = OutModel->jnt_dofadr[Id];
	return OutQposAdr >= 0 && OutDofAdr >= 0
		&& OutQposAdr < static_cast<int32>(OutModel->nq) && OutDofAdr < static_cast<int32>(OutModel->nv);
}

} // namespace

bool UMjJointRuntime::IsJoint(const UMjNodeComponent* Node)
{
	return IsElementOf(Node, {ElementType::Joint, ElementType::FreeJoint});
}

float UMjJointRuntime::GetPosition(const UMjNodeComponent* Joint)
{
	const UMjPhysicsEngine* Engine = nullptr;
	const mjModel* Model = nullptr;
	int32 QposAdr = 0;
	int32 DofAdr = 0;
	if (!ResolveJoint(Joint, Engine, Model, QposAdr, DofAdr))
	{
		return 0.0f;
	}
	return static_cast<float>(MjSnapshotValue(*Engine, QposAdr,
		[](const FMjRenderSnapshot& S) -> const TArray<mjtNum>& { return S.QPos; }));
}

float UMjJointRuntime::GetVelocity(const UMjNodeComponent* Joint)
{
	const UMjPhysicsEngine* Engine = nullptr;
	const mjModel* Model = nullptr;
	int32 QposAdr = 0;
	int32 DofAdr = 0;
	if (!ResolveJoint(Joint, Engine, Model, QposAdr, DofAdr))
	{
		return 0.0f;
	}
	return static_cast<float>(MjSnapshotValue(*Engine, DofAdr,
		[](const FMjRenderSnapshot& S) -> const TArray<mjtNum>& { return S.QVel; }));
}

float UMjJointRuntime::GetAcceleration(const UMjNodeComponent* Joint)
{
	const UMjPhysicsEngine* Engine = nullptr;
	const mjModel* Model = nullptr;
	int32 QposAdr = 0;
	int32 DofAdr = 0;
	if (!ResolveJoint(Joint, Engine, Model, QposAdr, DofAdr))
	{
		return 0.0f;
	}
	return static_cast<float>(MjSnapshotValue(*Engine, DofAdr,
		[](const FMjRenderSnapshot& S) -> const TArray<mjtNum>& { return S.QAcc; }));
}

void UMjJointRuntime::SetPosition(const UMjNodeComponent* Joint, float Position)
{
	const UMjPhysicsEngine* Bound = nullptr;
	const mjModel* Model = nullptr;
	int32 Id = 0;
	if (!ResolveJointId(Joint, Bound, Model, Id))
	{
		return;
	}
	if (UMjPhysicsEngine* Engine = AAMjManager::ResolveEngine(Joint))
	{
		Engine->ApplyJointPosition(Id, static_cast<double>(Position));
	}
}

void UMjJointRuntime::SetVelocity(const UMjNodeComponent* Joint, float Velocity)
{
	const UMjPhysicsEngine* Bound = nullptr;
	const mjModel* Model = nullptr;
	int32 Id = 0;
	if (!ResolveJointId(Joint, Bound, Model, Id))
	{
		return;
	}
	if (UMjPhysicsEngine* Engine = AAMjManager::ResolveEngine(Joint))
	{
		Engine->ApplyJointVelocity(Id, static_cast<double>(Velocity));
	}
}

FVector2D UMjJointRuntime::GetJointRange(const UMjNodeComponent* Joint)
{
	const UMjPhysicsEngine* Engine = nullptr;
	const mjModel* Model = nullptr;
	int32 Id = 0;
	if (!ResolveJointId(Joint, Engine, Model, Id))
	{
		return FVector2D::ZeroVector;
	}
	// A compiled range is model state, fixed for the life of a compile, so
	// there is nothing here a step could tear.
	return FVector2D(
		static_cast<float>(Model->jnt_range[Id * 2 + 0]),
		static_cast<float>(Model->jnt_range[Id * 2 + 1]));
}

FVector UMjJointRuntime::GetWorldAnchor(const UMjNodeComponent* Joint)
{
	const UMjPhysicsEngine* Engine = nullptr;
	const mjModel* Model = nullptr;
	int32 Id = 0;
	if (!ResolveJointId(Joint, Engine, Model, Id))
	{
		return FVector::ZeroVector;
	}
	double Anchor[3] = {0.0, 0.0, 0.0};
	if (!MjSnapshotRange(*Engine, Id * 3, 3, Anchor,
			[](const FMjRenderSnapshot& S) -> const TArray<mjtNum>& { return S.JntXAnchor; }))
	{
		return FVector::ZeroVector;
	}
	return URLabAxisConv::MjPositionToUe(Anchor);
}

FVector UMjJointRuntime::GetWorldAxis(const UMjNodeComponent* Joint)
{
	const UMjPhysicsEngine* Engine = nullptr;
	const mjModel* Model = nullptr;
	int32 Id = 0;
	if (!ResolveJointId(Joint, Engine, Model, Id))
	{
		return FVector::ForwardVector;
	}
	double Axis[3] = {0.0, 0.0, 0.0};
	if (!MjSnapshotRange(*Engine, Id * 3, 3, Axis,
			[](const FMjRenderSnapshot& S) -> const TArray<mjtNum>& { return S.JntXAxis; }))
	{
		return FVector::ForwardVector;
	}
	// An axis is a direction, so it takes the handedness flip without the
	// metres-to-centimetres scaling a position would.
	return FVector(
		static_cast<float>(Axis[0]),
		-static_cast<float>(Axis[1]),
		static_cast<float>(Axis[2]));
}

bool UMjTendonRuntime::IsTendon(const UMjNodeComponent* Node)
{
	return IsElementOf(Node, {ElementType::Spatial, ElementType::Fixed});
}

float UMjTendonRuntime::GetLength(const UMjNodeComponent* Tendon)
{
	const UMjPhysicsEngine* Engine = nullptr;
	const mjModel* Model = nullptr;
	int32 Id = 0;
	if (!ResolveTendon(Tendon, Engine, Model, Id))
	{
		return 0.0f;
	}
	return static_cast<float>(MjSnapshotValue(*Engine, Id,
		[](const FMjRenderSnapshot& S) -> const TArray<mjtNum>& { return S.TenLength; }));
}

float UMjTendonRuntime::GetVelocity(const UMjNodeComponent* Tendon)
{
	const UMjPhysicsEngine* Engine = nullptr;
	const mjModel* Model = nullptr;
	int32 Id = 0;
	if (!ResolveTendon(Tendon, Engine, Model, Id))
	{
		return 0.0f;
	}
	return static_cast<float>(MjSnapshotValue(*Engine, Id,
		[](const FMjRenderSnapshot& S) -> const TArray<mjtNum>& { return S.TenVelocity; }));
}

#else // URLAB_MJ_GEN

bool UMjJointRuntime::IsJoint(const UMjNodeComponent*)
{
	return false;
}

float UMjJointRuntime::GetPosition(const UMjNodeComponent*)
{
	return 0.0f;
}

void UMjJointRuntime::SetPosition(const UMjNodeComponent*, float)
{
}

float UMjJointRuntime::GetVelocity(const UMjNodeComponent*)
{
	return 0.0f;
}

void UMjJointRuntime::SetVelocity(const UMjNodeComponent*, float)
{
}

float UMjJointRuntime::GetAcceleration(const UMjNodeComponent*)
{
	return 0.0f;
}

FVector2D UMjJointRuntime::GetJointRange(const UMjNodeComponent*)
{
	return FVector2D::ZeroVector;
}

FVector UMjJointRuntime::GetWorldAnchor(const UMjNodeComponent*)
{
	return FVector::ZeroVector;
}

FVector UMjJointRuntime::GetWorldAxis(const UMjNodeComponent*)
{
	return FVector::ForwardVector;
}

bool UMjTendonRuntime::IsTendon(const UMjNodeComponent*)
{
	return false;
}

float UMjTendonRuntime::GetLength(const UMjNodeComponent*)
{
	return 0.0f;
}

float UMjTendonRuntime::GetVelocity(const UMjNodeComponent*)
{
	return 0.0f;
}

#endif // URLAB_MJ_GEN
