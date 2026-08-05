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
 * The engine's model and data, plus the compiled id `Node` bound to.
 *
 * Says nothing about whether the id is in range: the count to check against is
 * per family and lives on the caller, which knows which one it wants.
 */
bool ResolveBound(const UMjNodeComponent* Node, const mjModel*& OutModel, const mjData*& OutData, int32& OutId)
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
	const mjData* Data = Engine->GetData();
	if (Model == nullptr || Data == nullptr || Id.GetValue() < 0)
	{
		return false;
	}
	OutModel = Model;
	OutData = Data;
	OutId = Id.GetValue();
	return true;
}

/** A bound joint id, for the reads that index by joint rather than by slot. */
bool ResolveJointId(const UMjNodeComponent* Node, const mjModel*& OutModel, const mjData*& OutData, int32& OutId)
{
	if (!IsElementOf(Node, {ElementType::Joint, ElementType::FreeJoint}))
	{
		return false;
	}
	return ResolveBound(Node, OutModel, OutData, OutId) && OutId < static_cast<int32>(OutModel->njnt);
}

bool ResolveTendon(const UMjNodeComponent* Node, const mjModel*& OutModel, const mjData*& OutData, int32& OutId)
{
	if (!IsElementOf(Node, {ElementType::Spatial, ElementType::Fixed}))
	{
		return false;
	}
	return ResolveBound(Node, OutModel, OutData, OutId) && OutId < static_cast<int32>(OutModel->ntendon);
}

/** A bound joint, with its qpos and dof addresses already resolved. */
bool ResolveJoint(const UMjNodeComponent* Node, const mjModel*& OutModel, const mjData*& OutData,
	int32& OutQposAdr, int32& OutDofAdr)
{
	int32 Id = 0;
	if (!ResolveJointId(Node, OutModel, OutData, Id))
	{
		return false;
	}
	OutQposAdr = OutModel->jnt_qposadr[Id];
	OutDofAdr = OutModel->jnt_dofadr[Id];
	return OutQposAdr >= 0 && OutDofAdr >= 0
		&& OutQposAdr < static_cast<int32>(OutModel->nq) && OutDofAdr < static_cast<int32>(OutModel->nv);
}

}  // namespace

bool UMjJointRuntime::IsJoint(const UMjNodeComponent* Node)
{
	return IsElementOf(Node, {ElementType::Joint, ElementType::FreeJoint});
}

float UMjJointRuntime::GetPosition(const UMjNodeComponent* Joint)
{
	const mjModel* Model = nullptr;
	const mjData* Data = nullptr;
	int32 QposAdr = 0;
	int32 DofAdr = 0;
	if (!ResolveJoint(Joint, Model, Data, QposAdr, DofAdr))
	{
		return 0.0f;
	}
	return static_cast<float>(Data->qpos[QposAdr]);
}

float UMjJointRuntime::GetVelocity(const UMjNodeComponent* Joint)
{
	const mjModel* Model = nullptr;
	const mjData* Data = nullptr;
	int32 QposAdr = 0;
	int32 DofAdr = 0;
	if (!ResolveJoint(Joint, Model, Data, QposAdr, DofAdr))
	{
		return 0.0f;
	}
	return static_cast<float>(Data->qvel[DofAdr]);
}

float UMjJointRuntime::GetAcceleration(const UMjNodeComponent* Joint)
{
	const mjModel* Model = nullptr;
	const mjData* Data = nullptr;
	int32 QposAdr = 0;
	int32 DofAdr = 0;
	if (!ResolveJoint(Joint, Model, Data, QposAdr, DofAdr))
	{
		return 0.0f;
	}
	return static_cast<float>(Data->qacc[DofAdr]);
}

void UMjJointRuntime::SetPosition(const UMjNodeComponent* Joint, float Position)
{
	const mjModel* Model = nullptr;
	const mjData* Data = nullptr;
	int32 Id = 0;
	if (!ResolveJointId(Joint, Model, Data, Id))
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
	const mjModel* Model = nullptr;
	const mjData* Data = nullptr;
	int32 Id = 0;
	if (!ResolveJointId(Joint, Model, Data, Id))
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
	const mjModel* Model = nullptr;
	const mjData* Data = nullptr;
	int32 Id = 0;
	if (!ResolveJointId(Joint, Model, Data, Id))
	{
		return FVector2D::ZeroVector;
	}
	return FVector2D(
		static_cast<float>(Model->jnt_range[Id * 2 + 0]),
		static_cast<float>(Model->jnt_range[Id * 2 + 1]));
}

FVector UMjJointRuntime::GetWorldAnchor(const UMjNodeComponent* Joint)
{
	const mjModel* Model = nullptr;
	const mjData* Data = nullptr;
	int32 Id = 0;
	if (!ResolveJointId(Joint, Model, Data, Id))
	{
		return FVector::ZeroVector;
	}
	return URLabAxisConv::MjPositionToUe(&Data->xanchor[Id * 3]);
}

FVector UMjJointRuntime::GetWorldAxis(const UMjNodeComponent* Joint)
{
	const mjModel* Model = nullptr;
	const mjData* Data = nullptr;
	int32 Id = 0;
	if (!ResolveJointId(Joint, Model, Data, Id))
	{
		return FVector::ForwardVector;
	}
	// An axis is a direction, so it takes the handedness flip without the
	// metres-to-centimetres scaling a position would.
	const mjtNum* Axis = &Data->xaxis[Id * 3];
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
	const mjModel* Model = nullptr;
	const mjData* Data = nullptr;
	int32 Id = 0;
	if (!ResolveTendon(Tendon, Model, Data, Id))
	{
		return 0.0f;
	}
	return static_cast<float>(Data->ten_length[Id]);
}

float UMjTendonRuntime::GetVelocity(const UMjNodeComponent* Tendon)
{
	const mjModel* Model = nullptr;
	const mjData* Data = nullptr;
	int32 Id = 0;
	if (!ResolveTendon(Tendon, Model, Data, Id))
	{
		return 0.0f;
	}
	return static_cast<float>(Data->ten_velocity[Id]);
}

#else  // URLAB_MJ_GEN

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

#endif  // URLAB_MJ_GEN
