// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Elements/MjActuatorRuntime.h"

#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Spec/MjNodeComponent.h"

#if URLAB_MJ_GEN
#include "MuJoCo/Spec/MjGenHooks.h"
#include "MuJoCo/Gen/MjDispatch.gen.h"
#include "MuJoCo/Gen/MjElements.gen.h"
#endif

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

#if URLAB_MJ_GEN

namespace
{
using urlab::spec::psm::ElementType;

/**
 * The element types the schema lets `<actuator>` hold.
 *
 * Taken from the grammar rather than listed by hand, so an actuator kind
 * MuJoCo gains is one the library accepts the day the schema does. Built once;
 * the set is fixed for the life of the process.
 */
const TSet<ElementType>& ActuatorKinds()
{
	static const TSet<ElementType> Kinds = [] {
		TSet<ElementType> Out;
		urlab::spec::gen::ChildSlots(static_cast<const UMjActuator*>(nullptr),
			[&Out](int32, auto Tag) {
				using Leaf = typename decltype(Tag)::type;
				Out.Add(urlab::spec::gen::TMjElementType<Leaf>::Value);
			});
		return Out;
	}();
	return Kinds;
}

/** The compiled id `Node` bound to, if it is an actuator that bound at all. */
bool ResolveActuatorId(const UMjNodeComponent* Node, int32& OutId)
{
	if (!UMjActuatorRuntime::IsActuator(Node))
	{
		return false;
	}
	const TOptional<int32>& Id = Node->GetBoundId();
	if (!Id.IsSet() || Id.GetValue() < 0)
	{
		return false;
	}
	OutId = Id.GetValue();
	return true;
}

/**
 * The id plus the engine and its model, with the id known to be in range.
 *
 * The model carries the compiled shape an accessor indexes with; the values
 * come from the engine's published snapshot rather than from live mjData, so
 * two questions asked during one step cannot be answered out of two steps.
 */
bool ResolveBound(const UMjNodeComponent* Node, const UMjPhysicsEngine*& OutEngine,
	const mjModel*& OutModel, int32& OutId)
{
	int32 Id = -1;
	if (!ResolveActuatorId(Node, Id))
	{
		return false;
	}
	const UMjPhysicsEngine* Engine = AAMjManager::ResolveEngine(Node);
	if (Engine == nullptr)
	{
		return false;
	}
	const mjModel* Model = Engine->GetModel();
	if (Model == nullptr || Id >= Model->nu)
	{
		return false;
	}
	OutEngine = Engine;
	OutModel = Model;
	OutId = Id;
	return true;
}

} // namespace

bool UMjActuatorRuntime::IsActuator(const UMjNodeComponent* Node)
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
	return ActuatorKinds().Contains(Type);
}

AMjArticulation* UMjActuatorRuntime::OwningArticulation(const UMjNodeComponent* Actuator)
{
	if (Actuator == nullptr)
	{
		return nullptr;
	}
	return Cast<AMjArticulation>(Actuator->GetOwner());
}

void UMjActuatorRuntime::SetControl(const UMjNodeComponent* Actuator, double Value)
{
	int32 Id = -1;
	AMjArticulation* Art = OwningArticulation(Actuator);
	if (Art != nullptr && ResolveActuatorId(Actuator, Id))
	{
		Art->StageInternalControl(Id, Value);
	}
}

void UMjActuatorRuntime::SetNetworkControl(const UMjNodeComponent* Actuator, double Value)
{
	int32 Id = -1;
	AMjArticulation* Art = OwningArticulation(Actuator);
	if (Art != nullptr && ResolveActuatorId(Actuator, Id))
	{
		Art->StageNetworkControl(Id, Value);
	}
}

void UMjActuatorRuntime::ResetControl(const UMjNodeComponent* Actuator)
{
	int32 Id = -1;
	AMjArticulation* Art = OwningArticulation(Actuator);
	if (Art != nullptr && ResolveActuatorId(Actuator, Id))
	{
		Art->ClearStagedControl(Id);
	}
}

double UMjActuatorRuntime::GetControl(const UMjNodeComponent* Actuator)
{
	int32 Id = -1;
	const AMjArticulation* Art = OwningArticulation(Actuator);
	if (Art == nullptr || !ResolveActuatorId(Actuator, Id))
	{
		return 0.0f;
	}
	return Art->ResolveDesiredControl(Id);
}

double UMjActuatorRuntime::ResolveDesiredControl(const UMjNodeComponent* Actuator, uint8 Source)
{
	int32 Id = -1;
	const AMjArticulation* Art = OwningArticulation(Actuator);
	if (Art == nullptr || !ResolveActuatorId(Actuator, Id))
	{
		return 0.0f;
	}
	return Art->ResolveDesiredControl(Id, Source);
}

float UMjActuatorRuntime::GetAppliedControl(const UMjNodeComponent* Actuator)
{
	const UMjPhysicsEngine* Engine = nullptr;
	const mjModel* Model = nullptr;
	int32 Id = -1;
	if (!ResolveBound(Actuator, Engine, Model, Id))
	{
		return 0.0f;
	}
	return static_cast<float>(MjSnapshotValue(*Engine, Id,
		[](const FMjRenderSnapshot& S) -> const TArray<mjtNum>& { return S.Ctrl; }));
}

float UMjActuatorRuntime::GetForce(const UMjNodeComponent* Actuator)
{
	const UMjPhysicsEngine* Engine = nullptr;
	const mjModel* Model = nullptr;
	int32 Id = -1;
	if (!ResolveBound(Actuator, Engine, Model, Id))
	{
		return 0.0f;
	}
	return static_cast<float>(MjSnapshotValue(*Engine, Id,
		[](const FMjRenderSnapshot& S) -> const TArray<mjtNum>& { return S.ActuatorForce; }));
}

float UMjActuatorRuntime::GetLength(const UMjNodeComponent* Actuator)
{
	const UMjPhysicsEngine* Engine = nullptr;
	const mjModel* Model = nullptr;
	int32 Id = -1;
	if (!ResolveBound(Actuator, Engine, Model, Id))
	{
		return 0.0f;
	}
	return static_cast<float>(MjSnapshotValue(*Engine, Id,
		[](const FMjRenderSnapshot& S) -> const TArray<mjtNum>& { return S.ActuatorLength; }));
}

float UMjActuatorRuntime::GetVelocity(const UMjNodeComponent* Actuator)
{
	const UMjPhysicsEngine* Engine = nullptr;
	const mjModel* Model = nullptr;
	int32 Id = -1;
	if (!ResolveBound(Actuator, Engine, Model, Id))
	{
		return 0.0f;
	}
	return static_cast<float>(MjSnapshotValue(*Engine, Id,
		[](const FMjRenderSnapshot& S) -> const TArray<mjtNum>& { return S.ActuatorVelocity; }));
}

FVector2D UMjActuatorRuntime::GetControlRange(const UMjNodeComponent* Actuator)
{
	const UMjPhysicsEngine* Engine = nullptr;
	const mjModel* Model = nullptr;
	int32 Id = -1;
	if (!ResolveBound(Actuator, Engine, Model, Id))
	{
		return FVector2D::ZeroVector;
	}
	// A compiled control range is model state, fixed for the life of a
	// compile, so there is nothing here a step could tear.
	return FVector2D(static_cast<float>(Model->actuator_ctrlrange[2 * Id + 0]),
		static_cast<float>(Model->actuator_ctrlrange[2 * Id + 1]));
}

float UMjActuatorRuntime::GetActivation(const UMjNodeComponent* Actuator)
{
	const UMjPhysicsEngine* Engine = nullptr;
	const mjModel* Model = nullptr;
	int32 Id = -1;
	if (!ResolveBound(Actuator, Engine, Model, Id))
	{
		return 0.0f;
	}
	// Negative for a stateless actuator, which has no activation to report.
	const int32 ActAdr = Model->actuator_actadr[Id];
	return ActAdr >= 0
			 ? static_cast<float>(MjSnapshotValue(*Engine, ActAdr,
				   [](const FMjRenderSnapshot& S) -> const TArray<mjtNum>& { return S.Act; }))
			 : 0.0f;
}

TArray<float> UMjActuatorRuntime::GetGear(const UMjNodeComponent* Actuator)
{
	TArray<float> Out;
	const UMjPhysicsEngine* Engine = nullptr;
	const mjModel* Model = nullptr;
	int32 Id = -1;
	if (!ResolveBound(Actuator, Engine, Model, Id))
	{
		return Out;
	}
	Out.Reserve(6);
	for (int32 i = 0; i < 6; ++i)
	{
		Out.Add(static_cast<float>(Model->actuator_gear[6 * Id + i]));
	}
	return Out;
}

void UMjActuatorRuntime::SetGear(const UMjNodeComponent* Actuator, const TArray<float>& Gear)
{
	int32 Id = -1;
	if (!ResolveActuatorId(Actuator, Id))
	{
		return;
	}
	UMjPhysicsEngine* Engine = AAMjManager::ResolveEngine(Actuator);
	if (Engine == nullptr)
	{
		return;
	}
	const int32 Num = FMath::Min(Gear.Num(), 6);
	TArray<double, TInlineAllocator<6>> Values;
	Values.Reserve(Num);
	for (int32 i = 0; i < Num; ++i)
	{
		Values.Add(static_cast<double>(Gear[i]));
	}
	Engine->ApplyActuatorGear(Id, Values);
}

#else // URLAB_MJ_GEN

bool UMjActuatorRuntime::IsActuator(const UMjNodeComponent*)
{
	return false;
}

AMjArticulation* UMjActuatorRuntime::OwningArticulation(const UMjNodeComponent*)
{
	return nullptr;
}

void UMjActuatorRuntime::SetControl(const UMjNodeComponent*, double) {}
void UMjActuatorRuntime::SetNetworkControl(const UMjNodeComponent*, double) {}
void UMjActuatorRuntime::ResetControl(const UMjNodeComponent*) {}

double UMjActuatorRuntime::GetControl(const UMjNodeComponent*)
{
	return 0.0;
}

double UMjActuatorRuntime::ResolveDesiredControl(const UMjNodeComponent*, uint8)
{
	return 0.0;
}

float UMjActuatorRuntime::GetAppliedControl(const UMjNodeComponent*)
{
	return 0.0f;
}

float UMjActuatorRuntime::GetForce(const UMjNodeComponent*)
{
	return 0.0f;
}

float UMjActuatorRuntime::GetLength(const UMjNodeComponent*)
{
	return 0.0f;
}

float UMjActuatorRuntime::GetVelocity(const UMjNodeComponent*)
{
	return 0.0f;
}

FVector2D UMjActuatorRuntime::GetControlRange(const UMjNodeComponent*)
{
	return FVector2D::ZeroVector;
}

float UMjActuatorRuntime::GetActivation(const UMjNodeComponent*)
{
	return 0.0f;
}

TArray<float> UMjActuatorRuntime::GetGear(const UMjNodeComponent*)
{
	return TArray<float>();
}

void UMjActuatorRuntime::SetGear(const UMjNodeComponent*, const TArray<float>&) {}

#endif // URLAB_MJ_GEN
