// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Elements/MjSensorRuntime.h"

#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Spec/MjNodeComponent.h"

#if URLAB_MJ_GEN
#include "MuJoCo/Spec/MjGenHooks.h"
#include "MuJoCo/Gen/MjDispatch.gen.h"
#endif

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

void MjTransformSensorReading(TArray<float>& Values, EMjSensorValueKind Kind)
{
	switch (Kind)
	{
		case EMjSensorValueKind::Position:
			if (Values.Num() >= 3)
			{
				Values[0] *= 100.0f;
				Values[1] *= -100.0f;
				Values[2] *= 100.0f;
			}
			break;

		case EMjSensorValueKind::Direction:
		case EMjSensorValueKind::Vector3:
			if (Values.Num() >= 3)
			{
				Values[1] = -Values[1];
			}
			break;

		// MuJoCo orders a quaternion (w, x, y, z) and Unreal constructs from
		// (X, Y, Z, W); the sign flips are the handedness change, not the
		// reordering, and dropping them mirrors the model about Y.
		case EMjSensorValueKind::Quaternion:
			if (Values.Num() >= 4)
			{
				const float W = Values[0];
				const float X = Values[1];
				const float Y = Values[2];
				const float Z = Values[3];
				Values[0] = -X;
				Values[1] = Y;
				Values[2] = -Z;
				Values[3] = W;
			}
			break;

		case EMjSensorValueKind::GeomFromTo:
			if (Values.Num() >= 6)
			{
				for (int32 Base = 0; Base < 6; Base += 3)
				{
					Values[Base + 0] *= 100.0f;
					Values[Base + 1] *= -100.0f;
					Values[Base + 2] *= 100.0f;
				}
			}
			break;

		case EMjSensorValueKind::Scalar:
			break;
	}
}

#if URLAB_MJ_GEN

namespace
{
using urlab::spec::psm::ElementType;

/**
 * What each sensor kind means and how its output is shaped.
 *
 * Keyed on the schema element type, so a sensor kind the grammar gains shows up
 * here as a missing row rather than as a silently wrong reading: the lookup
 * fails closed and the sensor reads as an untransformed Generic scalar.
 *
 * `FixedDim` is the schema's fixed width, or -1 where only the compiled model
 * knows it. Nothing reads it to size a buffer -- m->sensor_dim does that -- so
 * it is here for validation and for callers sizing a message ahead of a compile.
 */
struct FSensorKind
{
	ElementType Type;
	EMjSensorSemantic Semantic;
	EMjSensorValueKind ValueKind;
	int32 FixedDim;
};

const FSensorKind SensorKinds[] = {
	{ElementType::Accelerometer, EMjSensorSemantic::Accel, EMjSensorValueKind::Vector3, 3},
	{ElementType::Actuatorfrc, EMjSensorSemantic::ActuatorFrc, EMjSensorValueKind::Scalar, 1},
	{ElementType::Actuatorpos, EMjSensorSemantic::ActuatorPos, EMjSensorValueKind::Scalar, 1},
	{ElementType::Actuatorvel, EMjSensorSemantic::ActuatorVel, EMjSensorValueKind::Scalar, 1},
	{ElementType::Ballangvel, EMjSensorSemantic::Generic, EMjSensorValueKind::Vector3, 3},
	{ElementType::Ballquat, EMjSensorSemantic::Generic, EMjSensorValueKind::Quaternion, 4},
	{ElementType::Camprojection, EMjSensorSemantic::Generic, EMjSensorValueKind::Scalar, 2},
	{ElementType::Clock, EMjSensorSemantic::Clock, EMjSensorValueKind::Scalar, 1},
	{ElementType::Distance, EMjSensorSemantic::Generic, EMjSensorValueKind::Scalar, 1},
	{ElementType::EKinetic, EMjSensorSemantic::Generic, EMjSensorValueKind::Scalar, 1},
	{ElementType::EPotential, EMjSensorSemantic::Generic, EMjSensorValueKind::Scalar, 1},
	{ElementType::Force, EMjSensorSemantic::Force, EMjSensorValueKind::Vector3, 3},
	{ElementType::Frameangacc, EMjSensorSemantic::FrameAngAcc, EMjSensorValueKind::Vector3, 3},
	{ElementType::Frameangvel, EMjSensorSemantic::FrameAngVel, EMjSensorValueKind::Vector3, 3},
	{ElementType::Framelinacc, EMjSensorSemantic::FrameLinAcc, EMjSensorValueKind::Vector3, 3},
	{ElementType::Framelinvel, EMjSensorSemantic::FrameLinVel, EMjSensorValueKind::Vector3, 3},
	{ElementType::Framepos, EMjSensorSemantic::FramePos, EMjSensorValueKind::Position, 3},
	{ElementType::Framequat, EMjSensorSemantic::FrameQuat, EMjSensorValueKind::Quaternion, 4},
	{ElementType::Framexaxis, EMjSensorSemantic::FrameAxis, EMjSensorValueKind::Direction, 3},
	{ElementType::Frameyaxis, EMjSensorSemantic::FrameAxis, EMjSensorValueKind::Direction, 3},
	{ElementType::Framezaxis, EMjSensorSemantic::FrameAxis, EMjSensorValueKind::Direction, 3},
	{ElementType::Fromto, EMjSensorSemantic::Generic, EMjSensorValueKind::GeomFromTo, 6},
	{ElementType::Gyro, EMjSensorSemantic::Gyro, EMjSensorValueKind::Vector3, 3},
	{ElementType::Insidesite, EMjSensorSemantic::Generic, EMjSensorValueKind::Scalar, 1},
	{ElementType::Jointactuatorfrc, EMjSensorSemantic::Generic, EMjSensorValueKind::Scalar, 1},
	{ElementType::Jointlimitfrc, EMjSensorSemantic::Generic, EMjSensorValueKind::Scalar, 1},
	{ElementType::Jointlimitpos, EMjSensorSemantic::Generic, EMjSensorValueKind::Scalar, 1},
	{ElementType::Jointlimitvel, EMjSensorSemantic::Generic, EMjSensorValueKind::Scalar, 1},
	{ElementType::Jointpos, EMjSensorSemantic::JointPos, EMjSensorValueKind::Scalar, 1},
	{ElementType::Jointvel, EMjSensorSemantic::JointVel, EMjSensorValueKind::Scalar, 1},
	{ElementType::Magnetometer, EMjSensorSemantic::Magnetometer, EMjSensorValueKind::Vector3, 3},
	{ElementType::Normal, EMjSensorSemantic::Generic, EMjSensorValueKind::Direction, 3},
	{ElementType::Rangefinder, EMjSensorSemantic::Rangefinder, EMjSensorValueKind::Scalar, -1},
	{ElementType::SensorContact, EMjSensorSemantic::Generic, EMjSensorValueKind::Scalar, -1},
	{ElementType::SensorPlugin, EMjSensorSemantic::Generic, EMjSensorValueKind::Scalar, -1},
	{ElementType::SensorUser, EMjSensorSemantic::Generic, EMjSensorValueKind::Scalar, -1},
	{ElementType::Subtreeangmom, EMjSensorSemantic::SubtreeAngMom, EMjSensorValueKind::Vector3, 3},
	{ElementType::Subtreecom, EMjSensorSemantic::SubtreeCom, EMjSensorValueKind::Position, 3},
	{ElementType::Subtreelinvel, EMjSensorSemantic::SubtreeLinVel, EMjSensorValueKind::Vector3, 3},
	{ElementType::Tactile, EMjSensorSemantic::Generic, EMjSensorValueKind::Scalar, -1},
	{ElementType::Tendonactuatorfrc, EMjSensorSemantic::Generic, EMjSensorValueKind::Scalar, 1},
	{ElementType::Tendonlimitfrc, EMjSensorSemantic::Generic, EMjSensorValueKind::Scalar, 1},
	{ElementType::Tendonlimitpos, EMjSensorSemantic::Generic, EMjSensorValueKind::Scalar, 1},
	{ElementType::Tendonlimitvel, EMjSensorSemantic::Generic, EMjSensorValueKind::Scalar, 1},
	{ElementType::Tendonpos, EMjSensorSemantic::Generic, EMjSensorValueKind::Scalar, 1},
	{ElementType::Tendonvel, EMjSensorSemantic::Generic, EMjSensorValueKind::Scalar, 1},
	{ElementType::Torque, EMjSensorSemantic::Torque, EMjSensorValueKind::Vector3, 3},
	{ElementType::Touch, EMjSensorSemantic::Touch, EMjSensorValueKind::Scalar, 1},
	{ElementType::Velocimeter, EMjSensorSemantic::Velocity, EMjSensorValueKind::Vector3, 3},
};

/** The row for `Type`, or null when the element is not a sensor. */
const FSensorKind* FindKind(ElementType Type)
{
	for (const FSensorKind& Kind : SensorKinds)
	{
		if (Kind.Type == Type)
		{
			return &Kind;
		}
	}
	return nullptr;
}

/** The row for `Node`, or null when it is not a sensor element at all. */
const FSensorKind* KindOf(const UMjNodeComponent* Node)
{
	if (Node == nullptr)
	{
		return nullptr;
	}
	ElementType Type;
	if (!urlab::spec::gen::ElementTypeOfNode(*Node, Type))
	{
		return nullptr;
	}
	return FindKind(Type);
}

/**
 * The compiled sensor slot `Node` occupies: its address and width in
 * d->sensordata, or false when there is nothing to read.
 *
 * Everything this touches is compiled state, so it is indexed straight off the
 * bound id. The spec knows the sensor exists; only the model knows where it
 * landed and how wide it is.
 */
bool ResolveSlot(const UMjNodeComponent* Node, const mjModel*& OutModel, const mjData*& OutData,
	int32& OutAdr, int32& OutDim)
{
	if (KindOf(Node) == nullptr)
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
	if (Model == nullptr || Data == nullptr)
	{
		return false;
	}
	if (Id.GetValue() < 0 || Id.GetValue() >= static_cast<int32>(Model->nsensor))
	{
		return false;
	}
	const int32 Adr = Model->sensor_adr[Id.GetValue()];
	const int32 Dim = Model->sensor_dim[Id.GetValue()];
	if (Adr < 0 || Dim <= 0 || Adr + Dim > static_cast<int32>(Model->nsensordata))
	{
		return false;
	}
	OutModel = Model;
	OutData = Data;
	OutAdr = Adr;
	OutDim = Dim;
	return true;
}

}  // namespace

bool UMjSensorRuntime::IsSensor(const UMjNodeComponent* Node)
{
	return KindOf(Node) != nullptr;
}

EMjSensorSemantic UMjSensorRuntime::GetSemantic(const UMjNodeComponent* Sensor)
{
	const FSensorKind* Kind = KindOf(Sensor);
	return Kind != nullptr ? Kind->Semantic : EMjSensorSemantic::Generic;
}

EMjSensorValueKind UMjSensorRuntime::GetValueKind(const UMjNodeComponent* Sensor)
{
	const FSensorKind* Kind = KindOf(Sensor);
	return Kind != nullptr ? Kind->ValueKind : EMjSensorValueKind::Scalar;
}

int32 UMjSensorRuntime::GetDimension(const UMjNodeComponent* Sensor)
{
	const mjModel* Model = nullptr;
	const mjData* Data = nullptr;
	int32 Adr = 0;
	int32 Dim = 0;
	return ResolveSlot(Sensor, Model, Data, Adr, Dim) ? Dim : 0;
}

TArray<float> UMjSensorRuntime::GetReading(const UMjNodeComponent* Sensor)
{
	TArray<float> Result;
	const mjModel* Model = nullptr;
	const mjData* Data = nullptr;
	int32 Adr = 0;
	int32 Dim = 0;
	if (!ResolveSlot(Sensor, Model, Data, Adr, Dim))
	{
		return Result;
	}
	Result.Reserve(Dim);
	for (int32 I = 0; I < Dim; ++I)
	{
		Result.Add(static_cast<float>(Data->sensordata[Adr + I]));
	}
	MjTransformSensorReading(Result, GetValueKind(Sensor));
	return Result;
}

float UMjSensorRuntime::GetScalarReading(const UMjNodeComponent* Sensor)
{
	const mjModel* Model = nullptr;
	const mjData* Data = nullptr;
	int32 Adr = 0;
	int32 Dim = 0;
	if (!ResolveSlot(Sensor, Model, Data, Adr, Dim))
	{
		return 0.0f;
	}
	return static_cast<float>(Data->sensordata[Adr]);
}

#else  // URLAB_MJ_GEN

bool UMjSensorRuntime::IsSensor(const UMjNodeComponent*)
{
	return false;
}

EMjSensorSemantic UMjSensorRuntime::GetSemantic(const UMjNodeComponent*)
{
	return EMjSensorSemantic::Generic;
}

EMjSensorValueKind UMjSensorRuntime::GetValueKind(const UMjNodeComponent*)
{
	return EMjSensorValueKind::Scalar;
}

int32 UMjSensorRuntime::GetDimension(const UMjNodeComponent*)
{
	return 0;
}

TArray<float> UMjSensorRuntime::GetReading(const UMjNodeComponent*)
{
	return {};
}

float UMjSensorRuntime::GetScalarReading(const UMjNodeComponent*)
{
	return 0.0f;
}

#endif  // URLAB_MJ_GEN
