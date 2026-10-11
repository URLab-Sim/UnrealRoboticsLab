// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#include "MuJoCo/Core/MjModelRuntime.h"

#include "MuJoCo/Entity/MjSensorSemanticTable.h"
#include "Utils/AssetCache.h"
#include "mujoco/mujoco.h"

namespace
{
FString ObjectName(const mjModel* Model, mjtObj Type, int32 Id)
{
	const char* Name = mj_id2name(Model, Type, Id);
	return Name ? FString(UTF8_TO_TCHAR(Name)) : FString();
}

void JointWidths(int32 Type, int32& Qpos, int32& Dof)
{
	switch (Type)
	{
	case mjJNT_FREE: Qpos = 7; Dof = 6; break;
	case mjJNT_BALL: Qpos = 4; Dof = 3; break;
	default: Qpos = 1; Dof = 1; break;
	}
}

bool AllFinite(const TArray<double>& Values)
{
	for (double Value : Values)
		if (!FMath::IsFinite(Value))
			return false;
	return true;
}

bool FitsArray(mjtSize Count)
{
	return Count >= 0 && Count <= MAX_int32;
}

bool ValidModelCounts(const mjModel* Model)
{
	return FitsArray(Model->nq) && FitsArray(Model->nv) && FitsArray(Model->nu)
		&& FitsArray(Model->na) && FitsArray(Model->nsensordata)
		&& FitsArray(Model->njnt) && FitsArray(Model->nactuator)
		&& FitsArray(Model->nsensor) && FitsArray(Model->nbody) && FitsArray(Model->ncam);
}

bool IdentityMatchesModel(const FMjModelIdentity& Identity, const mjModel* Model)
{
	return Identity.Generation > 0 && Identity.ContentSha256.Len() == 64
		&& Identity.MuJoCoVersion == mj_version()
		&& Identity.Nq == Model->nq && Identity.Nv == Model->nv
		&& Identity.Nu == Model->nu && Identity.Na == Model->na
		&& Identity.NSensorData == Model->nsensordata;
}

bool ValidRange(int32 Address, int32 Width, mjtSize Total)
{
	return Address >= 0 && Width >= 0 && FitsArray(Total)
		&& static_cast<int64>(Address) + static_cast<int64>(Width) <= Total;
}

void CopyScalars(const mjtNum* Source, int32 Count, TArray<double>& Out)
{
	Out.SetNumUninitialized(Count);
	for (int32 Index = 0; Index < Count; ++Index)
		Out[Index] = static_cast<double>(Source[Index]);
}
}

bool MjModelRuntime::BuildDescriptor(const mjModel* Model, uint64 Generation,
	FMjModelDescriptor& Out, FString& OutError)
{
	if (!Model)
	{
		OutError = TEXT("no model");
		return false;
	}
	if (!ValidModelCounts(Model))
	{
		OutError = TEXT("model dimensions exceed supported array bounds");
		return false;
	}
	const mjtSize WideSize = mj_sizeModel(Model);
	if (WideSize <= 0 || WideSize > MAX_int32)
	{
		OutError = TEXT("mj_sizeModel returned an invalid size");
		return false;
	}
	const int32 Size = static_cast<int32>(WideSize);
	TArray<uint8> Mjb;
	Mjb.SetNumUninitialized(Size);
	mj_saveModel(Model, nullptr, Mjb.GetData(), Size);

	FMjModelDescriptor Candidate;
	Candidate.Identity.ContentSha256 = FURLabAssetCache::Sha256Hex(Mjb);
	Candidate.Identity.Generation = Generation;
	Candidate.Identity.MuJoCoVersion = mj_version();
	Candidate.Identity.Nq = static_cast<int32>(Model->nq);
	Candidate.Identity.Nv = static_cast<int32>(Model->nv);
	Candidate.Identity.Nu = static_cast<int32>(Model->nu);
	Candidate.Identity.Na = static_cast<int32>(Model->na);
	Candidate.Identity.NSensorData = static_cast<int32>(Model->nsensordata);
	const int32 JointCount = static_cast<int32>(Model->njnt);
	Candidate.Joints.Reserve(JointCount);
	for (int32 Id = 0; Id < JointCount; ++Id)
	{
		FMjJointDescriptor D;
		D.Id = Id;
		D.Name = ObjectName(Model, mjOBJ_JOINT, Id);
		D.Type = Model->jnt_type[Id];
		D.QposAddress = Model->jnt_qposadr[Id];
		D.DofAddress = Model->jnt_dofadr[Id];
		JointWidths(D.Type, D.QposWidth, D.DofWidth);
		if (!ValidRange(D.QposAddress, D.QposWidth, Model->nq)
			|| !ValidRange(D.DofAddress, D.DofWidth, Model->nv))
		{
			OutError = TEXT("joint address is outside model dimensions");
			return false;
		}
		D.bLimited = Model->jnt_limited[Id] != 0;
		D.RangeMin = Model->jnt_range[2 * Id];
		D.RangeMax = Model->jnt_range[2 * Id + 1];
		Candidate.Joints.Add(MoveTemp(D));
	}
	Candidate.Actuators.Reserve(static_cast<int32>(Model->nactuator));
	const int32 ActuatorCount = static_cast<int32>(Model->nactuator);
	TBitArray<> CoveredControls(false, static_cast<int32>(Model->nu));
	for (int32 Id = 0; Id < ActuatorCount; ++Id)
	{
		FMjActuatorDescriptor D;
		D.Id = Id;
		D.Name = ObjectName(Model, mjOBJ_ACTUATOR, Id);
		D.ControlAddress = Model->actuator_ctrladr[Id];
		D.ControlWidth = Model->actuator_ctrlnum[Id];
		if (!ValidRange(D.ControlAddress, D.ControlWidth, Model->nu))
		{
			OutError = TEXT("actuator control span is outside model dimensions");
			return false;
		}
		D.Controls.Reserve(D.ControlWidth);
		for (int32 Offset = 0; Offset < D.ControlWidth; ++Offset)
		{
			const int32 ControlIndex = D.ControlAddress + Offset;
			if (CoveredControls[ControlIndex])
			{
				OutError = TEXT("actuator control spans overlap");
				return false;
			}
			CoveredControls[ControlIndex] = true;
			FMjActuatorControlDescriptor& Control = D.Controls.AddDefaulted_GetRef();
			Control.bLimited = Model->actuator_ctrllimited[ControlIndex] != 0;
			Control.RangeMin = Model->actuator_ctrlrange[2 * ControlIndex];
			Control.RangeMax = Model->actuator_ctrlrange[2 * ControlIndex + 1];
		}
		Candidate.Actuators.Add(MoveTemp(D));
	}
	if (CoveredControls.Contains(false))
	{
		OutError = TEXT("actuator control spans do not cover every scalar control");
		return false;
	}
	const int32 SensorCount = static_cast<int32>(Model->nsensor);
	Candidate.Sensors.Reserve(SensorCount);
	for (int32 Id = 0; Id < SensorCount; ++Id)
	{
		FMjSensorDescriptor D;
		D.Id = Id;
		D.Name = ObjectName(Model, mjOBJ_SENSOR, Id);
		D.Address = Model->sensor_adr[Id];
		D.Dimension = Model->sensor_dim[Id];
		if (!ValidRange(D.Address, D.Dimension, Model->nsensordata))
		{
			OutError = TEXT("sensor address is outside model dimensions");
			return false;
		}
		D.Type = Model->sensor_type[Id];
		D.Semantic = MjSensorSemantics::ForSensor(Model, Id);
		Candidate.Sensors.Add(MoveTemp(D));
	}
	const int32 BodyCount = static_cast<int32>(Model->nbody);
	for (int32 Id = 0; Id < BodyCount; ++Id)
		Candidate.Bodies.Add({Id, ObjectName(Model, mjOBJ_BODY, Id)});
	const int32 CameraCount = static_cast<int32>(Model->ncam);
	for (int32 Id = 0; Id < CameraCount; ++Id)
		Candidate.Cameras.Add({Id, ObjectName(Model, mjOBJ_CAMERA, Id)});
	Out = MoveTemp(Candidate);
	OutError.Reset();
	return true;
}

bool MjModelRuntime::CaptureObservation(const mjModel* Model, const mjData* Data,
	const FMjModelIdentity& Identity, uint64 ClockEpoch, uint64 StepSequence,
	FMjModelObservation& Out)
{
	if (!Model || !Data || Model->signature != Data->signature
		|| !ValidModelCounts(Model) || !IdentityMatchesModel(Identity, Model)
		|| sizeof(mjtNum) != sizeof(double))
		return false;
	FMjModelObservation Candidate;
	Candidate.Identity = Identity;
	Candidate.ClockEpoch = ClockEpoch;
	Candidate.StepSequence = StepSequence;
	Candidate.Time = Data->time;
	CopyScalars(Data->qpos, static_cast<int32>(Model->nq), Candidate.Qpos);
	CopyScalars(Data->qvel, static_cast<int32>(Model->nv), Candidate.Qvel);
	CopyScalars(Data->ctrl, static_cast<int32>(Model->nu), Candidate.Ctrl);
	CopyScalars(Data->sensordata, static_cast<int32>(Model->nsensordata), Candidate.SensorData);
	if (!FMath::IsFinite(Candidate.Time) || !AllFinite(Candidate.Qpos)
		|| !AllFinite(Candidate.Qvel) || !AllFinite(Candidate.Ctrl)
		|| !AllFinite(Candidate.SensorData))
		return false;
	Out = MoveTemp(Candidate);
	return true;
}

EMjModelStateError MjModelRuntime::CaptureState(const mjModel* Model, const mjData* Data,
	const FMjModelIdentity& Identity, uint64 ClockEpoch, uint32 StateMask,
	FMjModelState& Out)
{
	if (!Model || !Data)
		return EMjModelStateError::NoModel;
	if (Model->signature != Data->signature)
		return EMjModelStateError::WrongData;
	if (!ValidModelCounts(Model) || !IdentityMatchesModel(Identity, Model))
		return EMjModelStateError::WrongModel;
	if (sizeof(mjtNum) != sizeof(double))
		return EMjModelStateError::WrongScalar;
	if (StateMask == 0 || (StateMask & ~static_cast<uint32>(mjSTATE_INTEGRATION)) != 0)
		return EMjModelStateError::InvalidMask;
	const int32 Count = mj_stateSize(Model, StateMask);
	if (Count < 0)
		return EMjModelStateError::InvalidMask;
	FMjModelState Candidate;
	Candidate.Identity = Identity;
	Candidate.ClockEpoch = ClockEpoch;
	Candidate.StateMask = StateMask;
	Candidate.bCompleteCheckpoint = StateMask == static_cast<uint32>(mjSTATE_INTEGRATION);
	Candidate.Values.SetNumUninitialized(Count);
	mjtNum EmptyTarget = 0;
	mjtNum* Target = Count > 0
		? reinterpret_cast<mjtNum*>(Candidate.Values.GetData()) : &EmptyTarget;
	mj_getState(Model, Data, Target, StateMask);
	if (!AllFinite(Candidate.Values))
		return EMjModelStateError::NonFinite;
	Out = MoveTemp(Candidate);
	return EMjModelStateError::None;
}

EMjModelStateError MjModelRuntime::ValidateState(const mjModel* Model,
	const FMjModelIdentity& Identity, const FMjModelState& State)
{
	if (!Model)
		return EMjModelStateError::NoModel;
	if (Identity.MuJoCoVersion != mj_version()
		|| State.Identity.MuJoCoVersion != mj_version())
		return EMjModelStateError::WrongMuJoCoVersion;
	if (!ValidModelCounts(Model) || !IdentityMatchesModel(Identity, Model))
		return EMjModelStateError::WrongModel;
	if (!Identity.Matches(State.Identity))
		return EMjModelStateError::WrongModel;
	if (State.Scalar != EMjModelStateScalar::Float64)
		return EMjModelStateError::WrongScalar;
	if (sizeof(mjtNum) != sizeof(double))
		return EMjModelStateError::WrongScalar;
	if (State.bCompleteCheckpoint !=
		(State.StateMask == static_cast<uint32>(mjSTATE_INTEGRATION)))
		return EMjModelStateError::InvalidMask;
	if (State.StateMask == 0 ||
		(State.StateMask & ~static_cast<uint32>(mjSTATE_INTEGRATION)) != 0)
		return EMjModelStateError::InvalidMask;
	const int32 Count = mj_stateSize(Model, State.StateMask);
	if (Count < 0)
		return EMjModelStateError::InvalidMask;
	if (State.Values.Num() != Count)
		return EMjModelStateError::WrongLength;
	return AllFinite(State.Values) ? EMjModelStateError::None : EMjModelStateError::NonFinite;
}

EMjModelStateError MjModelRuntime::ApplyState(const mjModel* Model, mjData* Data,
	const FMjModelIdentity& Identity, const FMjModelState& State)
{
	const EMjModelStateError Error = ValidateState(Model, Identity, State);
	if (Error != EMjModelStateError::None || !Data)
		return Error != EMjModelStateError::None ? Error : EMjModelStateError::NoModel;
	if (Model->signature != Data->signature)
		return EMjModelStateError::WrongData;
	const mjtNum EmptySource = 0;
	const mjtNum* Source = State.Values.Num() > 0
		? reinterpret_cast<const mjtNum*>(State.Values.GetData()) : &EmptySource;
	mj_setState(Model, Data, Source, State.StateMask);
	return EMjModelStateError::None;
}
