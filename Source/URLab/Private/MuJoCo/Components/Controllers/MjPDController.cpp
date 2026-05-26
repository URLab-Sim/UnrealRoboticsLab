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

#include "MuJoCo/Components/Controllers/MjPDController.h"
#include "MuJoCo/Components/Actuators/MjActuator.h"
#include "Utils/URLabLogging.h"
#include "Dom/JsonObject.h"

UMjPDController::UMjPDController()
{
}

void UMjPDController::Bind(mjModel* m, mjData* d, const TMap<int32, UMjActuator*>& ActuatorIdMap)
{
	Super::Bind(m, d, ActuatorIdMap);

	// Initialize gain arrays to defaults if not pre-configured
	int32 N = Bindings.Num();
	if (Kp.Num() != N) { Kp.SetNum(N); for (auto& v : Kp) v = DefaultKp; }
	if (Kv.Num() != N) { Kv.SetNum(N); for (auto& v : Kv) v = DefaultKv; }
	if (TorqueLimits.Num() != N) { TorqueLimits.SetNum(N); for (auto& v : TorqueLimits) v = DefaultTorqueLimit; }

	UE_LOG(LogURLab, Log, TEXT("[PDController] Bound %d actuators with Kp[0]=%.2f, Kv[0]=%.2f, TorqueLimit[0]=%.1f"),
		N, N > 0 ? Kp[0] : 0.f, N > 0 ? Kv[0] : 0.f, N > 0 ? TorqueLimits[0] : 0.f);
}

void UMjPDController::ComputeAndApply(mjModel* m, mjData* d, uint8 Source)
{
	if (!bIsBound) return;

	for (int32 i = 0; i < Bindings.Num(); ++i)
	{
		const FActuatorBinding& B = Bindings[i];

		// Get the desired position target from ZMQ or UI
		float Target = B.Component->ResolveDesiredControl(Source);

		// Clamp target to joint range (matches position actuator ctrlrange behavior)
		int32 JntId = m->actuator_trnid[B.ActuatorMjID * 2];
		if (JntId >= 0 && JntId < m->njnt && m->jnt_limited[JntId])
		{
			float Lo = (float)m->jnt_range[JntId * 2];
			float Hi = (float)m->jnt_range[JntId * 2 + 1];
			Target = FMath::Clamp(Target, Lo, Hi);
		}

		// Read live joint state
		float Pos = (float)d->qpos[B.QposAddr];
		float Vel = (float)d->qvel[B.QvelAddr];

		// PD control law: torque = Kp * (target - pos) - Kv * vel
		float kp = (i < Kp.Num()) ? Kp[i] : DefaultKp;
		float kv = (i < Kv.Num()) ? Kv[i] : DefaultKv;
		float limit = (i < TorqueLimits.Num()) ? TorqueLimits[i] : DefaultTorqueLimit;

		float Torque = kp * (Target - Pos) - kv * Vel;
		Torque = FMath::Clamp(Torque, -limit, limit);

		d->ctrl[B.ActuatorMjID] = (mjtNum)Torque;
	}
}

void UMjPDController::SetGains(const TArray<float>& NewKp, const TArray<float>& NewKv, const TArray<float>& NewTorqueLimits)
{
	// Route legacy SetGains through ApplyConfig so all entry points share one path.
	TSharedPtr<FJsonObject> Params = MakeShared<FJsonObject>();

	auto BuildPerJointFromArray = [this](const TArray<float>& Arr, const FString& Field, TSharedPtr<FJsonObject>& Out)
	{
		if (Arr.Num() == 0) return;
		TSharedPtr<FJsonObject> Map = MakeShared<FJsonObject>();
		for (int32 i = 0; i < Arr.Num() && i < Bindings.Num(); ++i)
		{
			const FString Local = GetBindingLocalJointName(i);
			Map->SetNumberField(Local, Arr[i]);
		}
		Out->SetObjectField(Field, Map);
	};

	BuildPerJointFromArray(NewKp,           TEXT("kp"),           Params);
	BuildPerJointFromArray(NewKv,           TEXT("kv"),           Params);
	BuildPerJointFromArray(NewTorqueLimits, TEXT("torque_limit"), Params);

	ApplyConfig(Params);
}

FString UMjPDController::GetBindingLocalJointName(int32 Index) const
{
	if (!Bindings.IsValidIndex(Index) || !Bindings[Index].Component) return FString();
	const FString FullName = Bindings[Index].Component->GetMjName();
	const FString OwnerName = GetOwner() ? GetOwner()->GetName() : FString();
	const FString Prefix = OwnerName + TEXT("_");
	return FullName.StartsWith(Prefix) ? FullName.Mid(Prefix.Len()) : FullName;
}

void UMjPDController::GetConfigSchema(TSharedPtr<FJsonObject>& OutSchema) const
{
	if (!OutSchema.IsValid()) OutSchema = MakeShared<FJsonObject>();

	auto MakePerJoint = [](float Min) {
		TSharedPtr<FJsonObject> O = MakeShared<FJsonObject>();
		O->SetStringField(TEXT("type"),  TEXT("per_joint"));
		O->SetStringField(TEXT("dtype"), TEXT("float32"));
		O->SetNumberField(TEXT("min"),   Min);
		return O;
	};
	auto MakeScalar = [](float Min) {
		TSharedPtr<FJsonObject> O = MakeShared<FJsonObject>();
		O->SetStringField(TEXT("type"),  TEXT("scalar"));
		O->SetStringField(TEXT("dtype"), TEXT("float32"));
		O->SetNumberField(TEXT("min"),   Min);
		return O;
	};

	OutSchema->SetObjectField(TEXT("kp"),                  MakePerJoint(0.f));
	OutSchema->SetObjectField(TEXT("kv"),                  MakePerJoint(0.f));
	OutSchema->SetObjectField(TEXT("torque_limit"),        MakePerJoint(0.f));
	OutSchema->SetObjectField(TEXT("default_kp"),          MakeScalar(0.f));
	OutSchema->SetObjectField(TEXT("default_kv"),          MakeScalar(0.f));
	OutSchema->SetObjectField(TEXT("default_torque_limit"), MakeScalar(0.f));
}

void UMjPDController::GetCurrentConfig(TSharedPtr<FJsonObject>& OutParams) const
{
	if (!OutParams.IsValid()) OutParams = MakeShared<FJsonObject>();

	TSharedPtr<FJsonObject> KpMap     = MakeShared<FJsonObject>();
	TSharedPtr<FJsonObject> KvMap     = MakeShared<FJsonObject>();
	TSharedPtr<FJsonObject> TlMap     = MakeShared<FJsonObject>();
	for (int32 i = 0; i < Bindings.Num(); ++i)
	{
		const FString Local = GetBindingLocalJointName(i);
		if (Local.IsEmpty()) continue;
		KpMap->SetNumberField(Local, i < Kp.Num() ? Kp[i] : DefaultKp);
		KvMap->SetNumberField(Local, i < Kv.Num() ? Kv[i] : DefaultKv);
		TlMap->SetNumberField(Local, i < TorqueLimits.Num() ? TorqueLimits[i] : DefaultTorqueLimit);
	}
	OutParams->SetObjectField(TEXT("kp"),                   KpMap);
	OutParams->SetObjectField(TEXT("kv"),                   KvMap);
	OutParams->SetObjectField(TEXT("torque_limit"),         TlMap);
	OutParams->SetNumberField(TEXT("default_kp"),           DefaultKp);
	OutParams->SetNumberField(TEXT("default_kv"),           DefaultKv);
	OutParams->SetNumberField(TEXT("default_torque_limit"), DefaultTorqueLimit);
}

void UMjPDController::ApplyConfig(const TSharedPtr<FJsonObject>& InParams)
{
	if (!InParams.IsValid()) return;

	double D = 0.0;
	if (InParams->TryGetNumberField(TEXT("default_kp"), D))           DefaultKp           = (float)D;
	if (InParams->TryGetNumberField(TEXT("default_kv"), D))           DefaultKv           = (float)D;
	if (InParams->TryGetNumberField(TEXT("default_torque_limit"), D)) DefaultTorqueLimit  = (float)D;

	// Initialise per-joint arrays to current size, defaulting to current
	// effective values. Missing entries keep their current value, mirroring
	// the existing {prefix}/set_gains semantics.
	const int32 N = Bindings.Num();
	if (Kp.Num() != N)           { Kp.SetNum(N);           for (auto& v : Kp)           v = DefaultKp; }
	if (Kv.Num() != N)           { Kv.SetNum(N);           for (auto& v : Kv)           v = DefaultKv; }
	if (TorqueLimits.Num() != N) { TorqueLimits.SetNum(N); for (auto& v : TorqueLimits) v = DefaultTorqueLimit; }

	// Per-joint maps. Apply in a loop; missing joints keep their current value.
	auto ApplyMap = [this, N](const TSharedPtr<FJsonObject>* MapField, TArray<float>& Target)
	{
		if (!MapField || !MapField->IsValid()) return;
		for (int32 i = 0; i < N; ++i)
		{
			const FString Local = GetBindingLocalJointName(i);
			if (Local.IsEmpty()) continue;
			double V = 0.0;
			if ((*MapField)->TryGetNumberField(Local, V))
			{
				Target[i] = (float)V;
			}
		}
	};

	const TSharedPtr<FJsonObject>* KpField = nullptr;
	const TSharedPtr<FJsonObject>* KvField = nullptr;
	const TSharedPtr<FJsonObject>* TlField = nullptr;
	InParams->TryGetObjectField(TEXT("kp"),           KpField);
	InParams->TryGetObjectField(TEXT("kv"),           KvField);
	InParams->TryGetObjectField(TEXT("torque_limit"), TlField);
	ApplyMap(KpField, Kp);
	ApplyMap(KvField, Kv);
	ApplyMap(TlField, TorqueLimits);

	bEnabled = true;
}
