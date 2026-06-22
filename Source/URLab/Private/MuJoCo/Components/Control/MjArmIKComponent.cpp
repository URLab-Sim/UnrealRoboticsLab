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

#include "MuJoCo/Components/Control/MjArmIKComponent.h"
#include "MuJoCo/Components/Bodies/MjBody.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Core/Spec/MjSpecWrapper.h"
#include "MuJoCo/Utils/MjUtils.h"
#include "Utils/URLabLogging.h"
#include "EngineUtils.h"

namespace
{
UMjPhysicsEngine* GetEngine(const UObject* WorldCtx)
{
	if (AAMjManager* Manager = AAMjManager::GetManager())
	{
		if (Manager->PhysicsEngine)
			return Manager->PhysicsEngine;
	}
	if (WorldCtx)
	{
		if (UWorld* World = WorldCtx->GetWorld())
		{
			for (TActorIterator<AAMjManager> It(World); It; ++It)
			{
				if (It->PhysicsEngine)
					return It->PhysicsEngine;
			}
		}
	}
	return nullptr;
}

/** Spec name a UMjBody registers under: MjName if set, else the component name. */
FString BodySpecName(const UMjBody* Body)
{
	return Body->MjName.IsEmpty() ? Body->GetName() : Body->MjName;
}
} // namespace

UMjArmIKComponent::UMjArmIKComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;
}

void UMjArmIKComponent::RegisterToSpec(FMujocoSpecWrapper& Wrapper, mjsBody* ParentBody)
{
	// Idempotent: the articulation may register world-body children directly AND
	// via the helper pass — only build the mocap/weld once.
	if (m_MocapElement)
		return;

	if (EndEffectorBody.IsEmpty())
	{
		UE_LOG(LogURLab, Warning, TEXT("[MjArmIK] '%s' — no EndEffectorBody selected; skipping IK setup."), *GetName());
		return;
	}

	mjsBody* World = mjs_findBody(Wrapper.Spec, "world");
	if (!World)
	{
		UE_LOG(LogURLab, Error, TEXT("[MjArmIK] '%s' — child spec has no world body; cannot create mocap target."), *GetName());
		return;
	}

	// 1. Kinematic mocap body (the IK target). Mocap bodies are exempt from the
	//    inertia requirement, so a bare body is fine.
	m_MocapName = GetName() + TEXT("_ik_target");
	mjsBody* Mocap = mjs_addBody(World, nullptr);
	if (!Mocap)
	{
		UE_LOG(LogURLab, Error, TEXT("[MjArmIK] '%s' — mjs_addBody failed for mocap target."), *GetName());
		return;
	}
	mjs_setName(Mocap->element, TCHAR_TO_UTF8(*m_MocapName));
	Mocap->mocap = 1;
	m_MocapElement = Mocap->element;

	// 2. Weld the mocap target (name1) to the end-effector (name2). With an
	//    identity relpose the solver holds the end-effector coincident with the
	//    mocap body; because the mocap body is kinematic, the arm is what moves.
	mjsEquality* Eq = mjs_addEquality(Wrapper.Spec, nullptr);
	if (Eq)
	{
		const FString EqName = GetName() + TEXT("_ik_weld");
		mjs_setName(Eq->element, TCHAR_TO_UTF8(*EqName));
		Eq->type = mjEQ_WELD;
		Eq->objtype = mjOBJ_BODY;
		MjSetStringRaw(Eq->name1, m_MocapName);
		MjSetStringRaw(Eq->name2, EndEffectorBody);

		// data = [anchor(3), relpose pos(3), relpose quat(4), torquescale(1)].
		for (int i = 0; i < mjNEQDATA; ++i)
			Eq->data[i] = 0.0;
		Eq->data[6] = 1.0;                                  // relpose quat w (identity)
		Eq->data[10] = bMatchOrientation ? 1.0 : 0.0;      // torquescale (0 => position only)

		Eq->solref[0] = WeldSolref.X > 0.0 ? WeldSolref.X : 0.02;
		Eq->solref[1] = WeldSolref.Y > 0.0 ? WeldSolref.Y : 1.0;
		Eq->active = 1;
	}
	else
	{
		UE_LOG(LogURLab, Error, TEXT("[MjArmIK] '%s' — mjs_addEquality failed; arm will not track."), *GetName());
	}

	// 3. Gravity-compensate the arm chain so the weld can drag it cleanly.
	if (bGravityCompensation)
		ApplyGravcompChain(Wrapper);

	UE_LOG(LogURLab, Log, TEXT("[MjArmIK] '%s' — welded mocap '%s' to end-effector '%s' (orientation=%d, gravcomp=%d)."),
		*GetName(), *m_MocapName, *EndEffectorBody, bMatchOrientation ? 1 : 0, bGravityCompensation ? 1 : 0);
}

void UMjArmIKComponent::ApplyGravcompChain(FMujocoSpecWrapper& Wrapper) const
{
	AActor* Owner = GetOwner();
	if (!Owner)
		return;

	TArray<UMjBody*> Bodies;
	Owner->GetComponents<UMjBody>(Bodies);

	UMjBody* EE = nullptr;
	for (UMjBody* B : Bodies)
	{
		if (B && BodySpecName(B) == EndEffectorBody)
		{
			EE = B;
			break;
		}
	}
	if (!EE)
	{
		UE_LOG(LogURLab, Warning, TEXT("[MjArmIK] '%s' — could not find end-effector body '%s' for gravcomp."), *GetName(), *EndEffectorBody);
		return;
	}

	// Walk up the attachment chain (end-effector -> base) until we leave the
	// body hierarchy, flagging each MuJoCo body for gravity compensation.
	for (USceneComponent* Cur = EE; Cur; Cur = Cur->GetAttachParent())
	{
		UMjBody* B = Cast<UMjBody>(Cur);
		if (!B)
			break;
		if (mjsBody* SB = mjs_findBody(Wrapper.Spec, TCHAR_TO_UTF8(*BodySpecName(B))))
			SB->gravcomp = 1.0;
	}
}

void UMjArmIKComponent::Bind(mjModel* model, mjData* data, const FString& Prefix)
{
	Super::Bind(model, data, Prefix);

	m_MocapBodyId = -1;
	if (m_MocapElement)
		m_MocapBodyId = mjs_getId(m_MocapElement);
	if (m_MocapBodyId < 0 && !m_MocapName.IsEmpty() && model)
		m_MocapBodyId = mj_name2id(model, mjOBJ_BODY, TCHAR_TO_UTF8(*(Prefix + m_MocapName)));

	if (m_MocapBodyId < 0)
		UE_LOG(LogURLab, Warning, TEXT("[MjArmIK] '%s' — failed to resolve mocap body id (name='%s', prefix='%s')."), *GetName(), *m_MocapName, *Prefix);
}

void UMjArmIKComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!bDriveFromComponent || m_MocapBodyId < 0)
		return;

	UMjPhysicsEngine* Engine = GetEngine(this);
	if (!Engine)
		return;

	const FTransform T = GetComponentTransform();
	double Pos[3];
	double Quat[4];
	MjUtils::UEToMjPosition(T.GetLocation(), Pos);
	MjUtils::UEToMjRotation(T.GetRotation(), Quat);
	Engine->SubmitMocapPose(m_MocapBodyId, Pos, Quat);
}

#if WITH_EDITOR
TArray<FString> UMjArmIKComponent::GetBodyOptions() const
{
	return UMjComponent::GetSiblingComponentOptions(this, UMjBody::StaticClass());
}
#endif
