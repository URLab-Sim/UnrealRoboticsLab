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

#include "MuJoCo/Components/Control/MjFloatingBaseComponent.h"
#include "MuJoCo/Components/Bodies/MjBody.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Core/Spec/MjSpecWrapper.h"
#include "MuJoCo/Utils/MjUtils.h"
#include "Utils/URLabLogging.h"
#include "Engine/World.h"
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
} // namespace

UMjFloatingBaseComponent::UMjFloatingBaseComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;
}

void UMjFloatingBaseComponent::RegisterToSpec(FMujocoSpecWrapper& Wrapper, mjsBody* ParentBody)
{
	if (BaseBody.IsEmpty())
	{
		UE_LOG(LogURLab, Warning, TEXT("[MjFloatingBase] '%s' — no BaseBody selected; skipping."), *GetName());
		return;
	}

	// The base body has already been created in the child spec (bodies are set up
	// before helper components register). Flag it as a kinematic mocap body.
	mjsBody* Base = mjs_findBody(Wrapper.Spec, TCHAR_TO_UTF8(*BaseBody));
	if (!Base)
	{
		UE_LOG(LogURLab, Error, TEXT("[MjFloatingBase] '%s' — base body '%s' not found in spec; cannot make it floating."), *GetName(), *BaseBody);
		return;
	}

	Base->mocap = 1;
	UE_LOG(LogURLab, Log, TEXT("[MjFloatingBase] '%s' — base body '%s' set to kinematic mocap (hover height=%.1fcm)."), *GetName(), *BaseBody, HoverHeight);
}

void UMjFloatingBaseComponent::Bind(mjModel* model, mjData* data, const FString& Prefix)
{
	Super::Bind(model, data, Prefix);

	m_BaseBodyId = -1;
	if (!BaseBody.IsEmpty() && model)
		m_BaseBodyId = mj_name2id(model, mjOBJ_BODY, TCHAR_TO_UTF8(*(Prefix + BaseBody)));

	if (m_BaseBodyId < 0)
		UE_LOG(LogURLab, Warning, TEXT("[MjFloatingBase] '%s' — failed to resolve base body id (name='%s', prefix='%s')."), *GetName(), *BaseBody, *Prefix);
}

void UMjFloatingBaseComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!bDriveFromComponent || m_BaseBodyId < 0)
		return;

	UMjPhysicsEngine* Engine = GetEngine(this);
	UWorld* World = GetWorld();
	if (!Engine || !World)
		return;

	const FVector CompLoc = GetComponentLocation();

	// Cast straight down to find the floor under the base, ignoring our own actor.
	const FVector Start(CompLoc.X, CompLoc.Y, CompLoc.Z + TraceStartHeight);
	const FVector End(CompLoc.X, CompLoc.Y, CompLoc.Z + TraceStartHeight - (TraceStartHeight + TraceDistance));

	FCollisionQueryParams Params(FName(TEXT("MjFloatingBaseGroundTrace")), /*bTraceComplex=*/false);
	Params.AddIgnoredActor(GetOwner());

	FHitResult Hit;
	const bool bHit = World->LineTraceSingleByChannel(Hit, Start, End, TraceChannel, Params);
	const float TargetZ = bHit ? (Hit.ImpactPoint.Z + HoverHeight) : CompLoc.Z;

	const FVector TargetLoc(CompLoc.X, CompLoc.Y, TargetZ);
	const FQuat TargetRot = bKeepUpright
		? FRotator(0.0f, GetComponentRotation().Yaw, 0.0f).Quaternion()
		: GetComponentQuat();

	double Pos[3];
	double Quat[4];
	MjUtils::UEToMjPosition(TargetLoc, Pos);
	MjUtils::UEToMjRotation(TargetRot, Quat);
	Engine->SubmitMocapPose(m_BaseBodyId, Pos, Quat);
}

#if WITH_EDITOR
TArray<FString> UMjFloatingBaseComponent::GetBodyOptions() const
{
	return UMjComponent::GetSiblingComponentOptions(this, UMjBody::StaticClass());
}
#endif
