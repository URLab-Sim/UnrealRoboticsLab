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

#include "MuJoCo/Components/Bodies/MjBody.h"
#include "MuJoCo/Components/Bodies/MjFrame.h"
#include "Utils/URLabLogging.h"
#include "MuJoCo/Components/Geometry/MjGeom.h"
#include "MuJoCo/Components/Physics/MjInertial.h"
#include "MuJoCo/Components/Geometry/MjSite.h"
#include "MuJoCo/Components/Joints/MjJoint.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Core/Spec/MjSpecWrapper.h"
#include "MuJoCo/Core/MjRenderSnapshot.h"
#include "MuJoCo/Utils/MjXmlUtils.h"
#include "MuJoCo/Utils/MjUtils.h"
#include "MuJoCo/Utils/MjOrientationUtils.h"
#include "XmlNode.h"
#include "PhysicsEngine/BodySetup.h"
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

UMjBody::UMjBody()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = false;

	m_BodyView = BodyView();
}

void UMjBody::BeginPlay()
{
	Super::BeginPlay();
}

void UMjBody::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!m_IsSetup || !mocap || m_BodyView.id < 0)
		return;

	UMjPhysicsEngine* Engine = GetEngine(this);
	if (!Engine)
		return;

	double MjPos[3];
	double MjQuat[4];
	MjUtils::UEToMjPosition(GetComponentLocation(), MjPos);
	MjUtils::UEToMjRotation(GetComponentQuat(), MjQuat);
	Engine->SubmitMocapPose(m_BodyView.id, MjPos, MjQuat);
}

void UMjBody::ApplyRenderState(const FMjRenderSnapshot& Snap)
{
	if (!m_IsSetup || mocap)
	{
		return;
	}

	const int32 Id = m_BodyView.id;
	if (Id < 0)
	{
		return;
	}

	const int32 PosIdx = Id * 3;
	const int32 QuatIdx = Id * 4;
	if (Snap.XPos.Num() <= PosIdx + 2 || Snap.XQuat.Num() <= QuatIdx + 3)
	{
		UE_LOG(LogURLabBind, Warning,
			TEXT("MjBody::ApplyRenderState - Body '%s' (id=%d) out of range "
				 "of snapshot (XPos=%d, XQuat=%d). Disabling updates."),
			*GetName(), Id, Snap.XPos.Num(), Snap.XQuat.Num());
		m_IsSetup = false;
		return;
	}

	const FVector MuJoCoWorldPos = MjUtils::MjToUEPosition(&Snap.XPos[PosIdx]);
	const FQuat MuJoCoWorldQuat = MjUtils::MjToUERotation(&Snap.XQuat[QuatIdx]);

	FVector CorrectedPos = MuJoCoWorldPos;

	if (bIsQuickConverted)
	{
		const FVector OffsetVector = MuJoCoWorldQuat.RotateVector(m_MeshPivotOffset);
		CorrectedPos = MuJoCoWorldPos - OffsetVector;
	}

	SetWorldLocationAndRotation(CorrectedPos, MuJoCoWorldQuat);
}

void UMjBody::ExportTo(mjsBody* Element, mjsDefault* Default)
{
	if (!Element)
		return;

	// --- CODEGEN_EXPORT_START ---
	if (bOverride_SleepPolicy)
	{
		Element->sleep = static_cast<mjtSleepPolicy>(static_cast<uint8>(SleepPolicy));
	}
	if (bOverride_childclass)
		MjSetString(Element->childclass, childclass);
	if (bOverride_mocap)
		Element->mocap = mocap ? 1 : 0;
	if (bOverride_gravcomp)
		Element->gravcomp = gravcomp;
	// --- CODEGEN_EXPORT_END ---
}

void UMjBody::Setup(USceneComponent* Parent, mjsBody* ParentBody, FMujocoSpecWrapper* Wrapper)
{
	FTransform TargetTransform;

	bool bIsAttachingToWorld = (ParentBody && mjs_getId(ParentBody->element) == 0);

	if (bIsAttachingToWorld)
	{
		TargetTransform = GetComponentTransform();
	}
	else
	{
		TargetTransform = GetRelativeTransform();
	}

	FString NameToRegister = MjName.IsEmpty() ? GetName() : MjName;
	mjsBody* BodyToAttachTo = Wrapper->CreateBody(
		NameToRegister,
		ParentBody,
		TargetTransform);
	if (BodyToAttachTo)
	{
		m_SpecElement = BodyToAttachTo->element;
		// All per-attr writes (gravcomp / mocap / sleep / childclass) are
		// codegen-owned inside ExportTo's CODEGEN_EXPORT block.
		ExportTo(BodyToAttachTo, nullptr);
	}

	m_Root = Parent;

	TArray<USceneComponent*> DirectChildren = GetAttachChildren();

	for (USceneComponent* CurrentComponent : DirectChildren)
	{
		if (UMjBody* MjBodyComp = Cast<UMjBody>(CurrentComponent))
		{
			UE_LOG(LogURLab, Verbose, TEXT("LEVEL N: Detected MjsBody: %s. Creating external articulated body."),
				*MjBodyComp->GetName());
			m_Children.Add(MjBodyComp);
			MjBodyComp->Setup(this, BodyToAttachTo, Wrapper);
			continue;
		}

		if (UMjFrame* MjFrameComp = Cast<UMjFrame>(CurrentComponent))
		{
			UE_LOG(LogURLab, Verbose, TEXT("LEVEL N: Detected MjFrame: %s. Creating coordinate frame."),
				*MjFrameComp->GetName());
			MjFrameComp->Setup(this, BodyToAttachTo, Wrapper);
			continue;
		}

		if (CurrentComponent->GetClass()->ImplementsInterface(UMjSpecElement::StaticClass()))
		{
			IMjSpecElement* SpecElem = Cast<IMjSpecElement>(CurrentComponent);
			if (SpecElem)
			{
				if (UMjGeom* MjGeomComp = Cast<UMjGeom>(CurrentComponent))
				{
					if (BodyToAttachTo && MjGeomComp->Type == EMjGeomType::Mesh)
					{
						TArray<USceneComponent*> GeomChildren;
						MjGeomComp->GetChildrenComponents(true, GeomChildren);

						for (USceneComponent* ChildOfGeom : GeomChildren)
						{
							if (UStaticMeshComponent* SMC = Cast<UStaticMeshComponent>(ChildOfGeom))
							{
								Wrapper->PrepareMeshForMuJoCo(SMC, MjGeomComp->bComplexMeshRequired);
								break;
							}
						}
					}
				}

				if (UMjComponent* MjComp = Cast<UMjComponent>(CurrentComponent))
				{
					if (!MjComp->bIsDefault)
					{
						SpecElem->RegisterToSpec(*Wrapper, BodyToAttachTo);
					}
				}
				else
				{
					SpecElem->RegisterToSpec(*Wrapper, BodyToAttachTo);
				}

				m_SpecElements.Emplace(CurrentComponent);

				if (UMjGeom* Geom = Cast<UMjGeom>(CurrentComponent))
				{
					m_Geoms.Add(Geom);
				}
				else if (UMjJoint* Joint = Cast<UMjJoint>(CurrentComponent))
				{
					m_Joints.Add(Joint);
				}
				else if (UMjSensor* Sensor = Cast<UMjSensor>(CurrentComponent))
				{
					m_Sensors.Add(Sensor);
				}
				else if (UMjActuator* Actuator = Cast<UMjActuator>(CurrentComponent))
				{
					m_Actuators.Add(Actuator);
				}
			}
		}
	}
}

void UMjBody::ImportFromXml(const FXmlNode* Node, const FMjCompilerSettings& CompilerSettings)
{
	if (!Node)
		return;

	// --- CODEGEN_IMPORT_START ---
	MjXmlUtils::ReadAttrString(Node, TEXT("name"), MjName);
	if (MjXmlUtils::ReadAttrString(Node, TEXT("childclass"), childclass))
		bOverride_childclass = true;
	MjXmlUtils::ReadAttrBool(Node, TEXT("mocap"), mocap, bOverride_mocap);
	MjXmlUtils::ReadAttrFloat(Node, TEXT("gravcomp"), gravcomp, bOverride_gravcomp);
	MjUtils::ReadVec3InMeters(Node, TEXT("pos"), Pos, bOverride_Pos);
	{ // canonicalize orientation (quat/euler/axisangle/xyaxes/zaxis)
		double TmpQuat[4] = {1.0, 0.0, 0.0, 0.0};
		if (MjOrientationUtils::OrientationToMjQuat(Node, CompilerSettings, TmpQuat))
		{
			Quat = MjUtils::MjToUERotation(TmpQuat);
			bOverride_Quat = true;
		}
	}
	{ // canonicalize body.sleep -> EMjBodySleepPolicy + bOverride_SleepPolicy
		FString S = Node->GetAttribute(TEXT("sleep"));
		S = S.ToLower();
		if (S == TEXT("never"))
		{
			SleepPolicy = EMjBodySleepPolicy::Never;
			bOverride_SleepPolicy = true;
		}
		else if (S == TEXT("allowed"))
		{
			SleepPolicy = EMjBodySleepPolicy::Allowed;
			bOverride_SleepPolicy = true;
		}
		else if (S == TEXT("init"))
		{
			SleepPolicy = EMjBodySleepPolicy::InitAsleep;
			bOverride_SleepPolicy = true;
		}
	}
	if (bOverride_Pos)
		SetRelativeLocation(Pos);
	if (bOverride_Quat)
		SetRelativeRotation(Quat);
	// --- CODEGEN_IMPORT_END ---
}

void UMjBody::Bind(mjModel* Model, mjData* Data, const FString& Prefix)
{
	Super::Bind(Model, Data, Prefix);

	if (Model && Data)
	{
		BindAndCacheView(m_BodyView, Prefix);

		const bool bBound = m_BodyView.id != -1;
		m_IsSetup = bBound;
		SetComponentTickEnabled(bBound);
	}

	TArray<USceneComponent*> AllChildren;
	GetChildrenComponents(true, AllChildren);
	for (USceneComponent* Child : AllChildren)
	{
		if (UStaticMeshComponent* SMC = Cast<UStaticMeshComponent>(Child))
		{
			UStaticMesh* mesh = SMC->GetStaticMesh();
			if (mesh)
			{
				UBodySetup* BodySetup = mesh->GetBodySetup();
				if (BodySetup)
				{
					FVector LocalCenter = BodySetup->AggGeom.CalcAABB(FTransform::Identity).GetCenter();
					m_MeshPivotOffset = LocalCenter;
					break;
				}
			}
		}
	}

	// Child binding is handled by PostSetup's flat iteration over all components.
	// Calling Bind() here as well caused each child to be bound twice.
	// for (const auto& SpecElem : m_SpecElements)
	// {
	//     if (SpecElem)
	//     {
	//         SpecElem->Bind(Model, Data, Prefix);
	//     }
	// }
}

BodyView UMjBody::GetBodyView() const
{
	return m_BodyView;
}

FVector UMjBody::GetWorldPosition() const
{
	if (m_BodyView.id < 0 || !m_BodyView.xpos)
		return FVector::ZeroVector;
	return MjUtils::MjToUEPosition(m_BodyView.xpos);
}

FQuat UMjBody::GetWorldRotation() const
{
	if (m_BodyView.id < 0 || !m_BodyView.xquat)
		return FQuat::Identity;
	return MjUtils::MjToUERotation(m_BodyView.xquat);
}

FMuJoCoSpatialVelocity UMjBody::GetSpatialVelocity() const
{
	FMuJoCoSpatialVelocity Result;
	if (m_BodyView.id < 0 || !m_BodyView.cvel)
		return Result;

	// MuJoCo cvel: [ang_x, ang_y, ang_z, lin_x, lin_y, lin_z] (MuJoCo Frame, m/s and rad/s)
	// Unreal Frame: X -> X, Y -> -Y, Z -> Z

	// Linear Velocity (m/s -> cm/s)
	Result.Linear.X = (float)m_BodyView.cvel[3] * 100.0f;
	Result.Linear.Y = -(float)m_BodyView.cvel[4] * 100.0f;
	Result.Linear.Z = (float)m_BodyView.cvel[5] * 100.0f;

	// Angular Velocity (rad/s -> deg/s)
	Result.Angular.X = FMath::RadiansToDegrees((float)m_BodyView.cvel[0]);
	Result.Angular.Y = -FMath::RadiansToDegrees((float)m_BodyView.cvel[1]);
	Result.Angular.Z = FMath::RadiansToDegrees((float)m_BodyView.cvel[2]);

	return Result;
}

void UMjBody::ApplyForce(FVector force, FVector Torque)
{
	if (m_BodyView.id < 0)
		return;
	UMjPhysicsEngine* Engine = GetEngine(this);
	if (!Engine)
		return;

	const double InvScale = 0.01; // cm -> m
	double Xfrc[6];
	Xfrc[0] = static_cast<double>(Torque.X);
	Xfrc[1] = static_cast<double>(-Torque.Y);
	Xfrc[2] = static_cast<double>(Torque.Z);
	Xfrc[3] = static_cast<double>(force.X) * InvScale;
	Xfrc[4] = static_cast<double>(-force.Y) * InvScale;
	Xfrc[5] = static_cast<double>(force.Z) * InvScale;
	Engine->SubmitWrench(m_BodyView.id, Xfrc);
}

void UMjBody::ClearForce()
{
	if (m_BodyView.id < 0)
		return;
	if (UMjPhysicsEngine* Engine = GetEngine(this))
	{
		Engine->SubmitClearForce(m_BodyView.id);
	}
}

bool UMjBody::IsAwake() const
{
	if (m_BodyView.id < 0 || !m_BodyView._d)
		return true;
	return m_BodyView._d->body_awake[m_BodyView.id] != 0;
}

void UMjBody::Wake()
{
	if (m_BodyView.id < 0)
		return;
	if (UMjPhysicsEngine* Engine = GetEngine(this))
	{
		Engine->ApplyWakeBody(m_BodyView.id);
	}
}

void UMjBody::PutToSleep()
{
	if (m_BodyView.id < 0)
		return;
	if (UMjPhysicsEngine* Engine = GetEngine(this))
	{
		Engine->ApplySleepBody(m_BodyView.id);
	}
}

void UMjBody::RegisterToSpec(FMujocoSpecWrapper& Wrapper, mjsBody* ParentBody)
{
	// MjBody is handled recursively via Setup() in the parent MjBody.
	// This interface method is provided to satisfy IMjSpecElement but is not used in the standard flow.
	// If called explicitly, we warn and attempt to delegate to Setup, though this path is unusual.
	UE_LOG(LogURLab, Warning, TEXT("MjBody::RegisterToSpec called for %s. This path is liable to double-create bodies if not careful. Prefer Setup()."), *GetName());
	if (ParentBody && !m_IsSetup)
	{
		Setup(GetAttachParent(), ParentBody, &Wrapper);
	}
}

#if WITH_EDITOR
// --- CODEGEN_EDITOR_OPTIONS_START ---
TArray<FString> UMjBody::GetChildClassOptions() const
{
	return UMjComponent::GetSiblingComponentOptions(this, UMjDefault::StaticClass(), true);
}
// --- CODEGEN_EDITOR_OPTIONS_END ---
#endif
