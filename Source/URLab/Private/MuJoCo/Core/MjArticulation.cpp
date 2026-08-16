// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Core/MjArticulation.h"

#include "Camera/CameraComponent.h"
#include "Components/SceneComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/Blueprint.h"
#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "GameFramework/SpringArmComponent.h"
#include "InputAction.h"
#include "InputMappingContext.h"

#include "MuJoCo/Controllers/MjArticulationController.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Core/MjRenderSnapshot.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Spec/MjEffective.h"
#include "MuJoCo/Spec/MjElementIdentity.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Elements/MjActuatorRuntime.h"
#include "MuJoCo/Elements/MjBody.h"
#include "MuJoCo/Elements/MjGeom.h"
#include "MuJoCo/Elements/MjJointRuntime.h"
#include "MuJoCo/Elements/MjSensorRuntime.h"
#include "MuJoCo/Gen/Elements/Joints/MjJoint.gen.h"
#include "MuJoCo/Gen/Elements/Keyframes/MjKey.gen.h"
#include "MuJoCo/Gen/Elements/MjModel.gen.h"
#include "MuJoCo/Gen/Elements/Geometry/MjSite.gen.h"
#include "MuJoCo/Input/MjTwistController.h"
#include "MuJoCo/Utils/MjUtils.h"
#include "MuJoCo/Utils/URLabAxisConv.h"
#include "Utils/URLabLogging.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace
{
/**
 * Every spec element on `Actor`, in component order.
 *
 * Whole-actor rather than tree-walked: an element that is not attached under
 * the spec root is still an element the reader made, and the failure mode
 * of missing one is silent.
 */
TArray<UMjNodeComponent*> SpecNodes(const AActor& Actor)
{
	TArray<UMjNodeComponent*> Nodes;
	Actor.GetComponents(Nodes);
	return Nodes;
}

#if URLAB_MJ_GEN

using urlab::spec::psm::ElementType;

/**
 * The elements of `Actor` that are `Wanted`, class partials excluded.
 *
 * This is where the debug draws and the visibility toggles meet the authoring
 * tree rather than the compiled model, and a `<default>` partial is in that tree
 * without being an element. The compiler emits none for it, so a walk over bound
 * ids cannot reach one and neither may this -- or the joint axes and site
 * crosses of the model's classes get drawn at the origin beside its own.
 */
TArray<UMjNodeComponent*> NodesOfType(const AActor& Actor, ElementType Wanted)
{
	TArray<UMjNodeComponent*> Out;
	for (UMjNodeComponent* Node : SpecNodes(Actor))
	{
		ElementType Type;
		if (Node != nullptr && urlab::spec::MjElementTypeOfNode(*Node, Type) && Type == Wanted && !Node->IsClassPartial())
		{
			Out.Add(Node);
		}
	}
	return Out;
}

#endif // URLAB_MJ_GEN

/** The names an element answers to: its authored MJCF name and its own. */
void EachNameOf(const UMjNodeComponent& Node, TFunctionRef<void(const FString&)> Visit)
{
	const FString ComponentName = Node.GetName();
	if (!ComponentName.IsEmpty())
	{
		Visit(ComponentName);
	}
	if (Node.MjName.IsSet() && !Node.MjName.GetValue().IsEmpty() && Node.MjName.GetValue() != ComponentName)
	{
		Visit(Node.MjName.GetValue());
	}
}

} // namespace

AMjArticulation::AMjArticulation()
{
	PrimaryActorTick.bCanEverTick = true;

	DefaultSceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("ArticulationRoot"));
	RootComponent = DefaultSceneRoot;

	// The articulation is one MuJoCo spec and this is its root. It starts
	// empty: an <option> here would be a section of every articulation's
	// spec that nobody authored, and the scene's option is the manager's.
	Spec = CreateDefaultSubobject<UMjModel>(TEXT("Spec"));
	Spec->SetupAttachment(DefaultSceneRoot);
}

FSpecRef AMjArticulation::GetSpec() const
{
	return FSpecRef::OverActor(const_cast<AMjArticulation&>(*this));
}

FString AMjArticulation::GetCompiledPrefix() const
{
	return GetName() + TEXT("_");
}

bool AMjArticulation::ShouldTickIfViewportsOnly() const
{
	return bDrawDebugJoints || bDrawDebugCollision || bDrawDebugSites;
}

// --- Lifecycle -------------------------------------------------------------- //

void AMjArticulation::PostInitializeComponents()
{
	Super::PostInitializeComponents();

	if (FindComponentByClass<UMjTwistController>() != nullptr)
	{
		return;
	}

	UMjTwistController* TwistCtrl = NewObject<UMjTwistController>(this, TEXT("TwistController"));

	static UInputMappingContext* DefaultIMC = LoadObject<UInputMappingContext>(
		nullptr, TEXT("/UnrealRoboticsLab/Input/IMC_TwistControl.IMC_TwistControl"));
	static UInputAction* DefaultMove = LoadObject<UInputAction>(
		nullptr, TEXT("/UnrealRoboticsLab/Input/IA_TwistMove.IA_TwistMove"));
	static UInputAction* DefaultTurn = LoadObject<UInputAction>(
		nullptr, TEXT("/UnrealRoboticsLab/Input/IA_TwistTurn.IA_TwistTurn"));

	if (DefaultIMC != nullptr)
	{
		TwistCtrl->TwistMappingContext = DefaultIMC;
	}
	if (DefaultMove != nullptr)
	{
		TwistCtrl->MoveAction = DefaultMove;
	}
	if (DefaultTurn != nullptr)
	{
		TwistCtrl->TurnAction = DefaultTurn;
	}

	TwistCtrl->RegisterComponent();
}

void AMjArticulation::BeginPlay()
{
	Super::BeginPlay();
	UpdateGroup3Visibility();
}

void AMjArticulation::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	UEnhancedInputComponent* EIC = Cast<UEnhancedInputComponent>(PlayerInputComponent);
	if (EIC == nullptr)
	{
		return;
	}
	if (UMjTwistController* TwistCtrl = FindComponentByClass<UMjTwistController>())
	{
		TwistCtrl->BindInput(EIC);
	}
}

void AMjArticulation::PossessedBy(AController* NewController)
{
	Super::PossessedBy(NewController);

	APlayerController* PC = Cast<APlayerController>(NewController);
	if (PC == nullptr)
	{
		return;
	}

	UMjTwistController* TwistCtrl = FindComponentByClass<UMjTwistController>();
	if (TwistCtrl != nullptr && TwistCtrl->TwistMappingContext != nullptr)
	{
		if (ULocalPlayer* LP = PC->GetLocalPlayer())
		{
			if (UEnhancedInputLocalPlayerSubsystem* Subsystem = LP->GetSubsystem<UEnhancedInputLocalPlayerSubsystem>())
			{
				Subsystem->AddMappingContext(TwistCtrl->TwistMappingContext, 1);
			}
		}
	}

	// The camera hangs off a body rather than the actor so it follows the
	// physics rather than the placement the actor was spawned at.
	TArray<UMjBody*> Bodies = GetBodies();
	UMjBody* RootBody = Bodies.Num() > 0 ? Bodies[0] : FindComponentByClass<UMjBody>();
	if (RootBody == nullptr)
	{
		return;
	}

	USpringArmComponent* Arm = NewObject<USpringArmComponent>(this, TEXT("PossessCameraArm"));
	Arm->SetupAttachment(RootBody);
	Arm->TargetArmLength = PossessCameraDistance;
	Arm->SetRelativeRotation(FRotator(PossessCameraPitch, 0.0f, 0.0f));
	Arm->bDoCollisionTest = false;
	Arm->bUsePawnControlRotation = false;
	Arm->bEnableCameraLag = true;
	Arm->CameraLagSpeed = PossessCameraLagSpeed;
	Arm->CameraLagMaxDistance = 100.0f;
	Arm->bEnableCameraRotationLag = true;
	Arm->CameraRotationLagSpeed = PossessCameraRotationLagSpeed;
	Arm->SocketOffset = PossessCameraOffset;
	Arm->RegisterComponent();

	UCameraComponent* Cam = NewObject<UCameraComponent>(this, TEXT("PossessCamera"));
	Cam->SetupAttachment(Arm);
	Cam->RegisterComponent();

	Arm->ComponentTags.Add(TEXT("PossessCamera"));
	Cam->ComponentTags.Add(TEXT("PossessCamera"));
}

void AMjArticulation::UnPossessed()
{
	APlayerController* PC = Cast<APlayerController>(GetController());

	if (UMjTwistController* TwistCtrl = FindComponentByClass<UMjTwistController>())
	{
		TwistCtrl->ResetTwist();
		if (PC != nullptr && TwistCtrl->TwistMappingContext != nullptr)
		{
			if (ULocalPlayer* LP = PC->GetLocalPlayer())
			{
				if (UEnhancedInputLocalPlayerSubsystem* Subsystem = LP->GetSubsystem<UEnhancedInputLocalPlayerSubsystem>())
				{
					Subsystem->RemoveMappingContext(TwistCtrl->TwistMappingContext);
				}
			}
		}
	}

	TArray<UActorComponent*> ToRemove;
	for (UActorComponent* Comp : GetComponents())
	{
		if (Comp != nullptr && Comp->ComponentTags.Contains(TEXT("PossessCamera")))
		{
			ToRemove.Add(Comp);
		}
	}
	for (UActorComponent* Comp : ToRemove)
	{
		Comp->DestroyComponent();
	}

	Super::UnPossessed();
}

// --- Element index ---------------------------------------------------------- //

void AMjArticulation::IndexBoundElement(UMjNodeComponent& Node, int32 ObjType, int32 Id)
{
	FMjElementFamily& Family = ElementIndex.FindOrAdd(ObjType);
	Family.ById.Add(Id, &Node);
	EachNameOf(Node, [&Family, &Node](const FString& Name) { Family.ByName.Add(Name, &Node); });
}

void AMjArticulation::ClearElementIndex()
{
	ElementIndex.Reset();
	CachedController = nullptr;
}

void AMjArticulation::BindController(mjModel* Model, mjData* Data)
{
	CachedController = FindComponentByClass<UMjArticulationController>();
	if (CachedController == nullptr || Model == nullptr || Data == nullptr)
	{
		return;
	}

	TMap<int32, UMjNodeComponent*> Actuators;
	if (const FMjElementFamily* Family = ElementIndex.Find(mjOBJ_ACTUATOR))
	{
		for (const TPair<int32, TObjectPtr<UMjNodeComponent>>& Entry : Family->ById)
		{
			Actuators.Add(Entry.Key, Entry.Value);
		}
	}
	CachedController->Bind(Model, Data, Actuators);
}

UMjNodeComponent* AMjArticulation::GetComponentByMjId(int32 ObjType, int32 Id) const
{
	if (const FMjElementFamily* Family = ElementIndex.Find(ObjType))
	{
		if (const TObjectPtr<UMjNodeComponent>* Found = Family->ById.Find(Id))
		{
			return *Found;
		}
	}
	return nullptr;
}

UMjNodeComponent* AMjArticulation::GetComponentByName(int32 ObjType, const FString& Name) const
{
	if (const FMjElementFamily* Family = ElementIndex.Find(ObjType))
	{
		if (const TObjectPtr<UMjNodeComponent>* Found = Family->ByName.Find(Name))
		{
			return *Found;
		}
	}
	return nullptr;
}

TArray<UMjNodeComponent*> AMjArticulation::GetComponentsOfFamily(int32 ObjType) const
{
	TArray<UMjNodeComponent*> Out;
	if (const FMjElementFamily* Family = ElementIndex.Find(ObjType))
	{
		Out.Reserve(Family->ById.Num());
		for (const TPair<int32, TObjectPtr<UMjNodeComponent>>& Entry : Family->ById)
		{
			if (Entry.Value != nullptr)
			{
				Out.Add(Entry.Value);
			}
		}
	}
	return Out;
}

// --- Staged actuator control ------------------------------------------------ //

void AMjArticulation::ResetControlSlots(int32 SceneActuatorCount, TArray<int32> OwnedIds)
{
	// Reallocated rather than resized: the slots are only ever sized at a
	// compile, and a compile invalidates every id that indexed the old ones.
	ControlSlotCount = FMath::Max(0, SceneActuatorCount);
	if (ControlSlotCount == 0)
	{
		NetworkControl.Reset();
		InternalControl.Reset();
		OwnedActuatorIds.Reset();
		return;
	}

	NetworkControl = MakeUnique<std::atomic<double>[]>(ControlSlotCount);
	InternalControl = MakeUnique<std::atomic<double>[]>(ControlSlotCount);
	for (int32 i = 0; i < ControlSlotCount; ++i)
	{
		NetworkControl[i].store(0.0, std::memory_order_relaxed);
		InternalControl[i].store(0.0, std::memory_order_relaxed);
	}

	OwnedActuatorIds = MoveTemp(OwnedIds);
	OwnedActuatorIds.RemoveAll([this](int32 Id) { return Id < 0 || Id >= ControlSlotCount; });
}

void AMjArticulation::ClearControlSlots()
{
	NetworkControl.Reset();
	InternalControl.Reset();
	ControlSlotCount = 0;
	OwnedActuatorIds.Reset();
}

void AMjArticulation::StageNetworkControl(int32 ActuatorId, double Value)
{
	if (NetworkControl && ActuatorId >= 0 && ActuatorId < ControlSlotCount)
	{
		NetworkControl[ActuatorId].store(Value);
	}
}

void AMjArticulation::StageInternalControl(int32 ActuatorId, double Value)
{
	if (InternalControl && ActuatorId >= 0 && ActuatorId < ControlSlotCount)
	{
		InternalControl[ActuatorId].store(Value);
	}
}

void AMjArticulation::ClearStagedControl(int32 ActuatorId)
{
	StageNetworkControl(ActuatorId, 0.0);
	StageInternalControl(ActuatorId, 0.0);
}

double AMjArticulation::ResolveDesiredControl(int32 ActuatorId, uint8 Source) const
{
	if (ActuatorId < 0 || ActuatorId >= ControlSlotCount)
	{
		return 0.0f;
	}
	if (Source == 0)
	{
		return NetworkControl ? NetworkControl[ActuatorId].load() : 0.0;
	}
	return InternalControl ? InternalControl[ActuatorId].load() : 0.0;
}

double AMjArticulation::ResolveDesiredControl(int32 ActuatorId) const
{
	return ResolveDesiredControl(ActuatorId, ControlSource);
}

void AMjArticulation::ApplyControls(bool bSkipController)
{
	// Resolving the engine walks the level, so this spelling is game-thread only.
	// The worker uses the overload below and passes the model it already holds.
	if (UMjPhysicsEngine* Engine = AAMjManager::ResolveEngine(this))
	{
		ApplyControls(Engine->GetModel(), Engine->GetData(), bSkipController);
	}
}

void AMjArticulation::ApplyControls(mjModel* Model, mjData* Data, bool bSkipController)
{
	// The element index and the owned-id list are built at compile time on the
	// game thread with the worker joined, and are only read here. The worker
	// starts after the install completes, which is what makes them visible.
	if (Model == nullptr || Data == nullptr)
	{
		return;
	}

	if (bHoldingKeyframe)
	{
		if (bHoldViaQpos && HeldKeyframeQpos.Num() > 0)
		{
			// A free joint carries the world pose, so holding it would teleport
			// the robot back to wherever the keyframe was authored.
			for (int32 j = 0; j < Model->njnt; ++j)
			{
				const int32 JointType = Model->jnt_type[j];
				if (JointType == mjJNT_FREE)
				{
					continue;
				}
				const int32 QposAdr = Model->jnt_qposadr[j];
				const int32 DofAdr = Model->jnt_dofadr[j];
				const int32 NqPos = (JointType == mjJNT_BALL) ? 4 : 1;
				const int32 NvDof = (JointType == mjJNT_BALL) ? 3 : 1;

				for (int32 k = 0; k < NqPos && (QposAdr + k) < HeldKeyframeQpos.Num(); ++k)
				{
					Data->qpos[QposAdr + k] = static_cast<mjtNum>(HeldKeyframeQpos[QposAdr + k]);
				}
				for (int32 k = 0; k < NvDof; ++k)
				{
					Data->qvel[DofAdr + k] = 0.0;
				}
			}
		}
		else if (HeldKeyframeCtrl.Num() > 0)
		{
			const int32 Count = FMath::Min(HeldKeyframeCtrl.Num(), static_cast<int32>(Model->nu));
			for (int32 i = 0; i < Count; ++i)
			{
				Data->ctrl[i] = static_cast<mjtNum>(HeldKeyframeCtrl[i]);
			}
		}
		return;
	}

	if (!bSkipController && CachedController != nullptr && CachedController->bEnabled && CachedController->IsBound())
	{
		CachedController->ComputeAndApply(Model, Data, ControlSource);
		return;
	}

	if (bSkipController)
	{
		return;
	}

	// Only this articulation's own ids: writing every slot would push its unset
	// zeroes over the control of every other participant in the scene.
	for (const int32 Id : OwnedActuatorIds)
	{
		if (Id >= 0 && Id < Model->nu)
		{
			Data->ctrl[Id] = static_cast<mjtNum>(ResolveDesiredControl(Id, ControlSource));
		}
	}
}

// --- Runtime discovery ------------------------------------------------------ //

namespace
{
/** The MJCF names of an index family, or the component names where unnamed. */
TArray<FString> FamilyNames(const TArray<UMjNodeComponent*>& Nodes)
{
	TArray<FString> Names;
	Names.Reserve(Nodes.Num());
	for (const UMjNodeComponent* Node : Nodes)
	{
		if (Node == nullptr)
		{
			continue;
		}
		Names.Add(Node->MjName.IsSet() && !Node->MjName.GetValue().IsEmpty()
					  ? Node->MjName.GetValue()
					  : Node->GetName());
	}
	return Names;
}
} // namespace

TArray<UMjNodeComponent*> AMjArticulation::GetActuators() const
{
	return GetComponentsOfFamily(mjOBJ_ACTUATOR);
}

TArray<UMjNodeComponent*> AMjArticulation::GetJoints() const
{
	return GetComponentsOfFamily(mjOBJ_JOINT);
}

TArray<UMjNodeComponent*> AMjArticulation::GetSensors() const
{
	return GetComponentsOfFamily(mjOBJ_SENSOR);
}

TArray<UMjNodeComponent*> AMjArticulation::GetTendons() const
{
	return GetComponentsOfFamily(mjOBJ_TENDON);
}

TArray<UMjBody*> AMjArticulation::GetBodies() const
{
	TArray<UMjBody*> Out;
	for (UMjNodeComponent* Node : GetComponentsOfFamily(mjOBJ_BODY))
	{
		if (UMjBody* Body = Cast<UMjBody>(Node))
		{
			Out.Add(Body);
		}
	}
	return Out;
}

TArray<UMjGeom*> AMjArticulation::GetGeoms() const
{
	TArray<UMjGeom*> Out;
	for (UMjNodeComponent* Node : GetComponentsOfFamily(mjOBJ_GEOM))
	{
		if (UMjGeom* Geom = Cast<UMjGeom>(Node))
		{
			Out.Add(Geom);
		}
	}
	return Out;
}

TArray<FString> AMjArticulation::GetActuatorNames() const
{
	return FamilyNames(GetActuators());
}

TArray<FString> AMjArticulation::GetJointNames() const
{
	return FamilyNames(GetJoints());
}

TArray<FString> AMjArticulation::GetSensorNames() const
{
	return FamilyNames(GetSensors());
}

TArray<FString> AMjArticulation::GetTendonNames() const
{
	return FamilyNames(GetTendons());
}

TArray<FString> AMjArticulation::GetBodyNames() const
{
	return FamilyNames(GetComponentsOfFamily(mjOBJ_BODY));
}

UMjNodeComponent* AMjArticulation::GetActuator(const FString& Name) const
{
	return GetComponentByName(mjOBJ_ACTUATOR, Name);
}

UMjNodeComponent* AMjArticulation::GetJoint(const FString& Name) const
{
	return GetComponentByName(mjOBJ_JOINT, Name);
}

UMjNodeComponent* AMjArticulation::GetSensor(const FString& Name) const
{
	return GetComponentByName(mjOBJ_SENSOR, Name);
}

UMjNodeComponent* AMjArticulation::GetTendon(const FString& Name) const
{
	return GetComponentByName(mjOBJ_TENDON, Name);
}

UMjBody* AMjArticulation::GetBody(const FString& Name) const
{
	return Cast<UMjBody>(GetComponentByName(mjOBJ_BODY, Name));
}

UMjBody* AMjArticulation::GetBodyByMjId(int32 Id) const
{
	return Cast<UMjBody>(GetComponentByMjId(mjOBJ_BODY, Id));
}

UMjGeom* AMjArticulation::GetGeomByMjId(int32 Id) const
{
	return Cast<UMjGeom>(GetComponentByMjId(mjOBJ_GEOM, Id));
}

TArray<UMjNodeComponent*> AMjArticulation::GetElementsByTag(const FString& TagName) const
{
	TArray<UMjNodeComponent*> Out;
#if URLAB_MJ_GEN
	for (UMjNodeComponent* Node : SpecNodes(*this))
	{
		ElementType Type;
		if (Node == nullptr || !urlab::spec::MjElementTypeOfNode(*Node, Type))
		{
			continue;
		}
		if (TagName.Equals(urlab::spec::MjTagOf(Type), ESearchCase::IgnoreCase))
		{
			Out.Add(Node);
		}
	}
#endif
	return Out;
}

void AMjArticulation::WakeAll()
{
	for (UMjBody* Body : GetBodies())
	{
		if (Body != nullptr)
		{
			Body->Wake();
		}
	}
}

void AMjArticulation::SleepAll()
{
	for (UMjBody* Body : GetBodies())
	{
		if (Body != nullptr)
		{
			Body->PutToSleep();
		}
	}
}

// --- Keyframes -------------------------------------------------------------- //

TArray<UMjNodeComponent*> AMjArticulation::GetKeyframes() const
{
	TArray<UMjNodeComponent*> Out;
#if URLAB_MJ_GEN
	Out = NodesOfType(*this, ElementType::Key);
#endif
	return Out;
}

TArray<FString> AMjArticulation::GetKeyframeNames() const
{
	return FamilyNames(GetKeyframes());
}

bool AMjArticulation::ResetToKeyframe(const FString& KeyframeName)
{
	UMjPhysicsEngine* Engine = AAMjManager::ResolveEngine(this);
	mjModel* Model = Engine != nullptr ? Engine->GetModel() : nullptr;
	mjData* Data = Engine != nullptr ? Engine->GetData() : nullptr;
	if (Model == nullptr || Data == nullptr)
	{
		return false;
	}

	int32 KeyId = -1;
	if (KeyframeName.IsEmpty())
	{
		KeyId = 0;
	}
	else
	{
		// The compiled name carries this participant's prefix, but a caller
		// spelling the unprefixed authored name is asking the same question.
		const FString Prefixed = GetCompiledPrefix() + KeyframeName;
		KeyId = mj_name2id(Model, mjOBJ_KEY, TCHAR_TO_UTF8(*Prefixed));
		if (KeyId < 0)
		{
			KeyId = mj_name2id(Model, mjOBJ_KEY, TCHAR_TO_UTF8(*KeyframeName));
		}
	}

	if (KeyId < 0 || KeyId >= Model->nkey)
	{
		UE_LOG(LogURLab, Warning, TEXT("ResetToKeyframe: '%s' not found on '%s'"), *KeyframeName, *GetName());
		return false;
	}

	const mjtNum* KeyQpos = Model->key_qpos + KeyId * Model->nq;
	const mjtNum* KeyQvel = Model->key_qvel + KeyId * Model->nv;
	const mjtNum* KeyCtrl = Model->key_ctrl + KeyId * Model->nu;

	// Free joints hold the world pose. mj_resetDataKeyframe would set those too,
	// which throws the robot across the scene when all that was wanted is a pose.
	for (int32 j = 0; j < Model->njnt; ++j)
	{
		const int32 JointType = Model->jnt_type[j];
		if (JointType == mjJNT_FREE)
		{
			continue;
		}
		const int32 QposAdr = Model->jnt_qposadr[j];
		const int32 DofAdr = Model->jnt_dofadr[j];

		const int32 NqPos = (JointType == mjJNT_BALL) ? 4 : 1;
		for (int32 k = 0; k < NqPos; ++k)
		{
			Data->qpos[QposAdr + k] = KeyQpos[QposAdr + k];
		}
		const int32 NvDof = (JointType == mjJNT_BALL) ? 3 : 1;
		for (int32 k = 0; k < NvDof; ++k)
		{
			Data->qvel[DofAdr + k] = KeyQvel[DofAdr + k];
		}
	}

	for (int32 i = 0; i < Model->nu; ++i)
	{
		Data->ctrl[i] = KeyCtrl[i];
	}

	// Not a bare mj_forward: the accessors answer from the published snapshot, so
	// a reset nobody publishes is a reset nobody can read. ForwardSync is the one
	// place the forward pass and the publish happen as a single operation.
	Engine->ForwardSync();
	return true;
}

bool AMjArticulation::HoldKeyframe(const FString& KeyframeName)
{
	TArray<UMjNodeComponent*> Keys = GetKeyframes();
	UMjKey* Target = nullptr;

	if (KeyframeName.IsEmpty() && Keys.Num() > 0)
	{
		Target = Cast<UMjKey>(Keys[0]);
	}
	else
	{
		for (UMjNodeComponent* Node : Keys)
		{
			const bool bMatches = Node != nullptr
							   && ((Node->MjName.IsSet() && Node->MjName.GetValue() == KeyframeName) || Node->GetName() == KeyframeName);
			if (bMatches)
			{
				Target = Cast<UMjKey>(Node);
				break;
			}
		}
	}

	if (Target == nullptr)
	{
		UE_LOG(LogURLab, Warning, TEXT("HoldKeyframe: '%s' not found on '%s'"), *KeyframeName, *GetName());
		return false;
	}

	UMjPhysicsEngine* Engine = AAMjManager::ResolveEngine(this);
	if (Engine == nullptr)
	{
		return false;
	}

	// Ctrl first: holding through the actuators leaves the solver in charge of
	// how the pose is reached, where injecting qpos overrides it outright.
	if (Target->Ctrl.IsSet() && Target->Ctrl.GetValue().Num() > 0)
	{
		Engine->HoldKeyframe(/*bViaQpos=*/false, TArray<double>(), Target->Ctrl.GetValue());
		bHoldingKeyframe = true;
		return true;
	}

	if (Target->Qpos.IsSet() && Target->Qpos.GetValue().Num() > 0)
	{
		Engine->HoldKeyframe(/*bViaQpos=*/true, Target->Qpos.GetValue(), TArray<double>());
		bHoldingKeyframe = true;
		return true;
	}

	UE_LOG(LogURLab, Warning, TEXT("HoldKeyframe: '%s' has neither ctrl nor qpos"), *KeyframeName);
	return false;
}

void AMjArticulation::StopHoldKeyframe()
{
	bHoldingKeyframe = false;
	if (UMjPhysicsEngine* Engine = AAMjManager::ResolveEngine(this))
	{
		Engine->ReleaseKeyframeHold();
	}
}

// --- Convenience one-liners ------------------------------------------------- //

bool AMjArticulation::SetActuatorControl(const FString& ActuatorName, float Value)
{
	UMjNodeComponent* Actuator = GetActuator(ActuatorName);
	if (Actuator == nullptr)
	{
		return false;
	}
	UMjActuatorRuntime::SetControl(Actuator, Value);
	return true;
}

FVector2D AMjArticulation::GetActuatorRange(const FString& ActuatorName) const
{
	return UMjActuatorRuntime::GetControlRange(GetActuator(ActuatorName));
}

float AMjArticulation::GetJointAngle(const FString& JointName) const
{
	return UMjJointRuntime::GetPosition(GetJoint(JointName));
}

float AMjArticulation::GetSensorScalar(const FString& SensorName) const
{
	return UMjSensorRuntime::GetScalarReading(GetSensor(SensorName));
}

TArray<float> AMjArticulation::GetSensorReading(const FString& SensorName) const
{
	return UMjSensorRuntime::GetReading(GetSensor(SensorName));
}

// --- Presentation ----------------------------------------------------------- //

void AMjArticulation::ApplyRenderState(const FMjRenderSnapshot& Snap)
{
	for (UMjBody* Body : GetBodies())
	{
		if (Body != nullptr)
		{
			Body->ApplyRenderState(Snap);
		}
	}
}

void AMjArticulation::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (bDrawDebugCollision)
	{
		DrawDebugCollision();
	}
	if (bDrawDebugJoints)
	{
		DrawDebugJoints();
	}
	if (bDrawDebugSites)
	{
		DrawDebugSites();
	}
}

void AMjArticulation::DrawDebugCollision()
{
	UWorld* World = GetWorld();
	UMjPhysicsEngine* Engine = AAMjManager::ResolveEngine(this);
	mjModel* Model = Engine != nullptr ? Engine->GetModel() : nullptr;
	if (World == nullptr || Model == nullptr)
	{
		return;
	}

	// Poses are taken from the published snapshot in one visit and drawn after
	// it. The visitor runs under the lock the physics thread publishes behind,
	// and a robot's collision hulls are thousands of debug lines.
	struct FGeomPose
	{
		int32 Id = INDEX_NONE;
		mjtNum Pos[3] = {0.0, 0.0, 0.0};
		mjtNum Mat[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	};

	const TArray<UMjNodeComponent*> Geoms = GetComponentsOfFamily(mjOBJ_GEOM);
	TArray<FGeomPose> Poses;
	Poses.Reserve(Geoms.Num());

	Engine->WithRenderState([&Geoms, &Poses](const FMjRenderSnapshot& Snap) {
		for (const UMjNodeComponent* Geom : Geoms)
		{
			if (Geom == nullptr || !Geom->GetBoundId().IsSet())
			{
				continue;
			}
			const int32 Id = Geom->GetBoundId().GetValue();
			if (!Snap.GeomXPos.IsValidIndex(Id * 3 + 2) || !Snap.GeomXMat.IsValidIndex(Id * 9 + 8))
			{
				continue;
			}
			FGeomPose& Pose = Poses.AddDefaulted_GetRef();
			Pose.Id = Id;
			FMemory::Memcpy(Pose.Pos, &Snap.GeomXPos[Id * 3], sizeof(Pose.Pos));
			FMemory::Memcpy(Pose.Mat, &Snap.GeomXMat[Id * 9], sizeof(Pose.Mat));
		}
	});

	for (const FGeomPose& Pose : Poses)
	{
		MjUtils::DrawDebugGeom(World, Model, Pose.Id, Pose.Pos, Pose.Mat, FColor::Magenta, 100.0f);
	}
}

void AMjArticulation::DrawDebugJoints()
{
	UWorld* World = GetWorld();
	if (World == nullptr)
	{
		return;
	}

	UMjPhysicsEngine* Engine = AAMjManager::ResolveEngine(this);
	mjModel* Model = Engine != nullptr ? Engine->GetModel() : nullptr;

	TArray<UMjNodeComponent*> Joints = GetComponentsOfFamily(mjOBJ_JOINT);
	if (Joints.Num() == 0)
	{
#if URLAB_MJ_GEN
		Joints = NodesOfType(*this, ElementType::Joint);
#endif
	}

	for (UMjNodeComponent* Node : Joints)
	{
		if (Node == nullptr)
		{
			continue;
		}

		int32 MjType = mjJNT_HINGE;
		FVector Anchor = FVector::ZeroVector;
		FVector Axis = FVector::ForwardVector;
		bool bLimited = false;
		float RangeMin = 0.0f;
		float RangeMax = 0.0f;
		float CurrentPos = NAN;
		float RefPos = 0.0f;

		// The pose and the position come from the joint accessors, which read the
		// published snapshot; the shape of the joint -- its type, its range, its
		// reference -- comes from the model, which a compile fixes.
		const bool bCompiled = Model != nullptr && Node->GetBoundId().IsSet()
							&& Node->GetBoundId().GetValue() >= 0
							&& Node->GetBoundId().GetValue() < Model->njnt;

		if (bCompiled)
		{
			const int32 Id = Node->GetBoundId().GetValue();
			MjType = Model->jnt_type[Id];
			Anchor = UMjJointRuntime::GetWorldAnchor(Node);
			Axis = UMjJointRuntime::GetWorldAxis(Node);

			RangeMin = static_cast<float>(Model->jnt_range[Id * 2 + 0]);
			RangeMax = static_cast<float>(Model->jnt_range[Id * 2 + 1]);
			bLimited = RangeMin != 0.0f || RangeMax != 0.0f;

			if (MjType == mjJNT_HINGE || MjType == mjJNT_SLIDE)
			{
				CurrentPos = UMjJointRuntime::GetPosition(Node);
			}
			RefPos = static_cast<float>(Model->qpos0[Model->jnt_qposadr[Id]]);
		}
		else
		{
			// Nothing compiled, so the preview draws what the element authors.
			// Its default class is deliberately not consulted: resolving the
			// inheritance chain is the compiler's job, not the viewport's.
			UMjJoint* Joint = Cast<UMjJoint>(Node);
			if (Joint == nullptr)
			{
				continue;
			}
			switch (Joint->Type.Get(EMjJointType::hinge))
			{
				case EMjJointType::hinge:
					MjType = mjJNT_HINGE;
					break;
				case EMjJointType::slide:
					MjType = mjJNT_SLIDE;
					break;
				default:
					continue;
			}
			Anchor = Joint->GetComponentLocation();
			Axis = Joint->GetComponentTransform().TransformVectorNoScale(
				Joint->GetAxis().ToUnreal());
			const FVector2D Range = Joint->Range.Get(FVector2D::ZeroVector);
			RangeMin = static_cast<float>(Range.X);
			RangeMax = static_cast<float>(Range.Y);
			bLimited = Joint->Limited.Get(EMjTriState::auto_) == EMjTriState::true_
					|| RangeMin != 0.0f || RangeMax != 0.0f;
			RefPos = static_cast<float>(Joint->Ref.Get(0.0));
		}

		if (MjType != mjJNT_HINGE && MjType != mjJNT_SLIDE)
		{
			continue;
		}

		// MuJoCo stores a slide's travel in metres; the draw helper wants cm.
		if (MjType == mjJNT_SLIDE)
		{
			RangeMin *= 100.0f;
			RangeMax *= 100.0f;
			if (!FMath::IsNaN(CurrentPos))
			{
				CurrentPos *= 100.0f;
			}
			RefPos *= 100.0f;
		}

		MjUtils::DrawDebugJoint(World, Anchor, Axis, MjType, bLimited, RangeMin, RangeMax, CurrentPos, RefPos);
	}
}

void AMjArticulation::DrawDebugSites()
{
	UWorld* World = GetWorld();
	if (World == nullptr)
	{
		return;
	}

	UMjPhysicsEngine* Engine = AAMjManager::ResolveEngine(this);
	mjModel* Model = Engine != nullptr ? Engine->GetModel() : nullptr;

	TArray<UMjNodeComponent*> Sites = GetComponentsOfFamily(mjOBJ_SITE);
	if (Sites.Num() == 0)
	{
#if URLAB_MJ_GEN
		Sites = NodesOfType(*this, ElementType::Site);
#endif
	}

	// What to draw is settled first from the model and from what each element
	// authors, then the compiled ones take their position from the published
	// snapshot in a single visit, and only then is anything drawn: the visitor
	// holds the lock the physics thread publishes behind.
	struct FSiteDraw
	{
		FVector Pos = FVector::ZeroVector;
		float Radius = 1.0f;
		FColor Color = FColor(128, 128, 128, 200);
		int32 SnapshotId = INDEX_NONE;
		bool bPosed = true;
	};

	TArray<FSiteDraw> Draws;
	Draws.Reserve(Sites.Num());

	for (UMjNodeComponent* Node : Sites)
	{
		if (Node == nullptr)
		{
			continue;
		}

		const bool bCompiled = Model != nullptr && Node->GetBoundId().IsSet()
							&& Node->GetBoundId().GetValue() >= 0
							&& Node->GetBoundId().GetValue() < Model->nsite;

		if (bCompiled)
		{
			const int32 Id = Node->GetBoundId().GetValue();
			FSiteDraw& Draw = Draws.AddDefaulted_GetRef();
			Draw.SnapshotId = Id;
			Draw.bPosed = false;
			Draw.Radius = static_cast<float>(Model->site_size[Id * 3]) * 100.0f;
			const float* Rgba = &Model->site_rgba[Id * 4];
			Draw.Color = FColor(
				static_cast<uint8>(Rgba[0] * 255.0), static_cast<uint8>(Rgba[1] * 255.0),
				static_cast<uint8>(Rgba[2] * 255.0), 200);
		}
		else
		{
			UMjSite* Site = Cast<UMjSite>(Node);
			if (Site == nullptr)
			{
				continue;
			}
			FSiteDraw& Draw = Draws.AddDefaulted_GetRef();
			Draw.Pos = Site->GetComponentLocation();
			const TArray<double> Size = Site->Size.Get({0.005});
			Draw.Radius = static_cast<float>(Size.Num() > 0 ? Size[0] : 0.005) * 100.0f;
			Draw.Color = Site->Rgba.Get(FLinearColor(0.5f, 0.5f, 0.5f, 1.0f)).ToFColor(true);
			Draw.Color.A = 200;
		}
	}

	if (Engine != nullptr)
	{
		Engine->WithRenderState([&Draws](const FMjRenderSnapshot& Snap) {
			for (FSiteDraw& Draw : Draws)
			{
				if (Draw.SnapshotId == INDEX_NONE
					|| !Snap.SiteXPos.IsValidIndex(Draw.SnapshotId * 3 + 2))
				{
					continue;
				}
				Draw.Pos = URLabAxisConv::MjPositionToUe(&Snap.SiteXPos[Draw.SnapshotId * 3]);
				Draw.bPosed = true;
			}
		});
	}

	for (const FSiteDraw& Draw : Draws)
	{
		// A compiled site the snapshot cannot place yet would otherwise draw its
		// cross at the world origin, beside the model rather than on it.
		if (!Draw.bPosed)
		{
			continue;
		}

		const FVector Pos = Draw.Pos;
		const float Radius = FMath::Max(Draw.Radius, 0.5f);
		const float CrossSize = FMath::Max(Radius * 2.0f, 2.0f);
		DrawDebugPoint(World, Pos, 6.0f, Draw.Color, false, -1);
		DrawDebugLine(World, Pos - FVector(CrossSize, 0, 0), Pos + FVector(CrossSize, 0, 0), Draw.Color, false, -1, 0, 1.0f);
		DrawDebugLine(World, Pos - FVector(0, CrossSize, 0), Pos + FVector(0, CrossSize, 0), Draw.Color, false, -1, 0, 1.0f);
		DrawDebugLine(World, Pos - FVector(0, 0, CrossSize), Pos + FVector(0, 0, CrossSize), Draw.Color, false, -1, 0, 1.0f);
	}
}

void AMjArticulation::ToggleGroup3Visibility()
{
	bShowGroup3 = !bShowGroup3;
	UpdateGroup3Visibility();
}

void AMjArticulation::UpdateGroup3Visibility()
{
#if URLAB_MJ_GEN
	const TArray<UMjNodeComponent*> Nodes = NodesOfType(*this, ElementType::Geom);
	if (Nodes.Num() == 0)
	{
		return;
	}

	// The effective group, not the authored one. Models put `group` on a
	// <default class="collision"> and never on the geoms themselves -- both
	// cards.xml and Spot do -- so reading only what the geom authored finds
	// nothing to hide and the toggle does nothing at all.
	//
	// One context for the whole pass. It indexes every element and every class
	// to answer a single query, so building one per geom makes hiding a robot's
	// collision shapes quadratic in the size of the robot.
	const bool bResolved = urlab::spec::WithEffectiveDoc(*Nodes[0], [this, &Nodes](auto& Effective) {
		for (UMjNodeComponent* Node : Nodes)
		{
			UMjGeom* Geom = Cast<UMjGeom>(Node);
			if (Geom == nullptr)
			{
				continue;
			}
			int32 Group = Geom->Group.Get(0);
			Effective.ForEachLayer(static_cast<const UMjGeomBase&>(*Geom), [&Group](const auto& Layer) {
				if (!Layer.Group.IsSet())
				{
					return false;
				}
				Group = Layer.Group.GetValue();
				return true;
			});
			if (Group == 3)
			{
				Geom->SetGeomVisibility(bShowGroup3);
			}
		}
	});
	if (bResolved)
	{
		return;
	}

	// No spec to resolve against -- a detached actor, or a tree that has not
	// been built. The authored group is all there is to read.
	for (UMjNodeComponent* Node : Nodes)
	{
		UMjGeom* Geom = Cast<UMjGeom>(Node);
		if (Geom != nullptr && Geom->Group.Get(0) == 3)
		{
			Geom->SetGeomVisibility(bShowGroup3);
		}
	}
#endif
}

#if WITH_EDITOR

void AMjArticulation::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

	const FName PropertyName = PropertyChangedEvent.Property != nullptr
								 ? PropertyChangedEvent.Property->GetFName()
								 : NAME_None;
	if (PropertyName == GET_MEMBER_NAME_CHECKED(AMjArticulation, bShowGroup3))
	{
		UpdateGroup3Visibility();
	}
}

void AMjArticulation::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);
	UpdateGroup3Visibility();
}

void AMjArticulation::PostEditMove(bool bFinished)
{
	// Unreal rebuilds every construction-script component on every delta of a
	// drag, and a robot's spec is hundreds of them, each of which resolves
	// its pose and its shape through the default-class chain as it registers.
	// Nothing this actor's construction script does reads the transform that
	// moved, so the rebuild cannot say anything new until the drag ends -- where
	// it still runs, and where the spec picks the move up as it always did.
	//
	// The flag is put back before returning: the Blueprint editor branches on it
	// too, and it means something different there.
	UBlueprint* Blueprint = Cast<UBlueprint>(GetClass()->ClassGeneratedBy);
	const bool bDefer = !bFinished && Blueprint != nullptr && Blueprint->bRunConstructionScriptOnDrag;
	if (bDefer)
	{
		Blueprint->bRunConstructionScriptOnDrag = false;
	}

	Super::PostEditMove(bFinished);

	if (bDefer)
	{
		Blueprint->bRunConstructionScriptOnDrag = true;
	}
}

#endif // WITH_EDITOR
