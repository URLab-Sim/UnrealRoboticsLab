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

#pragma once

#include "CoreMinimal.h"
#include "MuJoCo/Components/MjComponent.h"
#include "MjArmIKComponent.generated.h"

/**
 * @class UMjArmIKComponent
 * @brief Drop-in "fake IK" for any MuJoCo arm using the mocap-weld trick.
 *
 * Add this component to an articulation, pick the end-effector body from the
 * dropdown, and at compile time it injects:
 *   1. a kinematic mocap body (the IK target), and
 *   2. a weld equality constraint between that mocap body and the chosen
 *      end-effector body.
 *
 * Moving the mocap body drags the arm to follow — MuJoCo's constraint solver
 * does the inverse kinematics, no Jacobian required. By default the target is
 * driven from this component's own world transform each tick (point it where
 * you want the hand to go; eventually parent it to a VR motion controller).
 * The same mocap body also stays drivable over the bridge via `set_mocap_pose`.
 *
 * No manual body/weld authoring is required — selecting the end-effector in the
 * details panel is the entire setup.
 *
 * TIP: place this component at (or near) the end-effector's rest pose so the
 * arm doesn't lurch toward a far target on the first frame.
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent, DisplayName = "MuJoCo Arm IK (Mocap Weld)"))
class URLAB_API UMjArmIKComponent : public UMjComponent
{
	GENERATED_BODY()

public:
	UMjArmIKComponent();

	/** End-effector body the IK acts on (dropdown of sibling MuJoCo bodies). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Arm IK", meta = (GetOptions = "GetBodyOptions"))
	FString EndEffectorBody;

	/** Match full 6-DOF pose (orientation + position). If false, position only (weld torquescale = 0). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Arm IK")
	bool bMatchOrientation = true;

	/** Apply gravity compensation to the arm chain (end-effector -> base) so the
	 *  weld drags the arm without needing tuned actuators. Works on any arm. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Arm IK")
	bool bGravityCompensation = true;

	/** If true, drive the IK target from this component's own world transform each
	 *  tick. If false, leave the mocap target to be driven externally (e.g. the
	 *  `set_mocap_pose` RPC from VR / Python). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Arm IK")
	bool bDriveFromComponent = true;

	/** Weld solver reference (timeconst, dampratio). Lower timeconst = stiffer
	 *  tracking; raise it to soften the pull. Defaults to MuJoCo's (0.02, 1). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Arm IK|Advanced")
	FVector2D WeldSolref = FVector2D(0.02, 1.0);

	/** MuJoCo name of the injected mocap target body (valid after Setup). */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Arm IK")
	FString GetMocapTargetName() const { return m_MocapName; }

	/** Compiled body id of the injected mocap target, or -1 if not bound. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Arm IK")
	int32 GetMocapBodyId() const { return m_MocapBodyId; }

	// --- IMjSpecElement / UMjComponent ---
	virtual void RegisterToSpec(class FMujocoSpecWrapper& Wrapper, mjsBody* ParentBody = nullptr) override;
	virtual void Bind(mjModel* model, mjData* data, const FString& Prefix = TEXT("")) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

#if WITH_EDITOR
	UFUNCTION()
	TArray<FString> GetBodyOptions() const;
#endif

private:
	/** Sets gravcomp=1 on every body from the end-effector up to (not incl.) world. */
	void ApplyGravcompChain(class FMujocoSpecWrapper& Wrapper) const;

	/** Local (un-prefixed) name of the mocap target body created in RegisterToSpec. */
	FString m_MocapName;

	/** Spec element of the injected mocap body, used to resolve its compiled id. */
	mjsElement* m_MocapElement = nullptr;

	/** Compiled mjModel body id of the mocap target. */
	int32 m_MocapBodyId = -1;
};
