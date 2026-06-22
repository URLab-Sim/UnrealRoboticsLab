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
#include "Engine/EngineTypes.h"
#include "MuJoCo/Components/MjComponent.h"
#include "MjFloatingBaseComponent.generated.h"

/**
 * @class UMjFloatingBaseComponent
 * @brief Makes a robot base a kinematic "floating" body that hovers above the
 *        ground, driven by Unreal each tick.
 *
 * Add this component to an articulation and pick the base body from the
 * dropdown. At compile time the chosen body is turned into a MuJoCo mocap body
 * (kinematic). Each tick the component takes its own world XY + yaw, casts a ray
 * straight down to find the floor, and teleports the base to
 * (XY, groundZ + HoverHeight, yaw). The arm (a dynamic descendant of the base)
 * rides along on top.
 *
 * REQUIREMENTS on the base body (model topology):
 *   - it must be a direct child of the world body, and
 *   - it must have no joints of its own (mocap bodies are jointless).
 * Its child bodies may be fully dynamic, jointed links — that's the arm.
 *
 * The base body also stays drivable over the bridge via `set_mocap_pose`.
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent, DisplayName = "MuJoCo Floating Base (Mocap Hover)"))
class URLAB_API UMjFloatingBaseComponent : public UMjComponent
{
	GENERATED_BODY()

public:
	UMjFloatingBaseComponent();

	/** Robot base body to float (dropdown of sibling MuJoCo bodies). Must be a
	 *  jointless direct child of the world body. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Floating Base", meta = (GetOptions = "GetBodyOptions"))
	FString BaseBody;

	/** Desired clearance above the traced ground, in centimetres. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Floating Base", meta = (ClampMin = "0.0"))
	float HoverHeight = 50.0f;

	/** Height above the component to begin the downward ground trace, in cm. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Floating Base", meta = (ClampMin = "0.0"))
	float TraceStartHeight = 500.0f;

	/** Maximum downward trace distance below the start point, in cm. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Floating Base", meta = (ClampMin = "1.0"))
	float TraceDistance = 100000.0f;

	/** Collision channel used for the ground trace. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Floating Base")
	TEnumAsByte<ECollisionChannel> TraceChannel = ECC_WorldStatic;

	/** Keep the base upright, using only the component's yaw. If false, the base
	 *  takes the component's full orientation. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Floating Base")
	bool bKeepUpright = true;

	/** If true, drive the base from this component's own world transform each
	 *  tick. If false, leave the base mocap to be driven externally (RPC/VR). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Floating Base")
	bool bDriveFromComponent = true;

	/** Compiled body id of the floating base, or -1 if not bound. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Floating Base")
	int32 GetBaseBodyId() const { return m_BaseBodyId; }

	// --- IMjSpecElement / UMjComponent ---
	virtual void RegisterToSpec(class FMujocoSpecWrapper& Wrapper, mjsBody* ParentBody = nullptr) override;
	virtual void Bind(mjModel* model, mjData* data, const FString& Prefix = TEXT("")) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

#if WITH_EDITOR
	UFUNCTION()
	TArray<FString> GetBodyOptions() const;
#endif

private:
	/** Compiled mjModel body id of the floating base. */
	int32 m_BaseBodyId = -1;
};
