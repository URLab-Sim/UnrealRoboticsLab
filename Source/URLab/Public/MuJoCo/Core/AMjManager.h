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
#include "GameFramework/Actor.h"
#include "MuJoCo/Core/MjCapabilities.h"
#include "AMjManager.generated.h"

class AMjArticulation;
class URenderCameraComponent;
class UMJModelComponent;
class UMjAppearanceStore;
class USceneComponent;

/**
 * @class AAMjManager
 * @brief Authoring scene root for compiling placed URLab content via MuJoCoSpec.
 *
 * In a game world it performs one owned compile/data handoff to the simulation
 * world subsystem and retires. It owns no runtime, worker, transport, render
 * view, model/data pointer, or runtime service registration.
 */
UCLASS()
class URLAB_API AAMjManager : public AActor
{
	GENERATED_BODY()

public:
	AAMjManager();

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|Scene")
	TObjectPtr<USceneComponent> SceneRoot;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|Scene")
	TObjectPtr<UMJModelComponent> SceneModel;

	/** Editor lookup for the authored scene root. */
	static AAMjManager* Instance;

	UFUNCTION(BlueprintPure, Category = "MuJoCo|Global", meta = (DisplayName = "Get MuJoCo Manager"))
	static AAMjManager* GetManager();

	void CollectCameras(TArray<URenderCameraComponent*>& Out) const;

	UMjAppearanceStore* GetAppearanceStore();

	// --- Capabilities (composable, orthogonal to the pose source) ---

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Capabilities")
	bool bStreamCameraFrames = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Capabilities")
	bool bAllowCameraRpc = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Capabilities")
	bool bAcceptInput = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Capabilities")
	bool bAcceptModelLoad = true;

	UFUNCTION(BlueprintPure, Category = "MuJoCo|Capabilities")
	bool HasCapability(EMjCapability Capability) const;

	// --- Remote Stepping ---

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Remote Stepping")
	int32 Seed = 0;

protected:
	FString AuthoringCompileError;

	UPROPERTY(Transient)
	TObjectPtr<UMjAppearanceStore> AppearanceStore;

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

public:
	void Compile();

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "MuJoCo|Status")
	FString GetLastCompileError() const;

	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Authoring")
	bool CompileModel();
};
