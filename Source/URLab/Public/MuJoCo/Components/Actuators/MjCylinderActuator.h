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
#include "MuJoCo/Components/Actuators/MjActuator.h"
#include "MjCylinderActuator.generated.h"

/**
 * @class UMjCylinderActuator
 * @brief Specific Cylinder Actuator component.
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class URLAB_API UMjCylinderActuator : public UMjActuator
{
	GENERATED_BODY()
public:
	// --- CODEGEN_PROPERTIES_START ---
	UPROPERTY(EditAnywhere, Category = "MuJoCo|Actuator", meta = (InlineEditConditionToggle))
	bool bOverride_timeconst = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta = (EditCondition = "bOverride_timeconst"))
	TArray<float> timeconst = {};

	UPROPERTY(EditAnywhere, Category = "MuJoCo|Actuator", meta = (InlineEditConditionToggle))
	bool bOverride_area = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta = (EditCondition = "bOverride_area"))
	float area = 0.0f;

	UPROPERTY(EditAnywhere, Category = "MuJoCo|Actuator", meta = (InlineEditConditionToggle))
	bool bOverride_diameter = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta = (EditCondition = "bOverride_diameter"))
	float diameter = 0.0f;

	UPROPERTY(EditAnywhere, Category = "MuJoCo|Actuator", meta = (InlineEditConditionToggle))
	bool bOverride_bias = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta = (EditCondition = "bOverride_bias"))
	float bias = 0.0f;
	// --- CODEGEN_PROPERTIES_END ---

	UMjCylinderActuator();

	virtual void ExportTo(mjsActuator* Element, mjsDefault* def = nullptr) override;

	virtual void ImportFromXml(const class FXmlNode* Node, const struct FMjCompilerSettings& CompilerSettings = FMjCompilerSettings{}) override;
};
