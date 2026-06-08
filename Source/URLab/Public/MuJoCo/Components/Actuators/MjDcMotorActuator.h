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
#include "MjDcMotorActuator.generated.h"

/**
 * @enum EMjDcMotorInput
 * @brief Control input mode for the DC motor actuator. Matches the MuJoCo
 *        upstream `dcmotorinput_map` in `src/xml/xml_native_reader.cc`.
 */
UENUM(BlueprintType)
enum class EMjDcMotorInput : uint8
{
    Voltage  = 0,
    Position = 1,
    Velocity = 2
};

/**
 * @class UMjDcMotorActuator
 * @brief Models a DC motor with optional electrical dynamics (inductance),
 *        cogging torque, thermal effects, and LuGre friction. Introduced in
 *        MuJoCo 3.7.0.
 *
 * Each array field is padded to the size that `mjs_setToDCMotor` expects:
 *   motorconst[2] / nominal[3] / saturation[3] / inductance[2]
 *   cogging[3] / controller[6] / thermal[6] / lugre[5]
 * Missing tail entries are zero by default (per upstream).
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class URLAB_API UMjDcMotorActuator : public UMjActuator
{
    GENERATED_BODY()

public:
    // --- CODEGEN_PROPERTIES_START ---
    UPROPERTY(EditAnywhere, Category = "MuJoCo|Actuator", meta=(InlineEditConditionToggle))
    bool bOverride_motorconst = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta=(EditCondition="bOverride_motorconst"))
    TArray<float> motorconst = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Actuator", meta=(InlineEditConditionToggle))
    bool bOverride_resistance = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta=(EditCondition="bOverride_resistance"))
    float resistance = 0.0f;

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Actuator", meta=(InlineEditConditionToggle))
    bool bOverride_nominal = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta=(EditCondition="bOverride_nominal"))
    TArray<float> nominal = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Actuator", meta=(InlineEditConditionToggle))
    bool bOverride_saturation = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta=(EditCondition="bOverride_saturation"))
    TArray<float> saturation = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Actuator", meta=(InlineEditConditionToggle))
    bool bOverride_inductance = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta=(EditCondition="bOverride_inductance"))
    TArray<float> inductance = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Actuator", meta=(InlineEditConditionToggle))
    bool bOverride_cogging = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta=(EditCondition="bOverride_cogging"))
    TArray<float> cogging = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Actuator", meta=(InlineEditConditionToggle))
    bool bOverride_controller = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta=(EditCondition="bOverride_controller"))
    TArray<float> controller = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Actuator", meta=(InlineEditConditionToggle))
    bool bOverride_thermal = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta=(EditCondition="bOverride_thermal"))
    TArray<float> thermal = {};

    UPROPERTY(EditAnywhere, Category = "MuJoCo|Actuator", meta=(InlineEditConditionToggle))
    bool bOverride_lugre = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Actuator", meta=(EditCondition="bOverride_lugre"))
    TArray<float> lugre = {};
    // --- CODEGEN_PROPERTIES_END ---

    // Hand-declared because the UPROPERTY type is a URLab enum (EMjDcMotorInput).
    // Codegen owns the import (XML "voltage"/"position"/"velocity" -> enum) via
    // xml_enum_attrs in codegen_rules.json. Export is via mjs_setToDCMotor's
    // input_mode param (introspected setto call), not a direct Element field.
    UPROPERTY(EditAnywhere, Category = "MuJoCo|MjActuator", meta=(InlineEditConditionToggle))
    bool bOverride_input = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjActuator", meta=(EditCondition="bOverride_input"))
    EMjDcMotorInput input = EMjDcMotorInput::Voltage;

    UMjDcMotorActuator();

    // --- DC motor parameters ---





















    virtual void ExportTo(mjsActuator* Element, mjsDefault* Default = nullptr) override;

    virtual void ImportFromXml(const class FXmlNode* Node, const struct FMjCompilerSettings& CompilerSettings = FMjCompilerSettings{}) override;
};
