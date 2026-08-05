// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// SPIKE CODE. Throwaway component used only by the TOptional UPROPERTY
// lifecycle spike tests (URLab.Spike.TOptional.*). Not part of the shipping
// plugin; delete together with MjSpikeOptionalTests.cpp.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "MjSpikeOptionalComponent.generated.h"

UENUM()
enum class EMjSpikeOptEnum : uint8
{
	Alpha,
	Beta,
	Gamma
};

UCLASS(ClassGroup = (URLabSpike), meta = (BlueprintSpawnableComponent))
class UMjSpikeOptionalComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, Category = "Spike")
	TOptional<double> OptDouble;

	UPROPERTY(EditAnywhere, Category = "Spike")
	TOptional<FVector> OptVector;

	UPROPERTY(EditAnywhere, Category = "Spike")
	TOptional<FQuat> OptQuat;

	UPROPERTY(EditAnywhere, Category = "Spike")
	TOptional<TArray<double>> OptArray;

	UPROPERTY(EditAnywhere, Category = "Spike")
	TOptional<FString> OptString;

	UPROPERTY(EditAnywhere, Category = "Spike")
	TOptional<EMjSpikeOptEnum> OptEnum;

	// Control for the details-panel comparison: a plain FQuat, so a difference
	// in row structure can be attributed to TOptional rather than to FQuat.
	UPROPERTY(EditAnywhere, Category = "Spike")
	FQuat PlainQuat = FQuat::Identity;

	// Hazard probe: BlueprintReadWrite on a TOptional. Expected to pass UHT
	// and C++ compile but produce an unusable Blueprint pin; the automation
	// test asserts what the K2 schema actually does with it.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spike")
	TOptional<int32> OptBlueprintInt;
};
