// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// The fixture the TOptional storage tests are run against
// (URLab.Storage.TOptional.*).
//
// One component carrying a TOptional of every shape the generated elements
// store -- scalar, vector, quaternion, array, string, enum -- so the tests can
// assert on the storage itself rather than on whichever element happens to use
// it. It is deliberately NOT a UMjNodeComponent: the question is what Unreal
// does with a TOptional UPROPERTY, and inheriting the spec machinery would put
// that machinery between the test and the answer.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "MjOptionalStorageComponent.generated.h"

UENUM()
enum class EMjOptionalStorageEnum : uint8
{
	Alpha,
	Beta,
	Gamma
};

UCLASS(ClassGroup = (URLab), meta = (BlueprintSpawnableComponent))
class UMjOptionalStorageComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, Category = "Storage")
	TOptional<double> OptDouble;

	UPROPERTY(EditAnywhere, Category = "Storage")
	TOptional<FVector> OptVector;

	UPROPERTY(EditAnywhere, Category = "Storage")
	TOptional<FQuat> OptQuat;

	UPROPERTY(EditAnywhere, Category = "Storage")
	TOptional<TArray<double>> OptArray;

	UPROPERTY(EditAnywhere, Category = "Storage")
	TOptional<FString> OptString;

	UPROPERTY(EditAnywhere, Category = "Storage")
	TOptional<EMjOptionalStorageEnum> OptEnum;

	// Control for the details-panel comparison: a plain FQuat, so a difference
	// in row structure can be attributed to TOptional rather than to FQuat.
	UPROPERTY(EditAnywhere, Category = "Storage")
	FQuat PlainQuat = FQuat::Identity;

	// Hazard probe: BlueprintReadWrite on a TOptional. Expected to pass UHT
	// and C++ compile but produce an unusable Blueprint pin; the automation
	// test asserts what the K2 schema actually does with it.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Storage")
	TOptional<int32> OptBlueprintInt;
};
