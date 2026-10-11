// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include "CoreMinimal.h"
#include "MuJoCo/Spec/MjGenHooks.h"

class UMjNodeComponent;
struct FSpecRef;
struct FMjSpecDiagnostic;
struct FMjXmlAttributeOverride;
struct FMjSpecParseResult;
struct FMjDocParseOptions;
class UBlueprint;
class AActor;

#if URLAB_MJ_GEN
namespace urlab::spec::io
{
// Direct serialization of the generated UE authoring tree. The FSpecRef
// supplies the SCS/instance graph and the descriptor thunks supply fields.
FString WriteUeXml(const FSpecRef& Spec, TArray<FMjSpecDiagnostic>* OutErrors,
	const TArray<FMjXmlAttributeOverride>* Overrides = nullptr);
FString WriteUeElementXml(const FSpecRef& Spec, const UMjNodeComponent& Node,
	TArray<FMjSpecDiagnostic>* OutErrors);
#if WITH_EDITOR
FMjSpecParseResult ParseUeIntoScs(UBlueprint& Blueprint, const FString& Xml,
	const FString& Filename, const FMjDocParseOptions& Options);
#endif
FMjSpecParseResult ParseUeIntoInstance(AActor& Actor, const FString& Xml,
	const FString& Filename, const FMjDocParseOptions& Options);
}
#endif
