// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// One profile per translation unit.
//
// ProtoSpec's reader instantiates a template set per element type; that is the
// heaviest instantiation cost in the library, and the rule that each profile
// gets its own TU keeps it constant however many profiles exist. This header is
// the narrow, non-template seam the rest of the module calls across, so nothing
// else has to include the reader or the writer.

#include "CoreMinimal.h"

#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN

class UBlueprint;
class UMjNodeComponent;

namespace urlab::spec::io
{

#if WITH_EDITOR
/** Read into the ambient FMjScsScope's Blueprint. */
FMjSpecParseResult ParseIntoScs(const FString& Xml, const FString& Filename, const FMjDocParseOptions& Options);

/** Write a spec held as Blueprint templates; needs an open FMjScsScope. */
FString WriteFromScs(const UMjNodeComponent& Root, TArray<FMjSpecDiagnostic>* OutErrors);
#endif

/** Read into the ambient FMjInstanceScope's actor. */
FMjSpecParseResult ParseIntoInstance(const FString& Xml, const FString& Filename, const FMjDocParseOptions& Options);

/** Write a spec held as live components. */
FString WriteFromInstance(const UMjNodeComponent& Root, TArray<FMjSpecDiagnostic>* OutErrors);

}  // namespace urlab::spec::io

#endif  // URLAB_MJ_GEN
