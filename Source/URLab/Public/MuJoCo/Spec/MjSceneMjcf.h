// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// The scene as MJCF text, for the clients that are handed text.
//
// The engine does not compile through text: it builds an mjSpec and compiles
// that. This is a separate product with separate consumers -- the bridge
// handshake and the render-farm upload -- and what they need is the AUTHORED
// shape, because they reconcile against what the user wrote. `mj_saveXMLString`
// serialises the COMPILED shape instead: actuator spellings come back as
// `<general>`, attached participants come back expanded and namespaced. That is
// a different document, so this writer stays.
//
// The one thing it borrows from the compile is the naming: an element the
// document left unnamed reaches the compiled model with a generated name, and a
// client reconciling by name has to be given the same one.

#include "CoreMinimal.h"

#include "MuJoCo/Spec/MjSceneAssembly.h"

/**
 * The scene's MJCF, named as the compiled model is.
 *
 * The manager's sections, the world-body content, and one `<asset><model/>`
 * plus `<attach prefix=.../>` pair per participant. Each participant's own MJCF
 * goes into `OutParticipantXml` keyed by the VFS name the scene references it
 * under.
 */
URLAB_API FString MjWriteSceneMjcf(const FSceneAssembly& Scene, TMap<FString, FString>& OutParticipantXml,
	TArray<FMjSpecDiagnostic>* OutErrors = nullptr);
