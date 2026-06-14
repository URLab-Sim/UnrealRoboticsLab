// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "Dom/JsonObject.h"

namespace URLabEditorOpHandlers
{
    /** Register all editor-op handlers with URLabOpRegistry. Called from
     *  URLabEditor's StartupModule. */
    void RegisterAll();

    /** Mirror — called from ShutdownModule. */
    void UnregisterAll();
}
