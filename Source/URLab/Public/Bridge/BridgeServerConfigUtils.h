// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "Bridge/BridgeServerConfig.h"

namespace URLabBridgeServerConfigUtils
{
    /** Resolve the absolute path of LocalUnrealRoboticsLab.ini. */
    URLAB_API FString GetIniPath();

    /** Read the [BridgeServer] section into Out. Missing keys keep their
     *  default value from the struct. Missing file is treated as all-default. */
    URLAB_API void LoadFromIni(FURLabBridgeServerConfig& Out);

    /** Write the full [BridgeServer] section to the INI. Creates the file /
     *  parent directories if missing. */
    URLAB_API void SaveToIni(const FURLabBridgeServerConfig& In);

    /** Section name used in the INI. Exposed for tests. */
    URLAB_API extern const TCHAR* SectionName;
}
