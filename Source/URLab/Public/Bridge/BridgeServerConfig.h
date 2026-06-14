// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "BridgeServerConfig.generated.h"

/**
 * @struct FURLabBridgeServerConfig
 * @brief Persisted settings for the bridge server. Lives in
 *        Plugins/UnrealRoboticsLab/Config/LocalUnrealRoboticsLab.ini
 *        under [BridgeServer]. Same INI as the Python override path.
 */
USTRUCT(BlueprintType)
struct URLAB_API FURLabBridgeServerConfig
{
	GENERATED_BODY()

	/** Start the bridge server when the editor opens. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Bridge")
	bool bAutoStart = true;

	/** TCP port for the request/reply step RPC channel. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Bridge")
	int32 StepPort = 5559;

	/** TCP port for the state PUB channel. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Bridge")
	int32 StatePort = 5555;

	/** When true, EndPlay tears the server down even if it was started by
	 *  the editor subsystem. Default false: an editor-spawned server stays
	 *  up across PIE cycles. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Bridge")
	bool bStopOnPIEEnd = false;
};
