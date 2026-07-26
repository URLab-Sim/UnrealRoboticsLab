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

/** Apply per-instance overrides on top of an already INI-loaded config.
 *  Precedence, highest first: command line, environment, INI.
 *  Command line: -URLabInstanceId=, -URLabInstanceIndex=, -URLabStepPort=,
 *  -URLabStatePort=, -URLabCamBasePort=, -URLabPortBase=, -URLabPortStride=,
 *  -URLabBindAddress=. Environment: the URLAB_* equivalents. When a farm
 *  index is set and a port was not explicitly overridden, the port derives
 *  from PortBase + Index*PortStride (see DerivePorts). Pure function of the
 *  passed struct plus this process's environment and command line. */
URLAB_API void ApplyEnvAndCommandLineOverrides(FURLabBridgeServerConfig& Cfg);

/** Resolve the derived fields (step/state/camera ports, instance id) from
 *  InstanceIndex, honouring any port that an override already set explicitly.
 *  Separated from ApplyEnvAndCommandLineOverrides so the derivation is unit
 *  testable without touching process env or command line:
 *    - InstanceIndex >= 0: StepPort/StatePort/CamBasePort =
 *      PortBase + Index*PortStride + {0,1,2} for each port not flagged explicit.
 *    - CamBasePort still 0 afterwards: defaults to StepPort + 2.
 *    - Empty InstanceId with a farm index: becomes "instance_{index}". */
URLAB_API void DerivePorts(FURLabBridgeServerConfig& Cfg,
	bool bStepExplicit, bool bStateExplicit, bool bCamExplicit);

/** Build the ZMQ endpoint for the camera at CameraIndex within this
 *  instance's camera port block: tcp://{BindAddress}:{CamBasePort + CameraIndex}.
 *  Cameras allocate one port each, upward from CamBasePort, so N instances on
 *  distinct CamBasePort blocks never collide. CameraIndex is clamped to >= 0. */
URLAB_API FString BuildCameraEndpoint(const FURLabBridgeServerConfig& Cfg, int32 CameraIndex);

/** Section name used in the INI. Exposed for tests. */
URLAB_API extern const TCHAR* SectionName;
} // namespace URLabBridgeServerConfigUtils
