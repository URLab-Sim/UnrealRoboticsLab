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

class UURLabRpcTransport;
class UURLabPublishTransport;
class UURLabBridgeServer;
class AAMjManager;

/**
 * @struct FMjExternalTransportProvider
 * @brief Factory hooks an optional, out-of-core transport module installs at
 *        startup so the core can create its transports without naming their
 *        concrete types.
 *
 * The baseline transports (ZMQ, SHM) are created directly by the core. An
 * additional transport module that ships as a separate, optional UE module
 * depends on the core and installs these hooks in its StartupModule; the core
 * invokes them through the abstract base pointers (UURLabRpcTransport /
 * UURLabPublishTransport) it already owns. When the optional module is absent
 * the hooks stay unbound and the core simply has no such transport.
 *
 * The factory is responsible for NewObject-ing its transport (with the supplied
 * outer) and any owner wiring (SetOwningBridge, consumer registration); the core
 * only calls TransportInit and adds the result to the appropriate owner list.
 */
DECLARE_DELEGATE_RetVal_OneParam(UURLabRpcTransport*, FMjMakeExternalRpcTransport, UURLabBridgeServer*);
DECLARE_DELEGATE_RetVal_OneParam(UURLabPublishTransport*, FMjMakeExternalPublishTransport, AAMjManager*);

struct URLAB_API FMjExternalTransportProvider
{
	/** Creates an external request/reply + control-in transport bound to the
	 *  bridge. Unbound when no external module is loaded. */
	static FMjMakeExternalRpcTransport MakeControlRpcTransport;

	/** Creates an external per-step state consumer transport owned by the
	 *  manager. Unbound when no external module is loaded. */
	static FMjMakeExternalPublishTransport MakeStatePublishTransport;

	/** True when an external module has installed the control RPC factory. */
	static bool HasControlRpcTransport();
};
