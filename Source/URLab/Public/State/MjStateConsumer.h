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

struct FMjStateSnapshot;

/**
 * @class IMjStateConsumer
 * @brief The transport-agnostic seam a typed consumer of the per-step state IR
 *        implements to receive the full snapshot after the collector builds it.
 *
 * Plain C++ abstract class (NOT a UE UINTERFACE) so multiple inheritance with a
 * UCLASS transport is straightforward, mirroring IMjSnapshotPublisher. Where
 * IMjSnapshotPublisher receives already-encoded msgpack bytes, a state consumer
 * receives the typed FMjStateSnapshot and does its own encoding; it is the seam
 * an out-of-core encoder (e.g. an optional message-bus module) registers against
 * instead of the manager naming the concrete transport type.
 *
 * ConsumeState runs on the physics thread inside the engine's CallbackMutex,
 * once per step, from AAMjManager::FanOutStateSnapshot. It is called in every
 * step mode regardless of the byte-fan-out pause, so a distinct consumer keeps
 * receiving state while Direct / Puppet mode suppresses the msgpack streams.
 */
class URLAB_API IMjStateConsumer
{
public:
	virtual ~IMjStateConsumer() = default;

	/** Consume one per-step state snapshot. Must be fast and non-blocking. */
	virtual void ConsumeState(const FMjStateSnapshot& Snapshot) = 0;
};
