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

#pragma once

#include "CoreMinimal.h"

class AAMjManager;
struct mjModel_;
struct mjData_;
typedef mjModel_ mjModel;
typedef mjData_ mjData;

/**
 * @brief Build the per-step state snapshot that drives live clients.
 *
 * The bytes returned here are the same shape the step server emits for
 * direct/puppet step replies, so a live client that subscribes to a
 * snapshot publisher sees the same wire format as a stepped client. Pure
 * builder -- no transport, no I/O. AAMjManager calls this once per
 * physics step and fans the bytes out to every IMjSnapshotPublisher.
 */
class URLAB_API FMjSnapshotProducer
{
public:
    /**
     * Build a msgpack-encoded `state_full` snapshot for the current step.
     *
     * The reply shape mirrors the step server's direct/puppet replies:
     *   { op: "state_full", time, step, per_articulation, entities }
     *
     * @param Manager    Manager owning the articulation list / scene cache.
     * @param m          Compiled mjModel (read only here).
     * @param d          Live mjData (read only here).
     * @param StepIndex  Monotonic frame counter to embed in the reply.
     * @return msgpack bytes; empty if Manager / m / d are not ready.
     */
    static TArray<uint8> BuildStateSnapshot(AAMjManager* Manager,
                                            mjModel* m, mjData* d,
                                            int64 StepIndex);
};
