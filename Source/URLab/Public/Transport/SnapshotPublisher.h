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

/**
 * @brief Wire-agnostic transport for state-full snapshots.
 *
 * Plain C++ abstract class (NOT a UE UINTERFACE) so multiple inheritance
 * with UCLASSes is straightforward. Implementations: UURLabZmqPublishTransport
 * (PUB on tcp://0.0.0.0:5555), UURLabShmPublishTransport (SHM ring buffer).
 *
 * The publisher does NOT decide what to ship -- `FMjSnapshotProducer`
 * builds the msgpack bytes once per step (in `AAMjManager`'s PostStep
 * callback); each publisher calls `PublishSnapshot(Bytes)` to fan out.
 */
class URLAB_API IMjSnapshotPublisher
{
public:
    virtual ~IMjSnapshotPublisher() = default;

    /** Ship the snapshot. Must be safe to call from the physics PostStep
     *  callback (i.e. fast and non-blocking). */
    virtual void PublishSnapshot(const TArray<uint8>& Bytes) = 0;
};
