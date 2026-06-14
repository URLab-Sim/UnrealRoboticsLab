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
#include "Transport/PublishTransport.h"
#include "Transport/SnapshotPublisher.h"
#include "Transport/ShmRegion.h"
#include "ShmPublishTransport.generated.h"

class AAMjManager;

/**
 * @class UURLabShmPublishTransport
 * @brief Shared-memory `state/full` publish transport.
 *
 * Plain UObject inheriting from `UURLabPublishTransport` — the manager
 * NewObjects it during PIE BeginPlay, calls `TransportInit`, and
 * registers it with `RegisterSnapshotPublisher`. Symmetric tear-down
 * via `TransportShutdown`.
 *
 * Wire layout (per slot): [u32 size][bytes payload]. Payload is the
 * msgpack-encoded snapshot built by FMjSnapshotProducer. Producer pattern
 * is the standard double-buffer + sequence fence.
 */
UCLASS()
class URLAB_API UURLabShmPublishTransport : public UURLabPublishTransport,
                                              public IMjSnapshotPublisher
{
    GENERATED_BODY()

public:
    UURLabShmPublishTransport() = default;

    /** Per-buffer slot size, default 16 KiB. State snapshots for normal
     *  scenes are well under 4 KiB; 16 KiB gives generous headroom for
     *  full-observation level + many articulations. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|SHM")
    int32 BufferStride = 16 * 1024;

    /** Optional explicit session id. If empty (default), the publisher
     *  uses the manager's `StepDispatcher` session id when available, or
     *  falls back to "live" so a single-instance install Just Works. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|SHM")
    FString SessionId;

    /** AAMjManager that owns this transport. Used to resolve the session
     *  id and to register/unregister the snapshot fan-out. Set by the
     *  manager via `SetOwningManager` before TransportInit. Body lives
     *  in the .cpp so the header only needs a forward decl of
     *  AAMjManager (TWeakObjectPtr assignment needs the full type). */
    void SetOwningManager(AAMjManager* InMgr);

    // UURLabPublishTransport contract.
    virtual bool TransportInit() override;
    virtual void TransportShutdown() override;
    virtual FString GetTransportName() const override { return TEXT("shm-snapshot"); }
    virtual void Publish(const FString& Topic,
                         const TArray<uint8>& Payload) override;

    // IMjSnapshotPublisher: route through to Publish("state/full", bytes)
    // so the manager's existing fan-out keeps working without a separate
    // code path.
    virtual void PublishSnapshot(const TArray<uint8>& Bytes) override
    {
        Publish(TEXT("state/full"), Bytes);
    }

    /** Resolved on-disk path for the state region (full path, after TransportInit). */
    FString GetStatePath() const { return ResolvedPath; }

    /** Convenience: directory holding all SHM files for this session. */
    static FString ResolveSessionDir(const FString& InSessionId);

private:
    TWeakObjectPtr<AAMjManager> OwningManager;
    FMjShmRegion StateRegion;
    FString ResolvedPath;
};
