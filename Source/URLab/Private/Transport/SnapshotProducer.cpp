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

#include "Transport/SnapshotProducer.h"
#include "Bridge/RpcDispatcher.h"
#include "Bridge/MsgpackHelpers.h"
#include "Dom/JsonObject.h"
#include "mujoco/mujoco.h"

TArray<uint8> FMjSnapshotProducer::BuildStateSnapshot(AAMjManager* Manager,
                                                      mjModel* m, mjData* d,
                                                      int64 StepIndex)
{
    if (!Manager || !m || !d) return {};

    TSharedPtr<FJsonObject> Snapshot = MakeShared<FJsonObject>();
    Snapshot->SetStringField(TEXT("op"),    TEXT("state_full"));
    Snapshot->SetNumberField(TEXT("time"),  d->time);
    Snapshot->SetNumberField(TEXT("step"),  static_cast<double>(StepIndex));
    FURLabRpcDispatcher::AppendClockFields(Snapshot, d->time);

    TSharedPtr<FJsonObject> Obs = FURLabRpcDispatcher::BuildStepObservations(
        Manager, m, d, FURLabRpcDispatcher::EObservationLevel::Standard);
    if (Obs.IsValid())   Snapshot->SetObjectField(TEXT("per_articulation"), Obs);

    TSharedPtr<FJsonObject> Entities = FURLabRpcDispatcher::BuildEntitiesBlock(Manager, m, d);
    if (Entities.IsValid()) Snapshot->SetObjectField(TEXT("entities"), Entities);

    TArray<uint8> Buf;
    FURLabMsgpackUtil::PackJsonObject(Snapshot, Buf);
    return Buf;
}
