// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#pragma once

#include "CoreMinimal.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"

/**
 * @file OpHelpers.h
 *
 * Shared helpers for RPC handler bodies:
 *  - error-reply construction.
 *  - `(actor_id OR actor_name)` resolution.
 *  - vec3 / rotation parsing from request JSON.
 *
 * Inline helpers (all header-only) so any handler / op declaration can
 * pull them in without a build-graph dependency.
 */
namespace URLabOpHelpers
{
    /** Build a standard error reply. Single source of truth for the
     *  `{"op": "error", "code": ..., "message": ...}` shape. */
    inline TSharedPtr<FJsonObject> MakeError(const FString& Code,
                                             const FString& Message)
    {
        TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
        Reply->SetStringField(TEXT("op"),      TEXT("error"));
        Reply->SetStringField(TEXT("code"),    Code);
        Reply->SetStringField(TEXT("message"), Message);
        return Reply;
    }

    /** Read the target-actor key from the request envelope. Wire
     *  format: `{"target": "<key>", "target_by": "actor_id" |
     *  "actor_name"}` (default `"actor_id"` when `target_by` is
     *  absent). Returns false + populates OutError if `target` is
     *  missing. */
    inline bool ResolveActorKey(const TSharedPtr<FJsonObject>& Req,
                                FString& OutKey, bool& OutByName,
                                FString& OutError)
    {
        OutKey.Empty();
        OutByName = false;
        OutError.Empty();

        FString Target, By;
        Req->TryGetStringField(TEXT("target"),    Target);
        Req->TryGetStringField(TEXT("target_by"), By);
        if (Target.IsEmpty())
        {
            OutError = TEXT("missing 'target' field");
            return false;
        }
        OutKey = Target;
        OutByName = By.Equals(TEXT("actor_name"), ESearchCase::IgnoreCase);
        return true;
    }

    /** Read a 3-element float array under Key. Falls back to Default
     *  if missing or wrong size. Returns true if the field was present
     *  and well-formed. */
    inline bool ReadVec3(const TSharedPtr<FJsonObject>& Obj,
                         const TCHAR* Key,
                         FVector& Out,
                         const FVector& Default)
    {
        const TArray<TSharedPtr<FJsonValue>>* Arr = nullptr;
        if (!Obj->TryGetArrayField(Key, Arr) || !Arr || Arr->Num() != 3)
        {
            Out = Default;
            return false;
        }
        Out.X = (*Arr)[0]->AsNumber();
        Out.Y = (*Arr)[1]->AsNumber();
        Out.Z = (*Arr)[2]->AsNumber();
        return true;
    }

    /** Read a rotation from the request: `rotation_quat` (xyzw) preferred,
     *  fall back to `rotation_euler` (degrees). Returns true if either
     *  field was present + well-formed. Out stays at FQuat::Identity if
     *  neither is set. */
    inline bool ReadRotation(const TSharedPtr<FJsonObject>& Req, FQuat& OutQ)
    {
        OutQ = FQuat::Identity;
        const TArray<TSharedPtr<FJsonValue>>* Q = nullptr;
        if (Req->TryGetArrayField(TEXT("rotation_quat"), Q) && Q && Q->Num() == 4)
        {
            OutQ = FQuat(
                (*Q)[0]->AsNumber(),
                (*Q)[1]->AsNumber(),
                (*Q)[2]->AsNumber(),
                (*Q)[3]->AsNumber());
            return true;
        }
        FVector Euler;
        if (ReadVec3(Req, TEXT("rotation_euler"), Euler, FVector::ZeroVector))
        {
            OutQ = FRotator(Euler.Y, Euler.Z, Euler.X).Quaternion();
            return true;
        }
        return false;
    }
}
