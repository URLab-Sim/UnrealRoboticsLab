// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"

// ============================================================================
// URLabLocaleSafeFloat
//
// FString::Printf("%f", X) respects the user's C locale and on a system
// running e.g. de_DE.UTF-8 it writes "1,5" instead of "1.5". MuJoCo's
// MJCF parser rejects comma-decimal floats; round-trip through Unreal
// on such a host shipped XML the simulator can't reload.
//
// These helpers emit a stable decimal-point representation regardless
// of locale. The codegen-emitted xml_passthrough body and the hand-
// rolled MJCF exporters route through them so the resulting XML
// parses on any host.
//
// Gating:
//   URLAB_USE_LOCALE_SAFE_FLOAT=1 routes codegen + hand-rolled emit
//   paths through these helpers. Defaults to 0 (no-op) so the existing
//   format strings stay untouched until the migration flips.
// ============================================================================

#ifndef URLAB_USE_LOCALE_SAFE_FLOAT
    #define URLAB_USE_LOCALE_SAFE_FLOAT 0
#endif

namespace URLabLocaleSafeFloat
{
    /** @brief Format ``InValue`` with ``Precision`` fractional digits
     *  using ``.`` as the decimal separator regardless of the host
     *  C locale. Returns an FString suitable for concatenation into
     *  MJCF attr strings. */
    FORCEINLINE FString Format(double InValue, int32 Precision = 6)
    {
        // FString::SanitizeFloat respects locale; build the formatter
        // ourselves: integer part + "." + fractional part. Negative
        // numbers carry their sign through the integer part.
        if (FMath::IsNaN(InValue))
        {
            return TEXT("nan");
        }
        if (!FMath::IsFinite(InValue))
        {
            return InValue < 0.0 ? TEXT("-inf") : TEXT("inf");
        }
        const bool bNegative = InValue < 0.0;
        const double Abs = FMath::Abs(InValue);
        const int64 IntPart = static_cast<int64>(Abs);
        double FracPart = Abs - static_cast<double>(IntPart);
        // Round the fractional part to ``Precision`` digits before
        // emitting so trailing 0.999999... doesn't print 1 too many.
        const double Scale = FMath::Pow(10.0, static_cast<double>(Precision));
        FracPart = FMath::RoundToDouble(FracPart * Scale) / Scale;
        const int64 FracDigits = static_cast<int64>(FracPart * Scale);
        return FString::Printf(
            TEXT("%s%lld.%0*lld"),
            bNegative ? TEXT("-") : TEXT(""),
            IntPart,
            Precision,
            FracDigits);
    }

    /** @brief Append a single attr-key=value pair using locale-safe
     *  float emission. Used by the codegen xml_passthrough body
     *  emitters when the rule sets ``locale_safe: true`` on the attr. */
    FORCEINLINE FString Attr(const TCHAR* Name, double InValue, int32 Precision = 6)
    {
        return FString::Printf(TEXT(" %s=\"%s\""), Name, *Format(InValue, Precision));
    }
} // namespace URLabLocaleSafeFloat
