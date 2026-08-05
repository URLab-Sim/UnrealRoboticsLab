// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// Diagnostic flattening, shared by both profile instantiation units.

#include "CoreMinimal.h"

#include "MuJoCo/Spec/MjSpecProfile.h"
#include "MuJoCo/Spec/MjSpecRef.h"

#if URLAB_MJ_GEN

THIRD_PARTY_INCLUDES_START
#include <vector>

#include "protospec/diag.h"
THIRD_PARTY_INCLUDES_END

namespace urlab::spec::io
{

inline void CollectDiagnostics(const std::vector<ps::Diagnostic>& In, TArray<FMjSpecDiagnostic>& Out)
{
	Out.Reserve(Out.Num() + static_cast<int32>(In.size()));
	for (const ps::Diagnostic& Diagnostic : In)
	{
		FMjSpecDiagnostic Entry;
		Entry.Message = gen::FMjStrPolicy::FromUtf8(Diagnostic.message);
		Entry.File = gen::FMjStrPolicy::FromUtf8(Diagnostic.loc.file);
		Entry.Line = static_cast<int32>(Diagnostic.loc.line);
		Entry.bUnsupportedElement = Diagnostic.kind == ps::Diagnostic::Kind::UnsupportedElement;
		Out.Add(MoveTemp(Entry));
	}
}

}  // namespace urlab::spec::io

#endif  // URLAB_MJ_GEN
