// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// Generated UE schema identity, defaults, fields and child dispatch used by
// direct XML authoring. The generated component graph is the only persistent
// document; MuJoCo's native XML parser builds the transient mjSpec.
// URLAB_MJ_GEN is disabled when the narrow ProtoSpec helpers are not staged.

#if defined(URLAB_PROTOSPEC) && URLAB_PROTOSPEC && __has_include("MuJoCo/Gen/MjDispatch.gen.h")
#define URLAB_MJ_GEN 1
#else
#define URLAB_MJ_GEN 0
#endif

#if URLAB_MJ_GEN

THIRD_PARTY_INCLUDES_START
#include "MuJoCo/Gen/MjDispatch.gen.h"
#include "MuJoCo/Gen/MjDefaults.gen.h"
#include "MuJoCo/Gen/MjVisit.gen.h"
THIRD_PARTY_INCLUDES_END

namespace urlab::spec
{
namespace gen = ::ps::ue;
namespace psm = ::ps::mjcf;
} // namespace urlab::spec

#endif // URLAB_MJ_GEN
