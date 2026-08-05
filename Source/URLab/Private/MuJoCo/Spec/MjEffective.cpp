// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Spec/MjEffective.h"

#if URLAB_MJ_GEN

namespace urlab::spec
{

// Editor and game thread both re-derive presentation, never at the same time and
// never across a suspension point, so the open scope is a plain pointer rather
// than thread-local storage. A scope that outlived its stack would be the bug
// either way.
FMjEffectiveScope* FMjEffectiveScope::Current = nullptr;

}  // namespace urlab::spec

#endif  // URLAB_MJ_GEN
