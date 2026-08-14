// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// The MJCF reader and writer instantiated for the Blueprint-template profile.

#include "MjMjcfIoInternal.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "MuJoCo/Spec/MjSpecProfile.h"
#include "MuJoCo/Spec/MjNodeFactories.h"

#include "MjMjcfIoDiagnostics.h"

THIRD_PARTY_INCLUDES_START
#include <filesystem>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

#include "default_classes.h"
#include "include.h"
#include "mjcf.h"
#include "mjcf_common.h"
#include "numeric.h"
#include "protospec/core.h"
#include "protospec/detail.h"
#include "protospec/model_core.h"
#include "protospec/profile.h"
#include "reflect.h"
#include "resolve.h"
#include "tinyxml2.h"
#include "types.h"
#include "xml_binding.h"
THIRD_PARTY_INCLUDES_END

namespace ps::mjcf::io
{
using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;
using xmlbind::AttrBinding;
using xmlbind::Bind;
using xmlbind::ElementBinding;
namespace sdk = ps::sdk;
} // namespace ps::mjcf::io

THIRD_PARTY_INCLUDES_START
#include "mjcf_reader.inc"
#include "mjcf_writer.inc"
THIRD_PARTY_INCLUDES_END

#define URLAB_MJ_IO_PROFILE urlab::spec::FMjScsProfile
#define URLAB_MJ_IO_FACTORY urlab::spec::FScsNodeFactory
#define URLAB_MJ_IO_PARSE ParseIntoScs
#define URLAB_MJ_IO_WRITE WriteFromScs
#define URLAB_MJ_IO_WRITE_ELEMENT WriteElementFromScs

#include "MjMjcfIo.inl"

#undef URLAB_MJ_IO_PROFILE
#undef URLAB_MJ_IO_FACTORY
#undef URLAB_MJ_IO_PARSE
#undef URLAB_MJ_IO_WRITE
#undef URLAB_MJ_IO_WRITE_ELEMENT

#endif // URLAB_MJ_GEN && WITH_EDITOR
