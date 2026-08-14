// The MJCF reader instantiated for the plain profile.
//
// One profile per translation unit: the reader's per-element template set is the
// heaviest instantiation cost in the library, and this rule keeps that cost per
// TU constant however many profiles exist. Everything keyed on ElementType, raw
// XML or diagnostics is in mjcf_common.cc and compiles once for all of them.

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <string>
#include <string_view>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <vector>

#include "default_classes.h"
#include "include.h"
#include "mjcf.h"
#include "mjcf_common.h"
#include "numeric.h"
#include "protospec/core.h"
#include "protospec/detail.h"
#include "protospec/model_core.h"
#include "protospec/plain_profile.h"
#include "protospec/profile.h"
#include "reflect.h"
#include "resolve.h"
#include "tinyxml2.h"
#include "types.h"
#include "xml_binding.h"

namespace ps::mjcf::io {

using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;
using xmlbind::AttrBinding;
using xmlbind::Bind;
using xmlbind::ElementBinding;
namespace sdk = ps::sdk;

}  // namespace ps::mjcf::io

#include "mjcf_reader.inc"

namespace ps::mjcf::io {

using PlainProfile = ps::sdk::plain::Plain;
using PlainFactory = ps::sdk::internal::DefaultNodeFactory<PlainProfile>;

ParseResult ParseMjcfString(const std::string& xml, const std::string& filename,
                            const ParseOptions& opts) {
  return ParseMjcfStringT<PlainProfile, PlainFactory>(xml, filename, opts);
}

ParseResult ParseMjcfFile(const std::string& path, const ParseOptions& opts) {
  return ParseMjcfFileT<PlainProfile, PlainFactory>(path, opts);
}

bool ResolverRegistryComplete(std::vector<std::string>* missing) {
  bool complete = true;
  for (std::size_t i = 0; i < xmlbind::BindingCount(); ++i) {
    const xmlbind::ElementBinding& b = xmlbind::BindingAt(i);
    auto check = [&](std::string_view name) {
      if (name.empty() ||
          common::LookupResolver(name) != common::Resolver::None) {
        return;
      }
      complete = false;
      if (missing) missing->emplace_back(name);
    };
    for (std::size_t a = 0; a < b.attr_count; ++a) check(b.attrs[a].resolver);
    for (std::size_t a = 0; a < b.input_alias_count; ++a)
      check(b.input_aliases[a].resolver);
  }
  return complete;
}

}  // namespace ps::mjcf::io
