// The MJCF writer instantiated for the plain profile. See mjcf_reader_plain.cc
// for the one-profile-per-translation-unit rule.

#include <array>
#include <cstdint>
#include <iterator>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include "keywords.h"
#include "mjcf.h"
#include "numeric.h"
#include "protospec/core.h"
#include "protospec/plain_profile.h"
#include "protospec/profile.h"
#include "types.h"
#include "xml_binding.h"

namespace ps::mjcf::io {

using xmlbind::AttrBinding;
using xmlbind::Bind;
using xmlbind::ElementBinding;
namespace sdk = ps::sdk;

}  // namespace ps::mjcf::io

#include "mjcf_writer.inc"

namespace ps::mjcf::io {

using PlainProfile = ps::sdk::plain::Plain;

std::string WriteMjcf(const Model& model, std::vector<ps::Diagnostic>* errors) {
  return WriteMjcfT<PlainProfile>(model, nullptr, errors);
}

std::string WriteMjcf(const Model& model, const AutoNames& auto_names,
                      std::vector<ps::Diagnostic>* errors) {
  return WriteMjcfT<PlainProfile>(model, &auto_names, errors);
}

}  // namespace ps::mjcf::io
