// The MJCF reader and writer instantiated for the mock profile.
//
// The whole of the one-profile-per-translation-unit rule, exercised: this file
// includes the same two .inc files the plain profile's TUs include, names a
// different profile, and gets a working reader and writer with no other change.

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <iterator>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <vector>

#include "default_classes.h"
#include "include.h"
#include "keywords.h"
#include "mjcf.h"
#include "mjcf_common.h"
#include "mock_profile.h"
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

namespace ps::mjcf::io {

using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;
using xmlbind::AttrBinding;
using xmlbind::Bind;
using xmlbind::ElementBinding;
namespace sdk = ps::sdk;

}  // namespace ps::mjcf::io

#include "mjcf_reader.inc"
#include "mjcf_writer.inc"

namespace mock {

using MockFactory = ps::sdk::internal::DefaultNodeFactory<Mock>;

ps::mjcf::io::ParseResultOf<Mock> ParseMock(const std::string& xml,
                                            const std::string& filename) {
  return ps::mjcf::io::ParseMjcfStringT<Mock, MockFactory>(xml, filename, {});
}

std::string WriteMock(const MModel& model) {
  return ps::mjcf::io::WriteMjcfT<Mock>(model, nullptr, nullptr);
}

}  // namespace mock
