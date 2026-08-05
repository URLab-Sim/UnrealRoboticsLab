// ProtoSpec MJCF IO: the public read/write surface.
//
// ParseMjcf turns an MJCF document (tinyxml2 under the hood) into a ps::Model;
// WriteMjcf serializes a Model back to a canonical MJCF string. Every MJCF
// element family ProtoSpec models as first-class data is understood -- the
// Model-level blocks, the body tree, defaults, assets, the contact/equality/
// tendon/actuator/sensor sections, custom/keyframe/extension, and the
// deformable/macro pass-throughs (the full IsSupported set in mjcf_reader.cc). A
// well-formed element outside those families is reported as a structured
// "unsupported element" diagnostic (ps::Diagnostic::Kind::UnsupportedElement)
// that a harness can use to skip a file, kept distinct from malformed-input.
//
// This is the one module that links tinyxml2; the core ps library stays
// dependency-free.
#ifndef PROTOSPEC_IO_MJCF_H
#define PROTOSPEC_IO_MJCF_H

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "protospec/core.h"
#include "protospec/diag.h"
#include "protospec/plain_profile.h"
#include "protospec/profile.h"
#include "types.h"

namespace ps::mjcf::io {

// Serial-keyed auto-name overrides. An element whose creation serial is a key
// here is emitted with the mapped `name="..."` attribute even if its own name
// field is empty; the tree is never mutated. The compile bridge uses this to
// give unnamed elements deterministic reserved names for binding (DR-10), so
// the serialization-time naming lives here (in the serializer) with no tree
// edit. Elements already carrying an authored name are unaffected (the bridge
// only enters unnamed elements).
using AutoNames = std::unordered_map<std::uint64_t, std::string>;

// Reader diagnostics are the shared ps::Diagnostic (protospec/diag.h): source
// "parse", kind MalformedInput for syntax/unknown/bad-value errors,
// UnsupportedElement for a well-formed element outside the supported families.

// True when at least one error is an unsupported-element report (and there are
// no malformed-input errors). Shared by every profile's ParseResult.
bool ErrorsUnsupportedOnly(const std::vector<ps::Diagnostic>& errors);

// The outcome of a parse: a document on success, else a list of diagnostics. NaN
// values raise non-fatal warnings (MuJoCo mju_warning parity) that do not fail
// the parse. Generic over the emission profile, because the owning handle for a
// document root is the profile's, not std::unique_ptr's.
template <class P>
struct ParseResultOf {
  ps::sdk::OwnerOf<P, ps::sdk::DocOf<P>> model;
  std::vector<ps::Diagnostic> errors;
  std::vector<ps::Diagnostic> warnings;

  bool ok() const { return errors.empty() && model != nullptr; }

  // True when at least one error is an unsupported-element report (and there are
  // no malformed-input errors): the document is well-formed but uses elements
  // outside the supported families. The harness skips such files.
  bool unsupported_only() const { return ErrorsUnsupportedOnly(errors); }
};

using ParseResult = ParseResultOf<ps::sdk::plain::Plain>;

// Parse-time options. The reader opens untrusted MJCF from disk inside a GUI
// host, so <include> resolution is a security boundary: by default an <include>
// whose resolved path escapes the root model's directory tree is rejected with a
// diagnostic (an exfiltration-shaped bug otherwise). Set allow_external_includes
// for trusted workflows that legitimately pull files from outside that tree.
struct ParseOptions {
  bool allow_external_includes = false;
};

// Parse MJCF from an in-memory string (filename is used only for diagnostics).
ParseResult ParseMjcfString(const std::string& xml,
                            const std::string& filename = "<string>",
                            const ParseOptions& opts = {});

// Parse MJCF from a file on disk.
ParseResult ParseMjcfFile(const std::string& path,
                          const ParseOptions& opts = {});

// Serialize a Model to a deterministic MJCF string (2-space indent, angles
// emitted verbatim in their authored unit per Q-ANGLE, shortest round-trip
// numeric formatting). Emits exactly the authored fields.
//
// A model can hold a value MJCF cannot represent: a ref-list entry (e.g.
// Flex.body) whose name contains whitespace would corrupt the space-joined wire
// form (it re-reads as two refs; MuJoCo's mjs_setStringVec has the same
// limitation). Such a model fails to write: the result is the empty string (a
// successful write always starts with "<mujoco") and, when `errors` is
// non-null, one diagnostic per offending entry is appended.
std::string WriteMjcf(const Model& model,
                      std::vector<ps::Diagnostic>* errors = nullptr);

// As above, but unnamed elements whose serial appears in `auto_names` are
// emitted with the mapped reserved name. Used by the compile bridge.
std::string WriteMjcf(const Model& model, const AutoNames& auto_names,
                      std::vector<ps::Diagnostic>* errors = nullptr);

// Diagnostic self-check: true iff every resolver name carried by the generated
// binding metadata (schema `resolver="..."`) has a handler in the reader's
// resolver registry. Unregistered names are appended to `missing` (if non-null).
// A unit test asserts this, so a new schema resolver without a matching reader
// function fails loudly at test time rather than silently no-opping at parse.
bool ResolverRegistryComplete(std::vector<std::string>* missing = nullptr);

}  // namespace ps::mjcf::io

#endif  // PROTOSPEC_IO_MJCF_H
