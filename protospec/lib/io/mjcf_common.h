// MJCF reader/writer: the profile-independent half.
//
// Everything here is keyed on ElementType, on raw XML, or on diagnostics --
// never on how a document stores a value -- so it compiles ONCE no matter how
// many emission profiles the reader and writer are instantiated for. That is
// most of the reader by line count: the 145-case support switch, the ~100-case
// name-namespace switch, the schema presence-constraint evaluator, the six
// raw-XML quirk hooks, numeric parsing and every diagnostic.
//
// The templated remainder lives in mjcf_reader.inc / mjcf_writer.inc, which each
// profile's own translation unit includes exactly once.
//
// What the split actually buys, measured (MSVC 19.44, /O2 /bigobj, warm, x64):
// the pre-split single reader TU took 13.6s; the templated reader TU now takes
// 27.5s and this file 2.2s. So the split is NOT a win for a plain-only build --
// profile genericity roughly doubles the templated half, and pulling the
// concrete half out recovers only 2.2s of that. Its value is the MARGINAL cost
// of a second profile: 27.5s rather than 29.7s, because this file is compiled
// once and shared. Recorded here rather than assumed, because the opposite was
// assumed before it was measured.
//
// Internal to lib/io. tinyxml2 appears in these signatures and must not leak
// into a consumer's include path, which is why this is a private header and why
// the templated halves are .inc files rather than public headers.
#ifndef PROTOSPEC_IO_MJCF_COMMON_H
#define PROTOSPEC_IO_MJCF_COMMON_H

#include <cstdint>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "include.h"
#include "mjcf.h"
#include "numeric.h"
#include "protospec/core.h"
#include "protospec/diag.h"
#include "reflect.h"
#include "tinyxml2.h"
#include "xml_binding.h"

namespace ps::mjcf::io::common {

using tinyxml2::XMLElement;
using xmlbind::AttrBinding;
using xmlbind::ElementBinding;

// The supported families: every element ProtoSpec models as first-class data.
// Everything else is a well-formed but unsupported element (skip signal), never
// a malformed input.
bool IsSupported(ElementType t);

// The MuJoCo name-uniqueness namespace an element's name lives in, as a stable
// label, or nullptr for elements that carry no namespaced name.
const char* NameNsLabel(ElementType et);

// The reader's resolver registry. A schema `resolver="..."` name reaches the
// reader only through this enum: LookupResolver maps the generated binding's
// names onto it, the reader's dispatch switches over it exhaustively (so a new
// enumerator is a compile error until it has a canonicalization), and
// ResolverRegistryComplete walks the generated binding asserting every declared
// name resolves to something other than None. A resolver added to the schema
// therefore cannot silently no-op.
enum class Resolver {
  None, Orientation, MaterialLayer, Inertia, Springlength, LightType,
  CylinderArea,
};

Resolver LookupResolver(std::string_view name);

// Does the schema fold this element's orientation spellings into one canonical
// quat? True exactly when the binding marks a field with the orientation
// resolver. WHICH elements those are is the schema's call: an element that
// spells the five attributes out separately instead of using the group
// (flexcomp) stores each as authored, and folding it would make the writer emit
// two specifiers at once.
bool FoldsOrientation(const ElementBinding& b);

// The reflect field id of the canonical field a named resolver owns on this
// element, or -1. Replaces the hand-written per-element overload sets that used
// to hand back a raw pointer into storage.
int ResolverFieldId(const ElementBinding& b, std::string_view resolver);

// --- ReaderBase ----------------------------------------------------------- //
// The reader's profile-independent state and methods: provenance, diagnostics,
// the nesting-depth cap, raw-XML attribute reads, numeric parsing, the schema
// presence constraints and the six quirk hooks that inspect XML only.
class ReaderBase {
 public:
  ReaderBase(std::string filename, std::vector<ps::Diagnostic>& errors,
             std::vector<ps::Diagnostic>& warnings,
             const ProvenanceMap* provenance = nullptr)
      : filename_(std::move(filename)),
        errors_(errors),
        warnings_(warnings),
        provenance_(provenance) {}

  // Provenance: elements spliced from an included file carry that file's
  // path and line via the pre-pass map; top-level elements take theirs from
  // tinyxml2 against the model filename.
  ps::SourceLoc Loc(const XMLElement* e) const;

  void Err(const XMLElement* e, std::string msg);
  void Unsupported(const XMLElement* e, std::string tag);
  void Warn(const XMLElement* e, std::string msg);

  // Parse-time duplicate-name diagnostic: at first sight of a name that repeats
  // within a MuJoCo name namespace, emit a WARNING carrying BOTH source
  // locations (the first occurrence in the message, this one in the loc). The
  // referential validator remains the authoritative error tier; this only
  // surfaces the collision early, matching stock MuJoCo's fail-at-read shape.
  void CheckDuplicateName(const XMLElement* xml, ElementType et);

  // Hostile inputs can nest elements arbitrarily deep, and reading recurses once
  // per level, so an unbounded document would exhaust the native stack before
  // any diagnostic is produced. Cap nesting far beyond any real model.
  static constexpr int kMaxElementDepth = 200;

  static bool Has(const XMLElement* xml, const char* attr) {
    return xml->Attribute(attr) != nullptr;
  }
  static bool Has(const XMLElement* xml, std::string_view attr) {
    return Has(xml, std::string(attr).c_str());
  }

  // Read exactly n doubles from an attribute (verbatim, no unit conversion).
  bool ReadDoubleArr(XMLElement* xml, const char* attr, int n, double* out);

  void ReportNumError(XMLElement* xml, const AttrBinding& ab, num::Status st);

  // Parse min_count..max_count numbers into buf; returns the count, or -1 on
  // error (already reported). min_count>0 enforces "not enough data".
  template <class S>
  int ReadNumberN(XMLElement* xml, const AttrBinding& ab, std::string_view text,
                  S* buf, int min_count, int max_count) {
    auto toks = num::Tokens(text);
    const int n = static_cast<int>(toks.size());
    if (n < min_count) {
      Err(xml, "attribute '" + std::string(ab.attr) +
                   "' does not have enough data");
      return -1;
    }
    if (n > max_count) {
      Err(xml, "attribute '" + std::string(ab.attr) + "' has too much data");
      return -1;
    }
    for (int i = 0; i < n; ++i) {
      if (!ParseScalar(xml, ab, toks[i], buf[i])) return -1;
    }
    return n;
  }

  template <class S>
  bool ParseScalar(XMLElement* xml, const AttrBinding& ab, std::string_view tok,
                   S& out) {
    if constexpr (std::is_floating_point_v<S>) {
      bool is_nan = false;
      num::Status st = num::ParseFloat<S>(tok, out, is_nan);
      if (st == num::Status::Ok) {
        if (is_nan) {
          Warn(xml, "attribute '" + std::string(ab.attr) +
                        "' contains NaN; please check it carefully");
        }
        return true;
      }
      ReportNumError(xml, ab, st);
      return false;
    } else {
      num::Status st = num::ParseInt<S>(tok, out);
      if (st == num::Status::Ok) return true;
      ReportNumError(xml, ab, st);
      return false;
    }
  }

  // The schema's exclusive / together / requires / oneof rows, evaluated on the
  // raw XML exactly as mjXSchema does, down to the wording. Reading them from
  // the generated tables is what lets the reader carry no per-element hook.
  void CheckConstraints(XMLElement* xml, ElementType type);

  void CheckUnknownAttributes(XMLElement* xml, const ElementBinding& b);

  // --- Raw-XML quirk hooks (no storage touched) --------------------------- //
  void InertialOrientExclusion(XMLElement* xml);
  void ActuatorTransmission(XMLElement* xml);
  void ContactSensorData(XMLElement* xml);
  void PluginSensorPairing(XMLElement* xml);
  void ConfigKeyUnique(XMLElement* xml);

 protected:
  std::string filename_;
  std::vector<ps::Diagnostic>& errors_;
  std::vector<ps::Diagnostic>& warnings_;
  const ProvenanceMap* provenance_ = nullptr;
  int depth_ = 0;  // current element-nesting depth (see kMaxElementDepth)
  // First-seen location per (namespace-label, name) for duplicate-name warnings.
  std::unordered_map<std::string, ps::SourceLoc> name_first_loc_;
};

}  // namespace ps::mjcf::io::common

#endif  // PROTOSPEC_IO_MJCF_COMMON_H
