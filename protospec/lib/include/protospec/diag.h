// ProtoSpec diagnostics: one structured finding shape shared by every layer.
//
// The MJCF reader/writer (protospec/lib/io), the validator (protospec/lib/validate),
// and the compile bridge (protospec/lib/compile) all raise findings with the same fields, so a front-end
// handles errors uniformly and diagnostics flow across layers without
// hand-translation. `source` names the producing pass/layer ("parse", "write",
// "validate", "serialize", "load", "bind", ...). Layer-specific classification
// survives as fields, never as a separate type:
//   - `kind` distinguishes the reader's well-formed-but-unsupported-element
//     report from ordinary malformed input (queried by ParseResult and the
//     unsupported-file harness); it defaults to None for every other source.
//   - `tier` carries the validator's structural/referential/semantic tier (which
//     callers filter on); it defaults to None for every other source.
//   - `tag` is optional context (the validator's element path, empty otherwise).
//
// Field order is a contract: `{severity, source, message, loc}` are the first
// four members so positional aggregate initialization stays valid for the
// bridge/native construction sites that predate this header.
#ifndef PROTOSPEC_DIAG_H
#define PROTOSPEC_DIAG_H

#include <string>

#include "protospec/core.h"

namespace ps {

struct Diagnostic {
  enum class Severity { Error, Warning };

  // Reader classification: a well-formed element outside the supported families
  // (UnsupportedElement) is kept distinct from malformed input so a harness can
  // skip such a file. None for non-reader sources.
  enum class Kind { None, MalformedInput, UnsupportedElement };

  // Validator tier (numeric values are rendered as "[tierN]" and filtered on by
  // callers). None for non-validate sources.
  enum class Tier { None = 0, Structural = 1, Referential = 2, Semantic = 3 };

  Severity severity = Severity::Error;
  std::string source;    // producing pass/layer
  std::string message;
  ps::SourceLoc loc;     // empty file for programmatic elements
  std::string tag;       // optional context (validate's element path)
  Kind kind = Kind::None;
  Tier tier = Tier::None;

  // "file:line: [label] message  (tag)". The label is "[tierN]" for validate
  // findings, else "[source]"; location, label, and tag are each omitted when
  // their field is empty/None.
  std::string Render() const {
    std::string s;
    if (!loc.file.empty()) {
      s += loc.file;
      if (loc.line > 0) {
        s += ':';
        s += std::to_string(loc.line);
      }
      s += ": ";
    }
    if (tier != Tier::None) {
      s += "[tier";
      s += std::to_string(static_cast<int>(tier));
      s += "] ";
    } else if (!source.empty()) {
      s += '[';
      s += source;
      s += "] ";
    }
    s += message;
    if (!tag.empty()) {
      s += "  (";
      s += tag;
      s += ')';
    }
    return s;
  }
};

}  // namespace ps

#endif  // PROTOSPEC_DIAG_H
