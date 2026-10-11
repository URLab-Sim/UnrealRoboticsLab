// MJCF <include> expansion and source provenance for direct UE import.
// The generated UE descriptors parse the expanded XML; no second document
// model or generic reader is involved.
#ifndef PROTOSPEC_IO_INCLUDE_H
#define PROTOSPEC_IO_INCLUDE_H

#include <string>
#include <unordered_map>
#include <vector>

#include "protospec/diag.h"
#include "protospec/core.h"
#include "tinyxml2.h"

namespace ps::mjcf::io {

// Spliced element -> its source location in the included file. Elements absent
// from the map originate in the top-level document and take their line from
// tinyxml2 directly.
using ProvenanceMap =
    std::unordered_map<const tinyxml2::XMLElement*, ps::SourceLoc>;

struct IncludeResult {
  ProvenanceMap provenance;
  std::vector<ps::Diagnostic> errors;  // MalformedInput on any include failure
};

// Expand every <include> in `doc` in place. `model_dir` is the directory of the
// top-level model file (relative includes resolve there first); `model_file` is
// the top-level file path, which seeds the once-per-file set so a self-include
// is rejected as a duplicate. Both may be empty for string input, in which case
// relative includes resolve against (and are confined to) the current working
// directory.
//
// By default an <include> whose resolved path escapes the root model's directory
// tree (`model_dir`, or the cwd when empty) is rejected with a MalformedInput
// diagnostic and its file is never opened -- a hardening measure since the host
// loads untrusted models. Pass allow_external_includes=true to permit external
// files for trusted workflows.
IncludeResult ExpandIncludes(tinyxml2::XMLDocument& doc,
                             const std::string& model_dir,
                             const std::string& model_file,
                             bool allow_external_includes = false);

}  // namespace ps::mjcf::io

#endif  // PROTOSPEC_IO_INCLUDE_H
