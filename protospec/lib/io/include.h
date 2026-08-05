// MJCF <include> pre-pass (Q-INC, DR-7): the one construct that cannot pass
// through as data. Before the table-driven reader runs, every <include> in the
// document is expanded in place with MuJoCo's exact rules (xml.cc:101-240):
// the file attribute is required, the element must have no children, each file
// is included at most once GLOBALLY (the set is seeded with the top-level
// file), expansion is allowed anywhere in the tree, and a relative file is
// resolved against the model directory first and then the including file's
// directory. One rule is ours, not MuJoCo's: include nesting is capped at 64
// levels (expansion recurses once per chained file, so depth must be bounded
// to keep the native stack safe); exceeding the cap is a normal reader error.
//
// Unlike MuJoCo, which nests the included subtree inside the surviving
// <include> wrapper, ProtoSpec splices the included root's children directly
// into the parent and drops the <include> element, yielding a flat document
// the reader consumes with no knowledge of includes. Because tinyxml2 clones
// do not carry parse line numbers across documents, provenance (DR-9) for
// every spliced element is captured into a side map keyed on the clone: the
// reader consults it so a SourceLoc points at the INCLUDED file's path and
// line, not the top-level file.
//
// Asset file contents are never loaded (paths are data, resolved later by the
// bridge); only the include files themselves are read, since include is file
// plumbing. The compiler meshdir/texturedir/assetdir capture MuJoCo performs
// during this pass (xml.cc:106-121) is a no-op here: those directories round
// trip verbatim through the normal <compiler> read and are consumed by the
// bridge at compile, not by include resolution.
#ifndef PROTOSPEC_IO_INCLUDE_H
#define PROTOSPEC_IO_INCLUDE_H

#include <string>
#include <unordered_map>
#include <vector>

#include "mjcf.h"
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
