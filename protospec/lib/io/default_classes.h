// Default-class read-time checks (family c, Q-AUTO context).
//
// ProtoSpec treats defaults as data: classes are read verbatim into the Default
// tree, written back verbatim, and NOTHING is resolved or applied (DR-1). The
// only rules enforced at read time are the two structural ones MuJoCo enforces
// in mjXReader::Default (xml_native_reader.cc:3034-3056), because they gate a
// well-formed <default> tree and MuJoCo rejects violators before compile:
//
//   * a top-level default class must be unnamed or exactly "main"
//     (:3052-3055, "top-level default class 'main' cannot be renamed");
//   * a nested default class name must be non-empty
//     (:3041-3044, "empty class name").
//
// Class-reference resolution (does a referenced class exist?) is deliberately
// NOT done here: it is referential validation (plan Section 9 tier 2, DR-8),
// consistent with how every other ref<T> in the reader is stored by name and
// resolved later. See test_io.cc TestUnknownClassRef for the grounding.
#ifndef PROTOSPEC_IO_DEFAULT_CLASSES_H
#define PROTOSPEC_IO_DEFAULT_CLASSES_H

#include <optional>
#include <string>
#include <vector>

#include "mjcf.h"
#include "protospec/model_core.h"
#include "protospec/plain_profile.h"
#include "protospec/profile.h"
#include "types.h"

namespace ps::mjcf::io {

namespace defaults_detail {

template <class P, class E>
void Err(std::vector<ps::Diagnostic>& errors, const E& d, std::string msg) {
  ps::Diagnostic diag;
  diag.source = "parse";
  diag.kind = ps::Diagnostic::Kind::MalformedInput;
  diag.message = std::move(msg);
  diag.loc = P::Ident::Loc(d);
  errors.push_back(std::move(diag));
}

// Recurse into subclasses, where every default must name a non-empty class.
template <class P, class E>
void CheckNested(const E& d, std::vector<ps::Diagnostic>& errors) {
  P::Tree::template ForEachChildOfType<E>(d, [&](const E& sub) {
    std::optional<ps::sdk::ViewOf<P>> n = ps::sdk::internal::NameOf<P>(sub);
    if (!n || P::Str::Empty(*n)) Err<P>(errors, sub, "empty class name");
    CheckNested<P>(sub, errors);
  });
}

}  // namespace defaults_detail

// Append a MalformedInput diagnostic for each default-tree class-name
// violation, each carrying the offending <default>'s SourceLoc.
template <class P>
void ValidateDefaultClassesT(const ps::sdk::DocOf<P>& model,
                             std::vector<ps::Diagnostic>& errors) {
  using DefaultT = ps::sdk::ElementOf<P, ElementType::Default>;
  P::Tree::template ForEachChildOfType<DefaultT>(
      model, [&](const DefaultT& d) {
        // Top level: unnamed or exactly "main" (xml_native_reader.cc:3052-3055).
        std::optional<ps::sdk::ViewOf<P>> n = ps::sdk::internal::NameOf<P>(d);
        if (n && !P::Str::Empty(*n) && !P::Str::EqualsUtf8(*n, "main")) {
          defaults_detail::Err<P>(
              errors, d, "top-level default class 'main' cannot be renamed");
        }
        defaults_detail::CheckNested<P>(d, errors);
      });
}

void ValidateDefaultClasses(const Model& model,
                            std::vector<ps::Diagnostic>& errors);

}  // namespace ps::mjcf::io

#endif  // PROTOSPEC_IO_DEFAULT_CLASSES_H
