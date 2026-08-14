// ProtoSpec SDK: same-document structural edits.
//
// Duplicate deep-clones an element in place, uniquely renaming every named
// element of the clone and remapping the clone's INTERNAL typed references to
// the new names; references pointing outside the clone are left untouched.
// Reparent moves an element to a new parent without touching names or refs.
// Both are keyed on a runtime element pointer, so an editor can drive them from
// a selection without knowing the concrete type.
//
// The sibling deletion verbs (DeleteRecursive, DeleteSubtree) live in refs.h,
// which owns the reference fallout they report.
#ifndef PROTOSPEC_SDK_EDITS_H
#define PROTOSPEC_SDK_EDITS_H

#include <cstddef>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

#include "protospec/builders.h"
#include "protospec/detail.h"
#include "protospec/plain_profile.h"
#include "protospec/profile.h"
#include "protospec/refs.h"
#include "protospec/traversal.h"

namespace ps::sdk {

namespace mj = ps::mjcf;

// --- Duplicate (same-document deep clone) --------------------------------- //

// Uniform result-object for Duplicate (see the convention note in refs.h). `ok`
// with `clone` (the clone's root element, ElementType matching the source) on
// success; `ok == false` with a `reason` when the source was not found.
struct DuplicateResult {
  bool ok = false;
  void* clone = nullptr;
  std::string reason;
  explicit operator bool() const { return ok; }
  // Convenience for the common "I know the concrete type" case.
  template <class T>
  T* As() const {
    return static_cast<T*>(clone);
  }
};

namespace detail {

// Uniquely rename every named element of the freshly-cloned subtree rooted at
// `clone`, and remap the clone's INTERNAL typed references to the new names;
// references pointing OUTSIDE the clone are left untouched. All bookkeeping runs
// through generic-lambda walks, never a per-type templated op with an embedded
// walk.
template <class P>
void RemapClone(DocOf<P>& model, const void* clone) {
  ParentMap<P> pm(model);
  auto in_sub = [&](const void* p) -> bool {
    for (const void* q = p; q;) {
      if (q == clone) return true;
      const typename ParentMap<P>::Node* n = pm.Lookup(q);
      if (!n) break;
      q = n->parent;
    }
    return false;
  };

  struct SubElem {
    mj::ElementType type;
    StrOf<P> name;
    std::function<void(ViewOf<P>)> set;
  };
  std::vector<SubElem> sub;
  std::map<int, std::unordered_set<std::string>> reserved;  // outside names
  WalkModelAll<P>(model, [&](auto& e) {
    using E = std::decay_t<decltype(e)>;
    if constexpr (!IsRoot<P, E>) {
      std::optional<ViewOf<P>> nm = NameOf<P>(e);
      if (!nm) return;
      const mj::ElementType t = ElementTypeOf<P, E>;
      if (in_sub(&e)) {
        auto* ep = &e;
        sub.push_back({t, P::Str::FromUtf8(P::Str::ToUtf8(*nm)),
                       [ep](ViewOf<P> s) { SetName<P>(*ep, s); }});
      } else {
        reserved[NameCategory(t)].insert(P::Str::ToUtf8(*nm));
      }
    }
  });

  struct Ren {
    mj::ElementType type;
    StrOf<P> oldn;
    StrOf<P> newn;
  };
  std::vector<Ren> renamed;
  for (auto& se : sub) {
    const int c = NameCategory(se.type);
    const std::string base = P::Str::ToUtf8(P::Str::View(se.name));
    std::string cand = base;
    for (int k = 1; reserved[c].count(cand); ++k)
      cand = base + "_" + std::to_string(k);
    reserved[c].insert(cand);
    if (cand != base) {
      StrOf<P> newn = P::Str::FromUtf8(cand);
      renamed.push_back({se.type, se.name, newn});
      se.set(P::Str::View(newn));
    }
  }
  if (renamed.empty()) return;

  WalkModelAll<P>(model, [&](auto& e) {
    using E = std::decay_t<decltype(e)>;
    if constexpr (!IsRoot<P, E>) {
      if (!in_sub(&e)) return;
      ScanRefs<P>(e, [&](int, const char*, auto&& slot,
                         const std::vector<mj::ElementType>& tgts) {
        for (const Ren& r : renamed) {
          if (P::Str::Equals(P::Str::View(r.oldn), slot.Get()) &&
              Contains(tgts, r.type)) {
            slot.Set(P::Str::View(r.newn));
            break;
          }
        }
      });
    }
  });
}

}  // namespace detail

// Deep-clone `elem` (a whole subtree) as its next sibling in the same document,
// with fresh serials. Names in the clone are re-uniqued against the rest of the
// document, and references INTERNAL to the clone are remapped to the new names;
// references pointing outside the clone are preserved. On success `ok` is true
// and `clone` is the clone's root element (type-erased; its ElementType matches
// `elem`'s); on failure `ok` is false with a `reason` when `elem` is not found.
// This is a same-document, unprefixed duplicate -- the everyday "copy this
// element" verb.
template <class P = plain::Plain>
DuplicateResult Duplicate(DocOf<P>& model, const void* elem) {
  void* clone = P::Tree::CloneAsNextSibling(model, elem);
  if (clone == nullptr) return {false, nullptr, "element is not in the model"};
  detail::RemapClone<P>(model, clone);
  return {true, clone, ""};
}

// --- Reparent (pure-tree move) -------------------------------------------- //

struct ReparentResult {
  bool ok = false;
  std::string reason;  // why the move was rejected (empty on success)
  // Uniform result-object contract (see refs.h): truthy when the move applied.
  explicit operator bool() const { return ok; }
};

// Move the body-context child `elem` (Body / Geom / Joint / FreeJoint / Site /
// Camera / Light / Frame) out of its current container and into `new_parent`
// (a Body or Frame; nullptr == the world body). This is a PURE TREE operation:
// the element keeps its authored local pose, so its WORLD pose changes with the
// new parent -- pose-preserving reparent is a compile-aware concern that stays
// with the bridge/editor (it needs the compiled parent world pose, which the SDK
// deliberately does not compute). Rejects, leaving the document untouched:
//   * `elem` is not a movable body-context child;
//   * `new_parent` is not a Body or Frame (nor the world);
//   * `new_parent` is `elem` itself or lies inside `elem`'s subtree (a cycle).
template <class P = plain::Plain>
ReparentResult Reparent(DocOf<P>& model, const void* elem, void* new_parent) {
  ReparentResult r;
  switch (P::Tree::Reparent(model, elem, new_parent)) {
    case MoveStatus::Ok:
      r.ok = true;
      break;
    case MoveStatus::NotMovable:
      r.reason = "element is not a movable body-context child";
      break;
    case MoveStatus::BadTarget:
      r.reason = "target is not a body or frame";
      break;
    case MoveStatus::Cycle:
      r.reason = "cannot reparent into its own subtree";
      break;
  }
  return r;
}

}  // namespace ps::sdk

#endif  // PROTOSPEC_SDK_EDITS_H
