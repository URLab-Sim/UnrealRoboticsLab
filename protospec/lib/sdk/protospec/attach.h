// ProtoSpec SDK: attach (namespaced subtree splice).
//
// Attach deep-clones a body subtree from one document (or another point in the
// same document) into a parent body, prefixing every name AND every internal
// typed reference of the clone so the graft is self-contained: joints/sites/geoms
// the clone refers to resolve within the clone, never colliding with the host.
// This mirrors MuJoCo's mjs_attach namespacing but is a pure tree operation --
// no compile, no source mutation (the source is cloned, not moved, unlike
// mjs_attach which mutates its source child).
//
// A name collision against the host is reported and blocks the attach (the host
// is left untouched), rather than producing two elements of one type sharing a
// name (which a valid model forbids, Q-NAMES). Assets and default classes the
// source relied on are NOT copied; a clone's prefixed asset/class references are
// left for the caller to satisfy (bring the assets over, or clear the refs).
#ifndef PROTOSPEC_SDK_ATTACH_H
#define PROTOSPEC_SDK_ATTACH_H

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

template <class P = plain::Plain>
struct AttachResult {
  // the grafted clone, when ok (AttachModel: the last body grafted, null when
  // the source had none)
  ElementOf<P, mj::ElementType::Body>* attached = nullptr;
  bool ok = false;                      // false when a collision blocked it
  std::vector<std::string> collisions;  // "type:name" clashes with the host
  // Uniform result-object contract (see refs.h): truthy when the splice applied.
  explicit operator bool() const { return ok; }
};

namespace detail {

// Prefix every name and every internal reference throughout a cloned subtree.
template <class P, class E>
void PrefixSubtree(E& clone, ViewOf<P> prefix) {
  WalkTree<P>(clone, [&](auto& e) {
    if (std::optional<ViewOf<P>> nm = NameOf<P>(e)) {
      StrOf<P> prefixed = P::Str::Concat(prefix, *nm);
      SetName<P>(e, P::Str::View(prefixed));
    }
    PrefixRefs<P>(e, prefix);
  });
}

// (category,name) key over the shared MuJoCo name namespaces; the category
// folding itself (detail.h NameCategory) is shared with Rename's collision
// rejection.
inline std::string Key(mj::ElementType t, std::string_view name) {
  return std::to_string(NameCategory(t)) + ':' + std::string(name);
}

// Every (category,name) an element of the host already uses.
template <class P>
std::unordered_set<std::string> HostNames(const DocOf<P>& model) {
  std::unordered_set<std::string> names;
  WalkModelAll<P>(model, [&](const auto& e) {
    using E = std::decay_t<decltype(e)>;
    if (std::optional<ViewOf<P>> nm = NameOf<P>(e))
      names.insert(Key(ElementTypeOf<P, E>, P::Str::ToUtf8(*nm)));
  });
  return names;
}

// Report every element of `clone` whose (category,name) the host already holds.
template <class P, class E>
void CollectCollisions(E& clone, const std::unordered_set<std::string>& host,
                       std::vector<std::string>& out) {
  WalkTree<P>(clone, [&](auto& e) {
    using X = std::decay_t<decltype(e)>;
    if (std::optional<ViewOf<P>> nm = NameOf<P>(e)) {
      const std::string utf8 = P::Str::ToUtf8(*nm);
      if (host.count(Key(ElementTypeOf<P, X>, utf8))) {
        out.push_back(
            std::string(mj::reflect::Describe(ElementTypeOf<P, X>).name) + ":" +
            utf8);
      }
    }
  });
}

}  // namespace detail

// Clone `src` (a body subtree), prefix all its names and internal references,
// check for collisions against the host `model`, and on success splice it under
// `parent`. On collision nothing is attached and the clashes are reported.
template <class P = plain::Plain>
AttachResult<P> Attach(DocOf<P>& model,
                       ElementOf<P, mj::ElementType::Body>& parent,
                       const ElementOf<P, mj::ElementType::Body>& src,
                       ViewOf<P> prefix) {
  using BodyT = ElementOf<P, mj::ElementType::Body>;
  AttachResult<P> result;
  OwnerOf<P, BodyT> clone = P::Ident::Clone(src);
  detail::PrefixSubtree<P>(*clone, prefix);

  const std::unordered_set<std::string> host = detail::HostNames<P>(model);
  detail::CollectCollisions<P>(*clone, host, result.collisions);

  if (!result.collisions.empty()) {
    result.ok = false;
    return result;  // host untouched
  }

  result.attached = &P::Tree::template Adopt<BodyT>(parent, kAppend,
                                                    std::move(clone));
  result.ok = true;
  return result;
}

// Attach every top-level body of another document's worldbody under `parent`,
// sharing one prefix. Collisions from any body abort the whole splice (nothing
// is attached) and are aggregated in the result.
template <class P = plain::Plain>
AttachResult<P> AttachModel(DocOf<P>& model,
                            ElementOf<P, mj::ElementType::Body>& parent,
                            const DocOf<P>& src, ViewOf<P> prefix) {
  using BodyT = ElementOf<P, mj::ElementType::Body>;
  AttachResult<P> result;
  const BodyT* world = P::Tree::template FirstChildOfType<BodyT>(src);
  if (world == nullptr) {
    result.ok = true;
    return result;
  }

  // Dry-run collision scan across all source bodies first.
  const std::unordered_set<std::string> host = detail::HostNames<P>(model);
  std::vector<OwnerOf<P, BodyT>> clones;
  P::Tree::template ForEachChildOfType<BodyT>(*world, [&](const BodyT& b) {
    OwnerOf<P, BodyT> clone = P::Ident::Clone(b);
    detail::PrefixSubtree<P>(*clone, prefix);
    detail::CollectCollisions<P>(*clone, host, result.collisions);
    clones.push_back(std::move(clone));
  });

  if (!result.collisions.empty()) {
    result.ok = false;
    return result;
  }
  for (auto& c : clones)
    result.attached =
        &P::Tree::template Adopt<BodyT>(parent, kAppend, std::move(c));
  result.ok = true;
  return result;
}

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
// Unlike Attach, this is a same-document, unprefixed duplicate -- the everyday
// "copy this element" verb.
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

#endif  // PROTOSPEC_SDK_ATTACH_H
