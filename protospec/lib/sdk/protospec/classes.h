// ProtoSpec SDK: default-class operations.
//
// Defaults are first-class data: a <default> class holds an unset-everything
// partial per defaultable family, and elements inherit unauthored fields from
// their class, its ancestor classes, `main`, and finally the IDL defaults, in
// that order (first authored value wins). These operations expose that layering.
//
// This header carries the QUERY half only -- Effective, EffectiveField,
// EffectiveRef and the layering they share. It reaches upward through
// ParentMap and nothing else, so a consumer of the class layering (the editor
// preview, which resolves an inherited value while a document is being edited)
// does not pull in the find, rename or builder machinery. The two mutating
// authoring transforms built on the same layering, FlattenDefaults and
// ExtractClass, live in protospec/class_edits.h.
//
// Scope: the layered merge is defined for families whose class partial has the
// same element type as the live element (geom, joint, site, camera, light,
// mesh, material, pair, and every actuator spelling that has a <default> block).
// MuJoCo's equality/tendon defaults use a distinct partial type and are out of
// scope for these merges; such elements are passed through unchanged. Which
// families those are is read from the schema (reflect::IsDefaultFamily), not
// from a hand list here.
#ifndef PROTOSPEC_SDK_CLASSES_H
#define PROTOSPEC_SDK_CLASSES_H

#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include "protospec/detail.h"
#include "protospec/parents.h"
#include "protospec/plain_profile.h"
#include "protospec/profile.h"

namespace ps::sdk {

namespace mj = ps::mjcf;

// --- Family mapping ------------------------------------------------------- //
// An element type has a same-type <default> class partial when the schema
// declares one under <default>. The generated table (reflect::kDefaultFamilies)
// is the single source; nothing here re-lists a family, and the coverage gate
// below checks the table against the runtime descriptor rather than against a
// second hand list.

template <class P, class T>
inline constexpr bool has_default_family_v =
    mj::reflect::IsDefaultFamily(ElementTypeOf<P, T>);

// The <default> children that are same-type partials but are missing from the
// generated family table -- empty by construction, and asserted so a change to
// the emitter that dropped a family fails loudly rather than silently passing
// that family through unmerged.
//
// A <default> child is a same-type partial when its class element has the SAME
// element type as the live element it defaults (geom, joint, ...). The two
// distinct-partial families use a dedicated type named `<Family>Default`
// (EqualityDefault, TendonDefault) and the recursive `subclasses` child is a
// nested Default; all three carry the `Default` name suffix and are out of scope
// for the same-type merge, so they are skipped here.
inline std::vector<mj::ElementType> DefaultFamilyCoverageGaps() {
  std::vector<mj::ElementType> gaps;
  const mj::reflect::ElementDescriptor& d =
      mj::reflect::Describe(mj::ElementType::Default);
  for (std::size_t i = 0; i < d.child_count; ++i) {
    std::string_view target = d.children[i].target;
    if (target.size() >= 7 && target.substr(target.size() - 7) == "Default") {
      continue;  // Default / EqualityDefault / TendonDefault: not same-type
    }
    const mj::reflect::ElementDescriptor* cd =
        mj::reflect::DescribeByName(target);
    if (cd && !mj::reflect::IsDefaultFamily(cd->type)) gaps.push_back(cd->type);
  }
  return gaps;
}

// --- Shared class-layering surface (ps::sdk::internal) -------------------- //
// The <default> class index and class-name resolution. These live in classes.h
// rather than model_core.h because ResolveClassName is defined over ParentMap
// (parents.h), which sits above model_core in the include order.

namespace internal {

// The profile's <default> element type.
template <class P>
using DefaultOf = ElementOf<P, mj::ElementType::Default>;

// Index of the <default> class tree: name -> node, node -> parent, plus the
// top-level blocks that make up the root `main` class.
//
// A document may carry SEVERAL top-level <default> blocks -- the norm once
// <include> merges two files' top-level blocks into one document. MuJoCo's
// reader has exactly one root default (`main`, created once in the mjCModel
// constructor) and feeds every top-level <default> section to it in document
// order: each block's direct element defaults merge field-wise into `main`
// (later blocks overwrite earlier per field) and each block's nested classes are
// added to the one shared, flat class namespace (xml/xml_native_reader.cc
// mjXReader::Default: at top level `def==nullptr`, so it takes `def =
// mjs_getSpecDefault(spec)` -- the single main -- rather than allocating a new
// one, and a top-level class name other than "" / "main" is rejected).
// Same-named classes across blocks are invalid: mjs_addDefault ->
// mjCModel::AddDefault checks the flat defaults_ list and returns null on a
// repeat, which the reader turns into "repeated default class name". The SDK
// does not validate, so it keeps the first occurrence of a duplicate key.
//
// A nested class snapshots `main` at the point it is parsed (mjCModel::AddDefault
// -> CopyWithoutChildren of the parent), so a class only inherits the root-level
// fields of the top-level blocks up to and including its own (later blocks' root
// fields never reach an earlier block's class). Roots() preserves that document
// order and RootRank() locates a block within it so the class chain can
// reproduce the snapshot.
template <class P>
class DefaultIndex {
 public:
  using Node = DefaultOf<P>;

  explicit DefaultIndex(const DocOf<P>& m) {
    P::Tree::template ForEachChildOfType<Node>(m, [&](const Node& d) {
      roots_.push_back(&d);
      Add(d, nullptr);
    });
  }

  const Node* ByNameOrRoot(ViewOf<P> n) const {
    // "" and "main" both name the root; with multiple top-level blocks it is the
    // LAST one, so a class-free element sees `main` with every block merged in.
    if (P::Str::Empty(n) || P::Str::EqualsUtf8(n, "main")) return root_;
    auto it = by_name_.find(P::Str::FromUtf8(P::Str::ToUtf8(n)));
    return it != by_name_.end() ? it->second : root_;
  }
  const Node* ParentOf(const Node* d) const {
    auto it = parent_.find(d);
    return it != parent_.end() ? it->second : nullptr;
  }
  // Top-level <default> blocks in document order; together they form `main`.
  const std::vector<const Node*>& Roots() const { return roots_; }
  // Document position of a top-level block in Roots(), or -1 if `d` is nested.
  int RootRank(const Node* d) const {
    for (int i = 0; i < static_cast<int>(roots_.size()); ++i)
      if (roots_[i] == d) return i;
    return -1;
  }

  // The class chain of `className`, highest priority first: the class, its
  // ancestors, then the top-level blocks that make up `main` (highest-ranked
  // first, since a later block overwrites an earlier one per field). One
  // resolution serves both the whole-element merge and a single-field query, so
  // the two can never disagree about precedence.
  std::vector<const Node*> Chain(ViewOf<P> className) const {
    std::vector<const Node*> out;
    for (const Node* d = ByNameOrRoot(className); d; d = ParentOf(d)) {
      if (ParentOf(d) == nullptr) {  // terminal: a top-level block -> `main`
        const int rank = RootRank(d);
        if (rank < 0) {
          out.push_back(d);
        } else {
          for (int i = rank; i >= 0; --i)
            if (roots_[i]) out.push_back(roots_[i]);
        }
        break;
      }
      out.push_back(d);
    }
    return out;
  }

 private:
  void Add(const Node& d, const Node* par) {
    std::optional<ViewOf<P>> nm = detail::NameOf<P>(d);
    StrOf<P> name =
        nm ? P::Str::FromUtf8(P::Str::ToUtf8(*nm)) : StrOf<P>();
    by_name_.emplace(name, &d);  // first occurrence wins (a duplicate is invalid)
    parent_[&d] = par;
    if (P::Str::Empty(P::Str::View(name)) ||
        P::Str::EqualsUtf8(P::Str::View(name), "main"))
      root_ = &d;
    P::Tree::template ForEachChildOfType<Node>(
        d, [&](const Node& s) { Add(s, &d); });
  }

  std::unordered_map<StrOf<P>, const Node*, StrHash<P>, StrEq<P>> by_name_;
  std::unordered_map<const Node*, const Node*> parent_;
  std::vector<const Node*> roots_;
  const Node* root_ = nullptr;
};

// The class an element authored directly, empty when none.
template <class P, class T>
ViewOf<P> OwnClass(const T& e) {
  const int id = detail::FieldIdByName(ElementTypeOf<P, T>, "dclass");
  if (id < 0) return ViewOf<P>();
  return P::Ref::NameAt(e, id);
}

// The class governing an element: its own class, else the nearest enclosing
// body-context childclass, else "" (the root/main class).
template <class P>
ViewOf<P> ResolveClassName(const ParentMap<P>& pm, ViewOf<P> own,
                           const void* elemPtr) {
  if (!P::Str::Empty(own)) return own;
  const typename ParentMap<P>::Node* n = pm.Lookup(elemPtr);
  for (const void* p = n ? n->parent : nullptr; p;) {
    const typename ParentMap<P>::Node* pn = pm.Lookup(p);
    if (!pn) break;
    if (!P::Str::Empty(P::Str::View(pn->childclass)))
      return P::Str::View(pn->childclass);
    p = pn->parent;
  }
  return ViewOf<P>();
}

// The root `main` default, created if absent.
template <class P>
DefaultOf<P>& EnsureRoot(DocOf<P>& model) {
  DefaultOf<P>* found = nullptr;
  P::Tree::template ForEachChildOfType<DefaultOf<P>>(
      model, [&](DefaultOf<P>& d) {
        if (found) return;
        std::optional<ViewOf<P>> n = detail::NameOf<P>(d);
        if (!n || P::Str::Empty(*n) || P::Str::EqualsUtf8(*n, "main"))
          found = &d;
      });
  if (found) return *found;
  DefaultOf<P>& d = detail::Create<P, DefaultOf<P>>(model);
  detail::SetName<P>(d, P::Str::View(P::Str::FromUtf8("main")));
  return d;
}

}  // namespace internal

// --- Field-wise "fill unauthored" merge ----------------------------------- //

namespace detail {

// The class-layering surface above is shared with the compiler (ps::sdk::
// internal); re-exported here under the `detail::` spelling the SDK's own
// class-merge code uses.
using internal::DefaultIndex;
using internal::DefaultOf;
using internal::EnsureRoot;
using internal::OwnClass;
using internal::ResolveClassName;

// Through the shape policy rather than by assigning the slot: a slot is not
// always a value. A profile whose reference fields reach Visit as an aliasing
// proxy would see `dst = src` rebind the proxy instead of authoring the field,
// which is a silent wrong answer where authoring the inner value is not.
template <class P, class U>
void MergeField(U& dst, const U& src) {
  if constexpr (P::Shape::template optional_v<U>) {
    if (!P::Shape::IsSet(dst) && P::Shape::IsSet(src))
      P::Shape::Author(dst, P::Shape::Read(src));
  }
}

// Fill each unauthored field of `dst` from `src` (same element type). Required
// (non-optional) fields are left untouched -- a class never overrides structure.
template <class P, class T>
struct MergeVisitor {
  const T* src;
  template <class U>
  void field(int id, const char*, U& dst) {
    const U* s = FieldAt<P, T, U>(*src, id);
    if (s) MergeField<P>(dst, *s);
  }
  template <class C>
  void child(int, const char*, C&) {}
  template <class C>
  void union_child(int, const char*, C&) {}
};

template <class P, class T>
void MergeUnset(T& dst, const T& src) {
  MergeVisitor<P, T> v{&src};
  P::Visit(dst, v);
}

// The class partial for family T carried by one <default> node, or nullptr.
template <class P, class T>
const T* FamilyPartial(const DefaultOf<P>& d) {
  return P::Tree::template FirstChildOfType<T>(d);
}

// Merge the class chain into `target`, highest priority first (MergeUnset fills
// only-still-unset fields, so an earlier merge wins).
template <class P, class T>
void MergeClassChain(const DefaultIndex<P>& idx, ViewOf<P> className, T& target) {
  for (const DefaultOf<P>* d : idx.Chain(className))
    if (const T* partial = FamilyPartial<P, T>(*d)) MergeUnset<P>(target, *partial);
}

}  // namespace detail

// --- Effective (query) ---------------------------------------------------- //

// The lookup state Effective needs, built once from a document: the parent map
// (childclass resolution) and the <default> class index, plus a per-element memo
// of the resolved class chain. A caller issuing many queries against an
// unchanged document (a drag frame, a panel render) builds one context for the
// batch instead of paying both walks per call, and a details panel asking for 30
// fields of one element resolves that element's chain ONCE rather than 30 times
// -- without the memo the allocation-free per-field form would trade one clone
// for thirty chain resolutions, which is worse on exactly the path it exists to
// speed up.
//
// Holds pointers into the document: any tree mutation invalidates it, so build
// it per frame/batch and discard -- never cache across edits.
template <class P = plain::Plain>
class EffectiveContext {
 public:
  explicit EffectiveContext(const DocOf<P>& m) : pm_(m), idx_(m) {}
  const ParentMap<P>& parents() const { return pm_; }
  const detail::DefaultIndex<P>& classes() const { return idx_; }

  // The class chain governing `e`, resolved once per element and remembered.
  template <class T>
  const std::vector<const detail::DefaultOf<P>*>& ChainFor(const T& e) const {
    auto it = chain_memo_.find(static_cast<const void*>(&e));
    if (it != chain_memo_.end()) return it->second;
    ViewOf<P> cls =
        detail::ResolveClassName<P>(pm_, detail::OwnClass<P, T>(e), &e);
    auto [ins, ok] = chain_memo_.emplace(static_cast<const void*>(&e),
                                         idx_.Chain(cls));
    (void)ok;
    return ins->second;
  }

 private:
  ParentMap<P> pm_;
  detail::DefaultIndex<P> idx_;
  mutable std::unordered_map<const void*,
                             std::vector<const detail::DefaultOf<P>*>>
      chain_memo_;
};

// The effective value of an element with all class layers resolved: a fresh copy
// of `e` with every unauthored field filled from its class chain and (when
// `apply_idl_defaults`) the IDL defaults. Does not mutate the document. For
// families without a same-type class partial, returns `e` plus IDL defaults.
// `ctx` must have been built from the document that owns `e`, after its last
// mutation.
template <class P = plain::Plain, class T>
OwnerOf<P, T> Effective(const EffectiveContext<P>& ctx, const T& e,
                        bool apply_idl_defaults = true) {
  OwnerOf<P, T> out = P::Ident::Clone(e);
  if constexpr (has_default_family_v<P, T>) {
    for (const detail::DefaultOf<P>* d : ctx.ChainFor(e))
      if (const T* partial = detail::FamilyPartial<P, T>(*d))
        detail::MergeUnset<P>(*out, *partial);
  }
  // Schema defaults are the lowest priority layer: fill only fields still unset
  // after element + class. ApplyDefault assigns unconditionally, so route it
  // through a fill-only merge rather than calling it on `out` directly.
  if (apply_idl_defaults) {
    detail::MergeUnset<P>(*out, P::template Defaults<T>());
  }
  return out;
}

// Single-query convenience: builds the lookup context for this one call.
template <class P = plain::Plain, class T>
OwnerOf<P, T> Effective(const DocOf<P>& model, const T& e,
                        bool apply_idl_defaults = true) {
  return Effective<P>(EffectiveContext<P>(model), e, apply_idl_defaults);
}

// --- Per-field Effective (allocation-free) -------------------------------- //

// The effective value of ONE field of `e`, resolved through the same layering
// Effective applies (element, class chain, `main`, IDL defaults) but without
// materializing a copy. `out` is written and true returned only when some layer
// authored the field; false leaves `out` untouched.
//
// This is the form a details panel wants: a component profile would otherwise
// pay an object allocation per queried field.
template <class P = plain::Plain, class T, class U>
bool EffectiveField(const EffectiveContext<P>& ctx, const T& e, int field_id,
                    U& out) {
  if (const U* own = detail::FieldAt<P, T, U>(e, field_id)) {
    if (P::Shape::IsSet(*own)) {
      out = *own;
      return true;
    }
  }
  if constexpr (has_default_family_v<P, T>) {
    for (const detail::DefaultOf<P>* d : ctx.ChainFor(e)) {
      const T* partial = detail::FamilyPartial<P, T>(*d);
      if (!partial) continue;
      if (const U* v = detail::FieldAt<P, T, U>(*partial, field_id)) {
        if (P::Shape::IsSet(*v)) {
          out = *v;
          return true;
        }
      }
    }
  }
  const T& defs = P::template Defaults<T>();
  if (const U* v = detail::FieldAt<P, T, U>(defs, field_id)) {
    if (P::Shape::IsSet(*v)) {
      out = *v;
      return true;
    }
  }
  return false;
}

// The effective NAME held by a reference field of `e`, resolved through the same
// layering. False (and `out` untouched) when no layer authored it.
template <class P = plain::Plain, class T>
bool EffectiveRef(const EffectiveContext<P>& ctx, const T& e, int field_id,
                  StrOf<P>& out) {
  auto own = P::Ref::NameAt(e, field_id);
  if (!P::Str::Empty(own)) {
    out = P::Str::FromUtf8(P::Str::ToUtf8(own));
    return true;
  }
  if constexpr (has_default_family_v<P, T>) {
    for (const detail::DefaultOf<P>* d : ctx.ChainFor(e)) {
      const T* partial = detail::FamilyPartial<P, T>(*d);
      if (!partial) continue;
      auto v = P::Ref::NameAt(*partial, field_id);
      if (!P::Str::Empty(v)) {
        out = P::Str::FromUtf8(P::Str::ToUtf8(v));
        return true;
      }
    }
  }
  return false;
}

}  // namespace ps::sdk

#endif  // PROTOSPEC_SDK_CLASSES_H
