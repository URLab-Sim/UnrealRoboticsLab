// ProtoSpec: shared model-core machinery (ps::sdk::internal).
//
// This is the internal seam SHARED between the ProtoSpec SDK's verbs and the
// hosts that bring their own document profile. It is NOT a stable public surface
// -- a consumer of the tree library programs against the verbs aggregated in
// sdk.h (Name, WalkModel, WalkSubtree, Rename, ...). But unlike detail.h
// (genuinely private to the SDK), this header is a real, named contract with
// more than one in-tree consumer, so it is written and documented as one.
//
// CONTRACT: any change to a symbol here must update every consumer in the same
// change -- the SDK headers (classes.h / parents.h and the fixture's traversal.h
// / refs.h / builders.h) and the host profiles. It carries no compatibility
// guarantee beyond that; it is not versioned and not exported.
//
// Everything here is generic over an emission profile `P` (profile.h). A profile
// supplies a per-element Visit hook handing a visitor every field by id + name
// and every child list, plus the five policies. That is enough to build a whole-
// tree walk, per-field probes, name access and reference rewriting with no
// element-specific and no storage-specific code, so both the SDK and the
// compiler stay thin, never need regenerating when the schema grows, and run
// unchanged on a second profile.
#ifndef PROTOSPEC_SDK_MODEL_CORE_H
#define PROTOSPEC_SDK_MODEL_CORE_H

#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include "protospec/core.h"
#include "protospec/profile.h"
#include "reflect.h"

namespace ps::sdk::internal {

namespace mj = ps::mjcf;

// --- Reflection lookups --------------------------------------------------- //

// The reflect field id of `name` on element type `t`, or -1. Field ids are the
// ids the profile's Visit hands out, so this is the bridge from a schema field
// name to a storage slot without naming a C++ member.
inline int FieldIdByName(mj::ElementType t, std::string_view name) {
  const mj::reflect::ElementDescriptor& d = mj::reflect::Describe(t);
  for (int i = 0; i < static_cast<int>(d.field_count); ++i)
    if (d.fields[i].name == name) return i;
  return -1;
}

// The member element types of a union, straight from the generated union
// descriptor. The single source of truth for a union's spellings is the schema
// (reflect::DescribeUnion); nothing here re-lists them.
inline std::vector<mj::ElementType> UnionMemberTypes(std::string_view name) {
  const mj::reflect::UnionDescriptor& u = mj::reflect::DescribeUnion(name);
  return {u.members, u.members + u.member_count};
}

inline bool Contains(const std::vector<mj::ElementType>& v, mj::ElementType t) {
  for (auto x : v)
    if (x == t) return true;
  return false;
}

// --- Per-field probe by id ------------------------------------------------ //
// Reads the field with a given id out of an element, matched by expected type
// U. Field ids are per-element and identical across two instances of the same
// type (same Visit order), so this recovers "the same field" on a sibling or
// clone. Only `field` callbacks participate (child ids are a separate namespace
// and are ignored).

template <class U>
struct ConstFieldGrab {
  int target;
  const U* out = nullptr;
  template <class W>
  void field(int id, const char*, const W& v) {
    if (id == target) {
      if constexpr (std::is_same_v<W, U>) out = &v;
    }
  }
  template <class C>
  void child(int, const char*, const C&) {}
  template <class C>
  void union_child(int, const char*, const C&) {}
};

template <class U>
struct MutFieldGrab {
  int target;
  U* out = nullptr;
  template <class W>
  void field(int id, const char*, W& v) {
    if (id == target) {
      if constexpr (std::is_same_v<W, U>) out = &v;
    }
  }
  template <class C>
  void child(int, const char*, C&) {}
  template <class C>
  void union_child(int, const char*, C&) {}
};

template <class P, class T, class U>
const U* FieldAt(const T& e, int id) {
  ConstFieldGrab<U> g{id};
  P::Visit(e, g);
  return g.out;
}
template <class P, class T, class U>
U* FieldAt(T& e, int id) {
  MutFieldGrab<U> g{id};
  P::Visit(e, g);
  return g.out;
}

// --- Shape-aware field access by id --------------------------------------- //
// The typed probe above needs the caller to name the storage type, which only
// the plain profile's own code can do. These reach a field by its schema id and
// route the value through P::Shape, so a caller states WHAT it wants to write
// (three doubles, a keyword index, a name) and never HOW the profile stores it.

namespace field_detail {

template <class Fn>
struct ApplyMut {
  int target;
  Fn* fn;
  bool hit = false;
  template <class W>
  void field(int id, const char*, W& v) {
    if (id != target || hit) return;
    if constexpr (!std::is_const_v<W>) {
      hit = (*fn)(v);
    }
  }
  template <class C>
  void child(int, const char*, C&) {}
  template <class C>
  void union_child(int, const char*, C&) {}
};

template <class Fn>
struct ApplyConst {
  int target;
  Fn* fn;
  bool hit = false;
  template <class W>
  void field(int id, const char*, const W& v) {
    if (id != target || hit) return;
    hit = (*fn)(v);
  }
  template <class C>
  void child(int, const char*, const C&) {}
  template <class C>
  void union_child(int, const char*, const C&) {}
};

}  // namespace field_detail

// Invoke `fn(slot)` on the storage slot of field `field_id`; `fn` returns true
// when it recognized the slot. Reports whether it fired.
template <class P, class E, class Fn>
bool ApplyToField(E& e, int field_id, Fn&& fn) {
  if (field_id < 0) return false;
  field_detail::ApplyMut<std::remove_reference_t<Fn>> v{field_id, &fn};
  P::Visit(e, v);
  return v.hit;
}

template <class P, class E, class Fn>
bool ReadField(const E& e, int field_id, Fn&& fn) {
  if (field_id < 0) return false;
  field_detail::ApplyConst<std::remove_reference_t<Fn>> v{field_id, &fn};
  P::Visit(e, v);
  return v.hit;
}

// Author a string-valued field from UTF-8 text.
template <class P, class E>
bool SetStrField(E& e, int field_id, std::string_view text) {
  return ApplyToField<P>(e, field_id, [&](auto& slot) {
    using S = typename P::Shape;
    using I = typename S::template inner_t<std::decay_t<decltype(slot)>>;
    if constexpr (S::template kind_v<I> == Shape::Str) {
      S::Author(slot, S::template StrMake<I>(text));
      return true;
    } else {
      return false;
    }
  });
}

// Author an arithmetic or boolean field.
template <class P, class Num, class E>
bool SetScalarField(E& e, int field_id, Num value) {
  return ApplyToField<P>(e, field_id, [&](auto& slot) {
    using S = typename P::Shape;
    using I = typename S::template inner_t<std::decay_t<decltype(slot)>>;
    if constexpr (S::template kind_v<I> == Shape::Scalar ||
                  S::template kind_v<I> == Shape::Bool) {
      S::Author(slot, static_cast<I>(value));
      return true;
    } else {
      return false;
    }
  });
}

// Author an enum field from its declaration index -- the schema's keyword order,
// which every profile's emission of that enum shares.
template <class P, class E>
bool SetEnumField(E& e, int field_id, int keyword_index) {
  return ApplyToField<P>(e, field_id, [&](auto& slot) {
    using S = typename P::Shape;
    using I = typename S::template inner_t<std::decay_t<decltype(slot)>>;
    if constexpr (S::template kind_v<I> == Shape::Enum) {
      S::Author(slot, S::template EnumMake<I>(keyword_index));
      return true;
    } else {
      return false;
    }
  });
}

// The declaration index of an enum field's authored value, or `fallback`.
template <class P, class E>
int GetEnumField(const E& e, int field_id, int fallback) {
  int out = fallback;
  ReadField<P>(e, field_id, [&](const auto& slot) {
    using S = typename P::Shape;
    using I = typename S::template inner_t<std::decay_t<decltype(slot)>>;
    if constexpr (S::template kind_v<I> == Shape::Enum) {
      if (S::IsSet(slot)) out = S::EnumIndex(S::Read(slot));
      return true;
    } else {
      return false;
    }
  });
  return out;
}

// Author a fixed-arity field from exactly N scalars, in MJCF component order.
template <class P, std::size_t N, class E, class S0>
bool SetFixedField(E& e, int field_id, const S0 (&values)[N]) {
  return ApplyToField<P>(e, field_id, [&](auto& slot) {
    using S = typename P::Shape;
    using I = typename S::template inner_t<std::decay_t<decltype(slot)>>;
    if constexpr (S::template kind_v<I> == Shape::Fixed) {
      using F = typename S::template fixed<I>;
      if constexpr (F::size == N) {
        typename F::scalar buf[N]{};
        for (std::size_t i = 0; i < N; ++i)
          buf[i] = static_cast<typename F::scalar>(values[i]);
        S::Author(slot, F::Make(buf));
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  });
}

// Author a sequence field (range or unbounded arity) from `n` scalars.
template <class P, class E, class S0>
bool SetSeqField(E& e, int field_id, const S0* values, std::size_t n) {
  return ApplyToField<P>(e, field_id, [&](auto& slot) {
    using S = typename P::Shape;
    using I = typename S::template inner_t<std::decay_t<decltype(slot)>>;
    if constexpr (S::template kind_v<I> == Shape::Range ||
                  S::template kind_v<I> == Shape::Unbounded) {
      using Q = typename S::template seq<I>;
      using Sc = typename Q::scalar;
      if constexpr (std::is_arithmetic_v<Sc>) {
        std::vector<Sc> buf(n);
        for (std::size_t i = 0; i < n; ++i) buf[i] = static_cast<Sc>(values[i]);
        S::Author(slot, Q::Make(buf.data(), n));
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  });
}

// Author an enum field from its MJCF keyword. False when the field is not an
// enum or the keyword is not one of its spellings.
template <class P, class E>
bool SetEnumFieldByKeyword(E& e, int field_id, std::string_view keyword) {
  bool ok = false;
  ApplyToField<P>(e, field_id, [&](auto& slot) {
    using S = typename P::Shape;
    using I = typename S::template inner_t<std::decay_t<decltype(slot)>>;
    if constexpr (S::template kind_v<I> == Shape::Enum) {
      I v{};
      if (S::template EnumFromMjcf<I>(keyword, v)) {
        S::Author(slot, v);
        ok = true;
      }
      return true;
    } else {
      return false;
    }
  });
  return ok;
}

// The authored text of a string field, as UTF-8. False (and `out` untouched)
// when the field is not a string or is unauthored.
template <class P, class E>
bool GetStrField(const E& e, int field_id, std::string& out) {
  bool got = false;
  ReadField<P>(e, field_id, [&](const auto& slot) {
    using S = typename P::Shape;
    using I = typename S::template inner_t<std::decay_t<decltype(slot)>>;
    if constexpr (S::template kind_v<I> == Shape::Str) {
      if (S::IsSet(slot)) {
        out = std::string(S::template StrGet<I>(S::Read(slot)));
        got = true;
      }
      return true;
    } else {
      return false;
    }
  });
  return got;
}

// True when the field at `field_id` is authored.
template <class P, class E>
bool IsFieldSet(const E& e, int field_id) {
  bool set = false;
  ReadField<P>(e, field_id, [&](const auto& slot) {
    set = P::Shape::IsSet(slot);
    return true;
  });
  return set;
}

// Make a field unauthored. A required field has no unauthored state and is left
// alone.
template <class P, class E>
void ClearField(E& e, int field_id) {
  ApplyToField<P>(e, field_id, [&](auto& slot) {
    P::Shape::Reset(slot);
    return true;
  });
}

// --- Tree construction ---------------------------------------------------- //

// Construct a T and link it into `parent` at `index` among the siblings that
// share its storage (kAppend == last). Construction and linkage are one step:
// a component profile has no meaningful detached node, and the plain profile
// loses nothing by fusing them.
template <class P, class T, class Parent>
T& Create(Parent& parent, std::size_t index = kAppend) {
  return P::Tree::template Adopt<T>(parent, index,
                                    P::Ident::template New<T>());
}

// The document's single section node of element type `Section`, created (empty,
// first) if absent. Backs World / EnsureAsset / EnsureActuatorSection / ...
template <class P, class Section, class D>
Section& EnsureSection(D& doc) {
  if (Section* s = P::Tree::template FirstChildOfType<Section>(doc)) return *s;
  return Create<P, Section>(doc, 0);
}

// The reader's construction seam: create the document root, and create a child
// under a parent at a given position. Construction and linkage are one call
// because a component profile cannot express a detached node, and the position
// is required because two reader sites insert other than at the end.
//
// A profile that needs several construction strategies over one storage model
// (an editor template graph versus a live instance tree) supplies its own
// factory instead of this one; the reader takes it as a separate parameter.
template <class P>
struct DefaultNodeFactory {
  static OwnerOf<P, DocOf<P>> CreateRoot() {
    return P::Ident::template New<DocOf<P>>();
  }
  template <class E, class Parent>
  static E& Create(Parent& parent, std::size_t index = kAppend) {
    return internal::Create<P, E>(parent, index);
  }
};

// --- Whole-tree walk ------------------------------------------------------ //
// Visits `root`, then every element nested beneath it in document order. `fn` is
// a callable accepting `auto& element` (its constness follows the walked
// object's). Only children descend, which is exactly the set of nested elements
// (a nested element never lives in a plain field).

template <class P, class E, class Fn>
void WalkTree(E& root, Fn&& fn) {
  fn(root);
  P::Tree::ForEachChild(root, [&](auto& c) { WalkTree<P>(c, fn); });
}

// Walk every "live" element of a document -- every top-level section except the
// <default> tree. Class-defining elements live only under `defaults` and are
// authoring templates, not model content; operations that act on real elements
// (flatten, find-by-name outside classes, referrer scans against live refs) use
// this to skip them. `defaults` is walked explicitly where a query needs the
// class tree itself.
template <class P, class D, class Fn>
void WalkModelLive(D& doc, Fn&& fn) {
  P::Doc::ForEachLiveSection(doc, [&](auto& section) { WalkTree<P>(section, fn); });
}

// Walk every element of a document, the root and the defaults included.
template <class P, class D, class Fn>
void WalkModelAll(D& doc, Fn&& fn) {
  WalkTree<P>(doc, std::forward<Fn>(fn));
}

// --- Name access ---------------------------------------------------------- //
// An element's authored name, whatever the profile calls the slot it lives in
// (the plain profile's `opt<string> name`, a required plain `string`, a
// Default's `dclass`). `nullopt` means nameless-or-unset: the two cases a caller
// must not distinguish. `has_name_v` answers the separate, type-level question
// "can this element type be named at all", which is what a UI needs to decide
// whether to offer a name field.

template <class P, class E>
std::optional<ViewOf<P>> NameOf(const E& e) {
  return P::Name(e);
}

template <class P, class E>
inline constexpr bool has_name_v = P::template has_name<std::decay_t<E>>;

template <class P, class E>
bool HasNameField() {
  return has_name_v<P, E>;
}

// Set an element's name. A no-op when the element type has no name.
template <class P, class E>
void SetName(E& e, ViewOf<P> name) {
  P::SetName(e, name);
}

// --- Reference prefixing -------------------------------------------------- //
// Prefix every typed reference name in one element (names handled separately).
// Cross-model namespacing (mjs_attach mirror) prefixes an entire cloned subtree;
// the compiler prefixes attach-flattened content the same way.
template <class P, class E>
void PrefixRefs(E& e, ViewOf<P> prefix) {
  P::Ref::ScanTyped(e, [&](int, const char*, auto&& slot, const auto&) {
    StrOf<P> prefixed = P::Str::Concat(prefix, slot.Get());
    slot.Set(P::Str::View(prefixed));
  });
}

}  // namespace ps::sdk::internal

#endif  // PROTOSPEC_SDK_MODEL_CORE_H
