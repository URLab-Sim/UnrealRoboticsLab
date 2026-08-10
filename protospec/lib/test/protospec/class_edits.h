// ProtoSpec SDK: the mutating default-class transforms.
//
// Unlike Effective (a pure query returning a computed copy, protospec/classes.h),
// FlattenDefaults and ExtractClass MUTATE the document. They are authoring
// operations, not part of any compile: FlattenDefaults bakes effective values
// into elements and drops the class tree; ExtractClass factors shared authored
// values out of a set of elements into a new class. Neither is reversible in
// place; clone the document first if you need the original.
#ifndef PROTOSPEC_SDK_CLASS_EDITS_H
#define PROTOSPEC_SDK_CLASS_EDITS_H

#include <string>
#include <type_traits>
#include <vector>

#include "protospec/classes.h"
#include "protospec/parents.h"
#include "protospec/refs.h"

namespace ps::sdk {

// --- FlattenDefaults (mutating) ------------------------------------------- //

// Bake each element's class-layered values into the element and drop the whole
// <default> tree. Only the authored class layers are baked (not the IDL
// defaults), so unauthored-everywhere fields stay unset and still resolve to
// MuJoCo's compiler defaults at compile. After this call the document has no
// classes and no class/childclass references, and compiles identically.
template <class P = plain::Plain>
void FlattenDefaults(DocOf<P>& model) {
  ParentMap<P> pm(model);
  detail::DefaultIndex<P> idx(model);

  detail::WalkModelLive<P>(model, [&](auto& e) {
    using E = std::decay_t<decltype(e)>;
    if constexpr (has_default_family_v<P, E>) {
      ViewOf<P> cls =
          detail::ResolveClassName<P>(pm, detail::OwnClass<P, E>(e), &e);
      detail::MergeClassChain<P>(idx, cls, e);
      const int id = detail::FieldIdByName(ElementTypeOf<P, E>, "dclass");
      if (id >= 0) ClearRefByField<P>(e, id);
    }
  });

  detail::WalkModelLive<P>(model, [&](auto& e) {
    using E = std::decay_t<decltype(e)>;
    // The three body-context containers carry the inherited class as
    // `childclass`; flattening resolves it away.
    const mj::ElementType t = ElementTypeOf<P, E>;
    if (t == mj::ElementType::Body || t == mj::ElementType::Frame ||
        t == mj::ElementType::Replicate) {
      const int id = detail::FieldIdByName(t, "childclass");
      if (id >= 0) ClearRefByField<P>(e, id);
    }
  });

  P::Tree::template ClearChildrenOfType<detail::DefaultOf<P>>(model);
}

// --- ExtractClass (mutating) ---------------------------------------------- //

namespace detail {

// For each field authored identically across every element, move that value
// into the class element and clear it on each source element. Identity fields
// are exempt: `name` names the element (referrers depend on it) and `dclass` is
// the class link itself (rewritten by ExtractClass to point at the new class) --
// neither may migrate into the class partial, where MJCF forbids them.
template <class P, class T>
struct ExtractVisitor {
  const std::vector<T*>* elems;
  template <class U>
  void field(int id, const char* fname, U& clsField) {
    if constexpr (P::Shape::template optional_v<U>) {
      const std::string_view f(fname);
      if (f == "name" || f == "dclass") return;
      const U* first = nullptr;
      bool all_equal = !elems->empty();
      for (T* e : *elems) {
        const U* v = FieldAt<P, T, U>(*e, id);
        if (!v || !P::Shape::IsSet(*v)) {
          all_equal = false;
          break;
        }
        if (!first)
          first = v;
        else if (!(*v == *first)) {
          all_equal = false;
          break;
        }
      }
      if (all_equal && first) {
        clsField = *first;
        for (T* e : *elems) {
          if (U* v = FieldAt<P, T, U>(*e, id)) P::Shape::Reset(*v);
        }
      }
    }
  }
  template <class C>
  void child(int, const char*, C&) {}
  template <class C>
  void union_child(int, const char*, C&) {}
};

}  // namespace detail

// Factor the fields shared (authored and equal) across `elems` into a new class
// `name`, clear those fields on each element, and point each element at the new
// class. The class is added under the root `main` default (created if absent).
// All elements must be the same defaultable family type. Returns the new class,
// or nullptr when `elems` is empty or the family has no class partial.
template <class P = plain::Plain, class T>
detail::DefaultOf<P>* ExtractClass(DocOf<P>& model, const std::vector<T*>& elems,
                                   ViewOf<P> name) {
  if (elems.empty()) return nullptr;
  if constexpr (!has_default_family_v<P, T>) {
    return nullptr;
  } else {
    using DefaultT = detail::DefaultOf<P>;
    DefaultT& root = detail::EnsureRoot<P>(model);
    DefaultT& cls = detail::Create<P, DefaultT>(root);
    detail::SetName<P>(cls, name);
    T& clsElem = detail::Create<P, T>(cls);

    detail::ExtractVisitor<P, T> v{&elems};
    P::Visit(clsElem, v);

    const int dclass_id = detail::FieldIdByName(ElementTypeOf<P, T>, "dclass");
    for (T* e : elems)
      if (dclass_id >= 0) SetRefByField<P>(*e, dclass_id, name);
    return &cls;
  }
}

}  // namespace ps::sdk

#endif  // PROTOSPEC_SDK_CLASS_EDITS_H
