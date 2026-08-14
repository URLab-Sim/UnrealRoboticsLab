// ProtoSpec SDK: the upward index over a document.
//
// A parent lookup and path index, built once from a document and queried many
// times. It is separate from the downward traversal verbs because the default-
// class query (classes.h) needs upward lookup and nothing else from traversal:
// keeping it here lets a consumer of the class layering avoid pulling in the
// find/walk/rename machinery it never calls.
#ifndef PROTOSPEC_SDK_PARENTS_H
#define PROTOSPEC_SDK_PARENTS_H

#include <optional>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include "protospec/detail.h"
#include "protospec/plain_profile.h"
#include "protospec/profile.h"
#include "reflect.h"

namespace ps::sdk {

namespace mj = ps::mjcf;

namespace parents_detail {
// True when the profile's Tree offers the erased base-pointer subtree walk
// (ForEachDescendant + DispatchConcrete). A profile with a common node base --
// the UE component tree -- sets Tree::kErasedParentWalk to route ParentMap onto
// a build that instantiates the per-node record step once (behind a single
// dispatch) instead of over the whole containment closure. The plain profile
// has no such base and keeps the generic recursive build.
template <class TR, class = void>
struct HasErasedWalk : std::false_type {};
template <class TR>
struct HasErasedWalk<TR, std::void_t<decltype(TR::kErasedParentWalk)>>
    : std::bool_constant<TR::kErasedParentWalk> {};
}  // namespace parents_detail

// A parent lookup + path index over a document, built once and queried many
// times. Construction is a single whole-tree walk; every element (including
// class elements under <default>) gets an entry. Element identity is the
// profile's node pointer. The map is a snapshot: rebuild it after structural
// edits.
//
// A profile whose parenthood is directly queryable (a component tree's attach
// parent) can supply its own type through `P::Doc::parent_map` instead, which
// removes the staleness footgun rather than reproducing it.
template <class P = plain::Plain>
class ParentMap {
 public:
  struct Node {
    const void* parent = nullptr;  // owning element, null for the root
    mj::ElementType type{};        // this element's type
    StrOf<P> name;                 // authored name (or dclass), else ""
    StrOf<P> childclass;           // body-context childclass, else ""
  };

  explicit ParentMap(const DocOf<P>& model) {
    const void* root = &model;
    Node n;
    n.parent = nullptr;
    n.type = ElementTypeOf<P, DocOf<P>>;
    if (std::optional<ViewOf<P>> nm = P::Doc::Name(model))
      n.name = P::Str::FromUtf8(P::Str::ToUtf8(*nm));
    nodes_[root] = std::move(n);
    if constexpr (parents_detail::HasErasedWalk<typename P::Tree>::value) {
      // Walk the subtree over base node pointers (one instantiation) and record
      // each node through a single dispatch to its concrete type -- instead of
      // the recursive generic walk below, which instantiates the record step for
      // the entire containment closure and costs ~20 GB of compiler memory.
      P::Tree::ForEachDescendant(model, [this](const auto& child, const void* parent) {
        P::Tree::DispatchConcrete(
            child, [this, parent](const auto& e) { this->AddNode(e, parent); });
      });
    } else {
      Record(model, root);
    }
  }

  // Parent of an element, or null when it is the document root or not indexed.
  template <class E>
  const void* ParentOf(const E& e) const {
    auto it = nodes_.find(&e);
    return it == nodes_.end() ? nullptr : it->second.parent;
  }

  const Node* Lookup(const void* ptr) const {
    auto it = nodes_.find(ptr);
    return it == nodes_.end() ? nullptr : &it->second;
  }

  // A "/"-joined path from the root to the element, each step the element's XML
  // tag plus its name in brackets when it has one (e.g.
  // "mujoco/worldbody/body[torso]/geom[shin]"). For diagnostics.
  template <class E>
  std::string PathTo(const E& e) const {
    return PathToPtr(&e);
  }

  std::string PathToPtr(const void* ptr) const {
    std::vector<const Node*> chain;
    for (const void* p = ptr; p;) {
      const Node* n = Lookup(p);
      if (!n) break;
      chain.push_back(n);
      p = n->parent;
    }
    std::string out;
    for (auto it = chain.rbegin(); it != chain.rend(); ++it) {
      if (!out.empty()) out += '/';
      out += mj::reflect::Describe((*it)->type).xml;
      const std::string nm = P::Str::ToUtf8(P::Str::View((*it)->name));
      if (!nm.empty()) {
        out += '[';
        out += nm;
        out += ']';
      }
    }
    return out;
  }

 private:
  // Record children of `e` (whose handle is `self`) as pointing back at `self`,
  // then recurse.
  template <class E>
  void Record(const E& e, const void* self) {
    P::Tree::ForEachChild(e, [&](const auto& c) { Add(c, self); });
  }

  // Record one element's entry. No recursion: the erased walk visits every
  // descendant itself, and the generic Add below adds the recursion back.
  template <class E>
  void AddNode(const E& e, const void* parent) {
    Node n;
    n.parent = parent;
    n.type = ElementTypeOf<P, E>;
    if (std::optional<ViewOf<P>> nm = detail::NameOf<P>(e))
      n.name = P::Str::FromUtf8(P::Str::ToUtf8(*nm));
    n.childclass = ChildClass(e);
    nodes_[&e] = std::move(n);
  }

  template <class E>
  void Add(const E& e, const void* parent) {
    AddNode(e, parent);
    Record(e, &e);
  }

  // childclass propagates defaults down the body tree; only the three
  // body-context containers carry it. Reached by schema field name, so no
  // element type is spelled here.
  template <class E>
  static StrOf<P> ChildClass(const E& e) {
    const mj::ElementType t = ElementTypeOf<P, E>;
    if (t != mj::ElementType::Body && t != mj::ElementType::Frame &&
        t != mj::ElementType::Replicate) {
      return StrOf<P>();
    }
    const int id = detail::FieldIdByName(t, "childclass");
    if (id < 0) return StrOf<P>();
    return P::Str::FromUtf8(P::Str::ToUtf8(P::Ref::NameAt(e, id)));
  }

  std::unordered_map<const void*, Node> nodes_;
};

}  // namespace ps::sdk

#endif  // PROTOSPEC_SDK_PARENTS_H
