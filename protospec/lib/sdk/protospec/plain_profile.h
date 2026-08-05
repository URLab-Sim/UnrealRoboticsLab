// ProtoSpec: the plain emission profile.
//
// The reference profile, and the one the CLI tools, the Python bindings and the
// differential harness run on: a document is a `ps::mjcf::Model` of owned values
// -- presence-tracked `ps::opt<T>` attributes, `ps::Ref<T>` names, child lists of
// `std::unique_ptr` (plus ordered union wrappers where the schema interleaves
// several spellings into one list).
//
// Every policy here is a thin shim over that storage. Nothing in this file makes
// a decision; the decisions are in profile.h, and this is what they cost when the
// storage is already exactly what the algorithms want.
#ifndef PROTOSPEC_SDK_PLAIN_PROFILE_H
#define PROTOSPEC_SDK_PLAIN_PROFILE_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

#include "protospec/core.h"
#include "protospec/profile.h"
#include "defaults.h"
#include "keywords.h"
#include "reflect.h"
#include "types.h"
#include "visit.h"

namespace ps::sdk::plain {

namespace mj = ps::mjcf;

struct Plain;

// --- Storage-shape matchers ----------------------------------------------- //
// The plain profile's private pattern matches on its own storage types. Nothing
// outside this file may match on them: that is the coupling Phase 2 removes.

template <class T>
struct is_opt : std::false_type {};
template <class T>
struct is_opt<ps::opt<T>> : std::true_type {
  using inner = T;
};

template <class T>
struct is_ref : std::false_type {};
template <class T>
struct is_ref<ps::Ref<T>> : std::true_type {
  using target = T;
};

template <class T>
struct is_std_array : std::false_type {};
template <class T, std::size_t N>
struct is_std_array<std::array<T, N>> : std::true_type {};

template <class T>
struct is_inline_vec : std::false_type {};
template <class T, std::size_t N>
struct is_inline_vec<ps::InlineVec<T, N>> : std::true_type {};

template <class T>
struct is_vector : std::false_type {};
template <class T>
struct is_vector<std::vector<T>> : std::true_type {};

// A single typed cross reference (DR-8), in either storage: `opt<Ref<T>>` for an
// optional attribute, a bare `Ref<T>` for one the schema marks required. Both
// must be seen by everything that scans references -- a required reference is
// still a reference, and missing it would silently exclude it from the referrer
// scan, the rename fixup and the dangling report.
template <class U>
struct opt_ref {
  static constexpr bool value = false;
};
template <class T>
struct opt_ref<ps::opt<ps::Ref<T>>> {
  static constexpr bool value = true;
  static constexpr bool optional = true;
  using target = T;
};
template <class T>
struct opt_ref<ps::Ref<T>> {
  static constexpr bool value = true;
  static constexpr bool optional = false;
  using target = T;
};

// As opt_ref, for a reference LIST (`ref<T>[]`, one space-separated attribute of
// names, e.g. <flex body="b1 b2">).
template <class U>
struct opt_ref_list {
  static constexpr bool value = false;
};
template <class T>
struct opt_ref_list<ps::opt<std::vector<ps::Ref<T>>>> {
  static constexpr bool value = true;
  static constexpr bool optional = true;
  using target = T;
};
template <class T>
struct opt_ref_list<std::vector<ps::Ref<T>>> {
  static constexpr bool value = true;
  static constexpr bool optional = false;
  using target = T;
};

// A union element type stores its alternatives in a
// `std::variant<std::unique_ptr<Members>...> node`; a concrete element type has
// no such member. `type` is that variant, or void for a concrete element.
template <class T, class = void>
struct union_node {
  using type = void;
};
template <class T>
struct union_node<T, std::void_t<decltype(std::declval<T&>().node)>> {
  using type = std::decay_t<decltype(std::declval<T&>().node)>;
};

template <class T, class Var>
struct variant_holds_ptr : std::false_type {};
template <class T, class... Ts>
struct variant_holds_ptr<T, std::variant<Ts...>>
    : std::disjunction<std::is_same<std::unique_ptr<T>, Ts>...> {};

template <class Var>
struct variant_member_types;
template <class... Ts>
struct variant_member_types<std::variant<Ts...>> {
  static std::vector<mj::ElementType> value() {
    return {mj::element_type_of<typename Ts::element_type>::value...};
  }
};

// The element types a `Ref<Target>` can name: the one type for a concrete
// target, or every member of the union for a namespace with several declarers
// (Ref<JointAny> names <joint> or <freejoint>). Read off the generated union's
// own variant, so a schema that grows or merges a namespace needs no edit here.
template <class Target>
inline std::vector<mj::ElementType> RefTargetTypes() {
  using Node = typename union_node<Target>::type;
  if constexpr (std::is_void_v<Node>) {
    return {mj::element_type_of<Target>::value};
  } else {
    return variant_member_types<Node>::value();
  }
}

// True when element type E is a valid target of a `Ref<T>`: E == T for a
// concrete target, or E a member of the union T (Ref<TendonAny> accepts
// Spatial/Fixed; Ref<ActuatorAny> accepts every actuator spelling). Consumed by
// the plain SetRef's compile-time target check, so a target of the wrong element
// type is a hard error rather than a silent mismatch.
template <class T, class E>
constexpr bool RefAcceptsTargetImpl() {
  using Node = typename union_node<T>::type;
  if constexpr (std::is_void_v<Node>)
    return std::is_same_v<E, T>;
  else
    return variant_holds_ptr<E, Node>::value;
}
template <class T, class E>
inline constexpr bool ref_accepts_target = RefAcceptsTargetImpl<T, E>();

// --- Ident ---------------------------------------------------------------- //

struct PlainIdent {
  using serial_t = std::uint64_t;

  template <class E>
  static constexpr bool has_serial = requires(const E& e) { e.serial; };

  template <class E>
  static serial_t Serial(const E& e) {
    if constexpr (has_serial<E>) {
      return e.serial;
    } else {
      (void)e;
      return 0;
    }
  }
  template <class E>
  static void SetSerial(E& e, serial_t s) {
    if constexpr (has_serial<E>) e.serial = s;
  }

  template <class E>
  static ps::SourceLoc Loc(const E& e) {
    return e.loc;
  }
  template <class E>
  static void SetLoc(E& e, ps::SourceLoc loc) {
    e.loc = std::move(loc);
  }

  template <class E>
  static std::unique_ptr<E> New() {
    return std::make_unique<E>();
  }

  // The generated deep clone. Its ordering guarantee -- the clone reproduces the
  // source's document order under ForEachChild exactly -- is what
  // CloneModelWithSerials's walk-bijection assert enforces.
  template <class E>
  static std::unique_ptr<E> Clone(const E& e) {
    return mj::Clone(e);
  }
};

// --- Tree ----------------------------------------------------------------- //

namespace tree_detail {

template <class Fn>
struct ChildIter {
  Fn* fn;
  template <class U>
  void field(int, const char*, U&) {}
  template <class C>
  void child(int, const char*, C& list) {
    for (auto& p : list)
      if (p) (*fn)(*p);
  }
  template <class C>
  void union_child(int, const char*, C& list) {
    for (auto& item : list)
      std::visit(
          [&](auto& p) {
            if (p) (*fn)(*p);
          },
          item.node);
  }
};

// The element type an owned-child list holds.
template <class C>
using owned_elem_t = typename C::value_type::element_type;

// Every child of `parent`, in document order, tagged with the id of the storage
// slot that holds it (the generated Visit child id, which is also the binding's
// child index).
template <class Fn>
struct SlottedChildIter {
  Fn* fn;
  template <class U>
  void field(int, const char*, U&) {}
  template <class C>
  void child(int cid, const char*, C& list) {
    for (auto& p : list)
      if (p) (*fn)(cid, *p);
  }
  template <class C>
  void union_child(int cid, const char*, C& list) {
    for (auto& item : list)
      std::visit(
          [&](auto& p) {
            if (p) (*fn)(cid, *p);
          },
          item.node);
  }
};

// The element types each storage slot of `parent` admits: one per owned child
// list, one per member of a union child list. Type-level, so the reader can
// construct a child of the right type without a 145-way runtime dispatch.
template <class Var, class Fn, std::size_t... Is>
void EachUnionAlt(int cid, Fn* fn, std::index_sequence<Is...>) {
  ((*fn)(cid,
         TypeTag<typename std::variant_alternative_t<Is, Var>::element_type>{}),
   ...);
}

template <class Fn>
struct SlotIter {
  Fn* fn;
  template <class U>
  void field(int, const char*, U&) {}
  template <class C>
  void child(int cid, const char*, C&) {
    (*fn)(cid, TypeTag<std::remove_const_t<
                   owned_elem_t<std::remove_const_t<C>>>>{});
  }
  template <class C>
  void union_child(int cid, const char*, C&) {
    using U = std::remove_const_t<typename std::remove_const_t<C>::value_type>;
    using Var = typename union_node<U>::type;
    EachUnionAlt<Var>(cid, fn,
                      std::make_index_sequence<std::variant_size_v<Var>>{});
  }
};

template <class T, class Fn>
struct TypedChildIter {
  Fn* fn;
  template <class U>
  void field(int, const char*, U&) {}
  template <class C>
  void child(int, const char*, C& list) {
    using X = std::remove_const_t<owned_elem_t<std::remove_const_t<C>>>;
    if constexpr (std::is_same_v<X, T>) {
      for (auto& p : list)
        if (p) (*fn)(*p);
    }
  }
  template <class C>
  void union_child(int, const char*, C& list) {
    using U = std::remove_const_t<typename std::remove_const_t<C>::value_type>;
    using Var = typename union_node<U>::type;
    if constexpr (variant_holds_ptr<T, Var>::value) {
      for (auto& item : list)
        std::visit(
            [&](auto& p) {
              using X = std::remove_const_t<
                  typename std::decay_t<decltype(p)>::element_type>;
              if constexpr (std::is_same_v<X, T>) {
                if (p) (*fn)(*p);
              }
            },
            item.node);
    }
  }
};

template <class T>
struct ChildClearer {
  template <class U>
  void field(int, const char*, U&) {}
  template <class C>
  void child(int, const char*, C& list) {
    if constexpr (std::is_same_v<std::remove_const_t<owned_elem_t<C>>, T>)
      list.clear();
  }
  template <class C>
  void union_child(int, const char*, C& list) {
    using U = typename C::value_type;
    using Var = typename union_node<U>::type;
    if constexpr (variant_holds_ptr<T, Var>::value) {
      std::erase_if(list, [](const U& item) {
        return std::holds_alternative<std::unique_ptr<T>>(item.node);
      });
    }
  }
};

// Insert an already-owned child into whichever of the parent's lists holds a T,
// at `index` within that list (kAppend == push_back). The first matching list
// wins, which is deterministic: no element declares two child lists that can
// both hold the same element type.
template <class T>
struct Adopter {
  std::unique_ptr<T> owned;
  std::size_t index;
  T* out = nullptr;

  template <class U>
  void field(int, const char*, U&) {}

  template <class C>
  void child(int, const char*, C& list) {
    if (out != nullptr || !owned) return;
    if constexpr (std::is_same_v<std::remove_const_t<owned_elem_t<C>>, T>) {
      out = owned.get();
      const std::size_t at = index >= list.size() ? list.size() : index;
      list.insert(list.begin() + static_cast<std::ptrdiff_t>(at),
                  std::move(owned));
    }
  }

  template <class C>
  void union_child(int, const char*, C& list) {
    if (out != nullptr || !owned) return;
    using U = typename C::value_type;
    using Var = typename union_node<U>::type;
    if constexpr (variant_holds_ptr<T, Var>::value) {
      out = owned.get();
      U item;
      item.node = std::move(owned);
      const std::size_t at = index >= list.size() ? list.size() : index;
      list.insert(list.begin() + static_cast<std::ptrdiff_t>(at),
                  std::move(item));
    }
  }
};

// `T` may be const-qualified; the match is on the unqualified element type and
// the result carries the walked object's constness.
template <class T>
struct FirstOf {
  using Bare = std::remove_const_t<T>;
  T* out = nullptr;
  template <class U>
  void field(int, const char*, U&) {}
  template <class C>
  void child(int, const char*, C& list) {
    if (out != nullptr) return;
    using X =
        std::remove_const_t<owned_elem_t<std::remove_const_t<C>>>;
    if constexpr (std::is_same_v<X, Bare>) {
      for (auto& p : list)
        if (p) {
          out = &*p;
          return;
        }
    }
  }
  template <class C>
  void union_child(int, const char*, C& list) {
    if (out != nullptr) return;
    using U = std::remove_const_t<typename std::remove_const_t<C>::value_type>;
    using Var = typename union_node<U>::type;
    if constexpr (variant_holds_ptr<Bare, Var>::value) {
      for (auto& item : list) {
        if (auto* p = std::get_if<std::unique_ptr<Bare>>(&item.node)) {
          if (*p) {
            out = &**p;
            return;
          }
        }
      }
    }
  }
};

// Remove the element identified by `target` from whichever child or union-child
// list owns it. Stops at the first match.
struct Remover {
  const void* target;
  bool* done;
  template <class U>
  void field(int, const char*, U&) {}
  template <class C>
  void child(int, const char*, C& list) {
    if (*done) return;
    for (auto it = list.begin(); it != list.end(); ++it) {
      if (it->get() == target) {
        list.erase(it);
        *done = true;
        return;
      }
    }
    for (auto& p : list) {
      if (*done) return;
      if (p) {
        Remover r{target, done};
        mj::Visit(*p, r);
      }
    }
  }
  template <class C>
  void union_child(int, const char*, C& list) {
    if (*done) return;
    for (auto it = list.begin(); it != list.end(); ++it) {
      bool match = false;
      std::visit(
          [&](auto& p) {
            if (p && static_cast<const void*>(p.get()) == target) match = true;
          },
          it->node);
      if (match) {
        list.erase(it);
        *done = true;
        return;
      }
    }
    for (auto& item : list) {
      if (*done) return;
      std::visit(
          [&](auto& p) {
            if (p) {
              Remover r{target, done};
              mj::Visit(*p, r);
            }
          },
          item.node);
    }
  }
};

// Insert a fresh-serial deep clone of the element at `target` immediately after
// it in its owning child/union list, and report the clone's (stable heap)
// address. The concrete element type is recovered inside the generated Visit
// hook, so mj::Clone is the only per-type code -- no whole-model walk is
// instantiated per element type.
struct Duplicator {
  const void* target;
  void** clone_out;
  bool* done;

  template <class U>
  void field(int, const char*, U&) {}

  template <class C>
  void child(int, const char*, C& list) {
    if (*done) return;
    for (std::size_t i = 0; i < list.size(); ++i) {
      if (list[i] && static_cast<const void*>(list[i].get()) == target) {
        auto clone = mj::Clone(*list[i]);
        *clone_out = clone.get();
        list.insert(list.begin() + static_cast<std::ptrdiff_t>(i) + 1,
                    std::move(clone));
        *done = true;
        return;
      }
    }
    for (auto& p : list) {
      if (*done) return;
      if (p) {
        Duplicator d{target, clone_out, done};
        mj::Visit(*p, d);
      }
    }
  }

  template <class C>
  void union_child(int, const char*, C& list) {
    if (*done) return;
    for (std::size_t i = 0; i < list.size(); ++i) {
      bool match = false;
      std::visit(
          [&](auto& p) {
            if (p && static_cast<const void*>(p.get()) == target) match = true;
          },
          list[i].node);
      if (match) {
        typename C::value_type node;
        std::visit(
            [&](auto& p) {
              auto clone = mj::Clone(*p);
              *clone_out = clone.get();
              node.node = std::move(clone);
            },
            list[i].node);
        list.insert(list.begin() + static_cast<std::ptrdiff_t>(i) + 1,
                    std::move(node));
        *done = true;
        return;
      }
    }
    for (auto& item : list) {
      if (*done) return;
      std::visit(
          [&](auto& p) {
            if (p) {
              Duplicator d{target, clone_out, done};
              mj::Visit(*p, d);
            }
          },
          item.node);
    }
  }
};

// Erase from every child / union-child list the elements a predicate selects,
// their subtrees going with them, then descend into the survivors.
template <class Pred>
struct PruneVisitor {
  Pred* pred;
  template <class U>
  void field(int, const char*, U&) {}
  template <class T>
  void child(int, const char*, std::vector<std::unique_ptr<T>>& list) {
    std::erase_if(list,
                  [&](const std::unique_ptr<T>& p) { return p && (*pred)(*p); });
    for (auto& p : list)
      if (p) {
        PruneVisitor sub{pred};
        mj::Visit(*p, sub);
      }
  }
  template <class U>
  void union_child(int, const char*, std::vector<U>& list) {
    std::erase_if(list, [&](const U& item) {
      bool rm = false;
      std::visit(
          [&](const auto& p) {
            if (p && (*pred)(*p)) rm = true;
          },
          item.node);
      return rm;
    });
    for (auto& item : list)
      std::visit(
          [&](auto& p) {
            if (p) {
              PruneVisitor sub{pred};
              mj::Visit(*p, sub);
            }
          },
          item.node);
  }
};

// Locate the BodyChildAny node whose held element is `target`, returning the
// owning list + index. Searches a subtree recursively (Body/Frame carry their
// own subtree).
inline bool FindOwningList(std::vector<mj::BodyChildAny>& sub,
                           const void* target,
                           std::vector<mj::BodyChildAny>** out_list,
                           std::size_t* out_i) {
  for (std::size_t i = 0; i < sub.size(); ++i) {
    bool match = false;
    std::visit(
        [&](auto& up) {
          if (up && static_cast<const void*>(up.get()) == target) match = true;
        },
        sub[i].node);
    if (match) {
      *out_list = &sub;
      *out_i = i;
      return true;
    }
    bool rec = false;
    std::visit(
        [&](auto& up) {
          using T = std::decay_t<decltype(*up)>;
          if constexpr (std::is_same_v<T, mj::Body> ||
                        std::is_same_v<T, mj::Frame>) {
            if (up && FindOwningList(up->subtree, target, out_list, out_i))
              rec = true;
          }
        },
        sub[i].node);
    if (rec) return true;
  }
  return false;
}

inline bool FindOwningListInModel(mj::Model& model, const void* target,
                                  std::vector<mj::BodyChildAny>** out_list,
                                  std::size_t* out_i) {
  for (auto& w : model.worldbody) {
    if (w && FindOwningList(w->subtree, target, out_list, out_i)) return true;
  }
  return false;
}

}  // namespace tree_detail

struct PlainTree {
  template <class T>
  using owner = std::unique_ptr<T>;

  // Every child element of `parent`, in document order. Constness follows the
  // walked object's.
  template <class E, class Fn>
  static void ForEachChild(E& parent, Fn&& fn) {
    tree_detail::ChildIter<std::remove_reference_t<Fn>> it{&fn};
    mj::Visit(parent, it);
  }

  // Every child of `parent`, in document order, with the id of the storage slot
  // holding it. The id is the generated Visit child id, which is also the index
  // into the element's binding/reflect child list -- so a caller can recover the
  // contextual XML tag without knowing the storage.
  template <class E, class Fn>
  static void ForEachChildAt(E& parent, Fn&& fn) {
    tree_detail::SlottedChildIter<std::remove_reference_t<Fn>> it{&fn};
    mj::Visit(parent, it);
  }

  // The element types `parent`'s storage slots admit: `fn(int slot, TypeTag<T>)`
  // once per (slot, admissible type) pair. Children sharing a slot share one
  // document order; children in different slots do not interleave.
  template <class E, class Fn>
  static void ForEachChildSlot(E& parent, Fn&& fn) {
    tree_detail::SlotIter<std::remove_reference_t<Fn>> it{&fn};
    mj::Visit(parent, it);
  }

  // Every child of `parent` that is exactly a T.
  template <class T, class E, class Fn>
  static void ForEachChildOfType(E& parent, Fn&& fn) {
    tree_detail::TypedChildIter<T, std::remove_reference_t<Fn>> it{&fn};
    mj::Visit(parent, it);
  }

  template <class T, class E>
  static auto FirstChildOfType(E& parent)
      -> std::conditional_t<std::is_const_v<E>, const T*, T*> {
    using R = std::conditional_t<std::is_const_v<E>, const T, T>;
    tree_detail::FirstOf<R> f;
    mj::Visit(parent, f);
    return f.out;
  }

  template <class T, class E>
  static void ClearChildrenOfType(E& parent) {
    tree_detail::ChildClearer<T> c;
    mj::Visit(parent, c);
  }

  // Link an owned child into the parent at `index` among the siblings that share
  // its storage (kAppend == last). Construction and linkage are one step, which
  // is what a component profile requires and what the plain profile is happy to
  // provide.
  template <class T, class E>
  static T& Adopt(E& parent, std::size_t index, owner<T> child) {
    tree_detail::Adopter<T> a{std::move(child), index, nullptr};
    mj::Visit(parent, a);
    return *a.out;
  }

  // Detach the node at `target` from wherever under `root` owns it. The plain
  // profile destroys it immediately; the contract only promises detachment plus
  // release at an unspecified later point, so a component profile's deferred GC
  // is a valid implementation and callers must not dereference `target` after.
  template <class Root>
  static bool Remove(Root& root, const void* target) {
    bool done = false;
    tree_detail::Remover r{target, &done};
    mj::Visit(root, r);
    return done;
  }

  template <class Root>
  static void* CloneAsNextSibling(Root& root, const void* target) {
    void* clone = nullptr;
    bool done = false;
    tree_detail::Duplicator d{target, &clone, &done};
    mj::Visit(root, d);
    return done ? clone : nullptr;
  }

  template <class Root, class Pred>
  static void PruneIf(Root& root, Pred&& pred) {
    tree_detail::PruneVisitor<std::remove_reference_t<Pred>> v{&pred};
    mj::Visit(root, v);
  }

  // The set of pointers in the subtree rooted at the movable child `elem`, for
  // the cycle check. Empty when `elem` is not a movable child.
  static void CollectMovableSubtree(mj::Model& model, const void* elem,
                                    std::unordered_set<const void*>& out) {
    std::vector<mj::BodyChildAny>* list = nullptr;
    std::size_t i = 0;
    if (!tree_detail::FindOwningListInModel(model, elem, &list, &i)) return;
    std::visit(
        [&](auto& up) {
          if (!up) return;
          WalkPtrs(*up, out);
        },
        (*list)[i].node);
  }

  // Move the body-context child `elem` into `new_parent` (a Body or Frame;
  // nullptr == the world body). Pure tree: the element keeps its authored local
  // pose.
  static MoveStatus Reparent(mj::Model& model, const void* elem,
                             const void* new_parent) {
    std::vector<mj::BodyChildAny>* src_list = nullptr;
    std::size_t src_i = 0;
    if (!tree_detail::FindOwningListInModel(model, elem, &src_list, &src_i))
      return MoveStatus::NotMovable;
    std::vector<mj::BodyChildAny>* dst = SubtreeOf(model, new_parent);
    if (dst == nullptr) return MoveStatus::BadTarget;

    std::unordered_set<const void*> subptrs;
    std::visit(
        [&](auto& up) {
          if (up) WalkPtrs(*up, subptrs);
        },
        (*src_list)[src_i].node);
    if (new_parent != nullptr && subptrs.count(new_parent))
      return MoveStatus::Cycle;

    mj::BodyChildAny moved = std::move((*src_list)[src_i]);
    src_list->erase(src_list->begin() + static_cast<std::ptrdiff_t>(src_i));
    dst->push_back(std::move(moved));
    return MoveStatus::Ok;
  }

 private:
  template <class E>
  static void WalkPtrs(E& e, std::unordered_set<const void*>& out) {
    out.insert(static_cast<const void*>(&e));
    ForEachChild(e, [&](auto& c) { WalkPtrs(c, out); });
  }

  // The subtree list of a container: a Body, a Frame, or the world (nullptr).
  static std::vector<mj::BodyChildAny>* SubtreeOf(mj::Model& model,
                                                  const void* parent) {
    if (parent == nullptr) {
      if (model.worldbody.empty() || !model.worldbody.front())
        model.worldbody.insert(model.worldbody.begin(),
                               std::make_unique<mj::Body>());
      return &model.worldbody.front()->subtree;
    }
    std::vector<mj::BodyChildAny>* out = nullptr;
    Find(model, parent, out);
    return out;
  }

  template <class E>
  static void Find(E& e, const void* parent,
                   std::vector<mj::BodyChildAny>*& out) {
    if (out != nullptr) return;
    using X = std::decay_t<E>;
    if constexpr (std::is_same_v<X, mj::Body> || std::is_same_v<X, mj::Frame>) {
      if (static_cast<const void*>(&e) == parent) {
        out = &e.subtree;
        return;
      }
    }
    ForEachChild(e, [&](auto& c) { Find(c, parent, out); });
  }
};

// --- Ref ------------------------------------------------------------------ //

namespace ref_detail {

using Slot = RefSlot<std::string_view>;

// An `opt<Ref<T>>`: authored when the optional holds a non-empty name. Assigning
// an empty name clears the field.
template <class T>
const typename Slot::Ops* OptRefOps() {
  using S = ps::opt<ps::Ref<T>>;
  static const typename Slot::Ops ops{
      [](void* o) { S* s = static_cast<S*>(o); return s->has_value() && !(*s)->name.empty(); },
      [](void* o) -> std::string_view {
        S* s = static_cast<S*>(o);
        return s->has_value() ? std::string_view((*s)->name) : std::string_view();
      },
      [](void* o, std::string_view v) {
        S* s = static_cast<S*>(o);
        if (v.empty()) {
          s->reset();
        } else if (s->has_value()) {
          (*s)->name.assign(v);
        } else {
          *s = ps::Ref<T>(std::string(v));
        }
      },
      [](void* o) { static_cast<S*>(o)->reset(); },
  };
  return &ops;
}

// A bare `Ref<T>` (a required reference): there is nowhere to reset to, so
// clearing empties the name and the validator reports the missing reference it
// now is.
template <class T>
const typename Slot::Ops* BareRefOps() {
  using S = ps::Ref<T>;
  static const typename Slot::Ops ops{
      [](void* o) { return !static_cast<S*>(o)->name.empty(); },
      [](void* o) -> std::string_view {
        return std::string_view(static_cast<S*>(o)->name);
      },
      [](void* o, std::string_view v) { static_cast<S*>(o)->name.assign(v); },
      [](void* o) { static_cast<S*>(o)->name.clear(); },
  };
  return &ops;
}

// The `opt<std::string>` behind a dynamic (target_from) reference, and the
// sibling keyword field that types it.
inline const typename Slot::Ops* OptStrOps() {
  using S = ps::opt<std::string>;
  static const typename Slot::Ops ops{
      [](void* o) { S* s = static_cast<S*>(o); return s->has_value() && !(*s)->empty(); },
      [](void* o) -> std::string_view {
        S* s = static_cast<S*>(o);
        return s->has_value() ? std::string_view(**s) : std::string_view();
      },
      [](void* o, std::string_view v) {
        S* s = static_cast<S*>(o);
        if (v.empty())
          s->reset();
        else
          *s = std::string(v);
      },
      [](void* o) { static_cast<S*>(o)->reset(); },
  };
  return &ops;
}

// Hand `on(field_id, field_name, slot, target_types)` one slot per authored
// typed reference: scalar fields, and one per entry of a reference list.
template <class OnRef>
struct RefScan {
  OnRef* on;
  template <class U>
  void field(int id, const char* name, U& v) {
    using D = std::decay_t<U>;
    static_assert(!std::is_const_v<U> || !opt_ref<D>::value,
                  "the reference scan hands out live slots and needs a mutable "
                  "element");
    if constexpr (opt_ref<D>::value) {
      using Tgt = typename opt_ref<D>::target;
      if constexpr (opt_ref<D>::optional) {
        if (v.has_value() && !v->name.empty())
          (*on)(id, name, Slot{&v, OptRefOps<Tgt>()}, RefTargetTypes<Tgt>());
      } else if (!v.name.empty()) {
        (*on)(id, name, Slot{&v, BareRefOps<Tgt>()}, RefTargetTypes<Tgt>());
      }
    } else if constexpr (opt_ref_list<D>::value) {
      using Tgt = typename opt_ref_list<D>::target;
      auto each = [&](auto& list) {
        for (auto& r : list)
          if (!r.name.empty())
            (*on)(id, name, Slot{&r, BareRefOps<Tgt>()}, RefTargetTypes<Tgt>());
      };
      if constexpr (opt_ref_list<D>::optional) {
        if (v.has_value()) each(*v);
      } else {
        each(v);
      }
    }
  }
  template <class C>
  void child(int, const char*, C&) {}
  template <class C>
  void union_child(int, const char*, C&) {}
};

// The `opt<std::string>` field at reflect id `id`, as a slot.
struct StrFieldGrab {
  int target;
  void* out = nullptr;
  template <class U>
  void field(int id, const char*, U& v) {
    if (id != target) return;
    if constexpr (std::is_same_v<std::decay_t<U>, ps::opt<std::string>>) {
      if constexpr (!std::is_const_v<U>) out = &v;
    }
  }
  template <class C>
  void child(int, const char*, C&) {}
  template <class C>
  void union_child(int, const char*, C&) {}
};

// The typed reference field at reflect id `id`, as a slot -- the by-id form of
// what RefScan hands out, for a caller that already knows which field it wants.
struct RefFieldGrab {
  int target;
  void* obj = nullptr;
  const typename Slot::Ops* ops = nullptr;
  template <class U>
  void field(int id, const char*, U& v) {
    if (id != target) return;
    using D = std::decay_t<U>;
    if constexpr (std::is_const_v<U>) {
      (void)v;
    } else if constexpr (opt_ref<D>::value) {
      using Tgt = typename opt_ref<D>::target;
      obj = &v;
      if constexpr (opt_ref<D>::optional)
        ops = OptRefOps<Tgt>();
      else
        ops = BareRefOps<Tgt>();
    }
  }
  template <class C>
  void child(int, const char*, C&) {}
  template <class C>
  void union_child(int, const char*, C&) {}
};

// The authored name of the typed reference at reflect id `id`, read-only.
struct ConstRefNameGrab {
  int target;
  std::string_view out;
  template <class U>
  void field(int id, const char*, const U& v) {
    if (id != target) return;
    using D = std::decay_t<U>;
    if constexpr (opt_ref<D>::value) {
      if constexpr (opt_ref<D>::optional) {
        if (v.has_value()) out = std::string_view(v->name);
      } else {
        out = std::string_view(v.name);
      }
    }
  }
  template <class C>
  void child(int, const char*, const C&) {}
  template <class C>
  void union_child(int, const char*, const C&) {}
};

}  // namespace ref_detail

struct PlainRef {
  using slot = RefSlot<std::string_view>;

  template <class E, class OnRef>
  static void ScanTyped(E& e, OnRef&& on) {
    ref_detail::RefScan<std::remove_reference_t<OnRef>> v{&on};
    mj::Visit(e, v);
  }

  template <class E>
  static slot SlotAt(E& e, int field_id) {
    ref_detail::RefFieldGrab g{field_id};
    mj::Visit(e, g);
    return g.ops != nullptr ? slot{g.obj, g.ops} : slot{};
  }

  // The authored name in the reference field at `field_id`, read-only: the
  // const-safe half of SlotAt, for an index build over a const document.
  template <class E>
  static std::string_view NameAt(const E& e, int field_id) {
    ref_detail::ConstRefNameGrab g{field_id};
    mj::Visit(e, g);
    return g.out;
  }

  // The name half (or keyword half) of a dynamic reference: a plain string field
  // the schema types at runtime through a sibling.
  template <class E>
  static slot DynSlot(E& e, int field_id) {
    ref_detail::StrFieldGrab g{field_id};
    mj::Visit(e, g);
    return g.out != nullptr ? slot{g.out, ref_detail::OptStrOps()} : slot{};
  }

  // Reset every typed reference of `e` whose name `pred(name, targets)` selects.
  // A reference LIST drops only the selected entries; an optional list emptied
  // that way resets to unauthored so the attribute disappears from the written
  // MJCF. That structural difference is why this is one profile operation rather
  // than a loop over slots.
  template <class E, class Pred>
  static void ClearTypedIf(E& e, Pred&& pred) {
    Clearer<std::remove_reference_t<Pred>> c{&pred};
    mj::Visit(e, c);
  }

 private:
  template <class Pred>
  struct Clearer {
    Pred* pred;
    template <class U>
    void field(int, const char*, U& v) {
      using D = std::decay_t<U>;
      if constexpr (opt_ref<D>::value) {
        using Tgt = typename opt_ref<D>::target;
        if constexpr (opt_ref<D>::optional) {
          if (v.has_value() && !v->name.empty() &&
              (*pred)(std::string_view(v->name), RefTargetTypes<Tgt>())) {
            v.reset();
          }
        } else if (!v.name.empty() &&
                   (*pred)(std::string_view(v.name), RefTargetTypes<Tgt>())) {
          v.name.clear();
        }
      } else if constexpr (opt_ref_list<D>::value) {
        using Tgt = typename opt_ref_list<D>::target;
        auto prune = [&](auto& list) {
          std::erase_if(list, [&](const auto& r) {
            return !r.name.empty() &&
                   (*pred)(std::string_view(r.name), RefTargetTypes<Tgt>());
          });
        };
        if constexpr (opt_ref_list<D>::optional) {
          if (v.has_value()) {
            prune(*v);
            if (v->empty()) v.reset();
          }
        } else {
          prune(v);
        }
      }
    }
    template <class C>
    void child(int, const char*, C&) {}
    template <class C>
    void union_child(int, const char*, C&) {}
  };
};

// --- Doc ------------------------------------------------------------------ //

struct PlainDoc {
  using doc_type = mj::Model;
  using root_type = mj::Model;
  using node_ptr = void*;

  template <class E>
  static constexpr bool is_root =
      std::is_same_v<std::remove_const_t<E>, mj::Model>;

  // Every top-level section EXCEPT the <default> tree, in document order. Class
  // elements live only under <default> and are authoring templates, not model
  // content; operations that act on real elements use this to skip them.
  template <class D, class Fn>
  static void ForEachLiveSection(D& doc, Fn&& fn) {
    Each(doc.compilers, fn);
    Each(doc.options, fn);
    Each(doc.sizes, fn);
    Each(doc.visuals, fn);
    Each(doc.statistics, fn);
    Each(doc.extensions, fn);
    Each(doc.customs, fn);
    Each(doc.assets, fn);
    Each(doc.worldbody, fn);
    Each(doc.deformables, fn);
    Each(doc.contacts, fn);
    Each(doc.equalitys, fn);
    Each(doc.tendons, fn);
    Each(doc.actuators, fn);
    Each(doc.sensors, fn);
    Each(doc.keyframes, fn);
  }

  static std::optional<std::string_view> Name(const mj::Model& m) {
    if (!m.model) return std::nullopt;
    return std::string_view(*m.model);
  }

 private:
  template <class L, class Fn>
  static void Each(L& list, Fn& fn) {
    for (auto& p : list)
      if (p) fn(*p);
  }
};

// --- Attribute storage shapes (io) ---------------------------------------- //

namespace shape_detail {

// Strip the presence wrapper, lazily: the non-optional arm must not name
// is_opt<T>::inner, which does not exist there.
template <class T, bool = is_opt<T>::value>
struct inner_of {
  using type = T;
};
template <class T>
struct inner_of<T, true> {
  using type = typename is_opt<T>::inner;
};

template <class I>
constexpr Shape KindOf() {
  if constexpr (is_ref<I>::value) {
    return Shape::Ref;
  } else if constexpr (std::is_same_v<I, std::string>) {
    return Shape::Str;
  } else if constexpr (std::is_same_v<I, bool>) {
    return Shape::Bool;
  } else if constexpr (std::is_enum_v<I>) {
    return Shape::Enum;
  } else if constexpr (is_std_array<I>::value) {
    return Shape::Fixed;
  } else if constexpr (is_inline_vec<I>::value) {
    return Shape::Range;
  } else if constexpr (is_vector<I>::value) {
    using S = typename I::value_type;
    if constexpr (is_ref<S>::value) {
      return Shape::RefList;
    } else if constexpr (std::is_enum_v<S>) {
      return Shape::EnumList;
    } else {
      return Shape::Unbounded;
    }
  } else if constexpr (std::is_arithmetic_v<I>) {
    return Shape::Scalar;
  } else {
    return Shape::Unknown;
  }
}

}  // namespace shape_detail

struct PlainShape {
  // --- Presence ---
  template <class T>
  static constexpr bool optional_v = is_opt<std::decay_t<T>>::value;

  template <class T>
  using inner_t = typename shape_detail::inner_of<std::decay_t<T>>::type;

  template <class T>
  static bool IsSet(const T& slot) {
    if constexpr (optional_v<T>) {
      return slot.has_value();
    } else {
      (void)slot;
      return true;
    }
  }
  template <class T>
  static const inner_t<T>& Read(const T& slot) {
    if constexpr (optional_v<T>) {
      return *slot;
    } else {
      return slot;
    }
  }
  template <class T>
  static void Author(T& slot, inner_t<T> v) {
    slot = std::move(v);
  }
  template <class T>
  static void Reset(T& slot) {
    if constexpr (optional_v<T>) slot.reset();
  }

  // --- Inner shape ---
  template <class I>
  static constexpr Shape kind_v = shape_detail::KindOf<std::decay_t<I>>();

  // --- Str / Ref scalars ---
  template <class I>
  static std::string_view StrGet(const I& v) {
    return std::string_view(v);
  }
  template <class I>
  static I StrMake(std::string_view v) {
    return I(v);
  }
  template <class I>
  static std::string_view RefGet(const I& v) {
    return std::string_view(v.name);
  }
  template <class I>
  static I RefMake(std::string_view v) {
    return I(std::string(v));
  }

  // --- Enums ---
  // Enum values travel between profiles as their schema declaration index, which
  // is also the MJCF keyword order; the concrete enum type is per-profile (a
  // plain `enum class`, a UENUM) but the indices agree by construction. The
  // keyword tables are per-profile too, so io converts through the policy rather
  // than naming a generated FromMjcf/ToMjcf overload set.
  template <class I>
  static I EnumMake(int index) {
    return static_cast<I>(index);
  }
  template <class I>
  static int EnumIndex(const I& v) {
    return static_cast<int>(v);
  }
  template <class I>
  static bool EnumFromMjcf(std::string_view kw, I& out) {
    return mj::FromMjcf(kw, out);
  }
  template <class I>
  static std::string_view EnumToMjcf(const I& v) {
    return mj::ToMjcf(v);
  }

  // --- Fixed arity ---
  // Whole-array transfer in MJCF component order, never indexed access: a
  // profile whose fixed-3 type is a struct of named members (or whose fixed-4
  // quaternion stores its components in a different order than MJCF authors
  // them) implements the permutation once, here, instead of every read and write
  // site assuming an indexable container.
  template <class I>
  struct fixed;

  // --- Sequences (range and unbounded) ---
  template <class I>
  struct seq;

  // --- Reference lists ---
  // A `ref<T>[]` attribute: several names in one space-joined attribute. Its
  // entries are names, not scalars, so it gets its own adapter rather than
  // riding the numeric sequence one.
  template <class I>
  struct reflist;
};

template <class S, std::size_t N>
struct PlainShape::fixed<std::array<S, N>> {
  using scalar = S;
  static constexpr std::size_t size = N;
  static void Load(const std::array<S, N>& v, S* out) {
    for (std::size_t i = 0; i < N; ++i) out[i] = v[i];
  }
  static std::array<S, N> Make(const S* in) {
    std::array<S, N> out{};
    for (std::size_t i = 0; i < N; ++i) out[i] = in[i];
    return out;
  }
};

template <class S, std::size_t N>
struct PlainShape::seq<ps::InlineVec<S, N>> {
  using scalar = S;
  static constexpr std::size_t max_size = N;  // 0 == unbounded
  static std::size_t Size(const ps::InlineVec<S, N>& v) { return v.size(); }
  static void Load(const ps::InlineVec<S, N>& v, S* out) {
    for (std::size_t i = 0; i < v.size(); ++i) out[i] = v[i];
  }
  static ps::InlineVec<S, N> Make(const S* in, std::size_t n) {
    ps::InlineVec<S, N> out;
    for (std::size_t i = 0; i < n; ++i) out.push_back(in[i]);
    return out;
  }
};

template <class T>
struct PlainShape::reflist<std::vector<ps::Ref<T>>> {
  using list = std::vector<ps::Ref<T>>;
  static std::size_t Size(const list& v) { return v.size(); }
  static std::string_view Get(const list& v, std::size_t i) {
    return std::string_view(v[i].name);
  }
  static list Make(const std::vector<std::string_view>& names) {
    list out;
    out.reserve(names.size());
    for (std::string_view n : names) out.push_back(ps::Ref<T>(std::string(n)));
    return out;
  }
};

template <class S>
struct PlainShape::seq<std::vector<S>> {
  using scalar = S;
  static constexpr std::size_t max_size = 0;
  static std::size_t Size(const std::vector<S>& v) { return v.size(); }
  static void Load(const std::vector<S>& v, S* out) {
    for (std::size_t i = 0; i < v.size(); ++i) out[i] = v[i];
  }
  static std::vector<S> Make(const S* in, std::size_t n) {
    return std::vector<S>(in, in + n);
  }
};

// --- The profile ---------------------------------------------------------- //

struct Plain {
  using Doc = PlainDoc;
  using Str = StdStrPolicy;
  using Tree = PlainTree;
  using Ref = PlainRef;
  using Ident = PlainIdent;
  using Shape = PlainShape;

  template <class E, class V>
  static void Visit(E& e, V&& v) {
    mj::Visit(e, std::forward<V>(v));
  }

  template <class E>
  static constexpr mj::ElementType type_of =
      mj::element_type_of<std::decay_t<E>>::value;

  template <mj::ElementType E>
  using element_t = mj::element_t<E>;

  // Stamp the schema's `=` defaults onto a fresh element. The lowest-priority
  // layer of the class merge, and per-profile: a profile emits its own default
  // table over its own storage.
  template <class E>
  static void ApplyDefault(E& e) {
    mj::ApplyDefault(e);
  }

  // That same layer as a shared, read-only element, which is how the class
  // merge asks for it. A profile whose elements are not stack-constructible
  // (a component profile's are host objects with an allocator) can still
  // answer this, which is why the merge asks for a prototype rather than
  // constructing one.
  template <class E>
  static const E& Defaults() {
    static const E defs = [] {
      E e;
      mj::ApplyDefault(e);
      return e;
    }();
    return defs;
  }

  // Most elements carry `opt<std::string> name`; one the schema marks required
  // (<numeric>, <text>, ...) carries a plain `std::string`, and a Default carries
  // `opt<string> dclass` as its identity.
  template <class E>
  static constexpr bool has_name = HasNameImpl<std::decay_t<E>>();

  // nullopt means "nameless or unset" -- the two cases the caller must not
  // distinguish; `has_name` answers the type-level question separately.
  template <class E>
  static std::optional<std::string_view> Name(const E& e) {
    using X = std::decay_t<E>;
    if constexpr (std::is_same_v<X, mj::Default>) {
      if (!e.dclass) return std::nullopt;
      return std::string_view(*e.dclass);
    } else if constexpr (requires { e.name; }) {
      using NT = std::decay_t<decltype(e.name)>;
      if constexpr (std::is_same_v<NT, ps::opt<std::string>>) {
        if (!e.name) return std::nullopt;
        return std::string_view(*e.name);
      } else if constexpr (std::is_same_v<NT, std::string>) {
        return std::string_view(e.name);
      } else {
        return std::nullopt;
      }
    } else {
      return std::nullopt;
    }
  }

  template <class E>
  static void SetName(E& e, std::string_view name) {
    using X = std::decay_t<E>;
    if constexpr (std::is_same_v<X, mj::Default>) {
      e.dclass = std::string(name);
    } else if constexpr (requires { e.name; }) {
      using NT = std::decay_t<decltype(e.name)>;
      if constexpr (std::is_same_v<NT, ps::opt<std::string>> ||
                    std::is_same_v<NT, std::string>) {
        e.name = std::string(name);
      }
    }
  }

 private:
  template <class X>
  static constexpr bool HasNameImpl() {
    if constexpr (std::is_same_v<X, mj::Default>) {
      return true;
    } else if constexpr (requires(X x) { x.name; }) {
      using NT = std::decay_t<decltype(std::declval<X>().name)>;
      return std::is_same_v<NT, ps::opt<std::string>> ||
             std::is_same_v<NT, std::string>;
    } else {
      return false;
    }
  }
};

}  // namespace ps::sdk::plain

#endif  // PROTOSPEC_SDK_PLAIN_PROFILE_H
