// ProtoSpec SDK: traversal and lookup.
//
// Downward queries (Find, ForEach*) are direct walks over the profile's Visit
// hook. Upward queries go through ParentMap (protospec/parents.h). Everything
// here is a query except
// SetName, which writes a single field; structural edits live in the sibling
// headers.
//
// Every verb is generic over an emission profile `P` (profile.h) and defaults to
// the plain profile, so an existing call site reads exactly as it did.
#ifndef PROTOSPEC_SDK_TRAVERSAL_H
#define PROTOSPEC_SDK_TRAVERSAL_H

#include <cassert>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "protospec/detail.h"
#include "protospec/parents.h"
#include "protospec/plain_profile.h"
#include "protospec/profile.h"
#include "reflect.h"

namespace ps::sdk {

namespace mj = ps::mjcf;

// --- Element identity ----------------------------------------------------- //
// Uniform name / type access over any element, so a consumer never reaches into
// ps::sdk::detail for the basics. `Name` returns the authored name (a Default's
// identity is its dclass), or nullopt for the nameless (Replicate, Config, ...)
// and for a nameable element that has none.

template <class P = plain::Plain, class E>
std::optional<ViewOf<P>> Name(const E& e) {
  return detail::NameOf<P>(e);
}

// Set (create) an element's name field; a no-op for nameless element types.
// Returns false, leaving the element untouched, when `name` starts with the
// reserved auto-name prefix (ps::kReservedNamePrefix) -- authored names must
// stay outside it so they can never collide with compile-time auto-names.
template <class P = plain::Plain, class E>
bool SetName(E& e, ViewOf<P> name) {
  if (P::Str::StartsWith(name, ps::kReservedNamePrefix)) return false;
  detail::SetName<P>(e, name);
  return true;
}

// The runtime element-type tag of a concrete element type.
template <class P = plain::Plain, class E>
mj::ElementType TypeOf(const E&) {
  return ElementTypeOf<P, E>;
}

// --- Element walks -------------------------------------------------------- //
// Invoke `fn(element&)` for `root` and every element beneath it, document order.
template <class P = plain::Plain, class E, class Fn>
void WalkSubtree(E& root, Fn&& fn) {
  detail::WalkTree<P>(root, std::forward<Fn>(fn));
}

// Invoke `fn(element&)` for every element in the document, the <default> class
// tree included. Const overload walks a const document.
template <class P = plain::Plain, class Fn>
void WalkModel(DocOf<P>& model, Fn&& fn) {
  detail::WalkModelAll<P>(model, std::forward<Fn>(fn));
}
template <class P = plain::Plain, class Fn>
void WalkModel(const DocOf<P>& model, Fn&& fn) {
  detail::WalkModelAll<P>(model, std::forward<Fn>(fn));
}

// Invoke `fn(element&)` for every element of the document EXCEPT the root: the
// root is the document, not a selectable/prunable element, and it carries no
// serial. This is the guarded "walk every element" the editor spelled out at six
// call sites; promoting it here removes that boilerplate and its easy-to-misplace
// guard.
template <class P = plain::Plain, class Fn>
void ForEachElement(DocOf<P>& model, Fn&& fn) {
  detail::WalkModelAll<P>(model, [&](auto& e) {
    if constexpr (!IsRoot<P, decltype(e)>) fn(e);
  });
}
template <class P = plain::Plain, class Fn>
void ForEachElement(const DocOf<P>& model, Fn&& fn) {
  detail::WalkModelAll<P>(model, [&](const auto& e) {
    if constexpr (!IsRoot<P, decltype(e)>) fn(e);
  });
}

// --- Find ----------------------------------------------------------------- //

// The first element of type T with the given name, or nullptr. Searches the
// whole document, class elements included. Names are unique per element type in
// a valid model, so this is effectively a keyed lookup.
template <class T, class P = plain::Plain>
T* Find(DocOf<P>& model, ViewOf<P> name) {
  T* found = nullptr;
  detail::WalkModelAll<P>(model, [&](auto& e) {
    using E = std::decay_t<decltype(e)>;
    if constexpr (std::is_same_v<E, T>) {
      if (!found) {
        std::optional<ViewOf<P>> nm = detail::NameOf<P>(e);
        if (nm && P::Str::Equals(*nm, name)) found = &e;
      }
    }
  });
  return found;
}

template <class T, class P = plain::Plain>
const T* Find(const DocOf<P>& model, ViewOf<P> name) {
  return Find<T, P>(const_cast<DocOf<P>&>(model), name);
}

// --- Find by serial ------------------------------------------------------- //
// Every authored element carries a process-unique serial (assigned at
// construction, preserved by the serial-aware clone) -- the stable identity a UI
// holds across edits, since the pointer moves on a tree mutation. These resolve
// that identity back to a live element with one generic document walk.

// A located element: its address and runtime type, or null when unfound.
struct Located {
  void* ptr = nullptr;
  mj::ElementType type{};
  explicit operator bool() const { return ptr != nullptr; }
};

// The element in `model` whose serial is `serial`, with its runtime type, or a
// null Located when none matches. Serial 0 never matches (it is the "no
// selection" sentinel), and the document root is excluded (it is the document,
// not a prunable/selectable element). Serials are unique, so the first match
// wins.
template <class P = plain::Plain>
Located FindBySerialTyped(DocOf<P>& model, std::uint64_t serial) {
  Located out;
  if (serial == 0) return out;
  detail::WalkModelAll<P>(model, [&](auto& e) {
    using E = std::decay_t<decltype(e)>;
    if (out.ptr) return;
    if constexpr (!IsRoot<P, E>) {
      if (P::Ident::template Serial<E>(e) == serial) {
        out.ptr = static_cast<void*>(&e);
        out.type = ElementTypeOf<P, E>;
      }
    }
  });
  return out;
}

// The element in `model` carrying `serial` as a type-erased pointer, or nullptr.
// Pair with reflect::Describe on the type from FindBySerialTyped to recover a
// concrete type; this overload is for callers that only need the address.
template <class P = plain::Plain>
void* FindBySerial(DocOf<P>& model, std::uint64_t serial) {
  return FindBySerialTyped<P>(model, serial).ptr;
}
template <class P = plain::Plain>
const void* FindBySerial(const DocOf<P>& model, std::uint64_t serial) {
  return FindBySerialTyped<P>(const_cast<DocOf<P>&>(model), serial).ptr;
}

// Typed pointer to the element carrying `serial`, iff it is a `T` (else
// nullptr). The typed slice of FindBySerialTyped: serials are process-unique, so
// the element carrying `serial` is a `T` exactly when its runtime type is T's.
template <class T, class P = plain::Plain>
T* FindBySerialAs(DocOf<P>& model, std::uint64_t serial) {
  const Located loc = FindBySerialTyped<P>(model, serial);
  if (loc && loc.type == ElementTypeOf<P, T>) return static_cast<T*>(loc.ptr);
  return nullptr;
}
template <class T, class P = plain::Plain>
const T* FindBySerialAs(const DocOf<P>& model, std::uint64_t serial) {
  return FindBySerialAs<T, P>(const_cast<DocOf<P>&>(model), serial);
}

// The creation serial of the element at `ptr`, or 0 when `ptr` is null, the
// document root, or not an element of `model`. The inverse of FindBySerial: a UI
// holding a raw element pointer (e.g. straight off a structural verb that
// returns one) recovers the stable serial identity to persist across edits.
template <class P = plain::Plain>
std::uint64_t SerialOf(const DocOf<P>& model, const void* ptr) {
  if (!ptr) return 0;
  std::uint64_t out = 0;
  ForEachElement<P>(model, [&](const auto& e) {
    if (out) return;
    if (static_cast<const void*>(&e) == ptr)
      out = P::Ident::Serial(e);
  });
  return out;
}

// The authored name of the element carrying `serial`, or "" when unfound or
// nameless. The serial->name read the editor open-coded per element kind.
template <class P = plain::Plain>
StrOf<P> NameOfSerial(const DocOf<P>& model, std::uint64_t serial) {
  StrOf<P> out;
  if (serial == 0) return out;
  bool found = false;
  ForEachElement<P>(model, [&](const auto& e) {
    if (found) return;
    if (P::Ident::Serial(e) == serial) {
      found = true;
      if (std::optional<ViewOf<P>> nm = detail::NameOf<P>(e))
        out = P::Str::FromUtf8(P::Str::ToUtf8(*nm));
    }
  });
  return out;
}

// A deep clone of `src` whose every element carries the SAME serial as its
// source counterpart. The profile's clone MINTS FRESH serials; this pairs source
// and clone in lockstep document order -- identical by construction, since a
// structural clone reproduces the walk exactly -- and copies each serial across.
// A snapshot is thus serial-identical to the tree it came from, which is what a
// UI needs for undo/redo (selection and unnamed-element auto-naming both key on
// the serial). Only safe when the clone REPLACES the source wholesale (never
// coexists with it): duplicate serials in one live model would break the
// fresh-serial invariant.
template <class P = plain::Plain>
OwnerOf<P, DocOf<P>> CloneModelWithSerials(const DocOf<P>& src) {
  OwnerOf<P, DocOf<P>> dst = P::Ident::Clone(src);
  std::vector<std::uint64_t> serials;
  ForEachElement<P>(src,
                    [&](const auto& e) { serials.push_back(P::Ident::Serial(e)); });
  std::size_t n_dst = 0;
  ForEachElement<P>(*dst, [&](auto&) { ++n_dst; });
  // The clone reproduces the source walk exactly -- a profile's clone owes that
  // ordering guarantee, stated on P::Ident::Clone -- so the serial list and the
  // clone's walk are a bijection. ASSERT it rather than silently
  // min()-truncating a mismatch: a divergence would leave some clone serials
  // fresh, corrupting selection and compile-state migration far from the cause.
  assert(serials.size() == n_dst &&
         "CloneModelWithSerials: clone walk diverged from source "
         "(serial/slot count mismatch) -- structural-clone invariant violated");
  std::size_t i = 0;
  ForEachElement<P>(*dst, [&](auto& e) {
    if (i < serials.size()) P::Ident::SetSerial(e, serials[i]);
    ++i;
  });
  return dst;
}

// --- Typed visitors ------------------------------------------------------- //

// Call `fn(T&)` for every T anywhere in the document (class elements included).
template <class T, class P = plain::Plain, class Fn>
void ForEachOfType(DocOf<P>& model, Fn&& fn) {
  detail::WalkModelAll<P>(model, [&](auto& e) {
    using E = std::decay_t<decltype(e)>;
    if constexpr (std::is_same_v<E, T>) fn(e);
  });
}

namespace detail {

// Walk a body-context container's children, emitting each T. When `recursive`,
// descend the body-context children that carry their own subtree (Body, Frame);
// a shallow walk stops at the immediate level, so a shallow ForEachGeom never
// crosses into a child body.
template <class P, class T, class Parent, class Fn>
void EmitFromSubtree(Parent& parent, bool recursive, Fn& fn) {
  P::Tree::ForEachChild(parent, [&](auto& c) {
    using E = std::decay_t<decltype(c)>;
    if constexpr (std::is_same_v<E, T>) fn(c);
    if (recursive) {
      const mj::ElementType t = ElementTypeOf<P, E>;
      if (t == mj::ElementType::Body || t == mj::ElementType::Frame)
        EmitFromSubtree<P, T>(c, recursive, fn);
    }
  });
}

}  // namespace detail

// Each Geom directly in `body` (recursive=false) or anywhere beneath it.
template <class P = plain::Plain, class Fn>
void ForEachGeom(ElementOf<P, mj::ElementType::Body>& body, bool recursive,
                 Fn&& fn) {
  detail::EmitFromSubtree<P, ElementOf<P, mj::ElementType::Geom>>(body, recursive,
                                                                 fn);
}
template <class P = plain::Plain, class Fn>
void ForEachJoint(ElementOf<P, mj::ElementType::Body>& body, bool recursive,
                  Fn&& fn) {
  detail::EmitFromSubtree<P, ElementOf<P, mj::ElementType::Joint>>(body, recursive,
                                                                  fn);
}
template <class P = plain::Plain, class Fn>
void ForEachSite(ElementOf<P, mj::ElementType::Body>& body, bool recursive,
                 Fn&& fn) {
  detail::EmitFromSubtree<P, ElementOf<P, mj::ElementType::Site>>(body, recursive,
                                                                 fn);
}
// Each Body directly in `body` (its immediate child bodies) or, recursively,
// every descendant body.
template <class P = plain::Plain, class Fn>
void ForEachBody(ElementOf<P, mj::ElementType::Body>& body, bool recursive,
                 Fn&& fn) {
  detail::EmitFromSubtree<P, ElementOf<P, mj::ElementType::Body>>(body, recursive,
                                                                 fn);
}

}  // namespace ps::sdk

#endif  // PROTOSPEC_SDK_TRAVERSAL_H
