// ProtoSpec: the emission-profile seam.
//
// One schema, several emissions. The plain profile (plain_profile.h) stores a
// document as owned C++ values; a host profile stores the same document as its
// own native objects (Unreal components, presence-wrapped properties, an
// attachment hierarchy). The SDK's algorithms and the MJCF reader/writer are
// written once against this seam and run unchanged on either.
//
// A profile is a tag type `P` carrying five policies plus a handful of
// element-level primitives:
//
//   P::Doc     what the document root is and what its top-level sections are
//   P::Str     how names and prefixes work
//   P::Tree    how child lists work (order, insert, remove, clone, move)
//   P::Ref     how typed references are stored and rewritten
//   P::Ident   identity, provenance, duplication
//
//   P::Visit(e, v)      the generated per-element field/child hook
//   P::type_of<E>       the schema ElementType of a profile element type
//   P::element_t<T>     the profile element type for a schema ElementType
//   P::Name / SetName / has_name    authored-name access
//   P::ApplyDefault(e)  stamp the schema's `=` defaults onto a fresh element
//   P::Defaults<E>()    that layer as a shared read-only element, which is how
//                       the class merge consumes it. Asked for rather than
//                       constructed, because a profile whose elements are host
//                       objects cannot produce one on the stack.
//
// One threaded parameter, five policies: the parameter count is the maintenance
// cost (every signature carries it), the policy count is the clarity benefit,
// and they are independent -- so take one of each. Adding a sixth policy later
// touches only the profile structs and the sites that use it.
//
// `ElementType` itself is NOT profile-scoped. It is schema identity, shared by
// every emission of that schema; only the storage behind it varies.
#ifndef PROTOSPEC_SDK_PROFILE_H
#define PROTOSPEC_SDK_PROFILE_H

#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include "reflect.h"
#include "types.h"

namespace ps::sdk {

namespace mj = ps::mjcf;

// --- Child positions ------------------------------------------------------ //
// A child's position among the siblings that share its storage: for the plain
// profile the index within the owning vector, for a component profile the
// sibling index the adapter sorts on. `kAppend` means "after the last".
//
// The coordinate is deliberately storage-relative rather than global: it is the
// only coordinate in which "insert immediately after this child" means the same
// thing in a profile with one list per child type and in a profile with a single
// shuffled child list ordered by sibling index.
inline constexpr std::size_t kAppend = static_cast<std::size_t>(-1);

// A type carried as a value, so an enumeration of the child types a parent
// admits can hand each one to a generic lambda.
template <class T>
struct TypeTag {
  using type = T;
};

// --- Reparent outcomes ---------------------------------------------------- //
// The move is a profile operation (the plain profile's movable set is the
// body-context union; a component profile's is whatever its adapter admits), so
// the rejection reasons come back as a closed set the SDK renders into text.
enum class MoveStatus {
  Ok,
  NotMovable,  // the element is not a movable child in this profile
  BadTarget,   // the destination is not a container
  Cycle,       // the destination is the element or lies inside its subtree
};

// --- Attribute storage shapes (io) ---------------------------------------- //
// The closed set of storage shapes an attribute value can take, after its
// presence wrapper is stripped. The reader and writer dispatch on this instead
// of pattern-matching the plain profile's concrete storage types.
enum class Shape {
  Unknown,    // not an attribute-bearing field (never reached through a binding)
  Bool,
  Scalar,     // an arithmetic scalar
  Enum,
  Str,
  Ref,        // a single typed reference, stored as a name
  Fixed,      // exactly N scalars
  Range,      // 0..N scalars
  Unbounded,  // any number of scalars
  EnumList,   // any number of enum keywords
  RefList,    // any number of reference names
};

// --- Reference slots ------------------------------------------------------ //
//
// A live, aliasing proxy over ONE stored reference name -- a scalar `Ref<T>`
// field, one entry of a reference list, or the name half of a dynamic
// (target_from) reference. Generated Visit and the reference scan hand these out
// BY VALUE; they never hand out a reference into storage, because a component
// profile has no addressable name string to hand out.
//
// Deliberately move-only. The failure mode this guards against is silent: a
// proxy accidentally captured by value into a lambda that outlives the scan
// would leave `Rename` reporting referrer rewrites it never performed, with no
// compile error and no test failure on a small model. Making the proxy
// non-copyable turns that mistake into a build break.
// Parameterized on the view type rather than on the profile, so a profile can
// name its own slot type while still being an incomplete type.
template <class View>
class RefSlot {
 public:
  using view = View;

  // The operations a concrete slot kind supplies, as a static table. `obj` is
  // the slot's aliased storage, never owned.
  struct Ops {
    bool (*is_set)(void* obj);
    view (*get)(void* obj);
    void (*set)(void* obj, view name);
    void (*clear)(void* obj);
  };

  RefSlot() = default;
  RefSlot(void* obj, const Ops* ops) : obj_(obj), ops_(ops) {}

  RefSlot(const RefSlot&) = delete;
  RefSlot& operator=(const RefSlot&) = delete;
  RefSlot(RefSlot&&) = default;
  RefSlot& operator=(RefSlot&&) = default;

  // False for an unbound slot (the field id named no reference).
  explicit operator bool() const { return ops_ != nullptr; }

  bool IsSet() const { return ops_ != nullptr && ops_->is_set(obj_); }
  view Get() const { return ops_ != nullptr ? ops_->get(obj_) : view{}; }
  // Assigning an empty name clears the slot.
  void Set(view name) const {
    if (ops_ != nullptr) ops_->set(obj_, name);
  }
  void Clear() const {
    if (ops_ != nullptr) ops_->clear(obj_);
  }

 private:
  void* obj_ = nullptr;
  const Ops* ops_ = nullptr;
};

// The slot type a profile's reference scan hands out.
template <class P>
using RefSlotAny = RefSlot<typename P::Str::view>;

// --- Policy concepts ------------------------------------------------------ //
// These state the contract each policy owes; they are checked where a profile is
// first used so a partial profile fails at its definition rather than deep
// inside an algorithm.

template <class S>
concept StrPolicy = requires(const typename S::string& s, typename S::view v,
                             std::string_view u) {
  typename S::string;
  typename S::view;
  { S::View(s) } -> std::same_as<typename S::view>;
  { S::FromUtf8(u) } -> std::same_as<typename S::string>;
  { S::ToUtf8(v) } -> std::same_as<std::string>;
  { S::Equals(v, v) } -> std::same_as<bool>;
  { S::EqualsUtf8(v, u) } -> std::same_as<bool>;
  { S::Empty(v) } -> std::same_as<bool>;
  { S::StartsWith(v, u) } -> std::same_as<bool>;
  { S::Concat(v, v) } -> std::same_as<typename S::string>;
};

template <class I>
concept IdentPolicy = requires {
  typename I::serial_t;
};

template <class D>
concept DocPolicy = requires {
  typename D::doc_type;
  typename D::node_ptr;
};

// A profile is usable when its five policies are present and its element
// primitives are spellable. Checked at the point of use rather than at the
// definition, so a profile can be assembled incrementally.
template <class P>
concept Profile = requires {
  typename P::Doc;
  typename P::Str;
  typename P::Tree;
  typename P::Ref;
  typename P::Ident;
} && StrPolicy<typename P::Str> && DocPolicy<typename P::Doc> &&
    IdentPolicy<typename P::Ident>;

// --- Convenience aliases -------------------------------------------------- //

template <class P>
using DocOf = typename P::Doc::doc_type;
template <class P>
using StrOf = typename P::Str::string;
template <class P>
using ViewOf = typename P::Str::view;
template <class P, class T>
using OwnerOf = typename P::Tree::template owner<T>;
template <class P, mj::ElementType E>
using ElementOf = typename P::template element_t<E>;

// The schema element type of a profile element type.
template <class P, class E>
inline constexpr mj::ElementType ElementTypeOf = P::template type_of<std::decay_t<E>>;

// True when E is the profile's document root (the plain profile's `Model`, a
// component profile's document handle). Root elements are the document, not
// content: they carry no serial, cannot be selected, renamed, deleted or
// referenced, and are skipped by every content walk.
template <class P, class E>
inline constexpr bool IsRoot = P::Doc::template is_root<std::decay_t<E>>;

// --- Shared string helpers ------------------------------------------------ //
// Transparent hash/equality over a profile's string type, so a name index can be
// probed with a view without materializing a string.
template <class P>
struct StrHash {
  using is_transparent = void;
  std::size_t operator()(ViewOf<P> v) const { return P::Str::Hash(v); }
  std::size_t operator()(const StrOf<P>& s) const {
    return P::Str::Hash(P::Str::View(s));
  }
};

template <class P>
struct StrEq {
  using is_transparent = void;
  bool operator()(ViewOf<P> a, ViewOf<P> b) const { return P::Str::Equals(a, b); }
  bool operator()(const StrOf<P>& a, ViewOf<P> b) const {
    return P::Str::Equals(P::Str::View(a), b);
  }
  bool operator()(ViewOf<P> a, const StrOf<P>& b) const {
    return P::Str::Equals(a, P::Str::View(b));
  }
  bool operator()(const StrOf<P>& a, const StrOf<P>& b) const {
    return P::Str::Equals(P::Str::View(a), P::Str::View(b));
  }
};

// --- Std-string string policy --------------------------------------------- //
// The Str policy for any profile whose names are `std::string`. Shipped here (not
// in the plain profile) because it is reusable: a profile that differs only in
// storage shape, not in string type, inherits it rather than restating it.
struct StdStrPolicy {
  using string = std::string;
  using view = std::string_view;

  static view View(const string& s) { return view(s); }
  static string FromUtf8(std::string_view u) { return string(u); }
  static std::string ToUtf8(view v) { return std::string(v); }
  static bool Equals(view a, view b) { return a == b; }
  static bool EqualsUtf8(view a, std::string_view b) { return a == b; }
  static bool Empty(view v) { return v.empty(); }
  static bool StartsWith(view v, std::string_view p) { return v.starts_with(p); }
  static string Concat(view a, view b) {
    string out;
    out.reserve(a.size() + b.size());
    out.append(a);
    out.append(b);
    return out;
  }
  static std::size_t Hash(view v) { return std::hash<view>{}(v); }
};

}  // namespace ps::sdk

#endif  // PROTOSPEC_SDK_PROFILE_H
