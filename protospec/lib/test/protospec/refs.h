// ProtoSpec SDK: typed references.
//
// References are stored as name strings with a typed target; the tree holds no
// pointers. This module gives the ergonomic operations that make that safe:
// resolve a ref to the element it names, find every referrer of an element,
// rename an element and fix up all referrers, and delete a subtree while
// reporting (or cascading) the references it would leave dangling.
//
// A "referrer" is any authored typed reference field (scalar or list entry)
// whose target-type set includes the referenced element's type and whose stored
// name matches -- plus the schema's DYNAMIC refs: string fields annotated
// `(target_from=sibling)`, whose target type is the runtime keyword the sibling
// holds (a frame sensor's objtype/objname pair). All of them are declared in the
// schema, which is what lets this module stay generic.
//
// The reference SLOT is the profile seam here. A scan hands out a move-only
// proxy over the stored name rather than a reference to it, because a profile
// whose names are engine properties has no name string to hand out. Reading is
// `slot.Get()`, rewriting is `slot.Set(...)`, and both go through the profile.
#ifndef PROTOSPEC_SDK_REFS_H
#define PROTOSPEC_SDK_REFS_H

#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <vector>

#include "protospec/detail.h"
#include "protospec/plain_profile.h"
#include "protospec/profile.h"
#include "protospec/traversal.h"
#include "reflect.h"

namespace ps::sdk {

namespace mj = ps::mjcf;

// A single authored reference that names a particular element. Its text fields
// are the report's, not the document's: UTF-8 at the diagnostic boundary, like
// ps::Diagnostic.
struct Referrer {
  detail::Handle element;  // the element carrying the ref
  std::string field;       // the ref field's IR name (e.g. "joint", "material")
  std::string refname;     // the stored name string
  std::string path;        // root-to-element path, for diagnostics
};

namespace detail {

// Invoke `on(field_id, field_name, slot, target_types)` for every authored
// reference of `e`: each scalar typed reference, each entry of a reference list,
// and each dynamic (target_from) reference whose sibling keyword resolves.
//
// `slot` is a live, move-only proxy over the stored name -- rewrite it to edit
// the reference in place (this is how Rename fixes referrers) or just read it.
// It is handed over BY VALUE and must not outlive the callback.
template <class P, class E, class OnRef>
void ScanRefs(E& e, OnRef&& on) {
  P::Ref::ScanTyped(e, on);

  // Dynamic refs (target_from, docs/refs_design.md category D): the schema marks
  // string fields whose target type is the runtime value of a SIBLING field
  // (objname (target_from=objtype)). The reflect descriptor carries the marking,
  // so rename fixup / referrer scan / delete cleanup reach these fields with no
  // per-element code. An unset or unknown keyword yields no targets and the name
  // is left alone -- opaque, never guessed at.
  if constexpr (!IsRoot<P, E>) {
    const mj::ElementType type = ElementTypeOf<P, E>;
    const mj::reflect::ElementDescriptor& desc = mj::reflect::Describe(type);
    for (int i = 0; i < static_cast<int>(desc.field_count); ++i) {
      const mj::reflect::FieldDescriptor& fd = desc.fields[i];
      if (fd.target_from.empty()) continue;
      const int sib = FieldIdByName(type, fd.target_from);
      if (sib < 0) continue;
      auto kw = P::Ref::DynSlot(e, sib);
      auto nm = P::Ref::DynSlot(e, i);
      if (!kw.IsSet() || !nm.IsSet()) continue;
      std::vector<mj::ElementType> targets =
          DynRefTargetTypes(P::Str::ToUtf8(kw.Get()));
      if (targets.empty()) continue;
      on(i, fd.name.data(), std::move(nm), targets);
    }
  }
}

// A name index over the profile's string type, probeable with a view.
template <class P>
using NameSet = std::unordered_set<StrOf<P>, StrHash<P>, StrEq<P>>;

// True when `newname` is already held by an element other than `self` in the
// same shared name namespace as `type` (NameCategory folding: the joint /
// tendon / actuator / sensor / equality spelling unions each share one
// namespace). Renaming onto such a name would fuse two elements' referrer sets.
template <class P>
bool NameTakenByOther(DocOf<P>& model, const void* self, mj::ElementType type,
                      ViewOf<P> newname) {
  const int cat = NameCategory(type);
  bool taken = false;
  WalkModelAll<P>(model, [&](auto& e) {
    using E = std::decay_t<decltype(e)>;
    if constexpr (!IsRoot<P, E>) {
      if (taken || static_cast<const void*>(&e) == self) return;
      if (NameCategory(ElementTypeOf<P, E>) != cat) return;
      if (std::optional<ViewOf<P>> nm = NameOf<P>(e))
        if (P::Str::Equals(*nm, newname)) taken = true;
    }
  });
  return taken;
}

// A name an element may be renamed to: non-empty and outside the reserved
// auto-name prefix (ps::kReservedNamePrefix, owned by compile-time binding
// names).
template <class P>
bool AssignableName(ViewOf<P> name) {
  return !P::Str::Empty(name) &&
         !P::Str::StartsWith(name, ps::kReservedNamePrefix);
}

}  // namespace detail

// --- Reference assignment ------------------------------------------------ //
// Every typed cross-reference in the document is a name with a typed target,
// never a tree pointer. These set or clear one without the consumer having to
// know the storage shape. An empty name clears the field (unauthored);
// otherwise it names `name`. No lookup is done here (that is Resolve's job);
// assigning a name that resolves to nothing is a validation concern.

// The generic form: the reference field at reflect id `field_id` of `elem`.
template <class P = plain::Plain, class E>
bool SetRefByField(E& elem, int field_id, ViewOf<P> name) {
  auto slot = P::Ref::SlotAt(elem, field_id);
  if (!slot) return false;
  slot.Set(name);
  return true;
}

// Point the reference at `field_id` at a concrete element by its authored name.
// Returns false (leaving the field untouched) when the target has no usable name
// -- an unnamed element cannot be referred to in MJCF -- or when the target's
// element type is not a valid target of that reference.
template <class P = plain::Plain, class E, class Target>
bool SetRefToElement(E& elem, int field_id, const Target& target) {
  const mj::reflect::ElementDescriptor& d =
      mj::reflect::Describe(ElementTypeOf<P, E>);
  if (field_id < 0 || field_id >= static_cast<int>(d.field_count)) return false;
  if (!detail::RefAcceptsType(d.fields[field_id], ElementTypeOf<P, Target>))
    return false;
  std::optional<ViewOf<P>> nm = detail::NameOf<P>(target);
  if (!nm || P::Str::Empty(*nm)) return false;
  return SetRefByField<P>(elem, field_id, *nm);
}

template <class P = plain::Plain, class E>
void ClearRefByField(E& elem, int field_id) {
  auto slot = P::Ref::SlotAt(elem, field_id);
  if (slot) slot.Clear();
}

// --- Plain-storage conveniences ------------------------------------------- //
// The plain profile's reference fields are addressable values, so a caller with
// one in hand can name it directly: `SetRef(g.material, mat)`. These are plain
// only, deliberately: a profile whose fields are not addressable reaches the
// same reference through SetRefByField.

template <class T>
void SetRef(ps::opt<ps::Ref<T>>& field, std::string_view name) {
  if (name.empty())
    field.reset();
  else
    field = ps::Ref<T>(std::string(name));
}

// `target`'s element type must be a valid target of `Ref<T>` (itself for a
// concrete ref, a union member for a union ref) -- otherwise this is a compile
// error, not a silent type-mismatched assignment.
template <class T, class E>
bool SetRef(ps::opt<ps::Ref<T>>& field, const E& target) {
  static_assert(plain::ref_accepts_target<T, std::decay_t<E>>,
                "SetRef target type is not a valid target for this Ref<T>");
  std::optional<std::string_view> nm = plain::Plain::Name(target);
  if (!nm || nm->empty()) return false;
  field = ps::Ref<T>(std::string(*nm));
  return true;
}

template <class T>
void ClearRef(ps::opt<ps::Ref<T>>& field) {
  field.reset();
}

// --- Resolve -------------------------------------------------------------- //

// The element a reference names, as a type-erased handle, or a null handle when
// nothing matches. `targets` is the target-type set the scan hands out beside
// the slot.
template <class P = plain::Plain>
detail::Handle ResolveName(DocOf<P>& model, ViewOf<P> name,
                           const std::vector<mj::ElementType>& targets) {
  if (P::Str::Empty(name)) return {};
  detail::Handle found;
  detail::WalkModelAll<P>(model, [&](auto& e) {
    if (found) return;
    using E = std::decay_t<decltype(e)>;
    if (detail::Contains(targets, ElementTypeOf<P, E>)) {
      if (std::optional<ViewOf<P>> nm = detail::NameOf<P>(e))
        if (P::Str::Equals(*nm, name)) found = detail::MakeHandle<P>(e);
    }
  });
  return found;
}

// The element the reference at `field_id` of `elem` names, or a null handle.
template <class P = plain::Plain, class E>
detail::Handle ResolveField(DocOf<P>& model, const E& elem, int field_id) {
  const std::vector<mj::ElementType> targets =
      detail::RefTargetsAt(ElementTypeOf<P, E>, field_id);
  if (targets.empty()) return {};
  return ResolveName<P>(model, P::Ref::NameAt(elem, field_id), targets);
}

// Plain-storage convenience: resolve a `Ref<Target>` value. `Ref<TendonAny>`
// resolves against both tendon spellings.
template <class Target>
detail::Handle Resolve(mj::Model& model, const ps::Ref<Target>& ref) {
  if (ref.empty()) return {};
  return ResolveName<plain::Plain>(model, std::string_view(ref.name),
                                   plain::RefTargetTypes<Target>());
}

// Typed convenience: resolve directly to a T*. Only a concrete element target
// can be resolved this way -- a union ref (Ref<JointAny>, Ref<TendonAny>, ...)
// names one of several spellings, so there is no single T to hand back; the
// static_assert makes that a compile error rather than a bad cast. Use Resolve
// and switch on the handle's element type for those.
template <class T>
T* ResolveTo(mj::Model& model, const ps::Ref<T>& ref) {
  static_assert(std::is_void_v<typename plain::union_node<T>::type>,
                "ResolveTo needs a concrete element target; a union ref "
                "resolves to one of several types -- use Resolve instead");
  detail::Handle h = Resolve(model, ref);
  return h ? static_cast<T*>(const_cast<void*>(h.ptr)) : nullptr;
}

// --- ScanRefs (reflection-driven reference visit) ------------------------- //

// Invoke `on` once for every authored reference OF a single element: scalar
// typed reference fields, each entry of a reference list, and the schema's
// dynamic (target_from) references. `on` is called as
//
//   on(int field_id, const char* field_name, RefSlotAny<P> slot,
//      const std::vector<mj::ElementType>& target_types)
//
// where `slot` is a LIVE, move-only proxy over the stored name -- `slot.Set(...)`
// edits the reference in place (this is how Rename fixes referrers),
// `slot.Get()` reads it -- and `target_types` is the set of element types the
// reference may name (a union ref names several). Reflection-driven: it needs no
// per-element code and tracks the schema automatically. Only `element` itself is
// visited (not its subtree); walk with WalkModel/WalkSubtree to reach every
// element. Never invoked for the document root (it holds no references).
//
// The slot must not be copied out of the callback: it is non-copyable precisely
// so an accidental by-value capture is a build break rather than a rename that
// silently updates nothing.
template <class P = plain::Plain, class E, class OnRef>
void ScanRefs(E& element, OnRef&& on) {
  detail::ScanRefs<P>(element, std::forward<OnRef>(on));
}

// --- FindReferrers -------------------------------------------------------- //

// Every authored reference in the document that names an element of type
// `targetType` with name `targetName`.
template <class P = plain::Plain>
std::vector<Referrer> FindReferrers(DocOf<P>& model, ViewOf<P> targetName,
                                    mj::ElementType targetType) {
  std::vector<Referrer> out;
  ParentMap<P> pm(model);
  detail::WalkModelAll<P>(model, [&](auto& e) {
    detail::Handle h = detail::MakeHandle<P>(e);
    detail::ScanRefs<P>(e, [&](int, const char* fname, auto&& slot,
                               const std::vector<mj::ElementType>& tgts) {
      if (P::Str::Equals(slot.Get(), targetName) &&
          detail::Contains(tgts, targetType)) {
        out.push_back(Referrer{h, fname, P::Str::ToUtf8(slot.Get()),
                               pm.PathToPtr(h.ptr)});
      }
    });
  });
  return out;
}

// Every referrer of a concrete element.
template <class P = plain::Plain, class E>
std::vector<Referrer> FindReferrers(DocOf<P>& model, const E& elem) {
  std::optional<ViewOf<P>> nm = detail::NameOf<P>(elem);
  if (!nm) return {};
  return FindReferrers<P>(model, *nm, ElementTypeOf<P, E>);
}

// --- UniqueName ----------------------------------------------------------- //

// A name unique within `type`'s MuJoCo name namespace: returns `base` when free,
// else `base_1`, `base_2`, ... MuJoCo namespaces names by category, not by exact
// element type -- the two joint spellings share one namespace, and every
// actuator / sensor / tendon / equality spelling shares its union's namespace
// (detail::NameCategory) -- so a name unique here cannot collide at compile time.
template <class P = plain::Plain>
StrOf<P> UniqueName(DocOf<P>& model, mj::ElementType type, ViewOf<P> base) {
  const int cat = detail::NameCategory(type);
  detail::NameSet<P> used;
  detail::WalkModelAll<P>(model, [&](auto& e) {
    using E = std::decay_t<decltype(e)>;
    if constexpr (!IsRoot<P, E>) {
      if (detail::NameCategory(ElementTypeOf<P, E>) == cat)
        if (std::optional<ViewOf<P>> nm = detail::NameOf<P>(e))
          used.insert(P::Str::FromUtf8(P::Str::ToUtf8(*nm)));
    }
  });
  const std::string b = P::Str::ToUtf8(base);
  if (!used.count(P::Str::FromUtf8(b))) return P::Str::FromUtf8(b);
  for (int k = 1;; ++k) {
    StrOf<P> c = P::Str::FromUtf8(b + "_" + std::to_string(k));
    if (!used.count(c)) return c;
  }
}

// --- Rename --------------------------------------------------------------- //
//
// RESULT-OBJECT CONVENTION. The structural verbs (Rename, Duplicate, Reparent,
// DeleteSubtree, Attach) all report through a small result object carrying `bool
// ok` (contextually convertible: `if (auto r = sdk::Rename(...))`) and, where a
// failure has a cause, a `std::string reason`. RenameResult additionally carries
// `updated` (referrer fields rewritten). Rename's former scalar `int` return
// survives one release as a deprecated conversion on RenameResult so existing
// call sites keep compiling; Duplicate's former `void*` return deliberately did
// NOT become a conversion operator (an implicit pointer conversion would win
// over the explicit `operator bool` in `if (r)`) -- its clone is the plain
// `.clone` field / `As<T>()`. Prefer `.ok` / `.updated` / `.clone`.
struct RenameResult {
  bool ok = false;         // the rename applied (or was an accepted no-op)
  int updated = 0;         // referrer fields rewritten (0 for a no-op / nameless
                           // element gaining its first name)
  std::string reason;      // why it was rejected (empty on success)
  explicit operator bool() const { return ok; }
  // DEPRECATED one-release compat with the former `int` return: the referrer
  // count on success, or -1 on rejection. Prefer `.ok` / `.updated`.
  operator int() const { return ok ? updated : -1; }  // NOLINT
};

// Rename an element and rewrite every referrer to match. On success `ok` is
// true and `updated` is the number of referrer fields rewritten -- 0 when the
// new name equals the old (an accepted no-op) or when the element was nameless
// (it gains the name; nothing can have referred to it). On rejection `ok` is
// false, `reason` says why, and the document is left untouched: `newname` is
// invalid (empty, inside the reserved auto-name prefix ps::kReservedNamePrefix)
// or already held by a DIFFERENT element in the same shared name namespace
// (renaming onto it would silently fuse the two elements' referrer sets). The
// pointer is excluded from this template so the runtime
// `Rename(model, const void*, newname)` below wins for a type-erased element
// pointer.
template <class P = plain::Plain, class E,
          std::enable_if_t<!std::is_pointer_v<E>, int> = 0>
RenameResult Rename(DocOf<P>& model, E& elem, ViewOf<P> newname) {
  std::optional<ViewOf<P>> cur = detail::NameOf<P>(elem);
  const StrOf<P> oldname =
      cur ? P::Str::FromUtf8(P::Str::ToUtf8(*cur)) : StrOf<P>();
  if (cur && P::Str::Equals(*cur, newname)) return {true, 0, ""};
  if (!cur && P::Str::Empty(newname)) return {true, 0, ""};
  const mj::ElementType targetType = ElementTypeOf<P, E>;
  if (!detail::AssignableName<P>(newname))
    return {false, 0, "name is empty or inside the reserved auto-name prefix"};
  if (detail::NameTakenByOther<P>(model, &elem, targetType, newname))
    return {false, 0, "name already held by another element of this category"};

  int updated = 0;
  detail::WalkModelAll<P>(model, [&](auto& other) {
    detail::ScanRefs<P>(other, [&](int, const char*, auto&& slot,
                                   const std::vector<mj::ElementType>& tgts) {
      if (P::Str::Equals(slot.Get(), P::Str::View(oldname)) &&
          detail::Contains(tgts, targetType)) {
        slot.Set(newname);
        ++updated;
      }
    });
  });
  detail::SetName<P>(elem, newname);
  return {true, updated, ""};
}

// --- DeleteRecursive ------------------------------------------------------ //

struct DeleteReport {
  bool removed = false;               // was the element found and unlinked
  std::vector<Referrer> dangling;     // references left pointing at nothing
  bool cascaded = false;              // dangling refs were cleared
  // Uniform result-object contract: truthy when the delete found and removed the
  // target (`if (sdk::DeleteSubtree(...)) ...`); `dangling` details the fallout.
  explicit operator bool() const { return removed; }
};

namespace detail {

template <class P>
struct NameType {
  StrOf<P> name;
  mj::ElementType type;
};

template <class P>
bool IsDeleted(const std::vector<NameType<P>>& deleted, ViewOf<P> name,
               const std::vector<mj::ElementType>& tgts) {
  for (const auto& d : deleted)
    if (P::Str::Equals(P::Str::View(d.name), name) && Contains(tgts, d.type))
      return true;
  return false;
}

// Reset (make unauthored) every reference of an element that names a deleted
// element, so no empty-but-present ref lingers. Reference LISTS drop only the
// deleted entries, which is why the typed half is a profile operation rather
// than a loop over slots.
template <class P, class E>
void ClearRefsOn(E& e, const std::vector<NameType<P>>& deleted) {
  P::Ref::ClearTypedIf(
      e, [&](ViewOf<P> name, const std::vector<mj::ElementType>& tgts) {
        return IsDeleted<P>(deleted, name, tgts);
      });
  if constexpr (!IsRoot<P, E>) {
    const mj::ElementType type = ElementTypeOf<P, E>;
    const mj::reflect::ElementDescriptor& desc = mj::reflect::Describe(type);
    for (int i = 0; i < static_cast<int>(desc.field_count); ++i) {
      const mj::reflect::FieldDescriptor& fd = desc.fields[i];
      if (fd.target_from.empty()) continue;
      const int sib = FieldIdByName(type, fd.target_from);
      if (sib < 0) continue;
      auto kw = P::Ref::DynSlot(e, sib);
      auto nm = P::Ref::DynSlot(e, i);
      if (!kw.IsSet() || !nm.IsSet()) continue;
      if (IsDeleted<P>(deleted, nm.Get(),
                       DynRefTargetTypes(P::Str::ToUtf8(kw.Get()))))
        nm.Clear();
    }
  }
}

// Collect the (name, type) of every named element of a subtree.
template <class P, class E>
std::vector<NameType<P>> CollectNames(E& root) {
  std::vector<NameType<P>> out;
  WalkTree<P>(root, [&](auto& e) {
    using X = std::decay_t<decltype(e)>;
    if (std::optional<ViewOf<P>> nm = NameOf<P>(e))
      out.push_back({P::Str::FromUtf8(P::Str::ToUtf8(*nm)),
                     ElementTypeOf<P, X>});
  });
  return out;
}

// Record every reference in the document that names a deleted element, and
// optionally clear them.
template <class P>
void ReportDangling(DocOf<P>& model, const std::vector<NameType<P>>& deleted,
                    DeleteReport& report, bool cascade) {
  ParentMap<P> pm(model);
  WalkModelAll<P>(model, [&](auto& other) {
    Handle h = MakeHandle<P>(other);
    ScanRefs<P>(other, [&](int, const char* fname, auto&& slot,
                           const std::vector<mj::ElementType>& tgts) {
      if (IsDeleted<P>(deleted, slot.Get(), tgts)) {
        report.dangling.push_back(
            Referrer{h, fname, P::Str::ToUtf8(slot.Get()), pm.PathToPtr(h.ptr)});
      }
    });
  });
  if (cascade && !report.dangling.empty()) {
    WalkModelAll<P>(model,
                    [&](auto& other) { ClearRefsOn<P>(other, deleted); });
    report.cascaded = true;
  }
}

}  // namespace detail

// Remove `elem` and its whole subtree from the document. Any reference elsewhere
// that named a removed element is reported as dangling (with its path). When
// `cascade` is true those references are cleared (set unauthored) so the
// document is left with no silent danglers; otherwise it is returned as-is with
// the danglers reported for the caller to resolve.
//
// After this returns, `elem` must not be dereferenced: the contract is
// detachment plus release at an unspecified later point.
template <class P = plain::Plain, class E,
          std::enable_if_t<!std::is_pointer_v<E>, int> = 0>
DeleteReport DeleteRecursive(DocOf<P>& model, E& elem, bool cascade = false) {
  DeleteReport report;
  std::vector<detail::NameType<P>> deleted = detail::CollectNames<P>(elem);

  const void* ptr = &elem;
  report.removed = P::Tree::Remove(model, ptr);
  if (!report.removed) return report;

  detail::ReportDangling<P>(model, deleted, report, cascade);
  return report;
}

// --- Runtime-typed Rename / DeleteSubtree --------------------------------- //
//
// `Rename<E>` / `DeleteRecursive<E>` above are keyed on an element's STATIC
// type. Driving them from a runtime element pointer would instantiate each
// across all ~140 families, and each re-instantiates a whole-document walk -- a
// ~140x140 fan-out that makes a translation unit take minutes to compile. These
// variants are keyed on the element POINTER instead: one generic walk locates
// it, recovers its runtime ElementType, and drives the same referrer
// bookkeeping through the runtime helpers. Same behaviour, a handful of walk
// instantiations rather than hundreds. This is the supported API for a consumer
// that resolves elements dynamically (by pick, serial, or path); reach for the
// `<E>` templates only when the static type is already in hand.

// Rename the element at `elem` (any element owned by `model`) to `newname` and
// rewrite every typed referrer. Same RenameResult contract as the `<E>` form.
template <class P = plain::Plain>
RenameResult Rename(DocOf<P>& model, const void* elem, ViewOf<P> newname) {
  bool found = false;
  bool had_name = false;
  StrOf<P> oldname;
  mj::ElementType type = ElementTypeOf<P, DocOf<P>>;
  std::function<void()> set_name;
  detail::WalkModelAll<P>(model, [&](auto& e) {
    using E = std::decay_t<decltype(e)>;
    if (found) return;
    if (static_cast<const void*>(&e) == elem) {
      found = true;
      type = ElementTypeOf<P, E>;
      if (std::optional<ViewOf<P>> nm = detail::NameOf<P>(e)) {
        had_name = true;
        oldname = P::Str::FromUtf8(P::Str::ToUtf8(*nm));
      }
      auto* ep = &e;
      StrOf<P> nn = P::Str::FromUtf8(P::Str::ToUtf8(newname));
      set_name = [ep, nn] { detail::SetName<P>(*ep, P::Str::View(nn)); };
    }
  });
  if (!found) return {false, 0, "element is not in the model"};
  if (had_name && P::Str::Equals(P::Str::View(oldname), newname))
    return {true, 0, ""};
  if (!had_name && P::Str::Empty(newname)) return {true, 0, ""};
  if (!detail::AssignableName<P>(newname))
    return {false, 0, "name is empty or inside the reserved auto-name prefix"};
  if (detail::NameTakenByOther<P>(model, elem, type, newname))
    return {false, 0, "name already held by another element of this category"};

  int updated = 0;
  detail::WalkModelAll<P>(model, [&](auto& other) {
    detail::ScanRefs<P>(other, [&](int, const char*, auto&& slot,
                                   const std::vector<mj::ElementType>& tgts) {
      if (P::Str::Equals(slot.Get(), P::Str::View(oldname)) &&
          detail::Contains(tgts, type)) {
        slot.Set(newname);
        ++updated;
      }
    });
  });
  set_name();
  return {true, updated, ""};
}

// Remove the subtree rooted at `elem` from `model`. Any reference elsewhere that
// named a removed element is reported as dangling (with its path). When
// `cascade`, those references are cleared (set unauthored) so the document has
// no silent danglers; otherwise it is left as-is for the caller to resolve.
// `report.removed` is false when `elem` is not found.
template <class P = plain::Plain>
DeleteReport DeleteSubtree(DocOf<P>& model, const void* elem,
                           bool cascade = false) {
  DeleteReport report;

  ParentMap<P> pm(model);
  if (!pm.Lookup(elem)) return report;  // not in the model: removed = false
  auto in_subtree = [&](const void* p) -> bool {
    for (const void* q = p; q;) {
      if (q == elem) return true;
      const typename ParentMap<P>::Node* n = pm.Lookup(q);
      if (!n) break;
      q = n->parent;
    }
    return false;
  };

  std::vector<detail::NameType<P>> deleted;
  detail::WalkModelAll<P>(model, [&](auto& e) {
    using E = std::decay_t<decltype(e)>;
    if (in_subtree(&e)) {
      if (std::optional<ViewOf<P>> nm = detail::NameOf<P>(e))
        deleted.push_back(
            {P::Str::FromUtf8(P::Str::ToUtf8(*nm)), ElementTypeOf<P, E>});
    }
  });

  report.removed = P::Tree::Remove(model, elem);
  if (!report.removed) return report;

  detail::ReportDangling<P>(model, deleted, report, cascade);
  return report;
}

// --- PruneSubtrees -------------------------------------------------------- //

// Remove every element for which `pred` returns true, subtrees included: pruning
// a body prunes everything beneath it, and a kept parent is still descended so
// its own selected children go too. `pred` is any callable `bool(const auto&
// element)` invoked on each element in document order; the document root is
// never offered to it (there is nothing to prune it from).
//
// A raw structural prune: unlike DeleteRecursive / DeleteSubtree it does NOT
// scan or clear references to removed elements, so a ref elsewhere may be left
// dangling. Use it for whole-partition drops (compile-input filtering, layer
// pruning) where dangling names are tolerated or reconciled separately; reach
// for the Delete verbs when referrer bookkeeping must stay consistent.
template <class P = plain::Plain, class Pred>
void PruneSubtrees(DocOf<P>& model, Pred&& pred) {
  P::Tree::PruneIf(model, std::forward<Pred>(pred));
}

}  // namespace ps::sdk

#endif  // PROTOSPEC_SDK_REFS_H
